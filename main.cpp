#include <iostream>
#include "GamepadControl.h"
#include <chrono>
#include <cmath>
#include "DronControl.h"

#include <signal.h>
#include "videoSource.h"
#include "videoOutput.h"
#include "detectNet.h"
#include "poseNet.h"
#include <set>

#include "jetson-utils/cudaFont.h"
#include "jetson-utils/cudaDraw.h"
#include "include/sort-cpp/KalmanTracker.h"
#include "include/sort-cpp/Hungarian.h"
#include <stdio.h>
#include <inttypes.h>

namespace {
	const auto LEFT_SHOULDER = "left_shoulder";
	const auto RIGHT_SHOULDER = "right_shoulder";
	const auto LEFT_ELBOW = "left_elbow";
	const auto RIGHT_ELBOW = "right_elbow";
	const auto LEFT_WRIST = "left_wrist";
	const auto RIGHT_WRIST = "right_wrist";
	const auto NECK = "neck";
	const auto ROTATECOEF = 0.2;
}

bool signal_recieved = false;
bool isTrackingActive = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
	}
}

double angleBtwThreePoints(const poseNet::ObjectPose::Keypoint& a, const poseNet::ObjectPose::Keypoint& b, const poseNet::ObjectPose::Keypoint& c)
{
	const auto x1 = a.x - b.x, x2 = c.x - b.x;
	const auto y1 = a.y - b.y, y2 = c.y - b.y;
	const auto dot = x1 * x2 + y1 * y2;
	const auto pcross = x1 * y2 - y1 * x2;
	return atan2(pcross, dot) * 180 / acos(-1.0);
}

poseNet::ObjectPose::Keypoint getPointCoords(poseNet* net, const poseNet::ObjectPose& pose, const std::string& point)
{
	const auto pointNet = net->FindKeypointID(point.c_str());
	if (pointNet == -1)
	{
		return {0, -1, -1};
	}

	const auto pointId = pose.FindKeypoint(pointNet);

	if (pointId == -1)
	{
		return {0, -1, -1};
	}
	return pose.Keypoints[pointId];
}

void drawTrackingStrategy(poseNet* net, const poseNet::ObjectPose& pose, uchar3* image, videoSource* input)
{
	cudaDrawCircle(image, input->GetWidth(), input->GetHeight(), input->GetWidth()/2, input->GetHeight()/2, 5, make_float4(250, 0, 0, 255));
	const auto neckPoint = getPointCoords(net, pose, NECK);
	if (neckPoint.ID != 0 && neckPoint.x != -1)
	{
		cudaDrawLine(image, input->GetWidth(), input->GetHeight(), neckPoint.x, neckPoint.y, input->GetWidth() / 2
					 , input->GetHeight() / 2, make_float4(0, 255, 0, 125));
	}
}

double getAngleBtwThreePoints(poseNet* net, const poseNet::ObjectPose& pose, const std::string& firstPoint, const std::string secondPoint, const std::string thirdPoint)
{
	const auto firstPointNet = net->FindKeypointID(firstPoint.c_str());
	const auto secondPointNet = net->FindKeypointID(secondPoint.c_str());
	const auto thirdPointNet = net->FindKeypointID(thirdPoint.c_str());
	if (firstPointNet == -1 || secondPointNet == -1 || thirdPointNet == -1)
	{
		return 0;
	}
	const auto firstPointId = pose.FindKeypoint(firstPointNet);
	const auto secondPointId = pose.FindKeypoint(secondPointNet);
	const auto thirdPointId = pose.FindKeypoint(thirdPointNet);
	if (firstPointId != -1 && secondPointId != -1 && thirdPointId != -1)
	{
		return angleBtwThreePoints(pose.Keypoints[firstPointId], pose.Keypoints[secondPointId], pose.Keypoints[thirdPointId]);
	}
	return 0;
}

void setTrackCorrection(poseNet* net, videoSource* input, const poseNet::ObjectPose& pose, DronControl& dron)
{
	const auto neckPoint = getPointCoords(net, pose, NECK);
	const auto centreX = input->GetWidth() / 2;
	const auto centreY = input->GetHeight() / 2;
	const int xToCenterDistance = (neckPoint.x - centreX) / centreX * 100;
	const int yToCenterDistance = (neckPoint.y - centreY) / centreY * 100;
	if (pose.ID == 0)
	{
		dron.setActivePoseTracking(neckPoint.x != -1);
		dron.setXCorrection(xToCenterDistance);
		dron.setYCorrection(yToCenterDistance);
	}
}

void drawAdditionalInfo(videoSource* input, uchar3* image, cudaFont* font, DronControl& dron)
{
	std::string tracking = "Tracking system is ";
	if (isTrackingActive)
	{
		tracking.append("ON");
	}
	else
	{
		tracking.append("OFF");
	}
	dron.updateDroneState();

	font->OverlayText(image, input->GetWidth(), input->GetHeight(),
					  tracking.c_str(), 5, 5, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));

	std::string batteryLvl = "Battery level - " + dron.getBatteryLevel();

	font->OverlayText(image, input->GetWidth(), input->GetHeight(),
					  batteryLvl.c_str(), 5, 42, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));

	std::string temp = "Drone temperature - " + dron.getTemp();

	font->OverlayText(image, input->GetWidth(), input->GetHeight(),
					  temp.c_str(), 5, 80, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
}

bool isArmUp(const double shoulderAngle, const double elbowAngle)
{
	if (shoulderAngle > 90 && shoulderAngle < 140)
	{
		if (abs(elbowAngle) > 100 )
		{
			return true;
		}
	}
	return false;
	//140 .. 90
	//100 .. 180
}

bool isArmToSide(const double shoulderAngle, const double elbowAngle)
{
	if (abs(shoulderAngle) >= 170 && abs(shoulderAngle) < 181)
	{
		if (abs(elbowAngle) >= 170 && abs(elbowAngle) < 181)
		{
			return true;
		}
	}
	return false;
}

bool isElbowUp(const double shoulderAngle, const double elbowAngle)
{
	if (abs(shoulderAngle) >= 160 && abs(shoulderAngle) < 181)
	{
		if (elbowAngle >= 70 && elbowAngle < 110)
		{
			return true;
		}
	}
	return false;
}

typedef struct TrackingBox
{
	int id;
	Rect_<float> box;
}TrackingBox;

// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < std::numeric_limits<double>::epsilon())
		return 0;

	return (double)(in / un);
}

int main()
{
	DronControl dron;
	dron.fireTrackingStateChanged.connect([](const bool state)
	{
		isTrackingActive = state;
	});


	videoOptions inp;
	inp.resource = "/dev/video0";
	inp.codec = videoOptions::Codec::CODEC_UNKNOWN;
	inp.width = 960;
	inp.height = 720;
	inp.deviceType = videoOptions::DeviceType::DEVICE_V4L2;
	videoSource* input = videoSource::Create(inp);

	videoOptions out;
	out.resource = "display";
	out.deviceType = videoOptions::DeviceType::DEVICE_DISPLAY;
	out.ioType = videoOptions::IoType::OUTPUT;
	videoOutput* output = videoOutput::Create(out);

	/*
	* create detection network
	*/
	poseNet* net = poseNet::Create(poseNet::DENSENET121_BODY);

	if( !net )
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 0;
	}

	// parse overlay flags
	const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("");

	/*
	 * create font for image overlay
	 */
	cudaFont* font = cudaFont::Create();


	if( !font )
	{
		LogError("imagenet:  failed to load font for overlay\n");
		return 0;
	}


	vector<TrackingBox> detData;
	vector<KalmanTracker> trackers;
	vector<Rect_<float>> predictedBoxes;
	vector<vector<double>> iouMatrix;
	vector<int> assignment;
	set<int> unmatchedDetections;
	vector<cv::Point> matchedPairs;
	vector<TrackingBox> frameTrackingResult;
	set<int> unmatchedTrajectories;
	unsigned int trkNum = 0;
	unsigned int detNum = 0;
	set<int> allItems;
	set<int> matchedItems;
	int max_age = 1;
	int min_hits = 3;
	double iouThreshold = 0.3;

	while( !signal_recieved )
	{
		// capture next image image
		uchar3* image = NULL;

		if( !input->Capture(&image, 1000) )
		{
			// check for EOS
			if( !input->IsStreaming() )
				break;

			LogError("detectnet:  failed to capture video frame\n");
			continue;
		}

		drawAdditionalInfo(input, image, font, dron);

		// run pose estimation
		std::vector<poseNet::ObjectPose> poses;

		if( !net->Process(image, input->GetWidth(), input->GetHeight(), poses, overlayFlags) )
		{
			LogError("posenet: failed to process frame\n");
			continue;
		}

		bool isFirstFrame = true;

		// render outputs
		if( output != NULL )
		{
			for (const auto& pose : poses)
			{
				TrackingBox tb;
				tb.id = -1;
				tb.box = Rect_<float>(Point_<float>(pose.Left, pose.Right), Point_<float>(pose.Top, pose.Bottom));
				detData.push_back(tb);
			}

			if (trackers.size() == 0 && isFirstFrame) // the first frame met
			{
				isFirstFrame = false;
				// initialize kalman trackers using first detections.
				for (unsigned int i = 0; i < detData.size(); i++)
				{
					KalmanTracker trk = KalmanTracker(detData[i].box);
					trackers.push_back(trk);
				}
				// output the first frame detections
				for (unsigned int id = 0; id < detData.size(); id++)
				{
					TrackingBox tb = detData[id];
					std::cout << id + 1 << "," << tb.box.x << "," << tb.box.y << "," << tb.box.width + tb.box.x << "," << tb.box.height + tb.box.y << ",1,-1,-1,-1" << std::endl;
				}
			}
			// 3.1. get predicted locations from existing trackers.
			predictedBoxes.clear();

			for (auto it = trackers.begin(); it != trackers.end();)
			{
				Rect_<float> pBox = (*it).predict();
				if (pBox.x >= 0 && pBox.y >= 0)
				{
					predictedBoxes.push_back(pBox);
					it++;
				}
				else
				{
					it = trackers.erase(it);
					//cerr << "Box invalid at frame: " << frame_count << endl;
				}
			}

			// 3.2. associate detections to tracked object (both represented as bounding boxes)
			// dets : detFrameData[fi]
			trkNum = predictedBoxes.size();
			detNum = detData.size();

			iouMatrix.clear();
			iouMatrix.resize(trkNum, vector<double>(detNum, 0));
			for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
			{
				for (unsigned int j = 0; j < detNum; j++)
				{
					// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
					iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detData[j].box);
				}
			}

			// solve the assignment problem using hungarian algorithm.
			// the resulting assignment is [track(prediction) : detection], with len=preNum
			HungarianAlgorithm HungAlgo;
			assignment.clear();
			HungAlgo.Solve(iouMatrix, assignment);
			// find matches, unmatched_detections and unmatched_predictions
			unmatchedTrajectories.clear();
			unmatchedDetections.clear();
			allItems.clear();
			matchedItems.clear();

			if (detNum > trkNum) //	there are unmatched detections
			{
				for (unsigned int n = 0; n < detNum; n++)
					allItems.insert(n);

				for (unsigned int i = 0; i < trkNum; ++i)
					matchedItems.insert(assignment[i]);

				set_difference(allItems.begin(), allItems.end(),
					matchedItems.begin(), matchedItems.end(),
					insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
			}
			else if (detNum < trkNum) // there are unmatched trajectory/predictions
			{
				for (unsigned int i = 0; i < trkNum; ++i)
					if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
						unmatchedTrajectories.insert(i);
			}
			else
				;

			// filter out matched with low IOU
			matchedPairs.clear();
			for (unsigned int i = 0; i < trkNum; ++i)
			{
				if (assignment[i] == -1) // pass over invalid values
					continue;
				if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
				{
					unmatchedTrajectories.insert(i);
					unmatchedDetections.insert(assignment[i]);
				}
				else
					matchedPairs.push_back(cv::Point(i, assignment[i]));
			}
			// 3.3. updating trackers

			// update matched trackers with assigned detections.
			// each prediction is corresponding to a tracker
			int detIdx, trkIdx;
			for (unsigned int i = 0; i < matchedPairs.size(); i++)
			{
				trkIdx = matchedPairs[i].x;
				detIdx = matchedPairs[i].y;
				trackers[trkIdx].update(detData[detIdx].box);
			}

			// create and initialise new trackers for unmatched detections
			for (auto umd : unmatchedDetections)
			{
				KalmanTracker tracker = KalmanTracker(detData[umd].box);
				trackers.push_back(tracker);
			}
			// get trackers' output
			frameTrackingResult.clear();
			for (auto it = trackers.begin(); it != trackers.end();)
			{
				if (((*it).m_time_since_update < 1) &&
					((*it).m_hit_streak >= min_hits))
				{
					TrackingBox res;
					res.box = (*it).get_state();
					res.id = (*it).m_id + 1;
					frameTrackingResult.push_back(res);
					it++;
				}
				else
				it++;

				// remove dead tracklet
				if (it != trackers.end() && (*it).m_time_since_update > max_age)
					it = trackers.erase(it);
			}
			for (auto tb : frameTrackingResult)
				std::cout << "," << tb.id << "," << tb.box.x << "," << tb.box.y << "," << tb.box.width << "," << tb.box.height << ",1,-1,-1,-1" << std::endl;
			detData.clear();

			output->Render(image, input->GetWidth(), input->GetHeight());

			// update status bar
			char str[256];
			sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
			output->SetStatus(str);

			// check if the user quit
			if( !output->IsStreaming() )
				signal_recieved = true;
		}

		// print out timing info
		//net->PrintProfilerTimes();

	}
	//					std::string str = std::to_string((unsigned int)pose.ID).append(" - person");
	//					font->OverlayText(image, input->GetWidth(), input->GetHeight(),
	//									  str.c_str(), pose.Left, pose.Top, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
	//					drawTrackingStrategy(net, pose, image, input);
	//					setTrackCorrection(net, input, pose, dron);
	return 0;
}
