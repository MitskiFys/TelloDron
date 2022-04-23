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

#include "jetson-utils/cudaFont.h"
#include "jetson-utils/cudaDraw.h"
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

struct PoseState
{
	bool isLeftArmUp = false;
	bool isRightArmUp = false;
	bool isLeftElbowUp = false;
	bool isRightElbowUp = false;
}poseState;

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

int main()
{
	DronControl dron;
	dron.fireTrackingStateChanged.connect([](const bool state)
	{
		isTrackingActive = state;
	});


	videoOptions inp;
	inp.resource = "/dev/video1";
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

		// render outputs
		if( output != NULL )
		{
			if( font != NULL)
			{
				for (const auto& pose : poses)
				{
					std::string str = std::to_string((unsigned int)pose.ID).append(" - person");
					font->OverlayText(image, input->GetWidth(), input->GetHeight(),
									  str.c_str(), pose.Left, pose.Top, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));

					drawTrackingStrategy(net, pose, image, input);
					//std::cout << getAngleBtwThreePoints(net, pose, LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST) << std::endl;
					setTrackCorrection(net, input, pose, dron);
					poseState.isLeftArmUp = isArmUp(getAngleBtwThreePoints(net, pose, NECK, LEFT_SHOULDER, LEFT_ELBOW),getAngleBtwThreePoints(net, pose, LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST));
					poseState.isRightArmUp = isArmUp(getAngleBtwThreePoints(net, pose, NECK, RIGHT_SHOULDER, RIGHT_ELBOW) * -1,getAngleBtwThreePoints(net, pose, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST));
					poseState.isLeftElbowUp = isElbowUp(getAngleBtwThreePoints(net, pose, NECK, LEFT_SHOULDER, LEFT_ELBOW),getAngleBtwThreePoints(net, pose, LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST));
					poseState.isRightElbowUp = isElbowUp(getAngleBtwThreePoints(net, pose, NECK, RIGHT_SHOULDER, RIGHT_ELBOW),getAngleBtwThreePoints(net, pose, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST) * -1);

					auto delta = 37;
					if (poseState.isLeftArmUp)
					{
						std::string arm = "Left arm up";

						font->OverlayText(image, input->GetWidth(), input->GetHeight(),
										  arm.c_str(), 5, 80 + delta, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
						delta += 37;
					}
					if (poseState.isRightArmUp)
					{
						std::string arm = "Right arm up";

						font->OverlayText(image, input->GetWidth(), input->GetHeight(),
										  arm.c_str(), 5, 80 + delta, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
						delta += 37;
					}
					if (poseState.isLeftElbowUp)
					{
						std::string arm = "Left elbow up";

						font->OverlayText(image, input->GetWidth(), input->GetHeight(),
										  arm.c_str(), 5, 80 + delta, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
						delta += 37;
					}
					if (poseState.isRightElbowUp)
					{
						std::string arm = "Right elbow up";

						font->OverlayText(image, input->GetWidth(), input->GetHeight(),
										  arm.c_str(), 5, 80 + delta, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));
						delta += 37;
					}


					//std::cout << poseState.isLeftArmUp << poseState.isRightArmUp << poseState.isLeftElbowUp << poseState.isRightElbowUp << std::endl;

					//const auto distanceToCentre = sqrt(pow((neckPoint.x - input->GetWidth() / 2), 2) + pow((neckPoint.y - input->GetHeight() / 2), 2));
					//getAngleBtwThreePoints(net, pose, LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST);
					//getAngleBtwThreePoints(net, pose, NECK, LEFT_SHOULDER, LEFT_ELBOW);
//					for (const auto& point : pose.Keypoints)
//					{
//						std::cout << net->GetKeypointName(point.ID) << std::endl;
//					}
				}

			}

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
	return 0;
}
