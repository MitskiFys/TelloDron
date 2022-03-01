#include <iostream>
#include "GamepadControl.h"
#include <chrono>
#include "DronControl.h"

#include <signal.h>
#include "videoSource.h"
#include "videoOutput.h"
#include "detectNet.h"

bool signal_recieved = false;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		LogVerbose("received SIGINT\n");
		signal_recieved = true;
	}
}

int main()
{
	DronControl dron;


	videoOptions opt;
	opt.resource = "/dev/video0";
	opt.codec = videoOptions::Codec::CODEC_UNKNOWN;
	opt.width = 960;
	opt.height = 720;
	opt.deviceType = videoOptions::DeviceType::DEVICE_V4L2;
	videoSource* frameSource = videoSource::Create(opt);

	videoOptions out;
	out.resource = "display";
	out.deviceType = videoOptions::DeviceType::DEVICE_DISPLAY;
	out.ioType = videoOptions::IoType::OUTPUT;
	videoOutput* output = videoOutput::Create(out);

	/*
	* create detection network
	*/
	detectNet* net = detectNet::Create();

	if( !net )
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 0;
	}

	// parse overlay flags
	const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr("");

	while( !signal_recieved )
	{
		// capture next image image
		uchar3* image = NULL;

		if( !frameSource->Capture(&image, 1000) )
		{
			// check for EOS
			if( !frameSource->IsStreaming() )
				break;

			LogError("detectnet:  failed to capture video frame\n");
			continue;
		}


		// detect objects in the frame
		detectNet::Detection* detections = NULL;

		const int numDetections = net->Detect(image, frameSource->GetWidth(), frameSource->GetHeight(), &detections, overlayFlags);

		if( numDetections > 0 )
		{
			LogVerbose("%i objects detected\n", numDetections);

			for( int n=0; n < numDetections; n++ )
			{
				LogVerbose("detected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
				LogVerbose("bounding box %i  (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
			}
		}

		// render outputs
		if( output != NULL )
		{
			output->Render(image, frameSource->GetWidth(), frameSource->GetHeight());

			// update the status bar
			char str[256];
			sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
			output->SetStatus(str);

			// check if the user quit
			if( !output->IsStreaming() )
				signal_recieved = true;
		}

		// print out timing info
		net->PrintProfilerTimes();

	}
	return 0;
}
