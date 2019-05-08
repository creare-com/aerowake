/*
	imageProcessing.h
	
	Declarations for PCG image processing.
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_PROC_H__
#define __PCG_PROC_H__

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <chrono> // C++11
#include <list>

#include <opencv2/opencv.hpp>

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

#include "json.hpp"
#include "logger.h"
#include "imageAcquisition.h"
#include "stereoCal.h"
#include "benchmarker.h"

// All stereo processing algorithms supported must be listed here
#include "ptCloudGenAlgs/dummyAlg.h"
#include "ptCloudGenAlgs/cpuFastWithBinnedKps.h"
// #include "ptCloudGenAlgs/gpuFastWithBinnedKps.h"

class ImageProcessing {
private:
	list<const Benchmarker *> * bms;
	Logger * logger;
	StereoCal cal_data;
	bool enableGpu;
	Benchmarker bmImgTotal;
	
	StereoPtCloudGenAlg * alg = NULL;
	string rosFrameId;
	unsigned int seqNum = 0;

	ros::Publisher rosPublisher;

public:
	ImageProcessing(list<const Benchmarker *> * _bms) : 
		bms(_bms),
		bmImgTotal("Total image processing")
	{
		bms->push_back(&bmImgTotal);
	}
	
	~ImageProcessing() {
		if(alg != NULL) {
			delete alg;
		}
	}
	void init(nlohmann::json options, Logger * lgr, ros::NodeHandle * rosNodeHandle);
	void visit(ImageDataSet imgData); // Handle everything
	bool areCvWindowsOpen();
	bool isAnyoneListening();
};

#endif // __PCG_PROC_H__
