/*
	stereoPtCloudGenAlg.h
	
	Parent class for all stereo point cloud generation algorithms
	
	2017-2-28  JDW  Created.
*/

#ifndef __PCG_ALGS_H__
#define __PCG_ALGS_H__

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <list>
#include <chrono> // C++11
#include <algorithm> // C++11, for sort()

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "logger.h"
#include "imageAcquisition.h"
#include "stereoCal.h"
#include "benchmarker.h"

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"


// Parent class of all point cloud generation algorithms
class StereoPtCloudGenAlg {
protected:
	list<const Benchmarker *> * bms;
	Logger * logger;
	bool showImages;
	double stereoDistThreshold;
	StereoCal cal_data;
	
	bool cvWindowsAreOpen = false;
	
	Benchmarker bmShowingImages;
	Benchmarker bmClearingPtCloud;
	
	
public:
	virtual void init(nlohmann::json options, Logger * lgr, StereoCal calData);
	
	// Processes a set of raw input images and creates a point cloud message.
	// If processing is not successful, the message will contain 0 points.
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
	
	// Implementors are permitted to open feedback windows.
	// This permits the caller to perform the required cvWait.
	bool areCvWindowsOpen() { return cvWindowsAreOpen; }
	
	StereoPtCloudGenAlg(list<const Benchmarker *> * _bms) : 
		bms(_bms),
		bmShowingImages  ("Showing images"      ),
		bmClearingPtCloud("Clearing point cloud")
	{
		bms->push_back(&bmShowingImages  );
		bms->push_back(&bmClearingPtCloud);
	}
	virtual ~StereoPtCloudGenAlg() {};
};
#endif // __PCG_ALGS_H__
