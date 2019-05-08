/*
	pcg.h
	
	Declarations for the Point Cloud Generator application.
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_H__
#define __PCG_H__

// Standard C++ libraries
#include <unistd.h> // for sync()
#include <list>
#include <chrono>

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

// Headers internal to this project
#include "imageAcquisition.h"
#include "imageProcessing.h"
#include "logger.h"
#include "attitudeTracker.h"
#include "lidarReader.h"
#include "dummyPointCloud.h"

class PcgMain {
private:
	static const char * DEFAULT_CONFIG_FILENAME;

	// Submodule objects
	list<const Benchmarker *> allBms;
	Logger logger;
	ImageAcquisition img_acquisition;
	ImageProcessing img_processing;
	AttitudeTracker attitude_tracker;
	LidarReader lidar;
	Benchmarker bmOneFrame;
	Benchmarker bmSyncFs;

	// ROS
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle rosNodeHandle;

	// Items controlled by the configuration file
	bool pcgEnabled; 
	bool useLidar; 
	bool useStereoVision; 
	bool correctLidarPointCloud;
	bool correctStereoPointCloud;
	bool outputPcRecEnabled;
	string recDataPath;
	string recOutputPcTsPattern;
	
	// Private methods
	void init(char * config_fn);
	void summarizeBenchmarksToLog();
	
public:
	int main(char * config_fn);
	PcgMain() : 
		allBms(),
		img_acquisition(&allBms),
		img_processing (&allBms),
		lidar          (&allBms),
		bmOneFrame  ("Frame total"),
		bmSyncFs    ("Synchronizing filesystem")
	{
		allBms.push_back(&bmOneFrame  );
		allBms.push_back(&bmSyncFs    );
	}
};

#endif // __PCG_H__
