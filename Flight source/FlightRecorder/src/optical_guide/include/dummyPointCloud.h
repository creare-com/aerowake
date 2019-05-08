/*
	dummyPointCloud.h
	
	False data for use in testing
	
	2017-07-25  JDW  Created.
*/

#ifndef __DUMMYPOINTCLOUD_H__
#define __DUMMYPOINTCLOUD_H__

#include <math.h>

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"



class DummyPointCloud {
public:

	static sensor_msgs::PointCloud getMsg(double t = 0) {
		const int SIZE = 13;
		const double SLOPE = 0.5;
		const double PERIOD_S = 1.0;
		const double AMPLITUDE = 0.01;
		sensor_msgs::PointCloud msg;
		
		msg.header.seq = 1;
		msg.header.frame_id = "/platform";
		msg.header.stamp.sec = 0;
		msg.header.stamp.nsec = 0;
		
		for(int i = 0; i < SIZE; ++i) {
			geometry_msgs::Point32 pt;
			double depth = -0.01 * i;
			double theta = (-t / PERIOD_S + ((double)i / (double)SIZE)) * (2.0 * M_PI);
			pt.x = depth;
			pt.y = depth * SLOPE;
			pt.z = AMPLITUDE * sin(theta);
			msg.points.push_back(pt);
			pt.y *= -1.0;
			msg.points.push_back(pt);
		}
		
		return msg; 
	};
};

#endif // __DUMMYPOINTCLOUD_H__
