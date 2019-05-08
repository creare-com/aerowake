/*
	attitudeTracker.h
	
	This class interfaces with the IMU (Inertial Measurement Unit) on the J120
	and performs all corrections required to report the platform's attitude.
	
	2017-05-16  JDW  Created.
*/

#ifndef __ATT_TRACKER_H__
#define __ATT_TRACKER_H__

#include <stdio.h>
#include <iostream>
#include <list>
#include <utility>
#include <thread>
// #include <sys/time.h>
#include <chrono> // C++11
#include "spiSensors/MPU9250.h"
#include "logger.h"





// Used as a struct
class Attitude {
public:
	// In radians
	double roll    = 0, // Positive means dipping the right wing
	       pitch   = 0, // Positive means raising the nose
	       heading = 0; // Clockwise from north
	// In radians/s
	double rollRate  = 0, // Same sign convention as above
	       pitchRate = 0, // Same sign convention as above
	       yawRate   = 0; // Same sign convention as above
	// In G, deviation from freefall
	double fwdAccel   = 0, 
	       rightAccel = 0, // Toward starboard; the right wing
	       downAccel  = 0; 
};

class AttitudeTracker {
private:
	Logger * logger;

	// Configuration variables
	double magCal[3];
	bool logDataToFile;
	// Indicies into the IMU accel & gyro axes; must be 0-2.
	// eg fwdAxis == 0 indicates the IMU's X axis is oriented facing
	// forward.  Set fwdAxisPol to -1 if that axis is facing backward.
	unsigned int fwdAxis   = 1,
	             rightAxis = 0,
	             downAxis  = 2;
	// Must be -1 or +1
	int fwdAxisPol   = -1,
	    rightAxisPol =  1,
	    downAxisPol  =  1;
	
	// Misc member data
	MPU9250 imu;
	ofstream dataFile;
	bool enabled;
	
	// The magnetometer is a different die, mounted differently.
	// It flips the X and Y axes while maintaining sign, then flips
	// the sign on the Z axis to maintain a right-handed coordinate system.
	// Because it would be way too easy otherwise.
	static const unsigned int MAG_AXIS_FOR_ACCEL_AXIS[3];
	static const int MAG_AXIS_SIGN_FOR_ACCEL_AXIS[3];
	static const char TIMESTAMP_FMT[];
public:
	void init(nlohmann::json options, Logger * lgr);
	Attitude estimateAttitude();
	
};


#endif // __ATT_TRACKER_H__
