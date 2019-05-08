/*

Main entry point for Optical Guide's (6298) magnetometer calibration routine.
When run, this program reads the J120 magnetometer repeatedly.  The user then
is expected to rotate the assembly in all axes.  The updated magnetometer
coefficients are then printed.

2017-05-09  JDW  Created

*/
#include <chrono>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include "spiSensors/MPU9250.h"

using json = nlohmann::json;
using namespace std;
using namespace chrono;

// Entry point
int main(int argc, char ** argv)
{
	// Initialize - we don't really need this logger, but the IMU does
	Logger logger;
	json loggerOptions = {
		{"verbosity", 0},
		{"stream", "cout"},
		{"timestampPattern", "[%Y-%m-%d %X] "}
	};
	logger.init(loggerOptions);
	MPU9250 imu;
	imu.init(&logger);
	double maxes[3], mins[3], coeffs[3];
	
	// Wait for the device to init
	cout << "When run as root, this program will output its best guess at "
		 << "calibration coefficients.  You must rotate the system in a full "
		 << "circle in each axis before these values are accurate." << endl;
	this_thread::sleep_for(milliseconds(1000));

	// CSV column headers
	cout << "rawX, rawY, rawZ, mag_cal[0],  mag_cal[1],  mag_cal[3]" << endl;
	
	// Loop until interrupted.
	bool first_loop = true;
	while(1) {
		imu.read_mag();
		// imu.read_all();
		this_thread::sleep_for(milliseconds(100));
		
		for(int i = 0; i < 3; ++i) {
			if(first_loop) {
				maxes[i] = imu.mag_data[i];
				mins [i] = imu.mag_data[i];
			} else {
				maxes[i] = max(maxes[i], imu.mag_data[i]);
				mins [i] = min(mins [i], imu.mag_data[i]);
			}
			coeffs[i] = (maxes[i] + mins[i]) / 2.0;
		}
		
		cout << imu.mag_data[0] << ", "
			 << imu.mag_data[1] << ", "
			 << imu.mag_data[2] << ", "
			 << coeffs[0] << ", "
			 << coeffs[1] << ", "
			 << coeffs[2] << ", "
			 << endl;
		first_loop = false;
	}
}