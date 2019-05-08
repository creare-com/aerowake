/*
	imu.h
	
	Class for reading the MPU-9250 IMU aboard the Auvidea J120.
	You must run successfully run ./JetsonTX1/install_spi_support.sh for this 
	to work correctly.
	
	Based on spidev_test.c
	
	2017-4-12  JDW  Created.
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "spiDevice.h"
#include "logger.h"

class Imu : public RegBasedSpiDevice {
private:
	// Register numbers as written in the datasheet (read/write bit left as 0)
	enum REG_NUM : uint8_t {
		GYRO_CONFIG = 0x1B,
		GYRO_X = 0x43, // Two bytes
		GYRO_Y = 0x45, // Two bytes
		GYRO_Z = 0x47, // Two bytes
		ACCEL_CONFIG = 0x1C,
		ACCEL_X = 0x3B, // Two bytes
		ACCEL_Y = 0x3D, // Two bytes
		ACCEL_Z = 0x3F, // Two bytes
		TEMPERATURE = 0x41, // Two bytes
	};
	static const uint8_t GYRO_FSR_BITMASK = 0x17;
	static const int GYRO_FSR_BITSHIFT = 3;
	static const uint8_t ACCEL_FSR_BITMASK = 0x17;
	static const int ACCEL_FSR_BITSHIFT = 3;
	static const double GYRO_RES [];
	static const double ACCEL_RES[];
	
	// Object state
	double gyroResolution; // Gyro full-scale range, only accurate after init
	double accelResolution; // Accelerometer full-scale range, only accurate after init
	
	// Pulls sensor FSR settings into member variables
	bool readSettings();
public:
	void init(Logger * lgr, std::string devicePath = "/dev/spidev3.0"); // TODO: add options
	bool readGyro(double &x, double &y, double &z);
	bool readAccelerometer(double &x, double &y, double &z);
};