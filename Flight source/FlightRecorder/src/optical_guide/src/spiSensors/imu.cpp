/*
	imu.cpp
	
	Class for reading the MPU-9250 IMU aboard the Auvidea J120.
	You must run successfully run ./JetsonTX1/install_spi_support.sh for this 
	to work correctly.
	
	Based on spidev_test.c
	
	2017-4-12  JDW  Created.
*/
#include <spiSensors/imu.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

// Possible gyro resolutions, in LSb/degree/sec
const double Imu::GYRO_RES[] = {
	131.0,
	65.5,
	32.8,
	16.4
};
// Possible acclerometer resolutions, in LSb/g
const double Imu::ACCEL_RES[] = {
	16384.0,
	8192.0,
	4096.0,
	2048.0
};

bool Imu::readSettings() {
	uint8_t gyro_settings, accel_settings;
	bool success = true;
	success = success && readByteRegister(GYRO_CONFIG,  &gyro_settings);
	success = success && readByteRegister(ACCEL_CONFIG, &accel_settings);
	
	int fs_sel, afs_sel;
	fs_sel  = (gyro_settings  & GYRO_FSR_BITMASK ) >> GYRO_FSR_BITSHIFT;
	afs_sel = (accel_settings & ACCEL_FSR_BITMASK) >> ACCEL_FSR_BITSHIFT;
	gyroResolution  = GYRO_RES [fs_sel ];
	accelResolution = ACCEL_RES[afs_sel];
	stringstream ss;
	ss << (success? "Got IMU settings" : "Failed to get IMU settings, defaults are") 
		<< ": gyro: " << gyro_settings 
		<< ", accel: " << accel_settings
		<< "." << endl
		<< "So gyroResolution=" << gyroResolution 
		<< ", accelResolution=" << accelResolution;
	logger->logDebug(ss.str());
	
	return success;
}

void Imu::init(Logger * lgr, std::string devicePath) {
	SpiDevice::init(lgr, devicePath);

	readSettings();
}

bool Imu::readGyro(double &x, double &y, double &z) {
	int16_t gx, gy, gz;
	bool success;
	success = readThreeWordRegisters(REG_NUM::GYRO_X,
		(uint16_t*)&gx,
		(uint16_t*)&gy,
		(uint16_t*)&gz);
	x = gx / gyroResolution;
	y = gy / gyroResolution;
	z = gz / gyroResolution;
	return success;
}


bool Imu::readAccelerometer(double &x, double &y, double &z) {
	int16_t ax, ay, az;
	bool success;
	success = readThreeWordRegisters(REG_NUM::ACCEL_X, 
		(uint16_t*)&ax,
		(uint16_t*)&ay,
		(uint16_t*)&az);
	x = ax / accelResolution;
	y = ay / accelResolution;
	z = az / accelResolution;
	return success;
}
