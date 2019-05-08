/*
	spiDevice.h
	
	Class for interacting with SPI devices from the Auvidea J120.
	You must run successfully run ./JetsonTX1/install_spi_support.sh for this 
	to work correctly.
	
	Based on spidev_test.c
	
	2017-4-12  JDW  Created.
	2017-5-1   JDW  Changed from MPU9250 support to general SPI device support.
*/

#ifndef __SPI_DEVICE_H__
#define __SPI_DEVICE_H__

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "logger.h"

class SpiDevice {
protected:
	// Object state
	Logger * logger;
	bool isOpen = false;
	int portFd;
	struct {
		uint8_t mode = 0;
		uint8_t bits = 8;
		uint32_t speed = 500000;
		uint16_t delay = 0;
	} spiSettings;
	
public:
	virtual void init(Logger * lgr, std::string devicePath = "/dev/spidev3.0");
	virtual ~SpiDevice();
};


// Support for devices that have I2C-like registers
class RegBasedSpiDevice : public SpiDevice {
protected:
	static const uint8_t READ_BIT = 0x80;
	
	// Shortcuts for writing a specific amount of data.
	// All of these return true for success.
	bool writeByteRegister(uint8_t reg, uint8_t data);

	// Shortcuts for reading a specific amount of data.
	// All of these return true for success.
	bool readByteRegister(uint8_t reg, uint8_t* dest);
	bool readWordRegister(uint8_t reg, uint16_t* dest);
	bool readThreeWordRegisters(uint8_t start_reg, uint16_t* dest0, uint16_t* dest1, uint16_t* dest2);
	
	// Generalized transfer function, called by reader functions.
	// Reads LEN bytes starting at register reg, and stores the results at dest.
	// Assumes at least LEN bytes are available at dest.
	template <int LEN>
	bool readDataStartingAtReg(uint8_t reg, uint8_t* dest);
	
	void test();
};

#endif // __SPI_DEVICE_H__