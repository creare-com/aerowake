/*
	SpiSensor.hpp
	
	Base class for all sensors accessed via the SPI port
	
	2019-06-24	JDW	Created.
*/

#ifndef __SPISENSOR_HPP__
#define __SPISENSOR_HPP__

#include <linux/spi/spidev.h>

class SpiSensor {
	
public:
	SpiSensor();
	virtual ~SpiSensor();
	
	/**
	 * Open and setup the port (on this machine) connected to the device.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param clockRateHz desired bus serial clock rate, in Hz
	 * @param mode SPI mode, where bit 0 is CPHA and bit 1 is CPOL.
	 */
	void open(const char * devName, unsigned int clockRateHz = 500000, unsigned int mode = 0) {
		spiPortFd = openPort(devName, clockRateHz, mode);
	}
	
	/**
	 * Get the file descriptor for the SPI port this device is using.
	 * @returns the file descriptor used by this object, or -1 if unavailable
	 */
	int getFd() { return spiPortFd; }
	/**
	 * Use a port that has been opened already by open() on another device or SpiSensor::openPort()
	 * Supports use of the spidev driver only.
	 * Will not configure the port, so make sure you only use this with compatible sensors.
	 * 
	 * @param fd
	 */
	void open(int fd, unsigned int clockRateOverrideHz = 0) {
		spiPortFd = fd;
		clockRateOverrideHz = clockRateOverrideHz;
	}
	
	/**
	 * Open and setup the port (on this machine) but don't associate it with any device yet.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param clockRateHz desired bus serial clock rate, in Hz
	 * @returns the file descriptor of the configured spidev port
	 */
	static int openPort(const char * devName, unsigned int clockRateHz = 500000, unsigned int mode = 0) {
		int fd;
		if((fd = open(devName, O_RDWR)) < 0) {
			throw new runtime_exception("Failed to open SPI port.");
		}
		configurePort(fd, clockRateHz, mode);
		return fd;
	}
	
	/**
	 * Call this when the CS line should remain asserted for several transactions.
	 * @param latch true to start asserting CS.  false to reset the state of the CS line so it is 
	 *   only asserted during read/write/transfer operations.
	 */
	void latchCs(bool latch) {
		// TODO
	}
	
	void write(const char * dataOut, unsigned int len) {
		transfer(NULL, dataOut, len);
	}
	
	void read(char * dataIn, unsigned int len) {
		transfer(dataIn, NULL, len);
	}
	
	void transfer(char * dataIn, const char * dataOut, unsigned int len) {
		// Initialize transfer settings with 0s
		spi_ioc_transfer xfer;
		memset(&xfer, 0, sizeof(xfer));
		xfer.tx_buf = dataOut;
		xfer.rx_buf = dataIn;
		xfer.len = len;
		xfer.speed_hz = clockRateOverrideHz;
		xfer.bits_per_word = 8;
		
		// Currently unused fields:
		// xfer.delay_usecs; // If nonzero, how long to delay after the last bit transfer before optionally deselecting the device before the next transfer.
		// xfer.cs_change; //True to deselect device before starting the next transfer.
		// xfer.tx_nbits;  // ??? Do we need to do anything with this?
		// xfer.rx_nbits;  // ??? Do we need to do anything with this?
		
		if (ioctl(spiPortFd, SPI_IOC_MESSAGE(0), &xfer) < 0) {
			throw new runtime_exception("Failed to transfer on SPI port.");
		}
	}
	
private:
	int spiPortFd = -1;
	unsigned int clockRateOverrideHz = 0;
	static void configurePort(int fd, unsigned int clockRateHz, unsigned int mode) {
		uint8_t mode8bit = mode;
		
		// Speed
		if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &clockRateHz) < 0) {
			throw new runtime_exception("Failed to set SPI port read max clock rate.");
		}
		if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &clockRateHz) < 0) {
			throw new runtime_exception("Failed to set SPI port write max clock rate.");
		}
		// SPI mode (CPHA|CPOL)
		if (ioctl(fd, SPI_IOC_RD_MODE, &mode8bit) < 0) {
			throw new runtime_exception("Failed to set SPI port read mode.");
		}
		if (ioctl(fd, SPI_IOC_WR_MODE, &mode8bit) < 0) {
			throw new runtime_exception("Failed to set SPI port write mode.");
		}
		
		// Other available settings not presently implemented:
		
		// SPI_CS_HIGH		
		// SPI_LSB_FIRST	
		// SPI_3WIRE		
		// SPI_LOOP		
		// SPI_NO_CS		
		// SPI_READY		
		// SPI_TX_DUAL		
		// SPI_TX_QUAD		
		// SPI_RX_DUAL		
		// SPI_RX_QUAD		
	}
}

#endif // __SPISENSOR_HPP__
