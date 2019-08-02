/*
	SpiDev.hpp
	
	Wrapper for the spidev kernel driver for SPI ports.
	
	2019-06-24	JDW	Created.
*/

#ifndef __SPIDEV_HPP__
#define __SPIDEV_HPP__
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <stdexcept>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std;

class SpiDev {
	
public:
	SpiDev() {}
	SpiDev(const SpiDev&) = default;
	SpiDev & operator=(const SpiDev&) = default;
	virtual ~SpiDev() {
		closePort();
	}
	
	/**
	 * Open and setup the port (on this machine) connected to the device.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param clockRateHz desired bus serial clock rate, in Hz
	 * @param mode SPI mode, where bit 0 is CPHA and bit 1 is CPOL.
	 */
	void openPort(const char * devName, unsigned int clockRateHz = 500000, unsigned int mode = 0) {
		spiPortFd = openPortStatic(devName, clockRateHz, mode);
		clockRateOverrideHz = clockRateOverrideHz;
	}
	
	/**
	 * Get the file descriptor for the SPI port this device is using.
	 * @returns the file descriptor used by this object, or -1 if unavailable
	 */
	int getFd() { return spiPortFd; }
	/**
	 * Use a port that has been opened already by openPort() on another device or SpiDev::openPortStatic()
	 * Supports use of the spidev driver only.
	 * Will not configure the port, so make sure you only use this with compatible sensors.
	 * 
	 * @param fd
	 */
	void openPort(int fd, unsigned int clockRateOverrideHz = 500000) {
		spiPortFd = fd;
		clockRateOverrideHz = clockRateOverrideHz;
	}
	
	/**
	 * Close the port.
	 */
	void closePort() {
		closePortStatic(spiPortFd);
	}
	
	bool isOpen() {
		return spiPortFd >= 0;
	}
	
	/**
	 * Open and setup the port (on this machine) but don't associate it with any device yet.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param clockRateHz desired bus serial clock rate, in Hz
	 * @returns the file descriptor of the configured spidev port
	 */
	static int openPortStatic(const char * devName, unsigned int clockRateHz = 500000, unsigned int mode = 0) {
		int fd;
		if((fd = open(devName, O_RDWR)) < 0) {
			throw runtime_error("Failed to open SPI port.");
		}
		configurePort(fd, clockRateHz, mode);
		return fd;
	}
	
	/**
	 * Close the port.
	 */
	static void closePortStatic(int & fd) {
		close(fd);
		fd = -1;
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
		xfer.tx_buf = (__u64)dataOut;
		xfer.rx_buf = (__u64)dataIn;
		xfer.len = len;
		xfer.speed_hz = clockRateOverrideHz;
		xfer.bits_per_word = 8;
		
		// Currently unused fields:
		// xfer.delay_usecs; // If nonzero, how long to delay after the last bit transfer before optionally deselecting the device before the next transfer.
		// xfer.cs_change; //True to deselect device before starting the next transfer.
		// xfer.tx_nbits;  // ??? Do we need to do anything with this?
		// xfer.rx_nbits;  // ??? Do we need to do anything with this?
		
		if(dataOut != NULL) {
			printf("SpiDev (%d): writing: ", spiPortFd);
			for(unsigned int i = 0; i < len; ++i) {
				printf("%02X ", dataOut[i]);
			}
			printf("\n");
		}
		
		if (ioctl(spiPortFd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
			throw runtime_error("Failed to transfer on SPI port.");
		}
		if(dataIn != NULL) {
			printf("SpiDev (%d): read: ", spiPortFd);
			for(unsigned int i = 0; i < len; ++i) {
				printf("%02X ", dataIn[i]);
			}
			printf("\n");
		}
	}
	
	unsigned int getClockRate() {
		__u32 clockRateHz;
		if (ioctl(spiPortFd, SPI_IOC_RD_MAX_SPEED_HZ, &clockRateHz) < 0) {
			throw runtime_error("Failed to get SPI port read max clock rate.");
		}
		return clockRateHz;
	}
	
private:
	int spiPortFd = -1;
	unsigned int clockRateOverrideHz = 0;
	static void configurePort(int fd, unsigned int clockRateHz, unsigned int mode) {
		uint8_t mode8bit = mode;
		
		// Speed
		if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &clockRateHz) < 0) {
			throw runtime_error("Failed to set SPI port max clock rate.");
		}
		// if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &clockRateHz) < 0) {
			// throw runtime_error("Failed to read SPI port max clock rate.");
		// }
		// SPI mode (CPHA|CPOL)
		if (ioctl(fd, SPI_IOC_WR_MODE, &mode8bit) < 0) {
			throw runtime_error("Failed to set SPI port mode.");
		}
		// if (ioctl(fd, SPI_IOC_RD_MODE, &mode8bit) < 0) {
			// throw runtime_error("Failed to read SPI port mode.");
		// }
		
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
};

#endif // __SPIDEV_HPP__
