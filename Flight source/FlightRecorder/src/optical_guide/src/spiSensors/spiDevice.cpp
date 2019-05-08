/*
	spiDevice.cpp
	
	Class for interacting with SPI devices from the Auvidea J120.
	You must run successfully run ./JetsonTX1/install_spi_support.sh for this 
	to work correctly.
	
	Based on spidev_test.c
	
	2017-4-12  JDW  Created.
	2017-5-1   JDW  Changed from MPU9250 support to general SPI device support.
*/
#include <spiSensors/spiDevice.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

bool RegBasedSpiDevice::writeByteRegister(uint8_t reg, uint8_t data) {
	// Make two temporary arrays on the stack.
	// Need an extra byte for the register number.
	uint8_t tx[2] = {reg, data};
	uint8_t rx[2];
	memset(rx, 0, sizeof(rx));

	struct spi_ioc_transfer tr;
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = 2;
	tr.delay_usecs = spiSettings.delay;
	tr.speed_hz = spiSettings.speed;
	tr.bits_per_word = spiSettings.bits;
	
	int ret = ioctl(portFd, SPI_IOC_MESSAGE(1), &tr);
	return ret >= 1;
}


bool RegBasedSpiDevice::readByteRegister(uint8_t reg, uint8_t* dest) {
	return readDataStartingAtReg<1>(reg, dest);
}

bool RegBasedSpiDevice::readWordRegister(uint8_t reg, uint16_t* dest) {
	uint8_t data[2];
	bool success = readDataStartingAtReg<2>(reg, (uint8_t*)data);
	// Only necessary on LSB-first platforms such as the Jetson
	*dest = data[0] << 8 | data[1];
	return success;
}

bool RegBasedSpiDevice::readThreeWordRegisters(uint8_t start_reg, uint16_t* dest0, uint16_t* dest1, uint16_t* dest2) {
	uint8_t data[6];
	bool success = readDataStartingAtReg<6>(start_reg, data);
	// Only necessary on LSB-first platforms such as the Jetson
	*dest0 = data[0] << 8 | data[1];
	*dest1 = data[2] << 8 | data[3];
	*dest2 = data[4] << 8 | data[5];
	return success;
}

template <int LEN>
bool RegBasedSpiDevice::readDataStartingAtReg(uint8_t reg, uint8_t* dest) {
	// Make two temporary arrays on the stack.
	// Need an extra byte for the register number.
	uint8_t tx[LEN+1];
	uint8_t rx[LEN+1];

	memset(tx, 0, LEN+1);
	memset(rx, 0, LEN+1);
	tx[0] = reg | READ_BIT;
	
	struct spi_ioc_transfer tr;
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = LEN+1;
	tr.delay_usecs = spiSettings.delay;
	tr.speed_hz = spiSettings.speed;
	tr.bits_per_word = spiSettings.bits;
	
	int ret = ioctl(portFd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		return false;
	}
	
	// Copy from temporary array into output array
	memcpy(dest, &(rx[1]), LEN);
	
	return true;
}

void RegBasedSpiDevice::test() {
	// TODO: This is an ugly hack to make the compiler generate all
	// the versions of this function that are required elsewhere.  There's 
	// definitely a smarter way to handle this but I haven't yet found it.
	uint8_t temp[21];
	readDataStartingAtReg<3>( (uint8_t)0, temp);
	readDataStartingAtReg<4>( (uint8_t)0, temp);
	readDataStartingAtReg<7>( (uint8_t)0, temp);
	readDataStartingAtReg<12>((uint8_t)0, temp);
	readDataStartingAtReg<21>((uint8_t)0, temp);
	
}

void SpiDevice::init(Logger * lgr, std::string devicePath) {
	const char *device = devicePath.c_str();
	spiSettings.mode = 0;
	spiSettings.bits = 8;
	spiSettings.speed = 500000;
	spiSettings.delay = 0;
	int ret = 0;
	logger = lgr;
	
	portFd = open(device, O_RDWR);
	if (ret < 0) {
		logger->logError("can't open SPI device"); 
		return;
	}
	isOpen = true;
	
	/*
	 * spi mode
	 */
	ret = ioctl(portFd, SPI_IOC_WR_MODE, &(spiSettings.mode));
	if (ret == -1) {
		logger->logError("can't set spi mode"); 
		close(portFd);
		return;
	}

	ret = ioctl(portFd, SPI_IOC_RD_MODE, &(spiSettings.mode));
	if (ret == -1) {
		logger->logError("can't get spi mode");
		close(portFd);
		return;
	}

	/*
	 * bits per word
	 */
	ret = ioctl(portFd, SPI_IOC_WR_BITS_PER_WORD, &(spiSettings.bits));
	if (ret == -1) {
		logger->logError("can't set bits per word");
		close(portFd);
		return;
	}

	ret = ioctl(portFd, SPI_IOC_RD_BITS_PER_WORD, &(spiSettings.bits));
	if (ret == -1) {
		logger->logError("can't get bits per word");
		close(portFd);
		return;
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(portFd, SPI_IOC_WR_MAX_SPEED_HZ, &(spiSettings.speed));
	if (ret == -1) {
		logger->logError("can't set max speed hz");
		close(portFd);
		return;
	}

	ret = ioctl(portFd, SPI_IOC_RD_MAX_SPEED_HZ, &(spiSettings.speed));
	if (ret == -1) {
		logger->logError("can't get max speed hz");
		close(portFd);
		return;
	}

	stringstream ss;
	ss << "spi mode = " << (int)(spiSettings.mode)
		<< ", bits per word = " << (int)(spiSettings.bits)
		<< ", max speed = " << spiSettings.speed << "Hz (" << spiSettings.speed/1000 << " KHz)";
	logger->logDebug(ss.str());
	
}
SpiDevice::~SpiDevice() {
	if(isOpen) {
		close(portFd);
	}
}
