/*
	SpiSensor.hpp
	
	Base class for all sensors accessed via the SPI port
	
	2019-06-24	JDW	Created.
*/

class SpiSensor {
	
public:
	SpiSensor();
	virtual ~SpiSensor();
	
	/**
	 * Open and setup the port (on this machine) connected to the device.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param maxClockRate maximum bus serial clock rate, in Hz
	 * @param mode (CPHA << 1) |  CPOL
	 */
	void open(const char * devName, unsigned int maxClockRate, unsigned int mode) {
		spiPortFd = openPort(devName);
		configurePort(maxClockRate);
	}
	
	/**
	 * Get the file descriptor for the SPI port this device is using.
	 * @returns the file descriptor used by this object, or -1 if unavailable
	 */
	int getFd() { return spiPortFd; }
	/**
	 * Use a port that has been opened already by open() on another device or SpiSensor::openPort()
	 * Supports use of the spidev driver only.
	 * 
	 * @param fd
	 */
	void open(int fd) {
		spiPortFd = fd;
	}
	
	/**
	 * Open and setup the port (on this machine) but don't associate it with any device yet.
	 * Supports use of the spidev driver only.
	 * 
	 * @param devName the name of the spidev device as a null-terminated cstring, eg "/dev/spidev0.0"
	 * @param maxClockRate maximum bus serial clock rate, in Hz
	 */
	static int openPort(const char * devName) {
		// TODO
	}
	
	/**
	 * Call this when the CS line should remain asserted for several transactions.
	 * @param latch true to start asserting CS.  false to reset the state of the CS line so it is 
	 *   only asserted during read/write/transfer operations.
	 */
	void latchCs(bool latch) {
		// TODO
	}
	
	void write(const char * data, unsigned int len) {
		// TODO
	}
	void read(char * data, unsigned int len) {
		// TODO
	}
	void transfer(char * data, unsigned int len) {
		// TODO
	}
	
private:
	int spiPortFd = -1;
	void configurePort(unsigned int maxClockRate, unsigned int mode) {
		// TODO
	}
}