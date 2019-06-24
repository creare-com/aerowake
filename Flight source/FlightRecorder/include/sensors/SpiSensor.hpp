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
	 */
	void open(const char * devName, unsigned int maxClockRate) {
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
	
private:
	int spiPortFd = -1;
	void configurePort(unsigned int maxClockRate) {
		// TODO
	}
}