/*
	WindProbeLogger.hpp
	
	Class for logging data from the Wake Swarm wind probe
	
	2019-06-24	JDW	Created.
*/

#ifndef __WINDPROBELOGGER_HPP__
#define __WINDPROBELOGGER_HPP__

#include <string>
#include <atomic>
#include <thread>

#include <sensors/DlhrPressureSensor.hpp>
#include <sensors/DlvPressureSensor.hpp>
#include <sensors/Max6682.hpp>
#include <Adg725.hpp>
#include <SpiDev.hpp>
#include <CsvLogger.hpp>


using namespace std;

class WindProbeLogger {
public:
	static const unsigned int NUM_DLHR_SENSORS = 12;
	/**
	 * Constructor. 
	 */
	WindProbeLogger(string recordingDir, string logFilenameFormat, string sensorPortName, string muxPortName, unsigned int clockRate);

	virtual ~WindProbeLogger() {
		stopLogging();
	}
	
	void startLogging();
	void stopLogging();
	
private:
	// Mux number for each sensor.
	// Note that these are 1-indexed on the board, and 0-indexed in software.
	enum MuxAssignment_e {
		DLHR_1  =  0,
		DLHR_2  =  1,
		DLHR_3  =  2,
		DLHR_4  =  3,
		DLHR_5  =  4,
		DLHR_6  =  5,
		DLHR_7  =  6,
		DLHR_8  =  7,
		DLHR_9  =  8,
		DLHR_10 =  9,
		DLHR_11 = 10,
		DLHR_12 = 11,
		DLV     = 12,
		MAX6682 = 13
	};

	// mux port on which each probe channel sensor resides
	static const unsigned char dlhrMuxNum[NUM_DLHR_SENSORS];
	static const unsigned int MS_PER_READ;

	string sensorPortName;
	string muxPortName;
	SpiDev sensorPort;
	SpiDev muxPort;
	unsigned int clockRate;
	
	unsigned int logIdDlhrPressure[NUM_DLHR_SENSORS];
	unsigned int logIdDlhrTemperature[NUM_DLHR_SENSORS];
	unsigned int logIdDlvPressure;
	unsigned int logIdDlvTemperature;
	unsigned int logIdMax6682AdcTicks;
	
	// Sensor reader objects
	Adg725 multiplexer;
	DLHR_L01D dlhrSensor;
	DLV_030A dlvSensor;
	Max6682 thermistorReader;

	CsvLogger logger;
	
	atomic<bool> runReadThread;
	thread readThread;
	
	static void threadFunc(WindProbeLogger * wp) {
		wp->readAndLog();
	}
	
	/**
	 * Continually read and log data until runReadThread becomes false.
	 *
	 */
	void readAndLog();
	
	/**
	 * Tell all the DLHR sensors to begin taking a reading.
	 */
	void startReadings();
	/**
	 * Retrieve a reading from every sensor and log it.
	 */
	void logReadings();
};

#endif // __WINDPROBELOGGER_HPP__

