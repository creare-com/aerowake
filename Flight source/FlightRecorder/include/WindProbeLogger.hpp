/*
	WindProbeLogger.hpp
	
	Class for logging data from the Wake Swarm wind probe
	
	2019-06-24	JDW	Created.
*/

#ifndef __WINDPROBELOGGER_HPP__
#define __WINDPROBELOGGER_HPP__

#include <string>

#include <sensors/DlhrPressureSensor.hpp>
#include <sensors/DlvPressureSensor.hpp>
#include <Adg725.hpp>
#include <SpiDev.hpp>
#include <CsvLogger.hpp>


using namespace std;

class WindProbeLogger {
public:
	static const unsigned int NUM_DLHR_SENSORS = 12;
	static const unsigned int DLV_MUX_CHANNEL = 13;
	static const unsigned int MAX6682_MUX_CHANNEL = 14;
	
	/**
	 * Constructor. 
	 */
	WindProbeLogger(string recordingDir, string logFilenameFormat, string sensorPortName, string muxPortName, unsigned int clockRate);

	virtual ~WindProbeLogger() {
		stopLogging();
	}
	
	void openPorts() {
		sensorPort.openPort(sensorPortName.c_str(), clockRate, 0);
		muxPort.openPort(muxPortName.c_str(), clockRate, 1); // CPHA = 1 for the muxPort
	}
	
	void startLogging();
	void stopLogging();
	
private:
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
	
	

	CsvLogger logger;
};

#endif // __WINDPROBELOGGER_HPP__

