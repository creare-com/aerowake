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
	/**
	 * Constructor. 
	 */
	WindProbeLogger(string recordingDir, string logFilenameFormat, string autopilotPort, int apBaudRate) :
		logger(recordingDir, logFilenameFormat)
	{ 
		SpiDev port;
		DLHR_L01D probeSensor(port);
		Adg725 mux(port);
		DLV_030A absSensor(port);
	}
	virtual ~WindProbeLogger() {
		stopLogging();
	}
	
	void startLogging();
	void stopLogging();
	
private:

	CsvLogger logger;
};

#endif // __WINDPROBELOGGER_HPP__

