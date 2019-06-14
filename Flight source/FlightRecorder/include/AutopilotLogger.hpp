/*
	AutopilotLogger.hpp
	
	Class for logging data from an mavlink2-compatible autopilot
	
	2019-06-14	JDW	Created.
*/

#ifndef __AUTOPILOTLOGGER_HPP__
#define __AUTOPILOTLOGGER_HPP__

#include <string>

#include <SerialPort.hpp>
#include <AutopilotInterface.hpp>
#include <CsvLogger.hpp>


using namespace std;

class AutopilotLogger {
public:
	/**
	 * Constructor. 
	 */
	AutopilotLogger(string recordingDir, string logFilenameFormat, string autopilotPort, int apBaudRate) :
		logger(recordingDir, logFilenameFormat),
		apSerialPort(autopilotPort.c_str(), apBaudRate),
		apIntf(&apSerialPort)
	{ 
		
	}
	virtual ~AutopilotLogger() {
		stopLogging();
	}
	
	void startLogging();
	void stopLogging();
	
private:
	Serial_Port apSerialPort;
	Autopilot_Interface apIntf;

	CsvLogger logger;
};

#endif // __AUTOPILOTLOGGER_HPP__

