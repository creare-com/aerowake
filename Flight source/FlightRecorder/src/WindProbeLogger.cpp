/*
	WindProbeLogger.cpp
	
	Class for logging data from the Wake Swarm wind probe
	
	2019-06-24	JDW	Created.
*/

#include <WindProbeLogger.hpp>

using namespace std;
/**
 * Constructor. 
 */
WindProbeLogger::WindProbeLogger(string recordingDir, string logFilenameFormat, string sensorPortName, string muxPortName, unsigned int clockRate) :
	logger(recordingDir, logFilenameFormat),
	sensorPortName(sensorPortName),
	muxPortName(muxPortName),
	clockRate(clockRate)
{ 
	// DLHR_L01D probeSensor(port);
	// Adg725 mux(port);
	// DLV_030A absSensor(port);
	
	// Set up log
	for(unsigned int sensorIdx = 0; sensorIdx < NUM_DLHR_SENSORS; ++sensorIdx) {
		stringstream pressColName;
		pressColName << "dlhr_pressure_" << (sensorIdx + 1) << "_inh2o";
		logIdDlhrPressure[sensorIdx] = logger.addColumn(pressColName.str());
		
		stringstream tempColName;
		tempColName << "dlhr_temperature_" << (sensorIdx + 1) << "_degC";
		logIdDlhrTemperature[sensorIdx] = logger.addColumn(tempColName.str());
	}
	
	logIdDlvPressure      = logger.addColumn("dlv_pressure_psi");
	logIdDlvTemperature   = logger.addColumn("dlv_temperature_degC");
	logIdMax6682AdcTicks  = logger.addColumn("max6682_temp_adc_ticks");
}
void WindProbeLogger::startLogging() {
	// Add all columns before this
	logger.startNewLogFile();
}
void WindProbeLogger::stopLogging() {

}
