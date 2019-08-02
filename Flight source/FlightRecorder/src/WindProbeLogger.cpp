/*
	WindProbeLogger.cpp
	
	Class for logging data from the Wake Swarm wind probe
	
	2019-06-24	JDW	Created.
*/

#include <WindProbeLogger.hpp>

using namespace std;

const unsigned char WindProbeLogger::dlhrMuxNum[WindProbeLogger::NUM_DLHR_SENSORS] = {
	DLHR_1 ,
	DLHR_2 ,
	DLHR_3 ,
	DLHR_4 ,
	DLHR_5 ,
	DLHR_6 ,
	DLHR_7 ,
	DLHR_8 ,
	DLHR_9 ,
	DLHR_10,
	DLHR_11,
	DLHR_12
};
const unsigned int WindProbeLogger::MS_PER_READ = 4;

/**
 * Constructor. 
 */
WindProbeLogger::WindProbeLogger(string recordingDir, string logFilenameFormat, string sensorPortName, string muxPortName, unsigned int clockRate) :
	logger(recordingDir, logFilenameFormat),
	sensorPortName(sensorPortName),
	muxPortName(muxPortName),
	clockRate(clockRate),
	multiplexer(muxPort),
	dlhrSensor(sensorPort),
	dlvSensor(sensorPort),
	thermistorReader(sensorPort)
{ 
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
	
	runReadThread = false;
}
void WindProbeLogger::startLogging() {
	// Add all columns before this
	logger.startNewLogFile();
	
	if(!sensorPort.isOpen()) {
		sensorPort.openPort(sensorPortName.c_str(), clockRate, 0);
	}
	if(!muxPort.isOpen()) {
		muxPort.openPort(muxPortName.c_str(), clockRate, 1); // CPHA = 1 for the muxPort
	}

	runReadThread = true;
	readThread = thread(threadFunc, this); // Starts the thread
}
void WindProbeLogger::stopLogging() {
	runReadThread = false;
	readThread.join();
}
/**
 * Tell all the DLHR sensors to begin taking a reading.
 */
void WindProbeLogger::startReadings() {
	for(unsigned int sensorIdx = 0; sensorIdx < NUM_DLHR_SENSORS; sensorIdx++) {
		multiplexer.setMux(dlhrMuxNum[sensorIdx]);
		dlhrSensor.commandReading();
	}
}
/**
 * Retrieve a reading from every sensor and log it.
 * Should be called at least 4ms after calling startReadings(),
 * or the DLHR values may be invalid.
 */
void WindProbeLogger::logReadings() {
	auto row = vector<CsvLogger::Cell>(NUM_DLHR_SENSORS*2 + 3);
	unsigned int col = 0;
	for(unsigned int sensorIdx = 0; sensorIdx < NUM_DLHR_SENSORS; sensorIdx++) {
		multiplexer.setMux(dlhrMuxNum[sensorIdx]);
		auto dlhrReading = dlhrSensor.retrieveReading();
		row[col++] = {logIdDlhrPressure[sensorIdx], dlhrReading.pressureInH20};
		row[col++] = {logIdDlhrTemperature[sensorIdx], dlhrReading.temperatureC};
	}
	
	multiplexer.setMux(DLV);
	auto dlvReading = dlvSensor.retrieveReading();
	row[col++] = {logIdDlvPressure, dlvReading.pressurePsi};
	row[col++] = {logIdDlvTemperature, dlvReading.temperatureC};
	
	multiplexer.setMux(MAX6682);
	int thermistorReading = thermistorReader.getAdcValue();
	row[col++] = {logIdMax6682AdcTicks, (double)thermistorReading};
	
	logger.logData(row);
}

/**
 * Continually read and log data until runReadThread becomes false.
 *
 */
void WindProbeLogger::readAndLog() {
	while(runReadThread) {
		startReadings();
		this_thread::sleep_for(milliseconds(MS_PER_READ));
		logReadings();
	}
}


