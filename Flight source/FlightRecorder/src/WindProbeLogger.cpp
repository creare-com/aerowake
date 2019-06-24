/*
	WindProbeLogger.cpp
	
	Class for logging data from the Wake Swarm wind probe
	
	2019-06-24	JDW	Created.
*/

#include <WindProbeLogger.hpp>

using namespace std;

void WindProbeLogger::startLogging() {
	
	// Set up log
	unsigned int logXX      = logger.addColumn("XX");
	// Add all columns before this
	logger.startNewLogFile();
	
}
void WindProbeLogger::stopLogging() {

}
