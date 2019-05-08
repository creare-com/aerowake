/*
	logger.h
	
	Declarations for logging
	
	2017-01-06  JDW  Created.
*/

#ifndef __PCG_LOG_H__
#define __PCG_LOG_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono> // C++11
#include <ctime> // Bits that C++11 is missing
#include "json.hpp"
using json = nlohmann::json;
using namespace std;

typedef enum LogVerbosityTag {
	LOG_VERB_SILENT  = 0,
	LOG_VERB_ERROR   = 1,
	LOG_VERB_WARNING = 2,
	LOG_VERB_INFO    = 3,
	LOG_VERB_DEBUG   = 4,
} LogVerbosity;

class Logger {
private:
	int statusLoggingLevel = LogVerbosity::LOG_VERB_INFO;
	ostream * statusLogStream;
	ofstream statusLogFile;
	bool haveStatusstatusLogStream = false;
	bool supportColors = false;

	string getTimeString();
	string timestampPattern = "";
	void log(string message, string color = "");
	
	// Terminal color codes.
	// Only works in terminals that support coloring.
	// Courtesy https://stackoverflow.com/a/30304782/415551
	const string COLOR_NONE   = "";
	const string COLOR_RESET  = "\x1B[0m";
	const string COLOR_RED    = "\x1B[31m";
	const string COLOR_YELLOW = "\x1B[33m";
	const string COLOR_GREEN  = "\x1B[32m";
	const string COLOR_CYAN   = "\x1B[36m";
	
public:
	void init(json options);
	void log(string message, LogVerbosity level);
	void logError   (string message);
	void logWarning (string message);
	void logInfo    (string message);
	void logDebug   (string message);
};


#endif // __PCG_LOG_H__
