/*
	CsvLogger.hpp
	
	Class for saving data to a CSV log.
	
	Usage:
	
	CsvLogger myLogger;
	auto col1 = myLogger.addColumn("foo");
	auto col2 = myLogger.addColumn("bar");
	myLogger.startNewLogFile();
	
	myLogger.logData(CsvLogger::cell(col1, 1));
	myLogger.logData(vector<CsvLogger::cell>({CsvLogger::cell(col1, 1), CsvLogger::cell(col1, 1)}));
	
	
	2019-06-13	JDW	Created.
*/

#ifndef __CSVLOGGER_HPP__
#define __CSVLOGGER_HPP__

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <date.h>

using namespace std;

class CsvLogger {
public:
	/**
	 * Constructor.  Will not attempt to open log file.
	 *
	 * @param recordingDir The directory in which to save the file(s)
	 * @param logFilenameFormat Pattern specifying the filename for log data. Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Avoid any characters not supported by the filesystem, such as colons.
	 * @param timestampFormat Pattern specifying the timestamp in the first column. Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.
	 */
	CsvLogger(string recordingDir = ".", string logFilenameFormat = "data_%F_%H-%M-%S.csv", string timestampFormat = "%H:%M:%S") :
		recordingDir      (recordingDir     ),
		logFilenameFormat (logFilenameFormat),
		timestampFormat   (timestampFormat  )
	{ 
		headers.push_back("time");
	}
	virtual ~CsvLogger() {
		closeFile();
	}
	
	/**
	 * Close old log file, if one was open.
	 * Open a new log file for subsequent calls to logData().
	 */
	void startNewLogFile();
	
	/**
	 * Call this before logging data.
	 * @param header Single-token header
	 * @return the column ID
	 */
	unsigned int addColumn(string header);
	
	/**
	 * A cell in a CSV table
	 */
	class Cell {
	public:
		unsigned int id;
		double value;
		
		Cell(unsigned int id, double value) : id(id), value(value) { ; }
		friend bool operator< (const Cell &c1, const Cell &c2) { return c1.value < c2.value; }
	};
	
	/**
	 * Log one timestamped value on its own line.
	 * @param value Value to log
	 */
	void logData(Cell value);
	/**
	 * Log a timestamped line with several values
	 * @param values Values to log
	 */
	void logData(vector<Cell> values);
	
	string getCurPath() { return curPath; }
	
private:
	string recordingDir;
	string logFilenameFormat;
	string timestampFormat;
	vector<string> headers;
	string curPath = "";
	
	ofstream logFile;
	
	/**
	 * Open a file based on the timestamp
	 */
	void openFile();
	void closeFile();
};

#endif // __CSVLOGGER_HPP__

