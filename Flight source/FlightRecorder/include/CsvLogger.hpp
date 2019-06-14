/*
	CsvLogger.hpp
	
	Class for saving data to a CSV log.
	
	Usage:
	
	CsvLogger myLogger;
	auto col1 = myLogger.addColumn("foo");
	auto col2 = myLogger.addColumn("bar");
	myLogger.startNewLogFile();
	
	myLogger.logData(CsvLogger::Cell(col1, 1));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col1, 2), CsvLogger::Cell(col2, 3)}));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col2, 5), CsvLogger::Cell(col1, 4)}));
	
	
	
	Contents of data_2019-06-13_14-50-48.210.csv:
	
	time,foo,bar
	14:50:48.210,1,
	14:50:48.210,2,3
	14:50:48.210,4,5
	
	
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
	void startNewLogFile() {
		openFile();
	}
	
	/**
	 * Call this before logging data.
	 * @param header Single-token header
	 * @return the column ID
	 */
	unsigned int addColumn(string header) {
		headers.push_back(header);
		// Column ID just happens to be the index
		return headers.size() - 1;
	}
	
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
	void logData(Cell value) {
		logData(vector<Cell>({value}));
	}
	/**
	 * Log a timestamped line with several values
	 * @param values Values to log
	 */
	void logData(vector<Cell> values) {
		if(logFile.is_open()) {
			// Timestamp is always the first column
			logFile << date::format(timestampFormat, date::floor<milliseconds>(system_clock::now()));
			logFile <<',';
			sort(values.begin(), values.end());
			auto it = values.begin();
			for(unsigned int col = 1; col < headers.size(); col++) {
				if(it != values.end()) {
					if(it->id == col) {
						if(isnan(it->value)) {
							cout << "Warning: NaN for column " << headers[col] << endl;
						}
						logFile << it->value;
						++it;
					}
				}
				if(col < headers.size() - 1) {
					logFile << ',';
				}
			}
			logFile << endl;
		}
	}
	
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
	void openFile() {
		if(logFile.is_open()) {
			closeFile();
		}
		
		stringstream nameStream;
		nameStream << recordingDir << '/' << date::format(logFilenameFormat, date::floor<milliseconds>(system_clock::now()));
		curPath = nameStream.str();
		logFile.open(curPath);
		
		for(unsigned int col = 0; col < headers.size(); col++) {
			logFile << headers[col];
			if(col < headers.size() - 1) {
				logFile << ',';
			}
		}
		logFile << endl;
	}
	void closeFile() {
		logFile.close();
		curPath = "";
	}

};

#endif // __CSVLOGGER_HPP__

