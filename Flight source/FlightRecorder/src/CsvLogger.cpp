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
#include <CsvLogger.hpp>

using namespace std;
using namespace std::chrono;

/**
 * Close old log file, if one was open.
 * Open a new log file for subsequent calls to logData().
 */
void CsvLogger::startNewLogFile() {
	openFile();
}

/**
 * Call this before logging data.
 * @param header Single-token header
 * @return the column ID
 */
unsigned int CsvLogger::addColumn(string header) {
	headers.push_back(header);
	// Column ID just happens to be the index
	return headers.size() - 1;
}
/**
 * Log one timestamped value on its own line.
 * @param value Value to log
 */
void CsvLogger::logData(Cell value) {
	logData(vector<Cell>({value}));
}
/**
 * Log a timestamped line with several values
 * @param values Values to log
 */
void CsvLogger::logData(vector<Cell> values) {
	if(logFile.is_open()) {
		// Timestamp is always the first column
		logFile << date::format(timestampFormat, date::floor<milliseconds>(system_clock::now()));
		logFile <<',';
		sort(values.begin(), values.end());
		auto it = values.begin();
		for(unsigned int col = 1; col < headers.size(); col++) {
			if(it != values.end()) {
				if(it->id == col) {
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
void CsvLogger::openFile() {
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
void CsvLogger::closeFile() {
	logFile.close();
	curPath = "";
}


int main() {
	CsvLogger myLogger;
	auto col1 = myLogger.addColumn("foo");
	auto col2 = myLogger.addColumn("bar");
	myLogger.startNewLogFile();
	
	myLogger.logData(CsvLogger::Cell(col1, 1));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col1, 2), CsvLogger::Cell(col2, 3)}));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col2, 5), CsvLogger::Cell(col1, 4)}));
	
	cout << "Saved to " << myLogger.getCurPath() << endl;
	
	return 0;
}
