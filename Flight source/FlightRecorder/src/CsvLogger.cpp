/*
	CsvLogger.hpp
	
	Class for saving data to a CSV log.
	
	Usage:
	
	CsvLogger myLogger;
	auto col1 = myLogger.addColumn("foo");
	auto col2 = myLogger.addColumn("bar");
	myLogger.startNewLogFile();
	
	myLogger.logData(CsvLogger::cell(col1, 1));
	myLogger.logData(vector({CsvLogger::cell(col1, 1), CsvLogger::cell(col1, 1)}));
	
	
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
	// Timestamp is always the first column
	cout << date::format(timestampFormat, date::floor<milliseconds>(system_clock::now()));
	cout <<',';
	sort(values.begin(), values.end());
	auto it = values.begin();
	for(unsigned int col = 1; col < headers.size(); col++) {
		if(it != values.end()) {
			if(it->id == col) {
				cout << it->value;
				++it;
			}
		}
		if(col < headers.size() - 1) {
			cout << ',';
		}
	}
	cout << endl;
}
void CsvLogger::openFile() {
	// TODO: open file
	for(unsigned int col = 0; col < headers.size(); col++) {
		cout << headers[col];
		if(col < headers.size() - 1) {
			cout << ',';
		}
	}
	cout << endl;
}
void CsvLogger::closeFile() {
	
}


int main() {
	CsvLogger myLogger;
	auto col1 = myLogger.addColumn("foo");
	auto col2 = myLogger.addColumn("bar");
	myLogger.startNewLogFile();
	
	myLogger.logData(CsvLogger::Cell(col1, 1));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col1, 2), CsvLogger::Cell(col2, 3)}));
	myLogger.logData(vector<CsvLogger::Cell>({CsvLogger::Cell(col2, 5), CsvLogger::Cell(col1, 4)}));
	
	return 0;
}
