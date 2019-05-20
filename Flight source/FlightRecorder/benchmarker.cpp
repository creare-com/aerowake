/*
	benchmarker.cpp
	
	Class for storing stereo calibration data
	
	2016-12-23  JDW  Created.
	2017-03-02  JDW  Moved into its own file.
*/
#include <benchmarker.h>
using namespace std;
using namespace chrono;

void Benchmarker::start() {
	startTime = steady_clock::now();
}

void Benchmarker::pause(int items_processed) {
	steady_clock::duration elapsed = (steady_clock::now() - startTime);
	totalDuration += elapsed;
	numItemsProcessed += items_processed;
}
void Benchmarker::resume() {
	// Is actually the same as start(), but this should be a separate method to
	// preserve encapsulation.
	start(); 
}

void Benchmarker::end(int items_processed) {
	pause(items_processed); // Stops timekeeping
	conclude(); // End of this iteration
}

void Benchmarker::conclude() {
	numTimesRun++; // End of this iteration
}

steady_clock::duration Benchmarker::getAvgTime() const {
	return (numTimesRun > 0) ? (totalDuration / numTimesRun) : milliseconds(0);
}

double Benchmarker::getAvgMs() const {
	return duration_cast<milliseconds>(getAvgTime()).count();
}
