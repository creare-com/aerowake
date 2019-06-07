/*
	benchmarker.hpp
	
	Class for profiling/benchmarking.
	
	2017-2-28	JDW	Created.
	2019-6-7	JDW	Refactored into a header-only library
*/

#ifndef __BENCHMARKER_HPP__
#define __BENCHMARKER_HPP__


#include <sstream>
#include <map>
#include <sys/time.h>
#include <chrono> // C++11

class Benchmarker {
private:
	std::string name;
	std::chrono::steady_clock::duration totalDuration;
	int numTimesRun;
	int numItemsProcessed;
	std::chrono::steady_clock::time_point startTime;

public:
	Benchmarker(std::string _name = "") :
		name(_name),
		totalDuration(std::chrono::seconds(0)),
		numTimesRun(0),
		numItemsProcessed(0)
	{;}
	virtual ~Benchmarker() {;}

	// An iteration of processing is about to begin
	void start() {
		startTime = std::chrono::steady_clock::now();
	}
	// Processing of this type has paused, but processing for this iteration is not finished.
	void pause(int items_processed = 0)	{
		std::chrono::steady_clock::duration elapsed = (std::chrono::steady_clock::now() - startTime);
		totalDuration += elapsed;
		numItemsProcessed += items_processed;
	}
	// Processing of this type is about to resume.  Equivalent to start().
	void resume(){
		start(); 
	}
	// An iteration of processing has concluded. Optionally, it processed N items.
	// Equivalent to pause() followed by conclude().
	void end(int items_processed = 0) {
		pause(items_processed); // Stops timekeeping
		conclude(); // End of this iteration
	}
	// An iteration of processing has concluded.
	// Use this when paused - it indicates the end of an iteration, without marking
	// the end time.
	void conclude() {
		numTimesRun++; // End of this iteration
	}
	// Get the total time spent in processing, divided by the number of iterations ran.
	std::chrono::steady_clock::duration getAvgTime() const {
		return (numTimesRun > 0) ? (totalDuration / numTimesRun) : std::chrono::milliseconds(0);
	}

	// Get the total time spent in processing, divided by the number of iterations ran.
	double getAvgMs() const {
		return std::chrono::duration_cast<std::chrono::milliseconds>(getAvgTime()).count();
	}

	void setName(std::string name) { this->name = name; }
	std::string getName() const { return name; }
	double getAvgItemsProcessed() const { 
		return numTimesRun > 0 ? numItemsProcessed / numTimesRun : 0; 
	}
	int getIterations() const { return numTimesRun; }
};

#endif // __BENCHMARKER_HPP__
