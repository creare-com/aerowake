/*
	benchmarker.h
	
	Class for profiling/benchmarking.
	
	2017-2-28  JDW  Created.
*/

#ifndef __PCG_BENCHMARKER_H__
#define __PCG_BENCHMARKER_H__


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
	void start();  
	// Processing of this type has paused, but processing for this iteration is not finished.
	void pause(int items_processed = 0);  
	// Processing of this type is about to resume.  Equivalent to start().
	void resume(); 
	// An iteration of processing has concluded. Optionally, it processed N items.
	// Equivalent to pause() followed by conclude().
	void end(int items_processed = 0);
	// An iteration of processing has concluded.
	// Use this when paused - it indicates the end of an iteration, without marking
	// the end time.
	void conclude();
	// Get the total time spent in processing, divided by the number of iterations ran.
	std::chrono::steady_clock::duration getAvgTime() const;
	// Get the total time spent in processing, divided by the number of iterations ran.
	double getAvgMs() const;
	void setName(std::string name) { this->name = name; }
	std::string getName() const { return name; }
	double getAvgItemsProcessed() const { 
		return numTimesRun > 0 ? numItemsProcessed / numTimesRun : 0; 
	}
	int getIterations() const { return numTimesRun; }
};

#endif // __PCG_BENCHMARKER_H__
