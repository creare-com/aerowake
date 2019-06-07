/*
 * benchmarker.hpp
 * 
 * Simple, very manually-controlled profiler object.
 * 
 * Usage example:
 * 
 * list<const Benchmarker *> myBms;
 * Benchmarker bmAddition("Addition");
 * Benchmarker bmMultiplication("Multiplication");
 * myBms.push_back(&bmAddition);
 * myBms.push_back(&bmMultiplication);
 * 
 * float accumAdd = 0;
 * float accumMult = 1;
 * 
 * for(int i = 0; i < 1000; i++) {
 *   bmAddition.start();
 *   accumAdd += 5;
 *   bmAddition.end();
 * 
 *   bmMultiplication.start();
 *   accumMult *= 5;
 *   bmMultiplication.end();
 * }
 * 
 * Benchmarker::summarizeBenchmarksToStream(myBms, cout);
 * 
 * 
 * 2017-02-28	JDW	Created.
 * 2019-06-07	JDW	Refactored into a header-only library
*/

#ifndef __BENCHMARKER_HPP__
#define __BENCHMARKER_HPP__


#include <ostream>
#include <sstream>
#include <map>
#include <sys/time.h>
#include <chrono> // C++11
#include <list> // C++11

using namespace std;

class Benchmarker {
private:
	string name;
	chrono::steady_clock::duration totalDuration;
	int numTimesRun;
	int numItemsProcessed;
	chrono::steady_clock::time_point startTime;

public:
	/**
	 * Constructor.
	 * 
	 * @param _name Human-readable name for the operation being measured
	 * 
	 */
	Benchmarker(string _name = "") :
		name(_name),
		totalDuration(chrono::seconds(0)),
		numTimesRun(0),
		numItemsProcessed(0)
	{;}
	virtual ~Benchmarker() {;}

	static void summarizeBenchmarksToStream(list<const Benchmarker *> allBms, ostream & stream) {
		for(auto bm = allBms.begin(); bm != allBms.end(); ++bm) {
			stream << endl << (*bm)->getName() << ": "  << (*bm)->getAvgMs() << "ms avg";
			if((*bm)->getAvgItemsProcessed() > 0) {
				stream << ", " << (*bm)->getAvgItemsProcessed() << " avg items";
			}
			stream << " (" << (*bm)->getIterations() << " iterations).";
		}
		stream << endl;
	}
	// An iteration of processing is about to begin
	void start() {
		startTime = chrono::steady_clock::now();
	}
	// Processing of this type has paused, but processing for this iteration is not finished.
	void pause(int items_processed = 0)	{
		chrono::steady_clock::duration elapsed = (chrono::steady_clock::now() - startTime);
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
	chrono::steady_clock::duration getAvgTime() const {
		return (numTimesRun > 0) ? (totalDuration / numTimesRun) : chrono::milliseconds(0);
	}

	// Get the total time spent in processing, divided by the number of iterations ran.
	double getAvgMs() const {
		return chrono::duration_cast<chrono::milliseconds>(getAvgTime()).count();
	}

	void setName(string name) { this->name = name; }
	string getName() const { return name; }
	double getAvgItemsProcessed() const { 
		return numTimesRun > 0 ? numItemsProcessed / numTimesRun : 0; 
	}
	int getIterations() const { return numTimesRun; }
};

#endif // __BENCHMARKER_HPP__
