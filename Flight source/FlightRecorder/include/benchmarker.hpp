/*
 * benchmarker.hpp
 * 
 * Simple, very manually-controlled profiler object.
 * 
 * 2017-02-28	JDW	Created.
 * 2019-06-07	JDW	Refactored into a header-only library
 * 
 * 
 * 
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
 * 
 * 
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

	/**
	 * Summarize a list of benchmarkers to a stream such as std::cout or a log file.
	 * Entries look like: <name>: <average time of one iteration>[, <average items processed>] (<iteration count>)
	 * 
	 * @param allBms A list of benchmarker objects
	 * @param stream The output stream, such as std::cout or a log file.
	 * 
	 */
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
	
	/**
	 * Call this to begin timing one iteration of processing.
	 * The next call should be to pause() or stop().
	 */
	void start() {
		startTime = chrono::steady_clock::now();
	}

	/**
	 * Call this to pause timing one iteration of processing.
	 * Call this only after calling start().  The next call should be to resume().
	 * 
	 * @param items_processed (optional) The number of items that have been processed since the call to start()
	 */
	void pause(int items_processed = 0)	{
		chrono::steady_clock::duration elapsed = (chrono::steady_clock::now() - startTime);
		totalDuration += elapsed;
		numItemsProcessed += items_processed;
	}
	
	/**
	 * Call this to resume timing of processing.
	 * Call this only after calling pause().
	 * The next call should be to pause() or stop().
	 */
	void resume() {
		start(); 
	}
	
	/**
	 * Call this to conclude timing one iteration of processing.
	 * Call this only after calling start() or resume().
	 * 
	 * @param items_processed (optional) The number of items that have been processed since the call to start() or the last call to resume()
	 */
	void end(int items_processed = 0) {
		pause(items_processed); // Stops timekeeping
		numTimesRun++; // End of this iteration
	}

	/**
	 * Get the average duration of a single iteration.
	 * 
	 * @return the total time spent in processing, divided by the number of iterations ran.
	 */
	chrono::steady_clock::duration getAvgTime() const {
		return (numTimesRun > 0) ? (totalDuration / numTimesRun) : chrono::milliseconds(0);
	}

	/**
	 * Get the average duration of a single iteration, in milliseconds.
	 * 
	 * @return the total time spent in processing, in ms, divided by the number of iterations ran.
	 */
	double getAvgMs() const {
		return chrono::duration_cast<chrono::milliseconds>(getAvgTime()).count();
	}
	
	/**
	 * Set the human-readable name of the benchmarker.
	 * 
	 * @param name the human-readable name of the benchmarker.
	 */
	void setName(string name) { this->name = name; }

	/**
	 * Get the human-readable name of the benchmarker.
	 * 
	 * @return the human-readable name of the benchmarker.
	 */
	string getName() const { return name; }
	
	/**
	 * Get the average number of items processed in one iteration.
	 * This will be zero, unless you have supplied the optional items_processed argument to end() and/or pause().
	 * 
	 * @return the average number of items processed in one iteration
	 */
	double getAvgItemsProcessed() const { 
		return numTimesRun > 0 ? numItemsProcessed / numTimesRun : 0; 
	}
	
	/**
	 * Get the number of iterations, defined as a pair of calls to start() and end().
	 * 
	 * @return the number of iterations
	 */
	int getIterations() const { return numTimesRun; }
};

#endif // __BENCHMARKER_HPP__
