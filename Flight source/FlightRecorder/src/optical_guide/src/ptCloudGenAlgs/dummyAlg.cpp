/*
	dummyAlg.cpp
	
	An algorithm that does nothing except delay.  See also: "Congress".
	
	2017-03-21  JDW  Created.
*/

#include <ptCloudGenAlgs/dummyAlg.h>
using namespace std;
using namespace std::chrono;

void DummyAlg::init(json options, Logger * lgr, StereoCal calData) {
	StereoPtCloudGenAlg::init(options, lgr, calData);
	
	// Load configuration options
	string cur_key = "";
	try {
		int delay_ms = 0;
		cur_key = "dummyOptions"; json dummy_section = options[cur_key];
		cur_key = "delayMsPerCycle";        delay_ms = dummy_section[cur_key];
		delayTimePerCycle = chrono::milliseconds(delay_ms);
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in processing section: "
			 << e.what() << endl;
		throw(e);
	}
}

void DummyAlg::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	// Does not actually process any images.
	
	// Call super though, so we can see the images
	StereoPtCloudGenAlg::processImages(imgData, msg);

	stringstream ss;
	ss << "Simulating " << chrono::duration_cast<chrono::milliseconds>(delayTimePerCycle).count()
	<< "ms of processing.";
	logger->logInfo(ss.str());

	this_thread::sleep_for(delayTimePerCycle);
}
