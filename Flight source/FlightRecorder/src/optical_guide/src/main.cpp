/*

Main entry point for Optical Guide's (6298) Point Cloud Generator (PCG) application.
Originally developed on Rough Bottom (8136) by MPU/DRC/JCW, modified for use here.

2016-12-21  JDW  Created from Rough Bottom code

*/
#include <pcg.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

using json = nlohmann::json;
using namespace std;

// Consider an option to override this with an argument
const char * PcgMain::DEFAULT_CONFIG_FILENAME = "../../config/pcgConfig.json";

void PcgMain::init(char * config_fn) {
	// Open and parse file.
	// Throw an exception on failure.
	json options;
	{
		if(config_fn == NULL)
		{ config_fn = (char*)DEFAULT_CONFIG_FILENAME; }
		ifstream config_file(config_fn);
		config_file >> options;
	}

	// Parse main-level options and pick out sections
	string cur_key = "";
	json logging_config;
	json acquisition_config;
	json processing_config;
	json attitude_config;
	json lidar_config;
	json output_rec_config;
	bool enable_gpu = true;
	try {
		cur_key = "useLidar";                   useLidar           = options[cur_key];
		cur_key = "useStereoVision";            useStereoVision    = options[cur_key];
		cur_key = "recDataPath";                recDataPath        = options[cur_key];
		cur_key = "enableGpu";                  enable_gpu         = options[cur_key];

		cur_key = "logging";          logging_config     = options[cur_key];
		cur_key = "imageAcquisition"; acquisition_config = options[cur_key];
		cur_key = "imageProcessing";  processing_config  = options[cur_key];
		cur_key = "attitudeTracker";  attitude_config    = options[cur_key];
		cur_key = "lidar";            lidar_config       = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\": " << e.what() << endl;
		throw e;
	}
	
	// Create a directory for this recording, and a subdir for output
	stringstream path;
	time_t now_s = chrono::system_clock::to_time_t(chrono::system_clock::now()); 
	path << put_time(localtime(&now_s), recDataPath.c_str());
	recDataPath = path.str();
	if(mkdir(recDataPath.c_str(), 0777) < 0) {
		cerr << "Couldn't create recording dir.  Error #" << errno << endl;
	}

	// Submodules will handle their own subdir creation
	logging_config    ["path"] = recDataPath;
	acquisition_config["path"] = recDataPath;
	attitude_config   ["path"] = recDataPath;
	lidar_config      ["path"] = recDataPath;
	
	// Use of the GPU should be disable-able in all submodules that use it
	processing_config["enableGpu"] = enable_gpu;
	
	// Pass subsections to component modules
	try { logger.init(logging_config); } catch(exception e) { cerr << "Could not initialize logger." << endl; }
	if(useStereoVision) {
		try { img_acquisition .init(acquisition_config, &logger); } 
		catch(exception e) { cerr << "Could not initialize image acquisition." << endl; }
		try { img_processing  .init(processing_config,  &logger, &rosNodeHandle); } 
		catch(exception e) { cerr << "Could not initialize image processing ." << endl; }
	}
	try { attitude_tracker.init(attitude_config,    &logger); } catch(exception e) { cerr << "Could not initialize attitude tracker ." << endl; }
	try { lidar           .init(lidar_config,       &logger, &rosNodeHandle); } catch(exception e) { cerr << "Could not initialize LIDAR reader." << endl; }
	
	
	// Copy the config file into the recording directory so we know what was used.
	{
		string save_path = recDataPath + "/active_config.json";
		ifstream config_file(config_fn, ios::binary);
		ofstream saved_config(save_path, ios::binary);
		saved_config << config_file.rdbuf();
	}
}

int PcgMain::main(char * config_fn)
{
	// Load configuration file and initialize
	try {
		init(config_fn);
	} catch(exception e) {
		cerr << "The PCG application requires a valid JSON file at "
			 << config_fn << ".  There was an error opening or parsing:"
			 << endl << e.what() << endl << "Exiting.";
		return 1;
	}
	
	// From this point forward, the logger has been initialized.
	// Thus, the use of cout and cerr are discouraged.  Please use the logging
	// functions instead.
	logger.logInfo("Initialized.");
	
	// Frame rate limit, in Hz
	ros::Rate loopRate(100);
	
	uint16_t pc_seq = 0;
	if(useStereoVision) {
		img_acquisition.start(); // Open connection to cameras
		// Start getting the first frame capturing in the background
		// (will report an error if no cameras present)
		img_acquisition.beginAcquisition();
	}
	while(ros::ok()) {
		bmOneFrame.start();
		
		// Store attitude
		attitude_tracker.estimateAttitude();
		
		if(useLidar) {
			lidar.visit();
		}

		if(useStereoVision && img_processing.isAnyoneListening()) {
			ImageDataSet image_data;
			// Acquire images, either from recording or reality
			image_data = img_acquisition.acquireImages();
			if(img_acquisition.isPlaybackEnabled()) {
				if(img_acquisition.isPlaybackComplete()) {
					logger.logInfo("Reached end of playback.");
					useStereoVision = false;
				}
			} else {
				// Get the next image capture started
				img_acquisition.beginAcquisition();
			}
			
			// Process images and deliver point clouds
			img_processing.visit(image_data);
		} 
		
		// summarizeBenchmarksToLog();
		
		// Sometimes we wish to sleep after processing, due to a max frame rate
		// or due to a preferred playback rate.
		ros::Duration sleep_time(0);
		if(img_acquisition.isPlaybackEnabled()) {
			sleep_time = img_acquisition.getFrameDelay();
			bmOneFrame.pause(); // Don't count this time in benchmarking one frame of processing
			// logger.logDebug("Sleep time: " + to_string(sleep_time.sec) + "s, " + to_string(sleep_time.nsec) + "ns");
			if(img_processing.areCvWindowsOpen()) {
				// We need a GUI-friendly form of sleep for the debug windows to update
				int ms_to_sleep = sleep_time.toNSec() / 1000;
				if(ms_to_sleep <= 0) { ms_to_sleep = 1; };
				cv::waitKey(ms_to_sleep);
			} else {
				// ROS sleep
				sleep_time.sleep();
			}
			bmOneFrame.resume();
		}


		// OS call to flush all file buffers
		bmSyncFs.start();
		sync();
		bmSyncFs.end();
		
		// If we didn't have anything to do this frame,
		// wait for a tiny bit.
		loopRate.sleep();
		
		bmOneFrame.end();
	}
	logger.logInfo("Got ROS exit signal; quitting.");
	img_acquisition.stop();
	return 0;
}

// Iterate through allBms and write a summary of benchmarkers
// to the logger object
void PcgMain::summarizeBenchmarksToLog() {
	stringstream ss;
	for(auto bm = allBms.begin(); bm != allBms.end(); ++bm) {
		ss << endl << (*bm)->getName() << ": "  << (*bm)->getAvgMs() << "ms avg";
		if((*bm)->getAvgItemsProcessed() > 0) {
			ss << ", " << (*bm)->getAvgItemsProcessed() << " avg items";
		}
		ss << " (" << (*bm)->getIterations() << " iterations).";
	}
	logger.logDebug(ss.str());
}


// Entry point
int main(int argc, char ** argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "pcg");

	PcgMain pcg;
	
	// We currently support exactly one argument, which is optional:
	// the path to the configuration file.
	char * config_fn = NULL;
	if(argc > 1) {
		config_fn = argv[1];
	}
	return pcg.main(config_fn);
}

