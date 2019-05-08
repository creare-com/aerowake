/*
	imageAcquisition.h
	
	Declarations for image acquisition
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_IMAQ_H__
#define __PCG_IMAQ_H__

#include <FlyCapture2.h>
#include <stdio.h>
#include <iostream>
#include <list>
#include <utility>
#include <thread>
#include <ctime>
#include <sys/stat.h>
#include <semaphore.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS libraries
#include "ros/ros.h"

#include "logger.h"
#include "benchmarker.h"

// Used like a struct, holds one frame of input data from camera-like sensors.
class ImageDataSet {
public:
	ros::Time acquisitionTime;
	cv::Mat imgVisibleL, imgVisibleR;
	bool imgVisibleLValid = false, imgVisibleRValid = false; // true for each succesfully acquired image
	cv::Mat imgInfrared;
	bool imgInfraredValid = false;

	unsigned long getTimeAcquiredS();
	unsigned long getTimeAcquiredMs(); // milliseconds within the second
	
	unsigned long getDurationSinceAcquiredMs(); // milliseconds between acquisition and now
};

struct CameraCallbackItems {
	cv::Mat image;   // Set by the callback
	sem_t * pSem;    // Used by the callback
	Logger * logger; // Used by the callback
	char side[6];    // "left " or "right".  Used by callback.
};

class ImageAcquisition {
private:
	static const string REC_EXT;
	FlyCapture2::BusManager mgr;
	FlyCapture2::Camera camL, camR;
	FlyCapture2::CameraInfo camInfoL, camInfoR;
	bool running;
	bool camLConnected, camRConnected;
	list<const Benchmarker *> * bms;
	Benchmarker bmCamControl;
	Benchmarker bmFlipImageCpu;
	Benchmarker bmSaveImages  ;
	Benchmarker bmLoadImages  ;
	Logger * logger;
	
	// True for success
	bool initPtGreyCam (unsigned int sn, list<pair<string, double>> cam_props, 
		FlyCapture2::Camera &cam, FlyCapture2::CameraInfo &camInfo);
	void startPtGreyCam(FlyCapture2::Camera &cam, CameraCallbackItems* cbItems);
	void stopPtGreyCam (FlyCapture2::Camera &cam);
	static cv::Mat cvMatFromFlyCap2Image(FlyCapture2::Image *img);
	void saveImages(ImageDataSet data);
	FlyCapture2::PropertyType propTypeFromString(string p);
	string propTypeToString(FlyCapture2::PropertyType  p);
	
	// Point Grey camera serial numbers
	unsigned int leftVisibleCamSn;
	unsigned int rightVisibleCamSn;
	
	// used for automatic control of camera properties
	bool autoGainControlEnabled = false;
	list<FlyCapture2::PropertyType> autoControlledProps;

	// Record and playback features
	bool playedFirstFrame = false;
	bool recordEnabled = false;
	bool playbackEnabled = false;
	string folderName;
	string timestampPattern;
	string filenamePattern;
	ofstream recIndexFile;
	ifstream playIndexFile;
	std::tm playBackDate; // Recordings presently only record time of day.  So playback needs a date.
	ros::Time lastFrameTime;
	bool playbackComplete = false;
	ros::Duration interframeDelay;
	bool flipImgL, flipImgR; // To flip or not
	int imgFlipCodeL, imgFlipCodeR; // Argument to cv::flip: 0 flips around x axis, 1 around y, -1 around both
	string autoGainControlValues = "";
	ros::Time lastAcquisitionTime;
	
	// Used in image acquisition callback
	sem_t imageWaitL, imageWaitR;
	struct timespec imageWaitTimeout = {1, 0};
	CameraCallbackItems cbItemsL = {cv::Mat(), NULL, NULL, "left "};
	CameraCallbackItems cbItemsR = {cv::Mat(), NULL, NULL, "right"};
	
	// Helper functions
	ros::Time readTimeFromTabSepInput(istream &input);
	// void captureImagesFromReality(ImageDataSet & data);
	void openImagesFromRecording(ImageDataSet & data);

public:
	ImageAcquisition(list<const Benchmarker *> * _bms) :
		mgr(),
		bms(_bms),
		bmCamControl  ("API interactions with cameras"),
		bmFlipImageCpu("Flipping images"),
		bmSaveImages  ("Saving images"),
		bmLoadImages  ("Loading images"),
		recIndexFile()
	{
		bms->push_back(&bmCamControl  );
		bms->push_back(&bmFlipImageCpu);
		bms->push_back(&bmSaveImages  );
		bms->push_back(&bmLoadImages  );
	}

	void init(nlohmann::json options, Logger * lgr);
	
	// Control operation
	void start();
	void stop();
	inline bool isRunning() { return running; }
	
	// Query state
	bool isPlaybackEnabled() { return playbackEnabled; }
	bool isPlaybackComplete() { return playbackComplete; }
	ros::Duration getFrameDelay() { return interframeDelay; }
	
	ImageDataSet acquireImages();

	static void flyCapImgEvent(FlyCapture2::Image * pImage, const void * pCallbackData);

	virtual void beginAcquisition();
};


#endif // __PCG_IMAQ_H__
