/*
	stereoPtCloudGenAlg.cpp
	
	Abstract base class containing logic common to all point cloud generation
	algorithms.
	
	2017-3-2  JDW  Created.
*/

#include <imageProcessing.h>
#include <ptCloudGenAlgs/stereoPtCloudGenAlg.h>
using namespace std;
using namespace std::chrono;
using namespace cv;

void StereoPtCloudGenAlg::init(json options, Logger * lgr, StereoCal calData) {
	logger = lgr;
	cal_data = calData;
	
	// Load configuration options
	string cur_key = "";
	try {
		cur_key = "showImages";          showImages          = options[cur_key];
		cur_key = "stereoDistThreshold"; stereoDistThreshold = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in processing section: "
			 << e.what() << endl;
		throw(e);
	}
}

// The default processImages function merely displays images to screen
void StereoPtCloudGenAlg::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	bmShowingImages.start();
	const char * L_WINDOW_NAME  = "Left camera image";
	const char * R_WINDOW_NAME  = "Right camera image";
	const char * IR_WINDOW_NAME = "Infrared camera image";
	const int WINDOW_WIDTH = 1000;
	// Base scaling factor off one image only, to match scale
	const double SCALE_FACTOR = (double)WINDOW_WIDTH / imgData.imgVisibleL.cols;
	// const double SCALE_FACTOR = 1;
	// bool showImgsUnrectified = !(imgData.imgVisibleLValid && imgData.imgVisibleRValid
		// && (imgData.imgVisibleL.size() == imgData.imgVisibleR.size()));
	bool showImgsUnrectified = true;
	
	cvWindowsAreOpen = false;
	if(imgData.imgVisibleLValid) {
		if (imgData.imgVisibleL.empty()){
			logger->logError("Left image is empty!");
		} else if(showImages) {
			cv::namedWindow(L_WINDOW_NAME, CV_WINDOW_NORMAL);
			if(showImgsUnrectified) {
				cv::imshow(L_WINDOW_NAME, imgData.imgVisibleL);
				cv::resizeWindow(L_WINDOW_NAME, SCALE_FACTOR * imgData.imgVisibleL.cols,
					SCALE_FACTOR * imgData.imgVisibleL.rows);
			} else {
				logger->logWarning("Choosing not to populate window");
			}
			
			// Won't appear until waitKey() call, but we leave that for the main loop.
			cvWindowsAreOpen = true;
		}
	}
	
	if(imgData.imgVisibleRValid) {
		if (imgData.imgVisibleR.empty()){
			logger->logError("Right image is empty!");
		} else if(showImages) {
			cv::namedWindow(R_WINDOW_NAME, CV_WINDOW_NORMAL);
			if(showImgsUnrectified) {
				cv::imshow(R_WINDOW_NAME, imgData.imgVisibleR);
				cv::resizeWindow(R_WINDOW_NAME, SCALE_FACTOR * imgData.imgVisibleR.cols,
					SCALE_FACTOR * imgData.imgVisibleR.rows);
			}
			// Won't appear until waitKey() call, but we leave that for the main loop.
			cvWindowsAreOpen = true;
		}
	}
	
	if(imgData.imgInfraredValid) {
		if (imgData.imgInfrared.empty()){
			logger->logError("Infrared image is empty!");
		} else if(showImages) {
			cv::namedWindow(IR_WINDOW_NAME, CV_WINDOW_NORMAL);
			cv::imshow(IR_WINDOW_NAME, imgData.imgInfrared);
			cv::resizeWindow(IR_WINDOW_NAME, SCALE_FACTOR * imgData.imgInfrared.cols,
				SCALE_FACTOR * imgData.imgInfrared.rows);
			// Won't appear until waitKey() call, but we leave that for the main loop.
			cvWindowsAreOpen = true;
		}
	}
	bmShowingImages.end();
}
