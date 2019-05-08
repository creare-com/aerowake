/*
	imageProcessing.h
	
	Declarations for PCG image processing
	
	2016-12-23  JDW  Created.
*/

#include <imageProcessing.h>
using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cuda;
void ImageProcessing::init(json options, Logger * lgr, ros::NodeHandle * rosNodeHandle) {
	logger = lgr;
	
	stringstream ss;
	ss << "OpenCV version is: " << CV_MAJOR_VERSION << '.' << CV_MINOR_VERSION;
	logger->logDebug(ss.str());
	
	// Load configuration options
	string curKey = "";
	string calFn = "";
	string algName = "";
	string rosTopic;
	int rosQueueSize;
	try {
		curKey = "rosTopic";      rosTopic     = options[curKey];
		curKey = "rosFrameId";    rosFrameId   = options[curKey];
		curKey = "rosQueueSize";  rosQueueSize = options[curKey];
		curKey = "stereoCalFile"; calFn        = options[curKey];
		curKey = "algorithm";     algName      = options[curKey];
		curKey = "enableGpu";     enableGpu    = options[curKey];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << curKey << "\" in processing section: "
			 << e.what() << endl;
		throw(e);
	}
	
	json stereo_cal;
	try {
		ifstream calFile(calFn);
		calFile >> stereo_cal;
	} catch(exception e) {
		cerr << "Could not parse JSON calibration file at "
			 << calFn << ".  There was an error opening or parsing:"
			 << endl << e.what() << endl << "Exiting.";
		throw(e);
	}
	// Initialize GPU
	int numGpus = 0;
	try {
		numGpus = cuda::getCudaEnabledDeviceCount();
	} catch (exception e) {
		logger->logError("Could not initialize CUDA!");
		logger->logError(e.what());
		enableGpu = false;
	}
	if(numGpus > 0) {
		stringstream ss;
		ss << "Have " << numGpus << " GPUs.";
		logger->logDebug(ss.str());
	} else {
		logger->logError("No GPU found!");
	}
	
	stereo_cal["enableGpu"] = enableGpu;

	// Load calibration
	cal_data.init(stereo_cal);

	// Initialize underlying algorithm
	if(algName == "CpuFastWithBinnedKps") {
		alg = new CpuFastWithBinnedKps(bms);
	// } else if(algName == "GpuFastWithBinnedKps") {
		// if(enableGpu) {
			// alg = new GpuFastWithBinnedKps(bms);
		// } else {
			// logger->logError("Cannot process on GPU without GPU enabled.");
			// alg = NULL; // Fail badly
		// }
	} else {
		// Default to doing nothing
		alg = new DummyAlg(bms);
	}
	alg->init(options, logger, cal_data);
	
	rosPublisher = rosNodeHandle->advertise<sensor_msgs::PointCloud>(rosTopic, rosQueueSize);
}

////////////////////////////////////////////////////////////
// Passthrough functions to underlyling algorithm
////////////////////////////////////////////////////////////

void ImageProcessing::visit(ImageDataSet imgData) {
	if(alg != NULL) {
		// Keep in mind, the call to rosPublisher.publish(msg) is more or less
		// free.  However, we want to save battery, so we don't want to call
		// alg->processImages() unless we have listeners.  So we don't bother
		// doing anything if there aren't listeners.
		if(isAnyoneListening()) {
			sensor_msgs::PointCloud msg;
			bmImgTotal.start();
			alg->processImages(imgData, msg);
			bmImgTotal.end();
		
			msg.header.seq = seqNum++;
			msg.header.frame_id = rosFrameId;
			msg.header.stamp = imgData.acquisitionTime;
			
			rosPublisher.publish(msg);
		}
	}
}

bool ImageProcessing::areCvWindowsOpen() {
	if(alg != NULL) {
		return alg->areCvWindowsOpen();
	} else {
		return false;
	}
}

bool ImageProcessing::isAnyoneListening() {
	return rosPublisher.getNumSubscribers() > 0;
}
