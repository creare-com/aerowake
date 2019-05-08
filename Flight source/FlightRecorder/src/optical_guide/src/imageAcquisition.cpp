/*
pcgImageAcquisition.cpp
 
This class handles acquiring images from the various cameras in the OG system.

2016-12-23  JDW  Created from Rough Bottom (8136) code for Optical Guide (6298)

*/

#include <imageAcquisition.h>
using namespace FlyCapture2;
using namespace std;
using namespace chrono;

unsigned long ImageDataSet::getTimeAcquiredS() {
	return acquisitionTime.sec;
}

unsigned long ImageDataSet::getTimeAcquiredMs() {
	return acquisitionTime.nsec / 1000;
}


static void discardImage( Image* pImage, const void* pCallbackData ) {
	;
}

const string ImageAcquisition::REC_EXT = "tif";

ros::Time ImageAcquisition::readTimeFromTabSepInput(istream &input) {
	// Assumes the next two tokens in the input stream are:
	// 2017-01-11_192901	489
	// Where the first token is in the timestampPattern format and the second is milliseconds.
	int millis;
	std::tm time_s;
	input >> std::get_time(&time_s, timestampPattern.c_str());
	input >> millis;
	time_s.tm_year = playBackDate.tm_year;
	time_s.tm_mon  = playBackDate.tm_mon;
	time_s.tm_mday = playBackDate.tm_mday;
	time_s.tm_isdst = playBackDate.tm_isdst;
	time_t seconds_since_epoch = std::mktime(&time_s);
	ros::Time timestamp = {(uint32_t)seconds_since_epoch, (uint32_t)(millis * 1000)};
	
	return timestamp;
}

bool ImageAcquisition::initPtGreyCam (unsigned int sn, list<pair<string, double>> cam_props, 
		Camera &cam, CameraInfo &camInfo) {
	Error error;
	// Find camera
	PGRGuid camGuid;
	error = mgr.GetCameraFromSerialNumber(sn, &camGuid);
	if (error != PGRERROR_OK)
	{
		stringstream ss;
		ss << "Failed to find camera for SN " << sn;
		logger->logError(ss.str());
		return false;
	}
	
	error = cam.Connect(&camGuid);
	if (error != PGRERROR_OK)
	{
		stringstream ss;
		ss << "Failed to connect to camera #" << sn;
		logger->logError(ss.str());
		return false;
	}
	
	// Get the camera info and print it out
	error = cam.GetCameraInfo(&camInfo);
	if ( error != PGRERROR_OK )
	{
		stringstream ss;
		ss << "Failed to find camera info for SN " << sn;
		logger->logWarning(ss.str());
	}
	
	// Send properties to the camera
	for(auto const &it : cam_props) {
		Property prop;
		prop.type = propTypeFromString(it.first);

		//// For testing
		// error = cam.GetProperty(&prop);
		// if ( error == PGRERROR_OK )
		// {
			// stringstream ss;
			// ss << "Old value of " << it.first << ":"
				// << "\nprop.type           = " << prop.type          
				// << "\nprop.present        = " << prop.present       
				// << "\nprop.absControl     = " << prop.absControl    
				// << "\nprop.onePush        = " << prop.onePush       
				// << "\nprop.onOff          = " << prop.onOff         
				// << "\nprop.autoManualMode = " << prop.autoManualMode
				// << "\nprop.valueA         = " << prop.valueA        
				// << "\nprop.valueB         = " << prop.valueB        
				// << "\nprop.absValue       = " << prop.absValue      
			// << endl;
			// logger->logInfo(ss.str());
		// }

		prop.absValue = it.second; // Not absolute in the mathematical sense.
		prop.absControl = true; // Use absValue, not valueA (valueA is in "ticks", absValue is in a human-readable unit)
		prop.onOff = true; // Use this property
		prop.autoManualMode = false; // Manual control for now; may be set to auto later.
		
		error = cam.SetProperty(&prop);
		stringstream ss;
		if ( error != PGRERROR_OK ) {
			ss << "Failed to set camera " << it.first << " to " << it.second;
			logger->logError(ss.str());
		} else {
			ss << "Succesfully set camera " << it.first << " to " << it.second;
			logger->logInfo(ss.str());
		}
	}
	
	// Configure camera trigger mode so that it only captures when commanded.
	// Currently using Overlapped Exposure Readout Trigger.
	// (Mode 14, section 7.1.5, page 42 of the Chameleon Technical Reference Manual)
	TriggerMode camTrigMode;
	camTrigMode.onOff = true;
	camTrigMode.source = 0;// No pins, software only
	camTrigMode.mode = 14;
	error = cam.SetTriggerMode(&camTrigMode);
	if ( error != PGRERROR_OK )
	{
		stringstream ss;
		ss << "Failed to set trigger mode for SN " << sn;
		logger->logWarning(ss.str());
	}
	
	return true;
}
void ImageAcquisition::startPtGreyCam(Camera &cam, CameraCallbackItems* cbItems) {
	Error error; // TODO: error handling
	error = cam.StartCapture(&discardImage);
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{ logger->logError("Bandwidth exceeded; can't start image capture."); }
	else if ( error != PGRERROR_OK )
	{ logger->logError("Failed to start image capture"); }
	else if ( error == PGRERROR_OK )
	{
		// For some reason, the first trigger often fails.
		// As a workaround, trigger once here.
		error = cam.FireSoftwareTrigger();
		if(error != PGRERROR_OK)
		{ logger->logWarning("Failed to trigger camera while starting."); }
		// Need to delay a bit for it to work.  Ideally we'd be doing some
		// sort of threading, but the core idea is to tolerate one or more
		// cameras silently failing to acquire.
		this_thread::sleep_for(milliseconds(1000));
		// cam.SetCallback(NULL) doesn't work so we just
		// restart the capture without a callback.
		error = cam.StopCapture();
		if(error != PGRERROR_OK)
		{ logger->logWarning("Failed to restart	camera."); }
		error = cam.StartCapture(flyCapImgEvent, cbItems);
		if(error != PGRERROR_OK)
		{ logger->logWarning("Failed to restart	camera."); }
	}
}
void ImageAcquisition::stopPtGreyCam (Camera &cam) {
	Error error; // TODO: error handling
	error = cam.StopCapture();
}

void ImageAcquisition::init(json options, Logger * lgr) {
	Error error; // TODO: error handling
	logger = lgr;
	
	// Load configuration options
	string index_fn;
	string data_path = "~";
	string rec_dir = "~";
	string playback_path = "~";
	string cur_key = "";
	int pb_year = 1970;
	int pb_month = 1;
	int pb_day = 1;
	list<pair<string, double>> cam_props;
	try {
		cur_key = "leftCamSn";                  leftVisibleCamSn       = options[cur_key];
		cur_key = "rightCamSn";                 rightVisibleCamSn      = options[cur_key];
		cur_key = "imageRecordPlayback"; json   rec_play_section       = options[cur_key];
		cur_key = "path";                       data_path              = options[cur_key];
		cur_key = "mode";                string mode_string            = rec_play_section[cur_key];
		recordEnabled   = (mode_string == "record");
		playbackEnabled = (mode_string == "playback");
		cur_key = "indexFile";                  index_fn               = rec_play_section[cur_key];
		cur_key = "recDir";                     rec_dir                = rec_play_section[cur_key];
		cur_key = "playbackPath";               playback_path          = rec_play_section[cur_key];
		cur_key = "timestampPattern";           timestampPattern       = rec_play_section[cur_key];
		cur_key = "playbackDate"; json          playback_date_section  = rec_play_section[cur_key];
		cur_key = "year";                       pb_year                = playback_date_section[cur_key];
		cur_key = "month";                      pb_month               = playback_date_section[cur_key];
		cur_key = "day";                        pb_day                 = playback_date_section[cur_key];
		cur_key = "visibleLightCamProps"; json  cam_props_section      = options[cur_key];
		cur_key = "flipImgL";                   flipImgL               = options[cur_key];
		cur_key = "flipImgR";                   flipImgR               = options[cur_key];
		cur_key = "imgFlipCodeL";               imgFlipCodeL           = options[cur_key];
		cur_key = "imgFlipCodeR";               imgFlipCodeR           = options[cur_key];
		cur_key = "visibleLightCamProps";
		for(auto const & cam_prop: cam_props_section) {
			// Properties are listed as "name", value, "unit".  
			// Units are there for the benefit of the configuration file editor.
			pair<string, double> prop;
			prop.first  = cam_prop[0];
			prop.second = cam_prop[1];
			cam_props.push_back(prop);
			
			// Some properties may be set to have the camera control them automatically
			string auto_or_man = cam_prop[3];
			if(auto_or_man.compare("auto") == 0) {
				autoControlledProps.push_back(propTypeFromString(cam_prop[0]));
				autoGainControlEnabled = true;
			}
		}
		
		// Assemble playback start day
		memset(&playBackDate, 0, sizeof(playBackDate));
		playBackDate.tm_year = pb_year - 1900; // Since 1900
		playBackDate.tm_mon  = pb_month - 1;   // Zero-indexed
		playBackDate.tm_mday = pb_day;         // One-indexed
		playBackDate.tm_isdst = 0;
		std::mktime(&playBackDate); // normalize
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in acquisition section: "
			 << e.what() << endl;
		throw(e);
		return;
	}
	
	if(recordEnabled) {
		folderName = data_path + rec_dir;
		string index_filename = data_path + index_fn;
		logger->logDebug("Will record images to this folder: " + folderName);
		logger->logDebug("Will record index to this file: " + index_filename);
		
		if(mkdir(folderName.c_str(), 0777) < 0) {
			stringstream ss;
			ss	<< "Couldn't create image recording dir.  Error #" << errno;
			logger->logError(ss.str());
		}
		recIndexFile.open(index_filename.c_str(), ios::out | ios::app);
		if(!recIndexFile.is_open())
		{ logger->logError("Couldn't open recording index file " + index_filename); }
	} else if (playbackEnabled) {
		folderName = playback_path + rec_dir;
		string index_filename = playback_path + index_fn;
		logger->logDebug("Will play images from this folder: " + folderName);
		logger->logDebug("Will play this index file: " + index_filename);
		
		playIndexFile.open(index_filename.c_str());
		if(!playIndexFile.is_open())
		{ logger->logError("Couldn't open recording index file " + index_filename); }

		// Note the start time, from which we'll compute deltas.
		lastFrameTime = readTimeFromTabSepInput(playIndexFile);
		time_t tt = lastFrameTime.sec;
		stringstream ss;
		ss << "Selected recording begins at ";
		ss << put_time(localtime(&tt), timestampPattern.c_str());
		logger->logDebug(ss.str());
		playIndexFile.seekg(0);//rewind
		playbackComplete = false;
		playedFirstFrame = false;
	}

	unsigned int numCams = 0;
	error = mgr.GetNumOfCameras(&numCams);
	if (error != PGRERROR_OK)
	{
		logger->logError("Failed to count cameras");
	}
	stringstream ss;
	ss << "See " << numCams << " cameras.";
	logger->logDebug(ss.str());
	
	bool success = initPtGreyCam(leftVisibleCamSn , cam_props, camL, camInfoL);
	ss.str(""); ss.clear();
	ss  << "Left  camera: "
		<< camInfoL.vendorName << ", "
		<< camInfoL.modelName  << ", " 
		<< camInfoL.serialNumber;
	if(success) {
		logger->logInfo(ss.str());
	} else {
		logger->logWarning("Couldn't connect to Left camera.  Won't get images from it.");
	}
	camLConnected = success;

	success = initPtGreyCam(rightVisibleCamSn, cam_props, camR, camInfoR);
	ss.str(""); ss.clear();
	ss  << "Right camera: "
		<< camInfoR.vendorName << ", "
		<< camInfoR.modelName  << ", " 
		<< camInfoR.serialNumber;
	if(success) {
		logger->logInfo(ss.str());
	} else {
		logger->logWarning("Couldn't connect to Right camera.  Won't get images from it.");
	}
	camRConnected = success;
	
	
	// With automatic gain control, we set the left camera to automatically
	// determine its own gain.  Then we copy that value to the right camera.
	if(autoGainControlEnabled && camLConnected) {
		for(auto const &pt : autoControlledProps) {
			Property prop;
			prop.type = pt;
			prop.onOff = true; // Use this property
			prop.autoManualMode = true; // Automatic control
			error = camL.SetProperty(&prop);
			if ( error != PGRERROR_OK ) {
				logger->logError("Failed to enable auto gain control on left camera.");
				// Don't try to copy over parameters later
				autoGainControlEnabled = false;
			} else {
				// All good
			}
		}
	} else {
		// Don't try to copy over parameters later
		autoGainControlEnabled = false;
	}
	
	// Initialize semaphores
	sem_init(&imageWaitL, 0, 0);
	sem_init(&imageWaitR, 0, 0);
	cbItemsL.pSem = &imageWaitL;
	cbItemsR.pSem = &imageWaitR;
	cbItemsL.logger = logger;
	cbItemsR.logger = logger;
}

void ImageAcquisition::start() {
	if(playbackEnabled) { 
		playIndexFile.clear();
		playIndexFile.seekg(0);
		playbackComplete = false;
		playedFirstFrame = false;
	} else {
		if(camLConnected) { startPtGreyCam(camL, &cbItemsL); }
		if(camRConnected) { startPtGreyCam(camR, &cbItemsR); }
		// TODO: start IR cam
		// TODO: error handling
	}
	running = true;
}

void ImageAcquisition::stop() {
	if(camLConnected) { stopPtGreyCam(camL); }
	if(camRConnected) { stopPtGreyCam(camR); }
	// TODO: stop IR cam
	// TODO: error handling
	running = false;
}

// void ImageAcquisition::captureImagesFromReality(ImageDataSet & data) {
	// Refactor into this method some time when we have the system available for testing
// }

void ImageAcquisition::openImagesFromRecording(ImageDataSet & data) {
	bmLoadImages.start();
	const int NUM_TOKENS = 8;  // Ignore tokens past a certain point
	const string DELIMITER = "\t";
	string valid;
	string timestamp="000000", millis="0";
	string left_img_path = "", right_img_path = "";
	string infrared_path = "";
	string line;
	if(!getline(playIndexFile, line)) {
		data.imgVisibleLValid = false;
		data.imgVisibleRValid = false;
		data.imgInfraredValid = false;
		logger->logWarning("Ran out of playback file!");
		playbackComplete = true;
		return;
	}
	size_t token_start = 0, token_end = 0;
	for(int token_num = 0; token_num < NUM_TOKENS; ++token_num) {
		string token;
		token_end = line.find(DELIMITER, token_start);
		if(token_end != string::npos) { 
			token = line.substr(token_start, token_end-token_start);
			
			// logger->logDebug("Token " + to_string(token_num) + " is " + to_string(token.length()) + " chars: --" + token + "--");
			
			// Store the token based on its column
			switch(token_num) {
				case 0: timestamp             =  token;             break;
				case 1: millis                =  token;             break;
				case 2: data.imgVisibleLValid = (token == "true");  break;
				case 3: left_img_path         = folderName + token; break;
				case 4: data.imgVisibleRValid = (token == "true");  break;
				case 5: right_img_path        = folderName + token; break;
				case 6: data.imgInfraredValid = (token == "true");  break;
				case 7: infrared_path         = folderName + token; break;
				default: break;
			}
			
			token_start = token_end + DELIMITER.length();
		} else {
			logger->logDebug("Ran out of playback file tokens early!");
			break;
		}
	}
	stringstream time_string(timestamp + " " + millis);
	ros::Time nextFrameTime = readTimeFromTabSepInput(time_string);
	lastAcquisitionTime = nextFrameTime;
	
	if(data.imgVisibleLValid) { 
		logger->logDebug("Opening " + left_img_path);
		data.imgVisibleL = cv::imread(left_img_path,  CV_LOAD_IMAGE_GRAYSCALE); 
		if(!data.imgVisibleL.data) {
			logger->logWarning("Couldn't open left image at " + left_img_path);
			data.imgVisibleLValid = false;
		}
	}
	data.acquisitionTime = lastAcquisitionTime;
	if(data.imgVisibleRValid) { 
		logger->logDebug("Opening " + right_img_path);
		data.imgVisibleR = cv::imread(right_img_path,  CV_LOAD_IMAGE_GRAYSCALE); 
		if(!data.imgVisibleR.data) {
			logger->logWarning("Couldn't open right image at " + right_img_path);
			data.imgVisibleRValid = false;
		}
	}
	if(data.imgInfraredValid) { 
		logger->logDebug("Opening " + infrared_path);
		data.imgInfrared = cv::imread(infrared_path,  CV_LOAD_IMAGE_GRAYSCALE); 
		if(!data.imgInfrared.data) {
			logger->logWarning("Couldn't open IR image at " + infrared_path);
			data.imgInfraredValid = false;
		}
	}

	if(playIndexFile.eof()) { 
		logger->logDebug("EOF on playback file"); 
		playbackComplete = true;
	} else {
		if(playedFirstFrame) {
			interframeDelay = (nextFrameTime - lastFrameTime);
		} else {
			interframeDelay = ros::Duration(0);
			playedFirstFrame = true;
		}
	}
	lastFrameTime = nextFrameTime;
	bmLoadImages.end();
}


ImageDataSet ImageAcquisition::acquireImages() {
	ImageDataSet data;
	if(running) {
		FlyCapture2::Image imgL, imgR;
		if(!playbackEnabled) {
			// New images sourced from reality
			Error error;
			bmCamControl.start();
			// If we're using automatic gain control, copy parameters from left to right
			if(autoGainControlEnabled && camLConnected && camRConnected) {
				autoGainControlValues = "";
				for(auto const &pt : autoControlledProps) {
					Property prop;
					prop.type = pt;
					error = camL.GetProperty(&prop);
					if ( error != PGRERROR_OK ) {
						logger->logError("Failed to read gain control from left camera.");
					} else {
						prop.absControl = true; // Use absValue, not valueA (valueA is in "ticks", absValue is in a human-readable unit)
						prop.onOff = true; // Use this property
						prop.autoManualMode = false; // Manual control
						error = camR.SetProperty(&prop);
						if ( error != PGRERROR_OK ) {
							logger->logError("Failed to set right camera gain.");
						} else {
							// Success
							autoGainControlValues = autoGainControlValues + propTypeToString(pt) + "=" + to_string(prop.absValue) + ",";
						} 
					}
				}
			} else {
				// No automatic gain control
			}
			bmCamControl.end();
			
			// Assumes someone has already called beginAcquisition()
			data.acquisitionTime = lastAcquisitionTime;
			
			if(camLConnected)
			{
				logger->logDebug("Waiting for left image");
				// sem_timedwait(&imageWaitL, &imageWaitTimeout);
				sem_wait(&imageWaitL);
				data.imgVisibleLValid = true;
			}
			if(!data.imgVisibleLValid)
			{ logger->logWarning("Error acquiring left image"); }
			if(camRConnected)
			{
				logger->logDebug("Waiting for right image");
				// sem_timedwait(&imageWaitR, &imageWaitTimeout);
				sem_wait(&imageWaitR);
				data.imgVisibleRValid = true;
			}
			data.imgVisibleRValid = (error == PGRERROR_OK);
			if(!data.imgVisibleRValid)
			{ logger->logWarning("Error acquiring right image"); }
			data.imgInfraredValid = false;
		} else {
			// Images sourced from a recording
			openImagesFromRecording(data);
		}
		
		if(!playbackEnabled) {
			// Convert
			data.imgVisibleL = cbItemsL.image;
			data.imgVisibleR = cbItemsR.image;
			
			// Flip
			// Mat imgLFlipped;
			if(flipImgL) {
				bmFlipImageCpu.start();
				cv::flip(data.imgVisibleL,data.imgVisibleL,imgFlipCodeL);
				bmFlipImageCpu.end();
			} else {
				// imgLFlipped = data.imgVisibleL;
			}
			// Mat imgRFlipped;
			if(flipImgR) {
				bmFlipImageCpu.start();
				cv::flip(data.imgVisibleR,data.imgVisibleR,imgFlipCodeR);
				bmFlipImageCpu.end();
			} else {
				// imgRFlipped = data.imgVisibleR;
			}

			
			// Record
			if(recordEnabled) {
				saveImages(data);
			}
		}
	}
	
	return data;
}

cv::Mat ImageAcquisition::cvMatFromFlyCap2Image(FlyCapture2::Image *img) {
	return cv::Mat(img->GetRows(), 
		img->GetCols(),
		CV_8UC1, 
		img->GetData(), 
		img->GetStride());
}

void ImageAcquisition::saveImages(ImageDataSet data) {
	bmSaveImages.start();
	time_t time_acq_s = data.getTimeAcquiredS();
	stringstream fn;
	string left_img_fn = "", right_img_fn = "";
	string infrared_fn = "~";
	
	// Make timestamp
	stringstream ts;
	ts <<  put_time(localtime(&time_acq_s), timestampPattern.c_str());
	string timestamp = ts.str();
	
	if(data.imgVisibleLValid) {
		if (data.imgVisibleL.empty()){
			logger->logWarning("Recording: left image is empty!");
		}
		fn.str(""); fn.clear();
		fn << "leftVisible_"  << timestamp
		     << "_" << setw(3) << setfill('0') << data.getTimeAcquiredMs() << '.' << REC_EXT;
		left_img_fn = fn.str();
		logger->logDebug("Saving L to: " + left_img_fn);
		cv::imwrite(folderName + left_img_fn, data.imgVisibleL);
	}

	if(data.imgVisibleRValid) {
		if (data.imgVisibleR.empty()){
			logger->logWarning("Recording: right image is empty!");
		}
		fn.str(""); fn.clear();
		fn << "rightVisible_"<< timestamp
		     << "_" << setw(3) << setfill('0') << data.getTimeAcquiredMs() << '.' << REC_EXT;
		right_img_fn = fn.str();
		logger->logDebug("Saving R to: " + right_img_fn);
		cv::imwrite(folderName + right_img_fn, data.imgVisibleR);
	}
	
	// Tab-separated, one line per time entry.
	recIndexFile << timestamp << '\t' << setw(3) << setfill('0') << data.getTimeAcquiredMs() << '\t'
	             << (data.imgVisibleLValid? "true" : "false") << '\t' << left_img_fn  << '\t'
	             << (data.imgVisibleRValid? "true" : "false") << '\t' << right_img_fn << '\t'
	             << (data.imgInfraredValid? "true" : "false") << '\t' << infrared_fn  << '\t'
				 << autoGainControlValues << '\t'
	<< endl;
	bmSaveImages.end(data.imgVisibleLValid? 1:0 + data.imgVisibleRValid? 1:0);
}

PropertyType ImageAcquisition::propTypeFromString(string p) {
	if(p.compare("brightness") == 0) {
		return PropertyType::BRIGHTNESS;
	} else if(p.compare("shutter") == 0) {
		return PropertyType::SHUTTER;
	} else if(p.compare("gain") == 0) {
		return PropertyType::GAIN;
	} else if(p.compare("exposure") == 0) {
		return PropertyType::AUTO_EXPOSURE;
	} else {
		logger->logError("Unknown camera property type \"" + p + "\"");
		return PropertyType::UNSPECIFIED_PROPERTY_TYPE;
	}
}

string ImageAcquisition::propTypeToString(PropertyType  p) {
	switch(p) {
		case BRIGHTNESS:                return "Brightness";                break;
		case AUTO_EXPOSURE:             return "Auto exposure";             break;
		case SHARPNESS:                 return "Sharpness";                 break;
		case WHITE_BALANCE:             return "White balance";             break;
		case HUE:                       return "Hue";                       break;
		case SATURATION:                return "Saturation";                break;
		case GAMMA:                     return "Gamma";                     break;
		case IRIS:                      return "Iris";                      break;
		case FOCUS:                     return "Focus";                     break;
		case ZOOM:                      return "Zoom";                      break;
		case PAN:                       return "Pan";                       break;
		case TILT:                      return "Tilt";                      break;
		case SHUTTER:                   return "Shutter";                   break;
		case GAIN:                      return "Gain";                      break;
		case TRIGGER_MODE:              return "Trigger mode";              break;
		case TRIGGER_DELAY:             return "Trigger delay";             break;
		case FRAME_RATE:                return "Frame rate";                break;
		case TEMPERATURE:               return "Temperature";               break;
		case UNSPECIFIED_PROPERTY_TYPE: return "Unspecified property type"; break;
		default:                        return "Unspecified property type"; break;
	}
}

void ImageAcquisition::flyCapImgEvent(Image * pImage, const void * pCallbackData) {
	((CameraCallbackItems*)pCallbackData)->image = cvMatFromFlyCap2Image(pImage);
	cv::Mat image = ((CameraCallbackItems*)pCallbackData)->image;
	((CameraCallbackItems*)pCallbackData)->logger->logDebug(string("Got ") + ((CameraCallbackItems*)pCallbackData)->side + " callback, image is " +
		to_string(image.cols) + "px wide x " + to_string(image.rows) + "px tall.");
	sem_post(((CameraCallbackItems*)pCallbackData)->pSem);
}

void ImageAcquisition::beginAcquisition() {
	Error error;
	if(camLConnected)
	{ 
		logger->logDebug("Firing left trigger.");
		error = camL.FireSoftwareTrigger(); 
	}
	if(error != PGRERROR_OK)
	{ logger->logWarning("Failed to trigger left camera."); }
	lastAcquisitionTime = ros::Time::now();
	if(camRConnected)
	{ 
		logger->logDebug("Firing right trigger.");
		error = camR.FireSoftwareTrigger(); 
	}
	if(error != PGRERROR_OK)
	{ logger->logWarning("Failed to trigger right camera."); }
}