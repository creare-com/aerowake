/*
	CameraLogger.hpp
	
	Class for saving images from a FLIR USB camera using the Spinnaker API
	
	2019-05-28  JDW  Created.
*/

#ifndef __CAMERALOGGER_HPP__
#define __CAMERALOGGER_HPP__

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <list>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#pragma GCC diagnostic pop
#include <opencv2/opencv.hpp>

#include <benchmarker.hpp>
#include <date.h>

using namespace std;
using namespace std::chrono;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class CameraLogger {
public:
    CameraLogger(string recordingDir, string imageFilenameFormat, string extension, list<const Benchmarker *> &allBms);
    virtual ~CameraLogger();
    
    /**
     * Find the first USB camera, configure it, and tell it to begin acquisition
     * Returns true for success, false otherwise
     */
    bool initCamera(string settingsFilePath = "");
    /**
     * Pull one frame from the camera and save it to disk at the configured location
     * Returns true for success, false otherwise
     */
    bool captureAndLogImage();
    
    
private:
    // Settings/configuration
    string recordingDir;
    string imageFilenameFormat;
    string extension;
    // Benchmarkers
    Benchmarker bmNextImage;
    Benchmarker bmSpinConv ;
    Benchmarker bmCvConv   ;
    Benchmarker bmSave     ;
    Benchmarker bmRel      ;
    
    // Spinnaker items kept persistently
    SystemPtr spinSystem = nullptr;
    InterfaceList interfaceList;
    CameraList camList;
    CameraPtr pCam = nullptr;
    
    static bool ConfigureExposure(INodeMap & nodeMap, bool autoExpose = false, double exposureTimeToSet = 2000000.0);
    static bool ApplyDefaultSettings(INodeMap & nodeMap);
    static bool ApplySpinnakerSettingsFile(INodeMap & nodeMap, const string filePath);
    static bool ApplySpinnakerEnumOption(INodeMap & nodeMap, const string& nodeName, const string& value);
    static bool ApplySpinnakerStringOption(INodeMap & nodeMap, const string& nodeName, const string& value);
    static bool ApplySpinnakerFloatOption(INodeMap & nodeMap, const string& nodeName, double value);
    static bool ApplySpinnakerIntOption(INodeMap & nodeMap, const string& nodeName, int value);
    static cv::Mat cvMatFromSpinnakerImage(ImagePtr img);
}; 

# endif // __CAMERALOGGER_HPP__