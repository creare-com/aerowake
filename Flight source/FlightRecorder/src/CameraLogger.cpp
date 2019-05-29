/*
	CameraLogger.cpp
	
	Class for saving images from a FLIR USB camera using the Spinnaker API
	
	2019-05-28  JDW  Created.
*/

#include <CameraLogger.hpp>
using namespace std;
using namespace std::chrono;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

CameraLogger::CameraLogger(string recordingDir, string imageFilenameFormat, string extension, list<const Benchmarker *> &allBms) :
    recordingDir(recordingDir), imageFilenameFormat(imageFilenameFormat), extension(extension),
    bmNextImage ("Next Image"),
    bmSpinConv  ("Spinnaker conversion"),
    bmCvConv    ("Conversion to OpenCV"),
    bmSave      ("Image save"),
    bmRel       ("Image release")
{
    allBms.push_back(&bmNextImage);
    allBms.push_back(&bmSpinConv);
    allBms.push_back(&bmCvConv);
    allBms.push_back(&bmSave);
    allBms.push_back(&bmRel);


}

CameraLogger::~CameraLogger() {
    try
    {
        if(pCam != nullptr) {
            pCam->EndAcquisition();
            pCam = nullptr;
        }
        camList.Clear();
        // Clear camera list before releasing system
        interfaceList.Clear();
        if(spinSystem != nullptr) {
            spinSystem->ReleaseInstance();
            spinSystem = nullptr;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error destructing CameraLogger object: " << e.what() << endl;
    }
}

/**
 * Find the first USB camera, configure it, and tell it to begin acquisition
 */
bool CameraLogger::initCamera(string settingsFilePath) {
    // Retrieve singleton reference to system object
    spinSystem = System::GetInstance();
    

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = spinSystem->GetLibraryVersion();
    cout << "Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build << endl << endl;

    interfaceList = spinSystem->GetInterfaces();
    unsigned int numInterfaces = interfaceList.GetSize();
    // cout << "Number of interfaces detected: " << numInterfaces << endl << endl;
    
    
    // Finish if there are no cameras
    if (numInterfaces == 0)
    {
        cout << "No spinnaker interfaces!" << endl;
        return false;
    }
    
    
    
    //
    // Create shared pointer interface
    //
    // *** NOTES ***
    // The InterfacePtr object is a smart pointer, and will generally clean 
    // itself up upon exiting its scope.
    //
    // *** LATER ***
    // However, if a smart interface pointer is created in the same scope that 
    // a system object is explicitly released (i.e. this scope), the reference to 
    // the interface must be broken by manually setting the pointer to nullptr.
    //
    InterfacePtr interfacePtr = nullptr;
    gcstring interfaceDisplayName = "";
    gcstring interfaceType = "";
    
    try
    {
        for (unsigned int i = 0; i < numInterfaces; i++)
        {    
            // Select interface
            interfacePtr = interfaceList.GetByIndex(i);
            INodeMap & nodeMapInterface = interfacePtr->GetTLNodeMap();

            // Query interface
            CStringPtr ptrInterfaceDisplayName = nodeMapInterface.GetNode("InterfaceDisplayName");
            if (IsAvailable(ptrInterfaceDisplayName) && IsReadable(ptrInterfaceDisplayName))
            {
                interfaceDisplayName = ptrInterfaceDisplayName->GetValue();
                // cout << interfaceDisplayName << endl;
            }
            else
            {
                cout << "Interface display name not readable" << endl;
            }
            CStringPtr ptrInterfaceType = nodeMapInterface.GetNode("InterfaceType");
            if (IsAvailable(ptrInterfaceType) && IsReadable(ptrInterfaceType))
            {
                interfaceType = ptrInterfaceType->GetValue();
                // cout << interfaceType << endl;
                if(interfaceType.compare("U3V") == 0) {
                    // This is the target interface
                    ptrInterfaceType = nullptr;
                    break;
                }
            }
            else
            {
                cout << "Interface type name not readable" << endl;
            }
            ptrInterfaceDisplayName = nullptr;
            ptrInterfaceType = nullptr;
            interfacePtr = nullptr;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error finding interface: " << e.what() << endl;
        return false;
    }
    
    try
    {
        // If we found the interface, connect to the first camera
        if(interfacePtr != nullptr) {
            camList = interfacePtr->GetCameras();
            
            if(camList.GetSize() > 0) {
                pCam = camList.GetByIndex(0);
                cout << "Found camera on " << interfaceDisplayName << "." << endl;
                
                // Initialize camera
                pCam->Init();
                
                // Retrieve TL device nodemap and print device information
                INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
                // Retrieve GenICam nodemap
                INodeMap & nodeMap = pCam->GetNodeMap();
                
                gcstring deviceSerialNumber("");
                CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
                if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
                {
                    deviceSerialNumber = ptrStringSerial->GetValue();

                    cout << "Device serial number retrieved as " << deviceSerialNumber << "" << endl;
                }
                cout << endl;
                
                if(settingsFilePath != "") {
                    ApplySpinnakerCsvSettingsFile(nodeMap, settingsFilePath);
                } else {
                    ApplyDefaultSettings(nodeMap);
                }
                
                
                // Tell it to become ready
                cout << "Beginning acquisition " << endl;
                pCam->BeginAcquisition();
                cout << "done. " << endl;
            }
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error connecting to camera: " << e.what() << endl;
        return false;
    }
    interfacePtr = nullptr;
    cout << "Done initializing." << endl;
    // Success!
    return true;
}
/**
 * Pull one frame from the camera and save it to disk at the configured location
 */
bool CameraLogger::captureAndLogImage() {
              
    cout << "Acquiring image" << endl;
    try
    {
        //
        // Retrieve next received image
        //
        // *** NOTES ***
        // Capturing an image houses images on the camera buffer. Trying
        // to capture an image that does not exist will hang the camera.
        //
        // *** LATER ***
        // Once an image from the buffer is saved and/or no longer 
        // needed, the image must be released in order to keep the 
        // buffer from filling up.
        //
        bmNextImage.start();
        ImagePtr pResultImage = pCam->GetNextImage();
        bmNextImage.end();

        if (pResultImage->IsIncomplete())
        {
            // Retrieve and print the image status description
            cout << "Image incomplete: "
                << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                << "" << endl << endl;
        }
        else
        {
            //
            // Print image information; height and width recorded in pixels
            //
            // *** NOTES ***
            // Images have quite a bit of available metadata including
            // things such as CRC, image status, and offset values, to
            // name a few.
            //
            // const size_t width = pResultImage->GetWidth();
            // const size_t height = pResultImage->GetHeight();
            // cout << "Grabbed image " << ", width = " << width << ", height = " << height << endl;

            //
            // Convert image to mono 8
            //
            // *** NOTES ***
            // Images can be converted between pixel formats by using 
            // the appropriate enumeration value. Unlike the original 
            // image, the converted one does not need to be released as 
            // it does not affect the camera buffer.
            //
            // When converting images, color processing algorithm is an
            // optional parameter.
            // 
            bmSpinConv.start();
            ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
            bmSpinConv.end();

            ostringstream filename;
            filename << recordingDir << "/" << date::format(imageFilenameFormat, date::floor<milliseconds>(system_clock::now())) << "." << extension;
            bmCvConv.start();
            cv::Mat cvImg = cvMatFromSpinnakerImage(convertedImage);
            bmCvConv.end();

            bmSave.start();
            // convertedImage->Save(filename.str().c_str());
            cv::imwrite(filename.str().c_str(), cvImg);
            cout << "Image saved at " << filename.str() << endl;
            bmSave.end();
        }

        //
        // Release image
        //
        // *** NOTES ***
        // Images retrieved directly from the camera (i.e. non-converted
        // images) need to be released in order to keep from filling the
        // buffer.
        //
        bmRel.start();
        pResultImage->Release();
        bmRel.end();

        cout << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        return false;
    }
    
    return true;
}


// This function configures a custom exposure time. Automatic exposure is turned 
// off in order to allow for the customization, and then the custom setting is 
// applied.
bool CameraLogger::ConfigureExposure(INodeMap & nodeMap, bool autoExpose, double exposureTimeToSet)
{
    bool result = true;
    result = result && ApplySpinnakerEnumOption(nodeMap, "ExposureAuto", autoExpose? "Continuous":"Off");
    if(!autoExpose) {
        result = result && ApplySpinnakerFloatOption(nodeMap, "ExposureTime", exposureTimeToSet);
    }
    return result;
}

bool CameraLogger::ApplyDefaultSettings(INodeMap & nodeMap) {
    return ConfigureExposure(nodeMap, true) && ApplySpinnakerEnumOption(nodeMap, "AcquisitionMode", "Continuous");
}

bool CameraLogger::ApplySpinnakerCsvSettingsFile(INodeMap & nodeMap, const string& filePath) {
    bool result = true;
    
    // Expects to have the first column be the node name, the second be "enum", "integer", "float", or "string", and the third the value.
    // No quotes anywhere.
    ifstream infile(filePath);
    string line;
    int lineNum = 0;
    while(getline(infile, line)) {
        lineNum++; // Needs to be at the start so we can say "continue" later on
        stringstream row(line);
        string nodeName;
        string nodeType;
        string valueStr;
        if(!getline(row, nodeName, ',')) {
            cout << "Could not read node name from line " << lineNum << endl;
            result = false;
            continue;
        }
        if(!getline(row, nodeType, ',')) {
            cout << "Could not read node type from line " << lineNum << endl;
            result = false;
            continue;
        }
        if(!getline(row, valueStr, ',')) {
            cout << "Could not read value from line " << lineNum << endl;
            result = false;
            continue;
        }
        
        if (nodeType == "enum") {
            result = result && ApplySpinnakerEnumOption(nodeMap, nodeName, valueStr);
        } else if (nodeType == "string") {
            result = result && ApplySpinnakerStringOption(nodeMap, nodeName, valueStr);
        } else if (nodeType == "integer") {
            stringstream valueStream(valueStr);
            int value;
            valueStream >> value;
            result = result && ApplySpinnakerIntOption(nodeMap, nodeName, value);
        } else if (nodeType == "float") {
            stringstream valueStream(valueStr);
            double value;
            valueStream >> value;
            result = result && ApplySpinnakerFloatOption(nodeMap, nodeName, value);
        } else {
            cout << "Unrecognized node type: " << nodeType << " on line " << lineNum << endl;
            result = false;
        }
        
    }
    
    return result;
}

bool CameraLogger::ApplySpinnakerEnumOption(INodeMap & nodeMap, const string& nodeName, const string& value) {
    try {
        CEnumerationPtr node = nodeMap.GetNode(nodeName.c_str());
        if (!IsAvailable(node)) 
        {
            cout << "Spinnaker node " << nodeName << " is unavailable." << endl;
            return false;
        }
        if (!IsWritable(node)) {
            cout << "Spinnaker node " << nodeName << " is not writeable." << endl;
            return false;
        }

        CEnumEntryPtr ptrEnumValue = node->GetEntryByName(value.c_str());
        if (!IsAvailable(ptrEnumValue) || !IsReadable(ptrEnumValue))
        {
            cout << "Spinnaker enum " << nodeName << " does not have a value called " << value << "." << endl;
            return false;
        }

        node->SetIntValue(ptrEnumValue->GetValue());

        cout << nodeName << " set to " << value << "." << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error applying setting " << nodeName << " to camera: " << e.what() << endl;
        return false;
    }
    // Success!
    return true;
}

bool CameraLogger::ApplySpinnakerStringOption(INodeMap & nodeMap, const string& nodeName, const string& value) {
    try {
        CStringPtr node = nodeMap.GetNode(nodeName.c_str());
        if (!IsAvailable(node)) 
        {
            cout << "Spinnaker node " << nodeName << " is unavailable." << endl;
            return false;
        }
        if (!IsWritable(node)) {
            cout << "Spinnaker node " << nodeName << " is not writeable." << endl;
            return false;
        }

        node->SetValue(value.c_str());

        cout << nodeName << " set to " << value << "." << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error applying setting " << nodeName << " to camera: " << e.what() << endl;
        return false;
    }
    // Success!
    return true;
}

bool CameraLogger::ApplySpinnakerFloatOption(INodeMap & nodeMap, const string& nodeName, double value) {
    try {
        CFloatPtr node = nodeMap.GetNode(nodeName.c_str());
        if (!IsAvailable(node)) 
        {
            cout << "Spinnaker node " << nodeName << " is unavailable." << endl;
            return false;
        }
        if (!IsWritable(node)) {
            cout << "Spinnaker node " << nodeName << " is not writeable." << endl;
            return false;
        }

        const double max = node->GetMax();
        const double min = node->GetMin();

        if (value > max) {
            cout << "Correcting camera option " << nodeName << " from " << value << " to its max of " << max << "." << endl;
            value = max;
        } else if(value < min) {
            cout << "Correcting camera option " << nodeName << " from " << value << " to its min of " << min << "." << endl;
            value = min;
        }

        node->SetValue(value);

        cout << nodeName << " set to " << value << "." << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error applying setting " << nodeName << " to camera: " << e.what() << endl;
        return false;
    }
    // Success!
    return true;
}

bool CameraLogger::ApplySpinnakerIntOption(INodeMap & nodeMap, const string& nodeName, int value) {
    try {
        CIntegerPtr node = nodeMap.GetNode(nodeName.c_str());
        if (!IsAvailable(node)) 
        {
            cout << "Spinnaker node " << nodeName << " is unavailable." << endl;
            return false;
        }
        if (!IsWritable(node)) {
            cout << "Spinnaker node " << nodeName << " is not writeable." << endl;
            return false;
        }

        const int max = node->GetMax();
        const int min = node->GetMin();

        if (value > max) {
            cout << "Correcting camera option " << nodeName << " from " << value << " to its max of " << max << "." << endl;
            value = max;
        } else if(value < min) {
            cout << "Correcting camera option " << nodeName << " from " << value << " to its min of " << min << "." << endl;
            value = min;
        }

        node->SetValue(value);

        cout << nodeName << " set to " << value << "." << endl;
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error applying setting " << nodeName << " to camera: " << e.what() << endl;
        return false;
    }
    // Success!
    return true;
}




cv::Mat CameraLogger::cvMatFromSpinnakerImage(ImagePtr img) {
    return cv::Mat(img->GetHeight(), 
        img->GetWidth(),
        CV_8UC1, 
        img->GetData(), 
        img->GetStride());
}

