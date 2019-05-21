
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <memory.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <opencv2/opencv.hpp>

#include "benchmarker.h"
#include "CLI11.hpp"
#include "date.h"

using namespace std;
using namespace std::chrono;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

// This function configures a custom exposure time. Automatic exposure is turned 
// off in order to allow for the customization, and then the custom setting is 
// applied.
int ConfigureExposure(INodeMap & nodeMap, bool autoExpose = false, double exposureTimeToSet = 2000000.0)
{
    int result = 0;

    cout << endl << endl << "*** CONFIGURING EXPOSURE ***" << endl << endl;

    try
    {
        //
        // Turn off automatic exposure mode
        //
        // *** NOTES ***
        // Automatic exposure prevents the manual configuration of exposure 
        // time and needs to be turned off.
        //
        // *** LATER ***
        // Exposure time can be set automatically or manually as needed. This
        // example turns automatic exposure off to set it manually and back
        // on in order to return the camera to its default state.
        //
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
        {
            cout << "Unable to disable automatic exposure (node retrieval). Aborting" << endl << endl;
            return -1;
        }

        if(autoExpose) {
            CEnumEntryPtr ptrExposureAutoOn = ptrExposureAuto->GetEntryByName("On");
            if (!IsAvailable(ptrExposureAutoOn) || !IsReadable(ptrExposureAutoOn))
            {
                cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting" << endl << endl;
                return -1;
            }

            ptrExposureAuto->SetIntValue(ptrExposureAutoOn->GetValue());

            cout << "Automatic exposure enabled" << endl;

        } else {
            CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
            if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
            {
                cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting" << endl << endl;
                return -1;
            }

            ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

            cout << "Automatic exposure disabled" << endl;

            //
            // Set exposure time manually; exposure time recorded in microseconds
            //
            // *** NOTES ***
            // The node is checked for availability and writability prior to the 
            // setting of the node. Further, it is ensured that the desired exposure 
            // time does not exceed the maximum. Exposure time is counted in 
            // microseconds. This information can be found out either by 
            // retrieving the unit with the GetUnit() method or by checking SpinView.
            // 
            CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
            if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
            {
                cout << "Unable to set exposure time. Aborting" << endl << endl;
                return -1;
            }

            // Ensure desired exposure time does not exceed the maximum
            const double exposureTimeMax = ptrExposureTime->GetMax();

            if (exposureTimeToSet > exposureTimeMax)
            {
                exposureTimeToSet = exposureTimeMax;
            }

            ptrExposureTime->SetValue(exposureTimeToSet);

            cout << std::fixed << "Exposure time set to " << exposureTimeToSet << " us" << endl << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

cv::Mat cvMatFromSpinnakerImage(ImagePtr img) {
    return cv::Mat(img->GetHeight(), 
        img->GetWidth(),
        CV_8UC1, 
        img->GetData(), 
        img->GetStride());
}

// Iterate through allBms and write a summary of benchmarkers
// to the console
void summarizeBenchmarksToLog(list<const Benchmarker *> allBms) {
	for(auto bm = allBms.begin(); bm != allBms.end(); ++bm) {
		cout << endl << (*bm)->getName() << ": "  << (*bm)->getAvgMs() << "ms avg";
		if((*bm)->getAvgItemsProcessed() > 0) {
			cout << ", " << (*bm)->getAvgItemsProcessed() << " avg items";
		}
		cout << " (" << (*bm)->getIterations() << " iterations).";
	}
    cout << endl;
}

/**
 * Call before attempting to open a file for writing.  Checks if the directory in the path exists,
 * and if not, attempts to create the directories leading up to it.
 *
 * fullPath - full path to file, such as /dir/subdir/filename, or ./dir/subdir/filename, or filename
 */
void ensureDirExists(const string &fullPath) {
    size_t slashLoc;
    cout << "Splitting: " << fullPath << endl;
    slashLoc = fullPath.find_last_of("/\\");
    
    string dir;
    // string file;
    if(slashLoc == string::npos) {
        // No / found, so this is a bare file.
        // So we don't need to do anything
        cout << "Writing to working dir; don't need to create anything." << endl;
    } else {
        dir = fullPath.substr(0, slashLoc);
        // file = fullPath.substr(slashLoc + 1);
        cout << "Dir: " << dir << endl;
        
        // Ensure existence if we have permissions.  Ignore errors.
        // WARNING: do not remove the single quotes.  They prevent bash command injection.
        system(("mkdir -p '" + dir + "'").c_str());
    }
}

int main(int argc, char** argv)
{
    CLI::App cmdOpts{"Wake Swarm Flight data recorder"};
    
    // Command line options
    string extension = "bmp";
    cmdOpts.add_option("-x", extension, "File format, as an extension, such as \"bmp\" or \"png\" (omit the quotes).  Default is \"" + extension + "\".  Must be supported by OpenCV's imwrite(). \"bmp\" is fast (10ms), \"png\" is compact.");
    string pathFormat = "./%F_%H-%M-%S";
    cmdOpts.add_option("-p", pathFormat, "Pattern specifying where to save the files and what to name them. Default: \"" + pathFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  '/' characters are directory separators.  If no leading '/', will be relative to working directory.  Avoid any characters not supported by the filesystem, such as colons.");
    CLI11_PARSE(cmdOpts, argc, argv); // This will exit if the user said "-h" or "--help"
    
    int result = 0;
    
    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: "
        << spinnakerLibraryVersion.major << "."
        << spinnakerLibraryVersion.minor << "."
        << spinnakerLibraryVersion.type << "."
        << spinnakerLibraryVersion.build << endl << endl;

    InterfaceList interfaceList = system->GetInterfaces();
    unsigned int numInterfaces = interfaceList.GetSize();
    // cout << "Number of interfaces detected: " << numInterfaces << endl << endl;

    // Finish if there are no cameras
    if (numInterfaces == 0)
    {
        // Clear camera list before releasing system
        interfaceList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "No interfaces!" << endl;
        cout << "Done! Press Enter to exit" << endl;
        getchar();

        return -1;
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
        result = -1;
    }
    
    try
    {
        // If we found the interface, connect to the first camera
        if(interfacePtr != nullptr) {
            CameraList camList = interfacePtr->GetCameras();
            
            if(camList.GetSize() > 0) {
                CameraPtr pCam = camList.GetByIndex(0);
                cout << "Found camera on " << interfaceDisplayName << "." << endl;
                
                
                // Retrieve TL device nodemap and print device information
                INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
                // Initialize camera
                pCam->Init();
                // Retrieve GenICam nodemap
                INodeMap & nodeMap = pCam->GetNodeMap();
                
                // Set exposure
                // ConfigureExposure(nodeMap, false, 2500);
                ConfigureExposure(nodeMap, true);

                CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
                if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
                {
                    cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting" << endl << endl;
                    return -1;
                }

                // Retrieve entry node from enumeration node
                CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
                if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
                {
                    cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting" << endl << endl;
                    return -1;
                }
                // Retrieve integer value from entry node
                const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
                // Set integer value from entry node as new value of enumeration node
                ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
                cout << "Acquisition mode set to continuous" << endl;
                gcstring deviceSerialNumber("");
                CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
                if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
                {
                    deviceSerialNumber = ptrStringSerial->GetValue();

                    cout << "Device serial number retrieved as " << deviceSerialNumber << "" << endl;
                }
                cout << endl;


                Benchmarker bmWholeFrame("Entire frame");
                Benchmarker bmNextImage ("Next Image");
                Benchmarker bmSpinConv  ("Spinnaker conversion");
                Benchmarker bmCvConv    ("Conversion to OpenCV");
                Benchmarker bmMkdir     ("Image directory creation");
                Benchmarker bmSave      ("Image save");
                Benchmarker bmRel       ("Image release");
                list<const Benchmarker *> allBms;
                allBms.push_back(&bmWholeFrame);
                allBms.push_back(&bmNextImage);
                allBms.push_back(&bmSpinConv);
                allBms.push_back(&bmCvConv);
                allBms.push_back(&bmMkdir);
                allBms.push_back(&bmSave);
                allBms.push_back(&bmRel);
                
                pCam->BeginAcquisition();
                cout << "Acquiring images" << endl;
                try
                {
                    while(true) {
                        bmWholeFrame.start();
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
                            filename << date::format(pathFormat, date::floor<milliseconds>(system_clock::now())) << "." << extension;
                            bmCvConv.start();
                            cv::Mat cvImg = cvMatFromSpinnakerImage(convertedImage);
                            bmCvConv.end();
                            bmMkdir.start();
                            ensureDirExists(filename.str());
                            bmMkdir.end();
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
                        bmWholeFrame.end();
                        summarizeBenchmarksToLog(allBms);
                    }
                }
                catch (Spinnaker::Exception &e)
                {
                    cout << "Error: " << e.what() << endl;
                    result = -1;
                }
                pCam->EndAcquisition();
            }
            camList.Clear();
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error connecting to camera: " << e.what() << endl;
        result = -1;
    }

    interfacePtr = nullptr;
    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit" << endl;
    getchar();

    return result;

}