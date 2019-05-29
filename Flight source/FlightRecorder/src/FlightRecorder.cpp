
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <list>

#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <memory.h>
#include <netdb.h>
#include <netinet/in.h>
#include <net/if.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <CameraLogger.hpp>
#include <benchmarker.hpp>
#include <CLI11.hpp>
#include <date.h>

using namespace std;
using namespace std::chrono;

static volatile sig_atomic_t caught_exit = 0;

extern "C" void handle_signal(int sig) {
    if(sig == SIGINT) {
        caught_exit = 1;
    }
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

int main(int argc, char** argv)
{
    CLI::App cmdOpts{"Wake Swarm Flight data recorder"};
    
    // Command line options
    string extension = "bmp";
    cmdOpts.add_option("-x", extension, "File format, as an extension, such as \"bmp\" or \"png\" (omit the quotes).  Default is \"" + extension + "\".  Must be supported by OpenCV's imwrite(). \"bmp\" is fast (10ms), \"png\" is compact.");
    string dirFormat = "./%F_%H-%M-%S";
    cmdOpts.add_option("-d", dirFormat, "Pattern specifying the directory at which to save the recording. Default: \"" + dirFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  '/' characters are directory separators.  If no leading '/', will be relative to working directory.  Avoid any characters not supported by the filesystem, such as colons.");
    string imageFilenameFormat = "img_%F_%H-%M-%S";
    cmdOpts.add_option("-i", imageFilenameFormat, "Pattern specifying the base filename for images captured from camera. Default: \"" + imageFilenameFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Extension will be supplied by another parameter.  Avoid any characters not supported by the filesystem, such as colons.");
    string cameraSettingsPath = "camera_settings.csv";
    cmdOpts.add_option("-c", cameraSettingsPath, "Path to camera settings file.  Default is " + cameraSettingsPath + ". Must follow a very specific CSV format - see example file.");
    CLI11_PARSE(cmdOpts, argc, argv); // This will exit if the user said "-h" or "--help"
    
    int result = 0;
    signal(SIGINT, handle_signal);
    
    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Create directory to store recording
    string recordingDir = date::format(dirFormat, date::floor<milliseconds>(system_clock::now()));
    cout << "Storing recording at: " << recordingDir << endl;
    // WARNING: do not remove the single quotes.  They prevent bash command injection.
    system(("mkdir -p '" + recordingDir + "'").c_str());
    
    
    list<const Benchmarker *> allBms;
    Benchmarker bmWholeFrame("Entire frame");
    // Benchmarker bmSync("Sync filesystem");
    allBms.push_back(&bmWholeFrame);
    // allBms.push_back(&bmSync);

    CameraLogger camLogger(recordingDir, imageFilenameFormat, extension, allBms);
    camLogger.initCamera(cameraSettingsPath);
    try {
        while(caught_exit == 0) {
            bmWholeFrame.start();
            
            camLogger.captureAndLogImage();
            
            // bmSync.start();
            // sync();
            // bmSync.end();
            
            bmWholeFrame.end();
        }
        cout << "Exiting." << endl;
    } catch (Exception e) {
        cout << "Error in main loop: " << e.what() << endl;
    }
    sync();
    summarizeBenchmarksToLog(allBms);

    return result;

}