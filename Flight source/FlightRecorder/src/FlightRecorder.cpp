
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <string>
#include <list>

#include <signal.h>

#include <benchmarker.hpp>
#include <AutopilotLogger.hpp>
#include <CameraLogger.hpp>
#include <WindProbeLogger.hpp>
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

/**
 * Return a human-readable string for a boolean
 */
static inline string boolToString(bool arg) {
	return arg? "1":"0";
}

int main(int argc, char** argv)
{
	CLI::App cmdOpts{"Wake Swarm Flight data recorder"};
	
	// Command line options
	bool recImages = true;
	cmdOpts.add_option("--images", recImages, "1 to enable, 0 to disable reading and recording images from the camera. Default is " + boolToString(recImages) + ".");
	string imageExtension = "png";
	cmdOpts.add_option("-x", imageExtension, "Camera image file format, as an extension, such as \"bmp\" or \"png\" (omit the quotes).  Default is \"" + imageExtension + "\".  Must be supported by OpenCV's imwrite(). \"bmp\" is fast (10ms), \"png\" is compact.");
	string dirFormat = "./%F_%H-%M-%S";
	// WARNING: You can inject bash commands here.  For example, try: -d "'; echo hello; '"
	cmdOpts.add_option("-d", dirFormat, "Pattern specifying the directory at which to save the recording. Default: \"" + dirFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  '/' characters are directory separators.  If no leading '/', will be relative to working directory.  Avoid any characters not supported by the filesystem, such as colons.");
	string imageFilenameFormat = "img_%F_%H-%M-%S";
	cmdOpts.add_option("-i", imageFilenameFormat, "Pattern specifying the base filename for images captured from camera. Default: \"" + imageFilenameFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Extension will be supplied by another parameter.  Avoid any characters not supported by the filesystem, such as colons.");
	string cameraSettingsPath = "camera_settings.csv";
	cmdOpts.add_option("-c", cameraSettingsPath, "Path to camera settings file.  Default is " + cameraSettingsPath + ". Must follow a very specific CSV format - see example file.");
	string apLogFilenameFormat = "ap_%F_%H-%M-%S.csv";

	bool recAp = true;
	cmdOpts.add_option("--ap", recAp, "1 to enable, 0 to disable connecting to and recording data from the autopilot.  Default is " + boolToString(recAp) + ".");
	bool autostart = false;
	cmdOpts.add_option("--autostart", autostart, "Wait for the autopilot to arm, then begin recording.  Use with --ap 1.  Default is " + boolToString(autostart) + ".");
	cmdOpts.add_option("-a", apLogFilenameFormat, "Pattern specifying the filename for data captured from the autopilot. Default: \"" + apLogFilenameFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Extension should be included here.  Avoid any characters not supported by the filesystem, such as colons.");
	string autopilotPort = "/dev/ttyS0"; // UART on the Pi
	cmdOpts.add_option("-p", autopilotPort, "Port to which the Pixhawk Autopilot is connected.  Default is " + autopilotPort + ".");
	int apBaudRate = 57600;
	cmdOpts.add_option("-r", apBaudRate, "Baud rate for communicating with the Pixhawk Autopilot.  Default is 57600.");

	bool recProbe = true;
	cmdOpts.add_option("--pressure", recProbe, "1 to enable, 0 to disable reading and recording pressures from the wind probe.  Default is " + boolToString(recProbe) + ".");
	string wpLogFilenameFormat = "wp_%F_%H-%M-%S.csv";
	cmdOpts.add_option("-w", wpLogFilenameFormat, "Pattern specifying the filename for data captured from the wind probe. Default: \"" + apLogFilenameFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Extension should be included here.  Avoid any characters not supported by the filesystem, such as colons.");
	unsigned int spiClock = 5000000; // Default to the max DLHR rate
	cmdOpts.add_option("--spiclk", spiClock, "SPI clock rate to use interacting with the sensors.  The PI can do 3.814 kHz to 125 MHz, and the DLHR sensors can go up to 5MHz.  Default is 5MHz.");
	string sensorPortName = "/dev/spidev0.1";
	cmdOpts.add_option("--sensorSpiPort", sensorPortName, "SPI device connected to the sensor part of the wind probe.  Default is \"" + sensorPortName + "\".");
	string muxPortName = "/dev/spidev0.0";
	cmdOpts.add_option("--muxSpiPort", sensorPortName, "SPI device connected to the multiplexer part of the wind probe.  Default is \"" + muxPortName + "\".");
	CLI11_PARSE(cmdOpts, argc, argv); // This will exit if the user said "-h" or "--help"
	
	int result = 0;
	signal(SIGINT, handle_signal);
	
	// Set up benchmarking
	list<const Benchmarker *> allBms;
	Benchmarker bmWholeFrame("Entire frame");
	Benchmarker bmSync("Sync filesystem");
	allBms.push_back(&bmWholeFrame);
	allBms.push_back(&bmSync);

	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	// cout << "recImages: " << recImages <<", recAp: " << recAp << ", recProbe: " << recProbe <<", autostart: " << autostart << endl;

	// Create directory to store recording
	string recordingDir = date::format(dirFormat, date::floor<milliseconds>(system_clock::now()));
	cout << "Storing recording at: " << recordingDir << endl;
	// Single quotes make it slightly harder to route a bash command in this way
	system(("mkdir -p '" + recordingDir + "'").c_str());
	
	// Setup autopilot logger
	AutopilotLogger apLogger(recordingDir, apLogFilenameFormat, autopilotPort, apBaudRate);
	if(recAp) {
		// Start logging data from the autopilot in another thread
		apLogger.startLogging();
	}
	
	// Setup wind probe logger
	WindProbeLogger wpLogger(recordingDir, wpLogFilenameFormat, sensorPortName, muxPortName, spiClock);
	if(recProbe) {
		wpLogger.openPorts();
		wpLogger.startLogging();
	}
	
	// Set up camera logger
	CameraLogger camLogger(recordingDir, imageFilenameFormat, imageExtension, allBms);
	if(recImages) {
		camLogger.initCamera(cameraSettingsPath);
	}
	try {
		while(caught_exit == 0) {
			bmWholeFrame.start();
			
			// This is the thread performing image logging
			if(recImages) { 
				camLogger.captureAndLogImage();
			}
			
			// Will move this to another thread later
			if(recProbe) {
				wpLogger.startReadings();
				this_thread::sleep_for(milliseconds(4));
				wpLogger.logReadings();
			}
			
			// Flush OS filesystem buffers to disk
			bmSync.start();
			sync();
			bmSync.end();
			
			bmWholeFrame.end();
		}
		cout << "Exiting." << endl;
	} catch (Exception e) {
		cout << "Error in main loop: " << e.what() << endl;
	}

	apLogger.stopLogging();

	sync();
	Benchmarker::summarizeBenchmarksToStream(allBms, cout);

	return result;

}