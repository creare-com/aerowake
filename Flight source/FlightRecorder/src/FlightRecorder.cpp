
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

#include <benchmarker.hpp>
#include <serial_port.h>
#include <autopilot_interface.h>
#include <CameraLogger.hpp>
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



int main(int argc, char** argv)
{
	CLI::App cmdOpts{"Wake Swarm Flight data recorder"};
	
	// Command line options
	string extension = "png";
	cmdOpts.add_option("-x", extension, "File format, as an extension, such as \"bmp\" or \"png\" (omit the quotes).  Default is \"" + extension + "\".  Must be supported by OpenCV's imwrite(). \"bmp\" is fast (10ms), \"png\" is compact.");
	string dirFormat = "./%F_%H-%M-%S";
	// WARNING: You can inject bash commands here.  For example, try: -d "'; echo hello; '"
	cmdOpts.add_option("-d", dirFormat, "Pattern specifying the directory at which to save the recording. Default: \"" + dirFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  '/' characters are directory separators.  If no leading '/', will be relative to working directory.  Avoid any characters not supported by the filesystem, such as colons.");
	string imageFilenameFormat = "img_%F_%H-%M-%S";
	cmdOpts.add_option("-i", imageFilenameFormat, "Pattern specifying the base filename for images captured from camera. Default: \"" + imageFilenameFormat + "\".  Will substitute flags found here: https://howardhinnant.github.io/date/date.html#to_stream_formatting.  Extension will be supplied by another parameter.  Avoid any characters not supported by the filesystem, such as colons.");
	string cameraSettingsPath = "camera_settings.csv";
	cmdOpts.add_option("-c", cameraSettingsPath, "Path to camera settings file.  Default is " + cameraSettingsPath + ". Must follow a very specific CSV format - see example file.");
	string autopilotPort = "/dev/ttyTHS1"; // UART0 on the Jetson TX2 with Auvidea J120's DTS
	cmdOpts.add_option("-p", autopilotPort, "Port to which the Pixhawk Autopilot is connected.  Default is " + autopilotPort + ".");
	int apBaudRate = 57600;
	cmdOpts.add_option("-r", apBaudRate, "Baud rate for communicating with the Pixhawk Autopilot.  Default is 57600.");
	CLI11_PARSE(cmdOpts, argc, argv); // This will exit if the user said "-h" or "--help"
	
	int result = 0;
	signal(SIGINT, handle_signal);
	
	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	// Create directory to store recording
	string recordingDir = date::format(dirFormat, date::floor<milliseconds>(system_clock::now()));
	cout << "Storing recording at: " << recordingDir << endl;
	// Single quotes make it slightly harder to route a bash command in this way
	system(("mkdir -p '" + recordingDir + "'").c_str());
	
	
	// Set up autopilot connection
	Serial_Port apSerialPort(autopilotPort.c_str(), apBaudRate);
	Autopilot_Interface apIntf(&apSerialPort);
	apSerialPort.start();
	apIntf.start();
	
	cout << "Sending heartbeat message" << endl;
	mavlink_heartbeat_t heartbeat;
	mavlink_message_t message;
	mavlink_msg_heartbeat_encode(/*uint8_t system_id*/ 0, /*uint8_t component_id*/0, &message, &heartbeat);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
	list<const Benchmarker *> allBms;
	Benchmarker bmWholeFrame("Entire frame");
	Benchmarker bmSync("Sync filesystem");
	allBms.push_back(&bmWholeFrame);
	allBms.push_back(&bmSync);

	CameraLogger camLogger(recordingDir, imageFilenameFormat, extension, allBms);
	camLogger.initCamera(cameraSettingsPath);
	try {
		while(caught_exit == 0) {
			bmWholeFrame.start();
			
			camLogger.captureAndLogImage();
			
			bmSync.start();
			sync();
			bmSync.end();
			
			bmWholeFrame.end();
		}
		cout << "Exiting." << endl;
	} catch (Exception e) {
		cout << "Error in main loop: " << e.what() << endl;
	}

	// De-initialize autopilot connection
	apIntf.disable_offboard_control();
	apIntf.stop();
	apSerialPort.stop();


	sync();
	Benchmarker::summarizeBenchmarksToStream(allBms, cout);

	return result;

}