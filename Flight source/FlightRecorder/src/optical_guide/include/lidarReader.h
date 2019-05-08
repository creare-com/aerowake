/*
	lidarReader.h
	
	Declarations for reading and packaging the LIDAR data
	
	2016-06-22  JDW  Created.
*/

#ifndef __PCG_LIDAR_READER_H__
#define __PCG_LIDAR_READER_H__

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <list>
#include <math.h>
#include <errno.h>
#include <iomanip>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "benchmarker.h"
#include "quanergyMsgs.h"
#include "logger.h"
#include "json.hpp"

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"


class LidarReader {
private:
	int sockLidar = -1; // Unix socket number
	struct sockaddr_in lidarSocketAddress;
	bool running = false; // We assume the TCP socket status maps 1:1 with the operation of the LIDAR, which is how the LIDAR is designed to operate

	// State variables
	char recvBuffer[M8PacketHeader::EXPECTED_SIZE_B];
	sensor_msgs::PointCloud * currentPointCloud = NULL;
	double lastSeenAzRad; // The most recent azimuth processed
	unsigned int nextPktStartIdx = 0;
	unsigned int nextPointNumToStore = 0;
	unsigned int seqNum = 0;

	// Constants based on the mechanical installation of the LIDAR
	static constexpr int    POSITIONS_PER_REV = 10400;
	static constexpr double POSITIONS_PER_RAD = ((double)POSITIONS_PER_REV) / (2.0 * M_PI); // Positive positions correspond to positive (counterclockwise) angles in the platform's frame of reference (aircraft convention)
	static constexpr double FORWARD_DIR_RAD = 45.0 * M_PI / 180.0; // Forward is at the 45 degree position in the LIDAR's frame of reference 
	static constexpr double SPIN_RATE_HZ = 10.0;
	static constexpr double FIRE_RATE_HZ = 53828.0;
	static constexpr double METERS_PER_RANGE_TICK = 1e-5; // 10 micrometers per tick
	
	// Configuration variables
	string rosFrameId;
	int lidarSpinupTimeS;
	bool logRawDataToFile;
	string folderName;
	string filenamePattern;
	unsigned int decimationRate = 1; // Keep 1 out of every N fires, where this is N
	// In platform reference frame (looking down onto the platform from the top of the arch)
	double minAngleToStoreRad; // we're only interested in points clockwise of here (0 is forward relative to the craft)
	double maxAngleToStoreRad; // we're only interested in points anticlockwise of here 
	unsigned int maxFiresPerPointCloud; // Computed at config time.

	// Benchmarking
	list<const Benchmarker *> * bms;
	Benchmarker bmHandlingIncomingData;
	Benchmarker bmProcessingM8Packet  ;
	Benchmarker bmStoringM8Data       ;
	
	Logger * logger;
	ofstream rawDataFile;
	ros::Publisher rosPublisher;

	bool processM8Packet(M8Packet * pkt); // Check a packet and call storage.  Return true if we should keep checking more packets this visit.
	void storeM8Data(const M8FiringData& fireData, double azimuth); // Store a vertical line in current point cloud
	void saveM8PktToDisk(M8Packet * pkt); // Used when logging input data.  Dump packet to disk.
	void publishPointCloud(); // We've finished a full sweep of our surroundings; begin the next point cloud
	void start(); // Opens a TCP connection to the LIDAR, which starts it spinning.
	void stop();  // Closes the TCP connection to the LIDAR, at which point it coasts to a stop
	void configureSocket();
	bool isSocketOpen();

public:
	LidarReader(list<const Benchmarker *> * _bms) :
		bms(_bms),
		bmHandlingIncomingData("Handling incoming LIDAR data"),
		bmProcessingM8Packet  ("Processing M8 packets"),
		bmStoringM8Data       ("Storing M8 data")
	{
		bms->push_back(&bmHandlingIncomingData);
		bms->push_back(&bmProcessingM8Packet  );
		bms->push_back(&bmStoringM8Data       );
	}
	virtual ~LidarReader();

	void visit(); // Handle everything
	bool isAnyoneListening();
	void init(nlohmann::json options, Logger * lgr, ros::NodeHandle * rosNodeHandle);
};


#endif // __PCG_LIDAR_READER_H__

