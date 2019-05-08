/*
	lidarReader.cpp

	Definitions for reading and packaging the LIDAR data

	2017-06-22  JDW  Created.
*/

#include <lidarReader.h>
using namespace std;
using namespace chrono;

LidarReader::~LidarReader() {
	delete currentPointCloud; // Importantly, this calls sensor_msgs::~PointCloud(), which destructs the points vector, releasing the bulk of our heap space.
}

void LidarReader::init(json options, Logger * lgr, ros::NodeHandle * rosNodeHandle) {
	logger = lgr;

	// Load configuration options
	string curKey = "";
	string lidarIpAddr;
	uint16_t lidarPort;
	string rosTopic;
	int rosQueueSize;
	try {
		curKey = "rosTopic";                rosTopic                = options[curKey];
		curKey = "rosFrameId";              rosFrameId              = options[curKey];
		curKey = "rosQueueSize";            rosQueueSize            = options[curKey];
		curKey = "lidarIpAddr";             lidarIpAddr             = options[curKey];
		curKey = "lidarPort";               lidarPort               = options[curKey];
		curKey = "lidarSpinupTimeS";        lidarSpinupTimeS        = options[curKey];
		curKey = "logRawDataToFile";        logRawDataToFile        = options[curKey];
		curKey = "path";                    folderName              = options[curKey];
		curKey = "filenamePattern";         filenamePattern         = options[curKey];
		curKey = "minAngleToStoreRad";      minAngleToStoreRad      = options[curKey];
		curKey = "maxAngleToStoreRad";      maxAngleToStoreRad      = options[curKey];
		curKey = "decimationRate";          decimationRate          = options[curKey];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << curKey << "\" in acquisition section: "
			 << e.what() << endl;
		throw(e);
		return;
	}

	// Try to parse the IP address and port
	memset(&lidarSocketAddress, 0, sizeof(lidarSocketAddress));
	lidarSocketAddress.sin_family = AF_INET;
	lidarSocketAddress.sin_port = htons(lidarPort);
	int ipParseStatus = inet_aton(lidarIpAddr.c_str(), &(lidarSocketAddress.sin_addr));
	if(ipParseStatus != 1) {
		logger->logError("Could not translate LIDAR IP address: " + lidarIpAddr);
	}

	// Allocate space to store point clouds.
	// In this iteration of the downsampling algorithm, we know the
	// maximum number of points in advance, so there is no need to
	// free and reallocate these during processing.
	int numVerticalLinesToStore = (FIRE_RATE_HZ / SPIN_RATE_HZ) / (double)(decimationRate);
	maxFiresPerPointCloud = numVerticalLinesToStore * M8_NUM_BEAMS;
	nextPointNumToStore = 0;
	lastSeenAzRad = -M_PI_2;

	if(decimationRate > M8_NUM_VERT_LINES_PER_PKT
		|| decimationRate < 1) {
		stringstream warnSs;
		warnSs << "decimationRate must be between 1 and " <<
			M8_NUM_VERT_LINES_PER_PKT << ".  Correcting from " << decimationRate;
		if(decimationRate < 1) { decimationRate = 1; }
		else if (decimationRate > M8_NUM_VERT_LINES_PER_PKT)
		{ decimationRate = M8_NUM_VERT_LINES_PER_PKT; }
		warnSs << " to " << decimationRate << ".";
		logger->logWarning(warnSs.str());
	}

	rosPublisher = rosNodeHandle->advertise<sensor_msgs::PointCloud>(rosTopic, rosQueueSize);
	currentPointCloud = new sensor_msgs::PointCloud();
}


// Opens a TCP connection to the LIDAR, which starts it spinning.
void LidarReader::start() {
	if (running) {
		logger->logDebug("Tried to start an already-started LIDAR.");
		return;
	}

	logger->logInfo("Starting LIDAR");
	sockLidar = socket(AF_INET, SOCK_STREAM, 0); // Creating the socket object does not open the socket
	fcntl(sockLidar, F_SETFL, O_NONBLOCK); // Set socket to nonblocking
	if (sockLidar >= 0) {
		// This will immediately return, regardless of success, since it's nonblocking.
		int connectStatus = connect(sockLidar, (struct sockaddr *)&lidarSocketAddress,sizeof(lidarSocketAddress));
		if(errno == EINPROGRESS) {
			// Will get here immediately; wait with a timeout to see if it got a connection to the remote.
			fd_set readFds;
			struct timeval timeout;
			FD_ZERO(&readFds);
			FD_SET(sockLidar, &readFds);
			timeout.tv_sec = 30;
			timeout.tv_usec = 0;

			int numConnections = select(sockLidar + 1, &readFds, NULL, NULL, &timeout);
			if(numConnections > 0) {

				if(isSocketOpen()) {
					logger->logInfo("Opened connection to LIDAR.");
					running = true;
					configureSocket();
				} else {
					logger->logInfo("LIDAR rejected connection.");
					close(sockLidar);
				}
			} else {
				logger->logError("Timeout connecting to LIDAR.");
				close(sockLidar);
			}

			// Open log file
			if(logRawDataToFile) {
				stringstream path;
				path.str(""); path.clear();
				time_t now_s = system_clock::to_time_t(system_clock::now());
				path << folderName <<
					put_time(localtime(&now_s), filenamePattern.c_str());
				rawDataFile.open(path.str(), ios::out | ofstream::app);
			}

			return;
		} else {
			switch(errno) {
				case EACCES       : logger->logError("LIDAR: socket access denied"); break;
				case EADDRINUSE   : logger->logError("LIDAR: local address in use."); break;
				case EADDRNOTAVAIL: logger->logError("LIDAR: EADDRNOTAVAIL "); break;
				case EAFNOSUPPORT : logger->logError("LIDAR: address family unsupported"); break;
				case EALREADY     : logger->logError("LIDAR: EALREADY     "); break;
				case EBADF        : logger->logError("LIDAR: EBADF        "); break;
				case ECONNREFUSED : logger->logError("LIDAR: connection refused"); break;
				case EFAULT       : logger->logError("LIDAR: EFAULT"); break;
				// case EINPROGRESS  : logger->logError("LIDAR: EINPROGRESS"); break;
				case EINTR        : logger->logError("LIDAR: EINTR"); break;
				case EISCONN      : logger->logError("LIDAR: socket already connected"); break;
				case ENETUNREACH  : logger->logError("LIDAR: network unreachable"); break;
				case ENOTSOCK     : logger->logError("LIDAR: ENOTSOCK"); break;
				case EPROTOTYPE   : logger->logError("LIDAR: EPROTOTYPE"); break;
				case ETIMEDOUT    : logger->logError("LIDAR: timeout while attempting connection."); break;
				default:            logger->logError("LIDAR: Error while attempting connection."); break;
			}
		}
	} else {
		logger->logError("Could not open LIDAR socket");
	}

	running = false;
}

// Closes the TCP connection to the LIDAR, at which point it coasts to a stop
void LidarReader::stop() {
	if (!running) {
		logger->logDebug("Tried to stop an already-stopped LIDAR.");
		return;
	}
	rawDataFile.close();

	logger->logInfo("Stopping LIDAR");
	close(sockLidar);

	running = false;
}

// Set up the socket with our various options
void LidarReader::configureSocket() {
	struct timeval timeout;
	timeout.tv_sec = 30;
	timeout.tv_usec = 0;

	// Set send data timeout, not to be confused with the connect timeout.
	// This is the timeout associated with the keep-alive packets.
	if(setsockopt(sockLidar, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
		logger->logError("Couldn't set socket transmit timeout.");
	}
	// Configure the connection to use keep-alive packets.
	int keepAlive = true;
	if(setsockopt(sockLidar, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(keepAlive)) < 0) {
		logger->logError("Couldn't set socket keepalive.");
	}
	int kaIdle = 30;
	int kaCount = 1;
	int kaInterval = 30;
	if(setsockopt(sockLidar, IPPROTO_TCP, TCP_KEEPIDLE, &kaIdle, sizeof(kaIdle)) < 0) {
		logger->logError("Couldn't configure TCP keep-alive.");
	}
	if(setsockopt(sockLidar, IPPROTO_TCP, TCP_KEEPCNT, &kaCount, sizeof(kaCount)) < 0) {
		logger->logError("Couldn't configure TCP keep-alive.");
	}
	if(setsockopt(sockLidar, IPPROTO_TCP, TCP_KEEPINTVL, &kaInterval, sizeof(kaInterval)) < 0) {
		logger->logError("Couldn't configure TCP keep-alive.");
	}
	// Set receive data timeout, not to be confused with the connect timeout.
	// The receive timeout is used only in blocking reads, so we don't need it
	// with this implementation; we set it to prevent future confusion.
	if(setsockopt(sockLidar, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
		logger->logError("Couldn't set socket receive timeout.");
	}
}

bool LidarReader::isSocketOpen() {
	int socketError;
	socklen_t len = sizeof(socketError);
	getsockopt(sockLidar, SOL_SOCKET, SO_ERROR, &socketError, &len);

	return socketError == 0;
}

// Checks for data in the TCP buffer.  Accumulates point cloud data.
// Transmits every time a complete revolution is received.
void LidarReader::visit() {
	if (!running) {
		if(isAnyoneListening()) {
			start();
		} else {
			return; // Don't process any more incoming data
		}
	} else {
		if(!isAnyoneListening()) {
			stop();
			// Send out anything we've already accumulated
			publishPointCloud();
			return;  // Don't process any more incoming data
		}
	}

	if(!isSocketOpen()) {
		logger->logError("Connection to LIDAR lost.");
		stop();
		// Send out anything we've already accumulated
		publishPointCloud();
		return;  // Don't process any more incoming data
	}

	bmHandlingIncomingData.start();
	int bytes_avail = 0;
	ioctl(sockLidar, FIONREAD, &bytes_avail);
	int packets_avail = bytes_avail / (M8PacketHeader::EXPECTED_SIZE_B); // intentional integer division
	// stringstream ss;
	// ss << "LIDAR has delivered " << packets_avail << " frames (" << bytes_avail << "B).";
	// logger->logDebug(ss.str());
	if(packets_avail > 0) {
		bool keep_processing = true;
		// int num_skipped = 0;
		for(int i = 0; i < packets_avail; ++i) {
			int ret_val = read(sockLidar, recvBuffer, M8PacketHeader::EXPECTED_SIZE_B);
			if(ret_val < 0) {
				stringstream ssErr;
				ssErr << "Got error " << ret_val << " while attempting to read from LIDAR socket.";
				logger->logError(ssErr.str());
				stop();
				break;
			} else if(ret_val < (int)(M8PacketHeader::EXPECTED_SIZE_B)) {
				stringstream ssErr;
				ssErr << "Only got " << ret_val << "B while attempting to read from LIDAR socket.";
				logger->logError(ssErr.str());
				break;
			} else { // ret_val >= M8PacketHeader::EXPECTED_SIZE_B
				// We have a full packet of data waiting in recvBuffer
				if(keep_processing) {
					bmProcessingM8Packet.start();
					keep_processing = processM8Packet((M8Packet*)(recvBuffer));
					// if(!keep_processing) {
						// num_skipped = packets_avail - i - 1;
					// }
					bmProcessingM8Packet.end();
				} else {
					// Drain the buffer by reading repeatedly
				}
			}
		}
	// This is an example of something that doesn't work.
	// When you're trying to trigger a receive timeout, the read operation
	// must be blocking.
	// } else {
		// logger->logDebug("Trying nonblocking read...");
		// int ret_val = read(sockLidar, recvBuffer, 0);
		// logger->logDebug("Done; got " + to_string(ret_val) + " bytes.");
	}
	bmHandlingIncomingData.end();

}

bool LidarReader::processM8Packet(M8Packet * pkt) {
	bool keep_examining_buf = true;

	// Confirm size and signature match
	if(pkt->header.isMessageValid()) {
		if (logRawDataToFile) {
			saveM8PktToDisk(pkt);
		}

		// The LIDAR returns a lot of junky samples before fully spun up.  Ignore them.
		if(pkt->header.getTimestampS() >= (unsigned)lidarSpinupTimeS) {
			// Iterate through the firings (vertical slices) in this message
			unsigned int i;
			for(i = nextPktStartIdx; i < M8_NUM_VERT_LINES_PER_PKT; i += decimationRate) {
				double pos_rad = (pkt->data.firingData[i].getPosition()) / POSITIONS_PER_RAD;
				pos_rad -= FORWARD_DIR_RAD;

				// Normalize to a range between -pi and +pi
				if(pos_rad >  M_PI) { pos_rad -= 2.0*M_PI; }
				if(pos_rad < -M_PI) { pos_rad += 2.0*M_PI; }

				// Simple azimuth gating
				if(    pos_rad > minAngleToStoreRad
					&& pos_rad < maxAngleToStoreRad) {
					bmStoringM8Data.start();
					storeM8Data(pkt->data.firingData[i], pos_rad);
					bmStoringM8Data.end();
				}

				// Check if we've scanned behind the craft, signaling the time to
				// begin a new point cloud
				if(pos_rad < 0 && lastSeenAzRad > 0) {
					publishPointCloud();
				}
				lastSeenAzRad = pos_rad;
			}
			// Maintain decimation across packet boundaries
			nextPktStartIdx = ((i-1) + decimationRate) % M8_NUM_VERT_LINES_PER_PKT;
		} else {
			stringstream ssDbg;
			ssDbg << "LIDAR spinning up for " << (lidarSpinupTimeS - pkt->header.getTimestampS()) <<" more seconds.";
			logger->logInfo(ssDbg.str());
			keep_examining_buf = false;
		}
	} else {
		stringstream ssErr;
		ssErr << "Received invalid message from LIDAR - size = "
			<< pkt->header.getMessageSizeB() << ", signature = "
			<< pkt->header.getPacketSignature() << ".";
		logger->logError(ssErr.str());
		stop();
		keep_examining_buf = false;
	}
	return keep_examining_buf;
}

void LidarReader::storeM8Data(const M8FiringData& fireData, double azimuth) {
	for(unsigned int i = 0; i < M8_NUM_BEAMS && nextPointNumToStore < maxFiresPerPointCloud; ++i) {
		double range_m = fireData.getReturnDistance(i) * METERS_PER_RANGE_TICK;
		// The LIDAR often describes points with zero range.  These are not useful.
		if(range_m > 0.0) {
			geometry_msgs::Point32 pt;

			double elevation = M_PI_2 - M8_VERTICAL_ANGLES_RAD[i];
			pt.x = (range_m * sin(elevation) * cos(azimuth));
			pt.y = (range_m * sin(elevation) * sin(azimuth));
			pt.z = (range_m * cos(elevation));

			currentPointCloud->points.push_back(pt);
			nextPointNumToStore++;
		}
	}
}

void LidarReader::saveM8PktToDisk(M8Packet * pkt) {
	if(rawDataFile.is_open()) {
		rawDataFile.write((const char*)(pkt), pkt->header.getMessageSizeB());
		// Intentionally do not flush
	} else {
		logger->logError("LIDAR raw data output file not open.");
	}
}

void LidarReader::publishPointCloud() {
	currentPointCloud->header.seq = seqNum++;
	currentPointCloud->header.frame_id = rosFrameId;
	// Acquisition time is a little hard to define for the LIDAR.
	// We receive data for each "frame" during the span of time it takes to
	// turn a full revolution (about 100ms).  Arbitrarily, we define acquisition
	// time as the end of that.
	currentPointCloud->header.stamp = ros::Time::now();

	logger->logDebug("Sending " + to_string(currentPointCloud->points.size()) + " LIDAR points.");
	rosPublisher.publish(*currentPointCloud);

	delete currentPointCloud; // Importantly, this calls sensor_msgs::~PointCloud(), which destructs the points vector, releasing the bulk of our heap space.
	currentPointCloud = new sensor_msgs::PointCloud();
	nextPointNumToStore = 0;

}

bool LidarReader::isAnyoneListening() {
	return rosPublisher.getNumSubscribers() > 0;
}
