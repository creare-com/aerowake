/*
	AutopilotLogger.cpp
	
	Class for logging data from an mavlink2-compatible autopilot
	
	2019-06-14	JDW	Created.
*/

#include <AutopilotLogger.hpp>

using namespace std;
/**
 * Constructor. 
 */
AutopilotLogger::AutopilotLogger(string recordingDir, string logFilenameFormat, string autopilotPort, int apBaudRate) :
	logger(recordingDir, logFilenameFormat),
	apSerialPort(autopilotPort.c_str(), apBaudRate),
	apIntf(&apSerialPort)
{ 
	// Set up log
	unsigned int logIdApTime      = logger.addColumn("ap_time_ms");
	unsigned int logIdRoll        = logger.addColumn("roll");
	unsigned int logIdPitch       = logger.addColumn("pitch");
	unsigned int logIdYaw         = logger.addColumn("yaw");
	unsigned int logIdRollSpeed   = logger.addColumn("roll_speed");
	unsigned int logIdPitchSpeed  = logger.addColumn("pitch_speed");
	unsigned int logIdYawSpeed    = logger.addColumn("yaw_speed");

	unsigned int logIdLat         = logger.addColumn("lat"); //[degE7] Latitude, expressed
	unsigned int logIdLon         = logger.addColumn("lon"); //[degE7] Longitude, expressed
	unsigned int logIdAlt         = logger.addColumn("alt"); //[mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	unsigned int logIdRelAlt      = logger.addColumn("relative_alt"); //[mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	unsigned int logIdVx          = logger.addColumn("vx"); //[cm/s] Ground X Speed (Latitude, positive north)
	unsigned int logIdVy          = logger.addColumn("vy"); //[cm/s] Ground Y Speed (Longitude, positive east)
	unsigned int logIdVz          = logger.addColumn("vz"); //[cm/s] Ground Z Speed (Altitude, positive down)
	unsigned int logIdHdg         = logger.addColumn("hdg"); //[cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	
	// Tell the autopilot interface to announce whenever a message arrives
	apIntf.registerAnnouncementCallbacks();
	
	// Use lambda functions to log whenever a message arrives.
	// This will happen on the reader thread.
	apIntf.cbReg_attitude_t(
		[
			this,
			logIdApTime    ,
			logIdRoll      ,
			logIdPitch     ,
			logIdYaw       ,
			logIdRollSpeed ,
			logIdPitchSpeed,
			logIdYawSpeed  
		](mavlink_attitude_t& att) {
		logger.logData(vector<CsvLogger::Cell>({
			CsvLogger::Cell(logIdApTime    , att.time_boot_ms),
			CsvLogger::Cell(logIdRoll      , att.roll        ),
			CsvLogger::Cell(logIdPitch     , att.pitch       ),
			CsvLogger::Cell(logIdYaw       , att.yaw         ),
			CsvLogger::Cell(logIdRollSpeed , att.rollspeed   ),
			CsvLogger::Cell(logIdPitchSpeed, att.pitchspeed  ),
			CsvLogger::Cell(logIdYawSpeed  , att.yawspeed    ),
		}));
	});

	apIntf.cbReg_global_position_int_t(
		[
			this,
			logIdApTime,
			logIdLat   ,
			logIdLon   ,
			logIdAlt   ,
			logIdRelAlt,
			logIdVx    ,
			logIdVy    ,
			logIdVz    ,
			logIdHdg    
		](mavlink_global_position_int_t& att) {
		logger.logData(vector<CsvLogger::Cell>({
			CsvLogger::Cell(logIdApTime, att.time_boot_ms),
			CsvLogger::Cell(logIdLat   , att.lat         ),
			CsvLogger::Cell(logIdLon   , att.lon         ),
			CsvLogger::Cell(logIdAlt   , att.alt         ),
			CsvLogger::Cell(logIdRelAlt, att.relative_alt),
			CsvLogger::Cell(logIdVx    , att.vx          ),
			CsvLogger::Cell(logIdVy    , att.vy          ),
			CsvLogger::Cell(logIdVz    , att.vz          ),
			CsvLogger::Cell(logIdHdg   , att.hdg         ),
		}));
	});

}

void AutopilotLogger::startLogging() {
	logger.startNewLogFile();
	
	apSerialPort.start();
	apIntf.start();
	
	cout << "Stopping data streams." << endl;
	apIntf.stopDataStream(MAV_DATA_STREAM_ALL);
	cout << "Sent." << endl;
	
	cout << "Sending data stream requests." << endl;
	apIntf.requestDataStream(MAV_DATA_STREAM_EXTRA1, 1);
	apIntf.requestDataStream(MAV_DATA_STREAM_POSITION, 1);
	// apIntf.requestDataStream(MAV_DATA_STREAM_RAW_SENSORS, 1); // This is supposed to get us GPS_RAW and GPS_STATUS messages.  It does not.
	cout << "Sent." << endl;
	
	cout << "Sending individual message rate requests." << endl;
	double messageRateHz = 1;
	apIntf.requestMessage(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 1000000.0 / messageRateHz);
	apIntf.requestMessage(MAVLINK_MSG_ID_GPS_RAW_INT, 1000000.0 / messageRateHz);
	apIntf.requestMessage(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 1000000.0 / messageRateHz);
	cout << "Sent." << endl;
}
void AutopilotLogger::stopLogging() {
	cout << "Stopping data streams." << endl;
	apIntf.stopDataStream(MAV_DATA_STREAM_ALL);
	cout << "Sent." << endl;

	cout << "Stopping individual message rate requests." << endl;
	apIntf.requestMessage(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, -1);
	cout << "Sent." << endl;

	// De-initialize autopilot connection
	apIntf.disable_offboard_control();
	apIntf.stop();
	apSerialPort.stop();
}

