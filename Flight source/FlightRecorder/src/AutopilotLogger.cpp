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
	
	mavlink_message_t message;
	
	cout << "Sending heartbeat message" << endl;
	mavlink_heartbeat_t heartbeat;
	mavlink_msg_heartbeat_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &heartbeat);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;

	
	cout << "Sending param read message" << endl;
	mavlink_param_request_read_t paramRead;
	memset(&paramRead, 0, sizeof(paramRead));
	paramRead.param_index = 1;
	paramRead.target_system = 1;
	paramRead.target_component = 1;
	mavlink_msg_param_request_read_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &paramRead);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;

	
	mavlink_command_long_t testCmd;
	memset(&testCmd, 0, sizeof(testCmd));
	testCmd.command = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ;
	testCmd.target_system = 1;
	testCmd.target_component = 1;
	testCmd.confirmation = 0;
	testCmd.param1 = 1;
	cout << "Sending test command with ID " << testCmd.command << "." << endl;
	mavlink_msg_command_long_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/42, &message, &testCmd);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
	mavlink_request_data_stream_t rds;
	memset(&rds, 0, sizeof(rds));
	rds.req_message_rate = 10; // Hz
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = MAV_DATA_STREAM_ALL;
	rds.start_stop = 0; // stop
	cout << "Stopping data streams." << endl;
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
	memset(&rds, 0, sizeof(rds));
	rds.req_message_rate = 1; // Hz
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = MAV_DATA_STREAM_EXTRA1;
	rds.start_stop = 1; // start
	cout << "Sending data stream request." << endl;
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
	memset(&rds, 0, sizeof(rds));
	rds.req_message_rate = 1; // Hz
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = MAV_DATA_STREAM_POSITION;
	rds.start_stop = 1; // start
	cout << "Sending data stream request." << endl;
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;

	double messageRateHz = 1;
	// mavlink_command_int_t intvlReq;
	// memset(&intvlReq, 0, sizeof(intvlReq));
	// intvlReq.target_system = 1;
	// intvlReq.target_component = 1;
	// intvlReq.frame = MAV_FRAME_GLOBAL;
	// intvlReq.current = 1;
	// intvlReq.autocontinue = 0;
	mavlink_command_long_t intvlReq;
	memset(&intvlReq, 0, sizeof(intvlReq));
	intvlReq.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	// intvlReq.command = MAV_CMD_REQUEST_MESSAGE ;
	intvlReq.target_system = 1;
	intvlReq.target_component = 1;
	intvlReq.confirmation = 0;
	intvlReq.param1 = MAVLINK_MSG_ID_ATTITUDE_QUATERNION ; /* The MAVLink message ID */
	// intvlReq.param1 = MAVLINK_MSG_ID_RAW_IMU; /* The MAVLink message ID */
	// intvlReq.param1 = MAVLINK_MSG_ID_HEARTBEAT; /* The MAVLink message ID */
	// intvlReq.param2 = 1000000 / messageRateHz; /* 	The interval between two messages. Set to -1 to disable and 0 to request default rate. */
	intvlReq.param2 = 0; /* 	The interval between two messages. Set to -1 to disable and 0 to request default rate. */
	cout << "Requesting message ID " << intvlReq.param1 << " every " << intvlReq.param2 << "us." << endl;
	// mavlink_msg_command_int_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &intvlReq);
	mavlink_msg_command_long_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &intvlReq);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
	memset(&intvlReq, 0, sizeof(intvlReq));
	// intvlReq.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	intvlReq.command = MAV_CMD_REQUEST_MESSAGE ;
	intvlReq.target_system = 1;
	intvlReq.target_component = 1;
	intvlReq.confirmation = 0;
	intvlReq.param1 = MAVLINK_MSG_ID_ATTITUDE_QUATERNION ; /* The MAVLink message ID */
	// intvlReq.param1 = MAVLINK_MSG_ID_RAW_IMU; /* The MAVLink message ID */
	// intvlReq.param1 = MAVLINK_MSG_ID_HEARTBEAT; /* The MAVLink message ID */
	cout << "Requesting message ID " << intvlReq.param1 << ", with index id " << intvlReq.param2 << "." << endl;
	// mavlink_msg_command_int_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &intvlReq);
	mavlink_msg_command_long_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &intvlReq);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;
	
}
void AutopilotLogger::stopLogging() {
	mavlink_request_data_stream_t rds;
	memset(&rds, 0, sizeof(rds));
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = MAV_DATA_STREAM_ALL;
	rds.start_stop = 0; // stop
	cout << "Stopping data streams." << endl;
	mavlink_message_t message;
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	apSerialPort.write_message(message);
	cout << "Sent." << endl;

	// De-initialize autopilot connection
	apIntf.disable_offboard_control();
	apIntf.stop();
	apSerialPort.stop();
}
