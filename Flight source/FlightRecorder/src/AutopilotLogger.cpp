/*
	AutopilotLogger.cpp
	
	Class for logging data from an mavlink2-compatible autopilot
	
	2019-06-14	JDW	Created.
*/

#include <AutopilotLogger.hpp>

using namespace std;

void AutopilotLogger::startLogging() {
	apIntf.registerAnnouncementCallbacks();
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
	rds.req_stream_id = MAV_DATA_STREAM_EXTENDED_STATUS;
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
	// De-initialize autopilot connection
	apIntf.disable_offboard_control();
	apIntf.stop();
	apSerialPort.stop();



}
