/****************************************************************************
 * 
 * Modified for Creare LLC by John Walthour (jdw@creare.com).
 * Original file was copied from:
 * https://github.com/mavlink/c_uart_interface_example.git @ 05072aa344483aa64d88f7cf70cafb713c589885
 * 
 * Original license is reproduced below.
 * 
 ****************************************************************************/

/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "AutopilotInterface.hpp"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					
					// Notify listeners
					cbv_heartbeat_t.fireCallbacks(current_messages.heartbeat);
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;

					// Notify listeners
					cbv_sys_status_t.fireCallbacks(current_messages.sys_status);
					break;
				}
				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					// Notify listeners
					cbv_battery_status_t.fireCallbacks(current_messages.battery_status);
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					// Notify listeners
					cbv_radio_status_t.fireCallbacks(current_messages.radio_status);
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					// Notify listeners
					cbv_local_position_ned_t.fireCallbacks(current_messages.local_position_ned);
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					// Notify listeners
					cbv_global_position_int_t.fireCallbacks(current_messages.global_position_int);
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					// Notify listeners
					cbv_position_target_local_ned_t.fireCallbacks(current_messages.position_target_local_ned);
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					// Notify listeners
					cbv_position_target_global_int_t.fireCallbacks(current_messages.position_target_global_int);
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					// Notify listeners
					cbv_highres_imu_t.fireCallbacks(current_messages.highres_imu);
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					// Notify listeners
					cbv_attitude_t.fireCallbacks(current_messages.attitude);
					break;
				}

				case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
				{
					mavlink_autopilot_version_t ver;
					mavlink_msg_autopilot_version_decode(&message, &ver);
					
					// Notify listeners
					cbv_autopilot_version_t.fireCallbacks(ver);
					break;
				}

				case MAVLINK_MSG_ID_COMMAND_ACK:
				{
					mavlink_command_ack_t ack;
					mavlink_msg_command_ack_decode(&message, &ack);
					// Notify listeners
					cbv_command_ack_t.fireCallbacks(ack);
					break;
				}

				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					mavlink_param_value_t paramValue;
					mavlink_msg_param_value_decode(&message, &paramValue);
					// Notify listeners
					cbv_param_value_t.fireCallbacks(paramValue);
					break;
				}

				case MAVLINK_MSG_ID_TIMESYNC:
				{
					mavlink_timesync_t timeSync;
					mavlink_msg_timesync_decode(&message, &timeSync);
					// Notify listeners
					cbv_timesync_t.fireCallbacks(timeSync);
					break;
				}

				default:
				{
					printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		//if ( writing_status > false ) {
		//	usleep(100); // look for components of batches at 10kHz
		//}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}

	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}



// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}








// ------------------------------------------------------------------------------
//  Message callback functions
// ------------------------------------------------------------------------------

void Autopilot_Interface::registerAnnouncementCallbacks() {
	
	cbReg_heartbeat_t                 (Autopilot_Interface::announce_heartbeat_t                  );
	cbReg_sys_status_t                (Autopilot_Interface::announce_sys_status_t                 );
	cbReg_battery_status_t            (Autopilot_Interface::announce_battery_status_t             );
	cbReg_radio_status_t              (Autopilot_Interface::announce_radio_status_t               );
	cbReg_local_position_ned_t        (Autopilot_Interface::announce_local_position_ned_t         );
	cbReg_global_position_int_t       (Autopilot_Interface::announce_global_position_int_t        );
	cbReg_position_target_local_ned_t (Autopilot_Interface::announce_position_target_local_ned_t  );
	cbReg_position_target_global_int_t(Autopilot_Interface::announce_position_target_global_int_t );
	cbReg_highres_imu_t               (Autopilot_Interface::announce_highres_imu_t                );
	cbReg_attitude_t                  (Autopilot_Interface::announce_attitude_t                   );
	cbReg_autopilot_version_t         (Autopilot_Interface::announce_autopilot_version_t          );
	cbReg_command_ack_t               (Autopilot_Interface::announce_command_ack_t                );
	cbReg_param_value_t               (Autopilot_Interface::announce_param_value_t                );
	cbReg_timesync_t                  (Autopilot_Interface::announce_timesync_t                   );
}

void Autopilot_Interface::announce_heartbeat_t                  (mavlink_heartbeat_t                 &) {
	printf("Received: MAVLINK_MSG_ID_HEARTBEAT\n");
}

void Autopilot_Interface::announce_sys_status_t                 (mavlink_sys_status_t                &) {
	printf("Received: MAVLINK_MSG_ID_SYS_STATUS\n");
}

void Autopilot_Interface::announce_battery_status_t             (mavlink_battery_status_t            &) {
	printf("Received: MAVLINK_MSG_ID_BATTERY_STATUS\n");
}

void Autopilot_Interface::announce_radio_status_t               (mavlink_radio_status_t              &) {
	printf("Received: MAVLINK_MSG_ID_RADIO_STATUS\n");
}

void Autopilot_Interface::announce_local_position_ned_t         (mavlink_local_position_ned_t        &) {
	printf("Received: MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
}

void Autopilot_Interface::announce_global_position_int_t        (mavlink_global_position_int_t       &) {
	printf("Received: MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
}

void Autopilot_Interface::announce_position_target_local_ned_t  (mavlink_position_target_local_ned_t &) {
	printf("Received: MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
}

void Autopilot_Interface::announce_position_target_global_int_t (mavlink_position_target_global_int_t&) {
	printf("Received: MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
}

void Autopilot_Interface::announce_highres_imu_t                (mavlink_highres_imu_t               &) {
	printf("Received: MAVLINK_MSG_ID_HIGHRES_IMU\n");
}

void Autopilot_Interface::announce_attitude_t                   (mavlink_attitude_t                  &) {
	printf("Received: MAVLINK_MSG_ID_ATTITUDE\n");
}

void Autopilot_Interface::announce_timesync_t                   (mavlink_timesync_t                  &) {
	printf("Received: MAVLINK_MSG_ID_TIMESYNC\n");
}

void Autopilot_Interface::announce_autopilot_version_t          (mavlink_autopilot_version_t         & ver) {
	if(ver.capabilities > 0) {
		printf("Autopilot capabilities: \n");
		
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT) { printf("Autopilot supports MISSION float message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT) { printf("Autopilot supports the new param float message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_INT) { printf("Autopilot supports MISSION_INT scaled integer message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_COMMAND_INT) { printf("Autopilot supports COMMAND_INT scaled integer message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_PARAM_UNION) { printf("Autopilot supports the new param union message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_FTP) { printf("Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET) { printf("Autopilot supports commanding attitude offboard.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED) { printf("Autopilot supports commanding position and velocity targets in local NED frame.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT) { printf("Autopilot supports commanding position and velocity targets in global scaled integers.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_TERRAIN) { printf("Autopilot supports terrain protocol / data handling.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET) { printf("Autopilot supports direct actuator control.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION) { printf("Autopilot supports the flight termination command.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION) { printf("Autopilot supports onboard compass calibration.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_MAVLINK2) { printf("Autopilot supports MAVLink version 2.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) { printf("Autopilot supports mission fence protocol.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_RALLY) { printf("Autopilot supports mission rally point protocol.\n"); }
		if(ver.capabilities & MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION) { printf("Autopilot supports the flight information protocol.\n"); }
	} else {
		printf("Autopilot reports no capabilities.\n");
	}
	printf("Autopilot UID: %lu\n", ver.uid);
	printf("Autopilot firmware version: %08X\n", ver.flight_sw_version);
}

void Autopilot_Interface::announce_command_ack_t                (mavlink_command_ack_t               &ack) {
	switch(ack.result) {
		case MAV_RESULT_ACCEPTED            : printf("Command ID %d ACCEPTED and EXECUTED\n", ack.command); break;  
		case MAV_RESULT_TEMPORARILY_REJECTED: printf("Command ID %d TEMPORARY REJECTED/DENIED\n", ack.command); break;
		case MAV_RESULT_DENIED              : printf("Command ID %d PERMANENTLY DENIED\n", ack.command); break;
		case MAV_RESULT_UNSUPPORTED         : printf("Command ID %d UNKNOWN/UNSUPPORTED\n", ack.command); break;
		case MAV_RESULT_FAILED              : printf("Command ID %d executed, but failed\n", ack.command); break;
		case MAV_RESULT_IN_PROGRESS         : printf("Command ID %d being executed\n", ack.command); break;
		default: printf("Command ID %d has unknown result %d.", ack.command, ack.result); break;
	}
}

void Autopilot_Interface::announce_param_value_t                (mavlink_param_value_t               &paramValue) {
	char nulltermParamId[17];
	nulltermParamId[16] = 0; // https://mavlink.io/en/messages/common.html#PARAM_VALUE
	memcpy(nulltermParamId, paramValue.param_id, 16);
	printf("Value of parameter %d (%s) is: %f\n", paramValue.param_index, nulltermParamId, paramValue.param_value);
}


/**
 * @brief Ask the autopilot to send us a specific stream of data
 * 
 * @param id Which stream to send
 * @param rate_Hz Desired rate of messages, in hertz
 */
void Autopilot_Interface::requestDataStream(MAV_DATA_STREAM id, int rate_Hz = 1) {
	mavlink_message_t message;
	mavlink_request_data_stream_t rds;
	memset(&rds, 0, sizeof(rds));
	rds.req_message_rate = rate_Hz; // Hz
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = id;
	rds.start_stop = 1; // start
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	write_message(message);
}

/**
 * @brief Stop a particular stream (or all streams)
 * 
 * @param id Which stream to stop
 */
void Autopilot_Interface::stopDataStream(MAV_DATA_STREAM id) {
	mavlink_message_t message;
	mavlink_request_data_stream_t rds;
	memset(&rds, 0, sizeof(rds));
	rds.target_system = 1;
	rds.target_component = 1;
	rds.req_stream_id = id;
	rds.start_stop = 0; // stop
	mavlink_msg_request_data_stream_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &rds);
	write_message(message);
}

/**
 * @brief Ask the autopilot to send us a message at a given interval
 * 
 * @param id The ID of the message to send to us
 * @param interval_us Microseconds between messages
 */
void Autopilot_Interface::requestMessage(uint id, double interval_us=0) {
	mavlink_message_t message;
	mavlink_command_long_t intvlReq;
	memset(&intvlReq, 0, sizeof(intvlReq));
	intvlReq.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	intvlReq.target_system = 1;
	intvlReq.target_component = 1;
	intvlReq.confirmation = 0;
	intvlReq.param1 = id ; /* The MAVLink message ID */
	intvlReq.param2 = interval_us; /* 	The interval between two messages. Set to -1 to disable and 0 to request default rate. */
	mavlink_msg_command_long_encode(/*uint8_t system_id*/ 1, /*uint8_t component_id*/0, &message, &intvlReq);
	write_message(message);
}
