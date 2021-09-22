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
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include "CallbackVector.hpp"
#include "SerialPort.hpp"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <atomic>
#include <functional>
#include <iostream>
#include <common/mavlink.h>

using namespace std;

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

												// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	atomic_char reading_status;
	char control_status;
	uint64_t write_count;

	void read_messages();
	int  write_message(mavlink_message_t message);

	void start();
	void stop();

	void start_read_thread();

	void handle_quit( int sig );

	// Call this once to announce all messages that arrive
	void registerAnnouncementCallbacks();
	
	// Use the functions below to register callbacks whenever a message is received.
	// Lambdas work well for these, but note that CallbackVector supports object member functions too.
	// Callbacks should return quickly, as they run on the message reader thread.
	void cbReg_heartbeat_t                 (function<void(mavlink_heartbeat_t                 &)> callback) { cbv_heartbeat_t                 .registerCallback(callback); }
	void cbReg_sys_status_t                (function<void(mavlink_sys_status_t                &)> callback) { cbv_sys_status_t                .registerCallback(callback); }
	void cbReg_battery_status_t            (function<void(mavlink_battery_status_t            &)> callback) { cbv_battery_status_t            .registerCallback(callback); }
	void cbReg_radio_status_t              (function<void(mavlink_radio_status_t              &)> callback) { cbv_radio_status_t              .registerCallback(callback); }
	void cbReg_local_position_ned_t        (function<void(mavlink_local_position_ned_t        &)> callback) { cbv_local_position_ned_t        .registerCallback(callback); }
	void cbReg_global_position_int_t       (function<void(mavlink_global_position_int_t       &)> callback) { cbv_global_position_int_t       .registerCallback(callback); }
	void cbReg_position_target_local_ned_t (function<void(mavlink_position_target_local_ned_t &)> callback) { cbv_position_target_local_ned_t .registerCallback(callback); }
	void cbReg_position_target_global_int_t(function<void(mavlink_position_target_global_int_t&)> callback) { cbv_position_target_global_int_t.registerCallback(callback); }
	void cbReg_highres_imu_t               (function<void(mavlink_highres_imu_t               &)> callback) { cbv_highres_imu_t               .registerCallback(callback); }
	void cbReg_attitude_t                  (function<void(mavlink_attitude_t                  &)> callback) { cbv_attitude_t                  .registerCallback(callback); }
	void cbReg_autopilot_version_t         (function<void(mavlink_autopilot_version_t         &)> callback) { cbv_autopilot_version_t         .registerCallback(callback); }
	void cbReg_command_ack_t               (function<void(mavlink_command_ack_t               &)> callback) { cbv_command_ack_t               .registerCallback(callback); }
	void cbReg_param_value_t               (function<void(mavlink_param_value_t               &)> callback) { cbv_param_value_t               .registerCallback(callback); }
	void cbReg_timesync_t                  (function<void(mavlink_timesync_t                  &)> callback) { cbv_timesync_t                  .registerCallback(callback); }
	void cbReg_gps_raw_int_t               (function<void(mavlink_gps_raw_int_t               &)> callback) { cbv_gps_raw_int_t               .registerCallback(callback); }
	
	/**
	 * @brief Ask the autopilot to send us a specific stream of data
	 * 
	 * @param id Which stream to send
	 * @param rate_Hz Desired rate of messages, in hertz
	 */
	void requestDataStream(MAV_DATA_STREAM id, int rate_Hz);
	/**
	 * @brief Stop a particular stream (or all streams)
	 * 
	 * @param id Which stream to stop
	 */
	void stopDataStream(MAV_DATA_STREAM id);
	/**
	 * @brief Ask the autopilot to send us a message at a given interval
	 * 
	 * @param id The ID of the message to send to us
	 * @param interval_us Microseconds between messages
	 */
	void requestMessage(uint id, double interval_us);

private:

	Serial_Port *serial_port;

	atomic_bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();

	// Callback vector for every supported message type
	CallbackVector<mavlink_heartbeat_t                 > cbv_heartbeat_t                 ;
	CallbackVector<mavlink_sys_status_t                > cbv_sys_status_t                ;
	CallbackVector<mavlink_battery_status_t            > cbv_battery_status_t            ;
	CallbackVector<mavlink_radio_status_t              > cbv_radio_status_t              ;
	CallbackVector<mavlink_local_position_ned_t        > cbv_local_position_ned_t        ;
	CallbackVector<mavlink_global_position_int_t       > cbv_global_position_int_t       ;
	CallbackVector<mavlink_position_target_local_ned_t > cbv_position_target_local_ned_t ;
	CallbackVector<mavlink_position_target_global_int_t> cbv_position_target_global_int_t;
	CallbackVector<mavlink_highres_imu_t               > cbv_highres_imu_t               ;
	CallbackVector<mavlink_attitude_t                  > cbv_attitude_t                  ;
	CallbackVector<mavlink_autopilot_version_t         > cbv_autopilot_version_t         ;
	CallbackVector<mavlink_command_ack_t               > cbv_command_ack_t               ;
	CallbackVector<mavlink_param_value_t               > cbv_param_value_t               ;
	CallbackVector<mavlink_timesync_t                  > cbv_timesync_t                  ;
	CallbackVector<mavlink_gps_raw_int_t               > cbv_gps_raw_int_t               ;

	static void announce_msg(string msg_name);
	static void announce_autopilot_version_t          (mavlink_autopilot_version_t         & ver);
	static void announce_command_ack_t                (mavlink_command_ack_t               &ack);
	static void announce_param_value_t                (mavlink_param_value_t               &paramValue);
	static void announce_timesync_t                   (mavlink_timesync_t                  &timeSync);
};



#endif // AUTOPILOT_INTERFACE_H_


