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
	// ATTITUDE ( #30 )
	uint logIdApTime      = logger.addColumn("ap_time_ms");
	uint logIdRoll        = logger.addColumn("roll");
	uint logIdPitch       = logger.addColumn("pitch");
	uint logIdYaw         = logger.addColumn("yaw");
	uint logIdRollSpeed   = logger.addColumn("roll_speed");
	uint logIdPitchSpeed  = logger.addColumn("pitch_speed");
	uint logIdYawSpeed    = logger.addColumn("yaw_speed");

	// GLOBAL_POSITION_INT ( #33 ) https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
	uint logIdLat         = logger.addColumn("global_lat"); //[degE7] Latitude, expressed
	uint logIdLon         = logger.addColumn("global_lon"); //[degE7] Longitude, expressed
	uint logIdAlt         = logger.addColumn("global_alt"); //[mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	uint logIdRelAlt      = logger.addColumn("global_relative_alt"); //[mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	uint logIdVx          = logger.addColumn("global_vx"); //[cm/s] Ground X Speed (Latitude, positive north)
	uint logIdVy          = logger.addColumn("global_vy"); //[cm/s] Ground Y Speed (Longitude, positive east)
	uint logIdVz          = logger.addColumn("global_vz"); //[cm/s] Ground Z Speed (Altitude, positive down)
	uint logIdHdg         = logger.addColumn("global_hdg"); //[cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	
	// GPS_RAW_INT ( #24 )
	uint logIdRawTimeUsec     = logger.addColumn("raw_time_usec"); // [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
	uint logIdRawLat          = logger.addColumn("raw_lat"); // [degE7] Latitude (WGS84, EGM96 ellipsoid)
	uint logIdRawLon          = logger.addColumn("raw_lon"); // [degE7] Longitude (WGS84, EGM96 ellipsoid)
	uint logIdRawAlt          = logger.addColumn("raw_alt"); // [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
	uint logIdRawEph          = logger.addColumn("raw_eph"); //  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	uint logIdRawEpv          = logger.addColumn("raw_epv"); //  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	uint logIdRawVel          = logger.addColumn("raw_vel"); // [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
	uint logIdRawCog          = logger.addColumn("raw_cog"); // [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	uint logIdRawFixType      = logger.addColumn("raw_fix_type"); // GPS_FIX_TYPE	GPS fix type.
	uint logIdRawSatsVisible  = logger.addColumn("raw_satellites_visible"); // Number of satellites visible. If unknown, set to 255
	uint logIdRawAltEllipsoid = logger.addColumn("raw_alt_ellipsoid"); //  [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
	uint logIdRawHAcc         = logger.addColumn("raw_h_acc"); // [mm] Position uncertainty. Positive for up.
	uint logIdRawVAcc         = logger.addColumn("raw_v_acc"); // [mm] Altitude uncertainty. Positive for up.
	uint logIdRawVelAcc       = logger.addColumn("raw_vel_acc"); // [mm] Speed uncertainty. Positive for up
	uint logIdRawHdgAcc       = logger.addColumn("raw_hdg_acc"); // [degE5] Heading / track uncertainty

	// LOCAL_POSITION_NED ( #32 )
	uint logIdPosNedTimeMs     = logger.addColumn("pos_ned_time_boot_ms");     // [ms] Timestamp (time since system boot).
	uint logIdPosNedX          = logger.addColumn("pos_ned_x");                // [m] X Position in NED frame*/
	uint logIdPosNedY          = logger.addColumn("pos_ned_y");                // [m] Y Position in NED frame*/
	uint logIdPosNedZ          = logger.addColumn("pos_ned_z");                // [m] Z Position in NED frame (note, altitude is negative in NED)
	uint logIdPosNedVX         = logger.addColumn("pos_ned_vx");               // [m/s] X velocity in NED frame
	uint logIdPosNedVY         = logger.addColumn("pos_ned_vy");               // [m/s] Y velocity in NED frame
	uint logIdPosNedVZ         = logger.addColumn("pos_ned_vz");               // [m/s] Z velocity in NED frame
	// uint logIdPosNedAFX        = logger.addColumn("pos_ned_afx");              // [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	// uint logIdPosNedAFY        = logger.addColumn("pos_ned_afy");              // [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	// uint logIdPosNedAFZ        = logger.addColumn("pos_ned_afz");              // [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	// uint logIdPosNedYaw        = logger.addColumn("pos_ned_yaw");              // [rad] yaw setpoint
	// uint logIdPosNedYawRate    = logger.addColumn("pos_ned_yaw_rate");         // [rad/s] yaw rate setpoint
	// uint logIdPosNedTypeMask   = logger.addColumn("pos_ned_type_mask");        // Bitmap to indicate which dimensions should be ignored by the vehicle.
	// uint logIdPosNedCoordFrame = logger.addColumn("pos_ned_coordinate_frame"); // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9


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
		](mavlink_global_position_int_t& pos) {
		logger.logData(vector<CsvLogger::Cell>({
			CsvLogger::Cell(logIdApTime, pos.time_boot_ms),
			CsvLogger::Cell(logIdLat   , pos.lat         ),
			CsvLogger::Cell(logIdLon   , pos.lon         ),
			CsvLogger::Cell(logIdAlt   , pos.alt         ),
			CsvLogger::Cell(logIdRelAlt, pos.relative_alt),
			CsvLogger::Cell(logIdVx    , pos.vx          ),
			CsvLogger::Cell(logIdVy    , pos.vy          ),
			CsvLogger::Cell(logIdVz    , pos.vz          ),
			CsvLogger::Cell(logIdHdg   , pos.hdg         ),
		}));
	});

	apIntf.cbReg_gps_raw_int_t(
		[
			this,
			logIdRawTimeUsec    ,
			logIdRawLat         ,
			logIdRawLon         ,
			logIdRawAlt         ,
			logIdRawEph         ,
			logIdRawEpv         ,
			logIdRawVel         ,
			logIdRawCog         ,
			logIdRawFixType     ,
			logIdRawSatsVisible ,
			logIdRawAltEllipsoid,
			logIdRawHAcc        ,
			logIdRawVAcc        ,
			logIdRawVelAcc      ,
			logIdRawHdgAcc      
		](mavlink_gps_raw_int_t& msg) {
		logger.logData(vector<CsvLogger::Cell>({
			CsvLogger::Cell(logIdRawTimeUsec    , msg.time_usec),
			CsvLogger::Cell(logIdRawLat         , msg.lat),
			CsvLogger::Cell(logIdRawLon         , msg.lon),
			CsvLogger::Cell(logIdRawAlt         , msg.alt),
			CsvLogger::Cell(logIdRawEph         , msg.eph),
			CsvLogger::Cell(logIdRawEpv         , msg.epv),
			CsvLogger::Cell(logIdRawVel         , msg.vel),
			CsvLogger::Cell(logIdRawCog         , msg.cog),
			CsvLogger::Cell(logIdRawFixType     , msg.fix_type),
			CsvLogger::Cell(logIdRawSatsVisible , msg.satellites_visible),
			CsvLogger::Cell(logIdRawAltEllipsoid, msg.alt_ellipsoid),
			CsvLogger::Cell(logIdRawHAcc        , msg.h_acc),
			CsvLogger::Cell(logIdRawVAcc        , msg.v_acc),
			CsvLogger::Cell(logIdRawVelAcc      , msg.vel_acc),
			CsvLogger::Cell(logIdRawHdgAcc      , msg.hdg_acc),
		}));
	});


	apIntf.cbReg_local_position_ned_t(
		[
			this,
			logIdPosNedTimeMs    ,
			logIdPosNedX         ,
			logIdPosNedY         ,
			logIdPosNedZ         ,
			logIdPosNedVX        ,
			logIdPosNedVY        ,
			logIdPosNedVZ        //,
			// logIdPosNedAFX       ,
			// logIdPosNedAFY       ,
			// logIdPosNedAFZ       ,
			// logIdPosNedYaw       ,
			// logIdPosNedYawRate   ,
			// logIdPosNedTypeMask  ,
			// logIdPosNedCoordFrame
		](mavlink_local_position_ned_t& msg) {
		logger.logData(vector<CsvLogger::Cell>({
			CsvLogger::Cell(logIdPosNedTimeMs    , msg.time_boot_ms),
			CsvLogger::Cell(logIdPosNedX         , msg.x),
			CsvLogger::Cell(logIdPosNedY         , msg.y),
			CsvLogger::Cell(logIdPosNedZ         , msg.z),
			CsvLogger::Cell(logIdPosNedVX        , msg.vx),
			CsvLogger::Cell(logIdPosNedVY        , msg.vy),
			CsvLogger::Cell(logIdPosNedVZ        , msg.vz),
			// CsvLogger::Cell(logIdPosNedAFX       , msg.),
			// CsvLogger::Cell(logIdPosNedAFY       , msg.),
			// CsvLogger::Cell(logIdPosNedAFZ       , msg.),
			// CsvLogger::Cell(logIdPosNedYaw       , msg.),
			// CsvLogger::Cell(logIdPosNedYawRate   , msg.),
			// CsvLogger::Cell(logIdPosNedTypeMask  , msg.),
			// CsvLogger::Cell(logIdPosNedCoordFrame, msg.),
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
	apIntf.stop();
	apSerialPort.stop();
}

