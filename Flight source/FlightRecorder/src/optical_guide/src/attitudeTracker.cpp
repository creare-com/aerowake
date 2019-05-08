/*
	attitudeTracker.cpp
	
	This class interfaces with the IMU (Inertial Measurement Unit) on the J120
	and performs all corrections required to report the platform's attitude.
	
	2017-05-16  JDW  Created.
*/

#include <attitudeTracker.h>
using json = nlohmann::json;
using namespace std;
using namespace chrono;

const unsigned int AttitudeTracker::MAG_AXIS_FOR_ACCEL_AXIS[3] = {1, 0, 2};
const int AttitudeTracker::MAG_AXIS_SIGN_FOR_ACCEL_AXIS[3] = {1, 1, -1};
const char AttitudeTracker::TIMESTAMP_FMT[] = "%Y-%m-%d %H:%M:%S";


void AttitudeTracker::init(json options, Logger * lgr) {
	logger = lgr;
	
	// Load configuration options
	string cur_key = "";
	string filename, data_path;
	try {
		cur_key = "imu";       json imu_section  = options[cur_key];
		cur_key = "enabled";   enabled           = options[cur_key];
		cur_key = "mag_cal";   json mag_cal      = imu_section[cur_key];
		for(int i = 0; i < 3; ++i) { magCal[i] = mag_cal[i]; }
		cur_key = "fwd_axis";       fwdAxis      = imu_section[cur_key];
		cur_key = "right_axis";     rightAxis    = imu_section[cur_key];
		cur_key = "down_axis";      downAxis     = imu_section[cur_key];
		cur_key = "fwd_axis_pol";   fwdAxisPol   = imu_section[cur_key];
		cur_key = "right_axis_pol"; rightAxisPol = imu_section[cur_key];
		cur_key = "down_axis_pol";  downAxisPol  = imu_section[cur_key];
		cur_key = "logDataToFile";   logDataToFile = options[cur_key];
		cur_key = "fileName";        filename      = options[cur_key];
		cur_key = "path";            data_path     = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in attitude tracker section: "
			 << e.what() << endl;
		throw(e);
		return;
	}
	
	if(enabled) {
		imu.init(logger);
	}

	
	// Set up log file
	if(logDataToFile) {
		dataFile.open(data_path + filename, ios::out);
		dataFile << "\"Time\","
			<< "\"Accel Fwd (G)\",\"Accel Right (G)\",\"Accel Down (G)\","
			<<  "\"Roll (deg)\",\"Pitch (deg)\",\"Heading (deg)\","
			<<  "\"Roll Rate (deg/s)\",\"Pitch Rate (deg/s)\",\"Yaw Rate (deg/s)\","
			<<  "\"Raw mag 0\",\"Raw mag 1\",\"Raw mag 2\","
			<< endl;
	}
	
}

Attitude AttitudeTracker::estimateAttitude() {
	Attitude att;
	
	if(enabled) {
		imu.read_all();
		
		// Read out acceleration and rotation rates
		att.rollRate   = imu.gyro_data [fwdAxis]   * fwdAxisPol   * M_PI / 180.0;
		att.pitchRate  = imu.gyro_data [rightAxis] * rightAxisPol * M_PI / 180.0;
		att.yawRate    = imu.gyro_data [downAxis]  * downAxisPol  * M_PI / 180.0;
		att.fwdAccel   = imu.accel_data[fwdAxis]   * fwdAxisPol;
		att.rightAccel = imu.accel_data[rightAxis] * rightAxisPol;
		att.downAccel  = imu.accel_data[downAxis]  * downAxisPol;
		
		
		// Read out and correct magnetometer reading.  Only saves direction; discards magnitude.
		// Alpha, Beta, and Gamma are relative to the magnetometer, not the platform.
		double corrected_mag_data[3];
		for(int i = 0; i < 3; ++i) {
			corrected_mag_data[i] = imu.mag_data[i] - magCal[i];
		}
		// Flip axes around to match the accelerometer and gyro
		double reoriented_mag_data[3];
		for(int i = 0; i < 3; ++i) {
			reoriented_mag_data[i] = corrected_mag_data[MAG_AXIS_FOR_ACCEL_AXIS[i]]
				* MAG_AXIS_SIGN_FOR_ACCEL_AXIS[i];
		}
		// stringstream imuSs;
		// imuSs << "Corrected mag X,Y,Z: (" << reoriented_mag_data[0] << ", " 
										  // << reoriented_mag_data[1] << ", " 
										  // << reoriented_mag_data[2] << ")";
		// logger->logInfo(imuSs.str());
		att.roll    = atan2(reoriented_mag_data[rightAxis] * rightAxisPol,
							reoriented_mag_data[fwdAxis]   * fwdAxisPol  );
		att.pitch   = atan2(reoriented_mag_data[downAxis]  * downAxisPol ,
							reoriented_mag_data[fwdAxis]   * fwdAxisPol  );
		att.heading = atan2(reoriented_mag_data[downAxis]  * downAxisPol ,
							reoriented_mag_data[rightAxis] * rightAxisPol);
		
		if(logDataToFile && dataFile.is_open()) {
			time_t now_s = system_clock::to_time_t(system_clock::now()); 
			dataFile << put_time(localtime(&now_s), TIMESTAMP_FMT) << ','
				<< att.fwdAccel    << ',' << att.rightAccel  << ',' << att.downAccel   << ','
				<< att.roll        << ',' << att.pitch       << ',' << att.heading     << ','
				<< att.rollRate    << ',' << att.pitchRate   << ',' << att.yawRate     << ','
				<< imu.mag_data[0] << ',' << imu.mag_data[1] << ',' << imu.mag_data[2] << ','
				<< endl;
		}
	}
	return att;
}