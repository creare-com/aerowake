/*
	stereoCal.cpp
	
	Class for storing stereo calibration data
	
	2016-12-23  JDW  Created.
	2017-03-02  JDW  Moved into its own file.
*/
#include <stereoCal.h>

using namespace std;
using json = nlohmann::json;
using namespace cv;

void StereoCal::init(json options) {
	string cur_key = "";
	json subsection;
	json l_cam_mat, r_cam_mat;
	json l_r_mat, r_r_mat;
	json l_p_mat, r_p_mat;
	json l_dist_coefs, r_dist_coefs;
	json r_mat, t_mat;
	bool enable_gpu = true;
	// Pull JSON matrices from main file
	try {
		cur_key = "enableGpu";        enable_gpu   = options   [cur_key];
		
		cur_key = "leftCamera";       subsection   = options   [cur_key];
		cur_key = "rotationMatrix";   l_r_mat      = subsection[cur_key];
		cur_key = "projectionMatrix"; l_p_mat      = subsection[cur_key];
		cur_key = "cameraMatrix";     l_cam_mat    = subsection[cur_key];
		cur_key = "distCoeffs";       l_dist_coefs = subsection[cur_key];

		cur_key = "rightCamera";      subsection   = options   [cur_key];
		cur_key = "rotationMatrix";   r_r_mat      = subsection[cur_key];
		cur_key = "projectionMatrix"; r_p_mat      = subsection[cur_key];
		cur_key = "cameraMatrix";     r_cam_mat    = subsection[cur_key];
		cur_key = "distCoeffs";       r_dist_coefs = subsection[cur_key];
		
		cur_key = "stereo";           subsection   = options   [cur_key];
		cur_key = "R";                r_mat        = subsection[cur_key];
		cur_key = "T";                t_mat        = subsection[cur_key];
		cur_key = "imageWidth";       imageWidth   = subsection[cur_key];
		cur_key = "imageHeight";      imageHeight  = subsection[cur_key];
		cur_key = "focalLengthPx";    focalLen     = subsection[cur_key];
		cur_key = "baselineCm";       baselineCm   = subsection[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in calibration file: "
			 << e.what() << endl;
		throw(e);
	}
	
	// Pull out matrix contents into CV matrices
	try {
		cur_key = "Left cameraMatrix";      loadInto(leftCamMatrix  , l_cam_mat);
		cur_key = "Right cameraMatrix";     loadInto(rightCamMatrix , r_cam_mat);
		cur_key = "Left rotationMatrix";    loadInto(R1, l_r_mat);
		cur_key = "Right rotationMatrix";   loadInto(R2, r_r_mat);
		cur_key = "Left projectionMatrix";  loadInto(P1, l_p_mat);
		cur_key = "Right projectionMatrix"; loadInto(P2, r_p_mat);
		cur_key = "Left distCoeffs";        loadInto(leftDistCoeffs , l_dist_coefs);
		cur_key = "Right distCoeffs";       loadInto(rightDistCoeffs, r_dist_coefs);
		cur_key = "R matrix";               loadInto(R, r_mat);
		cur_key = "T matrix";               loadInto(T, t_mat);
	} catch (domain_error e) {
		cerr << "JSON calibration matrix invalid.  Please see example file in config directory."
			 << endl << "While reading matrix \"" << cur_key << "\" in calibration file: "
			 << e.what() << endl;
		throw(e);
	}
	
	// Compute derived parameters
	Size size(imageWidth, imageHeight);
	// cv::stereoRectify(leftCamMatrix,  leftDistCoeffs, 
	                  // rightCamMatrix, rightDistCoeffs, 
	                  // size, R, T, R1, R2, P1, P2, Q);
	cv::initUndistortRectifyMap(leftCamMatrix,  leftDistCoeffs, 
		R1, P1, size, CV_32FC1, cpuUndistortMapsLeft[0],  cpuUndistortMapsLeft[1]);
	cv::initUndistortRectifyMap(rightCamMatrix, rightDistCoeffs, 
		R2, P2, size, CV_32FC1, cpuUndistortMapsRight[0], cpuUndistortMapsRight[1]);
	triangulationConst = focalLen * baselineCm;

	// Upload undistort maps to GPU
	if(enable_gpu) {
		for(int i = 0; i < 2; i++) {
			gpuUndistortMapsLeft [i].upload(cpuUndistortMapsLeft [i]);	
			gpuUndistortMapsRight[i].upload(cpuUndistortMapsRight[i]);
		}
	}
}

void StereoCal::loadInto(Mat_<double> &cvMat, json jsonMat) {
	for(int row = 0; row < cvMat.rows; ++row) {
		for(int col = 0; col < cvMat.cols; ++col) {
			double cell = jsonMat[row][col];
			cvMat[row][col] = cell;
		}
	}
}

