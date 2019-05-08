/*
	stereoCal.h
	
	Class for storing stereo calibration data
	
	2016-12-23  JDW  Created.
	2017-03-02  JDW  Moved into its own file.
*/
#ifndef __PCG_STERCAL_H__
#define __PCG_STERCAL_H__

#include "json.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>




class StereoCal {
private:
	static const int CAM_MAT_ROWS=3, CAM_MAT_COLS=3;
	static const int DIST_COEFFS=5;
	static const int R_MAT_ROWS=3, R_MAT_COLS=3;
	static const int P_MAT_ROWS=3, P_MAT_COLS=4;
	static const int T_MAT_ROWS=3, T_MAT_COLS=1;
	static const int Q_MAT_ROWS=4, Q_MAT_COLS=1;
	
	// Loaded from file
	unsigned int imageWidth  = 0;
	unsigned int imageHeight = 0;
	double focalLen  ; // In "pixel units"
	double baselineCm;
	double triangulationConst;
	
	// Loaded from file - computed with:
	// https://code.crearecomputing.com/LaserMetrology/LaserMetrologyCommon/blob/develop/PythonCommon/laser_metrology_toolbox/laser_metrology_toolbox/calibration/examples/stereo_example2.py
	// It would be cleaner to use arrays of matrices, but the default constructors
	// don't work as desired here, so we make them individual items.
	cv::Mat_<double> leftCamMatrix ;
	cv::Mat_<double> rightCamMatrix;
	cv::Mat_<double> leftDistCoeffs, rightDistCoeffs;
	cv::Mat_<double> R;
	cv::Mat_<double> T;
	
	// Computed from calibration file once just after loading
	cv::Mat_<double> R1; // Left  output rectification transform
	cv::Mat_<double> R2; // Right output rectification transform
	cv::Mat_<double> P1; // Left  output projection matrix
	cv::Mat_<double> P2; // Right output projection matrix
	cv::Mat_<double> Q ; // Disparity-to-depth mapping matrix
	        cv::Mat cpuUndistortMapsLeft[2];
	        cv::Mat cpuUndistortMapsRight[2];
	cv::cuda::GpuMat gpuUndistortMapsLeft[2];
	cv::cuda::GpuMat gpuUndistortMapsRight[2];
	
	
	// Copy from nlohmann::json matrix in the format [[1,2,3],[4,5,6],[7,8,9]]
	// into an OpenCV matrix.  Takes dimensions from the OpenCV matrix
	// and will throw an exception if the nlohmann::json matrix lacks sufficient elements.
	void loadInto(cv::Mat_<double> &cvMat, nlohmann::json jsonMat);

public:
	StereoCal() :
		leftCamMatrix(CAM_MAT_ROWS, CAM_MAT_COLS),
		rightCamMatrix(CAM_MAT_ROWS, CAM_MAT_COLS),
		leftDistCoeffs(1, DIST_COEFFS), rightDistCoeffs(1, DIST_COEFFS),
		R (R_MAT_ROWS, R_MAT_COLS),
		T (T_MAT_ROWS, T_MAT_COLS),
		R1(R_MAT_ROWS, R_MAT_COLS),
		R2(R_MAT_ROWS, R_MAT_COLS),
		P1(P_MAT_ROWS, P_MAT_COLS),
		P2(P_MAT_ROWS, P_MAT_COLS),
		Q (Q_MAT_ROWS, Q_MAT_COLS)
	{ ; }
	const cv::cuda::GpuMat* getGpuUndistortMapsLeft ()    { return (const cv::cuda::GpuMat*)gpuUndistortMapsLeft ; }
	const cv::cuda::GpuMat* getGpuUndistortMapsRight()    { return (const cv::cuda::GpuMat*)gpuUndistortMapsRight; }
	const          cv::Mat* getCpuUndistortMapsLeft ()    { return (const          cv::Mat*)cpuUndistortMapsLeft ; }
	const          cv::Mat* getCpuUndistortMapsRight()    { return (const          cv::Mat*)cpuUndistortMapsRight; }
	               cv::Mat  getCpuProjectionMatrixLeft () { return (               cv::Mat )P1; }
	               cv::Mat  getCpuProjectionMatrixRight() { return (               cv::Mat )P2; }
	const double getTriangulationConst() { return triangulationConst; }
	void init(nlohmann::json options);
};

#endif // __PCG_STERCAL_H__
