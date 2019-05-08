/*
	cpuPreUndistortAlg.cpp
	
	Abstract class for algorithms requiring image undistortion prior to processing
	
	2017-9-29  JDW  Created.
*/

#include <ptCloudGenAlgs/cpuPreUndistortAlg.h>

using namespace std;
using namespace std::chrono;
using namespace cv;

void CpuPreUndistortAlg::cpuUndistort(ImageDataSet imgData) {

	// Perform undistort
	bmUndistortOnCpu.start();
	auto undistortMapsLeft  = cal_data.getCpuUndistortMapsLeft ();
	auto undistortMapsRight = cal_data.getCpuUndistortMapsRight();
	cv::remap(imgData.imgVisibleL, imgLRect, undistortMapsLeft[0],  undistortMapsLeft[1],  INTER_LINEAR);
	cv::remap(imgData.imgVisibleR, imgRRect, undistortMapsRight[0], undistortMapsRight[1], INTER_LINEAR);
	bmUndistortOnCpu.end(2);
}

void CpuPreUndistortAlg::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	// Parent tasks
	StereoPtCloudGenAlg::processImages(imgData, msg);
	
	// Undistort and store to member data
	cpuUndistort(imgData);
}

