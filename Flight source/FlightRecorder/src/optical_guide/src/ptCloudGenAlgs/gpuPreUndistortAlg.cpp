/*
	gpuPreUndistortAlg.cpp
	
	Abstract class for algorithms requiring image undistortion prior to processing
	
	2017-2-28  JDW  Created.
*/

#include <ptCloudGenAlgs/gpuPreUndistortAlg.h>

using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cuda;

void GpuPreUndistortAlg::gpuUndistort(ImageDataSet imgData) {

	// Upload to GPU
	bmDataXferGpu.start();
	cuda::GpuMat imgLUnrect(imgData.imgVisibleL);
	cuda::GpuMat imgRUnrect(imgData.imgVisibleR);
	bmDataXferGpu.pause(2);

	// Perform undistort
	bmUndistortOnGpu.start();
	auto undistortMapsLeft  = cal_data.getGpuUndistortMapsLeft ();
	auto undistortMapsRight = cal_data.getGpuUndistortMapsRight();
	cv::cuda::remap(imgLUnrect, imgLRectGpu, undistortMapsLeft[0],  undistortMapsLeft[1],  INTER_LINEAR);
	cv::cuda::remap(imgRUnrect, imgRRectGpu, undistortMapsRight[0], undistortMapsRight[1], INTER_LINEAR);
	bmUndistortOnGpu.end(2);
}

void GpuPreUndistortAlg::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	// Parent tasks
	StereoPtCloudGenAlg::processImages(imgData, msg);
	
	// Undistort and store to member data
	gpuUndistort(imgData);
}

