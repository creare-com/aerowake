/*
	gpuPreUndistortAlg.h
	
	Parent class of algorithms that undistort images prior to processing.
	Uses GPU.
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_GPUPREUNDISTORTALG_H__
#define __PCG_GPUPREUNDISTORTALG_H__

#include "stereoPtCloudGenAlg.h"

class GpuPreUndistortAlg : public StereoPtCloudGenAlg {
private:
	// Use the member cal_data to undistort stereo input images.
	// Stores output in member data.
	void gpuUndistort(ImageDataSet imgData);

protected:
	// Rectified versions of input images, when stored in GPU memory
	cv::cuda::GpuMat imgLRectGpu, imgRRectGpu;

	// Member data
	Benchmarker bmDataXferGpu   ;
	Benchmarker bmUndistortOnGpu;

public:
	GpuPreUndistortAlg(list<const Benchmarker *> * _bms) :
		StereoPtCloudGenAlg(_bms),
		bmDataXferGpu   ("Data transfer to/from GPU"),
		bmUndistortOnGpu("Undistort images on GPU")
	{
		bms->push_back(&bmDataXferGpu   );
		bms->push_back(&bmUndistortOnGpu);
	}
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
};

#endif // __PCG_GPUPREUNDISTORTALG_H__
