/*
	cpuPreUndistortAlg.h
	
	Parent class of algorithms that undistort images prior to processing.
	Uses CPU.
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_CPUPREUNDISTORTALG_H__
#define __PCG_CPUPREUNDISTORTALG_H__

#include "stereoPtCloudGenAlg.h"

class CpuPreUndistortAlg : public StereoPtCloudGenAlg {
private:
	// Use the member cal_data to undistort stereo input images.
	// Stores output in member data.
	void cpuUndistort(ImageDataSet imgData);

protected:
	// Rectified versions of input images
	cv::Mat imgLRect, imgRRect;

	// Member data
	Benchmarker bmUndistortOnCpu;

public:
	CpuPreUndistortAlg(list<const Benchmarker *> * _bms) : 
		StereoPtCloudGenAlg(_bms),
		bmUndistortOnCpu("Undistort images on CPU")
	{
		bms->push_back(&bmUndistortOnCpu);
	}
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
};

#endif // __PCG_CPUPREUNDISTORTALG_H__
