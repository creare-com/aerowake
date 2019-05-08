/*
	postUndistortAlg.h
	
	Parent class of algorithms that undistort after processing
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_POSTUNDISTORTALG_H__
#define __PCG_POSTUNDISTORTALG_H__

#include "stereoPtCloudGenAlg.h"

class PostUndistortAlg : public StereoPtCloudGenAlg {
protected:
	// TODO
public:
	PostUndistortAlg(list<const Benchmarker *> * _bms) : StereoPtCloudGenAlg(_bms) { ; }
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
};
#endif // __PCG_POSTUNDISTORTALG_H__
