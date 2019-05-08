/*
	dummyAlg.h
	
	Algorithm that does nothing, but has a configurable sleep time to simulate
	processing.
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_DUMMYALG_H__
#define __PCG_DUMMYALG_H__

#include "stereoPtCloudGenAlg.h"

class DummyAlg : public StereoPtCloudGenAlg {
private:
	chrono::steady_clock::duration delayTimePerCycle;
public:
	DummyAlg(list<const Benchmarker *> * _bms) : StereoPtCloudGenAlg(_bms) { ; }
	virtual void init(nlohmann::json options, Logger * lgr, StereoCal calData);
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
};

#endif // __PCG_DUMMYALG_H__
