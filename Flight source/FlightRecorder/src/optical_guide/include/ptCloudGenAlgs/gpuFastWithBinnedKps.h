/*
	gpuFastWithBinnedKps.h
	
	This is a copy of CpuFastWithBinnedKps, updated to use GPU types and
	functions.  It would be awesome to have some kind of code-sharing relationship
	between the two, but there seem to be just enough differences to prevent the
	productive use of templates and inheritance.
	Uses GPU.
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_GPUFASTWITHBINNEDKPS_H__
#define __PCG_GPUFASTWITHBINNEDKPS_H__

#include "stereoPtCloudGenAlg.h"
#include "gpuPreUndistortAlg.h"

class GpuFastWithBinnedKps : public GpuPreUndistortAlg {
private:
	struct Bin {
		// Keypoint indices match descriptor rows
		vector<cv::KeyPoint> keypoints;
		cv::cuda::GpuMat descriptors;
	};

	static const unsigned int MAX_NUM_BINS = 128-1;
	// As of OpenCV 3.4.0, there seems to be a bug in the GPU ORB implementation.
	// It throws an exception if you try to compute a descriptor for a keypoint
	// closer than 12-13 pixels from the bottom of our 1536-row images.
	// It's unclear to me why.
	static const unsigned int ORB_IMAGE_BOTTOM_MARGIN = 15;
	
	// Helper function - performs per-image processing.
	// Returns a vector exactly numBins long.
	vector<Bin> findKeypointsAndDescriptors(cv::cuda::GpuMat image);

	// Helper function - the core of this particular algorithm.
	// Works on member data, specifically imgLRectGpu, imgRRectGpu
	void computeStereoPtCloudsGpu(sensor_msgs::PointCloud & msg);
	
	string getStatsOfKeypoints(vector<cv::KeyPoint> kps);
	
protected:
	// Member data
	Benchmarker bmBlurImage       ;
	Benchmarker bmFindingKps      ;
	Benchmarker bmComputingDesc   ;
	Benchmarker bmMatchingKps     ;
	Benchmarker bmFilteringKpsOrb ;
	Benchmarker bmFilteringKpsImp ;
	Benchmarker bmFilteringKpsDisp;
	Benchmarker bmComputingDepths ;

public:
	GpuFastWithBinnedKps(list<const Benchmarker *> * _bms) : 
		GpuPreUndistortAlg(_bms),
		bmBlurImage       ("Blurring image"),
		bmFindingKps      ("Finding keypoints"),
		bmComputingDesc   ("Computing descriptors"),
		bmMatchingKps     ("Matching keypoints"),
		bmFilteringKpsOrb ("Filtering keypoints (keep ORB from crashing)"),
		bmFilteringKpsImp ("Filtering keypoints (improvement)"),
		bmFilteringKpsDisp("Filtering keypoints (disparity)"),
		bmComputingDepths ("Computing depths")
	{
		bms->push_back(&bmBlurImage       );
		bms->push_back(&bmFindingKps      );
		bms->push_back(&bmComputingDesc   );
		bms->push_back(&bmMatchingKps     );
		bms->push_back(&bmFilteringKpsOrb );
		bms->push_back(&bmFilteringKpsImp );
		bms->push_back(&bmFilteringKpsDisp);
		bms->push_back(&bmComputingDepths );
	}
	virtual void init(nlohmann::json options, Logger * lgr, StereoCal calData);
	virtual void processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg);
private:

	// Algorithm parameters
	int blurKernelSize;
	int fastThreshold;
	int orbMaxDescs;
	unsigned int numBins;
	double minImprovementFactor;
	unsigned int minDisparityPx;
	
	// OpenCV processing objects
	cv::Ptr<cv::cuda::Filter> gaussianFilter;
	cv::Ptr<cv::FastFeatureDetector> fastAlgCpu;
	cv::Ptr<cv::cuda::DescriptorMatcher> descriptorMatcher;
};

#endif // __PCG_GPUFASTWITHBINNEDKPS_H__
