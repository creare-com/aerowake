/*
	cpuFastWithBinnedKps.h
	
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_CPUFASTWITHBINNEDKPS_H__
#define __PCG_CPUFASTWITHBINNEDKPS_H__

#include "stereoPtCloudGenAlg.h"
#include "cpuPreUndistortAlg.h"

class CpuFastWithBinnedKps : public CpuPreUndistortAlg {
private:
	static const unsigned int MAX_NUM_BINS = 128-1;
	
	// Helper function - performs per-image processing.
	// Input: image, algorithm parameters, member data
	// Output: keypoints & descriptors with common indices, bin boundary indicies
	void processImage(cv::Mat image, vector<cv::KeyPoint> &kp, cv::Mat &desc, 
		unsigned int (&bin_bound_idx)[MAX_NUM_BINS + 1], bool (&bin_has_points)[MAX_NUM_BINS + 1]);

	// Helper function - the core of this particular algorithm.
	// Works on member data, specifically imgLRect, imgRRect
	void computeStereoPtClouds(sensor_msgs::PointCloud & msg);
	
protected:
	// Member data
	Benchmarker bmBlurImage       ;
	Benchmarker bmFindingKps      ;
	Benchmarker bmSortingKps      ;
	Benchmarker bmComputingDesc   ;
	Benchmarker bmBinningKps      ;
	Benchmarker bmMatchingKps     ;
	Benchmarker bmFilteringKpsImp ;
	Benchmarker bmFilteringKpsDisp;
	Benchmarker bmComputingDepths ;

public:
	CpuFastWithBinnedKps(list<const Benchmarker *> * _bms) : 
		CpuPreUndistortAlg(_bms),
		bmBlurImage       ("Blurring image"),
		bmFindingKps      ("Finding keypoints"),
		bmSortingKps      ("Sorting keypoints"),
		bmComputingDesc   ("Computing descriptors"),
		bmBinningKps      ("Binning keypoints"),
		bmMatchingKps     ("Matching keypoints"),
		bmFilteringKpsImp ("Filtering keypoints (improvement)"),
		bmFilteringKpsDisp("Filtering keypoints (disparity)"),
		bmComputingDepths ("Computing depths")
	{
		bms->push_back(&bmBlurImage       );
		bms->push_back(&bmFindingKps      );
		bms->push_back(&bmSortingKps      );
		bms->push_back(&bmComputingDesc   );
		bms->push_back(&bmBinningKps      );
		bms->push_back(&bmMatchingKps     );
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
	cv::Ptr<cv::FastFeatureDetector> fastAlg;
	cv::Ptr<cv::ORB> orbAlg;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
};

#endif // __PCG_CPUFASTWITHBINNEDKPS_H__
