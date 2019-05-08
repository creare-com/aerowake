/*
	gpuFastWithBinnedKps.cpp
	
	Algorithm using the following steps:
		- Undistort images
		- Gaussian blur
		- Use FAST to find keypoints
		- Determine "bins": sets of rows which could contain matching features
		- Match within bins
		- Triangulate matches
	
	2017-09-29  JDW  Created.
*/

#include <ptCloudGenAlgs/gpuFastWithBinnedKps.h>
using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cuda;

void GpuFastWithBinnedKps::init(json options, Logger * lgr, StereoCal calData) {
	StereoPtCloudGenAlg::init(options, lgr, calData);
	
	// Load configuration options
	string cur_key = "";
	try {
		cur_key = "GpuFastWithBinnedKpsOptions"; json my_section = options[cur_key];
		cur_key = "blurKernelSize";        blurKernelSize       = my_section[cur_key];
		cur_key = "fastThreshold";         fastThreshold        = my_section[cur_key];
		cur_key = "orbMaxDescs";           orbMaxDescs          = my_section[cur_key];
		cur_key = "numBins";               numBins              = my_section[cur_key];
		cur_key = "minImprovementFactor";  minImprovementFactor = my_section[cur_key];
		cur_key = "minDisparityPx";        minDisparityPx       = my_section[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in processing section: "
			 << e.what() << endl;
		throw(e);
	}
	
	// Enforce bounds
	if(numBins > MAX_NUM_BINS) {
		numBins = MAX_NUM_BINS;
		logger->logWarning("GpuFastWithBinnedKps supports a maximum bin count of " + to_string(MAX_NUM_BINS));
	}
	
	// Create OpenCV processing objects
	Size ksize = {blurKernelSize, blurKernelSize};
	gaussianFilter = cuda::createGaussianFilter(CV_8U, CV_8U, ksize, blurKernelSize);
	fastAlgCpu = cv::FastFeatureDetector::create(fastThreshold);
	descriptorMatcher = cuda::DescriptorMatcher::createBFMatcher(NormTypes::NORM_HAMMING);
}

void GpuFastWithBinnedKps::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	if(imgData.imgVisibleLValid && imgData.imgVisibleRValid) {
		if(imgData.imgVisibleL.size() == imgData.imgVisibleR.size()) {
			// Perform inherited processing - includes undistortion
			GpuPreUndistortAlg::processImages(imgData, msg);

			// Perform processing specific to this algorithm
			computeStereoPtCloudsGpu(msg);
		} else {
			logger->logWarning("Need left and right images to be the same size.");
		}
	} else {
		logger->logWarning("Need a valid left and right image to compute stereo point cloud.");
	}
}

string GpuFastWithBinnedKps::getStatsOfKeypoints(vector<KeyPoint> kps) {
	auto keypoint = kps.begin();
	if(keypoint != kps.end()) {
		unsigned int max_x, max_y, min_x, min_y;
		max_x = min_x = keypoint->pt.x;
		max_y = min_y = keypoint->pt.y;
		for( ; keypoint != kps.end(); ++keypoint) {
			if(keypoint->pt.x > max_x) { max_x = keypoint->pt.x; }
			if(keypoint->pt.y > max_y) { max_y = keypoint->pt.y; }
			if(keypoint->pt.x < min_x) { min_x = keypoint->pt.x; }
			if(keypoint->pt.y < min_y) { min_y = keypoint->pt.y; }
		}
		return "Keypoint statistics: " + to_string(kps.size()) + " points.  X: " + 
			to_string(min_x) + "-" + to_string(max_x) + "; Y: " + 
			to_string(min_y) + "-" + to_string(max_y);
	} else {
		return "No keypoints.";
	}
}


vector<GpuFastWithBinnedKps::Bin> GpuFastWithBinnedKps::findKeypointsAndDescriptors(GpuMat image) {
	vector<GpuFastWithBinnedKps::Bin> bins;
	
	// Create bin structures via default constructors
	bins.resize(numBins);
	
	GpuMat imageBlurred;
	try {
		// Perform gaussian blur
		bmBlurImage.resume();
		gaussianFilter->apply(image, imageBlurred);
		bmBlurImage.pause();
	} catch (cv::Exception e) {
		logger->logError("Handled OpenCV exception while blurring image: " + string(e.what()));
		return bins;
	}
	bmDataXferGpu.resume();
	Mat imageBlurredCpu(image.rows, image.cols, image.type());
	imageBlurred.download(imageBlurredCpu);
	bmDataXferGpu.pause();
	
	// We confirmed earlier that both images are the same size
	unsigned int binHeight = (unsigned int)(imageBlurred.size().height / (double)numBins);
	for(auto bin = bins.begin(); bin != bins.end(); ++bin) {
		unsigned int binIdx = bin - bins.begin();
		
		// Get an image subset (on the CPU).  This is a shallow copy - that is,
		// no image data is transferred, just the boundaries.
		Mat imageSliceForBin = imageBlurredCpu.rowRange(binIdx * binHeight, (binIdx + 1) * binHeight);

		// Detect features (keypoints)
		// Stores to CPU-accessible RAM
		// As of OpenCV 3.4.0, there appears to be a bug in the GPU version of FAST.
		// It returns significantly fewer keypoints, from the top of the image, as if
		// it was interrupted partway through processing.  The number of keypoints it
		// returns changes every time I run it, even with the same input image, but it
		// tends to be something like 1/4 of the keypoints that the CPU version returns.
		// The CPU version also doesn't run significantly slower than the GPU version.
		// So we just do this part on the CPU.
		bmFindingKps.resume();
		fastAlgCpu->detect(imageSliceForBin, bin->keypoints);
		logger->logDebug("Found " + to_string(bin->keypoints.size()) + " features on CPU.");
		// Keypoints here are relative to the start of the bin.
		// Add an offset to make them relative to the start of the image.
		for(auto kp = bin->keypoints.begin(); kp != bin->keypoints.end(); ++kp) {
			kp->pt.y += binIdx * binHeight;
		}
		bmFindingKps.pause(bin->keypoints.size());

		// As of OpenCV 3.4.0, there seems to be a bug regarding computing keypoints
		// near the bottom of the image.  Remove keypoints too close to the bottom.
		// http://en.cppreference.com/w/cpp/algorithm/remove
		size_t numKpsUnfiltered = bin->keypoints.size();
		bmFilteringKpsOrb.resume();
		int maxVal = image.rows - ORB_IMAGE_BOTTOM_MARGIN;
		bin->keypoints.erase(std::remove_if(bin->keypoints.begin(), bin->keypoints.end(),
			[maxVal](KeyPoint kp){ return kp.pt.y >= maxVal; }
			), bin->keypoints.end());
		size_t numKpsRemoved = numKpsUnfiltered - bin->keypoints.size();
		bmFilteringKpsOrb.pause(numKpsUnfiltered);
		if(numKpsRemoved > 0) {
			logger->logDebug("Removed " + to_string(numKpsRemoved) + " keypoint(s) that would cause ORB to crash.");
		}
		
		// For debugging, do some quick statistics
		logger->logDebug(getStatsOfKeypoints(bin->keypoints));
		
		if(bin->keypoints.size() > 0) {
			try {
				bmComputingDesc.resume();
				Ptr<cuda::ORB> orbAlg = cuda::ORB::create(orbMaxDescs);
				orbAlg->compute(imageBlurred, bin->keypoints, bin->descriptors);
				bmComputingDesc.pause(bin->descriptors.rows);
				logger->logDebug("Bin " + to_string(binIdx) + ": computed " + to_string(bin->descriptors.rows) + " descriptors.");
			} catch (cv::Exception e) {
				logger->logWarning("Handled OpenCV exception while computing descriptors: " + string(e.what()));
			}
		}
	}

	return bins;
}

void GpuFastWithBinnedKps::computeStereoPtCloudsGpu(sensor_msgs::PointCloud & msg) {
	// Perform single-image parts of the processing chain
	vector<GpuFastWithBinnedKps::Bin> leftBinData  = findKeypointsAndDescriptors(imgLRectGpu);
	vector<GpuFastWithBinnedKps::Bin> rightBinData = findKeypointsAndDescriptors(imgRRectGpu);
	bmBlurImage      .conclude();
	bmFindingKps     .conclude();
	bmFilteringKpsOrb.conclude();
	bmComputingDesc  .conclude();
	bmDataXferGpu    .conclude();
	
	// Match+filter features
	bmMatchingKps    .start(); bmMatchingKps    .pause();
	bmFilteringKpsImp.start(); bmFilteringKpsImp.pause();
	// Final output of matching and filtering, to go into stereo vision computation.
	// leftFilteredKeypoints[i] matches rightFilteredKeypoints[i]
	vector<Point2f> leftFilteredKeypoints, rightFilteredKeypoints;

	// Both of these will be the same length
	auto leftBin  = leftBinData.begin();
	auto rightBin = rightBinData.begin();
	for(; leftBin != leftBinData.end() && rightBin != rightBinData.end(); ++leftBin, ++rightBin) {
		vector<DMatch> matchesWithSufficientImprovement;
		if(leftBin->descriptors.rows > 0 && rightBin->descriptors.rows > 0) {
			// Match
			bmMatchingKps.resume();
			vector<vector<DMatch>> matchesThisBin;
			
			try {
				// Find best 2 matches - 2 is inherent to the next step
				descriptorMatcher->knnMatch(leftBin->descriptors, rightBin->descriptors, matchesThisBin, 2);
				bmMatchingKps.pause(matchesThisBin.size());
				logger->logDebug("Bin " + to_string((int)(leftBin - leftBinData.begin())) + ": Found " + to_string(matchesThisBin.size()) + " matches.");
				
				// Filter by discarding matches where the next-best match is too similar
				bmFilteringKpsImp.resume();
				for(auto matchPair = matchesThisBin.begin(); matchPair != matchesThisBin.end(); ++matchPair) {
					// Dereferencing the iterator gets you type vector<DMatch> with size 2
					DMatch thisMatch = (*matchPair)[0], nextBestMatch = (*matchPair)[1];
					if(thisMatch.distance < (minImprovementFactor * nextBestMatch.distance)) {
						// These matches have indices relative to this bin's decriptors 
						// (row indices into leftBin->descriptors and rightBin->descriptors,
						// which are also indices into leftBin->keypoints and rightBin->keypoints)
						matchesWithSufficientImprovement.push_back(thisMatch);
					}
				}
			} catch (cv::Exception e) {
				logger->logWarning("Handled OpenCV exception while matching.  Please report this as a bug.");
			}
			bmFilteringKpsImp.pause(matchesThisBin.size());
		} else {
			// Can't do anything if one of the bins is empty, which can happen.
			// This is usually the case at startup, before the auto exposure has worked its magic.
		}
		bmMatchingKps    .conclude();
		bmFilteringKpsImp.conclude();
		logger->logDebug(to_string(matchesWithSufficientImprovement.size()) + " matches passed improvement filtering.");

		// Filter based on disparity
		bmFilteringKpsDisp.start(); 
		for(auto match = matchesWithSufficientImprovement.begin(); match != matchesWithSufficientImprovement.end(); ++match) {
			// Dereferencing the iterator gets you type DMatch.
			// In this case, queryIdx is an index into the left keypoint vector, and
			// trainIdx is an index into the right keypoint vector, because of the
			// argument order of knnMatch().
			
			// Make sure our binning is working as expected.  This should never
			// be a problem; this is to prevent segfaults if there is a problem.
			if(match->queryIdx < 0 || (unsigned)(match->queryIdx) >= leftBin->keypoints.size()) {
				logger->logWarning("Match iterator match->queryIdx of " + to_string(match->queryIdx) + " is outside vector leftBin->keypoints.  Please report this as a bug.");
			} else if(match->trainIdx < 0 || (unsigned)(match->trainIdx) >= rightBin->keypoints.size()) {
				logger->logWarning("Match iterator match->trainIdx of " + to_string(match->trainIdx) + " is outside vector rightBin->keypoints.  Please report this as a bug.");
			} else {
				// Safe to check
				if((leftBin->keypoints[match->queryIdx].pt.x - rightBin->keypoints[match->trainIdx].pt.x) > minDisparityPx) {
					leftFilteredKeypoints.push_back(leftBin->keypoints[match->queryIdx].pt);
					rightFilteredKeypoints.push_back(rightBin->keypoints[match->trainIdx].pt);
				}
			}
		}
		bmFilteringKpsDisp.end(rightFilteredKeypoints.size()); 
		logger->logDebug(to_string(leftFilteredKeypoints.size()) + " matches passed disparity filtering.");
	}
	
	// Compute depths.
	if(leftFilteredKeypoints.size() > 3) {
		// Compute depths
		bmComputingDepths.start();
		Mat leftKeypointsMat(leftFilteredKeypoints);
		Mat rightKeypointsMat(rightFilteredKeypoints);
		Mat points4d; // Points in homogeneous coordinates
		triangulatePoints(cal_data.getCpuProjectionMatrixLeft (),
						  cal_data.getCpuProjectionMatrixRight(),
						  leftKeypointsMat, rightKeypointsMat, points4d);
						  // leftFilteredKeypoints, rightFilteredKeypoints, points4d);
		points4d = points4d.t();
		logger->logDebug("points4d: " + to_string(points4d.rows) + "rows x " + to_string(points4d.cols) + " cols.");
		vector<Point3f> points3d; // Points in cartesian coordinates
		convertPointsFromHomogeneous(points4d, points3d);
		for(auto point : points3d) {
			geometry_msgs::Point32 pt;
			
			// In OpenCV image frame of reference.
			// Algorithm output is in mm, convert to m
			double depth = point.z / 1000.0;
			double right = point.x / 1000.0;
			double down  = point.y / 1000.0;
			
			// Coordinate convention is (X, Y, Z) = (Forward, Starboard, Down)
			// relative to vehicle.  REP-0103 (http://www.ros.org/reps/rep-0103.html)
			// says we should add the "_optical" suffix to our frame.
			pt.x = depth;
			pt.y = right;
			pt.z = down ;
			
			msg.points.push_back(pt);
		}
		bmComputingDepths.end(leftFilteredKeypoints.size());
	}

}
