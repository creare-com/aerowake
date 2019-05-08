/*
	cpuFastWithBinnedKps.cpp
	
	Algorithm using the following steps:
		- Undistort images
		- Gaussian blur
		- Use FAST to find keypoints
		- Determine "bins": sets of rows which could contain matching features
		- Match within bins
		- Triangulate matches
	
	2018-01-30  JDW  Created.
*/

#include <ptCloudGenAlgs/cpuFastWithBinnedKps.h>
using namespace std;
using namespace std::chrono;
using namespace cv;

void CpuFastWithBinnedKps::init(json options, Logger * lgr, StereoCal calData) {
	StereoPtCloudGenAlg::init(options, lgr, calData);
	
	// Load configuration options
	string cur_key = "";
	try {
		cur_key = "CpuFastWithBinnedKpsOptions"; json my_section = options[cur_key];
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
		logger->logWarning("CpuFastWithBinnedKps supports a maximum bin count of " + to_string(MAX_NUM_BINS));
	}
	
	// Create OpenCV processing objects
	
	fastAlg = cv::FastFeatureDetector::create(fastThreshold);
	orbAlg = cv::ORB::create(orbMaxDescs);
	descriptorMatcher = BFMatcher::create(NormTypes::NORM_HAMMING);
}

void CpuFastWithBinnedKps::processImages(ImageDataSet imgData, sensor_msgs::PointCloud &msg) {
	if(imgData.imgVisibleLValid && imgData.imgVisibleRValid) {
		if(imgData.imgVisibleL.size() == imgData.imgVisibleR.size()) {
			// Perform inherited processing - includes undistortion
			CpuPreUndistortAlg::processImages(imgData, msg);

			// Perform processing specific to this algorithm
			computeStereoPtClouds(msg);
		} else {
			logger->logWarning("Need left and right images to be the same size.");
		}
	} else {
		logger->logWarning("Need a valid left and right image to compute stereo point cloud.");
	}
}

static bool compareKeyPointsAscY(KeyPoint a, KeyPoint b) {
	// Return whether a should go before b
	return a.pt.y < b.pt.y;
}
void CpuFastWithBinnedKps::processImage(Mat image, vector<KeyPoint> &kp, Mat &desc,
		unsigned int (&bin_bound_idx)[MAX_NUM_BINS + 1], bool (&bin_has_points)[MAX_NUM_BINS + 1]) {
	// Perform gaussian blur
	bmBlurImage.resume();
	Size ksize = {blurKernelSize, blurKernelSize};
	Mat imageBlurred(image.rows, image.cols, image.type());
	GaussianBlur(image, imageBlurred, ksize, blurKernelSize);
	bmBlurImage.pause();
	
	// Detect features (keypoints)
	// Stores to CPU-accessible RAM
	bmFindingKps.resume();
	fastAlg->detect(imageBlurred, kp);
	bmFindingKps.pause(kp.size());
	logger->logDebug("Found " + to_string(kp.size()) + " features.");
	
	if(kp.size() > 0) {
		// For debugging, do some quick statistics
		auto it = kp.begin();
		unsigned int max_x, max_y, min_x, min_y;
		max_x = min_x = it->pt.x;
		max_y = min_y = it->pt.y;
		for( ; it != kp.end(); ++it) {
			if(it->pt.x > max_x) { max_x = it->pt.x; }
			if(it->pt.y > max_y) { max_y = it->pt.y; }
			if(it->pt.x < min_x) { min_x = it->pt.x; }
			if(it->pt.y < min_y) { min_y = it->pt.y; }
		}
		logger->logDebug("Keypoint statistics: max X: " + 
			to_string(max_x) + "; max Y: " + to_string(max_y) + "; min X: " + 
			to_string(min_x) + "; min Y: " + to_string(min_y));
		
		// Sort by y coordinate (on CPU)
		bmSortingKps.resume();
		sort(kp.begin(), kp.end(), compareKeyPointsAscY);
		bmSortingKps.pause(kp.size());
		
		// Compute descriptors for the keypoints found by FAST.
		// Note that ORB is also capable of finding keypoints, but here we direct it
		// to use those found by FAST (it's quicker this way).  This call will reduce 
		// the size of the keypoint vectors.
		bmComputingDesc.resume();
		orbAlg->compute(imageBlurred, kp, desc);
		bmComputingDesc.pause(desc.rows);
		logger->logDebug("Computed " + to_string(desc.rows) + " descriptors.");
		
		// Locate bin boundaries (indices into l_kp and r_kp).
		// We need to do this as indices rather than iterators, since
		// we must use them to index into the descriptor arrays.
		bmBinningKps.resume();
		bin_bound_idx[0] = 0;
		// We confirmed earlier that both images are the same size
		unsigned int bin_height = (unsigned int)(imageBlurred.size().height / (double)numBins);
		for(unsigned int bin = 0; bin <= numBins; ++bin) {
			// cout << "Bin " << bin << ": " << (bin * bin_height) << " to " << ((bin + 1) * bin_height) << ": ";
			KeyPoint boundary_pt(0.0f, (float)(bin * bin_height), 0.0);
			// Check if any keypoints are above or equal to the (numerically) higher edge of the bin
			auto boundary_result = lower_bound(kp.begin(), kp.end(), boundary_pt, compareKeyPointsAscY);
			if(boundary_result == kp.end()) {
				// no such point found at all
				bin_has_points[bin] = false;
				// cout << "no boundary found. " ;
				
				// This implies that all points were in previous bins.
				// Binning further isn't really useful.
				if(bin == 0) {
					bin_has_points[0] = true;
				} else {
					// was marked as having points last time around
				}
				// break;
			} else {
				// Was that point actually in a later bin?
				if(boundary_result->pt.y >= (float)((bin + 1) * bin_height)) {
					bin_has_points[bin] = false;
					// cout << "bound in next bin. ";
				} else {
					bin_has_points[bin] = true;
					// cout << "   boundary found. ";
				}
			}
			// Note that boundary_result might be kp.end()
			bin_bound_idx[bin] = (boundary_result - kp.begin());
			// cout << "i= " << bin_bound_idx[bin] << " of " << kp.size() << ".";
			// cout << endl;
			// cout.flush();
		}
		bmBinningKps.pause(kp.size());
	}
}

void CpuFastWithBinnedKps::computeStereoPtClouds(sensor_msgs::PointCloud & msg) {
	// Perform single-image parts of the processing chain
	vector<KeyPoint> l_kp, r_kp; // l_kp[0].pt.y
	Mat l_desc, r_desc;
	unsigned int l_bin_bound_idx [MAX_NUM_BINS + 1],
	             r_bin_bound_idx [MAX_NUM_BINS + 1];
	bool         l_bin_has_points[MAX_NUM_BINS + 1],
	             r_bin_has_points[MAX_NUM_BINS + 1];
	for(unsigned int i = 0; i < MAX_NUM_BINS + 1; ++i) {
		l_bin_bound_idx [i] = 0;
		r_bin_bound_idx [i] = 0;
		l_bin_has_points[i] = false;
		r_bin_has_points[i] = false;
	}
	processImage(imgLRect, l_kp, l_desc, l_bin_bound_idx, l_bin_has_points);
	processImage(imgRRect, r_kp, r_desc, r_bin_bound_idx, r_bin_has_points);
	bmBlurImage    .conclude();
	bmFindingKps   .conclude();
	bmSortingKps   .conclude();
	bmComputingDesc.conclude();
	bmBinningKps   .conclude();
	
	// Match+filter features
	vector<DMatch> matches;
	matches.reserve(l_kp.size()); // preallocate on a good guess
	bmMatchingKps    .start(); bmMatchingKps    .pause();
	bmFilteringKpsImp.start(); bmFilteringKpsImp.pause();
	logger->logDebug("l_desc is " + to_string(l_desc.rows) + "rows x " + to_string(l_desc.cols) + "cols.");
	logger->logDebug("r_desc is " + to_string(r_desc.rows) + "rows x " + to_string(r_desc.cols) + "cols.");
	if(l_desc.rows > 0 && r_desc.rows > 0) {
		for(unsigned int bin = 0; bin < numBins; ++bin) {
			if(l_bin_has_points[bin] && r_bin_has_points[bin]) {
				// Match
				bmMatchingKps.resume();
				vector<vector<DMatch>> matches_this_bin;
				
				try {
					auto l_bin_descs = l_desc.rowRange(l_bin_bound_idx[bin], l_bin_bound_idx[bin+1]);
					auto r_bin_descs = r_desc.rowRange(r_bin_bound_idx[bin], r_bin_bound_idx[bin+1]);
					// Find best 2 matches - 2 is inherent to the next step
					descriptorMatcher->knnMatch(l_bin_descs, r_bin_descs, matches_this_bin, 2);
					bmMatchingKps.pause(matches_this_bin.size());
					logger->logDebug("Bin " + to_string(bin) + ": Found " + to_string(matches_this_bin.size()) + " matches.");
					
					// Filter by discarding matches where the next-best match is too similar
					bmFilteringKpsImp.resume();
					for(auto it = matches_this_bin.begin(); it != matches_this_bin.end(); ++it) {
						// Dereferencing the iterator gets you type vector<DMatch> with size 2
						DMatch this_match = (*it)[0], next_best_match = (*it)[1];
						if(this_match.distance < (minImprovementFactor * next_best_match.distance)) {
							// Indices are into the submatrices, not the full matrix
							this_match.queryIdx += l_bin_bound_idx[bin];
							this_match.trainIdx += r_bin_bound_idx[bin];
							matches.push_back(this_match);
						}
					}
				} catch (cv::Exception e) {
					logger->logWarning("Handled OpenCV exception while matching.  Please report this as a bug.");
				}
				bmFilteringKpsImp.pause(matches_this_bin.size());
			} else {
				// Can't do anything if one of the bins is empty
			}
		}
		bmMatchingKps    .conclude();
		bmFilteringKpsImp.conclude();
		logger->logDebug(to_string(matches.size()) + " matches passed improvement filtering.");

		// Filter based on disparity
		bmFilteringKpsDisp.start(); 
		vector<Point2f> l_kp_filtered, r_kp_filtered; // l_kp[0].pt.y
		l_kp_filtered.reserve(matches.size());
		r_kp_filtered.reserve(matches.size());
		for(auto it = matches.begin(); it != matches.end(); ++it) {
			// Dereferencing the iterator gets you type DMatch.
			// In this case, queryIdx is an index into the left keypoint vector, and
			// trainIdx is an index into the right keypoint vector, because of the
			// argument order of knnMatch().
			
			// Make sure our binning is working as expected.  This should never
			// be a problem; this is to prevent segfaults if there is a problem.
			if(it->queryIdx < 0 || (unsigned)(it->queryIdx) >= l_kp.size()) {
				logger->logWarning("Match iterator it->queryIdx is outside vector l_kp.  Please report this as a bug.");
			} else if(it->trainIdx < 0 || (unsigned)(it->trainIdx) >= r_kp.size()) {
				logger->logWarning("Match iterator it->trainIdx is outside vector r_kp.  Please report this as a bug.");
			} else {
				// Safe to check
				if((l_kp[it->queryIdx].pt.x - r_kp[it->trainIdx].pt.x) > minDisparityPx) {
					l_kp_filtered.push_back(l_kp[it->queryIdx].pt);
					r_kp_filtered.push_back(r_kp[it->trainIdx].pt);
				}
			}
		}
		bmFilteringKpsDisp.end(matches.size()); 
		logger->logDebug(to_string(l_kp_filtered.size()) + " matches passed disparity filtering.");

		// Compute depths.
		if(l_kp_filtered.size() > 3) {
			// Compute depths
			bmComputingDepths.start();
			Mat l_kp_mat(l_kp_filtered);
			Mat r_kp_mat(r_kp_filtered);
			Mat points4d; // Points in homogeneous coordinates
			triangulatePoints(cal_data.getCpuProjectionMatrixLeft (),
							  cal_data.getCpuProjectionMatrixRight(),
							  l_kp_mat, r_kp_mat, points4d);
							  // l_kp_filtered, r_kp_filtered, points4d);
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
			bmComputingDepths.end(matches.size());
		}
	} else {
		// One or both of the images had no useful features
	}
}
