#include "surf_feature_tracker.h"

#include <vector>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Lab 5 | Deliverable 6.a - SURF
//  SURF is "SIFT's fast cousin": same interface (cv::xfeatures2d::SURF is a
//  cv::Feature2D), so every function below is the SIFT one with another object.
//  Descriptors are 64-D float (SURF::create(100, 4, 3, false, false) default:
//  upright = false, extended = false).
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

using namespace cv;
using namespace cv::xfeatures2d;

/**
   Surf feature tracker Constructor.
*/
SurfFeatureTracker::SurfFeatureTracker()
  : FeatureTracker(),
    detector(SURF::create()) {
  tracker_name_ = "surf";
}

/**
    Detects keypoints on a single image.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SurfFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
  VLOG(1) << "SURF detected " << keypoints->size() << " keypoints";
}

/**
    Computes one 64-D (non-extended) descriptor per keypoint.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image.
    @param[out] descriptors Descriptors for the given keypoints (N x 64, CV_32F).
*/
void SurfFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  detector->compute(img, *keypoints, *descriptors);
  CHECK_EQ(keypoints->size(), static_cast<size_t>(descriptors->rows));
}

/**
    Exactly the same FLANN + Lowe ratio-test logic as for SIFT (float
    descriptors -> default KD-tree/L2 FLANN configuration).
    @param[in] descriptors_1 First list of descriptors.
    @param[in] descriptors_2 Second list of descriptors.
    @param[out] matches List of k best matches between descriptors.
    @param[out] good_matches List of descriptors classified as "good"
*/
void SurfFeatureTracker::matchDescriptors(
                                          const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);

  FlannBasedMatcher flann_matcher;
  static constexpr float kLoweRatio = 0.8f;

  matches->clear();
  good_matches->clear();

  flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);

  for (const auto& match : *matches) {
    if (match.size() < 2) {
      continue;
    }
    if (match[0].distance < kLoweRatio * match[1].distance) {
      good_matches->push_back(match[0]);
    }
  }
  VLOG(1) << "SURF: " << matches->size() << " knn matches, "
          << good_matches->size() << " pass the ratio test";
}
