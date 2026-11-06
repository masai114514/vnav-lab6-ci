#include "sift_feature_tracker.h"

#include <vector>
#include <glog/logging.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Lab 5 | Deliverables 3, 4 (and 6.a) - SIFT
//  SIFT = Scale Invariant Feature Transform. The detector/descriptor is the
//  cv::xfeatures2d::SIFT object created in the constructor below; both
//  detectKeypoints() and describeKeypoints() are single cv::Feature2D calls on
//  that object. Matching uses FLANN (k-NN, k = 2) plus Lowe's ratio test.
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

using namespace cv;
using namespace cv::xfeatures2d;

/**
   Sift feature tracker Constructor.
*/
SiftFeatureTracker::SiftFeatureTracker()
  : FeatureTracker(),
    detector(SIFT::create()) {
  tracker_name_ = "sift";
}

/**
    Detects keypoints on a single image. `detector` is a cv::SIFT, i.e. a
    cv::Feature2D: one call is enough.
    @param[in] img Image input where to detect keypoints.
    @param[out] keypoints List of keypoints detected on the given image.
*/
void SiftFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<KeyPoint>* keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
  VLOG(1) << "SIFT detected " << keypoints->size() << " keypoints";
}

/**
    Computes one 128-D descriptor per keypoint.
    @param[in] img Image used to detect the keypoints.
    @param[in, out] keypoints List of keypoints detected on the image.
    @param[out] descriptors Descriptors for the given keypoints (N x 128, CV_32F).
*/
void SiftFeatureTracker::describeKeypoints(const cv::Mat& img,
                                           std::vector<KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  // SIFT descriptors are 128-dimensional float vectors; compute() keeps the
  // keypoint list untouched (it only drops keypoints too close to the border).
  detector->compute(img, *keypoints, *descriptors);
  CHECK_EQ(keypoints->size(), static_cast<size_t>(descriptors->rows));
}

/**
    Matches descriptors with FLANN + Lowe's ratio test.
    1. knnMatch(k = 2) gives, for each descriptor of image 1, its two nearest
       neighbours in image 2. `matches` is a N x 2 matrix.
    2. Keep the best match only if it is clearly better than the runner-up:
       match[0].distance < 0.8 * match[1].distance
       (SIFT paper, section 7.1: "ratio of closest to second-closest distance").

    @param[in] descriptors_1 First list of descriptors.
    @param[in] descriptors_2 Second list of descriptors.
    @param[out] matches List of k best matches between descriptors.
    @param[out] good_matches List of descriptors classified as "good"
*/
void SiftFeatureTracker::matchDescriptors(
                                          const cv::Mat& descriptors_1,
                                          const cv::Mat& descriptors_2,
                                          std::vector<std::vector<DMatch>>* matches,
                                          std::vector<cv::DMatch>* good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);

  // SIFT descriptors are CV_32F, so the default FLANN configuration
  // (KD-tree forest + L2 distance) is the right one.
  FlannBasedMatcher flann_matcher;

  static constexpr float kLoweRatio = 0.8f;

  matches->clear();
  good_matches->clear();

  // 1. Best two neighbours for every descriptor of image 1.
  flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);

  // 2. Lowe's ratio test.
  for (const auto& match : *matches) {
    if (match.size() < 2) {
      // Fewer than two neighbours (tiny image 2): cannot run the ratio test.
      continue;
    }
    if (match[0].distance < kLoweRatio * match[1].distance) {
      good_matches->push_back(match[0]);
    }
  }
  VLOG(1) << "SIFT: " << matches->size() << " knn matches, "
          << good_matches->size() << " pass the ratio test";
}
