#include "orb_feature_tracker.h"

#include <glog/logging.h>
#include <opencv2/features2d.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Lab 5 | Deliverable 6.a - ORB
//
//  ORB = Oriented FAST + Rotated BRIEF. It is ~2 orders of magnitude cheaper
//  than SIFT/SURF and its binary descriptor is the one used by ORB-SLAM, hence
//  the interest for VNAV.
//
//  Two practical points:
//    * descriptors are CV_8U (binary, 256 bit = 32 bytes) -> FLANN must use the
//      LSH index ("locality sensitive hashing") with a Hamming metric, not the
//      default KD-tree, which only works for float descriptors.
//    * the ORB detector is created with nlevels = 1, as suggested in the
//      handout: it prevents the "Unknown interpolation method in function
//      'resize'" failure of the pyramid code on some OpenCV builds.
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

using namespace cv;
using namespace cv::xfeatures2d;

OrbFeatureTracker::OrbFeatureTracker()
    : detector(ORB::create(500, 1.2f, 1)) { // nfeatures, scaleFactor, nlevels
  tracker_name_ = "orb";
}

void OrbFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
  VLOG(1) << "ORB detected " << keypoints->size() << " keypoints";
}

void OrbFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  // ORB descriptors: N x 32 bytes, CV_8U.
  detector->compute(img, *keypoints, *descriptors);
  CHECK_EQ(keypoints->size(), static_cast<size_t>(descriptors->rows));
}

void OrbFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);

  // Binary descriptors -> LSH index (table_number, key_size, multi_probe_level)
  FlannBasedMatcher flann_matcher(new flann::LshIndexParams(20, 10, 2));
  static constexpr float kLoweRatio = 0.8f;

  matches->clear();
  good_matches->clear();

  flann_matcher.knnMatch(descriptors_1, descriptors_2, *matches, 2);

  for (const auto &match : *matches) {
    if (match.size() < 2) {
      continue;
    }
    if (match[0].distance < kLoweRatio * match[1].distance) {
      good_matches->push_back(match[0]);
    }
  }
  VLOG(1) << "ORB: " << matches->size() << " knn matches, "
          << good_matches->size() << " pass the ratio test";
}
