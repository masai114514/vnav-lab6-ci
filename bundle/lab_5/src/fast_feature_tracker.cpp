#include "fast_feature_tracker.h"

#include <glog/logging.h>
#include <opencv2/features2d.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Lab 5 | Deliverable 6.a (optional) - FAST + BRIEF
//
//  A "hand made" detector/descriptor pair: FAST only finds corners (no
//  scale/orientation, no descriptor), BRIEF only describes (binary, 256 bit).
//  FAST is by far the cheapest detector of the four; BRIEF is very fast but
//  not rotation invariant and not scale invariant, so the matching quality on
//  a scaled pair of images is expected to be the worst of the four -- which is
//  exactly the point of the comparison.
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

using namespace cv;
using namespace cv::xfeatures2d;

FastFeatureTracker::FastFeatureTracker()
    : detector(FastFeatureDetector::create()),
      descriptor_extractor(BriefDescriptorExtractor::create()) {
  tracker_name_ = "fast_brief";
}

void FastFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
  CHECK_NOTNULL(keypoints);
  detector->detect(img, *keypoints);
  VLOG(1) << "FAST detected " << keypoints->size() << " keypoints";
}

void FastFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors);
  // BRIEF: N x 32 bytes, CV_8U. Keypoints too close to the border have no valid
  // sampling pattern, so compute() removes them -- hence the size check below
  // must be relaxed to "descriptors are a subset of keypoints".
  descriptor_extractor->compute(img, *keypoints, *descriptors);
  CHECK_GE(keypoints->size(), static_cast<size_t>(descriptors->rows));
}

void FastFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
  CHECK_NOTNULL(matches);
  CHECK_NOTNULL(good_matches);

  // BRIEF descriptors are binary -> LSH index, Hamming metric.
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
  VLOG(1) << "FAST+BRIEF: " << matches->size() << " knn matches, "
          << good_matches->size() << " pass the ratio test";
}
