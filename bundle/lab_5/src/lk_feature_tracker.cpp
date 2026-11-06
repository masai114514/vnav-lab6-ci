#include "lk_feature_tracker.h"

#include <algorithm>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Lab 5 | Deliverable 7 - Lucas-Kanade sparse optical flow
//
//  Unlike SIFT/SURF/ORB/FAST, LK does *not* describe-and-match: it follows the
//  very same physical points from frame k to frame k+1 by assuming
//    (i)   brightness constancy  I(x, y, t) = I(x + dx, y + dy, t + dt),
//    (ii)  small motion           (the flow is linearised in a window),
//    (iii) spatial coherence      (all pixels of a window share one motion).
//  Corners are the features that satisfy (ii) best in both directions, which
//  is why we use a Harris-style detector (goodFeaturesToTrack) and not FAST.
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

using namespace cv;
using namespace cv::xfeatures2d;

namespace {
// Harris/Shi-Tomasi corner detector parameters (cv::goodFeaturesToTrack).
constexpr int kMaxCorners = 500;
constexpr double kQualityLevel = 0.01;
constexpr double kMinDistance = 7.0;
// Re-detect corners when fewer than this many tracks survive.
constexpr size_t kMinTrackedPoints = 20;
// cv::calcOpticalFlowPyrLK window / pyramid parameters.
constexpr int kLkWindowSize = 21;
constexpr int kLkMaxLevel = 3;
}  // namespace

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  // interactive:=false 时（CI / 无显示器）不建窗口 —— namedWindow 在 GTK 初始化
  // 失败时会直接终止进程，轨迹图改由 show() 落盘成 PNG。
  interactive_ = ros::param::param<bool>("~interactive", true);
  if (interactive_) {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
  }
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  if (interactive_) {
    cv::destroyWindow(window_name_);
  }
}

/** Tracks the Harris corners of the previous frame into the current frame.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  // ---------------------------------------------------------------------------
  // 0. Gray level image, because LK works on intensities.
  cv::Mat gray;
  if (frame.channels() == 3) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else if (frame.channels() == 4) {
    cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
  } else {
    gray = frame;
  }

  // ---------------------------------------------------------------------------
  // 1. First frame (or too few surviving tracks): detect Harris corners.
  if (prev_frame_.empty() || prev_corners_.size() < kMinTrackedPoints) {
    cv::goodFeaturesToTrack(gray, prev_corners_, kMaxCorners, kQualityLevel,
                            kMinDistance);
    prev_frame_ = gray.clone();
    VLOG(1) << "LK: re-detected " << prev_corners_.size() << " Harris corners";
    return;  // Nothing to track yet.
  }

  // ---------------------------------------------------------------------------
  // 2. Track the previous corners into the current frame (pyramidal LK).
  std::vector<cv::Point2f> next_corners;
  std::vector<uchar> status;  // 1 = tracked, 0 = lost
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prev_frame_, gray, prev_corners_, next_corners, status,
                           err,
                           cv::Size(kLkWindowSize, kLkWindowSize), kLkMaxLevel);

  std::vector<cv::Point2f> prev_tracked, curr_tracked;
  prev_tracked.reserve(prev_corners_.size());
  curr_tracked.reserve(prev_corners_.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      prev_tracked.push_back(prev_corners_[i]);
      curr_tracked.push_back(next_corners[i]);
    }
  }

  // ---------------------------------------------------------------------------
  // 3. How many of the tracked pairs are consistent with the epipolar geometry?
  std::vector<uchar> inlier_mask;
  const size_t num_matches = prev_tracked.size();
  if (num_matches >= 8) {
    inlierMaskComputation(prev_tracked, curr_tracked, &inlier_mask);
  } else {
    inlier_mask.assign(num_matches, static_cast<uchar>(1));
  }
  if (inlier_mask.size() != num_matches) {
    inlier_mask.assign(num_matches, static_cast<uchar>(0));
  }
  const unsigned int num_inliers =
      static_cast<unsigned int>(cv::countNonZero(inlier_mask));

  // ---------------------------------------------------------------------------
  // 4. Visualisation: draw the displacement of every tracked point.
  show(frame, prev_tracked, curr_tracked);

  // ---------------------------------------------------------------------------
  // 5. Statistics, same accounting as FeatureTracker::trackFeatures so that the
  //    Harris + LK row can be appended to the table.
  //    - "keypoints img1" = corners that were carried over from the previous
  //      frame (the detector output for frame k-1),
  //    - "keypoints img2" = points that survived tracking into frame k,
  //    - "matches" = "good matches" = successfully tracked pairs (LK does not
  //      produce ambiguous matches, so the Lowe ratio test has no meaning),
  //    - "inliers" = pairs consistent with the fundamental matrix.
  const float new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  const float old_num_samples = static_cast<float>(num_samples_);
  const float inlier_ratio =
      num_matches == 0 ? 0.0f
                       : static_cast<float>(num_inliers) / static_cast<float>(num_matches);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(prev_corners_.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(num_matches)) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(num_matches)) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  avg_inlier_ratio_ = (avg_inlier_ratio_ * old_num_samples + inlier_ratio) / new_num_samples;
  ++num_samples_;

  // ---------------------------------------------------------------------------
  // 6. Hand the current frame over to the next iteration. Keep all the points
  //    that were tracked (not only the inliers) so that a temporary outlier
  //    does not kill a track; re-detection happens in step 1 when needed.
  prev_corners_ = curr_tracked;
  prev_frame_ = gray.clone();
}

/** Display image with tracked features from prev to curr on 'frame'.
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  cv::Mat canvas;
  if (frame.channels() == 1) {
    cv::cvtColor(frame, canvas, cv::COLOR_GRAY2BGR);
  } else {
    canvas = frame.clone();
  }

  const size_t num_points = std::min(prev.size(), curr.size());
  for (size_t i = 0; i < num_points; ++i) {
    // Track: previous position -> current position.
    cv::line(canvas, prev[i], curr[i], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    // Heads of the tracks.
    cv::circle(canvas, curr[i], 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
  }

  // In `mode:=0` (pair of images) block until a key, otherwise play on.
  const int mode = ros::param::param<int>("~mode", 1);

  // 轨迹图落盘：交付物 D6/D7 要的是「图」，窗口截图既不可复现也不适合放进报告。
  // 每 lk_save_every 帧存一张（默认 20；设 0 关闭）；output_dir 为空则完全不写盘。
  const std::string output_dir =
      ros::param::param<std::string>("~output_dir", std::string("lab5_output"));
  if (!output_dir.empty()) {
    static int frame_id = 0;
    const int save_every = ros::param::param<int>("~lk_save_every", 20);
    if (save_every > 0 && frame_id % save_every == 0) {
      if (!cv::utils::fs::exists(output_dir)) {
        cv::utils::fs::createDirectory(output_dir);
      }
      char path[1024];
      std::snprintf(path, sizeof(path), "%s/lk_tracks_%04d.png",
                    output_dir.c_str(), frame_id);
      if (cv::imwrite(path, canvas)) {
        ROS_INFO("Saved figure: %s", path);
      } else {
        ROS_WARN("Could not write figure: %s", path);
      }
    }
    ++frame_id;
  }

  if (!interactive_) {
    return;
  }
  cv::imshow(window_name_, canvas);
  if (mode == 0) {
    while (ros::ok() && cv::waitKey(10) == -1) {
    }
  } else {
    cv::waitKey(10);
  }
}

/** Print the running averages (same fields as FeatureTracker::printStats). */
void LKFeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}
