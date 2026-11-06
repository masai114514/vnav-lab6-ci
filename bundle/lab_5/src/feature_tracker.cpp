#include "feature_tracker.h"

#include <vector>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include <ros/ros.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a generic feature tracker base class
//  which you will augment with several derived classes for SIFT, SURF, ORB, and
//  FAST+BRIEF feature tracking.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.

using namespace cv;
using namespace cv::xfeatures2d;

namespace {

// Draw `matches` on top of `canvas` with the given colour. `mask` (may be empty)
// selects which matches are drawn.
void DrawMatchesColored(const cv::Mat &img_1,
                        const std::vector<KeyPoint> &keypoints_1,
                        const cv::Mat &img_2,
                        const std::vector<KeyPoint> &keypoints_2,
                        const std::vector<DMatch> &matches,
                        const Scalar &color,
                        cv::Mat *canvas,
                        int flags,
                        const std::vector<char> &mask) {
  CHECK_NOTNULL(canvas);
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, *canvas,
                  color, color, mask, flags);
}

}  // namespace

/** This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] img_1, img_2 Images where to track features.
 @param[out] matched_kp_1_kp_2 pair of vectors of keypoints with the same size
 so that matched_kp_1_kp_2.first[i] matches with matched_kp_1_kp_2.second[i].
*/
void FeatureTracker::trackFeatures(const cv::Mat &img_1,
                                   const cv::Mat &img_2,
                                   std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> *matched_kp_1_kp_2) {

  const int mode = ros::param::param<int>("~mode", 0);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 3 | Feature Descriptors (SIFT)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For your convenience,
  // here are two vectors of keypoints representing keypoints for image 1 and 2.
  std::vector<KeyPoint> keypoints_1, keypoints_2;

  // ~~~~ begin solution
  //   1. Detect the keypoints for each image independently.
  detectKeypoints(img_1, &keypoints_1);
  detectKeypoints(img_2, &keypoints_2);

  //   2. Display detected keypoints on both images, and save them for the
  //   deliverable.
  if (mode == 0) {
    cv::Mat keypoints_img_1, keypoints_img_2;
    cv::drawKeypoints(img_1, keypoints_1, keypoints_img_1, Scalar::all(-1),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(img_2, keypoints_2, keypoints_img_2, Scalar::all(-1),
                      DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    saveImage(keypoints_img_1, "keypoints_img1");
    saveImage(keypoints_img_2, "keypoints_img2");
    showImage(keypoints_img_1, "keypoints_img1");
    showImage(keypoints_img_2, "keypoints_img2");
  }
  // ~~~~ end solution

  //   3. Compute descriptors by filling in the skeleton function
  //   `describeKeypoints` in the relevant derived classes.
  cv::Mat descriptors_1, descriptors_2;
  // ~~~~ begin solution
  describeKeypoints(img_1, &keypoints_1, &descriptors_1);
  describeKeypoints(img_2, &keypoints_2, &descriptors_2);
  CHECK_EQ(keypoints_1.size(), static_cast<size_t>(descriptors_1.rows))
      << "Detected/described keypoint count mismatch for image 1";
  CHECK_EQ(keypoints_2.size(), static_cast<size_t>(descriptors_2.rows))
      << "Detected/described keypoint count mismatch for image 2";
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 3
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 | Descriptor-based Feature Matching
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  // Matches and good matches to be populated
  std::vector<std::vector<DMatch>> matches;
  std::vector<DMatch> good_matches;

  //   1. Match descriptor vectors using the FLANN matcher.
  // ~~~~ begin solution
  if (!descriptors_1.empty() && !descriptors_2.empty()) {
    matchDescriptors(descriptors_1, descriptors_2, &matches, &good_matches);
  } else {
    ROS_WARN("Empty descriptor matrix, skipping matching.");
  }
  // ~~~~ end solution

  //   2. Plot the matches using the opencv function 'drawMatches'.
  // ~~~~ begin solution
  // For a knnMatch result the "raw" match set is the first (best) neighbour of
  // every query descriptor, i.e. what a plain 1-NN match would return.
  std::vector<DMatch> knn_matches;
  knn_matches.reserve(matches.size());
  for (const auto &match_pair : matches) {
    if (!match_pair.empty()) {
      knn_matches.push_back(match_pair.front());
    }
  }

  if (mode == 0) {
    cv::Mat matches_img;
    DrawMatchesColored(img_1, keypoints_1, img_2, keypoints_2, knn_matches,
                       Scalar::all(-1), &matches_img,
                       DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, std::vector<char>());
    saveImage(matches_img, "matches_all_knn");
    showImage(matches_img, "matches_all_knn");

    cv::Mat good_matches_img;
    DrawMatchesColored(img_1, keypoints_1, img_2, keypoints_2, good_matches,
                       Scalar::all(-1), &good_matches_img,
                       DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, std::vector<char>());
    saveImage(good_matches_img, "matches_good_lowe");
    showImage(good_matches_img, "matches_good_lowe");
  }
  // ~~~~ end solution

  //   3. Use the function 'inlierMaskComputation' to get the inliers amongst
  //   the matches you computed. The two keypoint lists handed to that function
  //   must be *ordered by match* and have the same size.
  // ~~~~ begin solution
  std::vector<KeyPoint> matched_keypoints_1, matched_keypoints_2;
  matched_keypoints_1.reserve(good_matches.size());
  matched_keypoints_2.reserve(good_matches.size());
  for (const auto &match : good_matches) {
    matched_keypoints_1.push_back(keypoints_1[match.queryIdx]);
    matched_keypoints_2.push_back(keypoints_2[match.trainIdx]);
  }

  std::vector<uchar> inlier_mask;
  if (matched_keypoints_1.size() >= 8) {
    inlierMaskComputation(matched_keypoints_1, matched_keypoints_2, &inlier_mask);
  } else {
    ROS_WARN("Less than 8 good matches, skipping the epipolar inlier test.");
    inlier_mask.assign(matched_keypoints_1.size(), static_cast<uchar>(1));
  }
  if (inlier_mask.size() != good_matches.size()) {
    // findFundamentalMat returns an empty mask when it cannot fit a model.
    inlier_mask.assign(good_matches.size(), static_cast<uchar>(0));
  }
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 4
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 5 | Keypoint Matching Quality
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  unsigned int num_inliers;
  //   1. Draw the inlier (green) and outlier (red) matches. Draw all the good
  //   matches in red first, then re-use the same image to paint the inliers in
  //   green on top of it.
  // ~~~~ begin solution
  {
    cv::Mat inliers_img;
    DrawMatchesColored(img_1, keypoints_1, img_2, keypoints_2, good_matches,
                       Scalar(0, 0, 255), &inliers_img,
                       DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS, std::vector<char>());

    const std::vector<char> inlier_mask_char(inlier_mask.begin(), inlier_mask.end());
    DrawMatchesColored(img_1, keypoints_1, img_2, keypoints_2, good_matches,
                       Scalar(0, 255, 0), &inliers_img,
                       DrawMatchesFlags::DRAW_OVER_OUTIMG, inlier_mask_char);

    saveImage(inliers_img, "matches_inliers_outliers");
    showImage(inliers_img, "matches_inliers_outliers");

    num_inliers = static_cast<unsigned int>(cv::countNonZero(inlier_mask));
  }
  // ~~~~ end solution

  if (matched_kp_1_kp_2 != nullptr) {
    matched_kp_1_kp_2->first = matched_keypoints_1;
    matched_kp_1_kp_2->second = matched_keypoints_2;
  }

  //   2. Calculate the statistics to fill the table in the handout.
  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  const float inlier_ratio =
      good_matches.empty()
          ? 0.0f
          : static_cast<float>(num_inliers) / static_cast<float>(good_matches.size());
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(keypoints_1.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(keypoints_2.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(matches.size())) / new_num_samples;
  avg_num_good_matches_ = (avg_num_good_matches_ * old_num_samples + static_cast<float>(good_matches.size())) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  avg_inlier_ratio_ = (avg_inlier_ratio_ * old_num_samples + inlier_ratio) / new_num_samples;
  ++num_samples_;

  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 5
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
   *  Both keypoints_1 and keypoints_2 input parameters must be ordered by match
   * i.e. keypoints_1[0] has been matched to keypoints_2[0].
   * Therefore, both keypoints vectors must have the same length.
    @param[in] keypoints_1 List of keypoints detected on the first image.
    @param[in] keypoints_2 List of keypoints detected on the second image.
    @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
  */
void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint> &keypoints_1,
                                           const std::vector<KeyPoint> &keypoints_2,
                                           std::vector<uchar> *inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob, *inlier_mask);
  } catch (...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

/** Example of function to draw matches. Feel free to re-use this example or
 *  create your own. You will need to modify it in order to plot the different
 *  figures. You can add more functions to this class if needed.
 */
void FeatureTracker::drawMatches(const cv::Mat &img_1,
                                 const cv::Mat &img_2,
                                 const std::vector<KeyPoint> &keypoints_1,
                                 const std::vector<KeyPoint> &keypoints_2,
                                 const std::vector<std::vector<DMatch>> &matches) {
  cv::Mat img_matches;
  cv::drawMatches(img_1,
                  keypoints_1,
                  img_2,
                  keypoints_2,
                  matches,
                  img_matches,
                  Scalar::all(-1),
                  Scalar::all(-1),
                  std::vector<std::vector<char>>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  // Show detected matches
  showImage(img_matches, "tracked_features");
}

/** Show an image in a window, blocking or not depending on the running mode. */
void FeatureTracker::showImage(const cv::Mat &img, const std::string &window_name) const {
  // Headless（CI / 纯 SSH / WSL 没接 WSLg）时关掉交互：图已经被 saveImage 存成
  // PNG，再开窗口只会阻塞等待按键，或者在 GTK 初始化时直接抛异常。
  // 参数在 launch 里给：<param name="interactive" value="false"/>。
  if (!ros::param::param<bool>("~interactive", true)) {
    return;
  }
  const int mode = ros::param::param<int>("~mode", 0);
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, img);
  if (mode == 0) {
    // Wait for a key press, but let ROS also kill everything (instead of
    // waitKey(0) which would block roslaunch forever).
    while (ros::ok() && cv::waitKey(10) == -1) {
    }
  } else {
    // Video sequence: play continuously instead of frame-by-frame.
    cv::waitKey(10);
  }
}

/** Save a figure under `output_dir` (private ROS param, default ./lab5_output). */
void FeatureTracker::saveImage(const cv::Mat &img, const std::string &suffix) const {
  std::string output_dir;
  ros::param::param<std::string>("~output_dir", output_dir, std::string("lab5_output"));
  if (output_dir.empty()) {
    return;
  }
  if (!cv::utils::fs::exists(output_dir)) {
    cv::utils::fs::createDirectory(output_dir);
  }
  const std::string path = output_dir + "/" + tracker_name_ + "_" + suffix + ".png";
  if (cv::imwrite(path, img)) {
    ROS_INFO("Saved figure: %s", path.c_str());
  } else {
    ROS_WARN("Could not write figure: %s", path.c_str());
  }
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
