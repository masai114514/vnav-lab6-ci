/*
 * @file pose_estimation.cpp
 * @brief Estimates the pose from frame to frame.
 *
 * LAB 6 —— 无人机视觉定位（Motion estimation / Relative pose）
 * 本文件在官方骨架上完成了 D3（相机标定与去畸变）、D4（2D-2D 相对位姿：
 * 5 点、8 点、2 点）与 D5（3D-3D：Arun 3 点法），并补上了 RPE 评估与 CSV 落盘。
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>
#include <string>

#include <glog/logging.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// OpenGV
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/point_cloud/methods.hpp>

#include "lab6_utils.h"
#include "pose_estimation.h"
#include "tracker_shim.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2021  - Lab 6 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to use the following methods for motion estimation:
//  - Nister's 5 point algorithm (2D-2D correspondences)
//  - Longuet-Higgins 8 point algorithm (2D-2D correspondences)
//  - OpenGV's 2 point algorithm (2D-2D correspondences, known rotation)
//  - Arun's 3 point algorithm (3D-3D correspondences)
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Options.
//
// 官方骨架用 gflags（Google Flags）传参。我们改成 ROS 私有参数（~xxx），
// 原因有两个：
//   1) 与 ROS 生态一致，launch 文件里写 <param> 就能改，不必拼命令行；
//   2) 实测在 melodic 下 gflags 的全局 std::string（DEFINE_string）与 roscpp
//      的 std::string 跨库传递会把堆踩坏，启动后第一次 ros::NodeHandle::hasParam()
//      就 munmap_chunk(): invalid pointer 崩掉。用 param 可以完全避开。
struct Options {
  bool use_ransac = true;                 // 是否启用 RANSAC
  bool scale_translation = true;          // 2D-2D 的平移只有方向，按真值缩放便于统计
  int pose_estimator = 0;                 // 0: 5pt  1: 8pt  2: 2pt  3: Arun
  double ransac_threshold_bearing = 1e-4; // 2D-2D 的 1-cos 阈值
  double ransac_threshold_3d = 0.03;      // 3D-3D 的米制阈值
  int ransac_max_iterations = 2000;
  double ransac_probability = 0.99;
  bool refine = true;                     // 用内点做一次非线性精修
  std::string csv_path;                   // 逐帧 RPE 输出
  int max_frames = 0;                     // 0 = 不限制
};

static Options g_opt;

using namespace std;
namespace enc = sensor_msgs::image_encodings;

// Global definitions.
// Mono OpenGV RANSAC problems and adapters.
// See OpenGV examples on how to use:
// https://laurentkneip.github.io/opengv/page_how_to_use.html
using RansacProblem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using RansacProblemGivenRot = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using AdapterGivenRot = opengv::relative_pose::CentralRelativeAdapter;
using Adapter3D = opengv::point_cloud::PointCloudAdapter;
using RansacProblem3D = opengv::sac_problems::point_cloud::PointCloudSacProblem;

// Global variables. As a general coding style, global variables are followed
// by a trailing underscore (Google coding style).
std::unique_ptr<TrackerWrapper> feature_tracker_;
ros::Publisher pub_pose_estimation_, pub_pose_gt_;
ros::Subscriber sub_cinfo_;
geometry_msgs::PoseStamped curr_pose_;
geometry_msgs::PoseStamped prev_pose_;

// See definition of CameraParams in lab6_utils.h
// We declare these global variables, and we assign values to them later in the main function
CameraParams camera_params_;
cv::Mat R_camera_body, t_camera_body;
cv::Mat T_camera_body;
geometry_msgs::Pose pose_camera_body;
tf::Transform transform_camera_body;

// ---- 我们额外维护的状态 -----------------------------------------------------
std::ofstream g_csv;            // 逐帧 RPE 统计的落盘句柄
int g_frame_id = 0;             // 当前帧序号（从 0 开始）
int g_num_inliers = -1;         // 本帧 RANSAC 内点数（-1 表示该方法不用 RANSAC）
double g_rot_err_deg = 0.0;     // 旋转误差（度），便于对照讲义里 "1 度" 的口径
double g_rot_err_chordal = 0.0; // 旋转误差的 chordal 距离 ||R_gt - R_est||_F
double g_trans_err = 0.0;       // 平移误差（2D-2D 归一化后比较方向）
double g_depth_scale = 5000.0;  // 16 位深度图的量化系数（TUM: 5000 单位 = 1 米）
double g_min_depth = 0.3;       // 三维点有效深度下限（米）
double g_max_depth = 8.0;       // 三维点有效深度上限（米）


/**
 * @brief 把旋转矩阵之差换算成角度（度）
 *
 * 用单位四元数从 R_gt^T R_est 提取旋转角，等价于 geodesic distance。
 * 之所以同时记一份角度，是因为讲义给的验收标准是"旋转误差小于 1 度"。
 */
double rotationErrorDegrees(const Eigen::Matrix3d& R_gt, const Eigen::Matrix3d& R_est) {
  Eigen::Quaterniond q(Eigen::Matrix3d(R_gt.transpose() * R_est));
  q.normalize();
  double const angle = 2.0 * std::acos(std::min(1.0, std::fabs(q.w())));
  return angle * 180.0 / M_PI;
}


/** @brief 在 RANSAC 的内点集上精修模型（OpenGV 的 optimizeModelCoefficients）
 *
 * RANSAC 只负责"把外点挑掉"，它返回的是某一次最小采样的模型，精度有限。
 * 我们在此基础上调用 sac_model_->optimizeModelCoefficients，它内部会跑
 * relative_pose::optimize_nonlinear（或 point_cloud 对应的非线性优化），
 * 用全部内点把 R、t 再优化一遍。短基线数据上这一步能明显降误差。
 */
template <typename RansacT>
void refineRansacModel(RansacT& ransac, size_t min_inliers) {
  if (!g_opt.refine) return;
  if (ransac.inliers_.size() < min_inliers) return;
  opengv::transformation_t refined;
  ransac.sac_model_->optimizeModelCoefficients(
      ransac.inliers_, ransac.model_coefficients_, refined);
  ransac.model_coefficients_ = refined;
}


void poseCallbackTesse(const nav_msgs::Odometry::ConstPtr& msg){
  // This function takes a message from the simulator of type nav_msgs::Odometry,
  // and use it to update the ground-truth pose of the drone's camera frame.
  // 
  // Note that the simulator provides the pose of the drone's body frame,
  // Therefore, we need to use the fixed transformation from the drone's camera
  // to the drone's camera to obtain the pose of the drone's camera frame.
  // 
  // In math, we want to get T_{camera}^W, but the msg provides T_{body}^W,
  // so we should do T_{camera}^W = T_{body}^W * T_{camera}^{body} to get the 
  // drone's camera pose.

  // This is T_{body}^W
  curr_pose_.pose = msg->pose.pose;

  // Convert pose message to tf::Transform
  tf::Transform current_pose;
  tf::poseMsgToTF(curr_pose_.pose, current_pose);

  // Perform the coordinate transform T_{camera}^W = T_{body}^W * T_{camera}^{body}
  // and convert to pose message
  tf::poseTFToMsg(current_pose * transform_camera_body, curr_pose_.pose);

  // publish the converted pose message so we can visualize in rViz
  curr_pose_.header.frame_id = "world";
  pub_pose_gt_.publish(curr_pose_);
}

/**
 * @brief      Compute 3D bearing vectors from pixel points
 *
 * @param[in]  pts1              Feature correspondences from camera 1
 * @param[in]  pts2              Feature correspondences from camera 2
 * @param      bearing_vector_1  Bearing vector to pts1 in camera 1
 * @param      bearing_vector_2  Bearing vector to pts2 in camera 2
 */
void calibrateKeypoints(const std::vector<cv::Point2f>& pts1,
                        const std::vector<cv::Point2f>& pts2,
                        opengv::bearingVectors_t& bearing_vector_1,
                        opengv::bearingVectors_t& bearing_vector_2) {
    //
    // For this part, we perform:
    //   1. Use the function cv::undistortPoints to rectify the keypoints.
    //   2. Return the bearing vectors for each keypoint.
    //
    // DELIVERABLE 3 | Initial Setup
    // 相机内参与畸变系数已经在 main() 里填好（官方给的 Tesse 标定值，或由 ROS 参数
    // 覆盖）。这里把像素坐标先做去畸变、归一化到 z=1 的成像平面上，再转成单位
    // 观测向量。注意：OpenGV 内部只认单位向量，焦距必须在这里被"除掉"，
    // 否则 RANSAC 阈值（1-cos(alpha)）的口径就对不上了。
    //

    std::vector<cv::Point2f> points1_rect, points2_rect;
    cv::undistortPoints(pts1, points1_rect, camera_params_.K, camera_params_.D);
    cv::undistortPoints(pts2, points2_rect, camera_params_.K, camera_params_.D);

    for (auto const& pt: points1_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_1.push_back(bearing_vector.normalized());
    }

    for (auto const& pt: points2_rect){
      opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
      bearing_vector_2.push_back(bearing_vector.normalized());
    }
}

/**
 * @brief      Update pose estimate using previous absolue pose and estimated relative pose
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  relative_pose     estimated relative pose between current frame and previous frame
 * @param      output            estimated absolute pose of current frame
 */
void updatePoseEstimate(geometry_msgs::Pose const& prev_pose, geometry_msgs::Pose const& relative_pose, geometry_msgs::Pose& output) {
  tf::Transform prev, relative;
  tf::poseMsgToTF(prev_pose, prev);
  tf::poseMsgToTF(relative_pose, relative);
  tf::poseTFToMsg(prev*relative, output);
}

/**
 * @brief      Given an estimated translation up to scale, return an absolute translation with correct scale using ground truth
 *
 * @param[in]  prev_pose         ground-truth absolute pose of previous frame
 * @param[in]  curr_pose         ground-truth absolute pose of current frame
 * @param      translation       estimated translation between current frame and previous frame
 */
void scaleTranslation(geometry_msgs::Point& translation, geometry_msgs::PoseStamped const& prev_pose, geometry_msgs::PoseStamped const& curr_pose) {
  if (!g_opt.scale_translation) return;
  tf::Transform prev, curr;
  tf::poseMsgToTF(prev_pose.pose, prev);
  tf::poseMsgToTF(curr_pose.pose, curr);
  tf::Transform const relative_pose = prev.inverseTimes(curr);
  double const translation_scale = relative_pose.getOrigin().length();
  if (isnan(translation_scale) || isinf(translation_scale)) {
    ROS_WARN("Failed to scale translation");
    return;
  }
  double const old_scale = sqrt(pow(translation.x, 2) + pow(translation.y, 2) + pow(translation.z, 2));
  if (old_scale < 1e-12) return;   // 估计出来的平移几乎为零，缩放没有意义
  translation.x *= translation_scale / old_scale;
  translation.y *= translation_scale / old_scale;
  translation.z *= translation_scale / old_scale;
}


/** @brief     Compute Relative Pose Error (RPE) in translation and rotation.
    @param[in] gt_t_prev_frame ground-truth transform for previous frame.
    @param[in] gt_t_curr_frame ground-truth transform for current frame.
    @param[in] est_t_prev_frame estimated transform for previous frame.
    @param[in] est_t_curr_frame estimated transform for current frame.
*/
void evaluateRPE(const tf::Transform& gt_t_prev_frame,
                 const tf::Transform& gt_t_curr_frame,
                 const tf::Transform& est_t_prev_frame,
                 const tf::Transform& est_t_curr_frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4 (Part 2) | Evaluate Relative Pose Error (RPE)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. Follow the equations given in the README to compute the relative pose
  //     error at each time frame
  //   2. Print the error in translation and rotation.
  //   3. For plotting the statics, you might want to log the errors in a .txt or .csv file
  //
  // Hint: You might find tf::vectorTFToEigen and tf::matrixTFToEigen helpful

  tf::Transform const est_relative_pose = est_t_prev_frame.inverseTimes(est_t_curr_frame);
  tf::Transform const gt_relative_pose = gt_t_prev_frame.inverseTimes(gt_t_curr_frame);

  // ***************************** begin solution *****************************

  // 1) 估计与真值的"相对位姿"。注意两者都定义在上一帧的相机坐标系里：
  //    T^{prev}_{curr} = (T^W_prev)^{-1} T^W_curr（2D-2D 与 3D-3D 同口径）。
  Eigen::Vector3d t_est, t_gt;
  tf::vectorTFToEigen(est_relative_pose.getOrigin(), t_est);
  tf::vectorTFToEigen(gt_relative_pose.getOrigin(), t_gt);

  // 2) 平移误差。2D-2D（5/8/2 点）只能恢复到尺度，估计出来的平移模长恒为 1，
  //    因此按讲义要求把真值平移也归一化，比较的是"方向"而不是"长度"。
  //    3D-3D（Arun）没有尺度歧义，直接用原始平移比较。
  if (g_opt.pose_estimator < 3) {
    if (t_gt.norm() > 1e-12) t_gt.normalize();
    if (t_est.norm() > 1e-12) t_est.normalize();
  }
  g_trans_err = (t_gt - t_est).norm();

  // 3) 旋转误差：chordal distance ||R_gt - R_est||_F（讲义给的公式），
  //    另外换算一个以度为单位的 geodesic 角，方便对照验收标准。
  Eigen::Matrix3d R_est, R_gt;
  tf::matrixTFToEigen(est_relative_pose.getBasis(), R_est);
  tf::matrixTFToEigen(gt_relative_pose.getBasis(), R_gt);
  g_rot_err_chordal = (R_gt - R_est).norm();
  g_rot_err_deg = rotationErrorDegrees(R_gt, R_est);

  std::cout << "[RPE] frame " << g_frame_id
            << "  rot = " << std::fixed << std::setprecision(3) << g_rot_err_deg << " deg"
            << " (" << std::setprecision(5) << g_rot_err_chordal << " chordal)"
            << "  trans = " << std::setprecision(4) << g_trans_err
            << std::endl;

  // ***************************** end solution *****************************

}

/** @brief 深度图统一转成 float（米），并区分无效值
 *
 * 官方 office.bag 的深度是 32FC1、单位米；TUM RGB-D 的深度是 16UC1、
 * 1 单位 = 1/5000 米，"0" 表示该像素没有深度。为了让节点对两种数据都能跑，
 * 这里统一转成 CV_32F 的米制深度图。
 */
cv::Mat depthToMeters(const cv::Mat& depth_raw) {
  if (depth_raw.empty()) return cv::Mat();
  if (depth_raw.type() == CV_32F) return depth_raw;
  if (depth_raw.type() == CV_16U) {
    cv::Mat meters;
    depth_raw.convertTo(meters, CV_32F, 1.0 / g_depth_scale);
    return meters;
  }
  cv::Mat meters;
  depth_raw.convertTo(meters, CV_32F);
  return meters;
}

/** @brief 最近邻取深度，越界或非有限值返回 0（视为无效） */
inline double depthAt(const cv::Mat& depth_m, const cv::Point2f& pt) {
  int const u = static_cast<int>(std::lround(pt.x));
  int const v = static_cast<int>(std::lround(pt.y));
  if (u < 0 || v < 0 || u >= depth_m.cols || v >= depth_m.rows) return 0.0;
  float const d = depth_m.at<float>(v, u);
  if (!std::isfinite(d)) return 0.0;
  return static_cast<double>(d);
}

/** @brief OpenCV 的 [R|t]（x2 = R x1 + t）转成 OpenGV 的 [R|t]（x1 = R x2 + t）
 *
 * OpenCV 的 recoverPose 返回"从相机 1 到相机 2"的变换，OpenGV 的相对位姿
 * 返回的是"相机 2 在相机 1 下的位置"。两者互为逆：T_ogv = T_cv^{-1}。
 * 只有在不用 RANSAC 的那几条（可选）分支里会用到这个转换。
 */
geometry_msgs::Pose cvPoseToOpenGV(const cv::Mat& R_cv, const cv::Mat& t_cv) {
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(t_cv, t);
  Eigen::Matrix3d const R_inv = R.transpose();
  Eigen::Vector3d const t_inv = -R_inv * t;
  cv::Mat R_cv_inv, t_cv_inv;
  cv::eigen2cv(R_inv, R_cv_inv);
  cv::eigen2cv(t_inv, t_cv_inv);
  return cv2Pose(R_cv_inv, t_cv_inv);
}

/** @brief This function is called when a new image is published. This is
 *   where all the magic happens for this lab
 *  @param[in]  rgb_msg    RGB Camera message
 *  @param[in]  depth_msg  Depth Camera message
 */
void cameraCallback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 4-5 | A Suite of Methods for Motion Estimation
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //   1. (Deliverable 4) 2D-2D Correspondences
  //     - 5 point Nister (with RANSAC)
  //     - 8 point Longuet-Higgins (with RANSAC)
  //     - 2 point OpenGV (with RANSAC)
  //   2. (Deliverable 5) 3D-3D Correspondences
  //     - 3 point Arun (with and without RANSAC)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  cv::Mat bgr, depth;

  try {
    // Convert ROS msg type to OpenCV image type.
    bgr = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    depth = depthToMeters(cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert rgb or depth images.");
  }

  // for drawing
  cv::Mat view = bgr.clone();

  // cv::imshow("view", view);

  // The static keyword makes this variable to be created only once.
  // Therefore, this line of code will only be evaluated the first time
  // cameraCallback is called, but never again.
  // Nevertheless, the variable `prev_image` will continue to be available.
  // store images from previous time step
  static cv::Mat prev_bgr = bgr.clone();
  static cv::Mat prev_depth = depth.clone();

  // Track features returns the 2D-2D matches between images
  // in pixels (pixel coords in image 1 -> pixel coords in image 2).
  std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> matched_kp_1_kp_2;
  feature_tracker_->track(prev_bgr, bgr, &matched_kp_1_kp_2);

  int N = matched_kp_1_kp_2.first.size();
  std::cout << "Matched " << N << " keypoints" << std::endl;
  std::vector<cv::Point2f> pts1, pts2;
  
  cv::KeyPoint::convert(matched_kp_1_kp_2.first, pts1);
  cv::KeyPoint::convert(matched_kp_1_kp_2.second, pts2);

  // (TODO) undistort the image keypoints and convert them to bearing vectors 
  // using camera intrinsics
  // Use the calibrateKeypoints function defined
  // Store the results in bearing_vector_1 and bearing_vector_2 defined here.
  opengv::bearingVectors_t bearing_vector_1, bearing_vector_2;
  // ***************************** begin solution *****************************

  // DELIVERABLE 3：所有关键点必须先做去畸变、再转成单位观测向量，
  // 才能交给 RANSAC（否则畸变会让内点判定失真）。
  calibrateKeypoints(pts1, pts2, bearing_vector_1, bearing_vector_2);

  // ***************************** end solution *****************************

  // We create the central relative adapter, have a look at OpenGV's
  // documentation to understand how this class works.
  Adapter adapter_mono (bearing_vector_1, bearing_vector_2);

  // (TODO) Compute the pose estimate using the different techniques
  // here. You should fill the variable `pose_estimate` with your pose
  // estimation.
  geometry_msgs::PoseStamped pose_estimation;
  tf::poseTFToMsg(tf::Pose::getIdentity(), pose_estimation.pose);

  // initialize a relative pose estimate, you will update relative_pose_estimate using the following algorithms
  geometry_msgs::Pose relative_pose_estimate = pose_estimation.pose;
  g_num_inliers = -1;

  switch(g_opt.pose_estimator) {
  case 0: {
    // *********************************************************************************
    // ******************************** 5-point Method *********************************
    // *********************************************************************************

    // First, ensure we have the minimal number of correspondences to compute the relative pose.
    static constexpr size_t min_nr_of_correspondences = 5;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!g_opt.use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation

        // ***************************** begin solution *****************************

        // 只用前 5 个对应点解 Nister，得到一组本质矩阵候选；
        // 再用 extractPose 在所有对应点上用 recoverPose 选内点最多的那个解。
        opengv::bearingVectors_t bv1(bearing_vector_1.begin(), bearing_vector_1.begin() + 5);
        opengv::bearingVectors_t bv2(bearing_vector_2.begin(), bearing_vector_2.begin() + 5);
        Adapter const adapter_5pt(bv1, bv2);
        opengv::essentials_t const Es = opengv::relative_pose::fivept_nister(adapter_5pt);

        cv::Mat R_cv, t_cv;
        int const inliers = extractPose(Es, pts1, pts2, camera_params_, R_cv, t_cv);
        if (inliers > 0) {
          // OpenCV 的 [R|t] 与 OpenGV 的约定互逆，需要转换
          relative_pose_estimate = cvPoseToOpenGV(R_cv, t_cv);
          g_num_inliers = inliers;
        } else {
          ROS_WARN("5-point without RANSAC failed to recover a consistent pose.");
        }

        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``

        // ***************************** begin solution *****************************

        opengv::sac::Ransac<RansacProblem> ransac;
        ransac.sac_model_ = std::make_shared<RansacProblem>(adapter_mono, RansacProblem::NISTER);
        ransac.threshold_ = g_opt.ransac_threshold_bearing;
        ransac.max_iterations_ = g_opt.ransac_max_iterations;
        ransac.probability_ = g_opt.ransac_probability;

        if (ransac.computeModel() && ransac.inliers_.size() >= min_nr_of_correspondences) {
          refineRansacModel(ransac, min_nr_of_correspondences);
          // model_coefficients_ 是 OpenGV 的 transformation_t（3x4）：
          // x_prev = R * x_curr + t，正好与 tf 的 T^{prev}_{curr} 同口径。
          relative_pose_estimate = eigen2Pose(ransac.model_coefficients_);
          g_num_inliers = static_cast<int>(ransac.inliers_.size());
        } else {
          ROS_WARN("Nister's 5-point RANSAC did not find a valid model.");
        }

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Nister's algorithm.");
    }
    break;
  }
  case 1: {
    // *********************************************************************************
    // ******************************** 8-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 8;

    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!g_opt.use_ransac) {
        // Without RANSAC [OPTIONAL]
        // Consider using the "extractPose" function provided to obtain inliers, rotation, and translation
        // Consider using "cv2Pose(Rmat, tmat)" to recover the rotation Rmat and translation tmat

        // ***************************** begin solution *****************************

        // 不在 RANSAC 里的 8 点法就是"所有点一起做一次线性最小二乘"：
        // 先解基础矩阵 F，再用 E = K^T F K 转成本质矩阵（全程不采样、不剔点）。
        cv::Mat const F = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);
        cv::Mat const E = camera_params_.K.t() * F * camera_params_.K;
        cv::Mat R_cv, t_cv;
        int const inliers = cv::recoverPose(E, pts1, pts2, camera_params_.K, R_cv, t_cv);
        if (inliers > 0) {
          relative_pose_estimate = cvPoseToOpenGV(R_cv, t_cv);
          g_num_inliers = inliers;
        } else {
          ROS_WARN("8-point without RANSAC failed to recover a consistent pose.");
        }

       // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // Consider using ``eigen2Pose`` to convert the solution of ransac to ``relative_pose_estimate``


        // ***************************** begin solution *****************************

        // 与 5 点法共用同一个 sac problem，只是把采样器换成 EIGHTPT
        // （Longuet-Higgins 的 8 点法，最小样本 8 个点）。
        opengv::sac::Ransac<RansacProblem> ransac;
        ransac.sac_model_ = std::make_shared<RansacProblem>(adapter_mono, RansacProblem::EIGHTPT);
        ransac.threshold_ = g_opt.ransac_threshold_bearing;
        ransac.max_iterations_ = g_opt.ransac_max_iterations;
        ransac.probability_ = g_opt.ransac_probability;

        if (ransac.computeModel() && ransac.inliers_.size() >= min_nr_of_correspondences) {
          refineRansacModel(ransac, min_nr_of_correspondences);
          relative_pose_estimate = eigen2Pose(ransac.model_coefficients_);
          g_num_inliers = static_cast<int>(ransac.inliers_.size());
        } else {
          ROS_WARN("Longuet-Higgins' 8-point RANSAC did not find a valid model.");
        }

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to compute pose estimation using"
               " Longuet-Higgins' algorithm.");
    }
    break;
  }
  case 2: {
    // *********************************************************************************
    // ******************************** 2-point Method *********************************
    // *********************************************************************************

    static constexpr size_t min_nr_of_correspondences = 2;
    if (adapter_mono.getNumberCorrespondences() >= min_nr_of_correspondences) {
      
      // Obtain the rotation part from ground-truth
      tf::Transform curr_frame, prev_frame;
      tf::poseMsgToTF(curr_pose_.pose, curr_frame);
      tf::poseMsgToTF(prev_pose_.pose, prev_frame);
      Eigen::Matrix3d rotation;
      tf::matrixTFToEigen(prev_frame.inverseTimes(curr_frame).getBasis(), rotation);
      adapter_mono.setR12(rotation);

      // We only estimate the translation part
      if (!g_opt.use_ransac) {
        // Without RANSAC (OPTIONAL)
        // ***************************** begin solution *****************************

        // 直接取前两个对应点解平移（unrotate=true 表示先用 R12 把第二帧的观测
        // 向量转回第一帧，再解 t）。
        opengv::translation_t const t12 = opengv::relative_pose::twopt(adapter_mono, true);
        // transformation_t 是 3x4 的矩阵，Eigen 里对它没有无参 Identity()，
        // 这里直接把两个块都写满（12 个元素全覆盖，不会留下未初始化项）。
        opengv::transformation_t T;
        T.block<3, 3>(0, 0) = rotation;
        T.col(3) = t12;
        relative_pose_estimate = eigen2Pose(T);

        // ***************************** end solution *****************************
      } else {
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************

        // 2 点法的模型是 [R12 | t]，其中 R12 由适配器给定（真值旋转），
        // 因此 RANSAC 只对平移做采样与投票。
        opengv::sac::Ransac<RansacProblemGivenRot> ransac;
        ransac.sac_model_ = std::make_shared<RansacProblemGivenRot>(adapter_mono);
        ransac.threshold_ = g_opt.ransac_threshold_bearing;
        ransac.max_iterations_ = g_opt.ransac_max_iterations;
        ransac.probability_ = g_opt.ransac_probability;

        if (ransac.computeModel() && ransac.inliers_.size() >= min_nr_of_correspondences) {
          refineRansacModel(ransac, min_nr_of_correspondences);
          relative_pose_estimate = eigen2Pose(ransac.model_coefficients_);
          g_num_inliers = static_cast<int>(ransac.inliers_.size());
        } else {
          ROS_WARN("2-point RANSAC did not find a valid translation.");
        }

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate relative translation using 2pt algorithm.");
    }
    break;
  }
  case 3: {
    // *********************************************************************************
    // ******************************** 3-point Method *********************************
    // *********************************************************************************

    // Scale the bearing vectors to point clouds, by querying the depth values of each keypoint
    //
    // DELIVERABLE 5：官方骨架是"每个点都乘上深度"，但真实深度图里总有一批
    // 无效像素（TUM 用 0 表示没测到）。这些点如果留在点云里，会把 Arun 的
    // SVD 直接带偏，所以这里顺手做了有效性过滤。
    opengv::points_t cloud_1, cloud_2;
    for (int i = 0; i < N; i++) {
      double const d1 = depthAt(prev_depth, pts1[i]);
      double const d2 = depthAt(depth, pts2[i]);
      if (d1 < g_min_depth || d1 > g_max_depth) continue;
      if (d2 < g_min_depth || d2 > g_max_depth) continue;
      if (std::abs(bearing_vector_1[i](2,0)) < 1e-9 ||
          std::abs(bearing_vector_2[i](2,0)) < 1e-9) continue;

      // Normalize the bearing vectors such that the last entry is 1
      opengv::bearingVector_t const b1 = bearing_vector_1[i] / bearing_vector_1[i](2, 0);
      opengv::bearingVector_t const b2 = bearing_vector_2[i] / bearing_vector_2[i](2, 0);

      // Scale the bearing vectors so that the last entry is equal to depth
      cloud_1.push_back(b1 * d1);
      cloud_2.push_back(b2 * d2);
    }
    std::cout << "Valid depth for " << cloud_1.size() << "/" << N << " correspondences"
              << std::endl;

    // OpenGV PointCloud 3D-3D Adapter
    Adapter3D adapter_3d(cloud_1, cloud_2);

    static constexpr int min_nr_of_correspondences = 3; 
    if (adapter_3d.getNumberCorrespondences() >= min_nr_of_correspondences) {
      if (!g_opt.use_ransac){
        // Without RANSAC (Optional)
        // ***************************** begin solution *****************************

        // 用全部三维对应点做一次 Arun（SVD 闭式解），没有任何外点剔除。
        opengv::transformation_t const T = opengv::point_cloud::threept_arun(adapter_3d);
        relative_pose_estimate = eigen2Pose(T);

        // ***************************** end solution *****************************
      } else{
        // (TODO) With RANSAC
        // ***************************** begin solution *****************************

        // 3D-3D 的 RANSAC 阈值是米制距离（PointCloudSacProblem 用 ||p1 - T p2||）。
        opengv::sac::Ransac<RansacProblem3D> ransac;
        ransac.sac_model_ = std::make_shared<RansacProblem3D>(adapter_3d);
        ransac.threshold_ = g_opt.ransac_threshold_3d;
        ransac.max_iterations_ = g_opt.ransac_max_iterations;
        ransac.probability_ = g_opt.ransac_probability;

        if (ransac.computeModel() && ransac.inliers_.size() >= min_nr_of_correspondences) {
          refineRansacModel(ransac, min_nr_of_correspondences);
          relative_pose_estimate = eigen2Pose(ransac.model_coefficients_);
          g_num_inliers = static_cast<int>(ransac.inliers_.size());
        } else {
          ROS_WARN("Arun's 3-point RANSAC did not find a valid transformation.");
        }

        // ***************************** end solution *****************************
      }
    } else {
      ROS_WARN("Not enough correspondences to estimate absolute transform using Arun's 3pt algorithm.");
    }
    break;
  }
  default: {
    ROS_ERROR("Wrong pose_estimator flag!");
  }
  }

  // scale the relative pose estimate (for 5-pt, 2-pt and 8-pt, the translation is up to scale)
  if (g_opt.pose_estimator < 3) {
    scaleTranslation(relative_pose_estimate.position, prev_pose_, curr_pose_);
  }

  // pose estimate w.r.t. ground truth
  updatePoseEstimate(prev_pose_.pose, relative_pose_estimate, pose_estimation.pose);

  // Compute tf::Transform in order to do actual pose operations.
  tf::Transform gt_t_prev_frame, gt_t_curr_frame;
  tf::Transform est_t_prev_frame, est_t_curr_frame;
  tf::poseMsgToTF(pose_estimation.pose, est_t_curr_frame);
  tf::poseMsgToTF(curr_pose_.pose, gt_t_curr_frame);
  tf::poseMsgToTF(prev_pose_.pose, est_t_prev_frame);
  tf::poseMsgToTF(prev_pose_.pose, gt_t_prev_frame);

  // Evaluate pose errors
  evaluateRPE(gt_t_prev_frame, gt_t_curr_frame,est_t_prev_frame,est_t_curr_frame);

  // 把逐帧误差写进 CSV，供离线画图（Python 侧只负责画，不重算算法）
  if (g_csv.is_open()) {
    g_csv << g_frame_id << ","
          << g_opt.pose_estimator << ","
          << g_opt.use_ransac << ","
          << N << ","
          << g_num_inliers << ","
          << std::fixed << std::setprecision(6)
          << g_rot_err_deg << ","
          << g_rot_err_chordal << ","
          << g_trans_err << "\n";
    g_csv.flush();
  }

  // (TODO) Now, publish your estimated pose here. Since your pose estimate is
  // relative to the previous frame, use the ground-truth pose of the previous
  // frame to publish your pose_estimate in world frame. This will allow you to
  // see how far away is your estimate from the ground-truth one in Rviz.
  // Remember to subscirbe to the right topic in Rviz. Make this abosolute pose
  pose_estimation.header.frame_id = "world";
  pub_pose_estimation_.publish(pose_estimation);

  // Save for next iteration
  prev_bgr = bgr.clone();
  prev_depth = depth.clone();
  prev_pose_ = curr_pose_;

  ++g_frame_id;
  if (g_opt.max_frames > 0 && g_frame_id >= g_opt.max_frames) {
    ROS_INFO("Reached --max_frames=%d, shutting down.", g_opt.max_frames);
    if (g_csv.is_open()) g_csv.close();
    ros::shutdown();
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "keypoint_trackers");
  ros::NodeHandle local_nh("~");

  // populate camera intrinsics and distortion
  //
  // DELIVERABLE 3：官方 office.bag 的内参是 Tesse 的 415.69 / (360,240) / 无畸变。
  // 为了让节点也能跑别的地面真值数据集（比如本报告使用的 TUM RGB-D），
  // 这里把内参改成"默认值 = 官方值，可被 ROS 参数覆盖"。
  double fx = 415.69219381653056;
  double fy = 415.69219381653056;
  double cx = 360.0;
  double cy = 240.0;
  local_nh.param("fx", fx, fx);
  local_nh.param("fy", fy, fy);
  local_nh.param("cx", cx, cx);
  local_nh.param("cy", cy, cy);

  camera_params_.K = cv::Mat::zeros(3, 3, CV_64F);
  camera_params_.K.at<double>(0,0) = fx;
  camera_params_.K.at<double>(1,1) = fy;
  camera_params_.K.at<double>(0,2) = cx;
  camera_params_.K.at<double>(1,2) = cy;

  std::vector<double> dist;   // k1,k2,p1,p2[,k3]
  local_nh.param("dist", dist, std::vector<double>{});
  if (dist.empty()) {
    camera_params_.D = cv::Mat::zeros(cv::Size(5,1),CV_64F);
  } else {
    camera_params_.D = cv::Mat::zeros(cv::Size(static_cast<int>(dist.size()),1), CV_64F);
    for (size_t i = 0; i < dist.size(); ++i) camera_params_.D.at<double>(i,0) = dist[i];
  }

  local_nh.param("depth_scale", g_depth_scale, g_depth_scale);
  local_nh.param("min_depth", g_min_depth, g_min_depth);
  local_nh.param("max_depth", g_max_depth, g_max_depth);
  local_nh.param("use_ransac", g_opt.use_ransac, g_opt.use_ransac);
  local_nh.param("scale_translation", g_opt.scale_translation, g_opt.scale_translation);
  local_nh.param("pose_estimator", g_opt.pose_estimator, g_opt.pose_estimator);
  local_nh.param("ransac_threshold_bearing", g_opt.ransac_threshold_bearing,
                 g_opt.ransac_threshold_bearing);
  local_nh.param("ransac_threshold_3d", g_opt.ransac_threshold_3d, g_opt.ransac_threshold_3d);
  local_nh.param("ransac_max_iterations", g_opt.ransac_max_iterations,
                 g_opt.ransac_max_iterations);
  local_nh.param("ransac_probability", g_opt.ransac_probability, g_opt.ransac_probability);
  local_nh.param("refine", g_opt.refine, g_opt.refine);
  local_nh.param("max_frames", g_opt.max_frames, g_opt.max_frames);
  local_nh.param("csv_path", g_opt.csv_path, std::string(""));

  // Constant extrinsics of the camera wrt the body, T_{camera}^{body}
  T_camera_body = cv::Mat::zeros(cv::Size(4,4),CV_64F);
  T_camera_body.at<double>(0,2) = 1.0;
  T_camera_body.at<double>(1,0) = -1.0;
  T_camera_body.at<double>(1,3) = 0.05;
  T_camera_body.at<double>(2,1) = -1.0;
  T_camera_body.at<double>(3,3) = 1.0;
  // convert to geometry_msgs::Pose
  R_camera_body = T_camera_body(cv::Range(0,3),cv::Range(0,3));
  t_camera_body = T_camera_body(cv::Range(0,3),cv::Range(3,4));
  pose_camera_body = cv2Pose(R_camera_body,t_camera_body);
  // convert to tf transform
  tf::poseMsgToTF(pose_camera_body, transform_camera_body);

  // 打开 CSV（只在第一次调用时写表头）
  std::string const csv_file = g_opt.csv_path;
  if (!csv_file.empty()) {
    g_csv.open(csv_file, std::ios::out | std::ios::trunc);
    if (g_csv.is_open()) {
      g_csv << "frame,estimator,use_ransac,n_matches,n_inliers,"
               "rot_err_deg,rot_err_chordal,trans_err\n";
      ROS_INFO("Logging RPE statistics to %s", csv_file.c_str());
    } else {
      ROS_WARN("Could not open CSV file %s for writing.", csv_file.c_str());
    }
  }

  feature_tracker_.reset(new TrackerWrapper());

  // Subscribe to drone pose estimation.
  auto pose_sub = local_nh.subscribe("/ground_truth_pose", 10, poseCallbackTesse);

  // Subscribe to rgb and depth images.
  // 用 message_filters 的裸 Subscriber，不经过 image_transport：
  // image_transport 是系统包，会把 /usr/lib/.../libopencv_*.so.3.2 拖进进程，
  // 与课程依赖里自编的 OpenCV 3.4（带 nonfree）在同一进程内混链。
  message_filters::Subscriber<sensor_msgs::Image> sf_rgb(local_nh, "/rgb_images_topic", 1);
  message_filters::Subscriber<sensor_msgs::Image> sf_depth(local_nh, "/depth_images_topic", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sf_rgb, sf_depth);
  sync.registerCallback(cameraCallback);

  // Advertise drone pose.
  pub_pose_gt_ = local_nh.advertise<geometry_msgs::PoseStamped>("/gt_camera_pose", 1);
  pub_pose_estimation_ = local_nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);

  ros::AsyncSpinner spinner(2);   // 2 线程：odom 与图像可以并行进入
  spinner.start();
  ros::waitForShutdown();
  cv::destroyAllWindows();

  if (g_csv.is_open()) g_csv.close();

  return EXIT_SUCCESS;
}
