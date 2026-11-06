/*
 * @file self_flow.cpp
 * 16.485 - Lab 5 | [Optional] Deliverable 8 - Dense optical flow (Farneback)
 *
 * LK estimates the flow of a sparse set of corners. Farneback's algorithm
 * estimates the displacement of *every* pixel by fitting a quadratic
 * polynomial to the neighbourhood of each pixel in both frames and solving for
 * the translation that maps one polynomial onto the other.
 *
 * The flow field is visualised with the usual HSV encoding:
 *   hue        = direction of the motion,
 *   saturation = constant (1),
 *   value      = magnitude of the motion.
 * Feel free to edit any part of this file!
 */
#include <cstdio>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;

namespace {
cv::Mat prev_gray;
// 每隔多少帧把彩色光流图落盘一张（D8 的交付图）。
constexpr int kSaveEvery = 10;
int g_frame_id = 0;
// Farneback parameters: pyramid scale, levels, window size, iterations,
// polynomial neighbourhood size, Gaussian sigma, flags.
constexpr double kPyrScale = 0.5;
constexpr int kLevels = 3;
constexpr int kWinSize = 15;
constexpr int kIterations = 3;
constexpr int kPolyN = 5;
constexpr double kPolySigma = 1.2;
constexpr int kFlags = 0;

/** Map a 2-channel flow field to a BGR image (hue = direction, value = speed). */
cv::Mat FlowToColor(const cv::Mat &flow) {
  // Split into (dx, dy) planes and convert to magnitude / angle.
  cv::Mat planes[2];
  cv::split(flow, planes);
  cv::Mat magnitude, angle;
  cv::cartToPolar(planes[0], planes[1], magnitude, angle, true);  // angle in deg

  // Hue encodes the direction (OpenCV hue range is 0..179).
  cv::Mat hsv_planes[3];
  angle.convertTo(hsv_planes[0], CV_8U, 0.5);       // 0..360 deg -> 0..180
  hsv_planes[1] = cv::Mat::ones(magnitude.size(), CV_8U) * 255;
  cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
  magnitude.convertTo(hsv_planes[2], CV_8U);

  cv::Mat hsv, bgr;
  cv::merge(hsv_planes, 3, hsv);
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}
}  // namespace

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    // Convert ROS msg type to OpenCV image type.
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    if (!prev_gray.empty()) {
      // Dense flow of the whole image, from the previous to the current frame.
      cv::Mat flow;
      cv::calcOpticalFlowFarneback(prev_gray, gray, flow, kPyrScale, kLevels,
                                   kWinSize, kIterations, kPolyN, kPolySigma,
                                   kFlags);
      const cv::Mat flow_color = FlowToColor(flow);

      // 落盘：交付物 D8 要的是「彩色光流图」，不是窗口截图。
      const std::string out_dir =
          ros::param::param<std::string>("~output_dir", std::string(""));
      if (!out_dir.empty() && g_frame_id % kSaveEvery == 0) {
        if (!cv::utils::fs::exists(out_dir)) {
          cv::utils::fs::createDirectory(out_dir);
        }
        char path[1024];
        std::snprintf(path, sizeof(path), "%s/self_flow_%04d.png",
                      out_dir.c_str(), g_frame_id);
        if (cv::imwrite(path, flow_color)) {
          ROS_INFO("Saved figure: %s", path);
        }
      }
      ++g_frame_id;

      if (ros::param::param<bool>("~interactive", true)) {
        cv::imshow("view", flow_color);
      }
    } else {
      if (ros::param::param<bool>("~interactive", true)) {
        cv::imshow("view", image);
      }
    }
    prev_gray = gray;
    if (ros::param::param<bool>("~interactive", true)) {
      cv::waitKey(10);
    }

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  const bool interactive = local_nh.param<bool>("interactive", true);
  if (interactive) {
    cv::namedWindow("view", cv::WINDOW_NORMAL);
  }
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  if (interactive) {
    cv::destroyWindow("view");
  }
  return EXIT_SUCCESS;
}
