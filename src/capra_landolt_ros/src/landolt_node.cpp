/**
 * @file landolt_node.cpp
 * @brief ROS 2 Node for Landolt C ring detection using OpenCV.
 *
 * This node subscribes to an RGB image stream, processes it to find Landolt C 
 * rings (broken rings) using contour and convexity defect analysis, and 
 * determines the orientation of the gap. It supports different mapping modes 
 * for RoboCup tasks (generic, linear, omni) and includes a sharpness filter 
 * to reject blurry frames. It also provides a manual capture service.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "capra_landolt_msgs/msg/landolts.hpp"
#include "capra_landolt_msgs/msg/bounding_circles.hpp"
#include "capra_landolt_msgs/msg/point2f.hpp"

/**
 * @brief Structure to hold multiple Landolt C detection results.
 * * Stores the orientations (angles), radii, and center coordinates 
 * for all valid Landolt rings detected in a single frame.
 */
struct Gaps
{
  std::vector<float> angles;        ///< Gap orientations in degrees [0, 360).
  std::vector<float> radius;        ///< Radii of the detected rings in pixels.
  std::vector<cv::Point2f> centers; ///< Center coordinates (x, y) of the rings.
};

/**
 * @class LandoltNode
 * @brief Core ROS 2 node class for Landolt ring detection and publishing.
 */
class LandoltNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LandoltNode.
   * * Declares ROS 2 parameters, initializes publishers/subscribers, 
   * creates the manual capture service, and sets up the continuous 
   * publishing timer.
   */
  LandoltNode()
  : Node("landolt_node")
  {
    camera_topic_ = this->declare_parameter<std::string>(
      "camera_topic", "/camera/color/image_raw");

    threshold_value_ = this->declare_parameter<int>("threshold_value", 140);
    min_edge_ = this->declare_parameter<int>("min_edge", 12);
    min_ratio_circle_ = this->declare_parameter<double>("min_ratio_circle", 0.8);
    min_depth_ = this->declare_parameter<int>("min_depth", 10);
    publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);

    crop_scale_ = this->declare_parameter<double>("crop_scale", 1.5);
    choose_largest_detection_ = this->declare_parameter<bool>("choose_largest_detection", true);

    min_sharpness_ = this->declare_parameter<double>("min_sharpness", 80.0);
    show_rejected_blur_warning_ = this->declare_parameter<bool>(
      "show_rejected_blur_warning", true);

    /*
     * robocup_task_mode:
     *
     * generic -> T, TR, R, BR, B, BL, L, TL
     * linear  -> LP, LA, C, RA, RP
     * omni    -> LF, LB, C, RF, RB
     */
    robocup_task_mode_ = this->declare_parameter<std::string>(
      "robocup_task_mode", "generic");

    landolt_pub_ = this->create_publisher<capra_landolt_msgs::msg::Landolts>(
      "landolts", 10);

    bounding_pub_ = this->create_publisher<capra_landolt_msgs::msg::BoundingCircles>(
      "boundings", 10);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image", 10);

    captured_landolt_pub_ = this->create_publisher<capra_landolt_msgs::msg::Landolts>(
      "captured/landolts", 10);

    captured_bounding_pub_ = this->create_publisher<capra_landolt_msgs::msg::BoundingCircles>(
      "captured/boundings", 10);

    captured_crop_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "captured/landolt_crop", 10);

    captured_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "captured/debug_image", 10);

    orientation_pub_ = this->create_publisher<std_msgs::msg::String>(
      "captured/orientation", 10);

    capture_service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_landolt",
      std::bind(
        &LandoltNode::captureServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stored_detection_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&LandoltNode::publishStoredDetection, this));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LandoltNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Landolt detector started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Auto crop topic: /captured/landolt_crop");
    RCLCPP_INFO(this->get_logger(), "Orientation topic: /captured/orientation");
    RCLCPP_INFO(this->get_logger(), "Manual capture service: /capture_landolt");
    RCLCPP_INFO(this->get_logger(), "RoboCup task mode: %s", robocup_task_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "Min sharpness: %.2f", min_sharpness_);
  }

private:
  // ROS Parameters
  std::string camera_topic_;
  int threshold_value_;
  int min_edge_;
  double min_ratio_circle_;
  int min_depth_;
  bool publish_debug_image_;
  double crop_scale_;
  bool choose_largest_detection_;
  double min_sharpness_;
  bool show_rejected_blur_warning_;
  std::string robocup_task_mode_;

  std::mutex data_mutex_;

  // Latest frame state
  bool has_latest_frame_ = false;
  cv::Mat latest_raw_image_;
  cv::Mat latest_debug_image_;
  std_msgs::msg::Header latest_header_;
  Gaps latest_gaps_;

  // Stored (captured) detection state
  bool has_stored_detection_ = false;
  cv::Mat stored_crop_image_;
  cv::Mat stored_debug_image_;
  std_msgs::msg::Header stored_detection_header_;
  std::string stored_orientation_;
  double stored_sharpness_ = 0.0;
  Gaps stored_gaps_;

  // ROS 2 Interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<capra_landolt_msgs::msg::Landolts>::SharedPtr landolt_pub_;
  rclcpp::Publisher<capra_landolt_msgs::msg::BoundingCircles>::SharedPtr bounding_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::Publisher<capra_landolt_msgs::msg::Landolts>::SharedPtr captured_landolt_pub_;
  rclcpp::Publisher<capra_landolt_msgs::msg::BoundingCircles>::SharedPtr captured_bounding_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr captured_crop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr captured_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr orientation_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr stored_detection_timer_;

  /**
   * @brief Calculates the magnitude of a 2D vector.
   * * @param diff The 2D vector (cv::Point2f).
   * @return float The magnitude (length) of the vector.
   */
  static float magnitudePoint(const cv::Point2f & diff)
  {
    return std::sqrt(diff.dot(diff));
  }

  /**
   * @brief Normalizes a 2D vector to a length of 1.
   * * @param diff The 2D vector to normalize.
   * @return cv::Point2f The normalized vector. Returns (0,0) if magnitude is 0.
   */
  static cv::Point2f normalizePoint(const cv::Point2f & diff)
  {
    const float mag = magnitudePoint(diff);

    if (mag <= 1e-6f) {
      return cv::Point2f(0.0f, 0.0f);
    }

    return diff / mag;
  }

  /**
   * @brief Calculates the absolute angle between two 2D vectors.
   * * @param origin First vector.
   * @param dest Second vector.
   * @return float Angle in degrees from 0 to 360.
   */
  static float angleBetween(cv::Point2f origin, cv::Point2f dest)
  {
    const float dot = origin.x * dest.x + origin.y * dest.y;
    const float det = origin.x * dest.y - origin.y * dest.x;

    return std::atan2(det, dot) * static_cast<float>(180.0 / M_PI) + 180.0f;
  }

  /**
   * @brief Maps an angle in degrees to an 8-way compass direction.
   * * @param angle_deg The gap angle in degrees.
   * @return std::string Orientation string (T, TR, R, BR, B, BL, L, TL).
   */
  static std::string angleToGenericOrientation(float angle_deg)
  {
    while (angle_deg < 0.0f) {
      angle_deg += 360.0f;
    }

    while (angle_deg >= 360.0f) {
      angle_deg -= 360.0f;
    }

    if (angle_deg >= 337.5f || angle_deg < 22.5f) {
      return "R";
    }

    if (angle_deg >= 22.5f && angle_deg < 67.5f) {
      return "TR";
    }

    if (angle_deg >= 67.5f && angle_deg < 112.5f) {
      return "T";
    }

    if (angle_deg >= 112.5f && angle_deg < 157.5f) {
      return "TL";
    }

    if (angle_deg >= 157.5f && angle_deg < 202.5f) {
      return "L";
    }

    if (angle_deg >= 202.5f && angle_deg < 247.5f) {
      return "BL";
    }

    if (angle_deg >= 247.5f && angle_deg < 292.5f) {
      return "B";
    }

    return "BR";
  }

  /**
   * @brief Converts an angle to a task-specific orientation string.
   * * Maps the generic 8-way compass direction to a specific string format 
   * required by the active RoboCup task mode (generic, linear, or omni).
   * * @param angle_deg The gap angle in degrees.
   * @return std::string The task-specific orientation label.
   */
  std::string angleToOrientation(float angle_deg) const
  {
    const std::string generic = angleToGenericOrientation(angle_deg);

    if (robocup_task_mode_ == "generic") {
      return generic;
    }

    if (robocup_task_mode_ == "linear") {
      if (generic == "T") return "C";
      if (generic == "TL") return "LP";
      if (generic == "BL") return "LA";
      if (generic == "BR") return "RA";
      if (generic == "TR") return "RP";
      return generic;
    }

    if (robocup_task_mode_ == "omni") {
      if (generic == "R") return "C";
      if (generic == "T") return "LF";
      if (generic == "BR") return "LB";
      if (generic == "B") return "RF";
      if (generic == "BL") return "RB";
      return generic;
    }

    return generic;
  }

  /**
   * @brief Selects the best detection from a set of Gaps.
   * * @param gaps The Gaps struct containing detection data.
   * @param choose_largest If true, returns the index of the largest ring by radius. 
   * Otherwise, returns the first detection.
   * @return int The index of the best detection, or -1 if none exist.
   */
  static int selectBestDetection(const Gaps & gaps, bool choose_largest)
  {
    if (gaps.angles.empty() || gaps.radius.empty() || gaps.centers.empty()) {
      return -1;
    }

    if (gaps.angles.size() != gaps.radius.size() ||
        gaps.angles.size() != gaps.centers.size())
    {
      return -1;
    }

    if (!choose_largest) {
      return 0;
    }

    int best_idx = 0;
    float best_radius = gaps.radius[0];

    for (size_t i = 1; i < gaps.radius.size(); i++) {
      if (gaps.radius[i] > best_radius) {
        best_radius = gaps.radius[i];
        best_idx = static_cast<int>(i);
      }
    }

    return best_idx;
  }

  /**
   * @brief Computes a bounding box (cv::Rect) to crop the detected ring.
   * * @param image_size Dimensions of the original image to prevent out-of-bounds.
   * @param center The center point of the detected ring.
   * @param radius The radius of the detected ring.
   * @param crop_scale Multiplier to expand the bounding box beyond the radius.
   * @return cv::Rect The safe bounding box for cropping.
   */
  static cv::Rect computeCropRect(
    const cv::Size & image_size,
    const cv::Point2f & center,
    float radius,
    double crop_scale)
  {
    int half_size = static_cast<int>(std::round(radius * crop_scale));
    half_size = std::max(20, half_size);

    const int cx = static_cast<int>(std::round(center.x));
    const int cy = static_cast<int>(std::round(center.y));

    const int x_min = std::max(0, cx - half_size);
    const int y_min = std::max(0, cy - half_size);

    const int x_max = std::min(image_size.width, cx + half_size);
    const int y_max = std::min(image_size.height, cy + half_size);

    const int width = std::max(1, x_max - x_min);
    const int height = std::max(1, y_max - y_min);

    return cv::Rect(x_min, y_min, width, height);
  }

  /**
   * @brief Calculates the sharpness of an image using the Variance of Laplacian method.
   * * @param image The input image (BGR or Grayscale).
   * @return double Sharpness score. Higher values indicate a sharper image.
   */
  double computeSharpness(const cv::Mat & image)
  {
    if (image.empty()) {
      return 0.0;
    }

    cv::Mat gray;
    cv::Mat laplacian;

    if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image.clone();
    }

    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);

    return stddev.val[0] * stddev.val[0];
  }

  /**
   * @brief Main subscription callback for incoming RGB images.
   * * Converts ROS Image to OpenCV, executes the detection algorithm, 
   * updates state variables, and triggers publishers.
   * * @param msg The incoming ROS 2 Image message.
   */
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr img_ptr;

    try {
      img_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    Gaps gaps;

    findLandoltGaps(
      img_ptr->image,
      gaps,
      min_edge_,
      static_cast<float>(min_ratio_circle_),
      min_depth_);

    auto header = msg->header;
    header.stamp = this->now();

    cv::Mat debug_image = createDebugImage(img_ptr->image, gaps);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      latest_raw_image_ = img_ptr->image.clone();
      latest_debug_image_ = debug_image.clone();
      latest_header_ = header;
      latest_gaps_ = gaps;
      has_latest_frame_ = true;
    }

    publishLandolts(gaps);
    publishBoundings(gaps);

    if (!gaps.angles.empty()) {
      updateStoredDetection(img_ptr->image, debug_image, header, gaps, false);
    }

    if (publish_debug_image_) {
      publishDebugImage(header, debug_image);
    }
  }

  /**
   * @brief Attempts to update the persistent (stored) detection.
   * * Crops the best detection, evaluates its sharpness against `min_sharpness_`, 
   * and if valid (or if forced), stores it for continuous UI publishing.
   * * @param raw_image The full raw frame.
   * @param debug_image The frame with drawn debug artifacts.
   * @param header The ROS message header.
   * @param gaps The detected gaps data.
   * @param force_update If true, bypasses the sharpness filter.
   * @return true If the detection was successfully stored.
   * @return false If the detection was rejected (e.g., too blurry).
   */
  bool updateStoredDetection(
    const cv::Mat & raw_image,
    const cv::Mat & debug_image,
    const std_msgs::msg::Header & header,
    const Gaps & gaps,
    bool force_update)
  {
    const int best_idx = selectBestDetection(gaps, choose_largest_detection_);

    if (best_idx < 0) {
      return false;
    }

    const cv::Point2f selected_center = gaps.centers[best_idx];
    const float selected_radius = gaps.radius[best_idx];
    const float selected_angle = gaps.angles[best_idx];

    cv::Rect roi = computeCropRect(
      raw_image.size(),
      selected_center,
      selected_radius,
      crop_scale_);

    cv::Mat crop = raw_image(roi).clone();

    const double sharpness = computeSharpness(crop);

    if (!force_update && sharpness < min_sharpness_) {
      if (show_rejected_blur_warning_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Rejected blurry Landolt crop. Sharpness: %.2f < %.2f",
          sharpness,
          min_sharpness_);
      }
      return false;
    }

    const cv::Point2f local_center(
      selected_center.x - static_cast<float>(roi.x),
      selected_center.y - static_cast<float>(roi.y));

    cv::circle(
      crop,
      local_center,
      static_cast<int>(selected_radius),
      cv::Scalar(255, 0, 0),
      2);

    const std::string orientation = angleToOrientation(selected_angle);

    cv::putText(
      crop,
      orientation,
      cv::Point(10, 40),
      cv::FONT_HERSHEY_SIMPLEX,
      1.3,
      cv::Scalar(0, 255, 0),
      3);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      stored_crop_image_ = crop.clone();
      stored_debug_image_ = debug_image.clone();
      stored_detection_header_ = header;
      stored_orientation_ = orientation;
      stored_sharpness_ = sharpness;
      stored_gaps_ = gaps;
      has_stored_detection_ = true;
    }

    publishStoredDetection();

    return true;
  }

  /**
   * @brief Timer callback that continuously publishes the last valid stored detection.
   */
  void publishStoredDetection()
  {
    cv::Mat crop_image;
    cv::Mat debug_image;
    std_msgs::msg::Header header;
    std::string orientation;
    Gaps gaps;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (!has_stored_detection_ || stored_crop_image_.empty()) {
        return;
      }

      crop_image = stored_crop_image_.clone();
      debug_image = stored_debug_image_.clone();
      header = stored_detection_header_;
      header.stamp = this->now();
      orientation = stored_orientation_;
      gaps = stored_gaps_;
    }

    auto crop_msg = cv_bridge::CvImage(
      header,
      "bgr8",
      crop_image).toImageMsg();

    captured_crop_pub_->publish(*crop_msg);

    if (!debug_image.empty()) {
      auto debug_msg = cv_bridge::CvImage(
        header,
        "bgr8",
        debug_image).toImageMsg();

      captured_debug_pub_->publish(*debug_msg);
    }

    std_msgs::msg::String orientation_msg;
    orientation_msg.data = orientation;
    orientation_pub_->publish(orientation_msg);

    publishCapturedLandolts(gaps);
    publishCapturedBoundings(gaps);
  }

  /**
   * @brief ROS 2 Service callback to forcefully capture the current frame.
   * * Triggers an immediate capture of the latest frame, bypassing the 
   * blur/sharpness filter. Useful for manual overrides via UI.
   * * @param request The Trigger service request (empty).
   * @param response The Trigger service response (success boolean and message).
   */
  void captureServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    cv::Mat raw_image;
    cv::Mat debug_image;
    std_msgs::msg::Header header;
    Gaps gaps;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (!has_latest_frame_ || latest_raw_image_.empty()) {
        response->success = false;
        response->message = "No image has been received yet.";
        return;
      }

      raw_image = latest_raw_image_.clone();
      debug_image = latest_debug_image_.clone();
      header = latest_header_;
      header.stamp = this->now();
      gaps = latest_gaps_;
    }

    if (gaps.angles.empty()) {
      response->success = false;
      response->message = "No Landolt C detections available in the current frame.";
      return;
    }

    const bool updated = updateStoredDetection(raw_image, debug_image, header, gaps, true);

    if (!updated) {
      response->success = false;
      response->message = "Unable to store Landolt crop.";
      return;
    }

    const int best_idx = selectBestDetection(gaps, choose_largest_detection_);
    std::string orientation = "UNKNOWN";

    if (best_idx >= 0) {
      orientation = angleToOrientation(gaps.angles[best_idx]);
    }

    response->success = true;
    response->message =
      "Stored Landolt crop updated manually. Orientation: " + orientation +
      ", detections in frame: " + std::to_string(gaps.angles.size());
  }

  // --- Publisher Helper Methods ---

  void publishLandolts(const Gaps & gaps)
  {
    capra_landolt_msgs::msg::Landolts msg;
    msg.angles = gaps.angles;
    landolt_pub_->publish(msg);
  }

  void publishBoundings(const Gaps & gaps)
  {
    capra_landolt_msgs::msg::BoundingCircles msg;
    msg.radius = gaps.radius;
    msg.centers.reserve(gaps.centers.size());

    for (const auto & center_cv : gaps.centers) {
      capra_landolt_msgs::msg::Point2f center_msg;
      center_msg.x = center_cv.x;
      center_msg.y = center_cv.y;
      msg.centers.push_back(center_msg);
    }

    bounding_pub_->publish(msg);
  }

  void publishCapturedLandolts(const Gaps & gaps)
  {
    capra_landolt_msgs::msg::Landolts msg;
    msg.angles = gaps.angles;
    captured_landolt_pub_->publish(msg);
  }

  void publishCapturedBoundings(const Gaps & gaps)
  {
    capra_landolt_msgs::msg::BoundingCircles msg;
    msg.radius = gaps.radius;
    msg.centers.reserve(gaps.centers.size());

    for (const auto & center_cv : gaps.centers) {
      capra_landolt_msgs::msg::Point2f center_msg;
      center_msg.x = center_cv.x;
      center_msg.y = center_cv.y;
      msg.centers.push_back(center_msg);
    }

    captured_bounding_pub_->publish(msg);
  }

  /**
   * @brief Generates an image with drawn detection artifacts (circles and text).
   * * @param input_image The original BGR image.
   * @param gaps The detection data to draw.
   * @return cv::Mat The annotated debug image.
   */
  cv::Mat createDebugImage(const cv::Mat & input_image, const Gaps & gaps)
  {
    cv::Mat debug_image = input_image.clone();

    for (size_t i = 0; i < gaps.angles.size(); i++) {
      const cv::Point2f & c = gaps.centers[i];
      const float r = gaps.radius[i];

      cv::circle(
        debug_image,
        c,
        static_cast<int>(r),
        cv::Scalar(0, 0, 255),
        2);

      const std::string label = angleToOrientation(gaps.angles[i]);

      cv::putText(
        debug_image,
        label,
        cv::Point(static_cast<int>(c.x + 5), static_cast<int>(c.y - 5)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 0, 0),
        2);
    }

    return debug_image;
  }

  void publishDebugImage(
    const std_msgs::msg::Header & header,
    const cv::Mat & debug_image)
  {
    auto debug_msg = cv_bridge::CvImage(
      header,
      "bgr8",
      debug_image).toImageMsg();

    image_pub_->publish(*debug_msg);
  }

  /**
   * @brief Core computer vision algorithm to detect Landolt C gaps.
   * * This function converts the image to grayscale, applies a binary threshold, 
   * finds contours, and computes the convex hull. It then uses `cv::convexityDefects` 
   * to locate the deepest inward "dent" (the gap) on circular objects, effectively 
   * isolating the orientation of the Landolt C ring.
   * * @param imageRaw The raw BGR input image.
   * @param gaps [out] The struct populated with detected gap angles, radii, and centers.
   * @param minEdge Minimum number of points a contour must have to be considered.
   * @param minRatioCircle Minimum Area(Hull) / Area(MinEnclosingCircle) ratio to ensure circularity.
   * @param minDepth Minimum depth of the convexity defect to be considered a valid gap.
   */
  void findLandoltGaps(
    const cv::Mat & imageRaw,
    Gaps & gaps,
    int minEdge,
    float minRatioCircle,
    int minDepth)
  {
    cv::Mat thresholdMat;

    cv::cvtColor(imageRaw, thresholdMat, cv::COLOR_BGR2GRAY);
    cv::blur(thresholdMat, thresholdMat, cv::Size(3, 3));

    cv::threshold(
      thresholdMat,
      thresholdMat,
      threshold_value_,
      255,
      cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
      thresholdMat,
      contours,
      cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE,
      cv::Point(0, 0));

    for (auto & contour : contours) {
      if (static_cast<int>(contour.size()) <= minEdge) {
        continue;
      }

      std::vector<cv::Point> hull;
      cv::convexHull(contour, hull, true, true);

      const double hullArea = cv::contourArea(hull);

      float contourRadius = 0.0f;
      cv::Point2f contourCenter;
      cv::minEnclosingCircle(contour, contourCenter, contourRadius);

      const double minArea = contourRadius * contourRadius * M_PI;

      if (minArea <= 1e-6) {
        continue;
      }

      if (hullArea / minArea <= minRatioCircle) {
        continue;
      }

      std::vector<cv::Vec4i> defects;
      std::vector<int> hullsI;

      cv::convexHull(contour, hullsI, true, false);

      if (hullsI.size() < 3) {
        continue;
      }

      try {
        cv::convexityDefects(contour, hullsI, defects);
      } catch (const cv::Exception & e) {
        RCLCPP_WARN(this->get_logger(), "convexityDefects error: %s", e.what());
        continue;
      }

      std::vector<cv::Vec4i> deepDefects;

      for (const auto & v : defects) {
        const float depth = static_cast<float>(v[3]) / 256.0f;

        if (depth > minDepth) {
          deepDefects.push_back(v);
        }
      }

      // A valid Landolt C must have exactly ONE deep gap
      if (deepDefects.size() != 1) {
        continue;
      }

      const cv::Vec4i & v = deepDefects[0];

      const int startidx = v[0];
      const int endidx = v[1];
      const int faridx = v[2];

      if (startidx < 0 || endidx < 0 || faridx < 0 ||
          startidx >= static_cast<int>(contour.size()) ||
          endidx >= static_cast<int>(contour.size()) ||
          faridx >= static_cast<int>(contour.size()))
      {
        continue;
      }

      std::vector<cv::Point> points;
      points.emplace_back(contour[startidx]);
      points.emplace_back(contour[endidx]);

      float defectRadius = 0.0f;
      cv::Point2f defectCenter;

      cv::minEnclosingCircle(points, defectCenter, defectRadius);

      cv::Point2f dir = normalizePoint(cv::Point2f(contour[faridx]) - defectCenter);

      if (magnitudePoint(dir) <= 1e-6f) {
        continue;
      }

      const float defectAngle = angleBetween(dir, cv::Point2f(1, 0));

      gaps.angles.push_back(defectAngle);

      // Usamos centro y radio del contorno completo para recortar el Landolt completo.
      gaps.radius.push_back(contourRadius);
      gaps.centers.push_back(contourCenter);
    }
  }
};

/**
 * @brief Node entry point.
 * * Initializes the ROS 2 context, instantiates the LandoltNode, 
 * and spins it to handle callbacks.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandoltNode>());
  rclcpp::shutdown();

  return 0;
}