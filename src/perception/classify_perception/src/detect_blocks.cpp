#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <optional>

#include "block_geometry.hpp"

#include <classify_interfaces/msg/detected_block.hpp>
#include <classify_interfaces/msg/detected_blocks.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/calib3d.hpp>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using DetectedBlock = classify_interfaces::msg::DetectedBlock;

struct ColorRule
{
  const char* name;
  cv::Scalar low;
  cv::Scalar high;
  cv::Scalar second_low;
  cv::Scalar second_high;
  bool has_second;
  std::array<float, 3> rgb;
};

const std::array<ColorRule, 3> kColors{{
    {"red", cv::Scalar(0, 80, 50), cv::Scalar(12, 255, 255),
     cv::Scalar(170, 80, 50), cv::Scalar(179, 255, 255), true, {0.9f, 0.1f, 0.1f}},
    {"green", cv::Scalar(35, 80, 50), cv::Scalar(90, 255, 255),
     {}, {}, false, {0.1f, 0.8f, 0.1f}},
    {"blue", cv::Scalar(95, 80, 50), cv::Scalar(135, 255, 255),
     {}, {}, false, {0.1f, 0.2f, 0.9f}},
}};

}  // namespace

class BlockDetector : public rclcpp::Node
{
public:
  BlockDetector()
  : Node("detect_blocks"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto qos = rclcpp::SensorDataQoS();
    const auto rgb_topic = declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    optical_frame_ = declare_parameter<std::string>("optical_frame", "camera_color_optical_frame");
    reference_frame_ = declare_parameter<std::string>("reference_frame", "base_link");
    sync_tolerance_ = declare_parameter("sync_tolerance", 0.0);
    if (sync_tolerance_ < 0 || sync_tolerance_ > .033) throw std::runtime_error("RGB-D tolerance must be 0..33 ms");
    color_sub_.subscribe(this, rgb_topic, qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic, qos.get_rmw_qos_profile());
    if (sync_tolerance_ == 0) {
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), color_sub_, depth_sub_);
      sync_->registerCallback(&BlockDetector::on_rgbd, this);
    } else {
      ApproxPolicy policy(20); policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
      approx_sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(static_cast<const ApproxPolicy &>(policy), color_sub_, depth_sub_);
      approx_sync_->registerCallback(&BlockDetector::on_rgbd, this);
    }
    info_sub_ = create_subscription<CameraInfo>(
        info_topic, qos,
        [this](CameraInfo::ConstSharedPtr msg) { info_ = std::move(msg); });
    detections_pub_ = create_publisher<classify_interfaces::msg::DetectedBlocks>(
        "/classify/detections", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/classify/detection_markers", 10);
    cube_size_ = declare_parameter("cube_size", .05);
    expected_top_ = declare_parameter("table_top_z", .003) + cube_size_;
    xmin_ = declare_parameter("pickup_x_min", .30); xmax_ = declare_parameter("pickup_x_max", .45);
    ymax_ = declare_parameter("pickup_y_abs_max", .20);
    watchdog_ = create_wall_timer(std::chrono::milliseconds(300), [this]() {
      if (!have_pair_ || (now() - last_received_).seconds() > .8 ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_pair_wall_).count() > .8) {
        std_msgs::msg::Header header;
        header.stamp = now(); header.frame_id = reference_frame_;
        publish_empty(header, "No synchronized RGB-D/TF observation");
      }
    });
  }

private:
  bool in_pickup_zone(const tf2::Vector3& point) const
  {
    return point.x() > xmin_ && point.x() < xmax_ && std::abs(point.y()) < ymax_ &&
           point.z() > expected_top_ - cube_size_ - .01 && point.z() < expected_top_ + .01;
  }

  void update_rays()
  {
    if (!rays_.empty() && cached_k_ == info_->k && cached_d_ == info_->d &&
        rays_.cols == static_cast<int>(info_->width) && rays_.rows == static_cast<int>(info_->height)) return;
    std::vector<cv::Point2f> pixels, normalized;
    for (unsigned v = 0; v < info_->height; ++v) for (unsigned u = 0; u < info_->width; ++u)
      pixels.emplace_back(u,v);
    const cv::Mat k(3,3,CV_64F, const_cast<double*>(info_->k.data()));
    const cv::Mat d(info_->d, true);
    cv::undistortPoints(pixels, normalized, k, d);
    rays_.create(info_->height,info_->width,CV_32FC2);
    std::copy(normalized.begin(),normalized.end(),rays_.ptr<cv::Point2f>());
    cached_k_ = info_->k; cached_d_ = info_->d;
  }

  void publish_empty(const std_msgs::msg::Header& header, const std::string& status)
  {
    classify_interfaces::msg::DetectedBlocks output;
    output.header = header; output.header.frame_id = reference_frame_;
    output.camera_stable = false; output.status = status;
    detections_pub_->publish(output);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    markers_pub_->publish(markers);
  }

  void on_rgbd(Image::ConstSharedPtr color, Image::ConstSharedPtr depth_msg)
  {
    have_pair_ = true; last_received_ = now();
    last_pair_wall_ = std::chrono::steady_clock::now();
    if (!info_ || color->header.frame_id != optical_frame_ ||
        depth_msg->header.frame_id != color->header.frame_id ||
        info_->header.frame_id != color->header.frame_id) {
      publish_empty(color->header, "Waiting for matching camera frames/intrinsics");
      return;
    }
    const rclcpp::Time stamp(color->header.stamp);
    if (std::abs((stamp-rclcpp::Time(depth_msg->header.stamp)).seconds()) > sync_tolerance_ + 1e-9) {
      publish_empty(color->header, "RGB-D timestamps exceed tolerance");return;
    }
    if (last_processed_.nanoseconds() != 0 &&
        (stamp - last_processed_).seconds() >= 0.0 &&
        (stamp - last_processed_).seconds() < 0.2) {
      return;
    }
    if (info_->k[0] <= 0.0 || info_->k[4] <= 0.0 ||
        color->width != depth_msg->width || color->height != depth_msg->height ||
        color->width != info_->width || color->height != info_->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB-D dimensions or intrinsics invalid");
      publish_empty(color->header, "RGB-D dimensions or intrinsics invalid");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          reference_frame_, color->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.15));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera TF: %s", e.what());
      publish_empty(color->header, "Waiting for image-time camera TF");
      return;
    }

    last_processed_ = stamp;
    const auto& translation = transform.transform.translation;
    const auto& quaternion = transform.transform.rotation;
    const tf2::Vector3 camera_position(translation.x, translation.y, translation.z);
    const tf2::Quaternion camera_rotation(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    if (!have_camera_pose_ || stamp <= previous_camera_stamp_) {
      last_motion_ = stamp;
    } else {
      const double dt = (stamp - previous_camera_stamp_).seconds();
      if ((camera_position - previous_camera_position_).length() / dt > .015 ||
          previous_camera_rotation_.angleShortestPath(camera_rotation) / dt > .05) last_motion_ = stamp;
    }
    previous_camera_position_ = camera_position;
    previous_camera_rotation_ = camera_rotation;
    previous_camera_stamp_ = stamp; have_camera_pose_ = true;
    if ((stamp - last_motion_).seconds() < .4) {
      publish_empty(color->header, "Camera moving or settling; last confirmed scene retained");
      return;
    }

    cv_bridge::CvImageConstPtr rgb;
    cv_bridge::CvImageConstPtr depth;
    try {
      rgb = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
      depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image conversion failed: %s", e.what());
      publish_empty(color->header, "Image conversion failed");
      return;
    }
    if (depth->image.type() != CV_32FC1 && depth->image.type() != CV_16UC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported depth encoding: %s",
                           depth_msg->encoding.c_str());
      publish_empty(color->header, "Unsupported depth encoding");
      return;
    }

    if (!info_->distortion_model.empty() && info_->distortion_model != "plumb_bob" &&
        info_->distortion_model != "rational_polynomial") {
      publish_empty(color->header, "Unsupported camera distortion model");return;
    }
    try { update_rays(); } catch (const cv::Exception&) {
      publish_empty(color->header, "Invalid camera distortion/intrinsics");return;
    }
    cv::Mat hsv;
    cv::cvtColor(rgb->image, hsv, cv::COLOR_BGR2HSV);
    const auto& t = transform.transform.translation;
    const auto& q = transform.transform.rotation;
    const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    const tf2::Vector3 offset(t.x, t.y, t.z);

    classify_interfaces::msg::DetectedBlocks output;
    output.header = color->header;
    output.header.frame_id = reference_frame_;
    output.camera_stable = true;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    std::ostringstream summary;

    for (std::size_t color_index = 0; color_index < kColors.size(); ++color_index) {
      const auto& rule = kColors[color_index];
      cv::Mat mask;
      cv::inRange(hsv, rule.low, rule.high, mask);
      if (rule.has_second) {
        cv::Mat second;
        cv::inRange(hsv, rule.second_low, rule.second_high, second);
        cv::bitwise_or(mask, second, mask);
      }
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                       cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
      std::vector<std::vector<cv::Point>> contours;
      cv::Mat contour_input = mask.clone();
      cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


      for (std::size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
        if (cv::contourArea(contours[contour_index]) < 100.0) {
          continue;
        }
        cv::Mat component = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(component, contours, static_cast<int>(contour_index),
                         cv::Scalar(255), cv::FILLED);
        std::vector<cv::Vec3d> points;
        const auto bounds = cv::boundingRect(contours[contour_index]);
        if (bounds.x <= 1 || bounds.y <= 1 || bounds.br().x >= mask.cols - 1 ||
            bounds.br().y >= mask.rows - 1) continue;
        for (int v = bounds.y; v < bounds.y + bounds.height; ++v) {
          for (int u = bounds.x; u < bounds.x + bounds.width; ++u) {
            if (component.at<std::uint8_t>(v, u) == 0 || mask.at<std::uint8_t>(v, u) == 0) {
              continue;
            }
            double distance;
            if (depth->image.type() == CV_32FC1) {
              distance = depth->image.at<float>(v, u);
            } else {
              distance = depth->image.at<std::uint16_t>(v, u) / 1000.0;
            }
            if (!std::isfinite(distance) || distance < 0.05 || distance > 2.0) {
              continue;
            }
            const auto ray = rays_.at<cv::Vec2f>(v,u);
            const tf2::Vector3 camera_point(ray[0] * distance, ray[1] * distance, distance);
            const tf2::Vector3 base_point = rotation * camera_point + offset;
            if (in_pickup_zone(base_point)) {
              points.emplace_back(base_point.x(), base_point.y(), base_point.z());
            }
          }
        }
        const auto top = classify_perception::fit_top_face(points, expected_top_, cube_size_);
        if (!top) continue;
        const auto& best_top = top;
        const auto best_point_count = points.size();
      DetectedBlock block;
      block.color = rule.name;
      block.center.x = best_top->center[0];
      block.center.y = best_top->center[1];
      block.center.z = best_top->center[2];
      block.top_z = static_cast<float>(best_top->top_z);
      block.visible_points = static_cast<std::uint32_t>(best_point_count);
      block.confidence = static_cast<float>(best_top->confidence);
      block.top_width = static_cast<float>(best_top->width);
      block.top_depth = static_cast<float>(best_top->depth);
      block.plane_error = static_cast<float>(best_top->plane_error);
      block.yaw = static_cast<float>(best_top->yaw);
      output.blocks.push_back(block);
      summary << rule.name << "=(" << block.center.x << "," << block.center.y << ","
              << block.center.z << ") ";

      visualization_msgs::msg::Marker cube;
      cube.header = output.header;
      cube.ns = "detected_blocks";
      cube.id = static_cast<int>(output.blocks.size()-1);
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose.position = block.center;
      cube.pose.orientation.z = std::sin(block.yaw/2);
      cube.pose.orientation.w = std::cos(block.yaw/2);
      cube.scale.x = cube.scale.y = cube.scale.z = cube_size_;
      cube.color.r = rule.rgb[0];
      cube.color.g = rule.rgb[1];
      cube.color.b = rule.rgb[2];
      cube.color.a = 0.55f;
      cube.lifetime = rclcpp::Duration::from_seconds(1.0);
      markers.markers.push_back(cube);

      auto label = cube;
      label.ns = "detected_labels";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += 0.07;
      label.scale.z = 0.035;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = rule.name;
      markers.markers.push_back(label);
      }
    }

    output.status = "Reliable visible top surfaces; expected counts checked by scene bridge";
    detections_pub_->publish(output);
    markers_pub_->publish(markers);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Detected %zu reliable blocks: %s (%s)",
                         output.blocks.size(), summary.str().c_str(), output.status.c_str());
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  CameraInfo::ConstSharedPtr info_;
  rclcpp::Time last_processed_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_{0, 0, RCL_ROS_TIME}, last_motion_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_camera_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Vector3 previous_camera_position_;
  tf2::Quaternion previous_camera_rotation_;
  std::chrono::steady_clock::time_point last_pair_wall_{};
  bool have_pair_{false}, have_camera_pose_{false};
  std::string reference_frame_;
  double expected_top_{.053}, cube_size_{.05}, xmin_, xmax_, ymax_, sync_tolerance_;
  std::string optical_frame_;
  cv::Mat rays_;
  std::array<double,9> cached_k_{};
  std::vector<double> cached_d_;
  using SyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> approx_sync_;
  message_filters::Subscriber<Image> color_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<classify_interfaces::msg::DetectedBlocks>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlockDetector>());
  rclcpp::shutdown();
  return 0;
}
