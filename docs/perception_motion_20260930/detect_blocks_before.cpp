#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <classify_interfaces/msg/detected_block.hpp>
#include <classify_interfaces/msg/detected_blocks.hpp>
#include <cv_bridge/cv_bridge.h>
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

double median(std::vector<double> values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

struct Point3
{
  double x;
  double y;
  double z;
};

}  // namespace

class BlockDetector : public rclcpp::Node
{
public:
  BlockDetector()
  : Node("detect_blocks"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto qos = rclcpp::SensorDataQoS();
    color_sub_ = create_subscription<Image>(
        "/wrist_depth_sensor/image_raw", qos,
        [this](Image::ConstSharedPtr msg) { on_color(std::move(msg)); });
    depth_sub_ = create_subscription<Image>(
        "/wrist_depth_sensor/depth/image_raw", qos,
        [this](Image::ConstSharedPtr msg) { depth_ = std::move(msg); });
    info_sub_ = create_subscription<CameraInfo>(
        "/wrist_depth_sensor/camera_info", qos,
        [this](CameraInfo::ConstSharedPtr msg) { info_ = std::move(msg); });
    detections_pub_ = create_publisher<classify_interfaces::msg::DetectedBlocks>(
        "/classify/detections", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/classify/detection_markers", 10);
  }

private:
  static bool in_pickup_zone(const tf2::Vector3& point)
  {
    // Geometric workspace filter, not a hardcoded object pose. The bins start
    // at world x=0.355, while the blocks are on the table near x=0.30.
    // base_link is 0.347 m above world, so a block top is near base z=0.053.
    return point.x() > 0.20 && point.x() < 0.35 &&
           std::abs(point.y()) < 0.20 && point.z() > 0.0 && point.z() < 0.10;
  }

  void on_color(Image::ConstSharedPtr color)
  {
    if (!depth_ || !info_ || color->header.frame_id != "camera_optical_frame" ||
        depth_->header.frame_id != color->header.frame_id ||
        info_->header.frame_id != color->header.frame_id) {
      return;
    }
    const rclcpp::Time stamp(color->header.stamp);
    if (last_processed_.nanoseconds() != 0 &&
        (stamp - last_processed_).seconds() >= 0.0 &&
        (stamp - last_processed_).seconds() < 0.5) {
      return;
    }
    if (std::abs((stamp - rclcpp::Time(depth_->header.stamp)).seconds()) > 0.08 ||
        std::abs((stamp - rclcpp::Time(info_->header.stamp)).seconds()) > 0.08) {
      return;
    }
    if (info_->k[0] <= 0.0 || info_->k[4] <= 0.0 ||
        color->width != depth_->width || color->height != depth_->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB-D dimensions or intrinsics invalid");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          "base_link", color->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.15));
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera TF: %s", e.what());
      return;
    }

    cv_bridge::CvImageConstPtr rgb;
    cv_bridge::CvImageConstPtr depth;
    try {
      rgb = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
      depth = cv_bridge::toCvShare(depth_);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Image conversion failed: %s", e.what());
      return;
    }
    if (depth->image.type() != CV_32FC1 && depth->image.type() != CV_16UC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported depth encoding: %s",
                           depth_->encoding.c_str());
      return;
    }
    last_processed_ = stamp;

    cv::Mat hsv;
    cv::cvtColor(rgb->image, hsv, cv::COLOR_BGR2HSV);
    const auto& t = transform.transform.translation;
    const auto& q = transform.transform.rotation;
    const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    const tf2::Vector3 offset(t.x, t.y, t.z);
    const double fx = info_->k[0];
    const double fy = info_->k[4];
    const double cx = info_->k[2];
    const double cy = info_->k[5];

    classify_interfaces::msg::DetectedBlocks output;
    output.header = color->header;
    output.header.frame_id = "base_link";
    visualization_msgs::msg::MarkerArray markers;
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

      std::vector<Point3> best_points;
      for (std::size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
        if (cv::contourArea(contours[contour_index]) < 100.0) {
          continue;
        }
        cv::Mat component = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(component, contours, static_cast<int>(contour_index),
                         cv::Scalar(255), cv::FILLED);
        std::vector<Point3> points;
        const auto bounds = cv::boundingRect(contours[contour_index]);
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
            const tf2::Vector3 camera_point(
                (u - cx) * distance / fx, (v - cy) * distance / fy, distance);
            const tf2::Vector3 base_point = rotation * camera_point + offset;
            if (in_pickup_zone(base_point)) {
              points.push_back({base_point.x(), base_point.y(), base_point.z()});
            }
          }
        }
        if (points.size() > best_points.size()) {
          best_points = std::move(points);
        }
      }
      if (best_points.size() < 200) {
        continue;
      }

      std::vector<double> heights;
      heights.reserve(best_points.size());
      for (const auto& point : best_points) {
        heights.push_back(point.z);
      }
      auto quartile = heights.begin() + 3 * heights.size() / 4;
      std::nth_element(heights.begin(), quartile, heights.end());
      const double top_cut = *quartile - 0.004;
      std::vector<double> top_x, top_y, top_z;
      for (const auto& point : best_points) {
        if (point.z >= top_cut) {
          top_x.push_back(point.x);
          top_y.push_back(point.y);
          top_z.push_back(point.z);
        }
      }
      if (top_x.size() < 100) {
        continue;
      }

      DetectedBlock block;
      block.color = rule.name;
      block.center.x = median(std::move(top_x));
      block.center.y = median(std::move(top_y));
      block.top_z = static_cast<float>(median(std::move(top_z)));
      block.center.z = block.top_z - 0.025;  // Known 50 mm cube: top to centre.
      block.visible_points = static_cast<std::uint32_t>(best_points.size());
      output.blocks.push_back(block);
      summary << rule.name << "=(" << block.center.x << "," << block.center.y << ","
              << block.center.z << ") ";

      visualization_msgs::msg::Marker cube;
      cube.header = output.header;
      cube.ns = "detected_blocks";
      cube.id = static_cast<int>(color_index);
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose.position = block.center;
      cube.pose.orientation.w = 1.0;
      cube.scale.x = cube.scale.y = cube.scale.z = 0.05;
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

    detections_pub_->publish(output);
    markers_pub_->publish(markers);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Detected %zu blocks: %s",
                         output.blocks.size(), summary.str().c_str());
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Image::ConstSharedPtr depth_;
  CameraInfo::ConstSharedPtr info_;
  rclcpp::Time last_processed_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
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
