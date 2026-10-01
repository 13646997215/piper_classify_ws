#include "registration.hpp"
#include "depth_stress.hpp"
#include <memory>
#include <stdexcept>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
class Registration : public rclcpp::Node {
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Policy = message_filters::sync_policies::ExactTime<Image, Image>;
public:
  Registration() : Node("gemini335_registration") {
    auto color = declare_parameter<std::vector<double>>("color_intrinsics", {457.,461.,320.,240.});
    auto depth = declare_parameter<std::vector<double>>("depth_intrinsics", {372.,372.,320.,240.});
    auto offset = declare_parameter<std::vector<double>>("color_origin_in_depth", {.014,0.,.00202});
    if (color.size()!=4 || depth.size()!=4 || offset.size()!=3) throw std::runtime_error("Invalid camera profile dimensions");
    std::copy(color.begin(),color.end(),kc_.begin()); std::copy(depth.begin(),depth.end(),kd_.begin());
    offset_ = cv::Vec3d(offset[0],offset[1],offset[2]);
    min_z_=declare_parameter("min_z",.17); max_z_=declare_parameter("max_z",20.);
    baseline_=declare_parameter("stereo_baseline",.05);
    width_=declare_parameter("width",640); height_=declare_parameter("height",480);
    const auto seed=declare_parameter<int64_t>("stress_seed",0);
    if (seed<0 || seed>UINT32_MAX) throw std::runtime_error("Stress seed must be uint32");
    stress_=std::make_unique<classify_camera_sim::DepthStress>(
      declare_parameter("depth_noise_stddev_m",0.),declare_parameter("depth_dropout_probability",0.),static_cast<unsigned>(seed));
    if (stress_->enabled()) RCLCPP_WARN(get_logger(),"Depth stress enabled: synthetic perturbation, NOT measured Gemini noise");
    const auto qos=rclcpp::SensorDataQoS();
    color_.subscribe(this,"/camera/color/image_raw",qos.get_rmw_qos_profile());
    depth_.subscribe(this,"/camera/native/depth/image_raw",qos.get_rmw_qos_profile());
    sync_=std::make_shared<message_filters::Synchronizer<Policy>>(Policy(30),color_,depth_);
    sync_->registerCallback(&Registration::pair,this);
    aligned_=create_publisher<Image>("/camera/depth/image_raw",qos);
    color_info_=create_publisher<Info>("/camera/color/camera_info",qos);
    depth_info_=create_publisher<Info>("/camera/depth/camera_info",qos);
    native_info_=create_publisher<Info>("/camera/native/depth/camera_info",qos);
    old_rgb_=create_publisher<Image>("/wrist_depth_sensor/image_raw",qos);
    old_depth_=create_publisher<Image>("/wrist_depth_sensor/depth/image_raw",qos);
    old_info_=create_publisher<Info>("/wrist_depth_sensor/camera_info",qos);
    RCLCPP_INFO(get_logger(),"Gemini 335 D2C: separate optical centres, 16UC1 mm, nominal intrinsics; exact sensor pairing");
  }
private:
  Info info(const std_msgs::msg::Header& header, const classify_camera_sim::Intrinsics& k) {
    Info m; m.header=header; m.width=width_;m.height=height_;m.distortion_model="plumb_bob";
    m.d={0.,0.,0.,0.,0.}; m.k={k[0],0.,k[2],0.,k[1],k[3],0.,0.,1.};
    m.r={1.,0.,0.,0.,1.,0.,0.,0.,1.};m.p={k[0],0.,k[2],0.,0.,k[1],k[3],0.,0.,0.,1.,0.};return m;
  }
  void pair(Image::ConstSharedPtr rgb, Image::ConstSharedPtr depth) {
    if (rgb->header.frame_id!="camera_color_optical_frame" || depth->header.frame_id!="camera_depth_optical_frame" ||
        rgb->width!=static_cast<unsigned>(width_) || rgb->height!=static_cast<unsigned>(height_) ||
        depth->width!=rgb->width || depth->height!=rgb->height || depth->encoding!="32FC1") {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Native sensor profile/frame mismatch");return;
    }
    const auto source=cv_bridge::toCvShare(depth);
    const auto native=stress_->apply(source->image);
    const auto registered=classify_camera_sim::register_depth(native,kd_,kc_,width_,height_,offset_,min_z_,max_z_,baseline_);
    auto message=cv_bridge::CvImage(rgb->header,"16UC1",registered).toImageMsg();
    aligned_->publish(*message);old_depth_->publish(*message);old_rgb_->publish(*rgb);
    const auto ci=info(rgb->header,kc_);color_info_->publish(ci);depth_info_->publish(ci);old_info_->publish(ci);
    native_info_->publish(info(depth->header,kd_));
  }
  classify_camera_sim::Intrinsics kc_,kd_;cv::Vec3d offset_;
  std::unique_ptr<classify_camera_sim::DepthStress> stress_;
  double min_z_,max_z_,baseline_;int width_,height_;
  message_filters::Subscriber<Image> color_,depth_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<Image>::SharedPtr aligned_,old_rgb_,old_depth_;
  rclcpp::Publisher<Info>::SharedPtr color_info_,depth_info_,native_info_,old_info_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Registration>());rclcpp::shutdown();}
