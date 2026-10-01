#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include "bin_layout.hpp"

#include <classify_interfaces/msg/scene_status.hpp>
#include <classify_interfaces/msg/task_status.hpp>
#include <classify_interfaces/msg/task_result.hpp>
#include <classify_interfaces/msg/detected_blocks.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Status = classify_interfaces::msg::TaskStatus;
using Result = classify_interfaces::msg::TaskResult;
using Trigger = std_srvs::srv::Trigger;

class ClassifyTask : public rclcpp::Node
{
public:
  ClassifyTask() : Node("classify_task", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    objects_ = parameter<std::vector<std::string>>("objects", {"red", "blue"});
    if (objects_.empty() || objects_.size() > 2) throw std::runtime_error("Select red and/or blue only");
    for (const auto& color : objects_) if (color != "red" && color != "blue")
      throw std::runtime_error("Green is a reference object and must not be moved");
    if (objects_.size() == 2 && objects_[0] == objects_[1]) throw std::runtime_error("Duplicate task objects");
    for (const auto& c:objects_) expected_[c]=parameter<int>("expected_"+c+"_count",1);
    for (const auto& [c,n]:expected_) if (n<1 || n>4) throw std::runtime_error("Task supports 1..4 pieces per target colour");
    tcp_ = parameter("tcp_offset", .132);
    tilt_ = parameter("tool_tilt_degrees", 30.0) * M_PI / 180.;
    bin_tilt_ = parameter("bin_tool_tilt_degrees", 45.0) * M_PI / 180.;
    pre_ = parameter("pregrasp_distance", .075);
    lift_ = parameter("lift_distance", .18);
    clearance_ = parameter("release_clearance", .025);
    grip_ = parameter("gripper_grasp_position", .0225);
    timeout_ = parameter("planning_timeout", 30.);
    velocity_ = parameter("velocity_scaling", .2);
    acceleration_ = parameter("acceleration_scaling", .15);
    jump_ = parameter("cartesian_jump_threshold", 10.0);
    vision_timeout_ = parameter("vision_timeout", 20.);
    plan_only_ = parameter("plan_only", false);
    auto_start_ = parameter("auto_start", false);
    status_.state = "IDLE"; status_.total = 0; for (const auto& [c,n]:expected_) status_.total+=n;
    status_.detail = "Ready for /classify/start; green remains untouched";
    status_pub_ = create_publisher<Status>("/classify/status", rclcpp::QoS(1).transient_local());
    result_pub_ = create_publisher<Result>("/classify/result", rclcpp::QoS(1).transient_local());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/classify/task_markers", rclcpp::QoS(1).transient_local());
    scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    entity_client_ = create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
    scene_sub_ = create_subscription<classify_interfaces::msg::SceneStatus>(
      "/classify/scene_status", rclcpp::QoS(1).transient_local(),
      [this](classify_interfaces::msg::SceneStatus::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_); vision_ = *msg; vision_wall_ = Clock::now();
      });
    detections_sub_ = create_subscription<classify_interfaces::msg::DetectedBlocks>(
      "/classify/detections", 10,
      [this](classify_interfaces::msg::DetectedBlocks::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(mutex_); detections_ = *msg; detections_wall_ = Clock::now();
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        const auto found = std::find(msg->name.begin(), msg->name.end(), "joint7");
        if (found == msg->name.end()) return;
        std::lock_guard<std::mutex> guard(mutex_);
        status_.gripper_open_ratio = std::clamp(msg->position.at(found-msg->name.begin())/.035, 0., 1.);
      });
    grasp_sub_ = create_subscription<std_msgs::msg::String>("/classify/grasped_object",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { std::lock_guard<std::mutex> guard(mutex_);status_.held_object=msg->data; });
    start_service_ = create_service<Trigger>("/classify/start", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->success = start(); res->message = res->success ? "Task accepted; observe /classify/status and /classify/result" : "Task running, terminal failure/cancellation, or objects already placed; reset the simulation before retrying";
    });
    cancel_service_ = create_service<Trigger>("/classify/cancel", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->success = busy_; res->message = busy_ ? "Cancellation requested; gripper keeps its current position" : "No running task";
      if (busy_) { cancel_ = true; std::lock_guard<std::mutex> guard(motion_mutex_); if (arm_) arm_->stop(); if (hand_) hand_->stop(); }
    });
    timer_ = create_wall_timer(200ms, [this]() { publish(); if (auto_start_ && !auto_started_) { auto_started_ = true; start(); } });
  }

  ~ClassifyTask() override { cancel_ = true; if (worker_.joinable()) worker_.join(); }

private:
  template<typename T> T parameter(const std::string& name, T value) {
    return has_parameter(name) ? get_parameter(name).get_value<T>() : declare_parameter<T>(name, value);
  }

  bool start() {
    if (busy_.exchange(true)) return false;
    { std::lock_guard<std::mutex> guard(mutex_);
      if (!status_.placed_objects.empty() || status_.state == "FAILED" || status_.state == "CANCELLED") {
        busy_ = false; return false;
      }
      status_.completed = status_.succeeded = status_.failed = 0;
      started_ = Clock::now();
    }
    if (worker_.joinable()) worker_.join();
    cancel_ = false;
    worker_ = std::thread([this]() { run(); }); return true;
  }

  void check_cancel() const { if (cancel_ || !rclcpp::ok()) throw std::runtime_error("Task cancelled; current gripper position retained"); }

  void stage(const std::string& state, const std::string& name, const std::string& detail="") {
    check_cancel();
    { std::lock_guard<std::mutex> guard(mutex_); status_.state=state; status_.stage=name; status_.detail=detail; }
    RCLCPP_INFO(get_logger(), "%s: %s %s", state.c_str(), name.c_str(), detail.c_str()); publish();
  }

  void publish() {
    Status current;
    { std::lock_guard<std::mutex> guard(mutex_);
      status_.header.stamp=now(); status_.header.frame_id="base_link";
      if (busy_) status_.elapsed_sec=std::chrono::duration<double>(Clock::now()-started_).count();
      current=status_;
    }
    status_pub_->publish(current);
    visualization_msgs::msg::Marker marker;
    marker.header=current.header; marker.ns="task_status"; marker.id=0;
    marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.x=.58; marker.pose.position.y=.33; marker.pose.position.z=.20;
    marker.pose.orientation.w=1.; marker.scale.z=.025; marker.color.a=1.;
    marker.color.r=current.state=="FAILED" ? 1.f : .2f; marker.color.g=.9f; marker.color.b=1.f;
    std::string stage_name=current.stage;std::replace(stage_name.begin(),stage_name.end(),' ','_');
    std::ostringstream label; label << current.state << '_' << unsigned(current.completed) << '/' << unsigned(current.total);
    if (!current.current_object.empty()) label << "\n" << current.current_object << "=>" << current.target_bin;
    if (!stage_name.empty()) label << "\n" << stage_name;
    label << "\nGRIPPER=" << std::lround(current.gripper_open_ratio*100) << '%';
    if (!current.held_object.empty()) label << "\nHOLD=" << current.held_object;
    marker.text=label.str(); visualization_msgs::msg::MarkerArray array; array.markers.push_back(marker); markers_pub_->publish(array);
  }

  void wait_vision(const std::string& color) {
    const auto end=Clock::now()+std::chrono::duration<double>(vision_timeout_);
    while (Clock::now()<end) {
      check_cancel();
      { std::lock_guard<std::mutex> guard(mutex_);
        const auto& names=vision_.fresh_objects;
        if (vision_.camera_stable && std::chrono::duration<double>(Clock::now()-vision_wall_).count()<.8 &&
            now().seconds()-rclcpp::Time(vision_.header.stamp).seconds()<.8 &&
            std::find(names.begin(),names.end(),color)!=names.end() &&
            std::find(vision_.fresh_colors.begin(),vision_.fresh_colors.end(),"green")!=vision_.fresh_colors.end()) return;
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("Fresh stable vision unavailable for "+color+" and green reference");
  }

  planning_scene::PlanningScenePtr read_scene() {
    if (!scene_client_->wait_for_service(5s)) throw std::runtime_error("MoveIt scene service unavailable");
    auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>(); req->components.components=1023;
    auto future=scene_client_->async_send_request(req);
    if (future.wait_for(5s)!=std::future_status::ready) throw std::runtime_error("Planning scene request timed out");
    auto scene=std::make_shared<planning_scene::PlanningScene>(arm_->getRobotModel());
    scene->setPlanningSceneMsg(future.get()->scene);
    // ODE can report 1e-11 m beyond a prismatic stop. MTC's interpolation
    // uses exact bounds; clamp only numerical noise, never a real violation.
    auto& state=scene->getCurrentStateNonConst();
    if (!state.satisfiesBounds(1e-4)) throw std::runtime_error("Measured robot joints exceed limits");
    state.enforceBounds();state.update();return scene;
  }

  geometry_msgs::msg::Pose physical_pose(const std::string& id) {
    if (!entity_client_->wait_for_service(3s)) throw std::runtime_error("Gazebo verification service unavailable; restart simulation with the updated world");
    auto req=std::make_shared<gazebo_msgs::srv::GetEntityState::Request>(); req->name=id; req->reference_frame="world";
    auto future=entity_client_->async_send_request(req);
    if (future.wait_for(3s)!=std::future_status::ready) throw std::runtime_error("Gazebo verification timed out");
    const auto res=future.get(); if (!res->success) throw std::runtime_error("Gazebo entity missing: "+id);
    // Gazebo truth is expressed in world. Use the same measured base transform
    // as the planning scene instead of assuming a 347 mm mounting height.
    const Eigen::Isometry3d world_to_base = read_scene()->getFrameTransform("base_link").inverse();
    const auto& p=res->state.pose.position; const auto& q=res->state.pose.orientation;
    Eigen::Isometry3d object=Eigen::Isometry3d::Identity();
    object.translation()=Eigen::Vector3d(p.x,p.y,p.z);
    object.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
    const auto transformed=world_to_base*object; const Eigen::Quaterniond rotation(transformed.rotation());
    geometry_msgs::msg::Pose pose;pose.position.x=transformed.translation().x();
    pose.position.y=transformed.translation().y();pose.position.z=transformed.translation().z();
    pose.orientation.x=rotation.x();pose.orientation.y=rotation.y();pose.orientation.z=rotation.z();pose.orientation.w=rotation.w();
    return pose;
  }

  void validate_planned_pickup(const std::string& color, const Eigen::Vector3d& pickup) {
    std::lock_guard<std::mutex> guard(mutex_);
    const bool recent = detections_.camera_stable &&
      std::chrono::duration<double>(Clock::now()-detections_wall_).count()<.8 &&
      now().seconds()-rclcpp::Time(detections_.header.stamp).seconds()<.8;
    bool found=false, green=false;
    if (recent) for (const auto& b:detections_.blocks) {
      if (b.color=="green") green=true;
      if (b.color==color && (Eigen::Vector3d(b.center.x,b.center.y,b.center.z)-pickup).norm()<.005 &&
          std::abs(std::remainder(b.yaw-pickup_yaws_.at(status_.current_object),M_PI/2))<.04) found=true;
    }
    if (!recent || !found || !green)
      throw std::runtime_error("Planned pickup lost fresh vision or moved before execution; reset and reacquire");
  }

  geometry_msgs::msg::PoseStamped tcp_pose(const Eigen::Vector3d& point, double tilt, double yaw=0) const {
    geometry_msgs::msg::PoseStamped p; p.header.frame_id="base_link";
    p.pose.position.x=point.x(); p.pose.position.y=point.y(); p.pose.position.z=point.z();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(M_PI-tilt,Eigen::Vector3d::UnitY()));
    p.pose.orientation.x=q.x(); p.pose.orientation.y=q.y(); p.pose.orientation.z=q.z(); p.pose.orientation.w=q.w(); return p;
  }

  void add_move(mtc::Task& task, const std::string& name, const std::string& group,
                const mtc::solvers::PlannerInterfacePtr& planner, const std::string& goal) {
    auto s=std::make_unique<mtc::stages::MoveTo>(name,planner); s->setGroup(group); s->setGoal(goal); s->setTimeout(8.); task.add(std::move(s));
  }

  void add_pose(mtc::Task& task, const std::string& name, const Eigen::Vector3d& p,
                const mtc::solvers::PlannerInterfacePtr& planner, double tilt=-1., double yaw=0) {
    auto s=std::make_unique<mtc::stages::MoveTo>(name,planner); s->setGroup("arm"); s->setTimeout(8.);
    s->setIKFrame(Eigen::Translation3d(0,0,tcp_),"gripper_base"); s->setGoal(tcp_pose(p,tilt<0 ? tilt_ : tilt,yaw)); task.add(std::move(s));
  }

  classify_task::BinLayout bin_layout(const planning_scene::PlanningScenePtr& scene,const std::string& color) {
    const auto bin=scene->getWorld()->getObject("bin_"+color);
    if (!bin || bin->shapes_.size()!=5) throw std::runtime_error("Missing floor/four-wall bin geometry");
    const auto* floor=dynamic_cast<const shapes::Box*>(bin->shapes_[0].get());
    if (!floor) throw std::runtime_error("Bin floor must be a box");
    classify_task::BinLayout layout;
    layout.floor=scene->getFrameTransform("base_link").inverse()*bin->global_shape_poses_[0];
    layout.xmin=-floor->size[0]/2;layout.xmax=floor->size[0]/2;
    layout.ymin=-floor->size[1]/2;layout.ymax=floor->size[1]/2;layout.floor_top=floor->size[2]/2;layout.rim=0;
    for (size_t i=1;i<5;++i) {
      const auto* wall=dynamic_cast<const shapes::Box*>(bin->shapes_[i].get());
      if (!wall) throw std::runtime_error("Bin wall must be a box");
      const Eigen::Isometry3d wp=scene->getFrameTransform("base_link").inverse()*bin->global_shape_poses_[i];
      const Eigen::Isometry3d local=layout.floor.inverse()*wp;
      if (!local.linear().isApprox(Eigen::Matrix3d::Identity(),1e-5)) throw std::runtime_error("Bin wall axes must match floor axes");
      const auto& p=local.translation();
      if (std::abs(p.x())>std::abs(p.y())) {
        if (p.x()>0) layout.xmax=std::min(layout.xmax,p.x()-wall->size[0]/2);
        else layout.xmin=std::max(layout.xmin,p.x()+wall->size[0]/2);
      } else {
        if (p.y()>0) layout.ymax=std::min(layout.ymax,p.y()-wall->size[1]/2);
        else layout.ymin=std::max(layout.ymin,p.y()+wall->size[1]/2);
      }
      layout.rim=std::max(layout.rim,wp.translation().z()+wall->size[2]/2);
    }
    return layout;
  }

  void prepare_inventory() {
    const auto end=Clock::now()+std::chrono::duration<double>(vision_timeout_);
    while (Clock::now()<end) {
      check_cancel();
      { std::lock_guard<std::mutex> guard(mutex_);
        if (vision_.ready && vision_.camera_stable && std::chrono::duration<double>(Clock::now()-vision_wall_).count()<.8 &&
            now().seconds()-rclcpp::Time(vision_.header.stamp).seconds()<.8 && vision_.object_ids.size()==vision_.object_colors.size()) {
          queue_.clear();object_colors_.clear();
          for (const auto& c:objects_) {
            unsigned count=0;
            for (size_t i=0;i<vision_.object_ids.size();++i) if (vision_.object_colors[i]==c) {
              queue_.push_back(vision_.object_ids[i]);object_colors_[vision_.object_ids[i]]=c;++count;
            }
            if (count!=unsigned(expected_.at(c))) { queue_.clear();break; }
          }
          if (!queue_.empty()) break;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    if (queue_.empty()) throw std::runtime_error("Expected inventory not fully confirmed by fresh vision");
    // Reserve all slots before any motion: reject an undersized bin atomically.
    const auto scene=read_scene();std::map<std::string,unsigned> next;
    for (const auto& c:objects_) {
      layouts_[c]=bin_layout(scene,c);bin_slots_[c]=layouts_.at(c).slots(expected_.at(c));
    }
    for (const auto& id:queue_) slots_[id]=bin_slots_.at(object_colors_.at(id)).at(next[object_colors_.at(id)]++);
  }

  std::unique_ptr<mtc::Task> plan_object(const std::string& id, Eigen::Vector3d& pickup,
                                      Eigen::Vector3d& release, Eigen::Vector3d& resting) {
    const auto& color=object_colors_.at(id);
    auto scene=read_scene(); const auto block=scene->getWorld()->getObject(id);
    if (!block || block->shape_poses_.empty()) throw std::runtime_error("Missing confirmed object "+id);
    const Eigen::Isometry3d observed=scene->getFrameTransform("base_link").inverse()*block->global_shape_poses_[0];
    pickup=observed.translation();
    const double yaw=std::remainder(std::atan2(observed.linear()(1,0),observed.linear()(0,0)),M_PI/2);
    pickup_yaws_[id]=yaw;
    resting=slots_.at(id);release=resting;release.z()=layouts_.at(color).rim+.025+clearance_;
    RCLCPP_INFO(get_logger(),"VISION_TARGET %s color=%s xyz=(%.5f %.5f %.5f) yaw=%.2fdeg; slot=(%.3f %.3f %.3f)",
      id.c_str(),color.c_str(),pickup.x(),pickup.y(),pickup.z(),yaw*180/M_PI,resting.x(),resting.y(),resting.z());
    // MTC 0.1.3 sanitizes slashes into underscores; use a plain namespace.
    auto task=std::make_unique<mtc::Task>("classify_mtc"); task->setName(id+" to "+color+" bin"); task->setRobotModel(arm_->getRobotModel()); task->setTimeout(timeout_);
    auto pipeline=std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    auto joints=std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian=std::make_shared<mtc::solvers::CartesianPath>(); cartesian->setStepSize(.003); cartesian->setJumpThreshold(jump_); cartesian->setMinFraction(1.);
    for (const auto& planner : std::vector<mtc::solvers::PlannerInterfacePtr>{pipeline,joints,cartesian}) {
      planner->setMaxVelocityScalingFactor(velocity_); planner->setMaxAccelerationScalingFactor(acceleration_);
    }
    task->add(std::make_unique<mtc::stages::FixedState>("confirmed scene",scene));
    add_move(*task,"open gripper","gripper",joints,"open");
    add_pose(*task,"move above object",pickup+Eigen::Vector3d(0,0,pre_),pipeline,tilt_,yaw);
    add_pose(*task,"approach object",pickup,cartesian,tilt_,yaw);
    auto allow=std::make_unique<mtc::stages::ModifyPlanningScene>("allow finger contact");
    allow->allowCollisions(id,std::vector<std::string>{"link7","link8"},true); task->add(std::move(allow));
    auto close=std::make_unique<mtc::stages::MoveTo>("close gripper",joints); close->setGroup("gripper"); close->setGoal(std::map<std::string,double>{{"joint7",grip_},{"joint8",-grip_}}); task->add(std::move(close));
    auto attach=std::make_unique<mtc::stages::ModifyPlanningScene>("attach object"); attach->attachObject(id,"gripper_base"); task->add(std::move(attach));
    add_pose(*task,"lift object",pickup+Eigen::Vector3d(0,0,lift_),cartesian,tilt_,yaw);
    add_pose(*task,"carry above bin",release+Eigen::Vector3d(0,0,.045),pipeline,bin_tilt_);
    add_pose(*task,"lower above bin",release,cartesian,bin_tilt_);
    add_move(*task,"release gripper","gripper",joints,"open");
    auto detach=std::make_unique<mtc::stages::ModifyPlanningScene>("detach object"); detach->detachObject(id,"gripper_base"); task->add(std::move(detach));
    auto settle=std::make_unique<mtc::stages::ModifyPlanningScene>("settle in bin");
    moveit_msgs::msg::CollisionObject object; object.id=id; object.header.frame_id="base_link"; object.operation=object.ADD;
    shape_msgs::msg::SolidPrimitive shape; shape.type=shape.BOX; shape.dimensions={.05,.05,.05}; object.primitives.push_back(shape);
    geometry_msgs::msg::Pose pose; pose.position.x=resting.x();pose.position.y=resting.y();pose.position.z=resting.z();pose.orientation.w=1.;object.primitive_poses.push_back(pose);
    settle->addObject(object); task->add(std::move(settle));
    auto forbid=std::make_unique<mtc::stages::ModifyPlanningScene>("restore finger collision");
    forbid->allowCollisions(id,std::vector<std::string>{"link7","link8"},false);task->add(std::move(forbid));
    add_pose(*task,"retreat from bin",release+Eigen::Vector3d(0,0,.045),cartesian,bin_tilt_);
    add_move(*task,"return home","arm",pipeline,"home");
    if (task->plan(1)!=moveit::core::MoveItErrorCode::SUCCESS || task->solutions().empty()) {
      std::ostringstream error; task->explainFailure(error); throw std::runtime_error("MTC planning failed: "+error.str());
    }
    task->introspection().publishSolution(*task->solutions().front());
    return task;
  }

  void execute_solution(const mtc::SolutionBase& solution, const std::string& id,
                        const Eigen::Vector3d& pickup, const Eigen::Vector3d& resting) {
    const auto& color=object_colors_.at(id);
    check_cancel();
    if (const auto* sequence=dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
      for (const auto* item:sequence->solutions()) execute_solution(*item,id,pickup,resting);
      return;
    }
    if (const auto* wrapped=dynamic_cast<const mtc::WrappedSolution*>(&solution)) { execute_solution(*wrapped->wrapped(),id,pickup,resting); return; }
    const auto* part=dynamic_cast<const mtc::SubTrajectory*>(&solution); if (!part) throw std::runtime_error("Unknown MTC solution type");
    const auto name=solution.creator()->name();
    if (name=="confirmed scene") return;
    if (name=="open gripper") validate_planned_pickup(color,pickup);
    const bool placing=name.find("bin")!=std::string::npos || name=="release gripper" || name=="detach object" || name=="restore finger collision" || name=="return home";
    stage(placing ? "PLACING" : "PICKING",name);
    if (part->trajectory() && part->trajectory()->getWayPointCount()>1) {
      moveit_msgs::msg::RobotTrajectory trajectory; part->trajectory()->getRobotTrajectoryMsg(trajectory);
      if (arm_->execute(trajectory)!=moveit::core::MoveItErrorCode::SUCCESS) throw std::runtime_error("Controller execution failed at "+name);
    }
    check_cancel();
    moveit_task_constructor_msgs::msg::Solution message; part->toMsg(message);
    for (auto& item:message.sub_trajectory) {
      auto& diff=item.scene_diff;
      diff.is_diff=true; diff.robot_state.is_diff=true;
      diff.robot_state.joint_state=sensor_msgs::msg::JointState();
      diff.robot_state.multi_dof_joint_state=sensor_msgs::msg::MultiDOFJointState();
      if (!scene_interface_->applyPlanningScene(diff)) throw std::runtime_error("Scene update failed at "+name);
    }
    if (name=="close gripper") {
      const auto end=Clock::now()+1500ms;std::string model;
      while (Clock::now()<end) {
        check_cancel();
        { std::lock_guard<std::mutex> guard(mutex_);model=status_.held_object; }
        if (!model.empty()) break;
        std::this_thread::sleep_for(50ms);
      }
      const std::string prefix=color+"_block";
      if (model.empty() || (model!=prefix && !(model.rfind(prefix+"_",0)==0 && model.size()>prefix.size()+1 &&
          model.find_first_not_of("0123456789",prefix.size()+1)==std::string::npos)))
        throw std::runtime_error("No matching physical grasp after finger contact");
      const auto actual=physical_pose(model);const auto& p=actual.position;
      if ((Eigen::Vector3d(p.x,p.y,p.z)-pickup).norm()>.010)
        throw std::runtime_error("Physical grasp does not match visual target");
      physical_ids_[id]=model;
      RCLCPP_INFO(get_logger(),"GRASP_BINDING %s -> %s",id.c_str(),model.c_str());
    }
    if (name=="lift object") {
      const auto actual=physical_pose(physical_ids_.at(id));
      if (actual.position.z<pickup.z()+lift_*.7 || std::hypot(actual.position.x-pickup.x(),actual.position.y-pickup.y())>.03)
        throw std::runtime_error("Physical grasp failed: cube did not follow the lift");
      RCLCPP_INFO(get_logger(),"PHYSICAL_LIFT_OK %s z=%.5f",color.c_str(),actual.position.z);
    }
    if (name=="settle in bin") {
      std::this_thread::sleep_for(1500ms); const auto actual=physical_pose(physical_ids_.at(id));
      if (std::abs(actual.position.x-resting.x())>.027 || std::abs(actual.position.y-resting.y())>.027 || std::abs(actual.position.z-resting.z())>.012)
        throw std::runtime_error("Physical placement failed: cube is not settled inside its bin");
      Eigen::Isometry3d actual_tf=Eigen::Isometry3d::Identity();
      actual_tf.translation()=Eigen::Vector3d(actual.position.x,actual.position.y,actual.position.z);
      const auto& q=actual.orientation;actual_tf.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
      if (!layouts_.at(color).contains(actual_tf)) throw std::runtime_error("Released cube crosses bin wall or floor");
      for (const auto& [other,placed]:settled_) if (object_colors_.at(other)==color && !classify_task::separated(actual_tf,placed))
        throw std::runtime_error("Released cubes overlap or lack required gap");
      settled_[id]=actual_tf;
      auto object=scene_interface_->getObjects({id}).at(id);
      object.header.frame_id="base_link";object.pose=geometry_msgs::msg::Pose();object.pose.orientation.w=1.;object.primitive_poses={actual};object.operation=object.ADD;
      if (!scene_interface_->applyCollisionObject(object)) throw std::runtime_error("Could not synchronize placed cube pose");
      { std::lock_guard<std::mutex> guard(mutex_); status_.placed_objects.push_back(id);status_.placed_poses.push_back(actual);status_.completed++;status_.succeeded++;
        RCLCPP_INFO(get_logger(),"PROGRESS %u/%u: %s -> bin_%s",unsigned(status_.completed),unsigned(status_.total),id.c_str(),color.c_str()); }
      publish();RCLCPP_INFO(get_logger(),"PHYSICAL_PLACE_OK %s xyz=(%.5f %.5f %.5f)",color.c_str(),actual.position.x,actual.position.y,actual.position.z);
    }
  }

  void run() {
    Result result; result.header.frame_id="base_link";
    try {
      stage("DETECTING","initialize");
      tasks_.clear();queue_.clear();physical_ids_.clear();settled_.clear();
      { std::lock_guard<std::mutex> guard(motion_mutex_);
        if (!arm_) arm_=std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),"arm");
        if (!hand_) hand_=std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(),"gripper");
      }
      if (!scene_interface_) scene_interface_=std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
      arm_->startStateMonitor(5.);hand_->startStateMonitor(5.);
      // Gazebo truth is only an execution check; pickup goals come from the vision scene.
      const auto green_before=physical_pose("green_block");
      stage("DETECTING","confirm full inventory");prepare_inventory();
      for (const auto& id:queue_) {
        const auto& color=object_colors_.at(id);
        { std::lock_guard<std::mutex> guard(mutex_);status_.current_object=id;status_.target_bin="bin_"+color; }
        stage("DETECTING","wait for stable vision");wait_vision(id);
        stage("PLANNING","MTC pick and place");Eigen::Vector3d pickup,release,resting;
        tasks_.push_back(plan_object(id,pickup,release,resting));
        if (!plan_only_) execute_solution(*tasks_.back()->solutions().front(),id,pickup,resting);
      }
      // Recheck every released object after the entire batch, since a later
      // placement can disturb an earlier one. Publish the final measured poses.
      if (!plan_only_) {
        std::map<std::string,Eigen::Isometry3d> final;
        for (const auto& id:queue_) {
          const auto actual=physical_pose(physical_ids_.at(id));const auto& q=actual.orientation;
          Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
          transform.translation()=Eigen::Vector3d(actual.position.x,actual.position.y,actual.position.z);
          transform.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
          const auto& color=object_colors_.at(id);
          if (!layouts_.at(color).contains(transform)) throw std::runtime_error("Previously placed object left its bin: "+id);
          for (const auto& [other,pose]:final) if (object_colors_.at(other)==color && !classify_task::separated(transform,pose))
            throw std::runtime_error("Batch-end cubes overlap: "+id+" and "+other);
          final[id]=transform;
          auto object=scene_interface_->getObjects({id}).at(id);object.header.frame_id="base_link";
          object.pose=geometry_msgs::msg::Pose();object.pose.orientation.w=1.;object.primitive_poses={actual};object.operation=object.ADD;
          if (!scene_interface_->applyCollisionObject(object)) throw std::runtime_error("Final pose synchronization failed");
          { std::lock_guard<std::mutex> guard(mutex_);
            const auto index=std::find(status_.placed_objects.begin(),status_.placed_objects.end(),id)-status_.placed_objects.begin();
            status_.placed_poses.at(index)=actual;
          }
        }
      }
      const auto green_after=physical_pose("green_block");
      const auto& a=green_before.position;const auto& b=green_after.position;
      if (std::sqrt(std::pow(a.x-b.x,2)+std::pow(a.y-b.y,2)+std::pow(a.z-b.z,2))>.002)
        throw std::runtime_error("Green reference moved more than 2 mm");
      result.success=true; stage("DONE",plan_only_ ? "plans ready" : "batch complete",plan_only_ ? "Planning only; no objects moved" : "Red/blue placed; green unchanged; arm returned home");
    } catch (const std::exception& error) {
      result.success=false;
      { std::lock_guard<std::mutex> guard(mutex_);status_.state=cancel_ ? "CANCELLED" : "FAILED";
        status_.detail=cancel_ ? "Task cancelled; gripper retained; reset required before retry" : error.what();
        status_.failed=cancel_ ? 0 : 1; }
      if (cancel_) RCLCPP_WARN(get_logger(),"TASK_CANCELLED: gripper retained; reset required before retry");
      else RCLCPP_ERROR(get_logger(),"TASK_FAILED: %s",error.what());
      publish();
    }
    { std::lock_guard<std::mutex> guard(mutex_);
      status_.elapsed_sec=std::chrono::duration<double>(Clock::now()-started_).count();
      result.header.stamp=now();result.succeeded=status_.succeeded;result.failed=status_.failed;result.elapsed_sec=status_.elapsed_sec;result.detail=status_.detail;
      result.objects=status_.placed_objects;result.final_poses=status_.placed_poses;for (const auto& id:result.objects) { result.bins.push_back("bin_"+object_colors_.at(id));result.physical_objects.push_back(physical_ids_.at(id)); }
    }
    RCLCPP_INFO(get_logger(),"CLASSIFICATION_RESULT success=%s succeeded=%u failed=%u elapsed=%.1fs",
      result.success ? "true" : "false",unsigned(result.succeeded),unsigned(result.failed),result.elapsed_sec);
    for (size_t i=0;i<result.objects.size();++i) {
      const auto& p=result.final_poses[i].position;
      RCLCPP_INFO(get_logger(),"%-5s | %-8s | base xyz %.4f %.4f %.4f",result.objects[i].c_str(),result.bins[i].c_str(),p.x,p.y,p.z);
    }
    result_pub_->publish(result);busy_=false;publish();
  }

  template<typename T> using Subscription = typename rclcpp::Subscription<T>::SharedPtr;
  std::vector<std::string> objects_,queue_;
  std::map<std::string,int> expected_;
  std::map<std::string,std::string> object_colors_,physical_ids_;
  std::map<std::string,classify_task::BinLayout> layouts_;
  std::map<std::string,std::vector<Eigen::Vector3d>> bin_slots_;
  std::map<std::string,Eigen::Vector3d> slots_;
  std::map<std::string,double> pickup_yaws_;
  std::map<std::string,Eigen::Isometry3d> settled_;
  double tcp_,tilt_,bin_tilt_,pre_,lift_,clearance_,grip_,timeout_,velocity_,acceleration_,jump_,vision_timeout_;
  bool plan_only_,auto_start_,auto_started_{false};
  std::atomic<bool> busy_{false},cancel_{false};std::thread worker_;std::mutex mutex_,motion_mutex_;
  Clock::time_point started_{Clock::now()},vision_wall_{};
  Clock::time_point detections_wall_{};
  classify_interfaces::msg::DetectedBlocks detections_;
  Status status_;classify_interfaces::msg::SceneStatus vision_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_,hand_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_interface_;
  std::vector<std::unique_ptr<mtc::Task>> tasks_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr entity_client_;
  Subscription<classify_interfaces::msg::SceneStatus> scene_sub_;Subscription<sensor_msgs::msg::JointState> joint_sub_;
  Subscription<classify_interfaces::msg::DetectedBlocks> detections_sub_;
  Subscription<std_msgs::msg::String> grasp_sub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;rclcpp::Publisher<Result>::SharedPtr result_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_,cancel_service_;rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { auto node=std::make_shared<ClassifyTask>();rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);executor.spin(); }
  catch (const std::exception& error) { RCLCPP_FATAL(rclcpp::get_logger("classify_task"),"%s",error.what());rclcpp::shutdown();return 1; }
  rclcpp::shutdown();return 0;
}
