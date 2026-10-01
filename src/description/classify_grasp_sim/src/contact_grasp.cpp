#include <map>
#include <memory>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

namespace classify_grasp_sim
{
// An ideal hold constraint models the grasp only after physical side contact.
// No object coordinates, perception targets, services or teleports are used.
class ContactGrasp : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_=model;world_=model->GetWorld();node_=gazebo_ros::Node::Get(sdf);
    palm_=model->GetLink("link6"); fingers_[0]=model->GetLink("link7");fingers_[1]=model->GetLink("link8");
    actuator_=model->GetJoint("joint7");
    if (!palm_ || !fingers_[0] || !fingers_[1] || !actuator_) {
      RCLCPP_ERROR(node_->get_logger(),"Missing Piper palm/fingers/joint7");return;
    }
    contacts_=world_->Physics()->GetContactManager();contacts_->SetNeverDropContacts(true);
    publisher_=node_->create_publisher<std_msgs::msg::String>("/classify/grasped_object",rclcpp::QoS(1).transient_local());
    connection_=gazebo::event::Events::ConnectWorldUpdateEnd([this]() { update(); });
    publish();
    RCLCPP_INFO(node_->get_logger(),"Contact grasp active: both fingers, side normals, 50 ms contact; release on opening");
  }

  void Reset() override { release();candidate_.clear();contact_started_=-1.; }

private:
  void publish() {
    std_msgs::msg::String msg;msg.data=held_;publisher_->publish(msg);
  }
  void release() {
    if (joint_) { joint_->Detach();joint_->Fini();joint_.reset(); }
    if (!held_.empty()) {
      RCLCPP_INFO(node_->get_logger(),"CONTACT_RELEASE %s",held_.c_str());held_.clear();publish();
    }
  }
  void update() {
    const double time=world_->SimTime().Double();const double opening=actuator_->Position(0);
    if (joint_) {
      if (opening>grasp_opening_+.0025) release();
      if (time-last_publish_>.1) { publish();last_publish_=time; }
      return;
    }
    if (opening>.029) { candidate_.clear();contact_started_=-1.;return; }
    struct ContactSet { unsigned fingers=0;gazebo::physics::LinkPtr link; };
    std::map<std::string,ContactSet> candidates;
    const auto axis=palm_->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitY);
    for (unsigned i=0;i<contacts_->GetContactCount();++i) {
      const auto* contact=contacts_->GetContact(i);if (!contact || contact->count==0) continue;
      for (unsigned finger=0;finger<2;++finger) {
        gazebo::physics::Collision* other=nullptr;
        if (contact->collision1->GetLink()==fingers_[finger]) other=contact->collision2;
        else if (contact->collision2->GetLink()==fingers_[finger]) other=contact->collision1;
        if (!other || other->GetModel()==model_ || other->GetModel()->IsStatic()) continue;
        const auto name=other->GetModel()->GetName();
        bool workpiece=false;
        for (const auto& color : {"red", "blue", "green"}) {
          const std::string prefix=std::string(color)+"_block";
          if (name==prefix || (name.rfind(prefix+"_",0)==0 && name.size()>prefix.size()+1 &&
              name.find_first_not_of("0123456789",prefix.size()+1)==std::string::npos)) workpiece=true;
        }
        if (!workpiece) continue;
        bool side=false;
        for (int j=0;j<contact->count;++j) if (std::abs(contact->normals[j].Dot(axis))>.8) side=true;
        if (side) { auto& item=candidates[name];item.fingers|=1u<<finger;item.link=other->GetLink(); }
      }
    }
    std::string detected;gazebo::physics::LinkPtr object;
    for (const auto& [name,item]:candidates) if (item.fingers==3) { detected=name;object=item.link;break; }
    if (detected.empty()) { candidate_.clear();contact_started_=-1.;return; }
    if (candidate_!=detected) { candidate_=detected;contact_started_=time;return; }
    if (time-contact_started_<.05) return;
    joint_=world_->Physics()->CreateJoint("fixed",model_);
    joint_->SetName("classify_grasp_constraint");joint_->Load(palm_,object,ignition::math::Pose3d::Zero);joint_->Init();
    held_=detected;grasp_opening_=opening;candidate_.clear();contact_started_=-1.;
    RCLCPP_INFO(node_->get_logger(),"CONTACT_GRASP %s joint7=%.5f (two fingers, 50 ms)",held_.c_str(),opening);publish();
  }
  gazebo::physics::ModelPtr model_;gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr palm_,fingers_[2];gazebo::physics::JointPtr actuator_,joint_;
  gazebo::physics::ContactManager* contacts_{nullptr};gazebo::event::ConnectionPtr connection_;
  gazebo_ros::Node::SharedPtr node_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string held_,candidate_;double contact_started_{-1.},grasp_opening_{0.},last_publish_{0.};
};
GZ_REGISTER_MODEL_PLUGIN(ContactGrasp)
}
