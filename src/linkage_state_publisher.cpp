#include "rabbits_description/linkage_state_publisher.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rabbits_description/footprint.hpp"

namespace
{
constexpr const char * DEFAULT_LEG_JOINTS[] = {
  "right_front_leg_joint", "left_front_leg_joint", "right_back_leg_joint", "left_back_leg_joint"};
constexpr const char * DEFAULT_PREFIXES[] = {"right_front", "left_front", "right_back",
  "left_back"};
constexpr const char * DEFAULT_FILL_ZERO_JOINTS[] = {"yaw_joint", "pitch_joint", "roll_joint"};

}  // namespace

LinkageStatePublisher::LinkageStatePublisher(const rclcpp::NodeOptions & options)
: Node("linkage_state_publisher", options),
  leg_joints_(this->declare_parameter<std::vector<std::string>>(
      "leg_joints", std::vector<std::string>(DEFAULT_LEG_JOINTS, DEFAULT_LEG_JOINTS + LEGS))),
  prefixes_(this->declare_parameter<std::vector<std::string>>(
      "prefixes", std::vector<std::string>(DEFAULT_PREFIXES, DEFAULT_PREFIXES + LEGS))),
  linkage_(rabbits_description::LoadLegLinkage(*this))
{
  if (leg_joints_.size() != LEGS || prefixes_.size() != LEGS) {
    throw std::invalid_argument("leg_joints and prefixes must have 4 entries");
  }
  linkage_.Forward(0.0f, zero_);
  for (size_t i = 0; i < LEGS; ++i) {
    pose_[i] = zero_;
  }
  publish_footprint_ = this->declare_parameter<bool>("publish_footprint", true);
  footprint_frame_ = this->declare_parameter<std::string>("footprint_frame", "base_footprint");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  footprint_ = rabbits_description::LoadFootprintParams(*this);
  fill_zero_joints_ = this->declare_parameter<std::vector<std::string>>(
    "fill_zero_joints",
    std::vector<std::string>(DEFAULT_FILL_ZERO_JOINTS, DEFAULT_FILL_ZERO_JOINTS + 3));
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "motor_joint_states", 10,
    std::bind(&LinkageStatePublisher::JointStateCB, this, std::placeholders::_1));
}

void LinkageStatePublisher::JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  sensor_msgs::msg::JointState out = *msg;
  for (size_t i = 0; i < LEGS; ++i) {
    for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      if (msg->name[k] == leg_joints_[i]) {
        rabcl::LegPose p;
        if (linkage_.Forward(static_cast<float>(msg->position[k]), p)) {
          pose_[i] = p;  // 解けない θ のときは直前の姿勢を保つ
        }
        break;
      }
    }
  }
  const bool has_velocity = !msg->velocity.empty();
  const bool has_effort = !msg->effort.empty();
  // 入力に無い首関節を 0 で埋める（TF の鎖を切らない）
  for (const std::string & name : fill_zero_joints_) {
    if (std::find(msg->name.begin(), msg->name.end(), name) != msg->name.end()) {
      continue;
    }
    out.name.push_back(name);
    out.position.push_back(0.0);
    if (has_velocity) {
      out.velocity.push_back(0.0);
    }
    if (has_effort) {
      out.effort.push_back(0.0);
    }
  }
  for (size_t i = 0; i < LEGS; ++i) {
    // URDF の関節は正 = 車体が上がる向き（φ、ψ − α と逆）で、ゼロ姿勢からの変化分
    out.name.push_back(prefixes_[i] + "_bar_joint");
    out.position.push_back(-static_cast<double>(pose_[i].bar_angle - zero_.bar_angle));
    out.name.push_back(prefixes_[i] + "_coupler_joint");
    out.position.push_back(-static_cast<double>(pose_[i].coupler_angle - zero_.coupler_angle));
    for (int n = 0; n < 2; ++n) {
      if (has_velocity) {
        out.velocity.push_back(0.0);
      }
      if (has_effort) {
        out.effort.push_back(0.0);
      }
    }
  }
  last_stamp_ = msg->header.stamp;
  pub_->publish(out);
  if (publish_footprint_) {
    tf_->sendTransform(Footprint());
  }
}

geometry_msgs::msg::TransformStamped LinkageStatePublisher::Footprint() const
{
  const rabbits_description::ContactPoints c =
    rabbits_description::ContactPointsOf(pose_, footprint_);
  double q[4];
  double height;
  rabbits_description::PlaneFootprint(c, q, height);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = last_stamp_;
  t.header.frame_id = footprint_frame_;
  t.child_frame_id = base_frame_;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = height;
  t.transform.rotation.x = q[0];
  t.transform.rotation.y = q[1];
  t.transform.rotation.z = q[2];
  t.transform.rotation.w = q[3];
  return t;
}

#ifndef RABBITS_DESCRIPTION_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkageStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
#endif
