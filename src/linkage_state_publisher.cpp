#include "rabbits_description/linkage_state_publisher.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr const char * DEFAULT_LEG_JOINTS[] = {
  "right_front_leg_joint", "left_front_leg_joint", "right_back_leg_joint", "left_back_leg_joint"};
constexpr const char * DEFAULT_PREFIXES[] = {"right_front", "left_front", "right_back",
  "left_back"};

// [x, z] の 2 要素パラメータ
void LoadPair(rclcpp::Node & node, const std::string & name, float & x, float & z)
{
  const std::vector<double> v = node.declare_parameter<std::vector<double>>(name);
  if (v.size() != 2) {
    throw std::invalid_argument("parameter '" + name + "' must have 2 values [x, z]");
  }
  x = static_cast<float>(v[0]);
  z = static_cast<float>(v[1]);
}
}  // namespace

LinkageStatePublisher::LinkageStatePublisher(const rclcpp::NodeOptions & options)
: Node("linkage_state_publisher", options),
  leg_joints_(this->declare_parameter<std::vector<std::string>>(
      "leg_joints", std::vector<std::string>(DEFAULT_LEG_JOINTS, DEFAULT_LEG_JOINTS + LEGS))),
  prefixes_(this->declare_parameter<std::vector<std::string>>(
      "prefixes", std::vector<std::string>(DEFAULT_PREFIXES, DEFAULT_PREFIXES + LEGS))),
  linkage_(LoadLinkage())
{
  if (leg_joints_.size() != LEGS || prefixes_.size() != LEGS) {
    throw std::invalid_argument("leg_joints and prefixes must have 4 entries");
  }
  linkage_.Forward(0.0f, zero_);
  for (size_t i = 0; i < LEGS; ++i) {
    pose_[i] = zero_;
  }
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "motor_joint_states", 10,
    std::bind(&LinkageStatePublisher::JointStateCB, this, std::placeholders::_1));
}

rabcl::LegLinkageConfig LinkageStatePublisher::LoadLinkage()
{
  rabcl::LegLinkageConfig c = {};
  c.crank_length = static_cast<float>(this->declare_parameter<double>("linkage.crank_length"));
  c.crank_angle_zero =
    static_cast<float>(this->declare_parameter<double>("linkage.crank_angle_zero"));
  c.coupler_length = static_cast<float>(this->declare_parameter<double>("linkage.coupler_length"));
  c.bar_length = static_cast<float>(this->declare_parameter<double>("linkage.bar_length"));
  LoadPair(*this, "linkage.bar_pivot", c.bar_pivot_x, c.bar_pivot_z);
  LoadPair(*this, "linkage.coupler_pin", c.coupler_pin_x, c.coupler_pin_z);
  LoadPair(*this, "linkage.wheel", c.wheel_x, c.wheel_z);
  c.theta_max = static_cast<float>(this->declare_parameter<double>("linkage.theta_max"));
  return c;
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
  pub_->publish(out);
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
