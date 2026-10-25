#include "rabbits_description/linkage_state_publisher.hpp"

#include <cmath>
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
  publish_footprint_ = this->declare_parameter<bool>("publish_footprint", true);
  footprint_frame_ = this->declare_parameter<std::string>("footprint_frame", "base_footprint");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  leg_radius_ = static_cast<float>(this->declare_parameter<double>("linkage.leg_radius"));
  wheel_radius_ = static_cast<float>(this->declare_parameter<double>("linkage.wheel_radius"));
  leg_azimuth_ = this->declare_parameter<std::vector<double>>(
    "leg_azimuth", std::vector<double>{-M_PI / 4, M_PI / 4, -3 * M_PI / 4, 3 * M_PI / 4});
  if (leg_azimuth_.size() != LEGS) {
    throw std::invalid_argument("leg_azimuth must have 4 entries (FR, FL, BR, BL)");
  }
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
  last_stamp_ = msg->header.stamp;
  pub_->publish(out);
  if (publish_footprint_) {
    tf_->sendTransform(Footprint());
  }
}

geometry_msgs::msg::TransformStamped LinkageStatePublisher::Footprint() const
{
  // 接地点（base 座標）: 車輪中心の真下 wheel_radius
  double c[LEGS][3];
  for (size_t i = 0; i < LEGS; ++i) {
    const double r = leg_radius_ + pose_[i].wheel_x;
    c[i][0] = r * std::cos(leg_azimuth_[i]);
    c[i][1] = r * std::sin(leg_azimuth_[i]);
    c[i][2] = -static_cast<double>(pose_[i].wheel_drop) - wheel_radius_;
  }
  // 地面の法線 = 対角線（FR − BL）×（FL − BR）。上向き（z > 0）に揃える
  const double d1[3] = {c[0][0] - c[3][0], c[0][1] - c[3][1], c[0][2] - c[3][2]};
  const double d2[3] = {c[1][0] - c[2][0], c[1][1] - c[2][1], c[1][2] - c[2][2]};
  double n[3] = {d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2],
    d1[0] * d2[1] - d1[1] * d2[0]};
  double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  if (len < 1e-12) {
    n[0] = 0.0; n[1] = 0.0; n[2] = 1.0; len = 1.0;
  }
  if (n[2] < 0.0) {
    len = -len;
  }
  for (double & v : n) {
    v /= len;
  }
  // 高さ = 4 点の平均から法線方向に測った base 原点までの距離
  double mean[3] = {0.0, 0.0, 0.0};
  for (size_t i = 0; i < LEGS; ++i) {
    for (int k = 0; k < 3; ++k) {
      mean[k] += c[i][k] / LEGS;
    }
  }
  const double height = -(n[0] * mean[0] + n[1] * mean[1] + n[2] * mean[2]);
  // footprint の軸（base 座標で）: z = n、x = base の x を地面に投影、y = z × x
  double xf[3] = {1.0 - n[0] * n[0], -n[0] * n[1], -n[0] * n[2]};
  const double xl = std::sqrt(xf[0] * xf[0] + xf[1] * xf[1] + xf[2] * xf[2]);
  for (double & v : xf) {
    v /= xl;
  }
  const double yf[3] = {n[1] * xf[2] - n[2] * xf[1], n[2] * xf[0] - n[0] * xf[2],
    n[0] * xf[1] - n[1] * xf[0]};
  // base の姿勢（footprint 基準）の回転行列 R = [xf yf n]^T（行 = footprint の軸を base 座標で）
  const double R[3][3] = {{xf[0], xf[1], xf[2]}, {yf[0], yf[1], yf[2]}, {n[0], n[1], n[2]}};
  // 行列 → クォータニオン
  const double tr = R[0][0] + R[1][1] + R[2][2];
  double qw, qx, qy, qz;
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (R[2][1] - R[1][2]) / s;
    qy = (R[0][2] - R[2][0]) / s;
    qz = (R[1][0] - R[0][1]) / s;
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const double s = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
    qw = (R[2][1] - R[1][2]) / s;
    qx = 0.25 * s;
    qy = (R[0][1] + R[1][0]) / s;
    qz = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] > R[2][2]) {
    const double s = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
    qw = (R[0][2] - R[2][0]) / s;
    qx = (R[0][1] + R[1][0]) / s;
    qy = 0.25 * s;
    qz = (R[1][2] + R[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
    qw = (R[1][0] - R[0][1]) / s;
    qx = (R[0][2] + R[2][0]) / s;
    qy = (R[1][2] + R[2][1]) / s;
    qz = 0.25 * s;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = last_stamp_;
  t.header.frame_id = footprint_frame_;
  t.child_frame_id = base_frame_;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = height;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
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
