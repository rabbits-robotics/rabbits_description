#ifndef RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_
#define RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rabcl/controller/leg_linkage.hpp>

// 脚リンク機構の派生関節を足すミキサー。
// motor_joint_states（bridge / 偽ファームの関節 = モータ角）を受け、そのまま写したうえで
// 各脚の <prefix>_bar_joint（バー角）と <prefix>_coupler_joint（連結リンク角）を rabcl::LegLinkage で計算して足し、
// joint_states に 1 本で出す（1 トピック 1 発行元。robot_state_publisher はこれだけを読む）。
// URDF の関節は z 軸回りで正 = 車体が上がる向き（φ、ψ − α の上向き正と逆）。値はゼロ姿勢からの変化分:
//   bar = −(φ − φ0)、coupler = −((ψ − α) − (ψ0 − α0))。ゼロ姿勢では全関節が 0。
// 脚の無いメッセージでも直前の姿勢で派生分を足す。Forward が解けない脚は直前の値を保つ。
// 寸法は linkage.* パラメータ（config/gamma_linkage.yaml）
class LinkageStatePublisher : public rclcpp::Node
{
public:
  explicit LinkageStatePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  static constexpr size_t LEGS = 4;

  rabcl::LegLinkageConfig LoadLinkage();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::vector<std::string> leg_joints_;
  std::vector<std::string> prefixes_;
  rabcl::LegLinkage linkage_;
  rabcl::LegPose pose_[LEGS];
  rabcl::LegPose zero_ = {};  // ゼロ姿勢（θ = 0）
};

#endif  // RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_
