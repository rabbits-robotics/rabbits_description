#ifndef RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_
#define RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rabcl/controller/leg_linkage.hpp>

// 脚リンク機構の派生関節を足すミキサー。
// motor_joint_states（bridge / 偽ファームの関節 = モータ角）を受け、そのまま写したうえで
// 各脚の <prefix>_bar_joint（バー角）と <prefix>_coupler_joint（連結リンク角）を rabcl::LegLinkage で計算して足し、
// joint_states に 1 本で出す（1 トピック 1 発行元。robot_state_publisher はこれだけを読む）。
// URDF の関節は z 軸回りで正 = 車体が上がる向き（φ、ψ − α の上向き正と逆）。値はゼロ姿勢からの変化分:
//   bar = −(φ − φ0)、coupler = −((ψ − α) − (ψ0 − α0))。ゼロ姿勢では全関節が 0。
// 脚の無いメッセージでも直前の姿勢で派生分を足す。Forward が解けない脚は直前の値を保つ。
// 寸法は linkage.* パラメータ（config/gamma_linkage.yaml）。
// あわせて base_footprint → base_link の TF を出す（publish_footprint）: 4 輪の接地点（車輪中心の真下 wheel_radius）が
// 地面 z = 0 に乗るように、対角線 2 本の外積で地面の法線、4 点の平均で高さを決める。yaw は含めない
// （footprint の x は胴体の前を地面に投影した向き）。rviz の固定フレームを base_footprint にすると胴体の傾きが見える
class LinkageStatePublisher : public rclcpp::Node
{
public:
  explicit LinkageStatePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void JointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);
  // 今の脚の姿勢から base_footprint → base_link（stamp は最後に受けたメッセージ）
  geometry_msgs::msg::TransformStamped Footprint() const;

private:
  static constexpr size_t LEGS = 4;

  rabcl::LegLinkageConfig LoadLinkage();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  bool publish_footprint_ = true;
  std::string footprint_frame_;
  std::string base_frame_;
  float leg_radius_ = 0.0f;
  float wheel_radius_ = 0.0f;
  std::vector<double> leg_azimuth_;
  builtin_interfaces::msg::Time last_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::vector<std::string> leg_joints_;
  std::vector<std::string> prefixes_;
  rabcl::LegLinkage linkage_;
  rabcl::LegPose pose_[LEGS];
  rabcl::LegPose zero_ = {};  // ゼロ姿勢（θ = 0）
};

#endif  // RABBITS_DESCRIPTION__LINKAGE_STATE_PUBLISHER_HPP_
