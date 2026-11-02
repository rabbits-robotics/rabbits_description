// config/gamma_linkage.yaml の linkage.* を ROS パラメータから rabcl の設定構造体に読む。
// ミキサー（linkage_state_publisher）、rabbits_sim の偽ファーム、rabbits_gui の imu_footprint_node が共用する。
// パラメータが未宣言なら declare_parameter する（宣言済みなら読むだけなので、同じノードで複数の Load* を呼べる）。
// 足りない・型が違う・要素数が違うときは std::invalid_argument（what() にパラメータ名）
#ifndef RABBITS_DESCRIPTION__LINKAGE_PARAMS_HPP_
#define RABBITS_DESCRIPTION__LINKAGE_PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <rabcl/controller/body_posture.hpp>
#include <rabcl/controller/leg_linkage.hpp>

namespace rabbits_description
{

// 接地点の計算に要る寸法
struct FootprintParams
{
  float leg_radius;       // base 原点から脚モータ軸 M までの水平距離 [m]
  float wheel_radius;     // [m]
  double leg_azimuth[4];  // 各脚の方位角 [rad]（車体 x から左回り）FR, FL, BR, BL
};

// linkage.crank_length, crank_angle_zero, coupler_length, bar_length,
// bar_pivot[2], coupler_pin[2], wheel[2], theta_max
rabcl::LegLinkageConfig LoadLegLinkage(rclcpp::Node & node);

// LoadLegLinkage + linkage.drop_min, drop_max, tilt_radius, leg_azimuth[4]
rabcl::BodyPostureConfig LoadBodyPosture(rclcpp::Node & node);

// linkage.leg_radius, wheel_radius, leg_azimuth[4]
FootprintParams LoadFootprintParams(rclcpp::Node & node);

}  // namespace rabbits_description

#endif  // RABBITS_DESCRIPTION__LINKAGE_PARAMS_HPP_
