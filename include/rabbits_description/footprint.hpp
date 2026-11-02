// 4 輪の接地点と、それを地面 z = 0 に乗せる胴体の置き方（base_footprint → base_link の回転と高さ）。
// ミキサー（linkage_state_publisher）と rabbits_gui の imu_footprint_node が共用する。
// 座標は base 座標（x 前、y 左、z 上）。footprint は yaw を含まない（footprint の x = base の x を地面に投影した向き）
#ifndef RABBITS_DESCRIPTION__FOOTPRINT_HPP_
#define RABBITS_DESCRIPTION__FOOTPRINT_HPP_

#include <rabcl/controller/leg_linkage.hpp>

#include "rabbits_description/linkage_params.hpp"

namespace rabbits_description
{

// base 座標の接地点（FR, FL, BR, BL）
struct ContactPoints
{
  double p[4][3];
};

// 接地点 = 車輪中心の真下 wheel_radius。
// 水平距離 leg_radius + pose.wheel_x、方位 leg_azimuth[i]、z = −wheel_drop − wheel_radius
ContactPoints ContactPointsOf(const rabcl::LegPose pose[4], const FootprintParams & params);

// 4 輪が同じ平面に接地するとした footprint: 対角線 (p0 − p3) × (p1 − p2) を上向きに揃えた法線、
// 4 点の平均から法線方向に測った base 原点の高さ。quat_xyzw は footprint 基準の base の姿勢（yaw なし）
void PlaneFootprint(const ContactPoints & c, double quat_xyzw[4], double & height);

// 与えた姿勢 R（footprint 基準の base、xyzw）で最も低い接地点が z = 0 に乗る高さ = −min_i (R·p_i).z
double LowestWheelHeight(const ContactPoints & c, const double quat_xyzw[4]);

}  // namespace rabbits_description

#endif  // RABBITS_DESCRIPTION__FOOTPRINT_HPP_
