#include "rabbits_description/footprint.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace rabbits_description
{

namespace
{
constexpr size_t LEGS = 4;
}  // namespace

ContactPoints ContactPointsOf(const rabcl::LegPose pose[4], const FootprintParams & params)
{
  ContactPoints c = {};
  for (size_t i = 0; i < LEGS; ++i) {
    const double r = params.leg_radius + pose[i].wheel_x;
    c.p[i][0] = r * std::cos(params.leg_azimuth[i]);
    c.p[i][1] = r * std::sin(params.leg_azimuth[i]);
    c.p[i][2] = -static_cast<double>(pose[i].wheel_drop) - params.wheel_radius;
  }
  return c;
}

void PlaneFootprint(const ContactPoints & c, double quat_xyzw[4], double & height)
{
  // 地面の法線 = 対角線（FR − BL）×（FL − BR）。上向き（z > 0）に揃える
  const double d1[3] = {c.p[0][0] - c.p[3][0], c.p[0][1] - c.p[3][1], c.p[0][2] - c.p[3][2]};
  const double d2[3] = {c.p[1][0] - c.p[2][0], c.p[1][1] - c.p[2][1], c.p[1][2] - c.p[2][2]};
  double n[3] = {d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2],
    d1[0] * d2[1] - d1[1] * d2[0]};
  double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  if (len < 1e-12) {
    n[0] = 0.0;
    n[1] = 0.0;
    n[2] = 1.0;
    len = 1.0;
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
      mean[k] += c.p[i][k] / LEGS;
    }
  }
  height = -(n[0] * mean[0] + n[1] * mean[1] + n[2] * mean[2]);
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
  quat_xyzw[0] = qx;
  quat_xyzw[1] = qy;
  quat_xyzw[2] = qz;
  quat_xyzw[3] = qw;
}

double LowestWheelHeight(const ContactPoints & c, const double quat_xyzw[4])
{
  const tf2::Matrix3x3 R(tf2::Quaternion(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]));
  double min_z = 0.0;
  for (size_t i = 0; i < LEGS; ++i) {
    const tf2::Vector3 v = R * tf2::Vector3(c.p[i][0], c.p[i][1], c.p[i][2]);
    min_z = (i == 0) ? v.z() : std::min(min_z, v.z());
  }
  return -min_z;
}

}  // namespace rabbits_description
