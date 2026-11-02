#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "rabbits_description/footprint.hpp"

namespace
{

rabbits_description::FootprintParams Params()
{
  return {0.092f, 0.0375f, {-M_PI / 4, M_PI / 4, -3 * M_PI / 4, 3 * M_PI / 4}};
}

rabcl::LegPose Pose(float drop)
{
  rabcl::LegPose p = {};
  p.wheel_x = 0.04f;
  p.wheel_drop = drop;
  return p;
}

void Rpy(const double q[4], double & r, double & p, double & y)
{
  tf2::Matrix3x3(tf2::Quaternion(q[0], q[1], q[2], q[3])).getRPY(r, p, y);
}

}  // namespace

TEST(Footprint, ContactPointsUseRadiusAzimuthAndDrop)
{
  const rabcl::LegPose pose[4] = {Pose(0.1f), Pose(0.1f), Pose(0.1f), Pose(0.1f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  const double r = 0.092 + 0.04;
  EXPECT_NEAR(c.p[0][0], r * std::cos(-M_PI / 4), 1e-6);
  EXPECT_NEAR(c.p[0][1], r * std::sin(-M_PI / 4), 1e-6);
  EXPECT_NEAR(c.p[0][2], -0.1 - 0.0375, 1e-6);
  EXPECT_NEAR(c.p[1][1], r * std::sin(M_PI / 4), 1e-6);
  EXPECT_NEAR(c.p[3][0], r * std::cos(3 * M_PI / 4), 1e-6);
}

TEST(Footprint, PlaneIsLevelWhenDropsAreEqual)
{
  const rabcl::LegPose pose[4] = {Pose(0.1f), Pose(0.1f), Pose(0.1f), Pose(0.1f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  double q[4];
  double h;
  rabbits_description::PlaneFootprint(c, q, h);
  double r, p, y;
  Rpy(q, r, p, y);
  EXPECT_NEAR(r, 0.0, 1e-9);
  EXPECT_NEAR(p, 0.0, 1e-9);
  EXPECT_NEAR(y, 0.0, 1e-9);
  EXPECT_NEAR(h, 0.1 + 0.0375, 1e-6);
}

TEST(Footprint, PlanePitchesFrontDownWhenFrontDropsAreSmaller)
{
  const rabcl::LegPose pose[4] = {Pose(0.08f), Pose(0.08f), Pose(0.12f), Pose(0.12f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  double q[4];
  double h;
  rabbits_description::PlaneFootprint(c, q, h);
  double r, p, y;
  Rpy(q, r, p, y);
  EXPECT_GT(p, 0.01);  // REP-103: ピッチ正 = 前が下がる
  EXPECT_NEAR(r, 0.0, 1e-9);
  EXPECT_NEAR(y, 0.0, 1e-9);
}

TEST(Footprint, PlaneRollsRightDownWhenRightDropsAreSmaller)
{
  const rabcl::LegPose pose[4] = {Pose(0.08f), Pose(0.12f), Pose(0.08f), Pose(0.12f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  double q[4];
  double h;
  rabbits_description::PlaneFootprint(c, q, h);
  double r, p, y;
  Rpy(q, r, p, y);
  EXPECT_GT(r, 0.01);  // ロール正 = 右が下がる
  EXPECT_NEAR(p, 0.0, 1e-9);
}

TEST(Footprint, LowestWheelHeightPutsLowestContactOnGround)
{
  const rabcl::LegPose pose[4] = {Pose(0.08f), Pose(0.12f), Pose(0.10f), Pose(0.10f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  tf2::Quaternion q;
  q.setRPY(0.1, -0.05, 0.0);
  const double qa[4] = {q.x(), q.y(), q.z(), q.w()};
  const double h = rabbits_description::LowestWheelHeight(c, qa);
  const tf2::Matrix3x3 R(q);
  double min_z = 1e9;
  for (int i = 0; i < 4; ++i) {
    const tf2::Vector3 v = R * tf2::Vector3(c.p[i][0], c.p[i][1], c.p[i][2]);
    min_z = std::min(min_z, v.z() + h);
    EXPECT_GE(v.z() + h, -1e-9);
  }
  EXPECT_NEAR(min_z, 0.0, 1e-9);
}

TEST(Footprint, LowestWheelHeightEqualsPlaneHeightWhenLevel)
{
  const rabcl::LegPose pose[4] = {Pose(0.1f), Pose(0.1f), Pose(0.1f), Pose(0.1f)};
  const auto c = rabbits_description::ContactPointsOf(pose, Params());
  double q[4];
  double h;
  rabbits_description::PlaneFootprint(c, q, h);
  EXPECT_NEAR(rabbits_description::LowestWheelHeight(c, q), h, 1e-9);
}
