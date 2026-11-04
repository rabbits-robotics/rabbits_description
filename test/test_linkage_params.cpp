#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rabbits_description/linkage_params.hpp"

namespace
{

// gamma_linkage.yaml と同じ値（YAML は読まない）
std::map<std::string, rclcpp::ParameterValue> GammaParams()
{
  return {
    {"linkage.crank_length", rclcpp::ParameterValue(0.041)},
    {"linkage.crank_angle_zero", rclcpp::ParameterValue(1.0471976)},
    {"linkage.coupler_length", rclcpp::ParameterValue(0.03975)},
    {"linkage.bar_length", rclcpp::ParameterValue(0.060)},
    {"linkage.bar_pivot", rclcpp::ParameterValue(std::vector<double>{-0.026, -0.049})},
    {"linkage.coupler_pin", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0175})},
    {"linkage.wheel", rclcpp::ParameterValue(std::vector<double>{0.0383, -0.044})},
    {"linkage.theta_max", rclcpp::ParameterValue(1.5707963)},
    {"linkage.drop_min", rclcpp::ParameterValue(0.06906)},
    {"linkage.drop_max", rclcpp::ParameterValue(0.11527)},
    {"linkage.tilt_radius", rclcpp::ParameterValue(0.164)},
    {"linkage.leg_radius", rclcpp::ParameterValue(0.092)},
    {"linkage.wheel_radius", rclcpp::ParameterValue(0.0375)},
    {"linkage.leg_azimuth",
      rclcpp::ParameterValue(std::vector<double>{-0.7854, 0.7854, -2.3562, 2.3562})},
  };
}

rclcpp::NodeOptions Options(const std::map<std::string, rclcpp::ParameterValue> & params)
{
  std::vector<rclcpp::Parameter> overrides;
  for (const auto & [name, value] : params) {
    overrides.emplace_back(name, value);
  }
  return rclcpp::NodeOptions().parameter_overrides(overrides);
}

rclcpp::NodeOptions GammaOptions()
{
  return Options(GammaParams());
}

}  // namespace

class LinkageParamsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(LinkageParamsTest, LoadLegLinkageReadsAllFields)
{
  auto node = std::make_shared<rclcpp::Node>("t", GammaOptions());
  const rabcl::LegLinkageConfig c = rabbits_description::LoadLegLinkage(*node);
  EXPECT_FLOAT_EQ(c.crank_length, 0.041f);
  EXPECT_FLOAT_EQ(c.crank_angle_zero, 1.0471976f);
  EXPECT_FLOAT_EQ(c.coupler_length, 0.03975f);
  EXPECT_FLOAT_EQ(c.bar_length, 0.060f);
  EXPECT_FLOAT_EQ(c.bar_pivot_x, -0.026f);
  EXPECT_FLOAT_EQ(c.bar_pivot_z, -0.049f);
  EXPECT_FLOAT_EQ(c.coupler_pin_x, 0.0f);
  EXPECT_FLOAT_EQ(c.coupler_pin_z, 0.0175f);
  EXPECT_FLOAT_EQ(c.wheel_x, 0.0383f);
  EXPECT_FLOAT_EQ(c.wheel_z, -0.044f);
  EXPECT_FLOAT_EQ(c.theta_max, 1.5707963f);
}

TEST_F(LinkageParamsTest, LoadBodyPostureReadsDropsAndAzimuth)
{
  auto node = std::make_shared<rclcpp::Node>("t", GammaOptions());
  const rabcl::BodyPostureConfig c = rabbits_description::LoadBodyPosture(*node);
  EXPECT_FLOAT_EQ(c.drop_min, 0.06906f);
  EXPECT_FLOAT_EQ(c.drop_max, 0.11527f);
  EXPECT_FLOAT_EQ(c.tilt_radius, 0.164f);
  EXPECT_FLOAT_EQ(c.leg_azimuth[0], -0.7854f);
  EXPECT_FLOAT_EQ(c.leg_azimuth[3], 2.3562f);
  EXPECT_FLOAT_EQ(c.linkage.crank_length, 0.041f);
}

TEST_F(LinkageParamsTest, LoadFootprintParamsReadsRadiiAndAzimuth)
{
  auto node = std::make_shared<rclcpp::Node>("t", GammaOptions());
  const auto p = rabbits_description::LoadFootprintParams(*node);
  EXPECT_FLOAT_EQ(p.leg_radius, 0.092f);
  EXPECT_FLOAT_EQ(p.wheel_radius, 0.0375f);
  EXPECT_DOUBLE_EQ(p.leg_azimuth[0], -0.7854);
  EXPECT_DOUBLE_EQ(p.leg_azimuth[3], 2.3562);
}

TEST_F(LinkageParamsTest, LoadingBothPostureAndFootprintOnOneNodeWorks)
{
  // leg_azimuth を両方が読む。2 回目は宣言済みのパラメータを読むだけで例外にならない
  auto node = std::make_shared<rclcpp::Node>("t", GammaOptions());
  const auto c = rabbits_description::LoadBodyPosture(*node);
  const auto p = rabbits_description::LoadFootprintParams(*node);
  EXPECT_FLOAT_EQ(c.leg_azimuth[1], 0.7854f);
  EXPECT_DOUBLE_EQ(p.leg_azimuth[1], 0.7854);
}

TEST_F(LinkageParamsTest, MissingParameterThrowsWithName)
{
  auto node = std::make_shared<rclcpp::Node>("t");  // 何も渡さない
  try {
    rabbits_description::LoadLegLinkage(*node);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("linkage.crank_length"), std::string::npos) << e.what();
  }
}

TEST_F(LinkageParamsTest, WrongPairSizeThrows)
{
  auto params = GammaParams();
  params["linkage.bar_pivot"] = rclcpp::ParameterValue(std::vector<double>{1.0});
  auto node = std::make_shared<rclcpp::Node>("t", Options(params));
  try {
    rabbits_description::LoadLegLinkage(*node);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("linkage.bar_pivot"), std::string::npos) << e.what();
  }
}

TEST_F(LinkageParamsTest, AzimuthMustHaveFourValues)
{
  auto params = GammaParams();
  params["linkage.leg_azimuth"] = rclcpp::ParameterValue(std::vector<double>{0.0, 1.0, 2.0});
  auto node = std::make_shared<rclcpp::Node>("t", Options(params));
  EXPECT_THROW(rabbits_description::LoadBodyPosture(*node), std::invalid_argument);
}

TEST_F(LinkageParamsTest, WrongTypeThrowsWithName)
{
  auto params = GammaParams();
  params["linkage.theta_max"] = rclcpp::ParameterValue(std::string("x"));
  auto node = std::make_shared<rclcpp::Node>("t", Options(params));
  try {
    rabbits_description::LoadLegLinkage(*node);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("linkage.theta_max"), std::string::npos) << e.what();
  }
}

TEST_F(LinkageParamsTest, AlreadyDeclaredWithWrongTypeThrowsWithName)
{
  auto params = GammaParams();
  params.erase("linkage.theta_max");  // override を外し、同名を先に別の型で宣言する
  auto node = std::make_shared<rclcpp::Node>("t", Options(params));
  node->declare_parameter<std::string>("linkage.theta_max", "x");
  try {
    rabbits_description::LoadLegLinkage(*node);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("linkage.theta_max"), std::string::npos) << e.what();
  }
}
