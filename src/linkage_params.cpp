#include "rabbits_description/linkage_params.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace rabbits_description
{

namespace
{

// 宣言済みなら読む、未宣言なら宣言して読む。無い・型違いは std::invalid_argument
template<typename T>
T Require(rclcpp::Node & node, const std::string & name)
{
  try {
    if (node.has_parameter(name)) {
      return node.get_parameter(name).get_value<T>();
    }
    return node.declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    throw std::invalid_argument("parameter '" + name + "' is required (pass gamma_linkage.yaml)");
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    throw std::invalid_argument("parameter '" + name + "' is required (pass gamma_linkage.yaml)");
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    throw std::invalid_argument("parameter '" + name + "': " + e.what());
  }
}

float RequireFloat(rclcpp::Node & node, const std::string & name)
{
  return static_cast<float>(Require<double>(node, name));
}

// 要素数 n の double 配列
std::vector<double> RequireArray(rclcpp::Node & node, const std::string & name, size_t n)
{
  const std::vector<double> v = Require<std::vector<double>>(node, name);
  if (v.size() != n) {
    throw std::invalid_argument(
      "parameter '" + name + "' must have " + std::to_string(n) + " values");
  }
  return v;
}

// [x, z] の 2 要素
void RequirePair(rclcpp::Node & node, const std::string & name, float & x, float & z)
{
  const std::vector<double> v = RequireArray(node, name, 2);
  x = static_cast<float>(v[0]);
  z = static_cast<float>(v[1]);
}

}  // namespace

rabcl::LegLinkageConfig LoadLegLinkage(rclcpp::Node & node)
{
  rabcl::LegLinkageConfig c = {};
  c.crank_length = RequireFloat(node, "linkage.crank_length");
  c.crank_angle_zero = RequireFloat(node, "linkage.crank_angle_zero");
  c.coupler_length = RequireFloat(node, "linkage.coupler_length");
  c.bar_length = RequireFloat(node, "linkage.bar_length");
  RequirePair(node, "linkage.bar_pivot", c.bar_pivot_x, c.bar_pivot_z);
  RequirePair(node, "linkage.coupler_pin", c.coupler_pin_x, c.coupler_pin_z);
  RequirePair(node, "linkage.wheel", c.wheel_x, c.wheel_z);
  c.theta_max = RequireFloat(node, "linkage.theta_max");
  return c;
}

rabcl::BodyPostureConfig LoadBodyPosture(rclcpp::Node & node)
{
  rabcl::BodyPostureConfig c = {};
  c.linkage = LoadLegLinkage(node);
  c.drop_min = RequireFloat(node, "linkage.drop_min");
  c.drop_max = RequireFloat(node, "linkage.drop_max");
  c.tilt_radius = RequireFloat(node, "linkage.tilt_radius");
  const std::vector<double> az = RequireArray(node, "linkage.leg_azimuth", 4);
  for (size_t i = 0; i < 4; ++i) {
    c.leg_azimuth[i] = static_cast<float>(az[i]);
  }
  return c;
}

FootprintParams LoadFootprintParams(rclcpp::Node & node)
{
  FootprintParams p = {};
  p.leg_radius = RequireFloat(node, "linkage.leg_radius");
  p.wheel_radius = RequireFloat(node, "linkage.wheel_radius");
  const std::vector<double> az = RequireArray(node, "linkage.leg_azimuth", 4);
  for (size_t i = 0; i < 4; ++i) {
    p.leg_azimuth[i] = az[i];
  }
  return p;
}

}  // namespace rabbits_description
