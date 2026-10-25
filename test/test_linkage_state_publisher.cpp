#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rabbits_description/linkage_state_publisher.hpp"

using sensor_msgs::msg::JointState;

class LinkageStatePublisherTest : public ::testing::Test
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

  void SetUp() override
  {
    // gamma の寸法（config/gamma_linkage.yaml と同じ。YAML は読まない）
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("linkage.crank_length", 0.041),
      rclcpp::Parameter("linkage.crank_angle_zero", 1.0471976),
      rclcpp::Parameter("linkage.coupler_length", 0.03975),
      rclcpp::Parameter("linkage.bar_length", 0.060),
      rclcpp::Parameter("linkage.bar_pivot", std::vector<double>{-0.026, -0.049}),
      rclcpp::Parameter("linkage.coupler_pin", std::vector<double>{0.0, 0.0175}),
      rclcpp::Parameter("linkage.wheel", std::vector<double>{0.044, -0.0325}),
      rclcpp::Parameter("linkage.theta_max", 1.5707963),
      });
    node_ = std::make_shared<LinkageStatePublisher>(options);
    sub_ = node_->create_subscription<JointState>(
      "joint_states", 10, [this](JointState::SharedPtr msg) {received_.push_back(*msg);});
  }

  void TearDown() override
  {
    sub_.reset();
    node_.reset();
  }

  // JointStateCB を直接呼び、配信を受け取る
  void Feed(const JointState & in)
  {
    const size_t before = received_.size();
    node_->JointStateCB(std::make_shared<JointState>(in));
    const auto start = std::chrono::steady_clock::now();
    while (received_.size() == before &&
      std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200))
    {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  static JointState LegMsg(float fr, float fl, float br, float bl)
  {
    JointState m;
    m.header.stamp.sec = 12;
    m.header.stamp.nanosec = 34;
    m.name = {"right_front_leg_joint", "left_front_leg_joint", "right_back_leg_joint",
      "left_back_leg_joint", "yaw_joint"};
    m.position = {fr, fl, br, bl, 0.7};
    m.velocity = {0.1, 0.2, 0.3, 0.4, 0.5};
    m.effort = {1.0, 2.0, 3.0, 4.0, 5.0};
    return m;
  }

  static double Get(const JointState & m, const std::string & name)
  {
    for (size_t i = 0; i < m.name.size(); ++i) {
      if (m.name[i] == name) {
        return m.position[i];
      }
    }
    ADD_FAILURE() << "joint not found: " << name;
    return NAN;
  }

  std::shared_ptr<LinkageStatePublisher> node_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;
  std::vector<JointState> received_;
};

TEST_F(LinkageStatePublisherTest, MixesMotorAndDerivedJoints)
{
  Feed(LegMsg(0.0f, static_cast<float>(M_PI / 6), static_cast<float>(M_PI / 3),
    static_cast<float>(M_PI / 2)));
  ASSERT_EQ(received_.size(), 1u);
  const JointState & out = received_[0];
  ASSERT_EQ(out.name.size(), 13u);
  EXPECT_EQ(out.position.size(), 13u);
  EXPECT_EQ(out.velocity.size(), 13u);
  EXPECT_EQ(out.effort.size(), 13u);
  EXPECT_EQ(out.header.stamp.sec, 12);
  EXPECT_EQ(out.header.stamp.nanosec, 34u);
  // 入力はそのまま（順も保つ）
  EXPECT_EQ(out.name[4], "yaw_joint");
  EXPECT_DOUBLE_EQ(out.position[4], 0.7);
  EXPECT_DOUBLE_EQ(out.velocity[4], 0.5);
  EXPECT_DOUBLE_EQ(out.effort[4], 5.0);
  // 派生（ゼロ姿勢からの変化分、正 = 車体が上がる向き: bar = −(φ − 0.4821)、coupler = −((ψ − α) + 2.4496)）
  EXPECT_NEAR(Get(out, "right_front_bar_joint"), 0.0, 1e-3);      // θ = 0
  EXPECT_NEAR(Get(out, "right_front_coupler_joint"), 0.0, 1e-3);
  EXPECT_NEAR(Get(out, "left_front_bar_joint"), 0.4821 - 0.2073, 1e-3);   // θ = π/6
  EXPECT_NEAR(Get(out, "right_back_bar_joint"), 0.4821 + 0.1260, 1e-3);   // θ = π/3
  EXPECT_NEAR(Get(out, "left_back_coupler_joint"), -(2.4496 - 1.2593), 1e-3);  // θ = π/2（クランクに対して上向きに回る = 負）
  EXPECT_DOUBLE_EQ(out.velocity[12], 0.0);
  EXPECT_DOUBLE_EQ(out.effort[12], 0.0);
}

TEST_F(LinkageStatePublisherTest, PassesThroughJointStatesWithoutLegs)
{
  JointState m;
  m.name = {"yaw_joint"};
  m.position = {0.3};
  Feed(m);
  ASSERT_EQ(received_.size(), 1u);
  const JointState & out = received_[0];
  ASSERT_EQ(out.name.size(), 9u);
  EXPECT_EQ(out.velocity.size(), 0u);  // 入力に無い配列は足さない
  EXPECT_NEAR(Get(out, "right_front_bar_joint"), 0.0, 1e-3);  // Forward(0) の姿勢 = ゼロ
}

TEST_F(LinkageStatePublisherTest, KeepsLastPoseWhenOutOfRange)
{
  Feed(LegMsg(0.3f, 0.3f, 0.3f, 0.3f));
  ASSERT_EQ(received_.size(), 1u);
  const double bar = Get(received_[0], "right_front_bar_joint");
  Feed(LegMsg(3.3f, 3.3f, 3.3f, 3.3f));  // 解けない θ
  ASSERT_EQ(received_.size(), 2u);
  EXPECT_DOUBLE_EQ(Get(received_[1], "right_front_bar_joint"), bar);
}
