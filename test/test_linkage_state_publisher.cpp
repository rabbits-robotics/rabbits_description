#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>

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

  // gamma の寸法（config/gamma_linkage.yaml と同じ。YAML は読まない）
  static std::vector<rclcpp::Parameter> GammaOverrides()
  {
    return {
      rclcpp::Parameter("linkage.crank_length", 0.041),
      rclcpp::Parameter("linkage.crank_angle_zero", 1.0471976),
      rclcpp::Parameter("linkage.coupler_length", 0.03975),
      rclcpp::Parameter("linkage.bar_length", 0.060),
      rclcpp::Parameter("linkage.bar_pivot", std::vector<double>{-0.026, -0.049}),
      rclcpp::Parameter("linkage.coupler_pin", std::vector<double>{0.0, 0.0175}),
      rclcpp::Parameter("linkage.wheel", std::vector<double>{0.0383, -0.044}),
      rclcpp::Parameter("linkage.theta_max", 1.5707963),
      rclcpp::Parameter("linkage.leg_radius", 0.092),
      rclcpp::Parameter("linkage.wheel_radius", 0.0375),
      rclcpp::Parameter("linkage.drop_min", 0.06906),
      rclcpp::Parameter("linkage.drop_max", 0.11527),
      rclcpp::Parameter("linkage.tilt_radius", 0.164),
      rclcpp::Parameter(
        "linkage.leg_azimuth", std::vector<double>{-0.7854, 0.7854, -2.3562, 2.3562}),
    };
  }

  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(GammaOverrides());
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

  // base_footprint → base_link の roll / pitch / yaw と高さ
  struct Footprint
  {
    double roll, pitch, yaw, height;
  };
  Footprint GetFootprint() const
  {
    const geometry_msgs::msg::TransformStamped t = node_->Footprint();
    EXPECT_EQ(t.header.frame_id, "base_footprint");
    EXPECT_EQ(t.child_frame_id, "base_link");
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
      t.transform.rotation.w);
    Footprint f;
    tf2::Matrix3x3(q).getRPY(f.roll, f.pitch, f.yaw);
    f.height = t.transform.translation.z;
    EXPECT_DOUBLE_EQ(t.transform.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(t.transform.translation.y, 0.0);
    return f;
  }

  std::shared_ptr<LinkageStatePublisher> node_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;
  std::vector<JointState> received_;
};

TEST_F(LinkageStatePublisherTest, FootprintIsLevelWhenLegsAreEqual)
{
  Feed(LegMsg(0.15f, 0.15f, 0.15f, 0.15f));
  const Footprint f = GetFootprint();
  EXPECT_NEAR(f.roll, 0.0, 1e-6);
  EXPECT_NEAR(f.pitch, 0.0, 1e-6);
  EXPECT_NEAR(f.yaw, 0.0, 1e-6);
  EXPECT_NEAR(f.height, 0.06906 + 0.0375, 1e-4);  // 車輪の下がり + 車輪半径
}

TEST_F(LinkageStatePublisherTest, FootprintRollsRightDownWhenRightLegsAreShallower)
{
  // 右脚の下がりが小さい = 右側が低い → REP-103 でロール正。差 0.0462 m / 左右の接地点間 約 0.226 m
  Feed(LegMsg(0.15f, 1.42f, 0.15f, 1.42f));
  const Footprint f = GetFootprint();
  EXPECT_NEAR(f.roll, std::atan2(0.11527 - 0.06906, 0.226), 0.01);
  EXPECT_NEAR(f.pitch, 0.0, 1e-6);
  EXPECT_NEAR(f.yaw, 0.0, 1e-6);
}

TEST_F(LinkageStatePublisherTest, FootprintPitchesFrontDownWhenFrontLegsAreShallower)
{
  Feed(LegMsg(0.15f, 0.15f, 1.42f, 1.42f));
  const Footprint f = GetFootprint();
  EXPECT_GT(f.pitch, 0.1);  // 前が低い = ピッチ正
  EXPECT_NEAR(f.roll, 0.0, 1e-6);
}

TEST_F(LinkageStatePublisherTest, FootprintIsBroadcastOnTf)
{
  std::vector<tf2_msgs::msg::TFMessage> tfs;
  auto sub = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&](tf2_msgs::msg::TFMessage::SharedPtr m) {tfs.push_back(*m);});
  Feed(LegMsg(0.3f, 0.3f, 0.3f, 0.3f));
  const auto start = std::chrono::steady_clock::now();
  while (tfs.empty() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(300)) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(tfs.empty());
  ASSERT_EQ(tfs[0].transforms.size(), 1u);
  EXPECT_EQ(tfs[0].transforms[0].header.frame_id, "base_footprint");
  EXPECT_EQ(tfs[0].transforms[0].child_frame_id, "base_link");
  EXPECT_EQ(tfs[0].transforms[0].header.stamp.sec, 12);
}

TEST_F(LinkageStatePublisherTest, MixesMotorAndDerivedJoints)
{
  Feed(LegMsg(0.0f, static_cast<float>(M_PI / 6), static_cast<float>(M_PI / 3),
    static_cast<float>(M_PI / 2)));
  ASSERT_EQ(received_.size(), 1u);
  const JointState & out = received_[0];
  ASSERT_EQ(out.name.size(), 15u);  // 入力 5 + 首の 0 埋め 2（pitch, roll）+ 派生 8
  EXPECT_EQ(out.position.size(), 15u);
  EXPECT_EQ(out.velocity.size(), 15u);
  EXPECT_EQ(out.effort.size(), 15u);
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
  EXPECT_DOUBLE_EQ(out.velocity[14], 0.0);
  EXPECT_DOUBLE_EQ(out.effort[14], 0.0);
}

TEST_F(LinkageStatePublisherTest, PassesThroughJointStatesWithoutLegs)
{
  JointState m;
  m.name = {"yaw_joint"};
  m.position = {0.3};
  Feed(m);
  ASSERT_EQ(received_.size(), 1u);
  const JointState & out = received_[0];
  ASSERT_EQ(out.name.size(), 11u);  // 入力 1 + 首の 0 埋め 2 + 派生 8
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

TEST_F(LinkageStatePublisherTest, FillsMissingNeckJointsWithZero)
{
  JointState in = LegMsg(0.5f, 0.5f, 0.5f, 0.5f);  // yaw_joint は入っている、pitch / roll は無い
  Feed(in);
  ASSERT_FALSE(received_.empty());
  const JointState & out = received_.back();
  auto idx = [&](const std::string & n) {
      return std::find(out.name.begin(), out.name.end(), n) - out.name.begin();
    };
  ASSERT_LT(idx("pitch_joint"), static_cast<long>(out.name.size()));
  ASSERT_LT(idx("roll_joint"), static_cast<long>(out.name.size()));
  EXPECT_DOUBLE_EQ(out.position[idx("pitch_joint")], 0.0);
  EXPECT_DOUBLE_EQ(out.velocity[idx("roll_joint")], 0.0);
  EXPECT_DOUBLE_EQ(out.effort[idx("roll_joint")], 0.0);
  EXPECT_EQ(out.name.size(), out.position.size());
  EXPECT_EQ(out.name.size(), out.velocity.size());
  EXPECT_EQ(out.name.size(), out.effort.size());
}

TEST_F(LinkageStatePublisherTest, DoesNotDuplicateJointsPresentInInput)
{
  JointState in = LegMsg(0.5f, 0.5f, 0.5f, 0.5f);  // yaw_joint = 0.7 入り
  Feed(in);
  ASSERT_FALSE(received_.empty());
  const JointState & out = received_.back();
  EXPECT_EQ(std::count(out.name.begin(), out.name.end(), "yaw_joint"), 1);
  const auto i = std::find(out.name.begin(), out.name.end(), "yaw_joint") - out.name.begin();
  EXPECT_DOUBLE_EQ(out.position[i], 0.7);
}

TEST_F(LinkageStatePublisherTest, FillZeroJointsCanBeEmptied)
{
  auto overrides = GammaOverrides();
  overrides.emplace_back("fill_zero_joints", std::vector<std::string>{});
  auto options = rclcpp::NodeOptions().parameter_overrides(overrides);
  auto node = std::make_shared<LinkageStatePublisher>(options);
  std::vector<JointState> got;
  auto sub = node->create_subscription<JointState>(
    "joint_states", 10, [&](JointState::SharedPtr m) {got.push_back(*m);});
  node->JointStateCB(std::make_shared<JointState>(LegMsg(0.5f, 0.5f, 0.5f, 0.5f)));
  const auto start = std::chrono::steady_clock::now();
  while (got.empty() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(300)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got[0].name.size(), 13u);  // 入力 5 + 派生 8 のみ
}
