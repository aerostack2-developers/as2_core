#include "platform_state_machine.hpp"

#include <iostream>

// #include "core_libs.hpp"
#include "gtest/gtest.h"

TEST(PlatformStateMachineTest, Normal_functionality)
{
  as2::Node test_node("test_node");
  as2::PlatformStateMachine state_machine(&test_node);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::FLYING);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::LAND);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDING);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::LANDED);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::DISARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
}

TEST(PlatformStateMachineTest, EmergencyTest)
{
  as2::Node test_node("test_node");
  for (int i = -1; i < (int)as2_msgs::msg::PlatformStatus::LANDING; i++) {
    as2::PlatformStateMachine state_machine(&test_node);
    state_machine.setState(i);
    state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::EMERGENCY);
    EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
  }
}

TEST(PlatformStateMachineTest, CorrectFSMActivation)
{
  as2::Node test_node("test_node");
  as2::PlatformStateMachine state_machine(&test_node);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::TAKE_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::LAND);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::EMERGENCY);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
  state_machine.processEvent(as2_msgs::msg::PlatformStateMachineEvent::TOOK_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}