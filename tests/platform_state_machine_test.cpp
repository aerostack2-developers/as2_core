#include "platform_state_machine.hpp"
#include <iostream>
#include "gtest/gtest.h"

TEST(PlatformStateMachineTest, Normal_functionality) {
  as2::PlatformStateMachine state_machine;
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::TAKE_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2::PlatformStateMachine::Event::TOOK_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::FLYING);
  state_machine.processEvent(as2::PlatformStateMachine::Event::LAND);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDING);
  state_machine.processEvent(as2::PlatformStateMachine::Event::LANDED);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::DISARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
}


TEST(PlatformStateMachineTest, EmergencyTest) {
  for (int i = -1; i < (int) as2_msgs::msg::PlatformStatus::LANDING; i++){
    as2::PlatformStateMachine state_machine;
    state_machine.setState(i);
    state_machine.processEvent(as2::PlatformStateMachine::Event::EMERGENCY);
    EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
  }
}


TEST(PlatformStateMachineTest, CorrectFSMActivation) {
  as2::PlatformStateMachine state_machine;
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::DISARMED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::ARM);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::LANDED);
  state_machine.processEvent(as2::PlatformStateMachine::Event::TAKE_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2::PlatformStateMachine::Event::LAND);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::TAKING_OFF);
  state_machine.processEvent(as2::PlatformStateMachine::Event::EMERGENCY);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
  state_machine.processEvent(as2::PlatformStateMachine::Event::TOOK_OFF);
  EXPECT_EQ(state_machine.getState().state, as2_msgs::msg::PlatformStatus::EMERGENCY);
}


int main (int argc, char *argv[])
{    
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  
}