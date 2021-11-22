/*!*******************************************************************************************
 *  \file       aerial_platform.cpp
 *  \brief      Aerostack2 Aerial Platformm class implementation file.
 *  \authors    Miguel Fernandez Cortizas
 *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "aerial_platform.hpp"

namespace as2
{
AerialPlatform::AerialPlatform()
: as2::Node(std::string("platform")), state_machine_(as2::PlatformStateMachine(this))
{
  state_machine_ = as2::PlatformStateMachine(this);

  this->declare_parameter<bool>("simulation_mode", false);
  this->declare_parameter<float>("mass", 1.0);
  this->declare_parameter<float>("max_thrust", 0.0);

  this->get_parameter("simulation_mode", parameters_.simulation_mode);
  this->get_parameter("mass", parameters_.mass);
  this->get_parameter("max_thrust", parameters_.max_thrust);

  RCLCPP_INFO(this->get_logger(), "simulation_mode: %d", parameters_.simulation_mode);
  RCLCPP_INFO(this->get_logger(), "mass: %.2f kg", parameters_.mass);
  if (parameters_.max_thrust == 0) {
    RCLCPP_WARN(this->get_logger(), "max_thrust is 0 : CODE MAY FAIL IF THRUST IS NORMALIZED");
  } else {
    RCLCPP_INFO(this->get_logger(), "max_thrust: %.2f N", parameters_.max_thrust);
  }

  pose_command_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name("actuator_command/pose"), 10,
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      this->command_pose_msg_ = *msg.get();
    });

  twist_command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    this->generate_global_name("actuator_command/twist"), 10,
    [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
      this->command_twist_msg_ = *msg.get();
    });
  thrust_command_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    this->generate_global_name("actuator_command/thrust"), 10,
    [this](const as2_msgs::msg::Thrust::ConstSharedPtr msg) {
      this->command_thrust_msg_ = *msg.get();
    });

  set_platform_mode_srv_ = this->create_service<as2_msgs::srv::SetPlatformControlMode>(
    this->generate_global_name("/set_platform_control_mode"),
    std::bind(
      &AerialPlatform::setPlatformControlModeSrvCall, this,
      std::placeholders::_1,  // Corresponds to the 'request'  input
      std::placeholders::_2   // Corresponds to the 'response' input
      ));

  set_arming_state_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "set_arming_state", std::bind(
                          &AerialPlatform::setArmingStateSrvCall, this,
                          std::placeholders::_1,  // Corresponds to the 'request'  input
                          std::placeholders::_2   // Corresponds to the 'response' input
                          ));

  set_offboard_mode_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "set_offboard_mode", std::bind(
                           &AerialPlatform::setOffboardModeSrvCall, this,
                           std::placeholders::_1,  // Corresponds to the 'request'  input
                           std::placeholders::_2   // Corresponds to the 'response' input
                           ));

  platform_info_pub_ = this->create_publisher<as2_msgs::msg::PlatformInfo>(
    this->generate_global_name("platform/platform_status"), 10);

  platform_info_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((int64_t)(1000.0f / AS2_PLATFORM_INFO_PUB_FREQ_HZ)),
    std::bind(&AerialPlatform::publishPlatformInfo, this));
};

bool AerialPlatform::setArmingState(bool state)
{
  if (state == platform_info_msg_.armed && state == true) {
    RCLCPP_INFO(this->get_logger(), "UAV is already armed");
  } else if (state == platform_info_msg_.armed && state == false) {
    RCLCPP_INFO(this->get_logger(), "UAV is already disarmed");
  } else {
    if (ownSetArmingState(state)) {
      platform_info_msg_.armed = state;
      if (state) {
        handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
      } else {
        handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::DISARM);
      }
      return true;
    }
  }
  return false;
};

bool AerialPlatform::setOffboardControl(bool offboard)
{
  if (offboard == platform_info_msg_.offboard && offboard == true) {
    RCLCPP_INFO(this->get_logger(), "UAV is already in OFFBOARD mode");
  } else if (offboard == platform_info_msg_.offboard && offboard == false) {
    RCLCPP_INFO(this->get_logger(), "UAV is already in MANUAL mode");
  } else {
    if (ownSetOffboardControl(offboard)) {
      platform_info_msg_.offboard = offboard;
      return true;
    }
  }
  return false;
};

// as2_msgs::msg::PlatformInfo AerialPlatform::setPlatformInfo()
// {
//   platform_info_msg_ = *(ownSetPlatformInfo().get());
//   return platform_info_msg_;
// };

bool AerialPlatform::setPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg)
{
  return ownSetPlatformControlMode(msg);
};

bool AerialPlatform::sendCommand()
{
  if (!isControlModeSettled()) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Platform control mode is not settled yet");
    return false;
  } else {
    return ownSendCommand();
  }
};

//Services Callbacks

void AerialPlatform::setPlatformControlModeSrvCall(
  const std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Request> request,
  std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Response> response)
{
  bool success = this->setPlatformControlMode(request->control_mode);
  response->success = success;
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: UNABLE TO SET THIS CONTROL MODE TO THIS PLATFORM");
  } else {
    platform_info_msg_.current_control_mode = request->control_mode;
  }
};

void AerialPlatform::setOffboardModeSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setOffboardControl(request->data);
  if (response->success) {
    platform_info_msg_.offboard = request->data;
  }
}

void AerialPlatform::setArmingStateSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setArmingState(request->data);
  if (response->success) {
    platform_info_msg_.armed = request->data;
  }
}

};  // namespace as2