/*!*******************************************************************************************
 *  \file       aerial_platform.hpp
 *  \brief      Aerostack2 Aerial Platformm class header file.
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

#ifndef AEROSTACK2_AERIAL_PLATFORM_HPP_
#define AEROSTACK2_AERIAL_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "as2_core/platform_state_machine.hpp"
#include "as2_msgs/msg/platform_control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "as2_msgs/srv/set_platform_control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#define AS2_PLATFORM_INFO_PUB_FREQ_HZ 10

namespace as2
{
// TODO: Rethink how this parameters are used and how they are set.
struct AerialPlatformParameters
{
  float mass;
  float max_thrust;
  bool simulation_mode;
};

/**
 * @brief Base class for all Aerial platforms. It provides the basic functionality for the platform.
 *  It is responsible for handling the platform state machine and the platform status.
 *  It also handles the command subscriptions and the basic platform services.
 */

//TODO: Validate all the system in Pixhawk Class
class AerialPlatform : public as2::Node
{
private:
  // bool control_mode_settled_ = false;
  bool sending_commands_ = false;

  as2::AerialPlatformParameters parameters_;
  rclcpp::TimerBase::SharedPtr platform_info_timer_;
  as2::PlatformStateMachine state_machine_;

protected:
  geometry_msgs::msg::PoseStamped command_pose_msg_;
  geometry_msgs::msg::TwistStamped command_twist_msg_;
  as2_msgs::msg::Thrust command_thrust_msg_;
  as2_msgs::msg::PlatformInfo platform_info_msg_;

public:
  /**
   * @brief Construct a new Aerial Platform object, with default parameters.
   * 
   */
  AerialPlatform();

  /**
   * @brief Configures the platform sensors
   * 
   */
  virtual void configureSensors() = 0;

  /**
   * @brief publishes all the platform sensors data
   */
  virtual void publishSensorData() = 0;

  /**
   * @brief Handles how a command must be sended in the concrete platform.
   * 
   * @return true command is sended successfully.
   * @return false command is not sended.
   */
  virtual bool ownSendCommand() = 0;

  /**
   * @brief Handles how arming state has to be settled  in the concrete platform.
   * 
   * @param state true for arming the platform, false to disarm.
   * @return true Arming state is settled successfully.
   * @return false Arming state is not settled.
   */
  virtual bool ownSetArmingState(bool state) = 0;

  /**
   * @brief Handles how offboard mode has to be settled  in the concrete platform.
   * 
   * @param offboard true if offboard mode is enabled.
   * @return true Offboard mode is settled successfully.
   * @return false Offboard mode is not settled.
   */
  virtual bool ownSetOffboardControl(bool offboard) = 0;

  /**
   * @brief Handles how the control mode has to be settled  in the concrete platform.
   * 
   * @param control_mode as2_msgs::msg::PlatformControlMode with the new control mode.
   * @return true Control mode is settled successfully.
   * @return false Control mode is not settled.
   */
  virtual bool ownSetPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg) = 0;

private:
  /**
   * @brief Set the arm state of the platform.
   * 
   * @param state True to arm the platform, false to disarm it.
   * @return true Armimg state setted successfully.
   * @return false Armimg state not setted successfully.
   */
  bool setArmingState(bool state);

  /**
   * @brief  Set the offboard control mode.
   * 
   * @param offboard  True if the offboard control mode is enabled.
   * @return true if the offboard control mode is setted properly
   * @return false if the offboard control mode could not be setted.
   */
  bool setOffboardControl(bool offboard);

  /**
   * @brief  Set the control mode of the platform.
   * 
   * @param msg as2_msgs::msg::PlatformControlMode message with the new control mode desired.
   * @return true  If the control mode is set properly.
   * @return false If the control mode could not be set properly.
   */
  bool setPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg);

  /**
   * @brief Send command to the platform.
   * @return true if the command was sent successfully, false otherwise
   */
  bool sendCommand();

  // Getters
public:
  /**
   * @brief Set the State Machine Event object
   * 
   * @param event Event to 
   * @return true 
   * @return false 
   */
  bool handleStateMachineEvent(const as2_msgs::msg::PlatformStateMachineEvent & event)
  {
    return state_machine_.processEvent(event);
  };

  bool handleStateMachineEvent(const int8_t & event) { return state_machine_.processEvent(event); };

  /**
   * @brief Get the Flag Simulation Mode object
   * 
   * @return true The platform is in simulation mode
   * @return false There is a real platform
   */
  inline bool getFlagSimulationMode() const { return parameters_.simulation_mode; }

  /**
   * @brief Get the aircraft mass in kg.
   * @return float 
   */
  inline float getMass() const { return parameters_.mass; }

  /**
   * @brief Get the maximum thrust in N.
   * @return float 
   */
  inline float getMaxThrust() const { return parameters_.max_thrust; }

  //TODO: comment all the getters
  inline bool getArmingState() const { return platform_info_msg_.armed; }
  inline bool getConnectedStatus() const { return platform_info_msg_.connected; }
  inline bool getOffboardMode() const { return platform_info_msg_.offboard; }
  inline bool isControlModeSettled() const
  {
    return (
      platform_info_msg_.current_control_mode.control_mode !=
      platform_info_msg_.current_control_mode.UNSET);
  }

  // ROS publishers & subscribers
private:
  rclcpp::Publisher<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr thrust_command_sub_;

  /**
   * @brief Publishes the platform info message.
   */
  void publishPlatformInfo()
  {
    platform_info_msg_.header.stamp = rclcpp::Clock().now();
    platform_info_msg_.status = state_machine_.getState();
    platform_info_pub_->publish(platform_info_msg_);
  };

  // ROS Services & srv callbacks
private:
  rclcpp::Service<as2_msgs::srv::SetPlatformControlMode>::SharedPtr set_platform_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offboard_mode_srv_;

  /**
   * @brief Set Aircraft Control Mode Service Callback
   * 
   * @param request 
   * @param response 
   */
  void setPlatformControlModeSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Request> request,
    std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Response> response);

  /**
   * @brief Set Aircraft Arming State Service Callback
   * 
   * @param request
   * @param response
   */
  void setArmingStateSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Set Aircraft Offboard Mode Service Callback
   * 
   * @param request 
   * @param response 
   */
  void setOffboardModeSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

};  //class AerialPlatform
};  // namespace as2

#endif  //AEROSTACK2_NODE_HPP_
