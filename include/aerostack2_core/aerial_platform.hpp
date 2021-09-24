// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_AERIAL_PLATFORM_HPP_
#define AEROSTACK2_AERIAL_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"

#include "aerostack2_msgs/msg/platform_control_mode.hpp"
#include "aerostack2_msgs/msg/platform_status.hpp"
#include "aerostack2_msgs/srv/set_platform_control_mode.hpp"
#include "aerostack2_core/naming.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_srvs/srv/set_bool.hpp"

namespace aerostack2
{
class AerialPlatform : public aerostack2::Node
{
private:
  rclcpp::Publisher<aerostack2_msgs::msg::PlatformStatus>::SharedPtr platform_status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
  rclcpp::Subscription<aerostack2_msgs::msg::Thrust>::SharedPtr thrust_command_sub_;
  
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
  

  rclcpp::Service<aerostack2_msgs::srv::SetPlatformControlMode>::SharedPtr set_platform_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offboard_mode_srv_;

  bool control_mode_settled_ = false;
  aerostack2_msgs::msg::PlatformStatus platform_status_;

protected:
  aerostack2::names::global_topics::actuator_commands::POSE_COMMAND_TYPE command_pose_msg_;
  aerostack2::names::global_topics::actuator_commands::TWIST_COMMAND_TYPE command_twist_msg_;
  aerostack2::names::global_topics::actuator_commands::THRUST_COMMAND_TYPE command_thrust_msg_;

public:
  AerialPlatform();

  virtual void configureSensors() = 0;
  virtual void publishSensorData() = 0;

  virtual bool ownSetArmingState(bool state) = 0;
  virtual bool ownSetOffboardControl(bool offboard) = 0;
  virtual std::shared_ptr<aerostack2_msgs::msg::PlatformStatus> ownSetPlatformStatus() = 0;

  virtual bool ownSetPlatformControlMode(const aerostack2_msgs::msg::PlatformControlMode & msg) = 0;
  virtual bool ownSendCommand() = 0;

  bool getFlagSimulationMode(){return simulation_mode_enabled_;}

private:
  bool simulation_mode_enabled_ = false;
  rclcpp::TimerBase::SharedPtr platform_state_timer_;

  bool setArmingState(bool state);
  bool setOffboardControl(bool offboard);
  aerostack2_msgs::msg::PlatformStatus setPlatformStatus();

  bool setPlatformControlMode(const aerostack2_msgs::msg::PlatformControlMode & msg);
  bool sendCommand();

  // Service callbacks

  void setPlatformControlModeSrvCall(
    const std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Request> request,
    std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Response> response);

  void setArmingStateSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void setOffboardModeSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Publish functions
  void publishPlatformStatus();

};  //class AerialPlatform

};      // namespace aerostack2
#endif  //AEROSTACK2_NODE_HPP_
