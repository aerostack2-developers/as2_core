// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_AERIAL_PLATFORM_HPP_
#define AEROSTACK2_AERIAL_PLATFORM_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "as2_core/naming.hpp"
#include "as2_msgs/msg/platform_control_mode.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_msgs/srv/set_platform_control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace as2
{
struct AerialPlatformParameters
{
  float mass;
  float max_thrust;
  bool simulation_mode;
};

class AerialPlatform : public as2::Node
{
private:
  rclcpp::Publisher<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
  rclcpp::Subscription<as2_msgs::msg::Thrust>::SharedPtr thrust_command_sub_;

  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;

  rclcpp::Service<as2_msgs::srv::SetPlatformControlMode>::SharedPtr set_platform_mode_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_offboard_mode_srv_;

  bool control_mode_settled_ = false;
  as2_msgs::msg::PlatformInfo platform_info_;

protected:
  as2::names::global_topics::actuator_commands::POSE_COMMAND_TYPE command_pose_msg_;
  as2::names::global_topics::actuator_commands::TWIST_COMMAND_TYPE command_twist_msg_;
  as2::names::global_topics::actuator_commands::THRUST_COMMAND_TYPE command_thrust_msg_;

public:
  AerialPlatform();

  virtual void configureSensors() = 0;
  virtual void publishSensorData() = 0;

  virtual bool ownSetArmingState(bool state) = 0;
  virtual bool ownSetOffboardControl(bool offboard) = 0;
  virtual std::shared_ptr<as2_msgs::msg::PlatformInfo> ownSetPlatformInfo() = 0;

  virtual bool ownSetPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg) = 0;
  virtual bool ownSendCommand() = 0;

  inline bool getFlagSimulationMode() const { return parameters_.simulation_mode; }
  inline float getMass() const { return parameters_.mass; }
  inline float getMaxThrust() const { return parameters_.max_thrust; }

private:
  as2::AerialPlatformParameters parameters_;

  bool sending_commands_ = false;
  rclcpp::TimerBase::SharedPtr platform_state_timer_;

  bool setArmingState(bool state);
  bool setOffboardControl(bool offboard);
  as2_msgs::msg::PlatformInfo setPlatformInfo();

  bool setPlatformControlMode(const as2_msgs::msg::PlatformControlMode & msg);
  bool sendCommand();

  // Service callbacks

  void setPlatformControlModeSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Request> request,
    std::shared_ptr<as2_msgs::srv::SetPlatformControlMode::Response> response);

  void setArmingStateSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void setOffboardModeSrvCall(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Publish functions
  void publishPlatformInfo();

};  //class AerialPlatform

};      // namespace as2
#endif  //AEROSTACK2_NODE_HPP_
