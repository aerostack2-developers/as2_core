// "Copyright [year] <Copyright Owner>"

#include "aerial_platform.hpp"

using namespace aerostack2::names;
using namespace aerostack2::names;

namespace aerostack2
{
AerialPlatform::AerialPlatform() : aerostack2::Node(std::string("platform"))
{
  pose_command_sub_ =
    this->create_subscription<global_topics::actuator_commands::POSE_COMMAND_TYPE>(
      this->generate_topic_name(global_topics::actuator_commands::POSE_COMMAND), 10,
      [this](const global_topics::actuator_commands::POSE_COMMAND_TYPE::ConstSharedPtr msg) {
        this->command_pose_msg_ = *msg.get();
      });

  twist_command_sub_ =
    this->create_subscription<global_topics::actuator_commands::TWIST_COMMAND_TYPE>(
      this->generate_topic_name(global_topics::actuator_commands::TWIST_COMMAND), 10,
      [this](const global_topics::actuator_commands::TWIST_COMMAND_TYPE::ConstSharedPtr msg) {
        this->command_twist_msg_ = *msg.get();
      });

  set_platform_mode_srv_ = this->create_service<aerostack2_msgs::srv::SetPlatformControlMode>(
    this->generate_srv_name("/set_platform_control_mode"),
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

  platform_status_pub_ = this->create_publisher<global_topics::platform::PLATFORM_STATUS_TYPE>(
    this->generate_srv_name(global_topics::platform::PLATFORM_STATUS), 10);

  platform_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&AerialPlatform::setPlatformStatus, this));
};

bool AerialPlatform::setArmingState(bool state)
{
  if (state == platform_status_.armed && state == true) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already armed");
  } else if (state == platform_status_.armed && state == false) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already disarmed");
  } else {
    return ownSetArmingState(state);
  }
  return false;
};

bool AerialPlatform::setOffboardControl(bool offboard)
{
  if (offboard == platform_status_.offboard && offboard == true) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already in OFFBOARD mode");
  } else if (offboard == platform_status_.offboard && offboard == false) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already in MANUAL mode");
  } else {
    return ownSetOffboardControl(offboard);
  }
  return false;
};

aerostack2_msgs::msg::PlatformStatus AerialPlatform::setPlatformStatus()
{
  platform_status_ = ownSetPlatformStatus();
  publishPlatformStatus();
  return platform_status_;
};

bool AerialPlatform::setPlatformControlMode(const aerostack2_msgs::msg::PlatformControlMode & msg)
{
  control_mode_settled_ = ownSetPlatformControlMode(msg);
  return control_mode_settled_;
};
bool AerialPlatform::sendCommand(const aerostack2_msgs::msg::PlatformControlMode & msg)
{
  if (!control_mode_settled_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "ERROR: Sensor Platform control mode is not settled yet");
    return false;
  } else {
    return ownSendCommand(msg);
  }
};

//Services Callbacks

void AerialPlatform::setPlatformControlModeSrvCall(
  const std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Request> request,
  std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Response> response)
{
  bool sucess = this->setPlatformControlMode(request->control_mode);
  if (!sucess) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "ERROR: UNABLE TO SET THIS CONTROL MODE TO THIS PLATFORM");
  }
  response->success = sucess;
};

void AerialPlatform::setOffboardModeSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setOffboardControl(request->data);
}

void AerialPlatform::setArmingStateSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = setArmingState(request->data);
}

// Publish Functions
void AerialPlatform::publishPlatformStatus() { platform_status_pub_->publish(platform_status_); }

};  // namespace aerostack2