// "Copyright [year] <Copyright Owner>"

#include "aerial_platform.hpp"

using namespace aerostack2::names;
using namespace aerostack2::names;

namespace aerostack2
{
AerialPlatform::AerialPlatform() : aerostack2::Node(std::string("platform"))
{
  
  this->declare_parameter<bool>("simulation_mode", false);
  this->declare_parameter<float>("mass",1.0);
  this->declare_parameter<float>("max_thrust",0.0);

  this->get_parameter("simulation_mode", parameters_.simulation_mode);
  this->get_parameter("mass", parameters_.mass);
  this->get_parameter("max_thrust", parameters_.max_thrust);

  RCLCPP_INFO(this->get_logger(), "simulation_mode: %d", parameters_.simulation_mode);
  RCLCPP_INFO(this->get_logger(), "mass: %.2f kg", parameters_.mass);
  if (parameters_.max_thrust == 0){
    RCLCPP_WARN(this->get_logger(), "max_thrust is 0 : CODE MAY FAIL IF THRUST IS NORMALIZED");
  }else{
    RCLCPP_INFO(this->get_logger(), "max_thrust: %.2f N", parameters_.max_thrust);
  }
  
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

  thrust_command_sub_ =
    this->create_subscription<global_topics::actuator_commands::THRUST_COMMAND_TYPE>(
      this->generate_topic_name(global_topics::actuator_commands::THRUST_COMMAND), 10,
      [this](const global_topics::actuator_commands::THRUST_COMMAND_TYPE::ConstSharedPtr msg) {
        this->command_thrust_msg_ = *msg.get();
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

  // FIXME: Frecuency is hardcoded!!
  platform_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&AerialPlatform::setPlatformStatus, this));

  // // TODO: remove timer_test
  //  static auto timer_test = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() {
  //    std::cout << "simulation mode = " << this->simulation_mode_enabled_ << std::endl;
  //  });
  // FIXME: rethink this function
  //  static auto timer_commands_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
  //    if (this->sending_commands_){
  //      this->sendCommand();
  //    }
  //  });
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
  platform_status_ = *(ownSetPlatformStatus().get());
  publishPlatformStatus();
  return platform_status_;
};

bool AerialPlatform::setPlatformControlMode(const aerostack2_msgs::msg::PlatformControlMode & msg)
{
  control_mode_settled_ = ownSetPlatformControlMode(msg);
  return control_mode_settled_;
};
bool AerialPlatform::sendCommand()
{
  if (!control_mode_settled_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "ERROR: Platform control mode is not settled yet");
    return false;
  } else {
    return ownSendCommand();
  }
};

//Services Callbacks

void AerialPlatform::setPlatformControlModeSrvCall(
  const std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Request> request,
  std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Response> response)
{
  bool success = this->setPlatformControlMode(request->control_mode);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: UNABLE TO SET THIS CONTROL MODE TO THIS PLATFORM");
  }
  sending_commands_ = success;
  response->success = success;
};

void AerialPlatform::setOffboardModeSrvCall(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  bool success = setOffboardControl(request->data);
  // FIXME: Adapt this for px4
  control_mode_settled_ = success;
  sending_commands_ = success;
  response->success = success;
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