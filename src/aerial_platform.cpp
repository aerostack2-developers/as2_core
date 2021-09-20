// "Copyright [year] <Copyright Owner>"

#include "aerial_platform.hpp"

namespace aerostack2{

AerialPlatform::AerialPlatform(const std::string& name)
    : aerostack2::Node(name)
  {
    
    set_platform_mode_srv_ = this->create_service<aerostack2_msgs::srv::SetPlatformControlMode>("set_platform_control_mode", 
                std::bind(&AerialPlatform::setPlatformControlModeSrvCall,this, 
                std::placeholders::_1, // Corresponds to the 'request'  input
                std::placeholders::_2  // Corresponds to the 'response' input
                ));

    std::string topic_name = this->get_drone_id() + "/" + this->get_name() + "/platform_status";
    platform_status_pub_ = this->create_publisher<aerostack2_msgs::msg::PlatformStatus>(topic_name, 10);
  };

void AerialPlatform::setPlatformControlModeSrvCall( const std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Request> request,
                                                          std::shared_ptr<aerostack2_msgs::srv::SetPlatformControlMode::Response> response)
{
  bool sucess = this->setPlatformControlMode(request->control_mode);
  if (!sucess){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: UNABLE TO SET THIS CONTROL MODE TO THIS PLATFORM");
  }
  response->success = sucess;
}; 


  bool AerialPlatform::configureSensors(){
    sensors_configured_ = ownConfigureSensors();
    return sensors_configured_;
  };
  
  bool AerialPlatform::readSensors(){
    if (!sensors_configured_){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Sensor are not configured yet");
      return false;
    }
    else{
      return ownReadSensors();
    }
  };

  bool AerialPlatform::publishSensorData(){
    publishPlatformStatus();
    return ownPublishSensorData();
  };

  void AerialPlatform::publishPlatformStatus(){
    platform_status_pub_->publish(platform_status_);
  };


  bool AerialPlatform::setArmingState(bool state){
    if (state == platform_status_.armed && state == true){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already armed");
    }
    else if (state == platform_status_.armed && state == false){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already disarmed");
    }
    else{
      return ownSetArmingState(state);
    }
    return false;
  };

  bool AerialPlatform::setOffboardControl(bool offboard){
    if (offboard == platform_status_.offboard && offboard == true){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already in OFFBOARD mode");
    }
    else if (offboard == platform_status_.offboard && offboard == false){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UAV is already in MANUAL mode");
    }
    else{
      return ownSetOffboardControl(offboard);
    }
    return false;
  };

  aerostack2_msgs::msg::PlatformStatus AerialPlatform::setPlatformStatus(){
    platform_status_ = ownSetPlatformStatus();
    return platform_status_;
  };
  
  bool AerialPlatform::setPlatformControlMode(const aerostack2_msgs::msg::PlatformControlMode & msg){
    control_mode_settled_ = ownSetPlatformControlMode(msg);
    return control_mode_settled_;
  };
  bool AerialPlatform::sendCommand(const aerostack2_msgs::msg::PlatformControlMode & msg){
     if (!control_mode_settled_){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Sensor Platform control mode is not settled yet");
      return false;
    }
    else{
      return ownSendCommand(msg);
    }
  };



};  //end namespace