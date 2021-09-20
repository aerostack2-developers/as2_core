// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_SENSOR_HPP_
#define AEROSTACK2_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"

// TODO ADD CAMERA SUPPORT

namespace aerostack2
{
template <class T> class Sensor
{
public:
  Sensor(const std::string& id, const std::shared_ptr<aerostack2::Node>& node_ptr)
    :sensor_id_(id), node_ptr_(node_ptr)
    {
      std::string topic_name = node_ptr_->get_drone_id() + "/" + node_ptr->get_name() + "/" + sensor_id_;
      sensor_publisher_ = node_ptr_->create_publisher<T>(topic_name, 10);
    }
  
  // TODO TFs management
  void setStaticTransform(const std::string& frame_id,
                          const std::string& parent_frame_id,
                          const float& x,
                          const float& y,
                          const float& z,
                          const float& roll,
                          const float& pitch,
                          const float& yaw){};
  
  void registerSensor(){};

  bool new_measure_ = false;
  void addMeasure(const T& measurement){
    this->msg_data_ = measurement;
    new_measure_ = true;
  };

  void publishData(){
    if (new_measure_){
      this->sensor_publisher_->publish(msg_data_);
      new_measure_ = false;
    }
  }


private:
  std::shared_ptr<aerostack2::Node> node_ptr_;
  
  typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
  
  std::string sensor_id_;  
  T msg_data_;

}; //class Sensor

};      // namespace aerostack2
#endif  //AEROSTACK2_NODE_HPP_

