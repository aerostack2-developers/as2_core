// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_SENSOR_HPP_
#define AEROSTACK2_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "node.hpp"
#include "aerial_platform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"

// TODO ADD CAMERA SUPPORT

namespace aerostack2
{
class GenericSensor
{
public:
  GenericSensor(const std::string & id, aerostack2::Node * node_ptr)
  : sensor_id_(id), node_ptr_(node_ptr)
  {
    topic_name_ = node_ptr_->get_drone_id() + "/" + node_ptr->get_name() + "/" + sensor_id_;
  }

protected:
  std::string topic_name_;
  aerostack2::Node * node_ptr_ = nullptr;

private:
  std::string sensor_id_;

  // TODO: TFs management
  void setStaticTransform(
    const std::string & frame_id, const std::string & parent_frame_id, const float & x,
    const float & y, const float & z, const float & roll, const float & pitch, const float & yaw){};

  // TODO: Implement this
  void registerSensor(){};

};  //class Sensor

template <typename T>
class Sensor : public GenericSensor
{
public:
  Sensor(const std::string & id, aerostack2::Node * node_ptr) : GenericSensor(id, node_ptr)
  {
    sensor_publisher_ = node_ptr_->create_publisher<T>(this->topic_name_, 10);
  }

  void publishData(const T & msg) { this->sensor_publisher_->publish(msg); }

private:
  typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
  T msg_data_;

};  //class Sensor

};      // namespace aerostack2
#endif  //AEROSTACK2_NODE_HPP_
