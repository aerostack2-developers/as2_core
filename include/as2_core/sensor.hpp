// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_SENSOR_HPP_
#define AEROSTACK2_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <exception>

#include "aerial_platform.hpp"
#include "node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "GpsUtils.hpp"

// TODO ADD CAMERA SUPPORT

namespace as2
{
namespace sensors
{
class GenericSensor
{
public:
  GenericSensor(const std::string & id, as2::Node * node_ptr) : sensor_id_(id), node_ptr_(node_ptr)
  {
    topic_name_ = std::string(node_ptr->get_name()) + "/" + sensor_id_;
  }

protected:
  std::string topic_name_;
  as2::Node * node_ptr_ = nullptr;

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
  Sensor(const std::string & id, as2::Node * node_ptr) : GenericSensor(id, node_ptr)
  {
    sensor_publisher_ = node_ptr_->create_publisher<T>(this->topic_name_, 10);
  }

  void publishData(const T & msg) { this->sensor_publisher_->publish(msg); }

private:
  typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
  T msg_data_;

};  //class Sensor

class Camera : public GenericSensor
{
  Camera(const std::string & id, as2::Node * node_ptr) : GenericSensor(id, node_ptr)
  {
    image_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(this->topic_name_, 10);
    camera_info_publisher_ =
      node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(this->topic_name_ + "/info", 10);
  }

  //TODO
  void publishData(const sensor_msgs::msg::Image & msg)
  {
    this->image_publisher_->publish(msg);
    this->camera_info_publisher_->publish(this->camera_info_data_);
  }

  void publishRectifiedImage(const sensor_msgs::msg::Image & msg){};
  void publishCompressedImage(const sensor_msgs::msg::Image & msg){};

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;

  sensor_msgs::msg::Image image_data_;
  sensor_msgs::msg::CameraInfo camera_info_data_;

};  //class CameraSensor

class GPS: public GenericSensor {
private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  GpsUtils utils_;

public:
  GPS(const std::string &id, as2::Node *node_ptr): GenericSensor(id, node_ptr) {
    fix_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::NavSatFix>(this->topic_name_, 10);

    double lat0, lon0, h0;  // origin
    bool is_lat = node_ptr_->get_parameter_or(this->topic_name_ + "/origin/lat", lat0, 0.0);
    bool is_lon = node_ptr_->get_parameter_or(this->topic_name_ + "/origin/lon", lon0, 0.0);
    bool is_alt = node_ptr_->get_parameter_or(this->topic_name_ + "/origin/alt", h0, 0.0);
    if (is_lat && is_lon) {
      this->setOrigin(lat0, lon0, h0);
    } else {
      node_ptr_->set_on_parameters_set_callback(std::bind(&GPS::paramsCb, this, std::placeholders::_1));
    } // ROS2 Galactic --> https://docs.ros.org/en/galactic/Tutorials/Monitoring-For-Parameter-Changes-CPP.html
  }

  void publishData(const sensor_msgs::msg::NavSatFix &msg){
    this->fix_publisher_->publish(msg);
  }

private:
    rcl_interfaces::msg::SetParametersResult paramsCb(const std::vector<rclcpp::Parameter> & parameters) { 
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      auto is_lat = [](rclcpp::Parameter param) { return param.get_name().find("/origin/lat") != std::string::npos; };
      auto is_lon = [](rclcpp::Parameter param) { return param.get_name().find("/origin/lon") != std::string::npos; };
      auto is_h = [](rclcpp::Parameter param) { return param.get_name().find("/origin/alt") != std::string::npos; };
      auto lat_ptr = std::find_if(begin(parameters), end(parameters), is_lat);
      auto lon_ptr = std::find_if(begin(parameters), end(parameters), is_lon);
      auto h_ptr = std::find_if(begin(parameters), end(parameters), is_h);

      if (lat_ptr != parameters.end() && lon_ptr != parameters.end()) {
        double h;
        (h_ptr != parameters.end()) ? h = h_ptr->get_value<double>() : h = 0.0;
          try {
            setOrigin(lat_ptr->get_value<double>(), lon_ptr->get_value<double>(), h);
          } catch (std::exception &e) {
            RCLCPP_WARN(node_ptr_->get_logger(), e.what());
          }
      }
      return result;
    }

  void setOrigin(double lat, double lon, double h) {
    this->utils_.SetOrigin(lat, lon, h);
    double x, y, z;
    GpsUtils::LatLon2Ecef(lat, lon, h, x, y, z);
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter(this->topic_name_ + "/origin/ecef/x", x), 
                                             rclcpp::Parameter(this->topic_name_ + "/origin/ecef/y", y), 
                                             rclcpp::Parameter(this->topic_name_ + "/origin/ecef/z", z)};
    node_ptr_->set_parameters(params);
  }

};  // class GPS

using Imu = Sensor<sensor_msgs::msg::Imu>;
using Lidar = Sensor<sensor_msgs::msg::LaserScan>;

};  // namespace sensors
};  // namespace as2

#endif  //AEROSTACK2_NODE_HPP_
