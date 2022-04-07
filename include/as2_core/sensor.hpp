// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_SENSOR_HPP_
#define AEROSTACK2_SENSOR_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/range.hpp"

// TODO ADD CAMERA SUPPORT

namespace as2
{
  namespace sensors
  {
    class GenericSensor
    {
    public:
      GenericSensor(const std::string &id, as2::Node *node_ptr, int pub_freq = -1) : sensor_id_(id), node_ptr_(node_ptr), pub_freq_(pub_freq)
      {
        topic_name_ = std::string("sensor_measurements/") + sensor_id_;
      }

    protected:
      std::string topic_name_;
      as2::Node *node_ptr_ = nullptr;
      float pub_freq_;
      rclcpp::TimerBase::SharedPtr timer_;

    private:
      std::string sensor_id_;

      // TODO: TFs management
      void setStaticTransform(
          const std::string &frame_id, const std::string &parent_frame_id, const float &x,
          const float &y, const float &z, const float &roll, const float &pitch, const float &yaw){};

      // TODO: Implement this
      void registerSensor(){};

    }; // class GenericSensor

    template <typename T>
    class Sensor : public GenericSensor
    {
    public:
      Sensor(const std::string &id, as2::Node *node_ptr, int pub_freq = -1) : GenericSensor(id, node_ptr, pub_freq)
      {
        sensor_publisher_ = node_ptr_->create_publisher<T>(this->topic_name_, as2_names::topics::sensor_measurements::qos);

        if (this->pub_freq_ != -1)
        {
          timer_ = node_ptr_->create_wall_timer(std::chrono::milliseconds(1000/pub_freq), [this]()
                                                { publishData(); });
        }
      }

      void updateData(const T &msg)
      {
        if (this->pub_freq_ != -1)
        {
          this->msg_data_ = msg;
        }
        else
        {
          this->publishData(msg);
        }
      }

    private:
      void publishData() { 
        this->sensor_publisher_->publish(msg_data_);
      }
      void publishData(const T &msg) { this->sensor_publisher_->publish(msg); }

    private:
      typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
      T msg_data_;

    }; // class Sensor

    class Camera : public GenericSensor
    {
      Camera(const std::string &id, as2::Node *node_ptr) : GenericSensor(id, node_ptr)
      {
        image_publisher_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(this->topic_name_, 10);
        camera_info_publisher_ =
            node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(this->topic_name_ + "/info", 10);
      }

      // TODO
      void publishData(const sensor_msgs::msg::Image &msg)
      {
        this->image_publisher_->publish(msg);
        this->camera_info_publisher_->publish(this->camera_info_data_);
      }

      void publishRectifiedImage(const sensor_msgs::msg::Image &msg){};
      void publishCompressedImage(const sensor_msgs::msg::Image &msg){};

    private:
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;

      sensor_msgs::msg::Image image_data_;
      sensor_msgs::msg::CameraInfo camera_info_data_;

    }; // class CameraSensor

    using Imu = Sensor<sensor_msgs::msg::Imu>;
    using GPS = Sensor<sensor_msgs::msg::NavSatFix>;
    using Lidar = Sensor<sensor_msgs::msg::LaserScan>;
    using Battery = Sensor<sensor_msgs::msg::BatteryState>;
    using Barometer = Sensor<sensor_msgs::msg::FluidPressure>;
    using Magnometer = Sensor<sensor_msgs::msg::MagneticField>;
    using UltrasonicAltimeter = Sensor<sensor_msgs::msg::Range>;

  }; // namespace sensors
};   // namespace as2

#endif // AEROSTACK2_NODE_HPP_
