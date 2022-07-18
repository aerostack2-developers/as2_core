/*!*******************************************************************************************
 *  \file       sensor.hpp
 *  \brief      Sensor class for AS2 header file
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
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
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

// camera
#include <image_transport/image_transport.hpp>

namespace as2
{
  namespace sensors
  {
    class GenericSensor
    {
    public:
      GenericSensor(const std::string &topic_name, as2::Node *node_ptr, int pub_freq = -1) : topic_name_(topic_name), node_ptr_(node_ptr), pub_freq_(pub_freq)
      {
        // topic_name_ = std::string("sensor_measurements/") + sensor_id_;
      }

    protected:
      std::string topic_name_;
      as2::Node *node_ptr_ = nullptr;
      float pub_freq_;
      rclcpp::TimerBase::SharedPtr timer_;

    private:
      // std::string sensor_id_;

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
          timer_ = node_ptr_->create_wall_timer(std::chrono::milliseconds(1000 / pub_freq), [this]()
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
      void publishData()
      {
        this->sensor_publisher_->publish(msg_data_);
      }
      void publishData(const T &msg) { this->sensor_publisher_->publish(msg); }

    private:
      typename rclcpp::Publisher<T>::SharedPtr sensor_publisher_;
      T msg_data_;

    }; // class Sensor

    class Camera : public GenericSensor, std::enable_shared_from_this<rclcpp::Node>
    {
    public:
      Camera(const std::string &id, as2::Node *node_ptr);
      ~Camera();

      void setup();
      void updateData(const sensor_msgs::msg::Image &_img);
      void publishCameraData(const sensor_msgs::msg::Image &msg); // private
      void loadParameters(const std::string &file);
      void setParameters(const sensor_msgs::msg::CameraInfo &_camera_info);

      void publishRectifiedImage(const sensor_msgs::msg::Image &msg);
      // void publishCompressedImage(const sensor_msgs::msg::Image &msg);

    private:
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
      std::shared_ptr<image_transport::ImageTransport> image_transport_ptr_ = nullptr;
      image_transport::Publisher it_publisher_;

      sensor_msgs::msg::Image image_data_;
      sensor_msgs::msg::CameraInfo camera_info_data_;

      std::shared_ptr<rclcpp::Node> getSelfPtr();

      bool setup_ = true;
      bool camera_info_available_ = false;

    }; // class CameraSensor

    using Imu = Sensor<sensor_msgs::msg::Imu>;
    using GPS = Sensor<sensor_msgs::msg::NavSatFix>;
    using Lidar = Sensor<sensor_msgs::msg::LaserScan>;
    using Battery = Sensor<sensor_msgs::msg::BatteryState>;
    using Barometer = Sensor<sensor_msgs::msg::FluidPressure>;
    using Compass = Sensor<sensor_msgs::msg::MagneticField>;
    using RangeFinder = Sensor<sensor_msgs::msg::Range>;

  }; // namespace sensors
};   // namespace as2

#endif // AEROSTACK2_NODE_HPP_
