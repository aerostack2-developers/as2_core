#ifndef __AS2_NAMES_TOPICS_HPP__
#define __AS2_NAMES_TOPICS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names {
  namespace topics {
    namespace sensor_measurements {
      const rclcpp::QoS qos = rclcpp::SensorDataQoS();
      const std::string imu = "sensor_measurements/imu";
      const std::string lidar = "sensor_measurements/lidar";
      const std::string gps = "sensor_measurements/gps";
      }
    namespace self_localization {
      const rclcpp::QoS qos = rclcpp::SensorDataQoS();
      const std::string odom = "self_localization/odom";
    }
    namespace motion_reference {
      const rclcpp::QoS qos = rclcpp::SensorDataQoS();
      const std::string pose = "motion_reference/pose";
      const std::string twist = "motion_reference/twist";
      const std::string trajectory = "motion_reference/trajectory";
      // const rclcpp::QoS qos_wp = rclcpp::ServicesQoS();
      // const std::string wayp = "motion_reference/waypoints";
      const rclcpp::QoS qos_waypoint = rclcpp::QoS(10);
      const std::string modify_waypoint = "motion_reference/modify_waypoint";
    }
    namespace actuator_command {
      const rclcpp::QoS qos = rclcpp::SensorDataQoS();
      const std::string pose = "actuator_command/pose";
      const std::string twist = "actuator_command/twist";
      const std::string thrust = "actuator_command/thrust";
    }
    namespace platform{
      const rclcpp::QoS qos = rclcpp::QoS(10);
      const std::string info = "platform/info";
    }
    namespace controller{
      const rclcpp::QoS qos_info = rclcpp::QoS(10);
      const std::string info = "controller/info";
    }
  }
}
#endif // __AS2_NAMES_TOPICS_HPP__
