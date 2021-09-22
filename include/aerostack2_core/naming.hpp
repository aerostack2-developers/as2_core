#ifndef AEROSTACK_NAMING_HPP
#define AEROSTACK_NAMING_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <aerostack2_msgs/msg/platform_status.hpp>

namespace aerostack2
{
namespace names
{
namespace global_topics
{
namespace actuator_commands
{
const std::string POSE_COMMAND = "actuator_command/pose";
using POSE_COMMAND_TYPE = geometry_msgs::msg::PoseStamped;

const std::string TWIST_COMMAND = "actuator_command/twist";
using TWIST_COMMAND_TYPE = geometry_msgs::msg::TwistStamped;

}  // namespace actuator_commands

namespace platform
{
const std::string PLATFORM_STATUS = "platform/platform_status";
using PLATFORM_STATUS_TYPE = aerostack2_msgs::msg::PlatformStatus;

}  // namespace platform

}  // namespace global_topics

namespace local_topics
{
const std::string IMU_DATA = "imu";
using IMU_DATA_TYPE = sensor_msgs::msg::Imu;
}  // namespace local_topics

}  // namespace names
}  // namespace aerostack2
#endif  // AEROSTACK_NAMING_HPP