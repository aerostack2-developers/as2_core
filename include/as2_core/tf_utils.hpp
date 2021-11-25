#ifndef TF_UTILS_HPP_
#define TF_UTILS_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

std::string generateTfName(std::string _namespace, std::string _frame_name);

geometry_msgs::msg::TransformStamped getTransformation(
  const std::string & _frame_id, const std::string & _child_frame_id, double _translation_x,
  double _translation_y, double _translation_z, double _roll, double _pitch, double _yaw);

#endif  //TF_UTILS_HPP_