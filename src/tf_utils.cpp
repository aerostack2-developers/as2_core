#include "tf_utils.hpp"

std::string generateTfName(std::string _namespace, std::string _frame_name)
{
  std::string tf_name;

  if (_namespace.find("/") == 0) {
    _namespace = _namespace.substr(1);
  }
  if (_frame_name.find("/") == 0) {
    _frame_name = _frame_name.substr(1);
  }

  if (_namespace != "") {
    return _namespace + "/" + _frame_name;
  } else {
    return _frame_name;
  }
}

geometry_msgs::msg::TransformStamped getTransformation(
  const std::string & _frame_id, const std::string & _child_frame_id, double _translation_x,
  double _translation_y, double _translation_z, double _roll, double _pitch, double _yaw)
{
  geometry_msgs::msg::TransformStamped transformation;

  transformation.header.frame_id = _frame_id;
  transformation.child_frame_id = _child_frame_id;
  transformation.transform.translation.x = _translation_x;
  transformation.transform.translation.y = _translation_y;
  transformation.transform.translation.z = _translation_z;
  tf2::Quaternion q;
  q.setRPY(_roll, _pitch, _yaw);
  transformation.transform.rotation.x = q.x();
  transformation.transform.rotation.y = q.y();
  transformation.transform.rotation.z = q.z();
  transformation.transform.rotation.w = q.w();

  return transformation;
}