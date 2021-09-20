// "Copyright [year] <Copyright Owner>"

#ifndef AEROSTACK2_NODE_HPP_
#define AEROSTACK2_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"

namespace aerostack2
{
class Node : public rclcpp::Node
{
public:
  Node(const std::string& name) : rclcpp::Node(name){};
  std::string get_drone_id(){return "drone0";};
};
}  // namespace aerostack2

#endif  //AEROSTACK2_NODE_HPP_
