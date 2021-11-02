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


#include "naming.hpp"

namespace aerostack2
{
/**
 * @brief Basic Aerostack2 Node, it heritages all the functionality of an rclcpp::Node
 * 
 */
class Node : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Node object
   * 
   * @param name Node name
   */
  Node(const std::string & name) : rclcpp::Node(name){};
  std::string get_drone_id() { return "";};

  /**
   * @brief 
   * 
   * @param name 
   * @return std::string 
   */

  std::string generate_local_topic_name(const std::string & name)
  {
    if (name.find("/") == 0) {
      return this->get_name() + name;
    } else {
      return std::string(this->get_name()) + "/" + name;
    }
  };

  /**
   * @brief 
   * 
   * @param name 
   * @return std::string 
   */
  std::string generate_topic_name(const std::string & name);

  /**
   * @brief 
   * 
   * @param name 
   * @return std::string 
   */
  std::string generate_srv_name(const std::string & name) { return generate_topic_name(name); }

  /** 
   * @brief convert srv_name in the local scope of the node
   * 
   * @param name 
   * @return std::string 
   */
  std::string generate_local_srv_name(const std::string & name)
  {
    return generate_local_topic_name(name);
  }
};
}  // namespace aerostack2

#endif  //AEROSTACK2_NODE_HPP_
