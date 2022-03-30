#ifndef __AEROSTACK2_CORE_FUNCTIONS_HPP__
#define __AEROSTACK2_CORE_FUNCTIONS_HPP__

#include "as2_core/node.hpp"
#include "as2_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace as2
{
/**
 * @brief Executes the main loop of the node
 * 
 * @param node node to execute the main loop
 * @param run_function function to be executed in the main loop. Node frequency must be higher than 0
 */

void spinLoop(std::shared_ptr<as2::Node> node, std::function<void()> run_function = nullptr)
{
  if (node->get_loop_frequency() <= 0) {
    rclcpp::spin(node->get_node_base_interface());
    return;
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    if (run_function != nullptr) run_function();
    if (!node->sleep()) {
      // if sleep returns false, it means that loop rate cannot keep up with the desired rate
      // RCLCPP_INFO(
      //   node->get_logger(),
      //   "Spin loop rate exceeded, stable frequency [%.3f Hz] cannot be assured ",
      //   node->get_loop_frequency());
    }
  }
}

};  // namespace as2

#endif  // __AEROSTACK2_CORE_FUNCTIONS_HPP__
