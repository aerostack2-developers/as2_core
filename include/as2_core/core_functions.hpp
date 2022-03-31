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

void spinLoop(std::shared_ptr<as2::Node> node, std::function<void()> run_function = nullptr);

};  // namespace as2

#endif  // __AEROSTACK2_CORE_FUNCTIONS_HPP__
