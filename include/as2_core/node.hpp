/*!*******************************************************************************************
 *  \file       node.hpp
 *  \brief      Aerostack2 node header file.
 *  \authors    Miguel Fernandez Cortizas
 *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
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

#ifndef AEROSTACK2_NODE_HPP_
#define AEROSTACK2_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2
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

  /**
   * @brief transform an string into local topic name inside drone namespace and node namespace
   * 
   * @param name source string
   * @return std::string  result name
   */
  std::string generate_local_name(const std::string & name);

  /**
   * @brief transform an string into global topic name inside drone namespace
   * 
   * @param name source string
   * @return std::string result name
   */
  std::string generate_global_name(const std::string & name);

  /**
   * @brief Executes the main loop of the node
   * 
   * @param freq expecify the frequency of the main loop, if -1 the main loop will run as fast as possible
   * @param run_function function to be executed in the main loop
   */
  void spinLoop(float freq = -1, std::function<void()> run_function = nullptr)
  {
    if (freq <= 0) {
      rclcpp::spin(this->get_node_base_interface());
      return;
    }

    rclcpp::Rate loop_rate(freq);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (run_function != nullptr) run_function();
      if (!loop_rate.sleep()) {
        // if sleep returns false, it means that loop rate cannot keep up with the desired rate
        RCLCPP_INFO(
          this->get_logger(), "Spin loop rate exceeded, stable frequency cannot be assured ");
      }
    }
  }
};

}  // namespace as2

#endif  //AEROSTACK2_NODE_HPP_
