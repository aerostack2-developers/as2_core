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
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include "as2_core/rate.hpp"
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
  Node(const std::string & name) : rclcpp::Node(name)
  {
    this->declare_parameter<float>("node_frequency", -1.0);
    this->get_parameter("node_frequency", loop_frequency_);
    RCLCPP_INFO(this->get_logger(), "node [%s] base frequency= %f", loop_frequency_);

    if (loop_frequency_ > 0.0) {
      loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
    }
  };

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

private:
  /**
   * @brief frequency of the spin cycle of the node
   */
  double loop_frequency_;
  std::shared_ptr<as2::Rate> loop_rate_ptr_;

public:
  /**
   * @brief sleeps the node to ensure node_frecuency desired 
   * 
   * @return true the node is sleeping
   * @return false the node is not sleeping, this means that desired frequency is not reached
   */
  bool sleep()
  {
    if (loop_rate_ptr_) {
      return loop_rate_ptr_->sleep();
    } else {
      throw std::runtime_error("Node::sleep() called but no node_frequency defined");
      return false;
    };
  };
  /**
   * @brief Get the loop frequency object 
   * 
   * @return double frequency in Hz
   */
  inline double get_loop_frequency() { return loop_frequency_; }

  bool preset_loop_frequency(double frequency)
  {
    if (frequency <= 0) return true;  // default frequency is -1
    if (loop_rate_ptr_) {
      RCLCPP_INFO(
        this->get_logger(), "Preset Loop Frequency [%d Hz] was overwrite to launcher params to %d",
        (int)frequency, (int)loop_frequency_);
      return false;
    }
    loop_frequency_ = frequency;
    loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
    return true;
  };
};

}  // namespace as2

#endif  // AEROSTACK2_NODE_HPP_
