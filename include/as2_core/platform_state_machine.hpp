/*!*******************************************************************************************
 *  \file       platform_state_machine.hpp
 *  \brief      Aerostack2 Platform State Machine Header file.
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

#ifndef AEROSTACK2_PLATFORM_STATE_MACHINE_HPP_
#define AEROSTACK2_PLATFORM_STATE_MACHINE_HPP_

#include <string>
#include <vector>

#include "as2_core/node.hpp"
#include "as2_msgs/msg/platform_status.hpp"

namespace as2
{
/**
 * @brief Data Structure for defining the state machine transitions.
 */

struct StateMachineTransition
{
  std::string transition_name;
  int8_t from_state_id;
  int8_t transition_id;
  int8_t to_state_id;
};

/**
 * @brief This class implements the Platform State Machine,
 * which is in charge of handling the state of the platform using a FSM (Finite State Machine).
 * This state machine consist on 6 states:
 *   - DISARMED -> The platform is not armed.
 *   - LANDED -> The platform is armed and landed.
 *   - TAKING_OFF -> The platform is taking off.
 *   - FLYING -> The platform is on air.
 *   - LANDING -> The platform is landing.
 *   - EMERGENCY -> The platform is in emergency mode.
 * 
 * The events that can trigger the state machine are:
 *   - ARM 
 *   - DISARM
 *   - TAKE_OFF
 *   - TOOK_OFF
 *   - LAND
 *   - LANDED
 *   - EMERGENCY
 *  TODO: add figure of the state machine
 */

class PlatformStateMachine
{
public:
  enum Event {
    ARM,
    DISARM,
    TAKE_OFF,
    TOOK_OFF,
    LAND,
    LANDED,
    EMERGENCY,
  };

public:
  /**
   * @brief Constructor of the Platform State Machine.
   * @param node_ptr Pointer to an aerostack2 node.
   */
  PlatformStateMachine()
  {
    state_.state = as2_msgs::msg::PlatformStatus::DISARMED;
    defineTransitions();
  };
  // PlatformStateMachine(as2::Node* node){
  //   state_.state = as2_msgs::msg::PlatformStatus::DISARMED;
  // };
  ~PlatformStateMachine(){};

  /**
   * @brief This function is in charge of handling the state machine.
   * @param event The event that triggers the state machine.
   */
  void processEvent(const Event & event)
  {
    // Get the current state
    int8_t current_state = state_.state;

    // Get the transition that matches the current state and the event
    StateMachineTransition transition = getTransition(current_state, event);

    // If the transition is valid, change the state
    if (transition.transition_id != -1) {
      state_.state = transition.to_state_id;
      RCLCPP_INFO(rclcpp::get_logger("FSM transition"), transition.transition_name);
    }
  };

  /**
   * @brief Get the Transition object
   * 
   * @param current_state 
   * @param event 
   * @return StateMachineTransition 
   */
  StateMachineTransition getTransition(int8_t current_state, const Event & event)
  {
    StateMachineTransition transition;
    transition.transition_id = -1;
    for (int i = 0; i < transitions_.size(); i++) {
      if (
        transitions_[i].from_state_id == current_state && transitions_[i].transition_id == event) {
        transition = transitions_[i];
        break;
      }
    }
    return transition;
  };

  /**
   * @brief This function returns the current state of the state machine
   * @return The current Platform Status of the state machine
   */
  as2_msgs::msg::PlatformStatus getState() { return state_; };
  void setState(as2_msgs::msg::PlatformStatus state) { state_ = state; };
  void setState(const int8_t & state) { state_.state = state; };

private:
  std::vector<StateMachineTransition> transitions_;
  as2_msgs::msg::PlatformStatus state_;

  /**
   * @brief in this function the state machine is created based on the transitions.
   * its called in the constructor of the class.
   */
  void defineTransitions()
  {
    transitions_.clear();
    transitions_.reserve(11);

    // INTIAL_STATE -> [TRANSITION] -> FINAL_STATE

    // DISARMED -> [ARM] -> ARMED
    transitions_.emplace_back(StateMachineTransition{
      "ARM", as2_msgs::msg::PlatformStatus::DISARMED, Event::ARM,
      as2_msgs::msg::PlatformStatus::LANDED});

    // LANDED -> [DISARM] -> DISARMED
    transitions_.emplace_back(StateMachineTransition{
      "DISARM", as2_msgs::msg::PlatformStatus::LANDED, Event::DISARM,
      as2_msgs::msg::PlatformStatus::DISARMED});

    // LANDED -> [TAKE_OFF] -> TAKING_OFF
    transitions_.emplace_back(StateMachineTransition{
      "TAKE_OFF", as2_msgs::msg::PlatformStatus::LANDED, Event::TAKE_OFF,
      as2_msgs::msg::PlatformStatus::TAKING_OFF});

    // TAKING_OFF -> [TOOK_OFF] -> FLYING
    transitions_.emplace_back(StateMachineTransition{
      "TOOK_OFF", as2_msgs::msg::PlatformStatus::TAKING_OFF, Event::TOOK_OFF,
      as2_msgs::msg::PlatformStatus::FLYING});

    // FLYING -> [LAND] -> LANDING
    transitions_.emplace_back(StateMachineTransition{
      "LAND", as2_msgs::msg::PlatformStatus::FLYING, Event::LAND,
      as2_msgs::msg::PlatformStatus::LANDING});

    // LANDING -> [LANDED] -> LANDED
    transitions_.emplace_back(StateMachineTransition{
      "LANDED", as2_msgs::msg::PlatformStatus::LANDING, Event::LANDED,
      as2_msgs::msg::PlatformStatus::LANDED});

    // EMERGENCY TRANSITIONS
    transitions_.emplace_back(StateMachineTransition{
      "EMERGENCY", as2_msgs::msg::PlatformStatus::DISARMED, Event::EMERGENCY,
      as2_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
      "EMERGENCY", as2_msgs::msg::PlatformStatus::LANDED, Event::EMERGENCY,
      as2_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
      "EMERGENCY", as2_msgs::msg::PlatformStatus::TAKING_OFF, Event::EMERGENCY,
      as2_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
      "EMERGENCY", as2_msgs::msg::PlatformStatus::FLYING, Event::EMERGENCY,
      as2_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
      "EMERGENCY", as2_msgs::msg::PlatformStatus::LANDING, Event::EMERGENCY,
      as2_msgs::msg::PlatformStatus::EMERGENCY});
  };
};

}  //namespace as2

#endif
