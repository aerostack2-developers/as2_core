#ifndef __CONTROL_MODE_UTILS_HPP__
#define __CONTROL_MODE_UTILS_HPP__

#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "as2_msgs/msg/control_mode.hpp"

// const std::string config_path =
//     "/home/miguel/aerostack2_ws/src/aerostack2/stack_devel/controller_manager/"
//     "controller_plugin_base/available_modes.yaml";
//

namespace as2 {

// # ------------- mode codification (4 bits) ----------------------
// #
// # unset             = 0 = 0b00000000
// # hover             = 1 = 0b00010000
// # acro              = 2 = 0b00100000
// # attitude          = 3 = 0b00110000
// # speed             = 4 = 0b01000000
// # speed_in_a_plane  = 5 = 0b01010000
// # position          = 6 = 0b01100000
// # trajectory        = 7 = 0b01110000
// #
// #-------------- yaw codification --------------------------------
// #
// # angle             = 0 = 0b00000000
// # speed             = 1 = 0b00000100
// #
// # frame codification
// #
// # local_frame_flu   = 0 = 0b00000000
// # global_frame_enu  = 1 = 0b00000001
// # global_frame_lla  = 2 = 0b00000010
// #
// #-----------------------------------------------------------------

uint8_t convertAS2ControlModeToUint8t(const as2_msgs::msg::ControlMode& mode);
as2_msgs::msg::ControlMode convertUint8tToAS2ControlMode(uint8_t control_mode_uint8t);

void printControlMode(const as2_msgs::msg::ControlMode& mode);
void printControlMode(uint8_t control_mode_uint8t);

}  // namespace as2

#endif  // __CONTROL_MODE_UTILS_HPP__
