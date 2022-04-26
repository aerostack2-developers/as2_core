#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names
{
    namespace services
    {
        namespace platform
        {
            const std::string set_arming_state = "set_arming_state";
            // const std::string set_arming_state = "platform/set_arming_state";
            const std::string set_offboard_mode = "set_offboard_mode";
            // const std::string set_offboard_mode = "platform/set_offboard_mode";
            const std::string set_platform_control_mode = "set_platform_control_mode";
            // const std::string set_control_mode = "platform/set_control_mode";
            const std::string takeoff = "platform_takeoff";
            // const std::string takeoff = "platform/takeoff";
            const std::string land = "platform_land";
            // const std::string land = "platform/land";
            const std::string set_platform_state_machine_event = "platform/state_machine_event";
            // const std::string set_state_machine_event = "platform/state_machine_event";
            const std::string list_control_modes = "platform/list_control_modes";
        }
        namespace controller
        {
            const std::string set_control_mode = "controller/set_control_mode";
            const std::string list_control_modes = "controller/list_control_modes";
        }
        namespace motion_reference
        {
            const std::string send_traj_wayp = "";
        }
        namespace gps
        {
            const std::string get_origin = "";
            const std::string set_origin = "";
            const std::string path_to_geopath = "";
            const std::string geopath_to_path = "";
        }
        const std::string set_speed = "";  // TODO
    }
}
