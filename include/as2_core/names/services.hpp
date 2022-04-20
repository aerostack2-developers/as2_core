#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names
{
    namespace services
    {
        namespace platform
        {
            const std::string setplatformcontrolmode = "platform/";
            const std::string setplatformstatemachineevent = "platform/";
        }
        namespace motion_reference
        {
            const std::string sendtrajwayp = "motion_reference/";
            const std::string setcontrolmode = "set_controller_control_mode/";
        }
        namespace gps
        {
            const std::string getorigin = "";
            const std::string setorigin = "";
            const std::string path2geopath = "";
            const std::string geopath2path = "";
        }
        const std::string setspeed = "";  // TODO
    }
}
