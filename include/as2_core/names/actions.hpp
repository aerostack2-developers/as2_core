#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace as2_names
{
    namespace actions
    {
        namespace behaviours
        {
            const std::string takeoff = "TakeOffBehaviour";
            const std::string gotowaypoint = "GoToWaypointBehaviour";
            const std::string followpath = "FollowPathBehaviour";
            const std::string land = "LandBehaviour";
        }
    }
}
