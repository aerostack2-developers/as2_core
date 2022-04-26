#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "as2_core/control_mode_utils/control_mode_utils.hpp"
#include "as2_core/yaml_utils/yaml_utils.hpp"

// const std::string yaml_file_path = "tests/yaml_reading_text.yaml";
// const std::string config_path =
//     "/home/miguel/aerostack2_ws/src/aerostack2/stack_devel/controller_manager/"
//     "controller_plugin_base/available_modes.yaml";
const std::string config_path =
    "/home/miguel/aerostack2_ws/install/controller_plugin_differential_flatness/share/"
    "controller_plugin_differential_flatness";

int main(int argc, char* argv[]) {
  auto tags = as2::find_tag_from_project_exports_path(config_path, "available_modes");
  for (auto& tag : tags) {
    std::cout << tag << std::endl;
  }

  return 0;
}
