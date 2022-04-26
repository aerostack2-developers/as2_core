#include "as2_core/yaml_utils/yaml_utils.hpp"

namespace as2 {
std::filesystem::path get_project_export_path_from_xml_path(const std::filesystem::path& xml_path) {
  std::filesystem::path config_path = xml_path;
  while (config_path.has_parent_path() && config_path.parent_path() != "/") {
    if (config_path.parent_path().filename() == "share") {
      return config_path;
    }
    config_path = config_path.parent_path();
  }
  return std::filesystem::path();
};

std::vector<std::filesystem::path> find_yaml_files_inside(const std::filesystem::path& dir) {
  std::vector<std::filesystem::path> files;
  for (auto& p : std::filesystem::recursive_directory_iterator(dir)) {
    if (p.path().extension() == ".yaml") {
      files.push_back(p.path());
    }
  }
  return files;
};

// Recursive function to find the tag inside the yaml file
YAML::Node find_tag_in_yaml_node(const YAML::Node& node, const std::string& tag) {
  if (node.IsSequence() || node.IsMap()) {
    for (auto& child : node) {
      if (child.first.as<std::string>() == tag) {
        return child.second;
      }
      auto result = find_tag_in_yaml_node(child.second, tag);
      if (result != YAML::Node()) {
        return result;
      }
    }
  } else if (node.IsScalar()) {
    if (node.as<std::string>() == tag) {
      return node;
    }
  }
  return YAML::Node();
};

// template <typename T>
// std::vector<T> find_tag_in_yaml_file(const std::filesystem::path& yaml_file,
//                                      const std::string& tag) {
//   std::vector<T> result;
//   std::ifstream config_file(yaml_file);
//   // check if file exists
//   if (!config_file.good()) {
//     throw std::runtime_error("Could not open yaml file: " + yaml_file.string());
//   }
//   YAML::Node node = YAML::LoadFile(yaml_file);
//   for (const auto& elem : find_tag_in_yaml_node(node, tag)) {
//     T value = elem.as<T>();
//     result.emplace_back(value);
//   }
//   return result;
// };

// template <typename T>
// std::vector<T> read_tag_from_project_exports_path(const std::filesystem::path& project_exports_path,
//                                                   const std::string& tag) {
//   std::vector<T> values;
//   std::vector<std::filesystem::path> yaml_files = find_yaml_files_inside(project_exports_path);
//   if (yaml_files.size() == 0) {
//     std::cerr << "Could not find yaml files with available modes" << std::endl;
//     return std::vector<T>();
//   }

//   for (const auto& yaml_file : yaml_files) {
//     auto tags = find_tag_in_yaml_file<T>(yaml_file, tag);
//     if (tags.size() > 0) {
//       values = tags;
//     }
//   }
//   return values;
// }

YAML::Node search_tag_across_multiple_yaml_files(
    const std::vector<std::filesystem::path>& yaml_files, const std::string& tag) {
  for (const auto& yaml_file : yaml_files) {
    YAML::Node config = YAML::LoadFile(yaml_file.string());
    auto available_modes_node = as2::find_tag_in_yaml_node(config, tag);
    if (available_modes_node) {
      return available_modes_node;
    }
  }
  return YAML::Node();
};

uint8_t parse_uint_from_string(const std::string& str) {
  uint8_t value;
  // check if string begins with 0b or 0x and remove it
  // if it does not, it is assumed to be decimal
  if (str.substr(0, 2) == "0b") {
    auto string_value = str.substr(2);
    value = stoi(string_value, 0, 2);
  } else if (str.substr(0, 2) == "0x") {
    std::stringstream ss(str.substr(2));
    ss >> std::hex >> value;
  } else {
    std::stringstream ss(str);
    ss >> value;
  }
  return value;
};

};  // namespace as2
