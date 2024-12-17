#include "tf_relay/tf_relay_json_parser.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

std::vector<std::string> load_robot_namespaces_from_json(
  const std::string & json_file_path,
  rclcpp::Logger logger)
{
  std::vector<std::string> namespaces;

  std::ifstream json_file(json_file_path);
  if (!json_file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open JSON file: %s", json_file_path.c_str());
    return namespaces;
  }

  nlohmann::json j;
  try {
    json_file >> j;
  } catch (const nlohmann::json::parse_error &e) {
    RCLCPP_ERROR(logger, "JSON parse error: %s", e.what());
    return namespaces;
  }

  for (auto& [robot_key, robot_data] : j.items()) {
    if (robot_data.contains("namespace") && robot_data["namespace"].is_string()) {
      std::string ns = robot_data["namespace"];
      namespaces.push_back(ns);
    } else {
      RCLCPP_WARN(logger, "Entry '%s' does not contain a valid 'namespace' field.", robot_key.c_str());
    }
  }

  RCLCPP_INFO(logger, "Loaded %zu robot namespaces from JSON.", namespaces.size());
  return namespaces;
}
