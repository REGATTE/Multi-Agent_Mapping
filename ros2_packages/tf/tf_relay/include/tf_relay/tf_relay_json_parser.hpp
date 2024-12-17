#ifndef TF_RELAY_JSON_PARSER_HPP_
#define TF_RELAY_JSON_PARSER_HPP_

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Load robot namespaces from a JSON file.
 *
 * The JSON file should have a structure like:
 * {
 *   "robot_1": { "namespace": "scout_1", ... },
 *   "robot_2": { "namespace": "jackal_1", ... }
 * }
 *
 * @param json_file_path The path to the JSON file.
 * @param logger A ROS2 logger for logging errors and warnings.
 * @return A vector of namespaces extracted from the JSON file.
 */
std::vector<std::string> load_robot_namespaces_from_json(
  const std::string & json_file_path,
  rclcpp::Logger logger
);

#endif  // TF_RELAY_JSON_PARSER_HPP_
