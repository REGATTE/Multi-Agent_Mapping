#ifndef TF_RELAY_TF_HPP_
#define TF_RELAY_TF_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

/**
 * @class TfRelay
 * @brief A ROS2 node that subscribes to multiple robots' /tf topics and republishes them with a namespace prefix.
 *
 * The purpose of this class is to avoid name collisions in TF frames when multiple robots with
 * identical frame names (e.g., "base_link") are present in the same environment. By prefixing each
 * robot's frames with their respective namespaces (e.g., "robot_1/base_link"), we ensure that
 * visualization and tooling software recognizes the frames as distinct.
 */
class TfRelay : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new TfRelay node.
   *
   * @param options Node options for parameter initialization and execution management.
   * This constructor declares a 'robot_namespaces' parameter and sets up subscribers/publishers
   * based on that parameter.
   */
  TfRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback function for incoming TF messages.
   *
   * This function prefixes each transform's frame IDs with the robot namespace if they are not
   * already prefixed. The 'world' frame is left unchanged.
   *
   * @param msg Shared pointer to the incoming TF message.
   * @param ns The namespace corresponding to the robot whose TF is being relayed.
   */
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & ns);
  /// A list of robot namespaces that we will relay TF frames for.
  std::vector<std::string> robot_namespaces_;
  /// Map from robot namespace to the TF subscriber for that namespace.
  std::unordered_map<std::string, rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subscribers_;
  /// Map from robot namespace to the TF publisher for that namespace.
  std::unordered_map<std::string, rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> tf_publishers_;
};

#endif
