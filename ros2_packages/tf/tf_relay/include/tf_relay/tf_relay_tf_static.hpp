#ifndef TF_RELAY_TF_STATIC_HPP_
#define TF_RELAY_TF_STATIC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <string>
#include <vector>
#include <unordered_map>

/**
 * @class TfStaticRelay
 * @brief A ROS2 node that subscribes to multiple robots' /tf_static topics and republishes them with a namespace prefix.
 *
 * Similar to TfRelay, but for static transforms. Static transforms do not change over time and are
 * typically broadcast on the /tf_static topic. By prefixing the frames, we avoid collisions when
 * multiple robots with identical static frame names coexist in the same environment.
 */
class TfStaticRelay : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new TfStaticRelay node.
   *
   * @param options Node options for parameter initialization and execution management.
   * This constructor declares a 'robot_namespaces' parameter and sets up subscribers/publishers
   * based on that parameter.
   */
  TfStaticRelay(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback function for incoming TF static messages.
   *
   * This function prefixes each static transform's frame IDs with the robot namespace if they are not
   * already prefixed. The 'world' frame is left unchanged.
   *
   * @param msg Shared pointer to the incoming TF static message.
   * @param ns The namespace corresponding to the robot whose TF_STATIC is being relayed.
   */
  void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & ns);

  /// A list of robot namespaces that we will relay static TF frames for.
  std::vector<std::string> robot_namespaces_;

  /// Map from robot namespace to the static TF subscriber for that namespace.
  std::unordered_map<std::string, rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_static_subscribers_;

  /// Map from robot namespace to the static TF publisher for that namespace.
  std::unordered_map<std::string, rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> tf_static_publishers_;
};

#endif
