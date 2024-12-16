#include "tf_relay/tf_relay_tf.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

TfRelay::TfRelay(const rclcpp::NodeOptions & options)
: Node("tf_relay_tf", options)
{
  // Get the list of robot namespaces
  this->declare_parameter<std::vector<std::string>>("robot_namespaces", std::vector<std::string>());

  this->robot_namespaces_ = this->get_parameter("robot_namespaces").as_string_array();

  if (robot_namespaces_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No robot namespaces provided. Nothing will be relayed.");
    return;
  }

  for (auto & ns : robot_namespaces_) {
    // Input topic: ns + "/tf"
    // Output topic: ns + "/relayed/tf"
    std::string input_topic = ns + "/tf";
    std::string output_topic = ns + "/relayed/tf";

    tf_publishers_[ns] = this->create_publisher<tf2_msgs::msg::TFMessage>(output_topic, 10);
    tf_subscribers_[ns] = this->create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic, 10,
      [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        this->tfCallback(msg, ns);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Relaying TF from [%s] to [%s]", input_topic.c_str(), output_topic.c_str());
  }
}

void TfRelay::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & ns)
{
  const std::string prefix = ns + "/";  // Construct once for efficiency
  auto out_msg = tf2_msgs::msg::TFMessage();
  out_msg.transforms.reserve(msg->transforms.size());

  for (auto & t : msg->transforms) {
    auto transform = t;

    // If parent frame is not world and doesn't start with prefix, add prefix
    if (!transform.header.frame_id.empty() && transform.header.frame_id != "world") {
      if (transform.header.frame_id.compare(0, prefix.size(), prefix) != 0) {
        transform.header.frame_id = prefix + transform.header.frame_id;
      }
    }

    // If child frame doesn't start with prefix, add it
    if (!transform.child_frame_id.empty()) {
      if (transform.child_frame_id.compare(0, prefix.size(), prefix) != 0) {
        transform.child_frame_id = prefix + transform.child_frame_id;
      }
    }

    out_msg.transforms.push_back(transform);
  }

  tf_publishers_[ns]->publish(out_msg);
}
