#include "tf_relay/tf_relay_tf_static.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include "tf_relay/tf_relay_json_parser.hpp"  // Include the new parser

TfStaticRelay::TfStaticRelay(const rclcpp::NodeOptions & options)
: Node("tf_relay_tf_static", options)
{
  // Declare parameters
  this->declare_parameter("robot_namespaces", std::vector<std::string>());
  this->declare_parameter("use_json", false);
  this->declare_parameter("json_file_path", "");

  bool use_json = this->get_parameter("use_json").as_bool();

  if (use_json) {
    std::string json_path = this->get_parameter("json_file_path").as_string();
    robot_namespaces_ = load_robot_namespaces_from_json(json_path, this->get_logger());
  } else {
    robot_namespaces_ = this->get_parameter("robot_namespaces").as_string_array();
  }

  if (robot_namespaces_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No robot namespaces provided for static TF. No transforms will be relayed.");
    return;
  }

  for (auto & ns : robot_namespaces_) {
    std::string input_topic = ns + "/tf_static";
    std::string output_topic = ns + "/relayed/tf_static";

    tf_static_publishers_[ns] = this->create_publisher<tf2_msgs::msg::TFMessage>(output_topic, 10);
    tf_static_subscribers_[ns] = this->create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic,
      10,
      [this, ns](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        this->tfStaticCallback(msg, ns);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Relaying static TF from [%s] to [%s]", input_topic.c_str(), output_topic.c_str());
  }
}

void TfStaticRelay::tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg, const std::string & ns)
{
  const std::string prefix = ns + "/";
  auto out_msg = tf2_msgs::msg::TFMessage();
  out_msg.transforms.reserve(msg->transforms.size());

  for (auto & t : msg->transforms) {
    auto transform = t;
    if (!transform.header.frame_id.empty() && transform.header.frame_id != "world") {
      if (transform.header.frame_id.compare(0, prefix.size(), prefix) != 0) {
        transform.header.frame_id = prefix + transform.header.frame_id;
      }
    }

    if (!transform.child_frame_id.empty()) {
      if (transform.child_frame_id.compare(0, prefix.size(), prefix) != 0) {
        transform.child_frame_id = prefix + transform.child_frame_id;
      }
    }

    out_msg.transforms.push_back(transform);
  }

  tf_static_publishers_[ns]->publish(out_msg);
}