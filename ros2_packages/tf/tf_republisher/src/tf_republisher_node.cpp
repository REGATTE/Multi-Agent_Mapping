#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <string>

class TfRepublisher : public rclcpp::Node {
public:
    TfRepublisher(const std::string &namespace_)
        : Node("tf_republisher_node"), namespace_(namespace_) {
        // Subscribe to the namespace-specific TF topic
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            namespace_ + "/relayed/tf", 10,
            std::bind(&TfRepublisher::tfCallback, this, std::placeholders::_1));

        // Publish to the global /tf topic
        publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        RCLCPP_INFO(this->get_logger(), "TF Republisher initialized for namespace: %s", namespace_.c_str());
    }

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        // Republish the received message
        publisher_->publish(*msg);
    }

    std::string namespace_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: tf_republisher_node <namespace>");
        return 1;
    }

    auto node = std::make_shared<TfRepublisher>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
