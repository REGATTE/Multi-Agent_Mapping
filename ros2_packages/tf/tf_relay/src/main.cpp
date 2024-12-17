#include <rclcpp/rclcpp.hpp>
#include "tf_relay/tf_relay_tf.hpp"
#include "tf_relay/tf_relay_tf_static.hpp"

/**
 * @brief Entry point for the tf_relay executable.
 *
 * This file sets up the ROS2 environment, creates one instance of TfRelay and one instance
 * of TfStaticRelay, and then spins them to process incoming TF and TF_static transforms.
 */
int main(int argc, char * argv[])
{
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  // Create instances of the TF and TF static relay nodes
  auto tf_node = std::make_shared<TfRelay>();
  auto tf_static_node = std::make_shared<TfStaticRelay>();

  // Use a multithreaded executor to handle callbacks concurrently, which
  // can help maintain real-time-ish performance depending on system capabilities.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(tf_node);
  exec.add_node(tf_static_node);

  // Spin and process callbacks until shutdown
  exec.spin();

  // Shutdown the ROS2 system when finished
  rclcpp::shutdown();
  return 0;
}
