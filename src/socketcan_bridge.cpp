#include <cstdio>
#include <memory>
#include "socketcan_adapter/socketcan_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
    std::make_shared<polymath::socketcan::SocketcanBridgeNode>();
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node->get_node_base_interface());

  node->configure();
  node->activate();

  executor.spin();

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();

  return 0;
}
