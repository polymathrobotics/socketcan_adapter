#ifndef SOCKETCAN_ADAPTER__SOCKETCAN_BRIDGE_NODE_HPP_
#define SOCKETCAN_ADAPTER__SOCKETCAN_BRIDGE_NODE_HPP_

#include "socketcan_adapter/socketcan_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "can_msgs/msg/frame.hpp"


namespace polymath
{
namespace socketcan
{

class SocketcanBridgeNode : public rclcpp_lifecycle::LifecycleNode
{

public:
  /// @brief Construct the socketcan bridge node
  /// @param options
  explicit SocketcanBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destruct SocketcanBridgeBNode
  ~SocketcanBridgeNode();

protected:
  using rclcpp_lifecycle_callback_return =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Configure the Node and socket
  /// @param state
  /// @return lifecycle callback return
  rclcpp_lifecycle_callback_return on_configure(const rclcpp_lifecycle::State & state) override;

  /// @brief Activate the Node and receive thread
  /// @param state
  /// @return lifecycle callback return
  rclcpp_lifecycle_callback_return on_activate(const rclcpp_lifecycle::State & state) override;

  /// @brief Deactivate the Node and stop the thread
  /// @param state
  /// @return lifecycle callback return
  rclcpp_lifecycle_callback_return on_deactivate(const rclcpp_lifecycle::State & state) override;

  /// @brief Cleanup the Node and close the Socket
  /// @param state
  /// @return lifecycle callback return
  rclcpp_lifecycle_callback_return on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// @brief Shutdown the node and clean up
  /// @param state
  /// @return lifecycle callback return
  rclcpp_lifecycle_callback_return on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// @brief Convert vector of strings to Filter Vector for socketcan
  /// @param filter_vector Vector of strings to convert
  /// @return filter_vector_t, a vector of filters
  SocketcanAdapter::filter_vector_t stringVectorToFilterVector(
    const std::vector<std::string> & filter_vector);

  /// @brief Publish CanFrame, used as a callback for the socketcam adapter
  /// @param frame CanFrame const
  void publishCanFrame(std::unique_ptr<const CanFrame> frame);

  /// @brief Callback if an error is received
  /// @param error
  void socketErrorCallback(SocketcanAdapter::socket_error_string_t error);

  /// @brief Transmit can_msgs frame type
  /// @param frame  can_msgs::msg::Frame::UniquePtr
  void transmitCanFrame(can_msgs::msg::Frame::UniquePtr frame);

  /// @brief Set up the socketcan adapter
  std::unique_ptr<SocketcanAdapter> socketcan_adapter_{nullptr};

  /// @brief canframe publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_publisher_{nullptr};

  /// @brief canframe subscriber
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frame_subscriber_{nullptr};
};

} // namespace socketcan
} // namespace polymath

#endif // SOCKETCAN_ADAPTER__SOCKETCAN_BRIDGE_NODE_HPP_
