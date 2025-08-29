# socketcan_adapter_ros

A ROS2 wrapper for the socketcan_adapter library, providing ROS2 nodes and launch files for easy integration with ROS2 systems.

## Dependencies

- [socketcan_adapter](../socketcan_adapter/README.md) - Core SocketCAN library
- rclcpp - ROS2 C++ client library
- rclcpp_lifecycle - ROS2 lifecycle node support
- can_msgs - ROS2 CAN message definitions

## Build

Install dependencies:

```bash
rosdep install -i -y --from-paths socketcan_adapter_ros
```

Build:

```bash
colcon build --packages-up-to socketcan_adapter_ros
```

## Usage

### Launch the CAN Bridge Node

```bash
ros2 launch socketcan_adapter_ros socketcan_bridge_launch.py
```

### Launch Parameters

- `can_interface`: CAN interface to connect to (default: "can0")
- `can_error_mask`: CAN error mask (default: 0x1FFFFFFF - everything allowed)
- `can_filter_list`: CAN filters as list of ID/mask pairs (default: [])
- `join_filters`: Use joining logic for filters (default: false)
- `auto_configure`: Automatically configure the lifecycle node (default: true)
- `auto_activate`: Automatically activate the lifecycle node post configuration (default: true)

### Example Launch with Parameters

```bash
ros2 launch socketcan_adapter_ros socketcan_bridge_launch.py \
    can_interface:=can1 \
    can_error_mask:=0x1F \
    can_filter_list:="[{id: 0x123, mask: 0x7FF}, {id: 0x456, mask: 0x7FF}]"
```

### Manual Lifecycle Management

If you prefer manual control over the lifecycle:

```bash
# Launch without auto-activation
ros2 launch socketcan_adapter_ros socketcan_bridge_launch.py \
    auto_configure:=false \
    auto_activate:=false

# Configure the node
ros2 lifecycle set /socketcan_bridge configure

# Activate the node
ros2 lifecycle set /socketcan_bridge activate
```

## Topics

### Published Topics

- `/can_rx` (`can_msgs/msg/Frame`) - Received CAN frames from the bus

### Subscribed Topics

- `/can_tx` (`can_msgs/msg/Frame`) - CAN frames to transmit to the bus

## Node Details

The `socketcan_bridge` node is implemented as a ROS2 lifecycle node

### Parameters

The node accepts the following ROS2 parameters:

- `can_interface` (string): Name of the CAN interface (e.g., "can0", "vcan0")
- `error_mask` (int): CAN error mask for filtering error frames
- `filters` (array): List of CAN filter objects with `id` and `mask` fields
- `join_filters` (bool): Whether to use joining logic for multiple filters

## Requirements

- ROS2 (Humble or later)
- Linux system with SocketCAN support
- [socketcan_adapter](../socketcan_adapter/README.md) library
- Active CAN interface (real or virtual)
