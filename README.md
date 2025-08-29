# SocketCAN Adapter

A modular SocketCAN driver library and ROS2 integration for Linux-based systems. This repository provides both a standalone C++ library and ROS2 wrapper for easy integration with robotics applications.

## Architecture

This package is split into two complementary components:

### 📚 [socketcan_adapter](./socketcan_adapter/README.md)
Pure C++ SocketCAN library with no ROS dependencies.
- Core SocketCAN functionality
- Thread-safe operations
- Callback-based asynchronous processing
- Configurable filters and error handling
- Can be used in non-ROS projects

### 🤖 [socketcan_adapter_ros](./socketcan_adapter_ros/README.md)
ROS2 wrapper providing nodes and launch files.
- ROS2 lifecycle node
- Integration with `can_msgs`
- Launch files with parameter support
- Depends on the core `socketcan_adapter` library

## Quick Start

### Build Both Packages

```bash
# Install dependencies
rosdep install -i -y --from-paths .

# Build everything
colcon build --packages-up-to socketcan_adapter_ros
```

### Launch ROS2 CAN Bridge

```bash
ros2 launch socketcan_adapter_ros socketcan_bridge_launch.py
```

### Use Core Library in C++

```c++
#include "socketcan_adapter/socketcan_adapter.hpp"

using namespace polymath::socketcan;

SocketcanAdapter adapter("can0");
adapter.openSocket();
// ... see socketcan_adapter README for full example
```

## Requirements

### System Requirements
- Linux with SocketCAN support
- C++17 compatible compiler
- CMake 3.8+

### ROS2 Requirements (for ROS package only)
- ROS2 Humble or later
- can_msgs package

## Testing

### Core Library Tests

```bash
colcon test --packages-select socketcan_adapter
```

### ROS2 Package Tests

```bash
colcon test --packages-select socketcan_adapter_ros
```

### Hardware Tests
Set `CAN_AVAILABLE=1` to enable hardware-dependent tests:

```bash
CAN_AVAILABLE=1 colcon test --packages-select socketcan_adapter
```

## Virtual CAN Setup

For testing without hardware:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Contributing

We welcome contributions! Please see our [contributing guidelines](CONTRIBUTING.md) for details on:
- Code style and standards
- Testing requirements
- Pull request process
- Issue reporting

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for a detailed history of changes.

## Support

- 📖 **Documentation**: See individual package READMEs for detailed usage
- 🐛 **Bug Reports**: Please use GitHub Issues with detailed reproduction steps
- 💡 **Feature Requests**: Open a GitHub Issue with your use case
- 📧 **Contact**: engineering@polymathrobotics.com

## Security

If you discover a security vulnerability, please send an email to security@polymathrobotics.com. All security vulnerabilities will be promptly addressed.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for the full license text.

## Authors & Maintainers

- **Zeerek Ahmad** - Original author - zeerekahmad@hotmail.com
- **Polymath Robotics Engineering Team** - Maintainers - engineering@polymathrobotics.com

## Acknowledgments

- Linux SocketCAN developers for the underlying CAN bus support
- ROS2 community for the robotics middleware framework
- Contributors and users who help improve this project
