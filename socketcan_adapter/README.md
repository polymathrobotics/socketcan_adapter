# socketcan_adapter

A C++ SocketCAN driver library for Linux-based systems. This library provides a high-level interface to Linux SocketCAN without any ROS dependencies.

## Classes of Note

### CanFrame
`CanFrame` Class - This class wraps the C-level `can_frame` structure, encapsulating CAN message details like the CAN ID, data, timestamp, and frame type (DATA, ERROR, or REMOTE). By providing a robust API for creating and managing CAN frames, CanFrame simplifies interaction with raw CAN data and offers utilities like ID masking, setting error types, and timestamp management.

Example highlights:

- Flexible constructors for `can_frame` struct and raw data inputs.
- Functions to modify frame type, ID type (standard/extended), and length.
- Helper methods to access CAN frame data, ID.

Does not implement CanFD yet.

### SocketcanAdapter
`SocketcanAdapter` Class - The `SocketcanAdapter` abstracts and manages socket operations for CAN communication. It initializes and configures the socket, applies filters, and handles CAN frame transmission and reception. The adapter offers error handling, thread-safe operations, and optional callback functions for asynchronous frame and error processing.

Key features:

- Configurable receive timeout and threading for reception.
- `setFilters` and setErrorMaskOverwrite to apply CAN filters and error masks.
- A callback-based system for handling received frames and errors asynchronously.
- Supports multiple send and receive methods, including `std::shared_ptr` for efficient memory management.
- Together, `CanFrame` and `SocketcanAdapter` simplify interaction with CAN networks, allowing developers to focus on high-level application logic instead of low-level socket and data handling.

## Build

This package can be built with the ROS2 ament toolchain or as a standalone CMake project.

### ROS2 Build
Install dependencies:

```bash
rosdep install -i -y --from-paths socketcan_adapter
```

Build:

```bash
colcon build --packages-select socketcan_adapter
```

### Standalone CMake Build

```bash
mkdir build && cd build
cmake ..
make
```

## Sample Usage

```c++
#include "socketcan_adapter/socketcan_adapter.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include <iostream>
#include <thread>
#include <vector>

using namespace polymath::socketcan;

int main() {
    // Initialize SocketcanAdapter with the CAN interface name (e.g., "can0")
    SocketcanAdapter adapter("can0");

    // Open the CAN socket
    if (!adapter.openSocket()) {
        std::cerr << "Failed to open CAN socket!" << std::endl;
        return -1;
    }

    // Step 1: Set up a filter to allow only messages with ID 0x123
    std::vector<struct can_filter> filters = {{0x123, CAN_SFF_MASK}};
    if (auto error = adapter.setFilters(filters)) {
        std::cerr << "Error setting filters: " << *error << std::endl;
        return -1;
    }

    // Step 2: Set up a callback function to handle received CAN frames
    adapter.setOnReceiveCallback([](std::unique_ptr<const CanFrame> frame) {
        std::cout << "Received CAN frame with ID: " << std::hex << frame->get_id() << std::endl;
        auto data = frame->get_data();
        std::cout << "Data: ";
        for (const auto& byte : data) {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
    });

    // Step 3: Start the reception thread
    if (!adapter.startReceptionThread()) {
        std::cerr << "Failed to start reception thread!" << std::endl;
        adapter.closeSocket();
        return -1;
    }

    // Step 4: Prepare a CAN frame to send
    canid_t raw_id = 0x123;
    std::array<unsigned char, CAN_MAX_DLC> data = {0x11, 0x22, 0x33, 0x44};
    uint64_t timestamp = 0; // Placeholder timestamp
    CanFrame frame(raw_id, data, timestamp);

    // Step 5: Send the CAN frame
    if (auto error = adapter.send(frame)) {
        std::cerr << "Failed to send CAN frame: " << *error << std::endl;
    } else {
        std::cout << "Sent CAN frame with ID: " << std::hex << raw_id << std::endl;
    }

    // Keep the application running for 10 seconds to allow for frame reception
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Step 5: Clean up - close the socket and stop the reception thread
    adapter.joinReceptionThread();
    adapter.closeSocket();

    return 0;
}
```

## Testing

Run tests:

```bash
colcon test --packages-select socketcan_adapter
```

Note: Hardware tests require `CAN_AVAILABLE=1` environment variable and a functioning CAN interface.


### Creating Virtual CAN Interface

For testing without hardware:

```bash
# Load the vcan module
sudo modprobe vcan

# Create a virtual CAN interface
sudo ip link add dev vcan0 type vcan

# Bring up the interface
sudo ip link set up vcan0
```
