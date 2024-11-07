# socketcan_adapter
Socketcan Driver Library for Linux based PCs and ROS2 nodes

# Build
Socketcan adapter can be built with the ros2 ament toolchain. All requirements can be installed via rosdep

Install the dependencies!
```bash
rosdep install -i -y --from-paths socketcan_adapter
```

Build it!
```bash
colcon build --packages-up-to socketcan_adapter
```

# Library
## Classes of Note
### CanFrame
`CanFrame` Class - This class wraps the C-level `can_frame` structure, encapsulating CAN message details like the CAN ID, data, timestamp, and frame type (DATA, ERROR, or REMOTE). By providing a robust API for creating and managing CAN frames, CanFrame simplifies interaction with raw CAN data and offers utilities like ID masking, setting error types, and timestamp management.

Example highlights:

- Flexible constructors for `can_frame` struct and raw data inputs.
- Functions to modify frame type, ID type (standard/extended), and length.
- Helper methods to access CAN frame data, ID, and timestamp.

Does not implement CanFD yet.

### SocketcanAdapter 
`SocketcanAdapter` Class - The `SocketcanAdapter` abstracts and manages socket operations for CAN communication. It initializes and configures the socket, applies filters, and handles CAN frame transmission and reception. The adapter offers error handling, thread-safe operations, and optional callback functions for asynchronous frame and error processing.

Key features:

- Configurable receive timeout and threading for reception.
- `setFilters` and setErrorMaskOverwrite to apply CAN filters and error masks.
- A callback-based system for handling received frames and errors asynchronously.
- Supports multiple send and receive methods, including `std::shared_ptr` for efficient memory management.
- Together, `CanFrame` and `SocketcanAdapter` simplify interaction with CAN networks, allowing developers to focus on - high-level application logic instead of low-level socket and data handling.

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

# ROS2 Node
To make usage even easier, this package comes with a ROS2 node with default settings!

## Launch
```bash
ros2 launch socketcan_adapter socketcan_bridge_launch.py
```

launch args:
- `can_interface`: can interface to connect to (default: 0)
- `can_error_mask`: can error mask (default: 0x1FFFFFFF aka everything allowed)
- `can_filter_list`: can filters (default: [])
- `join_filters`: use joining logic for filters (default: false)
- `auto_configure`: automatically configure the lifecycle node
- `auto_activate`: automatically activate the lifecycle node post configuration 
