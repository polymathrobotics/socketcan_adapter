# Release Notes

## v0.1.0
__INITIAL RELEASE__

### Features

- CanFrame wrapper around struct can_frame
- SocketcanAdapter wrapper around socketcan sockets
  - Ability to send and receive CanFrame types
  - Built in Threading to receive data
- SocketcanBridgeNode to run a ros2 passthrough node
