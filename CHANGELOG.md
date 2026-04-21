# Release Notes

## v0.0.0
__INITIAL RELEASE__

### Features

- CanFrame wrapper around struct can_frame
- SocketcanAdapter wrapper around socketcan sockets
  - Ability to send and receive CanFrame types
  - Built in Threading to receive data
- SocketcanBridgeNode to run a ros2 passthrough node

## v0.2.1

### Features
- Now with `axiomatic_adapter` package!
- Introduces an additional interface between Axiomatic's CAN-ETH converter and CAN messages
- Has a wrapper to put ETH CAN traffic onto a socketcan interface using `SocketcanAdapter`
