# Axiomatic Adapter
Library and Adapter for the Axiomatic CAN-ETH converter.

For more information on the decoding/endcoding and product, see the links below:

https://www.notion.so/polymathrobotics/Axiomatic-CAN-to-Ethernet-Converter-08e078d8914f40d6b7cd99ebf39fe1b0

https://products.axiomatic.com/viewitems/connectivity/ethernet-can-converters

## Usage
### Socketcan-Axiomatic Bridge

```bash
ros2 run axiomatic_adapter axiomatic_socketcan_bridge [CAN_INTERFACE_NAME] [IP_ADDRESS] [PORT] [OPTIONAL]--retry-connection[-r] [OPTIONAL]--max-retry-attempts [OPTIONAL]--verbose[-v]

# Examples
# generic example to bridge vcan0 with axiomatic using 192.168.50.34:4000
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000

# this will continue retrying to connect forever and not exit on first failure. It will also print more detailed logs
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 -r -v

# this will attempt to reconnect a max number of 100 times before failing
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 -r --max-retry-attempts 100
```

### Library

```cpp
// construct the adapter
std::string ip_address = "192.168.0.34";
std::string port = "4000";
std::chrono::milliseconds receive_timeout_ms(100);

// the two functions passed in are the receive and error callback functions, receive timeout has a default
polymath::can::AxiomaticAdapter adapter(
  ip_address,
  port,
  [](std::unique_ptr<const CanFrame> /*frame*/) { /* No-op */ },
  [](polymath::can::AxiomaticAdapter::socket_error_string_t /*error*/) { /*do nothing*/ },
  receive_timeout_ms
);

// open the socket
adapter.openSocket();

// start the reception thread. At this point, it's running
adapter.startReceptionThread();

// on shutdown/destruction, thread will join and socket will close
```

## KNOWN ISSUES
1. There seems to be an issue with massive amounts of CAN data causing timing inconsistencies through the adapter library
   1. This has not been fully diagnosed
