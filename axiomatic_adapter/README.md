# Axiomatic Adapter
Library and Adapter for the Axiomatic CAN-ETH converter.

For more information on the decoding/endcoding and product, see the links below:

https://www.notion.so/polymathrobotics/Axiomatic-CAN-to-Ethernet-Converter-08e078d8914f40d6b7cd99ebf39fe1b0

https://products.axiomatic.com/viewitems/connectivity/ethernet-can-converters

## Usage
### Socketcan-Axiomatic Bridge

```bash
ros2 run axiomatic_adapter axiomatic_socketcan_bridge [CAN_INTERFACE_NAME] [IP_ADDRESS] [PORT] [OPTIONAL]--retry-connection[-r] [OPTIONAL]--max-retry-attempts [OPTIONAL]--verbose[-v] [OPTIONAL]--no-tcp-nodelay

# Examples
# generic example to bridge vcan0 with axiomatic using 192.168.50.34:4000
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000

# this will continue retrying to connect forever and not exit on first failure. It will also print more detailed logs
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 -r -v

# this will attempt to reconnect a max number of 100 times before failing
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 -r --max-retry-attempts 100

# disable TCP_NODELAY (re-enable Nagle's algorithm). Default is on; only use if
# you are pushing bulk traffic where throughput matters more than per-frame latency.
# See the Library section below for the full tradeoff discussion.
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 --no-tcp-nodelay
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
  receive_timeout_ms,
  /*tcp_nodelay=*/true  // optional, defaults to true; see below
);

// open the socket
adapter.openSocket();

// start the reception thread. At this point, it's running
adapter.startReceptionThread();

// on shutdown/destruction, thread will join and socket will close
```

#### `tcp_nodelay` parameter

Controls whether `TCP_NODELAY` is set on the socket after a successful connect.
Defaults to `true`.

- **`true` (default)**: Nagle's algorithm is disabled. Each CAN frame leaves the
  host as its own TCP segment. The converter's delayed-ACK behaviour otherwise
  interacts with Nagle to produce 30–450 ms inter-segment stalls under sustained
  CAN traffic, which breaks UDS-style request/response timing (flash sessions,
  control loops). This is the right choice for any latency-sensitive workload.
- **`false`**: Nagle's algorithm stays enabled. The kernel may coalesce many
  small CAN frames into fewer, larger TCP segments. This lowers
  packets-per-second on the network and reduces per-segment header overhead
  (~40 bytes IP+TCP per ~24 byte CAN frame payload), at the cost of much higher
  worst-case latency per individual frame. Useful only if you are pushing bulk
  data where overall throughput matters more than per-frame latency, or if the
  network path / receiving device cannot keep up at high PPS.

If you have any doubt, leave it at the default.

## KNOWN ISSUES
1. Axiomatic-specific heartbeat messages are not handled and are deliberately skipped. Using TCP, this does not cause issues. In future updates heartbeat messages should be consumed and sent as needed
2. Axiomatic-specific status messages are ignored and deliberately skipped. In future revisions this should be handled and reported as necessary.
3. CAN FD is not supported
4. Only TCP mode is supported; no UDP support (heartbeats are required for UDP support)
