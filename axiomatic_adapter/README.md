# Axiomatic Adapter
Library and Adapter for Axiomatic CAN-Ethernet converters.

For more information on the decoding/encoding, see:

https://www.notion.so/polymathrobotics/Axiomatic-CAN-to-Ethernet-Converter-08e078d8914f40d6b7cd99ebf39fe1b0

## Supported Models

| `--model` | Device | Protocol reference |
| --- | --- | --- |
| `ax140900` (default) | [AX140900 CAN/Ethernet Converter](https://www.axiomatic.com/product/canethernet-converter-ax140900/) | [Ethernet to CAN Converter Communication Protocol](https://www.axiomatic.com/wp-content/uploads/Ethernet-to-CAN-Converter-Communication-Protocol.pdf) |
| `ax142100a` | [AX142100A Protocol Converter, Ethernet/RS-422/2x RS-232/CAN](https://www.axiomatic.com/product/protocol-converter-ethernet-rs-422-2-rs-232-can-sae-j1939-ax142100a/) | [UMAX142100A](https://www.axiomatic.com/wp-content/uploads/UMAX142100A.pdf), section 4.2 |

Every model shares the 11-byte `AXIO` message header in `axiomatic_protocol.hpp`.
Each model is a subclass of `AxiomaticCodec` (`axiomatic_codec.hpp`) in `include/axiomatic_adapter/models/` and `src/models/`.
A codec takes bytes and `CanFrame`s only, so it can be used without a socket.

### Adding a model

1. Add `include/axiomatic_adapter/models/<model>.hpp` and `src/models/<model>.cpp` with a `<model>::Codec` deriving `AxiomaticCodec`.
   Override `protocolId`, `encode` (build on `protocol::encodeMessage`), and `decodeMessage` (one message body; return `false` for Message IDs that carry no CAN frames).
   The base class `decode` splits the buffer into messages and calls `decodeMessage` for each.
2. Add the source to the `axiomatic_adapter` library in `CMakeLists.txt`.
3. Add its entry to `codecFactories` in `src/axiomatic_codec.cpp`.
4. Add a decode test of the manual's example bytes in `test/axiomatic_codec_test.cpp`; the round trip test covers every entry in `codecFactories`.

## Usage
### Socketcan-Axiomatic Bridge

```bash
ros2 run axiomatic_adapter axiomatic_socketcan_bridge [CAN_INTERFACE_NAME] [IP_ADDRESS] [PORT] [OPTIONAL]--model[-m] [OPTIONAL]--retry-connection[-r] [OPTIONAL]--max-retry-attempts [OPTIONAL]--verbose[-v] [OPTIONAL]--no-tcp-nodelay

# Examples
# generic example to bridge vcan0 with axiomatic using 192.168.50.34:4000
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000

# bridge vcan0 with an AX142100A
ros2 run axiomatic_adapter axiomatic_socketcan_bridge vcan0 192.168.50.34 4000 --model ax142100a

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
  /*tcp_nodelay=*/true,  // optional, defaults to true; see below
  std::make_unique<polymath::can::ax142100a::Codec>()  // optional, defaults to ax140900::Codec
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
5. AX142100A raw data (serial) payloads are skipped; only CAN frames are delivered.
6. AX142100A 11-bit IDs are assumed to be 2 bytes on the wire; the manual only shows a 29-bit example.
