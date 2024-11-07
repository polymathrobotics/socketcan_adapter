#ifndef SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_
#define SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_

#include <string>
#include <functional>
#include <linux/can.h>
#include <atomic>
#include <chrono>
#include <optional>
#include <thread>
#include <poll.h>
#include "socketcan_adapter/visibility_control.h"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath
{
namespace socketcan
{

/// @brief State of socket, error, open or closed
enum class SocketState
{
  ERROR = -1,
  OPEN = 0,
  CLOSED = 1,
};

/// @brief FilterMode for how to add filters
enum class FilterMode
{
  /// TODO: Add additional filtermodes for "add" and "remove"
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/6
  OVERWRITE = 0,
};

constexpr std::chrono::duration<float> SOCKET_RECEIVE_TIMEOUT_S = std::chrono::duration<float>(0.1);
constexpr std::chrono::duration<float> JOIN_RECEPTION_TIMEOUT_S = std::chrono::duration<float>(0.1);
constexpr int32_t CLOSED_SOCKET_VALUE = -1;
constexpr nfds_t NUM_SOCKETS_IN_ADAPTER = 1;

/// @class polymath::socketcan::SocketcanAdapter
/// @brief Creates and manages a socketcan instance and simplifies the interface.
/// Generally does not throw, but returns booleans to tell you success
class SocketcanAdapter : public std::enable_shared_from_this<SocketcanAdapter>
{
public:
  /// @brief Mapped to std lib, but should be remapped to Polymath Safety compatible versions
  using socket_error_string_t = std::string;
  using filter_vector_t = std::vector<struct can_filter>;

  /// @brief SocketcanAdapter Class Init
  /// @param interface_name Interface for the socket to initialize on
  SocketcanAdapter(
    const std::string & interface_name,
    const std::chrono::duration<float> & receive_timeout_s = SOCKET_RECEIVE_TIMEOUT_S);

  /// @brief Destructor for SocketcanAdapter
  virtual ~SocketcanAdapter();

  /// @brief Open Socket
  /// @return bool successfully opened socket
  bool openSocket();

  /// @brief Close Socket
  /// @return bool successfully closed socket
  bool closeSocket();

  /// @brief Set a number of filters, vectorized
  /// @param filters reference to a vector of can filters to set for the socket
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> setFilters(
    const filter_vector_t & filters,
    FilterMode mode = FilterMode::OVERWRITE);

  /// Shared ptr to a vector is technically more efficient than a vector of shared_ptrs
  /// TODO: Vectors are harder to justify in MISRA, so might want to use Array
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/7
  /// @brief Set a number of filters, vectorized
  /// @param filters INPUT shared ptr to a vector of can filters to set for the socket
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> setFilters(
    const std::shared_ptr<filter_vector_t> filters,
    FilterMode mode = FilterMode::OVERWRITE);

  /// @brief Set the error mask
  /// @param error_mask INPUT error maskfor the socket to pass through
  /// @return optional string containing error information
  std::optional<socket_error_string_t> setErrorMaskOverwrite(const can_err_mask_t & error_mask);

  /// @brief Set the socket to Join Filters and do a logical AND instead of OR
  /// @param error_mask INPUT whether to set JOINED FILTERS
  /// @return optional string containing error information
  std::optional<socket_error_string_t> setJoinOverwrite(const bool & join);

  /// @brief Receive with a reference to a CanFrame to fill
  /// @param frame OUTPUT CanFrame to fill
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> receive(CanFrame & can_frame);

  /// TODO: Switch to unique ptr
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  /// @brief Receive with a reference to a CanFrame to fill
  /// @param frame OUTPUT CanFrame to fill via shared_ptr
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> receive(std::shared_ptr<CanFrame> frame);

  /// @brief Receive returns the received CanFrame
  /// @return optional CanFrame that was received by reference
  /// nullopt is returned if no canframe received, acts like null
  std::optional<const CanFrame> receive();

  /// @brief Start a reception thread (calls callback)
  /// @return success on started
  bool startReceptionThread();

  /// @brief Stop and join reception thread
  /// @param timeout_s INPUT timeout in seconds, <=0 means no timeout
  /// @return success on closed and joined thread
  bool joinReceptionThread(
    const std::chrono::duration<float> & timeout_s = JOIN_RECEPTION_TIMEOUT_S);

  /// @brief Set receive callback function if thread is used
  /// @param callback_function INPUT To be called on receipt of a can frame
  /// @return success on receive callback set
  bool setOnReceiveCallback(
    std::function<void(std::unique_ptr<const CanFrame> frame)> && callback_function);

  /// @brief Set receive callback function if thread is used
  /// @param callback_function INPUT To be called on receipt of a can frame
  /// @return success on error callback set
  bool setOnErrorCallback(
    std::function<void(socket_error_string_t error)> && callback_function);

  /// @brief Transmit a can frame via socket
  /// @param frame INPUT const reference to the frame
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const CanFrame & frame);

  /// @brief Transmit a can frame via socket
  /// @param frame INPUT shared_ptr to frame. Convert non-const to const by doing
  /// ```c++
  /// std::shared_ptr<CanFrame> frame = std::make_shared<CanFrame>();
  /// // do work on frame then lock the frame as const before sending
  /// std::shared_ptr<const CanFrame> frame = frame
  /// ```
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const std::shared_ptr<const CanFrame> frame);

  /// @brief Set receive timeout
  /// @param reecive_timeout_s std::chrono::duration<float> sets receive timeout in seconds
  void set_receive_timeout(const std::chrono::duration<float> & recive_timeout_s);

  /// @brief Get state of socket
  /// @return SocketState data type detailing OPEN or CLOSED
  SocketState get_socket_state();

  /// @brief Get interface
  /// @return Can interface
  std::string get_interface();

  /// @brief Checks if the receive thread is running
  /// @return True if the thread is running, false otherwise
  bool is_thread_running();

private:
  /// @brief Wraps C level socket operations to set can_filter frames
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendFilters();

  /// @brief Wraps C level socket operations to set error mask frames
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendErrorMask();

  /// @brief Wraps C level socket operations to set whether to join filters
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendJoin();

  std::string interface_name_;
  std::chrono::duration<float> receive_timeout_s_;

  int32_t socket_file_descriptor_;

  std::function<void(std::unique_ptr<const CanFrame> frame)> receive_callback_unique_ptr_;
  std::function<void(socket_error_string_t error)> receive_error_callback_;
  std::atomic<bool> thread_running_;
  std::atomic<bool> stop_thread_requested_;
  SocketState socket_state_;

  std::thread can_receive_thread_;

  /// @brief Store the filter list in case we want to be able to add, subtract in addition to overriding
  filter_vector_t filter_list_;
  can_err_mask_t error_mask_;
  bool join_;
};

}   // namespace socketcan
} // namespace polymath

#endif // SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_
