// Copyright (c) 2025 AeroForge
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>
#include <optional>
#include <chrono>

namespace af
{
namespace ipc
{

/**
 * @brief Unix domain socket client with timeout and reconnection support.
 *
 * Provides a simple interface for connecting to a Unix socket server,
 * sending/receiving data with timeouts, and automatic reconnection on failure.
 */
class SocketClient
{
public:
  /**
   * @brief Construct a socket client.
   *
   * @param socket_path Path to the Unix domain socket
   * @param timeout_ms Default timeout for send/receive operations (milliseconds)
   */
  SocketClient(const std::string& socket_path, int timeout_ms = 500);

  /**
   * @brief Destructor - closes connection if open.
   */
  ~SocketClient();

  // Non-copyable
  SocketClient(const SocketClient&) = delete;
  SocketClient& operator=(const SocketClient&) = delete;

  /**
   * @brief Connect to the socket server.
   *
   * @param timeout_ms Connection timeout in milliseconds
   * @return true if connected successfully, false otherwise
   */
  bool connect(int timeout_ms = 5000);

  /**
   * @brief Disconnect from the server.
   */
  void disconnect();

  /**
   * @brief Check if currently connected.
   *
   * @return true if connected, false otherwise
   */
  bool is_connected() const { return connected_; }

  /**
   * @brief Send data to the server.
   *
   * Sends a 4-byte length prefix followed by the data.
   *
   * @param data Data to send
   * @return true if sent successfully, false on error
   */
  bool send(const std::vector<uint8_t>& data);

  /**
   * @brief Receive data from the server.
   *
   * Reads a 4-byte length prefix, then reads that many bytes of data.
   * Uses the configured timeout.
   *
   * @return Data received, or empty optional on error/timeout
   */
  std::optional<std::vector<uint8_t>> receive();

  /**
   * @brief Send data and receive response.
   *
   * Convenience method that sends data and waits for a response.
   *
   * @param data Data to send
   * @return Response data, or empty optional on error/timeout
   */
  std::optional<std::vector<uint8_t>> send_and_receive(const std::vector<uint8_t>& data);

  /**
   * @brief Get the socket path.
   */
  const std::string& socket_path() const { return socket_path_; }

  /**
   * @brief Get the configured timeout.
   */
  int timeout_ms() const { return timeout_ms_; }

  /**
   * @brief Set the timeout for send/receive operations.
   *
   * @param timeout_ms Timeout in milliseconds
   */
  void set_timeout(int timeout_ms) { timeout_ms_ = timeout_ms; }

private:
  /**
   * @brief Receive exactly n bytes from the socket.
   *
   * @param buffer Buffer to receive into
   * @param n Number of bytes to receive
   * @return true if received successfully, false on error/timeout
   */
  bool recv_exact(uint8_t* buffer, size_t n);

  std::string socket_path_;
  int timeout_ms_;
  int socket_fd_;
  bool connected_;
};

} // namespace ipc
} // namespace af
