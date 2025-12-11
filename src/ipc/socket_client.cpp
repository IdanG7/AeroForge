// Copyright (c) 2025 AeroForge
// SPDX-License-Identifier: MIT

#include "aeroforge/ipc/socket_client.hpp"
#include <spdlog/spdlog.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <algorithm>

namespace af
{
namespace ipc
{

SocketClient::SocketClient(const std::string& socket_path, int timeout_ms)
  : socket_path_(socket_path), timeout_ms_(timeout_ms), socket_fd_(-1), connected_(false)
{
}

SocketClient::~SocketClient()
{
  disconnect();
}

bool SocketClient::connect(int timeout_ms)
{
  if (connected_)
  {
    spdlog::debug("[SocketClient] Already connected to {}", socket_path_);
    return true;
  }

  // Create socket
  socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    spdlog::error("[SocketClient] Failed to create socket: {}", strerror(errno));
    return false;
  }

  // Set non-blocking for connect with timeout
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  // Setup socket address
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  // Attempt connection
  int result = ::connect(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));

  if (result < 0 && errno != EINPROGRESS)
  {
    spdlog::error("[SocketClient] Failed to connect to {}: {}", socket_path_, strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Wait for connection to complete (or timeout)
  if (result < 0)
  {
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLOUT;

    int poll_result = poll(&pfd, 1, timeout_ms);

    if (poll_result <= 0)
    {
      spdlog::error("[SocketClient] Connection timeout to {}", socket_path_);
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    // Check if connection succeeded
    int error = 0;
    socklen_t len = sizeof(error);
    getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len);

    if (error != 0)
    {
      spdlog::error("[SocketClient] Connection failed: {}", strerror(error));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
  }

  // Restore blocking mode
  fcntl(socket_fd_, F_SETFL, flags);

  connected_ = true;
  spdlog::info("[SocketClient] Connected to {}", socket_path_);
  return true;
}

void SocketClient::disconnect()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

bool SocketClient::send(const std::vector<uint8_t>& data)
{
  if (!connected_)
  {
    spdlog::error("[SocketClient] Cannot send: not connected");
    return false;
  }

  // Validate message size (max 1MB)
  if (data.size() > 1024 * 1024)
  {
    spdlog::error("[SocketClient] Message too large: {} bytes", data.size());
    return false;
  }

  // Send length prefix (4 bytes, big-endian)
  uint32_t length = static_cast<uint32_t>(data.size());
  uint8_t length_bytes[4] = {static_cast<uint8_t>((length >> 24) & 0xFF),
                              static_cast<uint8_t>((length >> 16) & 0xFF),
                              static_cast<uint8_t>((length >> 8) & 0xFF),
                              static_cast<uint8_t>(length & 0xFF)};

  ssize_t sent = ::send(socket_fd_, length_bytes, 4, 0);
  if (sent != 4)
  {
    spdlog::error("[SocketClient] Failed to send length prefix: {}", strerror(errno));
    disconnect();
    return false;
  }

  // Send data payload
  size_t total_sent = 0;
  while (total_sent < data.size())
  {
    sent = ::send(socket_fd_, data.data() + total_sent, data.size() - total_sent, 0);
    if (sent < 0)
    {
      spdlog::error("[SocketClient] Failed to send data: {}", strerror(errno));
      disconnect();
      return false;
    }
    total_sent += sent;
  }

  return true;
}

std::optional<std::vector<uint8_t>> SocketClient::receive()
{
  if (!connected_)
  {
    spdlog::error("[SocketClient] Cannot receive: not connected");
    return std::nullopt;
  }

  // Receive length prefix (4 bytes)
  uint8_t length_bytes[4];
  if (!recv_exact(length_bytes, 4))
  {
    return std::nullopt;
  }

  // Parse length (big-endian)
  uint32_t length = (static_cast<uint32_t>(length_bytes[0]) << 24) |
                    (static_cast<uint32_t>(length_bytes[1]) << 16) |
                    (static_cast<uint32_t>(length_bytes[2]) << 8) | (static_cast<uint32_t>(length_bytes[3]));

  // Validate length (max 1MB)
  if (length == 0 || length > 1024 * 1024)
  {
    spdlog::error("[SocketClient] Invalid message length: {}", length);
    disconnect();
    return std::nullopt;
  }

  // Receive data payload
  std::vector<uint8_t> data(length);
  if (!recv_exact(data.data(), length))
  {
    return std::nullopt;
  }

  return data;
}

std::optional<std::vector<uint8_t>> SocketClient::send_and_receive(const std::vector<uint8_t>& data)
{
  if (!send(data))
  {
    return std::nullopt;
  }

  return receive();
}

bool SocketClient::recv_exact(uint8_t* buffer, size_t n)
{
  size_t total_received = 0;

  while (total_received < n)
  {
    // Use poll to implement timeout
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    int poll_result = poll(&pfd, 1, timeout_ms_);

    if (poll_result < 0)
    {
      spdlog::error("[SocketClient] Poll error: {}", strerror(errno));
      disconnect();
      return false;
    }
    else if (poll_result == 0)
    {
      spdlog::warn("[SocketClient] Receive timeout after {}ms", timeout_ms_);
      disconnect();
      return false;
    }

    // Data available, receive it
    ssize_t received = recv(socket_fd_, buffer + total_received, n - total_received, 0);

    if (received < 0)
    {
      spdlog::error("[SocketClient] Receive error: {}", strerror(errno));
      disconnect();
      return false;
    }
    else if (received == 0)
    {
      // Connection closed by peer
      spdlog::warn("[SocketClient] Connection closed by peer");
      disconnect();
      return false;
    }

    total_received += received;
  }

  return true;
}

} // namespace ipc
} // namespace af
