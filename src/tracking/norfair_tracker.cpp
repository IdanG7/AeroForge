// Copyright (c) 2025 AeroForge
// SPDX-License-Identifier: MIT

#include "aeroforge/plugin.hpp"
#include "aeroforge/ipc/socket_client.hpp"
#include "aeroforge/ipc/msgpack_serializer.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>

namespace af
{

/**
 * @brief Multi-object tracker using Norfair Python service via IPC.
 *
 * This tracker communicates with a separate Python process running the Norfair
 * tracking library. It provides robust multi-object tracking with configurable
 * parameters and graceful fallback behavior when the service is unavailable.
 *
 * Features:
 * - Unix domain socket communication with MessagePack serialization
 * - Automatic reconnection with exponential backoff
 * - Graceful fallback to pass-through mode on errors
 * - Configurable timeout and reconnection parameters
 */
class NorfairTracker : public ITracker
{
public:
  /**
   * @brief Construct a Norfair tracker.
   *
   * @param socket_path Path to the Unix domain socket
   * @param timeout_ms Timeout for socket operations (milliseconds)
   * @param auto_reconnect Enable automatic reconnection on failure
   * @param reconnect_max_interval_s Maximum reconnection interval (seconds)
   */
  NorfairTracker(const std::string& socket_path, int timeout_ms = 500,
                bool auto_reconnect = true, double reconnect_max_interval_s = 10.0)
    : socket_path_(socket_path),
      timeout_ms_(timeout_ms),
      auto_reconnect_(auto_reconnect),
      reconnect_max_interval_s_(reconnect_max_interval_s),
      client_(std::make_unique<ipc::SocketClient>(socket_path, timeout_ms)),
      reconnect_interval_s_(1.0),
      last_reconnect_attempt_(std::chrono::steady_clock::now()),
      fallback_mode_(false)
  {
    spdlog::info("[NorfairTracker] Initialized with socket: {}", socket_path_);
    spdlog::info("[NorfairTracker] Timeout: {}ms, Auto-reconnect: {}", timeout_ms_, auto_reconnect_);

    // Attempt initial connection
    if (!client_->connect())
    {
      spdlog::warn("[NorfairTracker] Failed to connect on initialization, entering fallback mode");
      fallback_mode_ = true;
    }
  }

  void process(Frame& frame) override { (void)frame; }

  std::vector<Detection> update(double dt, const std::vector<Detection>& detections) override
  {
    // If no detections, return empty
    if (detections.empty())
    {
      return {};
    }

    // Check if we should attempt reconnection
    if (fallback_mode_ && auto_reconnect_)
    {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration<double>(now - last_reconnect_attempt_).count();

      if (elapsed >= reconnect_interval_s_)
      {
        spdlog::info("[NorfairTracker] Attempting reconnection...");
        last_reconnect_attempt_ = now;

        if (client_->connect())
        {
          spdlog::info("[NorfairTracker] Reconnected successfully");
          fallback_mode_ = false;
          reconnect_interval_s_ = 1.0; // Reset backoff
        }
        else
        {
          // Exponential backoff
          reconnect_interval_s_ = std::min(reconnect_interval_s_ * 2.0, reconnect_max_interval_s_);
          spdlog::warn("[NorfairTracker] Reconnection failed, next attempt in {:.1f}s",
                      reconnect_interval_s_);
        }
      }
    }

    // If in fallback mode, return input detections unchanged
    if (fallback_mode_)
    {
      return detections;
    }

    // Serialize request
    auto request_data = ipc::MessagePackSerializer::serialize_update_request(dt, detections);

    // Send request and receive response
    auto response_data = client_->send_and_receive(request_data);

    if (!response_data.has_value())
    {
      spdlog::warn("[NorfairTracker] Failed to communicate with service, entering fallback mode");
      fallback_mode_ = true;
      return detections;
    }

    // Deserialize response
    std::vector<Detection> tracked_detections;
    std::string error_msg;

    if (!ipc::MessagePackSerializer::deserialize_update_response(response_data.value(),
                                                                tracked_detections, error_msg))
    {
      spdlog::error("[NorfairTracker] Failed to parse response: {}", error_msg);
      fallback_mode_ = true;
      return detections;
    }

    spdlog::debug("[NorfairTracker] Tracked {} objects from {} detections", tracked_detections.size(),
                 detections.size());

    return tracked_detections;
  }

  const char* name() const override { return "NorfairTracker"; }

private:
  std::string socket_path_;
  int timeout_ms_;
  bool auto_reconnect_;
  double reconnect_max_interval_s_;

  std::unique_ptr<ipc::SocketClient> client_;

  // Reconnection state
  double reconnect_interval_s_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_;

  // Fallback mode (pass-through when service unavailable)
  bool fallback_mode_;
};

} // namespace af
