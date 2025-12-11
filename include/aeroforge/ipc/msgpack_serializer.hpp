// Copyright (c) 2025 AeroForge
// SPDX-License-Identifier: MIT

#pragma once

#include "aeroforge/core.hpp"
#include <msgpack.hpp>
#include <vector>
#include <string>
#include <optional>

namespace af
{
namespace ipc
{

/**
 * @brief MessagePack serialization helpers for AeroForge<->Norfair protocol.
 */
class MessagePackSerializer
{
public:
  /**
   * @brief Serialize an update request.
   *
   * Creates a MessagePack-encoded request message with detections.
   *
   * @param dt Time delta since last update (seconds)
   * @param detections Detections to send
   * @return Serialized message bytes
   */
  static std::vector<uint8_t> serialize_update_request(double dt,
                                                       const std::vector<Detection>& detections);

  /**
   * @brief Deserialize an update response.
   *
   * Parses a MessagePack-encoded response message and extracts detections.
   *
   * @param data Serialized message bytes
   * @param detections_out Output vector for detections
   * @param error_msg_out Output string for error message (if any)
   * @return true if successful, false on error
   */
  static bool deserialize_update_response(const std::vector<uint8_t>& data,
                                         std::vector<Detection>& detections_out,
                                         std::string& error_msg_out);

private:
  // Protocol version
  static constexpr int PROTOCOL_VERSION = 1;
};

} // namespace ipc
} // namespace af
