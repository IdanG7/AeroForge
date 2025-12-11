// Copyright (c) 2025 AeroForge
// SPDX-License-Identifier: MIT

#include "aeroforge/ipc/msgpack_serializer.hpp"
#include <spdlog/spdlog.h>

namespace af
{
namespace ipc
{

std::vector<uint8_t> MessagePackSerializer::serialize_update_request(
  double dt, const std::vector<Detection>& detections)
{
  // Build request map
  msgpack::sbuffer buffer;
  msgpack::packer<msgpack::sbuffer> packer(buffer);

  // Start map with 4 fields: version, cmd, dt, detections
  packer.pack_map(4);

  // version field
  packer.pack("version");
  packer.pack(PROTOCOL_VERSION);

  // cmd field
  packer.pack("cmd");
  packer.pack("update");

  // dt field
  packer.pack("dt");
  packer.pack(dt);

  // detections field
  packer.pack("detections");
  packer.pack_array(detections.size());

  for (const auto& det : detections)
  {
    // Each detection is a map with 4 fields: id, bbox, centroid, score
    packer.pack_map(4);

    // id
    packer.pack("id");
    packer.pack(det.id);

    // bbox [x, y, w, h]
    packer.pack("bbox");
    packer.pack_array(4);
    packer.pack(det.bbox.x);
    packer.pack(det.bbox.y);
    packer.pack(det.bbox.width);
    packer.pack(det.bbox.height);

    // centroid [cx, cy]
    packer.pack("centroid");
    packer.pack_array(2);
    packer.pack(det.centroid.x);
    packer.pack(det.centroid.y);

    // score
    packer.pack("score");
    packer.pack(det.score);
  }

  // Convert to vector
  std::vector<uint8_t> result(buffer.data(), buffer.data() + buffer.size());
  return result;
}

bool MessagePackSerializer::deserialize_update_response(const std::vector<uint8_t>& data,
                                                       std::vector<Detection>& detections_out,
                                                       std::string& error_msg_out)
{
  try
  {
    // Deserialize MessagePack
    msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<const char*>(data.data()), data.size());
    msgpack::object obj = oh.get();

    // Expect a map
    if (obj.type != msgpack::type::MAP)
    {
      error_msg_out = "Response is not a map";
      return false;
    }

    // Convert to map
    std::map<std::string, msgpack::object> response_map;
    obj.convert(response_map);

    // Check version (optional)
    if (response_map.count("version"))
    {
      int version = response_map["version"].as<int>();
      if (version != PROTOCOL_VERSION)
      {
        spdlog::warn("[MessagePack] Protocol version mismatch: got {}, expected {}", version,
                     PROTOCOL_VERSION);
      }
    }

    // Check success field
    if (!response_map.count("success"))
    {
      error_msg_out = "Missing 'success' field in response";
      return false;
    }

    bool success = response_map["success"].as<bool>();

    if (!success)
    {
      // Extract error message
      if (response_map.count("error"))
      {
        error_msg_out = response_map["error"].as<std::string>();
      }
      else
      {
        error_msg_out = "Unknown error";
      }
      return false;
    }

    // Extract detections
    if (!response_map.count("detections"))
    {
      error_msg_out = "Missing 'detections' field in response";
      return false;
    }

    msgpack::object detections_obj = response_map["detections"];

    if (detections_obj.type != msgpack::type::ARRAY)
    {
      error_msg_out = "Detections field is not an array";
      return false;
    }

    // Parse each detection
    detections_out.clear();
    msgpack::object_array detections_array = detections_obj.via.array;

    for (size_t i = 0; i < detections_array.size; ++i)
    {
      msgpack::object det_obj = detections_array.ptr[i];

      if (det_obj.type != msgpack::type::MAP)
      {
        error_msg_out = "Detection is not a map";
        return false;
      }

      std::map<std::string, msgpack::object> det_map;
      det_obj.convert(det_map);

      Detection det;

      // Parse id
      if (det_map.count("id"))
      {
        det.id = det_map["id"].as<int>();
      }

      // Parse bbox [x, y, w, h]
      if (det_map.count("bbox"))
      {
        std::vector<int> bbox = det_map["bbox"].as<std::vector<int>>();
        if (bbox.size() == 4)
        {
          det.bbox.x = bbox[0];
          det.bbox.y = bbox[1];
          det.bbox.width = bbox[2];
          det.bbox.height = bbox[3];
        }
      }

      // Parse centroid [cx, cy]
      if (det_map.count("centroid"))
      {
        std::vector<double> centroid = det_map["centroid"].as<std::vector<double>>();
        if (centroid.size() == 2)
        {
          det.centroid.x = static_cast<float>(centroid[0]);
          det.centroid.y = static_cast<float>(centroid[1]);
        }
      }

      // Parse score
      if (det_map.count("score"))
      {
        det.score = det_map["score"].as<float>();
      }

      detections_out.push_back(det);
    }

    return true;
  }
  catch (const std::exception& e)
  {
    error_msg_out = std::string("MessagePack parse error: ") + e.what();
    return false;
  }
}

} // namespace ipc
} // namespace af
