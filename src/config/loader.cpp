#include "aeroforge/config.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace af
{

// Helper to convert YAML node to ConfigValue
static ConfigValue yaml_to_config_value(const YAML::Node& node)
{
  if (node.IsScalar())
  {
    // Try bool
    try
    {
      return node.as<bool>();
    }
    catch (...)
    {
    }

    // Try int
    try
    {
      return node.as<int>();
    }
    catch (...)
    {
    }

    // Try double
    try
    {
      return node.as<double>();
    }
    catch (...)
    {
    }

    // Fallback to string
    return node.as<std::string>();
  }

  if (node.IsSequence())
  {
    // Try vector<int>
    try
    {
      return node.as<std::vector<int>>();
    }
    catch (...)
    {
    }

    // Try vector<double>
    try
    {
      return node.as<std::vector<double>>();
    }
    catch (...)
    {
    }
  }

  throw std::runtime_error("Unsupported YAML node type");
}

PipelineConfig load_config(const std::string& yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  PipelineConfig config;

  // Parse input
  if (root["pipeline"]["input"])
  {
    auto input = root["pipeline"]["input"];
    config.input.type = input["type"].as<std::string>("webcam");
    config.input.device = input["device"].as<int>(0);

    if (input["size"])
    {
      config.input.width = input["size"]["width"].as<int>(1280);
      config.input.height = input["size"]["height"].as<int>(720);
      config.input.fps = input["size"]["fps"].as<int>(30);
    }

    if (input["path"])
    {
      config.input.path = input["path"].as<std::string>();
    }
  }

  // Parse modules
  if (root["pipeline"]["modules"])
  {
    for (const auto& mod_node : root["pipeline"]["modules"])
    {
      ModuleConfig mod;
      mod.name = mod_node["name"].as<std::string>();
      mod.type = mod_node["type"].as<std::string>();

      if (mod_node["params"])
      {
        for (const auto& param : mod_node["params"])
        {
          std::string key = param.first.as<std::string>();
          mod.params[key] = yaml_to_config_value(param.second);
        }
      }

      config.modules.push_back(mod);
    }
  }

  // Parse sinks
  if (root["pipeline"]["sinks"])
  {
    for (const auto& sink_node : root["pipeline"]["sinks"])
    {
      SinkConfig sink;
      sink.type = sink_node["type"].as<std::string>();

      if (sink_node["params"])
      {
        for (const auto& param : sink_node["params"])
        {
          std::string key = param.first.as<std::string>();
          sink.params[key] = yaml_to_config_value(param.second);
        }
      }

      config.sinks.push_back(sink);
    }
  }

  // Parse safety
  if (root["safety"])
  {
    auto safety = root["safety"];
    config.safety.require_hold_to_enable =
        safety["require_hold_to_enable"].as<bool>(true);
    config.safety.e_stop_key = safety["e_stop_key"].as<std::string>("SPACE");

    if (safety["limits"])
    {
      auto limits = safety["limits"];
      config.safety.max_speed_mps = limits["max_speed_mps"].as<double>(6.0);
      config.safety.min_follow_dist_m = limits["min_follow_dist_m"].as<double>(2.0);
      config.safety.max_follow_dist_m = limits["max_follow_dist_m"].as<double>(10.0);
    }

    if (safety["geofence"])
    {
      auto geo = safety["geofence"];
      config.safety.min_alt_m = geo["min_alt_m"].as<double>(3.0);
      config.safety.max_alt_m = geo["max_alt_m"].as<double>(60.0);
      config.safety.max_radius_m = geo["max_radius_m"].as<double>(120.0);
    }
  }

  return config;
}

Eigen::Vector3d get_vec3(const ConfigMap& map, const std::string& key,
                         const Eigen::Vector3d& default_val)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    return default_val;
  }

  if (auto* vec = std::get_if<std::vector<double>>(&it->second))
  {
    if (vec->size() >= 3)
    {
      return Eigen::Vector3d((*vec)[0], (*vec)[1], (*vec)[2]);
    }
  }

  return default_val;
}

} // namespace af
