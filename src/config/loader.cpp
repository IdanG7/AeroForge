#include "aeroforge/config.hpp"

#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace af
{
namespace
{

// Helper to convert YAML node to ConfigValue
ConfigValue yaml_to_config_value(const YAML::Node& node)
{
  if (node.IsScalar())
  {
    try
    {
      return node.as<bool>();
    }
    catch (...)
    {
    }

    try
    {
      return node.as<int>();
    }
    catch (...)
    {
    }

    try
    {
      return node.as<double>();
    }
    catch (...)
    {
    }

    return node.as<std::string>();
  }

  if (node.IsSequence())
  {
    try
    {
      return node.as<std::vector<int>>();
    }
    catch (...)
    {
    }

    try
    {
      return node.as<std::vector<double>>();
    }
    catch (...)
    {
    }
  }

  throw std::runtime_error("unsupported value (expected a scalar or a list of numbers)");
}

[[noreturn]] void fail(const std::string& origin, const std::string& what)
{
  throw std::runtime_error("Config '" + origin + "': " + what);
}

// Typed read with a message that names the key
template <typename T>
T read_key(const YAML::Node& parent, const char* key, const T& fallback, const std::string& origin,
           const std::string& path)
{
  const YAML::Node node = parent[key];
  if (!node.IsDefined() || node.IsNull())
  {
    return fallback;
  }
  try
  {
    return node.as<T>();
  }
  catch (const YAML::Exception&)
  {
    fail(origin, path + "." + key + " has the wrong type");
  }
}

ConfigMap read_params(const YAML::Node& params, const std::string& origin, const std::string& path)
{
  ConfigMap out;
  if (!params.IsDefined() || params.IsNull())
  {
    return out;
  }
  if (!params.IsMap())
  {
    fail(origin, path + ".params must be a mapping of name: value");
  }
  for (const auto& param : params)
  {
    const std::string key = param.first.as<std::string>();
    try
    {
      out[key] = yaml_to_config_value(param.second);
    }
    catch (const std::exception& e)
    {
      fail(origin, path + ".params." + key + ": " + e.what());
    }
  }
  return out;
}

void require(bool ok, const std::string& origin, const std::string& what)
{
  if (!ok)
  {
    fail(origin, what);
  }
}

PipelineConfig from_root(const YAML::Node& root, const std::string& origin)
{
  if (!root.IsMap() || !root["pipeline"].IsDefined() || !root["pipeline"].IsMap())
  {
    fail(origin, "missing top-level 'pipeline:' section");
  }
  const YAML::Node pipeline = root["pipeline"];
  PipelineConfig config;

  // ── input ──
  const YAML::Node input = pipeline["input"];
  if (input.IsDefined())
  {
    require(input.IsMap(), origin, "pipeline.input must be a mapping");
    config.input.type = read_key<std::string>(input, "type", "webcam", origin, "pipeline.input");
    config.input.device = read_key<int>(input, "device", 0, origin, "pipeline.input");
    config.input.path = read_key<std::string>(input, "path", "", origin, "pipeline.input");
    config.input.loop = read_key<bool>(input, "loop", false, origin, "pipeline.input");
    if (input["mirror"].IsDefined())
    {
      config.input.mirror = read_key<bool>(input, "mirror", false, origin, "pipeline.input");
    }

    const YAML::Node size = input["size"];
    if (size.IsDefined())
    {
      require(size.IsMap(), origin, "pipeline.input.size must be a mapping");
      config.input.width = read_key<int>(size, "width", 1280, origin, "pipeline.input.size");
      config.input.height = read_key<int>(size, "height", 720, origin, "pipeline.input.size");
      config.input.fps = read_key<int>(size, "fps", 30, origin, "pipeline.input.size");
    }
  }

  const auto& in = config.input;
  require(in.type == "webcam" || in.type == "file" || in.type == "stream", origin,
          "pipeline.input.type '" + in.type + "' is not supported (use webcam, file or stream)");
  require(in.type == "webcam" || !in.path.empty(), origin,
          "pipeline.input.path is required when pipeline.input.type is '" + in.type + "'");
  require(in.device >= 0, origin, "pipeline.input.device must be >= 0");
  require(in.width > 0 && in.height > 0 && in.fps > 0, origin,
          "pipeline.input.size width, height and fps must all be > 0");

  // ── modules ──
  const YAML::Node modules = pipeline["modules"];
  if (modules.IsDefined() && !modules.IsNull())
  {
    require(modules.IsSequence(), origin, "pipeline.modules must be a list");
    size_t index = 0;
    for (const auto& mod_node : modules)
    {
      const std::string path = "pipeline.modules[" + std::to_string(index++) + "]";
      require(mod_node.IsMap(), origin, path + " must be a mapping with 'type'");
      ModuleConfig mod;
      mod.type = read_key<std::string>(mod_node, "type", "", origin, path);
      require(!mod.type.empty(), origin, path + " is missing 'type'");
      mod.name = read_key<std::string>(mod_node, "name", mod.type, origin, path);
      mod.params = read_params(mod_node["params"], origin, path);
      config.modules.push_back(std::move(mod));
    }
  }

  // ── sinks ──
  const YAML::Node sinks = pipeline["sinks"];
  if (sinks.IsDefined() && !sinks.IsNull())
  {
    require(sinks.IsSequence(), origin, "pipeline.sinks must be a list");
    size_t index = 0;
    for (const auto& sink_node : sinks)
    {
      const std::string path = "pipeline.sinks[" + std::to_string(index++) + "]";
      require(sink_node.IsMap(), origin, path + " must be a mapping with 'type'");
      SinkConfig sink;
      sink.type = read_key<std::string>(sink_node, "type", "", origin, path);
      require(!sink.type.empty(), origin, path + " is missing 'type'");
      sink.params = read_params(sink_node["params"], origin, path);
      config.sinks.push_back(std::move(sink));
    }
  }

  // ── safety ──
  const YAML::Node safety = root["safety"];
  if (safety.IsDefined() && !safety.IsNull())
  {
    require(safety.IsMap(), origin, "safety must be a mapping");
    auto& s = config.safety;
    s.require_hold_to_enable = read_key<bool>(safety, "require_hold_to_enable", true, origin, "safety");
    s.e_stop_key = read_key<std::string>(safety, "e_stop_key", "SPACE", origin, "safety");
    s.stale_timeout_s = read_key<double>(safety, "stale_timeout_s", 0.5, origin, "safety");

    const YAML::Node limits = safety["limits"];
    if (limits.IsDefined())
    {
      require(limits.IsMap(), origin, "safety.limits must be a mapping");
      s.max_speed_mps = read_key<double>(limits, "max_speed_mps", 6.0, origin, "safety.limits");
      s.min_follow_dist_m = read_key<double>(limits, "min_follow_dist_m", 2.0, origin, "safety.limits");
      s.max_follow_dist_m = read_key<double>(limits, "max_follow_dist_m", 10.0, origin, "safety.limits");
    }

    const YAML::Node geo = safety["geofence"];
    if (geo.IsDefined())
    {
      require(geo.IsMap(), origin, "safety.geofence must be a mapping");
      s.min_alt_m = read_key<double>(geo, "min_alt_m", 3.0, origin, "safety.geofence");
      s.max_alt_m = read_key<double>(geo, "max_alt_m", 60.0, origin, "safety.geofence");
      s.max_radius_m = read_key<double>(geo, "max_radius_m", 120.0, origin, "safety.geofence");
    }
  }

  const auto& s = config.safety;
  require(std::isfinite(s.max_speed_mps) && s.max_speed_mps > 0.0, origin,
          "safety.limits.max_speed_mps must be > 0");
  require(std::isfinite(s.min_follow_dist_m) && std::isfinite(s.max_follow_dist_m) &&
              s.min_follow_dist_m >= 0.0 && s.min_follow_dist_m < s.max_follow_dist_m,
          origin, "safety.limits must satisfy 0 <= min_follow_dist_m < max_follow_dist_m");
  require(std::isfinite(s.min_alt_m) && std::isfinite(s.max_alt_m) && s.min_alt_m < s.max_alt_m, origin,
          "safety.geofence must satisfy min_alt_m < max_alt_m");
  require(std::isfinite(s.max_radius_m) && s.max_radius_m > 0.0, origin,
          "safety.geofence.max_radius_m must be > 0");
  require(std::isfinite(s.stale_timeout_s) && s.stale_timeout_s > 0.0, origin,
          "safety.stale_timeout_s must be > 0");

  return config;
}

} // namespace

PipelineConfig load_config(const std::string& yaml_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const YAML::BadFile&)
  {
    fail(yaml_path, "file not found or not readable");
  }
  catch (const YAML::Exception& e)
  {
    fail(yaml_path, std::string("not valid YAML: ") + e.what());
  }
  return from_root(root, yaml_path);
}

PipelineConfig parse_config(const std::string& yaml_text, const std::string& origin)
{
  YAML::Node root;
  try
  {
    root = YAML::Load(yaml_text);
  }
  catch (const YAML::Exception& e)
  {
    fail(origin, std::string("not valid YAML: ") + e.what());
  }
  return from_root(root, origin);
}

std::optional<double> get_number(const ConfigMap& map, const std::string& key)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    return std::nullopt;
  }
  if (auto* d = std::get_if<double>(&it->second))
  {
    return *d;
  }
  if (auto* i = std::get_if<int>(&it->second))
  {
    return static_cast<double>(*i);
  }
  throw std::runtime_error("parameter '" + key + "' must be a number");
}

std::optional<std::vector<double>> get_numbers(const ConfigMap& map, const std::string& key,
                                               size_t expected_size)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    return std::nullopt;
  }

  std::vector<double> out;
  if (auto* d = std::get_if<std::vector<double>>(&it->second))
  {
    out = *d;
  }
  else if (auto* i = std::get_if<std::vector<int>>(&it->second))
  {
    out.assign(i->begin(), i->end());
  }
  else
  {
    throw std::runtime_error("parameter '" + key + "' must be a list of numbers");
  }

  if (out.size() != expected_size)
  {
    throw std::runtime_error("parameter '" + key + "' must have exactly " + std::to_string(expected_size) +
                             " values");
  }
  return out;
}

Eigen::Vector3d get_vec3(const ConfigMap& map, const std::string& key, const Eigen::Vector3d& default_val)
{
  const auto values = get_numbers(map, key, 3);
  if (!values)
  {
    return default_val;
  }
  return Eigen::Vector3d((*values)[0], (*values)[1], (*values)[2]);
}

} // namespace af
