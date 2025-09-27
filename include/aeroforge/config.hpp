#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// Config value types
// ──────────────────────────────────────────────────────────────────────────

using ConfigValue =
    std::variant<bool, int, double, std::string, std::vector<int>, std::vector<double>>;

using ConfigMap = std::unordered_map<std::string, ConfigValue>;

// ──────────────────────────────────────────────────────────────────────────
// Pipeline module config
// ──────────────────────────────────────────────────────────────────────────

struct ModuleConfig
{
  std::string name;
  std::string type;
  ConfigMap params;
};

// ──────────────────────────────────────────────────────────────────────────
// Input config
// ──────────────────────────────────────────────────────────────────────────

struct InputConfig
{
  std::string type; // webcam | file | dji
  int device{0};
  int width{1280};
  int height{720};
  int fps{30};
  std::string path; // for file input
};

// ──────────────────────────────────────────────────────────────────────────
// Sink config
// ──────────────────────────────────────────────────────────────────────────

struct SinkConfig
{
  std::string type; // hud | recorder
  ConfigMap params;
};

// ──────────────────────────────────────────────────────────────────────────
// Safety config
// ──────────────────────────────────────────────────────────────────────────

struct SafetyConfig
{
  bool require_hold_to_enable{true};
  std::string e_stop_key{"SPACE"};
  double max_speed_mps{6.0};
  double min_follow_dist_m{2.0};
  double max_follow_dist_m{10.0};
  double min_alt_m{3.0};
  double max_alt_m{60.0};
  double max_radius_m{120.0};
};

// ──────────────────────────────────────────────────────────────────────────
// Top-level pipeline config
// ──────────────────────────────────────────────────────────────────────────

struct PipelineConfig
{
  InputConfig input;
  std::vector<ModuleConfig> modules;
  std::vector<SinkConfig> sinks;
  SafetyConfig safety;
};

// ──────────────────────────────────────────────────────────────────────────
// Config loader
// ──────────────────────────────────────────────────────────────────────────

PipelineConfig load_config(const std::string& yaml_path);

// ──────────────────────────────────────────────────────────────────────────
// Helper accessors
// ──────────────────────────────────────────────────────────────────────────

template <typename T>
T get_param(const ConfigMap& map, const std::string& key, const T& default_val)
{
  auto it = map.find(key);
  if (it == map.end())
  {
    return default_val;
  }
  if (auto* val = std::get_if<T>(&it->second))
  {
    return *val;
  }
  return default_val;
}

Eigen::Vector3d get_vec3(const ConfigMap& map, const std::string& key,
                         const Eigen::Vector3d& default_val);

} // namespace af
