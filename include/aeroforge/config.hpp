#pragma once

#include <Eigen/Core>
#include <optional>
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
  std::string type{"webcam"}; // webcam | file | stream
  int device{0};              // webcam index
  int width{1280};            // requested size (webcam); recorded sources use their own
  int height{720};
  int fps{30};                // requested rate (webcam) / fallback when a file reports none
  std::string path;           // file: video path; stream: URL
  std::optional<bool> mirror; // flip horizontally; default: true for webcam, else false
  bool loop{false};           // file only: restart at end of file
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
  double stale_timeout_s{0.5}; // a target older than this never drives control
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

// Loads and validates a pipeline config. Throws std::runtime_error whose message names
// the file, the offending key and the accepted values.
PipelineConfig load_config(const std::string& yaml_path);

// Same, from YAML text. `origin` is used in error messages.
PipelineConfig parse_config(const std::string& yaml_text, const std::string& origin);

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

// Numeric accessors that accept both integer and floating-point YAML values
// (`kp: [1, 1, 1]` and `kp: [1.0, 1.0, 1.0]` are equivalent). They return nullopt when
// the key is missing and throw std::runtime_error when it has the wrong type or length.
std::optional<double> get_number(const ConfigMap& map, const std::string& key);
std::optional<std::vector<double>> get_numbers(const ConfigMap& map, const std::string& key,
                                               size_t expected_size);

Eigen::Vector3d get_vec3(const ConfigMap& map, const std::string& key,
                         const Eigen::Vector3d& default_val);

} // namespace af
