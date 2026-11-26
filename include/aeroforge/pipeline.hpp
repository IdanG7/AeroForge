#pragma once

#include "config.hpp"
#include "plugin.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace af
{

struct StageTimings
{
  double detect_ms{0.0};
  double track_ms{0.0};
  double estimate_ms{0.0};
  double control_ms{0.0};

  double total_ms() const { return detect_ms + track_ms + estimate_ms + control_ms; }
};

struct PipelineOutput
{
  std::vector<Detection> detections; // raw detector output
  std::vector<Detection> tracked;    // after the tracker stage (same as detections if none)
  std::optional<Pose3D> pose;        // camera frame; check pose->validity
  // Unlimited controller output in the body frame. It must go through SafetyGate before
  // it is sent anywhere.
  std::optional<Eigen::Vector3d> velocity_cmd_body;
  StageTimings timings;
};

// Synchronous detect -> track -> estimate -> control chain built from a PipelineConfig.
//
// It runs on the caller's thread: at the measured per-frame cost of the built-in modules
// a second thread would add latency and hand-off cost without raising throughput. The
// exception is the PyTracking detector, which talks to another process and can block up
// to its timeout.
class Pipeline
{
public:
  // Throws std::runtime_error naming the module and parameter when the config is unusable.
  explicit Pipeline(const PipelineConfig& config);
  ~Pipeline();

  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;

  // Process one frame. `control_enabled` false skips the controller and clears its
  // integrators, so control always restarts from a clean state.
  PipelineOutput step(const Frame& frame, const Telemetry& telemetry, bool control_enabled);

  // Non-null when the configured detector follows user-selected regions.
  ITargetTracker* target_tracker();

  // Start following `roi` of `frame`. Clears filter and controller history first. Returns
  // false if the detector is not selectable or the region is unusable.
  bool select_target(const cv::Rect& roi, const Frame& frame);

  // Drop targets and all filter/controller history.
  void reset();

  const char* detector_name() const;
  bool has_tracker() const;
  bool has_estimator() const;
  bool has_controller() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace af
