#pragma once

#include "core.hpp"
#include "telemetry.hpp"

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <vector>

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// Base Processor Interface
// ──────────────────────────────────────────────────────────────────────────

struct IProcessor
{
  virtual ~IProcessor() = default;
  virtual void process(Frame& frame) = 0;
  virtual const char* name() const = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Detector Interface
// ──────────────────────────────────────────────────────────────────────────

struct IDetector : IProcessor
{
  virtual std::vector<Detection> detect(const Frame& frame) = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Target tracker: a detector that follows a user-selected region
// ──────────────────────────────────────────────────────────────────────────

struct ITargetTracker : IDetector
{
  // Start tracking the region `roi` of `frame`. Returns false if the region is unusable.
  virtual bool set_roi(const cv::Rect& roi, const cv::Mat& frame) = 0;
  // Drop all targets. Safe to call when nothing is tracked.
  virtual void reset() = 0;
  virtual bool is_tracking() const = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Tracker Interface
// ──────────────────────────────────────────────────────────────────────────

struct ITracker : IProcessor
{
  virtual std::vector<Detection> update(double dt, const std::vector<Detection>& detections) = 0;
  // Forget all track state (new target selected, source restarted).
  virtual void reset() {}
};

// ──────────────────────────────────────────────────────────────────────────
// Pose/Range Estimation
// ──────────────────────────────────────────────────────────────────────────

// Target position relative to the camera, in the camera optical frame:
// X right, Y down, Z forward (along the optical axis). Metres.
struct Pose3D
{
  Eigen::Vector3d position_m{0, 0, 0};
  Eigen::Vector3d velocity_mps{0, 0, 0};
  double timestamp_s{0.0};
  Validity validity{Validity::Fresh};
};

struct IEstimator : IProcessor
{
  virtual std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                         const Telemetry& telemetry) = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Controller Interface
// ──────────────────────────────────────────────────────────────────────────

struct IController : IProcessor
{
  // Returns a velocity command in the body frame (X forward, Y right, Z down), m/s, or
  // nullopt when no valid command can be computed from `target_rel`.
  virtual std::optional<Eigen::Vector3d> velocity_cmd(const Pose3D& target_rel,
                                                      const Telemetry& drone_state) = 0;
  // Clear integrators and history. Call when control is disabled or the target changes.
  virtual void reset() = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Sink Interface (HUD, recorder, logger, etc.)
// ──────────────────────────────────────────────────────────────────────────

struct ISink : IProcessor
{
  virtual void consume(const Frame& frame) = 0;
};

} // namespace af
