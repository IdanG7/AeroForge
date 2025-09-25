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
// Tracker Interface
// ──────────────────────────────────────────────────────────────────────────

struct ITracker : IProcessor
{
  virtual std::vector<Detection> update(double dt, const std::vector<Detection>& detections) = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Pose/Range Estimation
// ──────────────────────────────────────────────────────────────────────────

struct Pose3D
{
  Eigen::Vector3d position_m{0, 0, 0};
  Eigen::Vector3d velocity_mps{0, 0, 0};
  double timestamp_s{0.0};
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
  virtual std::optional<Eigen::Vector3d> velocity_cmd(const Pose3D& target_rel,
                                                       const Telemetry& drone_state) = 0;
};

// ──────────────────────────────────────────────────────────────────────────
// Sink Interface (HUD, recorder, logger, etc.)
// ──────────────────────────────────────────────────────────────────────────

struct ISink : IProcessor
{
  virtual void consume(const Frame& frame) = 0;
};

} // namespace af
