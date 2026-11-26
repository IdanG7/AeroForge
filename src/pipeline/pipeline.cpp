#include "aeroforge/pipeline.hpp"

#include "aeroforge/modules.hpp"
#include "aeroforge/pytracking_iface.hpp"

#include <cmath>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{
namespace
{

double required_number(const ModuleConfig& mod, const char* key)
{
  const auto v = get_number(mod.params, key);
  if (!v)
  {
    throw std::runtime_error("parameter '" + std::string(key) + "' is required");
  }
  return *v;
}

double number_or(const ModuleConfig& mod, const char* key, double fallback)
{
  return get_number(mod.params, key).value_or(fallback);
}

Eigen::Vector3d required_vec3(const ModuleConfig& mod, const char* key)
{
  const auto v = get_numbers(mod.params, key, 3);
  if (!v)
  {
    throw std::runtime_error("parameter '" + std::string(key) + "' is required");
  }
  return {(*v)[0], (*v)[1], (*v)[2]};
}

Eigen::Vector3d vec3_or(const ModuleConfig& mod, const char* key, const Eigen::Vector3d& fallback)
{
  const auto v = get_numbers(mod.params, key, 3);
  return v ? Eigen::Vector3d((*v)[0], (*v)[1], (*v)[2]) : fallback;
}

CameraIntrinsics intrinsics_from(const ModuleConfig& mod)
{
  CameraIntrinsics k;
  k.fx = required_number(mod, "fx");
  k.fy = required_number(mod, "fy");
  k.cx = required_number(mod, "cx");
  k.cy = required_number(mod, "cy");
  return k;
}

const char* kKnownTypes = "detectors: interactive, colorball, aruco, pytracking; "
                          "trackers: kalman, norfair; estimators: size_based, ground_plane; "
                          "controllers: pid_velocity";

} // namespace

struct Pipeline::Impl
{
  std::unique_ptr<IDetector> detector;
  ITargetTracker* target_tracker{nullptr}; // alias of detector when it is selectable
  std::unique_ptr<ITracker> tracker;
  std::unique_ptr<IEstimator> estimator;
  std::unique_ptr<IController> controller;

  bool have_last_frame{false};
  double last_frame_time_s{0.0};

  void set_detector(std::unique_ptr<IDetector> d, const ModuleConfig& mod)
  {
    if (detector)
    {
      throw std::runtime_error("more than one detector is configured ('" + mod.type + "' after '" +
                               detector->name() + "')");
    }
    detector = std::move(d);
  }

  void set_target_tracker(std::unique_ptr<ITargetTracker> t, const ModuleConfig& mod)
  {
    ITargetTracker* raw = t.get();
    set_detector(std::move(t), mod);
    target_tracker = raw;
  }

  void add_module(const ModuleConfig& mod)
  {
    const std::string& type = mod.type;
    if (type == "interactive" || type == "template")
    {
      set_target_tracker(make_template_tracker(get_param<std::string>(mod.params, "perf_csv", "")), mod);
    }
    else if (type == "pytracking")
    {
#ifdef _WIN32
      const char* default_python = "python";
#else
      const char* default_python = "python3";
#endif
      set_target_tracker(
          make_pytracking_detector(
              get_param<std::string>(mod.params, "wrapper_path", "python/pytracking_wrapper.py"),
              get_param<std::string>(mod.params, "tracker_name", "dimp"),
              get_param<std::string>(mod.params, "python_cmd", default_python)),
          mod);
    }
    else if (type == "colorball")
    {
      const Eigen::Vector3d lo = required_vec3(mod, "hsv_lo");
      const Eigen::Vector3d hi = required_vec3(mod, "hsv_hi");
      set_detector(make_colorball_detector(cv::Scalar(lo.x(), lo.y(), lo.z()),
                                           cv::Scalar(hi.x(), hi.y(), hi.z()),
                                           number_or(mod, "min_area_px", 200.0)),
                   mod);
    }
    else if (type == "aruco")
    {
      set_detector(make_aruco_detector(static_cast<int>(number_or(mod, "dict_id", 0.0))), mod);
    }
    else if (type == "kalman")
    {
      tracker = make_kalman_tracker(number_or(mod, "q", 0.02), number_or(mod, "r", 0.20));
    }
    else if (type == "norfair")
    {
      tracker = make_norfair_tracker(
          get_param<std::string>(mod.params, "socket_path", "/tmp/aeroforge_norfair.sock"),
          static_cast<int>(number_or(mod, "timeout_ms", 500.0)),
          get_param<bool>(mod.params, "auto_reconnect", true),
          number_or(mod, "reconnect_max_interval_s", 10.0));
    }
    else if (type == "size_based")
    {
      estimator = make_size_based_estimator(required_number(mod, "object_diameter_m"), intrinsics_from(mod));
    }
    else if (type == "ground_plane")
    {
      estimator = make_ground_plane_estimator(intrinsics_from(mod), number_or(mod, "camera_tilt_deg", 0.0));
    }
    else if (type == "pid_velocity")
    {
      controller = make_pid_velocity_controller(
          required_vec3(mod, "desired_offset_m"), required_vec3(mod, "kp"),
          vec3_or(mod, "ki", Eigen::Vector3d::Zero()), vec3_or(mod, "kd", Eigen::Vector3d::Zero()),
          required_number(mod, "max_speed_mps"));
    }
    else
    {
      throw std::runtime_error(std::string("unknown type (") + kKnownTypes + ")");
    }
  }
};

Pipeline::Pipeline(const PipelineConfig& config) : impl_(std::make_unique<Impl>())
{
  for (const auto& mod : config.modules)
  {
    try
    {
      impl_->add_module(mod);
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("pipeline module '" + mod.name + "' (type " + mod.type + "): " + e.what());
    }
  }

  if (!impl_->detector)
  {
    spdlog::info("No detector configured: using the interactive tracker (select a target)");
    auto t = make_template_tracker();
    impl_->target_tracker = t.get();
    impl_->detector = std::move(t);
  }

  spdlog::info("Pipeline: detector={}, tracker={}, estimator={}, controller={}", impl_->detector->name(),
               impl_->tracker ? impl_->tracker->name() : "none",
               impl_->estimator ? impl_->estimator->name() : "none",
               impl_->controller ? impl_->controller->name() : "none");
}

Pipeline::~Pipeline() = default;

PipelineOutput Pipeline::step(const Frame& frame, const Telemetry& telemetry, bool control_enabled)
{
  PipelineOutput out;
  if (frame.img.empty())
  {
    return out;
  }

  double dt = 0.0;
  if (impl_->have_last_frame)
  {
    dt = frame.timestamp_s - impl_->last_frame_time_s;
  }
  impl_->last_frame_time_s = frame.timestamp_s;
  impl_->have_last_frame = true;

  {
    ScopedTimer t(out.timings.detect_ms);
    out.detections = impl_->detector->detect(frame);
  }

  {
    ScopedTimer t(out.timings.track_ms);
    if (impl_->tracker && std::isfinite(dt) && dt > 0.0 && dt < 1.0)
    {
      out.tracked = impl_->tracker->update(dt, out.detections);
    }
    else
    {
      out.tracked = out.detections;
    }
  }

  {
    ScopedTimer t(out.timings.estimate_ms);
    if (impl_->estimator && !out.tracked.empty())
    {
      out.pose = impl_->estimator->estimate(out.tracked.front(), frame, telemetry);
      if (out.pose && !out.pose->position_m.allFinite())
      {
        out.pose->validity = Validity::Invalid;
      }
    }
  }

  {
    ScopedTimer t(out.timings.control_ms);
    if (impl_->controller)
    {
      if (control_enabled && out.pose && out.pose->validity == Validity::Fresh)
      {
        out.velocity_cmd_body = impl_->controller->velocity_cmd(*out.pose, telemetry);
      }
      else
      {
        impl_->controller->reset();
      }
    }
  }

  return out;
}

ITargetTracker* Pipeline::target_tracker()
{
  return impl_->target_tracker;
}

bool Pipeline::select_target(const cv::Rect& roi, const Frame& frame)
{
  if (!impl_->target_tracker)
  {
    return false;
  }
  if (impl_->tracker)
  {
    impl_->tracker->reset();
  }
  if (impl_->controller)
  {
    impl_->controller->reset();
  }
  return impl_->target_tracker->set_roi(roi, frame.img);
}

void Pipeline::reset()
{
  if (impl_->target_tracker)
  {
    impl_->target_tracker->reset();
  }
  if (impl_->tracker)
  {
    impl_->tracker->reset();
  }
  if (impl_->controller)
  {
    impl_->controller->reset();
  }
  impl_->have_last_frame = false;
}

const char* Pipeline::detector_name() const
{
  return impl_->detector->name();
}

bool Pipeline::has_tracker() const
{
  return impl_->tracker != nullptr;
}

bool Pipeline::has_estimator() const
{
  return impl_->estimator != nullptr;
}

bool Pipeline::has_controller() const
{
  return impl_->controller != nullptr;
}

} // namespace af
