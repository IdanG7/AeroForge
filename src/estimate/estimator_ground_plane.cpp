#include "aeroforge/modules.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <stdexcept>

namespace af
{

void validate_intrinsics(const CameraIntrinsics& k, const char* who); // estimator_size_based.cpp

namespace
{

// Ground-plane intersection (pinhole model, no distortion).
//
// The pixel defines a ray d = (x_n, y_n, 1) in the camera frame (X right, Y down,
// Z forward). The camera sits `h` metres above a flat, level ground plane and its optical
// axis points `tilt` radians below the horizon, with no roll. The downward component of
// the ray is
//
//   down = y_n * cos(tilt) + sin(tilt)
//
// and the ray reaches the ground at scale t = h / down, giving the camera-frame point
// p = t * d. Rays with down <= 0 point at or above the horizon and never hit the ground.
//
// Assumptions: the target's contact point with the ground is at the detection centroid
// (use the bottom of the bounding box for tall objects), the ground is flat and level,
// telemetry altitude is height above that ground, and the vehicle is not rolled or
// pitched beyond the configured camera tilt.
class GroundPlaneEstimator : public IEstimator
{
public:
  GroundPlaneEstimator(const CameraIntrinsics& k, double camera_tilt_deg)
      : k_(k), tilt_rad_(camera_tilt_deg * 3.14159265358979323846 / 180.0)
  {
    spdlog::info("[GroundPlaneEstimator] fx={:.1f}, fy={:.1f}, tilt={:.1f} deg", k.fx, k.fy,
                 camera_tilt_deg);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                 const Telemetry& telemetry) override
  {
    if (telemetry.validity != Validity::Fresh || !std::isfinite(telemetry.altitude_m) ||
        telemetry.altitude_m < kMinAltitudeM)
    {
      return std::nullopt;
    }
    if (!std::isfinite(det.centroid.x) || !std::isfinite(det.centroid.y))
    {
      return std::nullopt;
    }

    const double x_norm = (det.centroid.x - k_.cx) / k_.fx;
    const double y_norm = (det.centroid.y - k_.cy) / k_.fy;

    const double down = y_norm * std::cos(tilt_rad_) + std::sin(tilt_rad_);
    if (down < kMinDownComponent)
    {
      return std::nullopt; // at or above the horizon, or too shallow to be reliable
    }

    const double t = telemetry.altitude_m / down;
    const Eigen::Vector3d p(x_norm * t, y_norm * t, t);
    if (!p.allFinite() || p.norm() > kMaxRangeM)
    {
      return std::nullopt;
    }

    Pose3D pose;
    pose.position_m = p;
    pose.timestamp_s = frame.timestamp_s;
    pose.validity = det.validity;
    return pose;
  }

  const char* name() const override { return "GroundPlaneEstimator"; }

private:
  static constexpr double kMinAltitudeM = 0.1;
  // tan(~1.1 deg): below this the range changes by metres per pixel
  static constexpr double kMinDownComponent = 0.02;
  static constexpr double kMaxRangeM = 500.0;

  CameraIntrinsics k_;
  double tilt_rad_;
};

} // namespace

std::unique_ptr<IEstimator> make_ground_plane_estimator(const CameraIntrinsics& intrinsics,
                                                        double camera_tilt_deg)
{
  validate_intrinsics(intrinsics, "ground_plane");
  if (!std::isfinite(camera_tilt_deg) || camera_tilt_deg < 0.0 || camera_tilt_deg > 90.0)
  {
    throw std::invalid_argument(
        "ground_plane: camera_tilt_deg must be between 0 (level) and 90 (straight down)");
  }
  return std::make_unique<GroundPlaneEstimator>(intrinsics, camera_tilt_deg);
}

} // namespace af
