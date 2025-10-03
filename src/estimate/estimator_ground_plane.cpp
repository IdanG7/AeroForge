#include "aeroforge/plugin.hpp"
#include <spdlog/spdlog.h>

namespace af
{

class GroundPlaneEstimator : public IEstimator
{
public:
  GroundPlaneEstimator(double fx, double fy, double cx, double cy, double camera_tilt_deg = 0.0)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), camera_tilt_deg_(camera_tilt_deg)
  {
    spdlog::info("[GroundPlaneEstimator] fx={:.1f}, fy={:.1f}, tilt={:.1f} deg", fx, fy,
                 camera_tilt_deg);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                 const Telemetry& telemetry) override
  {
    // Assumes target is on ground plane (z=0 in world frame)
    // Uses drone altitude and camera tilt to triangulate

    double altitude_m = telemetry.altitude_m;
    if (altitude_m <= 0.1)
    {
      return std::nullopt; // too low or invalid
    }

    // Normalized image coordinates
    double x_norm = (det.centroid.x - cx_) / fx_;
    double y_norm = (det.centroid.y - cy_) / fy_;

    // Simple pinhole projection (ignoring camera tilt for now)
    // For ground plane: altitude / (y_norm) ≈ range in forward direction
    double range_forward = altitude_m / std::abs(y_norm);

    // Clamp to reasonable values
    if (range_forward > 100.0 || range_forward < 0.5)
    {
      return std::nullopt;
    }

    Pose3D pose;
    pose.position_m.x() = x_norm * range_forward;
    pose.position_m.y() = 0.0; // ground plane
    pose.position_m.z() = range_forward;
    pose.timestamp_s = frame.timestamp_s;

    return pose;
  }

  const char* name() const override { return "GroundPlaneEstimator"; }

private:
  double fx_, fy_, cx_, cy_;
  double camera_tilt_deg_;
};

} // namespace af
