#include "aeroforge/plugin.hpp"
#include <spdlog/spdlog.h>

namespace af
{

class SizeBasedEstimator : public IEstimator
{
public:
  SizeBasedEstimator(double object_diameter_m, double fx, double fy, double cx, double cy)
      : object_diameter_m_(object_diameter_m), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
    spdlog::info("[SizeBasedEstimator] object_diameter={:.2f}m, fx={:.1f}, fy={:.1f}",
                 object_diameter_m, fx, fy);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                 const Telemetry& telemetry) override
  {
    (void)telemetry; // not used in this simple estimator

    if (det.bbox.width <= 0 || det.bbox.height <= 0)
    {
      return std::nullopt;
    }

    // Average diameter in pixels
    double d_pixels = (det.bbox.width + det.bbox.height) / 2.0;

    // Range estimation: Z ≈ (fx * D) / d_pixels
    double range_m = (fx_ * object_diameter_m_) / d_pixels;

    // Bearing from image coordinates
    double u = det.centroid.x;
    double v = det.centroid.y;

    // Normalized image coordinates
    double x_norm = (u - cx_) / fx_;
    double y_norm = (v - cy_) / fy_;

    // 3D position in camera frame (X=right, Y=down, Z=forward)
    Pose3D pose;
    pose.position_m.x() = x_norm * range_m;
    pose.position_m.y() = y_norm * range_m;
    pose.position_m.z() = range_m;
    pose.timestamp_s = frame.timestamp_s;

    return pose;
  }

  const char* name() const override { return "SizeBasedEstimator"; }

private:
  double object_diameter_m_;
  double fx_, fy_, cx_, cy_;
};

} // namespace af
