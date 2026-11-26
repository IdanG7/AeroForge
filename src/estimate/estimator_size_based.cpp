#include "aeroforge/modules.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <stdexcept>

namespace af
{
namespace
{

// Monocular range from apparent size (pinhole model, no distortion):
//
//   Z = f * D / d
//
// D = real object diameter (m), d = apparent diameter (px), f = focal length (px).
// Assumptions: the object's real size is known, it is roughly as wide as it is tall, the
// bounding box is tight, and the object is near the optical axis (off-axis objects are
// measured along Z, not along the ray). Range error is proportional to the error in D and
// to the relative error of the bounding box size.
class SizeBasedEstimator : public IEstimator
{
public:
  SizeBasedEstimator(double object_diameter_m, const CameraIntrinsics& k)
      : object_diameter_m_(object_diameter_m), k_(k)
  {
    spdlog::info("[SizeBasedEstimator] object_diameter={:.2f}m, fx={:.1f}, fy={:.1f}",
                 object_diameter_m, k.fx, k.fy);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                 const Telemetry& telemetry) override
  {
    (void)telemetry;

    if (det.bbox.width <= 0 || det.bbox.height <= 0 || !std::isfinite(det.centroid.x) ||
        !std::isfinite(det.centroid.y))
    {
      return std::nullopt;
    }

    // Apparent size per axis, converted with the matching focal length
    const double range_from_w = (k_.fx * object_diameter_m_) / det.bbox.width;
    const double range_from_h = (k_.fy * object_diameter_m_) / det.bbox.height;
    const double range_m = 0.5 * (range_from_w + range_from_h);
    if (!std::isfinite(range_m) || range_m <= 0.0)
    {
      return std::nullopt;
    }

    const double x_norm = (det.centroid.x - k_.cx) / k_.fx;
    const double y_norm = (det.centroid.y - k_.cy) / k_.fy;

    Pose3D pose;
    pose.position_m.x() = x_norm * range_m;
    pose.position_m.y() = y_norm * range_m;
    pose.position_m.z() = range_m;
    pose.timestamp_s = frame.timestamp_s;
    pose.validity = det.validity;

    return pose;
  }

  const char* name() const override { return "SizeBasedEstimator"; }

private:
  double object_diameter_m_;
  CameraIntrinsics k_;
};

} // namespace

void validate_intrinsics(const CameraIntrinsics& k, const char* who)
{
  if (!std::isfinite(k.fx) || !std::isfinite(k.fy) || !std::isfinite(k.cx) ||
      !std::isfinite(k.cy) || k.fx <= 0.0 || k.fy <= 0.0)
  {
    throw std::invalid_argument(std::string(who) +
                                ": camera intrinsics fx and fy must be > 0 and finite");
  }
}

std::unique_ptr<IEstimator> make_size_based_estimator(double object_diameter_m,
                                                      const CameraIntrinsics& intrinsics)
{
  if (!std::isfinite(object_diameter_m) || object_diameter_m <= 0.0)
  {
    throw std::invalid_argument(
        "size_based: object_diameter_m must be set to the real size of the tracked object "
        "(> 0). Range cannot be estimated from size when the size is unknown.");
  }
  validate_intrinsics(intrinsics, "size_based");
  return std::make_unique<SizeBasedEstimator>(object_diameter_m, intrinsics);
}

} // namespace af
