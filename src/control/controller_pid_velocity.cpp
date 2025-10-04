#include "aeroforge/plugin.hpp"
#include "aeroforge/math.hpp"
#include <spdlog/spdlog.h>

namespace af
{

class PIDVelocityController : public IController
{
public:
  PIDVelocityController(const Eigen::Vector3d& desired_offset_m, const Eigen::Vector3d& kp,
                        const Eigen::Vector3d& ki, const Eigen::Vector3d& kd,
                        double max_speed_mps)
      : desired_offset_m_(desired_offset_m),
        pid_(kp, ki, kd, max_speed_mps),
        max_speed_mps_(max_speed_mps)
  {
    spdlog::info("[PIDVelocityController] desired_offset=[{:.1f}, {:.1f}, {:.1f}], "
                 "kp=[{:.2f}, {:.2f}, {:.2f}], max_speed={:.1f}",
                 desired_offset_m.x(), desired_offset_m.y(), desired_offset_m.z(), kp.x(),
                 kp.y(), kp.z(), max_speed_mps);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Eigen::Vector3d> velocity_cmd(const Pose3D& target_rel,
                                               const Telemetry& drone_state) override
  {
    (void)drone_state; // not used in this simple controller

    // Error: difference between target position and desired offset
    Eigen::Vector3d error = target_rel.position_m - desired_offset_m_;

    // Compute velocity command via PID
    double dt = 0.033; // ~30 Hz (should be actual dt in real system)
    Eigen::Vector3d cmd = pid_.compute(error, dt);

    // Clamp magnitude
    double mag = cmd.norm();
    if (mag > max_speed_mps_)
    {
      cmd = cmd / mag * max_speed_mps_;
    }

    spdlog::debug("[PIDVelocityController] error=[{:.2f}, {:.2f}, {:.2f}], cmd=[{:.2f}, "
                  "{:.2f}, {:.2f}]",
                  error.x(), error.y(), error.z(), cmd.x(), cmd.y(), cmd.z());

    return cmd;
  }

  const char* name() const override { return "PIDVelocityController"; }

private:
  Eigen::Vector3d desired_offset_m_;
  PIDController pid_;
  double max_speed_mps_;
};

} // namespace af
