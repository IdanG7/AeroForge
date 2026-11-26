#include "aeroforge/math.hpp"
#include "aeroforge/modules.hpp"

#include <cmath>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{
namespace
{

// Follows a target by driving the body-frame position error to zero.
//
//   target_body = camera_to_body(target_rel)          (forward-looking, level camera)
//   error       = target_body + desired_offset        (desired_offset = vehicle relative
//                                                       to target, so the target should
//                                                       appear at -desired_offset)
//   cmd_body    = PID(error), limited to max_speed
//
// The time step comes from consecutive target timestamps, so the controller behaves the
// same at any frame rate and during replay.
class PIDVelocityController : public IController
{
public:
  PIDVelocityController(const Eigen::Vector3d& desired_offset_m, const Eigen::Vector3d& kp,
                        const Eigen::Vector3d& ki, const Eigen::Vector3d& kd, double max_speed_mps)
      : desired_offset_m_(desired_offset_m), pid_(kp, ki, kd, max_speed_mps), max_speed_mps_(max_speed_mps)
  {
    spdlog::info("[PIDVelocityController] desired_offset=[{:.1f}, {:.1f}, {:.1f}], "
                 "kp=[{:.2f}, {:.2f}, {:.2f}], max_speed={:.1f}",
                 desired_offset_m.x(), desired_offset_m.y(), desired_offset_m.z(), kp.x(), kp.y(), kp.z(),
                 max_speed_mps);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Eigen::Vector3d> velocity_cmd(const Pose3D& target_rel, const Telemetry& drone_state) override
  {
    (void)drone_state;

    if (target_rel.validity != Validity::Fresh || !target_rel.position_m.allFinite() ||
        !std::isfinite(target_rel.timestamp_s))
    {
      return std::nullopt;
    }

    double dt = 0.0; // 0 => proportional-only step (see PIDController::compute)
    if (have_last_)
    {
      dt = target_rel.timestamp_s - last_timestamp_s_;
      if (dt <= 0.0 || dt > kMaxDtS)
      {
        // Time went backwards or there was a gap: history is meaningless
        pid_.reset();
        dt = 0.0;
      }
    }
    last_timestamp_s_ = target_rel.timestamp_s;
    have_last_ = true;

    const Eigen::Vector3d error = camera_to_body(target_rel.position_m) + desired_offset_m_;
    Eigen::Vector3d cmd = pid_.compute(error, dt);

    const double mag = cmd.norm();
    if (mag > max_speed_mps_)
    {
      cmd *= max_speed_mps_ / mag;
    }

    return cmd;
  }

  void reset() override
  {
    pid_.reset();
    have_last_ = false;
  }

  const char* name() const override { return "PIDVelocityController"; }

private:
  static constexpr double kMaxDtS = 1.0;

  Eigen::Vector3d desired_offset_m_;
  PIDController pid_;
  double max_speed_mps_;
  double last_timestamp_s_{0.0};
  bool have_last_{false};
};

} // namespace

std::unique_ptr<IController> make_pid_velocity_controller(const Eigen::Vector3d& desired_offset_m,
                                                          const Eigen::Vector3d& kp,
                                                          const Eigen::Vector3d& ki,
                                                          const Eigen::Vector3d& kd, double max_speed_mps)
{
  if (!desired_offset_m.allFinite() || !kp.allFinite() || !ki.allFinite() || !kd.allFinite())
  {
    throw std::invalid_argument("pid_velocity: desired_offset_m, kp, ki and kd must be finite");
  }
  if ((kp.array() < 0.0).any() || (ki.array() < 0.0).any() || (kd.array() < 0.0).any())
  {
    throw std::invalid_argument("pid_velocity: gains kp, ki and kd must be >= 0");
  }
  if (!std::isfinite(max_speed_mps) || max_speed_mps <= 0.0)
  {
    throw std::invalid_argument("pid_velocity: max_speed_mps must be > 0");
  }
  return std::make_unique<PIDVelocityController>(desired_offset_m, kp, ki, kd, max_speed_mps);
}

} // namespace af
