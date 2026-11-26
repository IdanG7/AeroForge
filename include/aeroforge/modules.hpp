#pragma once

// Factories for the built-in pipeline modules. Each factory validates its parameters and
// throws std::invalid_argument with an actionable message when they are unusable.

#include "plugin.hpp"
#include <memory>
#include <string>

namespace af
{

// Pinhole camera intrinsics in pixels. No lens distortion model: detections are assumed to
// come from an undistorted (or low-distortion) image.
struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

// ── Detectors ─────────────────────────────────────────────────────────────

// Largest HSV-thresholded blob. Expects BGR input.
std::unique_ptr<IDetector> make_colorball_detector(const cv::Scalar& hsv_lo,
                                                   const cv::Scalar& hsv_hi,
                                                   double min_area_px);

// ArUco markers (cv::aruco predefined dictionary id, e.g. cv::aruco::DICT_4X4_50 == 0).
std::unique_ptr<IDetector> make_aruco_detector(int dict_id);

// Native CPU tracker for user-selected regions (up to 5 targets). If `perf_csv_path` is
// not empty, per-frame tracking quality is written to that CSV file.
std::unique_ptr<ITargetTracker> make_template_tracker(const std::string& perf_csv_path = {});

// ── Trackers ──────────────────────────────────────────────────────────────

// Constant-velocity Kalman smoothing of the first detection's centroid.
std::unique_ptr<ITracker> make_kalman_tracker(double q, double r);

// Multi-object association by the Norfair Python service (services/norfair) over a Unix
// domain socket. POSIX only: on Windows the connection always fails. While the service is
// unreachable the tracker passes detections through unchanged and, if `auto_reconnect`,
// retries with exponential backoff up to `reconnect_max_interval_s`. Each update blocks for
// up to `timeout_ms` waiting for the service.
std::unique_ptr<ITracker> make_norfair_tracker(const std::string& socket_path,
                                               int timeout_ms = 500, bool auto_reconnect = true,
                                               double reconnect_max_interval_s = 10.0);

// ── Estimators ────────────────────────────────────────────────────────────

// Range from apparent size. `object_diameter_m` is the real size of the tracked object and
// must be known; the estimate is only as good as that number.
std::unique_ptr<IEstimator> make_size_based_estimator(double object_diameter_m,
                                                      const CameraIntrinsics& intrinsics);

// Intersects the pixel ray with a flat ground plane using the telemetry altitude.
// `camera_tilt_deg` is the downward pitch of the optical axis below the horizon.
std::unique_ptr<IEstimator> make_ground_plane_estimator(const CameraIntrinsics& intrinsics,
                                                        double camera_tilt_deg);

// ── Controllers ───────────────────────────────────────────────────────────

// `desired_offset_m` is where the vehicle should sit relative to the target, in the body
// frame (X forward, Y right, Z down): [-3, 0, 0] means "3 m behind the target".
std::unique_ptr<IController> make_pid_velocity_controller(const Eigen::Vector3d& desired_offset_m,
                                                          const Eigen::Vector3d& kp,
                                                          const Eigen::Vector3d& ki,
                                                          const Eigen::Vector3d& kd,
                                                          double max_speed_mps);

} // namespace af
