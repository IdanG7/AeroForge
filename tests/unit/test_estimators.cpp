#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include "aeroforge/math.hpp"
#include "aeroforge/modules.hpp"
#include <cmath>
#include <limits>

using Catch::Approx;

namespace
{

const af::CameraIntrinsics kCam{900.0, 900.0, 640.0, 360.0};

af::Detection detection_at(double u, double v, int w, int h)
{
  af::Detection det;
  det.centroid = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
  det.bbox = cv::Rect(static_cast<int>(u - w / 2.0), static_cast<int>(v - h / 2.0), w, h);
  return det;
}

af::Telemetry telemetry_at_altitude(double altitude_m)
{
  af::Telemetry tel;
  tel.validity = af::Validity::Fresh;
  tel.altitude_m = altitude_m;
  return tel;
}

} // namespace

TEST_CASE("Coordinate frame helpers", "[math][frames]")
{
  // A point 5 m ahead, 1 m right, 2 m below the camera
  const Eigen::Vector3d cam(1.0, 2.0, 5.0);
  const Eigen::Vector3d body = af::camera_to_body(cam);
  REQUIRE(body.x() == Approx(5.0)); // forward
  REQUIRE(body.y() == Approx(1.0)); // right
  REQUIRE(body.z() == Approx(2.0)); // down

  // Heading east: body-forward is NED-east
  const Eigen::Vector3d ned = af::body_to_ned(Eigen::Vector3d(1.0, 0.0, 0.5), 90.0);
  REQUIRE(ned.x() == Approx(0.0).margin(1e-12));
  REQUIRE(ned.y() == Approx(1.0));
  REQUIRE(ned.z() == Approx(0.5));

  // Rotation preserves length
  const Eigen::Vector3d v(1.5, -2.0, 0.3);
  REQUIRE(af::body_to_ned(v, 37.0).norm() == Approx(v.norm()));
}

TEST_CASE("Size-based estimator recovers synthetic ground truth", "[estimate]")
{
  auto est = af::make_size_based_estimator(0.20, kCam);
  af::Frame frame;
  frame.timestamp_s = 12.5;

  // 0.2 m object, 3 m away, 0.5 m right and 0.25 m below the optical axis
  const double z = 3.0, x = 0.5, y = 0.25;
  const int size_px = static_cast<int>(std::lround(kCam.fx * 0.20 / z)); // 60 px
  const auto det = detection_at(kCam.cx + kCam.fx * x / z, kCam.cy + kCam.fy * y / z, size_px,
                                size_px);

  const auto pose = est->estimate(det, frame, af::Telemetry{});
  REQUIRE(pose.has_value());
  REQUIRE(pose->position_m.z() == Approx(z).epsilon(0.01));
  REQUIRE(pose->position_m.x() == Approx(x).epsilon(0.01));
  REQUIRE(pose->position_m.y() == Approx(y).epsilon(0.01));
  REQUIRE(pose->timestamp_s == Approx(12.5));
  REQUIRE(pose->validity == af::Validity::Fresh);

  SECTION("Predicted detections give predicted poses")
  {
    af::Detection predicted = det;
    predicted.validity = af::Validity::Predicted;
    const auto p = est->estimate(predicted, frame, af::Telemetry{});
    REQUIRE(p.has_value());
    REQUIRE(p->validity == af::Validity::Predicted);
  }

  SECTION("Degenerate boxes are rejected")
  {
    REQUIRE_FALSE(est->estimate(detection_at(640, 360, 0, 20), frame, af::Telemetry{}));
    REQUIRE_FALSE(est->estimate(detection_at(640, 360, 20, -1), frame, af::Telemetry{}));
  }
}

TEST_CASE("Size-based estimator refuses an unknown object size", "[estimate]")
{
  REQUIRE_THROWS_AS(af::make_size_based_estimator(0.0, kCam), std::invalid_argument);
  REQUIRE_THROWS_AS(af::make_size_based_estimator(-1.0, kCam), std::invalid_argument);
  REQUIRE_THROWS_AS(
      af::make_size_based_estimator(std::numeric_limits<double>::quiet_NaN(), kCam),
      std::invalid_argument);
  REQUIRE_THROWS_AS(af::make_size_based_estimator(0.2, af::CameraIntrinsics{0, 900, 640, 360}),
                    std::invalid_argument);
}

TEST_CASE("Ground-plane estimator recovers synthetic ground truth", "[estimate]")
{
  const double tilt_deg = 30.0;
  const double tilt = tilt_deg * 3.14159265358979323846 / 180.0;
  auto est = af::make_ground_plane_estimator(kCam, tilt_deg);
  af::Frame frame;

  // Ground point 15 m ahead and 2 m to the right of a camera 10 m up
  const double h = 10.0, forward = 15.0, lateral = 2.0;
  // Level frame (right, down, forward) rotated into the camera frame pitched down by tilt
  const Eigen::Vector3d cam(lateral, -forward * std::sin(tilt) + h * std::cos(tilt),
                            forward * std::cos(tilt) + h * std::sin(tilt));
  const auto det = detection_at(kCam.cx + kCam.fx * cam.x() / cam.z(),
                                kCam.cy + kCam.fy * cam.y() / cam.z(), 20, 20);

  const auto pose = est->estimate(det, frame, telemetry_at_altitude(h));
  REQUIRE(pose.has_value());
  REQUIRE(pose->position_m.x() == Approx(cam.x()).epsilon(0.01));
  REQUIRE(pose->position_m.y() == Approx(cam.y()).epsilon(0.01));
  REQUIRE(pose->position_m.z() == Approx(cam.z()).epsilon(0.01));

  SECTION("Rays at or above the horizon never produce a position")
  {
    // With a 30 deg down tilt the horizon is at y_n = -tan(30 deg)
    const double v_horizon = kCam.cy - kCam.fy * std::tan(tilt);
    REQUIRE_FALSE(est->estimate(detection_at(640, v_horizon, 20, 20), frame,
                                telemetry_at_altitude(h)));
    REQUIRE_FALSE(est->estimate(detection_at(640, v_horizon - 100, 20, 20), frame,
                                telemetry_at_altitude(h)));
  }

  SECTION("Telemetry that is not fresh is never used")
  {
    af::Telemetry stale = telemetry_at_altitude(h);
    stale.validity = af::Validity::Stale;
    REQUIRE_FALSE(est->estimate(det, frame, stale));
    REQUIRE_FALSE(est->estimate(det, frame, af::Telemetry{})); // unavailable
  }

  SECTION("Invalid altitude is rejected")
  {
    REQUIRE_FALSE(est->estimate(det, frame, telemetry_at_altitude(0.0)));
    REQUIRE_FALSE(est->estimate(det, frame, telemetry_at_altitude(-5.0)));
    REQUIRE_FALSE(est->estimate(
        det, frame, telemetry_at_altitude(std::numeric_limits<double>::infinity())));
  }
}

TEST_CASE("Ground-plane estimator with a level camera", "[estimate]")
{
  auto est = af::make_ground_plane_estimator(kCam, 0.0);
  af::Frame frame;
  // 10 m up, looking level: a ground point 20 m ahead appears at y_n = 10/20
  const auto det = detection_at(kCam.cx, kCam.cy + kCam.fy * 0.5, 20, 20);
  const auto pose = est->estimate(det, frame, telemetry_at_altitude(10.0));
  REQUIRE(pose.has_value());
  REQUIRE(pose->position_m.z() == Approx(20.0).epsilon(0.01));
  REQUIRE(pose->position_m.y() == Approx(10.0).epsilon(0.01));

  // The image centre looks at the horizon
  REQUIRE_FALSE(est->estimate(detection_at(kCam.cx, kCam.cy, 20, 20), frame,
                              telemetry_at_altitude(10.0)));

  REQUIRE_THROWS_AS(af::make_ground_plane_estimator(kCam, 120.0), std::invalid_argument);
}
