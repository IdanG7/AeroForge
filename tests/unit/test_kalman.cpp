#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "aeroforge/math.hpp"

using Catch::Approx;

TEST_CASE("KalmanFilter1D basic behavior", "[math]")
{
  af::KalmanFilter1D kf(0.01, 0.1);

  SECTION("Initialization")
  {
    kf.init(10.0, 0.0);
    REQUIRE(kf.state() == Approx(10.0));
    REQUIRE(kf.velocity() == Approx(0.0));
  }

  SECTION("Predict advances state")
  {
    kf.init(0.0, 1.0); // position=0, velocity=1
    kf.predict(1.0);   // dt=1

    // State should be approximately position=1, velocity=1
    REQUIRE(kf.state() == Approx(1.0).margin(0.1));
    REQUIRE(kf.velocity() == Approx(1.0).margin(0.1));
  }

  SECTION("Update corrects state")
  {
    kf.init(0.0, 0.0);
    kf.predict(1.0);
    kf.update(5.0); // measurement

    // State should move towards measurement
    REQUIRE(kf.state() > 0.0);
    REQUIRE(kf.state() < 5.0);
  }
}

TEST_CASE("KalmanFilter3D basic behavior", "[math]")
{
  af::KalmanFilter3D kf(0.01, 0.1);

  SECTION("Initialization")
  {
    Eigen::Vector3d pos(1, 2, 3);
    Eigen::Vector3d vel(0.1, 0.2, 0.3);
    kf.init(pos, vel);

    REQUIRE(kf.position().x() == Approx(1.0));
    REQUIRE(kf.position().y() == Approx(2.0));
    REQUIRE(kf.position().z() == Approx(3.0));
    REQUIRE(kf.velocity().x() == Approx(0.1));
  }

  SECTION("Predict advances state")
  {
    Eigen::Vector3d pos(0, 0, 0);
    Eigen::Vector3d vel(1, 0, 0);
    kf.init(pos, vel);
    kf.predict(1.0);

    // X position should be ~1, velocity ~1
    REQUIRE(kf.position().x() == Approx(1.0).margin(0.1));
    REQUIRE(kf.velocity().x() == Approx(1.0).margin(0.1));
  }

  SECTION("Update corrects state")
  {
    kf.init(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    kf.predict(1.0);
    kf.update(Eigen::Vector3d(5, 5, 5));

    // State should move towards measurement
    REQUIRE(kf.position().x() > 0.0);
    REQUIRE(kf.position().x() < 5.0);
  }
}
