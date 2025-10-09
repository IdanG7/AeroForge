#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "aeroforge/math.hpp"

using Catch::Approx;

TEST_CASE("PIDController basic behavior", "[math]")
{
  Eigen::Vector3d kp(1.0, 1.0, 1.0);
  Eigen::Vector3d ki(0.0, 0.0, 0.0);
  Eigen::Vector3d kd(0.0, 0.0, 0.0);
  double max_output = 10.0;

  af::PIDController pid(kp, ki, kd, max_output);

  SECTION("Proportional only")
  {
    Eigen::Vector3d error(1.0, 2.0, 3.0);
    double dt = 0.1;

    Eigen::Vector3d output = pid.compute(error, dt);

    REQUIRE(output.x() == Approx(1.0));
    REQUIRE(output.y() == Approx(2.0));
    REQUIRE(output.z() == Approx(3.0));
  }

  SECTION("Output clamping")
  {
    Eigen::Vector3d error(20.0, 20.0, 20.0);
    double dt = 0.1;

    Eigen::Vector3d output = pid.compute(error, dt);

    REQUIRE(output.x() <= max_output);
    REQUIRE(output.y() <= max_output);
    REQUIRE(output.z() <= max_output);
  }

  SECTION("Reset clears state")
  {
    Eigen::Vector3d error(5.0, 5.0, 5.0);
    pid.compute(error, 0.1);
    pid.reset();

    // After reset, should behave as if first call
    Eigen::Vector3d output = pid.compute(error, 0.1);
    REQUIRE(output.x() == Approx(5.0));
  }
}
