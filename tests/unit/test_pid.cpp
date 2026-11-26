#include "aeroforge/math.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>

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

TEST_CASE("PIDController rejects invalid inputs", "[math][safety]")
{
  af::PIDController pid(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.5, 0.5, 0.5),
                        Eigen::Vector3d(0.1, 0.1, 0.1), 10.0);

  SECTION("Zero dt yields a finite output")
  {
    pid.compute(Eigen::Vector3d(1, 1, 1), 0.1);
    Eigen::Vector3d out = pid.compute(Eigen::Vector3d(2, 2, 2), 0.0);
    REQUIRE(out.allFinite());
  }

  SECTION("Negative dt does not integrate backwards")
  {
    Eigen::Vector3d a = pid.compute(Eigen::Vector3d(1, 1, 1), 0.1);
    Eigen::Vector3d b = pid.compute(Eigen::Vector3d(1, 1, 1), -5.0);
    REQUIRE(b.allFinite());
    REQUIRE(b.x() == Approx(a.x()));
  }

  SECTION("Non-finite error produces zero output and does not poison state")
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d out = pid.compute(Eigen::Vector3d(nan, 1, 1), 0.1);
    REQUIRE(out.isZero());
    Eigen::Vector3d next = pid.compute(Eigen::Vector3d(1, 1, 1), 0.1);
    REQUIRE(next.allFinite());
  }

  SECTION("Integral term is limited so the output recovers after saturation")
  {
    for (int i = 0; i < 1000; ++i)
    {
      pid.compute(Eigen::Vector3d(100, 100, 100), 0.1);
    }
    // After the error changes sign, the output must leave saturation quickly.
    Eigen::Vector3d out;
    for (int i = 0; i < 50; ++i)
    {
      out = pid.compute(Eigen::Vector3d(-1, -1, -1), 0.1);
    }
    REQUIRE(out.x() < 10.0);
  }
}
