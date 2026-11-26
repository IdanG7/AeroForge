#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include "../support/synthetic.hpp"
#include "aeroforge/modules.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>

using Catch::Approx;

TEST_CASE("ColorBallDetector finds a synthetic ball", "[integration][vision]")
{
  auto det = af::make_colorball_detector(cv::Scalar(30, 120, 80), cv::Scalar(90, 255, 255), 200);

  af::Frame frame;
  frame.img = synth::ball_frame(cv::Point(200, 150), 40);

  const auto found = det->detect(frame);
  REQUIRE(found.size() == 1);
  REQUIRE(found[0].centroid.x == Approx(200.0).margin(2.0));
  REQUIRE(found[0].centroid.y == Approx(150.0).margin(2.0));
  REQUIRE(found[0].bbox.width == Approx(80).margin(6));
  REQUIRE(found[0].bbox.height == Approx(80).margin(6));
  REQUIRE(found[0].score > 0.8f);
  REQUIRE(found[0].score <= 1.0f);
  REQUIRE(found[0].validity == af::Validity::Fresh);
}

TEST_CASE("ColorBallDetector reports nothing when there is nothing", "[integration][vision]")
{
  auto det = af::make_colorball_detector(cv::Scalar(30, 120, 80), cv::Scalar(90, 255, 255), 200);
  af::Frame frame;

  SECTION("Empty frame")
  {
    REQUIRE(det->detect(frame).empty());
  }

  SECTION("No ball in the image")
  {
    frame.img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(120, 120, 120));
    REQUIRE(det->detect(frame).empty());
  }

  SECTION("Ball smaller than min_area_px")
  {
    frame.img = synth::ball_frame(cv::Point(200, 150), 5);
    REQUIRE(det->detect(frame).empty());
  }

  SECTION("Wrong colour")
  {
    frame.img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(120, 120, 120));
    cv::circle(frame.img, cv::Point(200, 150), 40, cv::Scalar(0, 0, 220), cv::FILLED); // red
    REQUIRE(det->detect(frame).empty());
  }

  SECTION("Greyscale input is rejected instead of crashing")
  {
    frame.img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(120));
    REQUIRE(det->detect(frame).empty());
  }
}

TEST_CASE("ColorBallDetector parameter validation", "[integration][vision]")
{
  REQUIRE_THROWS_AS(
      af::make_colorball_detector(cv::Scalar(90, 0, 0), cv::Scalar(30, 255, 255), 200),
      std::invalid_argument);
  REQUIRE_THROWS_AS(
      af::make_colorball_detector(cv::Scalar(30, 0, 0), cv::Scalar(90, 255, 255), -1),
      std::invalid_argument);
}

TEST_CASE("ArUcoDetector finds a generated marker", "[integration][vision]")
{
  const int dict_id = cv::aruco::DICT_4X4_50;
  auto det = af::make_aruco_detector(dict_id);

  cv::Mat marker;
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(dict_id), 7, 200, marker);

  af::Frame frame;
  frame.img = cv::Mat(480, 640, CV_8UC1, cv::Scalar(255));
  marker.copyTo(frame.img(cv::Rect(220, 140, 200, 200)));
  cv::cvtColor(frame.img, frame.img, cv::COLOR_GRAY2BGR);

  const auto found = det->detect(frame);
  REQUIRE(found.size() == 1);
  REQUIRE(found[0].id == 7);
  REQUIRE(found[0].centroid.x == Approx(320.0).margin(2.0));
  REQUIRE(found[0].centroid.y == Approx(240.0).margin(2.0));
  REQUIRE(found[0].bbox.width == Approx(200).margin(4));

  SECTION("No marker, no detection")
  {
    frame.img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    REQUIRE(det->detect(frame).empty());
  }

  REQUIRE_THROWS_AS(af::make_aruco_detector(-1), std::invalid_argument);
  REQUIRE_THROWS_AS(af::make_aruco_detector(9999), std::invalid_argument);
}
