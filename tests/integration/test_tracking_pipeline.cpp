#include "../support/synthetic.hpp"
#include "aeroforge/config.hpp"
#include "aeroforge/io.hpp"
#include "aeroforge/modules.hpp"
#include "aeroforge/pipeline.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <opencv2/videoio.hpp>

using Catch::Approx;

namespace
{

af::Frame make_frame(int i, cv::Mat img)
{
  af::Frame f;
  f.seq = static_cast<uint64_t>(i);
  f.timestamp_s = i / 30.0;
  f.img = std::move(img);
  return f;
}

af::Telemetry fresh_telemetry()
{
  af::Telemetry tel;
  tel.validity = af::Validity::Fresh;
  tel.altitude_m = 10.0;
  return tel;
}

double distance(cv::Point2f a, cv::Point2f b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::string temp_video(const char* name)
{
  return (std::filesystem::temp_directory_path() / name).string();
}

// Writes `frames` frames of the synthetic tracking sequence as Motion-JPEG
void write_video(const std::string& path, int frames)
{
  cv::VideoWriter writer(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0,
                         cv::Size(synth::kWidth, synth::kHeight));
  REQUIRE(writer.isOpened());
  for (int i = 0; i < frames; ++i)
  {
    writer.write(synth::tracking_frame(i));
  }
}

} // namespace

TEST_CASE("Template tracker follows a moving synthetic target", "[integration][tracking]")
{
  auto tracker = af::make_template_tracker();
  REQUIRE_FALSE(tracker->is_tracking());

  SECTION("Nothing is reported before a target is selected")
  {
    REQUIRE(tracker->detect(make_frame(0, synth::tracking_frame(0))).empty());
  }

  SECTION("Unusable selections are rejected")
  {
    const cv::Mat img = synth::tracking_frame(0);
    REQUIRE_FALSE(tracker->set_roi(cv::Rect(10, 10, 0, 50), img));
    REQUIRE_FALSE(tracker->set_roi(cv::Rect(10, 10, 5, 5), img));       // too small
    REQUIRE_FALSE(tracker->set_roi(cv::Rect(5000, 5000, 60, 60), img)); // outside the image
    REQUIRE_FALSE(tracker->set_roi(cv::Rect(10, 10, 60, 60), cv::Mat()));
    REQUIRE_FALSE(tracker->is_tracking());
  }

  REQUIRE(tracker->set_roi(synth::target_rect(0), synth::tracking_frame(0)));
  REQUIRE(tracker->is_tracking());

  const int kFrames = 90;
  double worst_error_px = 0.0;
  for (int i = 1; i < kFrames; ++i)
  {
    const auto dets = tracker->detect(make_frame(i, synth::tracking_frame(i)));
    REQUIRE(dets.size() == 1);
    REQUIRE(dets[0].validity == af::Validity::Fresh);
    worst_error_px = std::max(worst_error_px, distance(dets[0].centroid, synth::target_center(i)));
  }
  INFO("worst centroid error: " << worst_error_px << " px");
  REQUIRE(worst_error_px < 8.0);

  SECTION("Losing the target is observable: predicted, then gone")
  {
    bool saw_predicted = false;
    bool saw_measured_after_loss = false;
    int frames_until_gone = -1;
    for (int i = 0; i < 80; ++i)
    {
      // Target removed from the scene
      const auto dets = tracker->detect(make_frame(kFrames + i, synth::tracking_frame(kFrames + i, false)));
      for (const auto& d : dets)
      {
        if (d.validity == af::Validity::Predicted)
        {
          saw_predicted = true;
        }
        else if (i > 10)
        {
          saw_measured_after_loss = true;
        }
      }
      if (!tracker->is_tracking())
      {
        frames_until_gone = i;
        break;
      }
    }
    REQUIRE(saw_predicted);
    REQUIRE_FALSE(saw_measured_after_loss); // never invents measurements of a missing target
    REQUIRE(frames_until_gone > 0);
    REQUIRE(frames_until_gone <= 60);
    REQUIRE(tracker->detect(make_frame(999, synth::tracking_frame(0, false))).empty());
  }

  SECTION("Reset stops tracking and a new target can be selected")
  {
    tracker->reset();
    REQUIRE_FALSE(tracker->is_tracking());
    REQUIRE(tracker->detect(make_frame(kFrames, synth::tracking_frame(kFrames))).empty());
    REQUIRE(tracker->set_roi(synth::target_rect(kFrames), synth::tracking_frame(kFrames)));
    const auto dets = tracker->detect(make_frame(kFrames + 1, synth::tracking_frame(kFrames + 1)));
    REQUIRE(dets.size() == 1);
    REQUIRE(distance(dets[0].centroid, synth::target_center(kFrames + 1)) < 8.0);
  }
}

TEST_CASE("Recorded video replay is deterministic", "[integration][io]")
{
  const std::string path = temp_video("aeroforge_test_replay.avi");
  const int kFrames = 30;
  write_video(path, kFrames);

  af::InputConfig input;
  input.type = "file";
  input.path = path;

  auto source = af::make_frame_source(input);
  REQUIRE(source->open());
  REQUIRE_FALSE(source->is_live());
  REQUIRE(source->nominal_fps() == Approx(30.0));
  REQUIRE(source->frame_size() == cv::Size(synth::kWidth, synth::kHeight));

  af::Frame frame;
  int count = 0;
  while (source->read(frame))
  {
    REQUIRE(frame.seq == static_cast<uint64_t>(count));
    REQUIRE(frame.timestamp_s == Approx(count / 30.0));
    REQUIRE(source->now_s() == Approx(frame.timestamp_s));
    REQUIRE(frame.img.size() == cv::Size(synth::kWidth, synth::kHeight));
    ++count;
  }
  REQUIRE(count == kFrames);
  REQUIRE(source->at_end());
  REQUIRE_FALSE(source->read(frame)); // stays at the end
  source->close();

  SECTION("Looping keeps sequence numbers and timestamps increasing")
  {
    input.loop = true;
    auto looped = af::make_frame_source(input);
    REQUIRE(looped->open());
    double last_t = -1.0;
    for (int i = 0; i < kFrames + 5; ++i)
    {
      REQUIRE(looped->read(frame));
      REQUIRE(frame.seq == static_cast<uint64_t>(i));
      REQUIRE(frame.timestamp_s > last_t);
      last_t = frame.timestamp_s;
    }
    REQUIRE_FALSE(looped->at_end());
  }

  std::remove(path.c_str());
}

TEST_CASE("Frame source failures are reported, not thrown", "[integration][io][failure]")
{
  af::InputConfig input;
  input.type = "file";
  input.path = temp_video("aeroforge_this_file_does_not_exist.avi");
  auto source = af::make_frame_source(input);
  REQUIRE_FALSE(source->open());
  af::Frame frame;
  REQUIRE_FALSE(source->read(frame));

  input.type = "file";
  input.path.clear();
  REQUIRE_THROWS_AS(af::make_frame_source(input), std::invalid_argument);
  input.type = "dji";
  REQUIRE_THROWS_AS(af::make_frame_source(input), std::invalid_argument);
}

TEST_CASE("Pipeline: detect, track, estimate and control on synthetic frames", "[integration][pipeline]")
{
  // 0.2 m ball, f = 600 px, drawn with a 30 px radius  =>  range = 600 * 0.2 / 60 = 2.0 m
  const char* yaml = R"(
pipeline:
  modules:
    - name: detector
      type: colorball
      params: { hsv_lo: [30, 120, 80], hsv_hi: [90, 255, 255], min_area_px: 200 }
    - name: tracker
      type: kalman
      params: { q: 0.02, r: 0.2 }
    - name: estimator
      type: size_based
      params: { object_diameter_m: 0.2, fx: 600, fy: 600, cx: 320, cy: 240 }
    - name: controller
      type: pid_velocity
      params: { desired_offset_m: [-3, 0, 0], kp: [0.8, 0.8, 0.6], max_speed_mps: 3 }
)";
  af::Pipeline pipeline(af::parse_config(yaml, "test"));
  REQUIRE(pipeline.target_tracker() == nullptr); // colorball is automatic
  REQUIRE(pipeline.has_tracker());
  REQUIRE(pipeline.has_estimator());
  REQUIRE(pipeline.has_controller());

  const auto tel = fresh_telemetry();
  af::PipelineOutput out;
  for (int i = 0; i < 30; ++i)
  {
    out = pipeline.step(make_frame(i, synth::ball_frame(cv::Point(200 + 2 * i, 240), 30)), tel, true);
    REQUIRE(out.detections.size() == 1);
    REQUIRE(out.tracked.size() == 1);
    REQUIRE(out.pose.has_value());
    REQUIRE(out.pose->position_m.z() == Approx(2.0).margin(0.15));
    REQUIRE(out.velocity_cmd_body.has_value());
    REQUIRE(out.velocity_cmd_body->allFinite());
    REQUIRE(out.timings.detect_ms >= 0.0);
  }
  // Ball is 2 m ahead but should be 3 m ahead: back away (negative forward speed)
  REQUIRE(out.velocity_cmd_body->x() < 0.0);
  // Ball is left of the image centre: move left (negative right speed)
  REQUIRE(out.velocity_cmd_body->y() < 0.0);

  SECTION("The controller is skipped when control is not enabled")
  {
    out = pipeline.step(make_frame(30, synth::ball_frame(cv::Point(260, 240), 30)), tel, false);
    REQUIRE(out.pose.has_value());
    REQUIRE_FALSE(out.velocity_cmd_body.has_value());
  }

  SECTION("Losing the target clears the pose and the command")
  {
    out = pipeline.step(
        make_frame(30, cv::Mat(synth::kHeight, synth::kWidth, CV_8UC3, cv::Scalar(120, 120, 120))), tel,
        true);
    REQUIRE(out.detections.empty());
    REQUIRE_FALSE(out.pose.has_value());
    REQUIRE_FALSE(out.velocity_cmd_body.has_value());
  }

  SECTION("Empty frames are ignored")
  {
    out = pipeline.step(make_frame(30, cv::Mat()), tel, true);
    REQUIRE(out.detections.empty());
    REQUIRE_FALSE(out.pose.has_value());
  }
}

TEST_CASE("Pipeline: interactive tracking through a replayed video", "[integration][pipeline][io]")
{
  const std::string path = temp_video("aeroforge_test_pipeline.avi");
  const int kFrames = 60;
  write_video(path, kFrames);

  af::PipelineConfig config = af::parse_config(R"(
pipeline:
  modules:
    - { name: detector, type: interactive }
    - { name: tracker, type: kalman, params: { q: 0.02, r: 0.2 } }
)",
                                               "test");
  config.input.type = "file";
  config.input.path = path;

  af::Pipeline pipeline(config);
  auto source = af::make_frame_source(config.input);
  REQUIRE(source->open());

  af::Frame frame;
  int tracked_frames = 0;
  double worst_error_px = 0.0;
  for (int i = 0; source->read(frame); ++i)
  {
    if (i == 0)
    {
      REQUIRE(pipeline.select_target(synth::target_rect(0), frame));
    }
    const auto out = pipeline.step(frame, fresh_telemetry(), false);
    if (i > 0 && !out.tracked.empty())
    {
      ++tracked_frames;
      worst_error_px = std::max(worst_error_px, distance(out.tracked[0].centroid, synth::target_center(i)));
    }
  }
  INFO("tracked " << tracked_frames << " frames, worst error " << worst_error_px << " px");
  REQUIRE(tracked_frames == kFrames - 1);
  REQUIRE(worst_error_px < 10.0);

  pipeline.reset();
  REQUIRE_FALSE(pipeline.target_tracker()->is_tracking());

  source->close();
  std::remove(path.c_str());
}

namespace
{

// The synthetic target blended with unrelated structure: `alpha` = 1 is the clean target
cv::Mat degraded_target_frame(double alpha)
{
  cv::Mat blocks(16, 16, CV_8UC3);
  cv::RNG rng(4321);
  rng.fill(blocks, cv::RNG::UNIFORM, 0, 256);
  cv::Mat clutter;
  cv::resize(blocks, clutter, cv::Size(synth::kTargetSize, synth::kTargetSize), 0, 0, cv::INTER_NEAREST);

  cv::Mat frame = synth::background();
  cv::Mat patch;
  cv::addWeighted(synth::target_patch(), alpha, clutter, 1.0 - alpha, 0.0, patch);
  patch.copyTo(frame(synth::target_rect(0)));
  return frame;
}

} // namespace

TEST_CASE("Template tracker re-acquires a degraded target late in an occlusion", "[integration][tracking]")
{
  auto tracker = af::make_template_tracker();
  REQUIRE(tracker->set_roi(synth::target_rect(0), synth::tracking_frame(0)));

  int i = 1;
  for (; i <= 20; ++i)
  {
    REQUIRE(tracker->detect(make_frame(i, synth::tracking_frame(0))).size() == 1);
  }

  // Hidden for 44 frames: past the point where the prediction stops contributing, one
  // frame short of the target being dropped
  for (int k = 0; k < 44; ++k, ++i)
  {
    tracker->detect(make_frame(i, synth::tracking_frame(0, false)));
  }
  REQUIRE(tracker->is_tracking());

  // Back, but only a moderate visual match. The prediction's weight used to go negative
  // here and vetoed the match, so the target was dropped two frames later.
  const auto dets = tracker->detect(make_frame(i, degraded_target_frame(0.40)));
  REQUIRE(dets.size() == 1);
  REQUIRE(dets[0].validity == af::Validity::Fresh);
  REQUIRE(distance(dets[0].centroid, synth::target_center(0)) < 8.0);
  REQUIRE(tracker->is_tracking());
}
