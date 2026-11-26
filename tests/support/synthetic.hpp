#pragma once

// Deterministic synthetic scenes used as ground truth by the tests and by the video
// fixture generator. Nothing here depends on wall-clock time or unseeded randomness.

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace synth
{

inline constexpr int kWidth = 640;
inline constexpr int kHeight = 480;
inline constexpr int kTargetSize = 64;

// Smooth, low-contrast texture so that the target stands out but the background is not flat
inline cv::Mat background(cv::Size size = cv::Size(kWidth, kHeight))
{
  cv::Mat small(size.height / 16, size.width / 16, CV_8UC3);
  cv::RNG rng(1234);
  rng.fill(small, cv::RNG::UNIFORM, 90, 150);
  cv::Mat bg;
  cv::resize(small, bg, size, 0, 0, cv::INTER_CUBIC);
  return bg;
}

// High-contrast patch of coloured blocks: plenty of template and ORB structure
inline cv::Mat target_patch(int size = kTargetSize)
{
  cv::Mat blocks(8, 8, CV_8UC3);
  cv::RNG rng(99);
  rng.fill(blocks, cv::RNG::UNIFORM, 0, 256);
  cv::Mat patch;
  cv::resize(blocks, patch, cv::Size(size, size), 0, 0, cv::INTER_NEAREST);
  return patch;
}

// Top-left corner of the target in frame `i`: moves 3 px right and 1 px down per frame
inline cv::Point target_origin(int i)
{
  return {100 + 3 * i, 120 + i};
}

inline cv::Rect target_rect(int i)
{
  const cv::Point o = target_origin(i);
  return {o.x, o.y, kTargetSize, kTargetSize};
}

inline cv::Point2f target_center(int i)
{
  const cv::Point o = target_origin(i);
  return {o.x + kTargetSize / 2.0f, o.y + kTargetSize / 2.0f};
}

inline cv::Mat tracking_frame(int i, bool with_target = true)
{
  cv::Mat frame = background();
  if (with_target)
  {
    target_patch().copyTo(frame(target_rect(i)));
  }
  return frame;
}

// Saturated green disc (HSV about H=60, S=255, V=200) on a grey background
inline cv::Mat ball_frame(cv::Point center, int radius, cv::Size size = cv::Size(kWidth, kHeight))
{
  cv::Mat frame(size, CV_8UC3, cv::Scalar(120, 120, 120));
  cv::circle(frame, center, radius, cv::Scalar(0, 200, 0), cv::FILLED, cv::LINE_AA);
  return frame;
}

} // namespace synth
