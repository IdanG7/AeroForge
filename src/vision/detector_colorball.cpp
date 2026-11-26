#include "aeroforge/modules.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{
namespace
{

class ColorBallDetector : public IDetector
{
public:
  ColorBallDetector(const cv::Scalar& hsv_lo, const cv::Scalar& hsv_hi, double min_area_px)
      : hsv_lo_(hsv_lo), hsv_hi_(hsv_hi), min_area_px_(min_area_px)
  {
    spdlog::info("[ColorBallDetector] HSV range: [{}, {}, {}] - [{}, {}, {}], min_area: {}", hsv_lo[0],
                 hsv_lo[1], hsv_lo[2], hsv_hi[0], hsv_hi[1], hsv_hi[2], min_area_px);
  }

  void process(Frame& frame) override { (void)frame; }

  std::vector<Detection> detect(const Frame& frame) override
  {
    if (frame.img.empty() || frame.img.type() != CV_8UC3)
    {
      return {};
    }

    cv::Mat hsv;
    cv::cvtColor(frame.img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_lo_, hsv_hi_, mask);

    // Morphological operations to reduce noise
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
      return {};
    }

    auto it = std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b)
                               { return cv::contourArea(a) < cv::contourArea(b); });

    double area = cv::contourArea(*it);
    if (area < min_area_px_)
    {
      return {};
    }

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(*it, center, radius);
    if (!(radius > 0.0f)) // also rejects NaN
    {
      return {};
    }

    Detection det;
    det.id = 0;
    det.centroid = center;
    det.bbox = cv::Rect(static_cast<int>(center.x - radius), static_cast<int>(center.y - radius),
                        static_cast<int>(2 * radius), static_cast<int>(2 * radius));
    // Circularity: 1.0 for a filled disc, lower for irregular blobs
    det.score = static_cast<float>(std::min(1.0, area / (CV_PI * radius * radius)));

    return {det};
  }

  const char* name() const override { return "ColorBallDetector"; }

private:
  cv::Scalar hsv_lo_;
  cv::Scalar hsv_hi_;
  double min_area_px_;
};

} // namespace

std::unique_ptr<IDetector> make_colorball_detector(const cv::Scalar& hsv_lo, const cv::Scalar& hsv_hi,
                                                   double min_area_px)
{
  for (int i = 0; i < 3; ++i)
  {
    if (!std::isfinite(hsv_lo[i]) || !std::isfinite(hsv_hi[i]) || hsv_lo[i] > hsv_hi[i])
    {
      throw std::invalid_argument(
          "colorball: hsv_lo must be <= hsv_hi in every channel (H 0-179, S 0-255, V 0-255)");
    }
  }
  if (!std::isfinite(min_area_px) || min_area_px < 0.0)
  {
    throw std::invalid_argument("colorball: min_area_px must be >= 0");
  }
  return std::make_unique<ColorBallDetector>(hsv_lo, hsv_hi, min_area_px);
}

} // namespace af
