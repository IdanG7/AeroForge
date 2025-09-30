#include "aeroforge/plugin.hpp"
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace af
{

class ColorBallDetector : public IDetector
{
public:
  ColorBallDetector(const cv::Scalar& hsv_lo, const cv::Scalar& hsv_hi, double min_area_px)
      : hsv_lo_(hsv_lo), hsv_hi_(hsv_hi), min_area_px_(min_area_px)
  {
    spdlog::info("[ColorBallDetector] HSV range: [{}, {}, {}] - [{}, {}, {}], min_area: {}",
                 hsv_lo[0], hsv_lo[1], hsv_lo[2], hsv_hi[0], hsv_hi[1], hsv_hi[2],
                 min_area_px);
  }

  void process(Frame& frame) override
  {
    // Not used in this implementation
    (void)frame;
  }

  std::vector<Detection> detect(const Frame& frame) override
  {
    if (frame.img.empty())
    {
      return {};
    }

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(frame.img, hsv, cv::COLOR_BGR2HSV);

    // Threshold
    cv::Mat mask;
    cv::inRange(hsv, hsv_lo_, hsv_hi_, mask);

    // Morphological operations to reduce noise
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
      return {};
    }

    // Find largest contour
    auto it = std::max_element(contours.begin(), contours.end(),
                               [](const auto& a, const auto& b)
                               { return cv::contourArea(a) < cv::contourArea(b); });

    double area = cv::contourArea(*it);
    if (area < min_area_px_)
    {
      return {};
    }

    // Compute centroid and radius
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*it, center, radius);

    Detection det;
    det.id = 0;
    det.centroid = center;
    det.bbox = cv::Rect(static_cast<int>(center.x - radius),
                        static_cast<int>(center.y - radius), static_cast<int>(2 * radius),
                        static_cast<int>(2 * radius));
    det.score = static_cast<float>(area / (CV_PI * radius * radius)); // circularity

    return {det};
  }

  const char* name() const override { return "ColorBallDetector"; }

private:
  cv::Scalar hsv_lo_;
  cv::Scalar hsv_hi_;
  double min_area_px_;
};

} // namespace af
