#include "aeroforge/plugin.hpp"
#include <opencv2/aruco.hpp>
#include <spdlog/spdlog.h>

namespace af
{

class ArUcoDetector : public IDetector
{
public:
  ArUcoDetector(int dict_id = cv::aruco::DICT_4X4_50)
  {
    dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
    params_ = cv::aruco::DetectorParameters::create();
    spdlog::info("[ArUcoDetector] Initialized with dictionary ID: {}", dict_id);
  }

  void process(Frame& frame) override { (void)frame; }

  std::vector<Detection> detect(const Frame& frame) override
  {
    if (frame.img.empty())
    {
      return {};
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::detectMarkers(frame.img, dictionary_, corners, ids, params_);

    if (ids.empty())
    {
      return {};
    }

    std::vector<Detection> detections;
    for (size_t i = 0; i < ids.size(); ++i)
    {
      Detection det;
      det.id = ids[i];

      // Compute bounding box from corners
      auto& c = corners[i];
      int min_x = static_cast<int>(
          std::min({c[0].x, c[1].x, c[2].x, c[3].x}));
      int min_y = static_cast<int>(
          std::min({c[0].y, c[1].y, c[2].y, c[3].y}));
      int max_x = static_cast<int>(
          std::max({c[0].x, c[1].x, c[2].x, c[3].x}));
      int max_y = static_cast<int>(
          std::max({c[0].y, c[1].y, c[2].y, c[3].y}));

      det.bbox = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);

      // Centroid
      det.centroid.x = (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f;
      det.centroid.y = (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f;

      det.score = 1.0f;

      detections.push_back(det);
    }

    return detections;
  }

  const char* name() const override { return "ArUcoDetector"; }

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};

} // namespace af
