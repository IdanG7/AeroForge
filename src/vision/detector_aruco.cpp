#include "aeroforge/modules.hpp"

#include <algorithm>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{
namespace
{

// Uses the ArUco API that lives in the main OpenCV objdetect module (OpenCV >= 4.7), so
// opencv_contrib is not required.
class ArUcoDetector : public IDetector
{
public:
  explicit ArUcoDetector(int dict_id)
      : detector_(cv::aruco::getPredefinedDictionary(dict_id), cv::aruco::DetectorParameters())
  {
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
    detector_.detectMarkers(frame.img, corners, ids);

    std::vector<Detection> detections;
    detections.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const auto& c = corners[i];
      if (c.size() != 4)
      {
        continue;
      }

      Detection det;
      det.id = ids[i];

      const float min_x = std::min({c[0].x, c[1].x, c[2].x, c[3].x});
      const float min_y = std::min({c[0].y, c[1].y, c[2].y, c[3].y});
      const float max_x = std::max({c[0].x, c[1].x, c[2].x, c[3].x});
      const float max_y = std::max({c[0].y, c[1].y, c[2].y, c[3].y});

      det.bbox = cv::Rect(static_cast<int>(min_x), static_cast<int>(min_y), static_cast<int>(max_x - min_x),
                          static_cast<int>(max_y - min_y));
      det.centroid.x = (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f;
      det.centroid.y = (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f;
      det.score = 1.0f;

      detections.push_back(det);
    }

    return detections;
  }

  const char* name() const override { return "ArUcoDetector"; }

private:
  cv::aruco::ArucoDetector detector_;
};

} // namespace

std::unique_ptr<IDetector> make_aruco_detector(int dict_id)
{
  if (dict_id < 0 || dict_id > cv::aruco::DICT_ARUCO_MIP_36h12)
  {
    throw std::invalid_argument("aruco: dict_id is not a predefined cv::aruco dictionary (0 = DICT_4X4_50)");
  }
  return std::make_unique<ArUcoDetector>(dict_id);
}

} // namespace af
