#include "aeroforge/core.hpp"
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

namespace af
{

class WebcamCapture
{
public:
  WebcamCapture(int device_id, int width, int height, int fps)
      : device_id_(device_id), width_(width), height_(height), fps_(fps), seq_(0)
  {
  }

  bool open()
  {
    cap_.open(device_id_);
    if (!cap_.isOpened())
    {
      spdlog::error("Failed to open webcam device {}", device_id_);
      return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    // Verify settings
    int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    spdlog::info("Webcam opened: {}x{} @ {} FPS (requested {}x{} @ {} FPS)", actual_width,
                 actual_height, actual_fps, width_, height_, fps_);

    return true;
  }

  void close()
  {
    if (cap_.isOpened())
    {
      cap_.release();
    }
  }

  bool read(Frame& frame)
  {
    if (!cap_.isOpened())
    {
      return false;
    }

    bool success = cap_.read(frame.img);
    if (!success)
    {
      return false;
    }

    frame.seq = seq_++;
    frame.timestamp_s = now_seconds();
    return true;
  }

private:
  cv::VideoCapture cap_;
  int device_id_;
  int width_;
  int height_;
  int fps_;
  uint64_t seq_;
};

} // namespace af
