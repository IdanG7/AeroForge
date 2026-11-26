#include "aeroforge/io.hpp"

#include <cmath>
#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{
namespace
{

// OpenCV's Media Foundation backend probes hardware transforms when it opens a camera.
// On the development machine that made the first frame arrive after about 50 s; with the
// probe disabled it arrives after 1.6 s at the same 30 FPS (measured 2026-11-26, OpenCV
// 4.11/4.12). An explicit setting in the environment is left alone.
void disable_slow_msmf_probe()
{
#ifdef _WIN32
  const char* name = "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS";
  size_t required = 0;
  if (getenv_s(&required, nullptr, 0, name) == 0 && required == 0)
  {
    _putenv_s(name, "0");
  }
#endif
}

// ──────────────────────────────────────────────────────────────────────────
// Live source: webcam device or network stream URL
// ──────────────────────────────────────────────────────────────────────────

class LiveCapture : public IFrameSource
{
public:
  explicit LiveCapture(const InputConfig& cfg) : cfg_(cfg), mirror_(cfg.mirror.value_or(cfg.type == "webcam"))
  {
  }

  bool open() override
  {
    if (cfg_.type == "webcam")
    {
      disable_slow_msmf_probe();
      cap_.open(cfg_.device);
    }
    else
    {
      cap_.open(cfg_.path);
    }

    if (!cap_.isOpened())
    {
      spdlog::error("Failed to open {}", describe());
      return false;
    }

    if (cfg_.type == "webcam")
    {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg_.width);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
      cap_.set(cv::CAP_PROP_FPS, cfg_.fps);
    }

    const double fps = cap_.get(cv::CAP_PROP_FPS);
    fps_ = (std::isfinite(fps) && fps > 0.0) ? fps : static_cast<double>(cfg_.fps);
    size_ = cv::Size(static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                     static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));

    spdlog::info("Opened {}: {}x{} @ {:.1f} FPS (requested {}x{} @ {} FPS)", describe(),
                 static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                 static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), fps_, cfg_.width, cfg_.height,
                 cfg_.fps);
    return true;
  }

  void close() override { cap_.release(); }

  bool read(Frame& out) override
  {
    if (!cap_.isOpened() || !cap_.read(out.img) || out.img.empty())
    {
      ++failed_reads_;
      try_reopen();
      return false;
    }

    if (mirror_)
    {
      cv::flip(out.img, out.img, 1);
    }
    out.seq = seq_++;
    out.timestamp_s = now_seconds();
    return true;
  }

  bool at_end() const override { return false; }
  bool is_live() const override { return true; }
  double now_s() const override { return now_seconds(); }
  double nominal_fps() const override { return fps_; }
  cv::Size frame_size() const override { return size_; }
  uint64_t failed_reads() const override { return failed_reads_; }

  std::string describe() const override
  {
    return cfg_.type == "webcam" ? "webcam " + std::to_string(cfg_.device) : "stream " + cfg_.path;
  }

private:
  void try_reopen()
  {
    const double now = now_seconds();
    if (now - last_reopen_s_ < 1.0)
    {
      return;
    }
    last_reopen_s_ = now;
    spdlog::warn("{}: read failed ({} total), trying to reopen", describe(), failed_reads_);
    cap_.release();
    open();
  }

  InputConfig cfg_;
  bool mirror_;
  cv::VideoCapture cap_;
  double fps_{30.0};
  cv::Size size_;
  uint64_t seq_{0};
  uint64_t failed_reads_{0};
  double last_reopen_s_{-1.0e9};
};

// ──────────────────────────────────────────────────────────────────────────
// Recorded source: video file (or image sequence pattern) replayed on media time
// ──────────────────────────────────────────────────────────────────────────

class FileCapture : public IFrameSource
{
public:
  explicit FileCapture(const InputConfig& cfg) : cfg_(cfg), mirror_(cfg.mirror.value_or(false)) {}

  bool open() override
  {
    cap_.open(cfg_.path);
    if (!cap_.isOpened())
    {
      spdlog::error("Failed to open video file '{}' (check the path and that the codec is "
                    "supported by this OpenCV build)",
                    cfg_.path);
      return false;
    }
    const double fps = cap_.get(cv::CAP_PROP_FPS);
    fps_ = (std::isfinite(fps) && fps > 0.0 && fps < 1000.0) ? fps : static_cast<double>(cfg_.fps);
    at_end_ = false;
    size_ = cv::Size(static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                     static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)));
    spdlog::info("Opened {}: {}x{} @ {:.2f} FPS, loop={}", describe(),
                 static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                 static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), fps_, cfg_.loop);
    return true;
  }

  void close() override { cap_.release(); }

  bool read(Frame& out) override
  {
    if (!cap_.isOpened() || at_end_)
    {
      return false;
    }

    if (!cap_.read(out.img) || out.img.empty())
    {
      if (cfg_.loop && seq_ > 0)
      {
        // Restart the file; media time keeps increasing so downstream dt stays positive
        cap_.release();
        cap_.open(cfg_.path);
        if (cap_.isOpened() && cap_.read(out.img) && !out.img.empty())
        {
          return stamp(out);
        }
      }
      at_end_ = true;
      return false;
    }
    return stamp(out);
  }

  bool at_end() const override { return at_end_; }
  bool is_live() const override { return false; }
  double now_s() const override { return last_timestamp_s_; }
  double nominal_fps() const override { return fps_; }
  cv::Size frame_size() const override { return size_; }
  uint64_t failed_reads() const override { return 0; }
  std::string describe() const override { return "file " + cfg_.path; }

private:
  bool stamp(Frame& out)
  {
    if (mirror_)
    {
      cv::flip(out.img, out.img, 1);
    }
    out.seq = seq_++;
    out.timestamp_s = static_cast<double>(out.seq) / fps_;
    last_timestamp_s_ = out.timestamp_s;
    return true;
  }

  InputConfig cfg_;
  bool mirror_;
  cv::VideoCapture cap_;
  double fps_{30.0};
  cv::Size size_;
  uint64_t seq_{0};
  double last_timestamp_s_{0.0};
  bool at_end_{false};
};

} // namespace

std::unique_ptr<IFrameSource> make_frame_source(const InputConfig& input)
{
  if (input.type == "webcam")
  {
    return std::make_unique<LiveCapture>(input);
  }
  if (input.type == "file" || input.type == "stream")
  {
    if (input.path.empty())
    {
      throw std::invalid_argument("pipeline.input.path is required when input.type is '" + input.type + "'");
    }
    if (input.type == "file")
    {
      return std::make_unique<FileCapture>(input);
    }
    return std::make_unique<LiveCapture>(input);
  }
  throw std::invalid_argument("pipeline.input.type '" + input.type +
                              "' is not supported (use webcam, file or stream)");
}

} // namespace af
