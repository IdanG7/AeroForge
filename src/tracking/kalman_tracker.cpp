#include "aeroforge/plugin.hpp"
#include "aeroforge/math.hpp"
#include <spdlog/spdlog.h>

namespace af
{

class KalmanTracker : public ITracker
{
public:
  KalmanTracker(double q, double r) : q_(q), r_(r)
  {
    spdlog::info("[KalmanTracker] q={}, r={}", q, r);
  }

  void process(Frame& frame) override { (void)frame; }

  std::vector<Detection> update(double dt, const std::vector<Detection>& detections) override
  {
    if (detections.empty())
    {
      // No measurements; just predict
      if (initialized_)
      {
        filter_.predict(dt);
      }
      return {};
    }

    // For simplicity, track only the first detection
    const auto& det = detections[0];

    if (!initialized_)
    {
      // Initialize filter with first detection
      filter_.init(det.centroid.x, det.centroid.y);
      initialized_ = true;
      spdlog::debug("[KalmanTracker] Initialized at ({:.1f}, {:.1f})", det.centroid.x,
                    det.centroid.y);
    }
    else
    {
      // Predict + update
      filter_.predict(dt);
      filter_.update(det.centroid.x, det.centroid.y);
    }

    // Create smoothed detection
    Detection smoothed = det;
    smoothed.centroid.x = static_cast<float>(filter_.state_x());
    smoothed.centroid.y = static_cast<float>(filter_.state_y());

    // Update bbox center
    int w = smoothed.bbox.width;
    int h = smoothed.bbox.height;
    smoothed.bbox.x = static_cast<int>(smoothed.centroid.x - w / 2.0f);
    smoothed.bbox.y = static_cast<int>(smoothed.centroid.y - h / 2.0f);

    return {smoothed};
  }

  const char* name() const override { return "KalmanTracker"; }

private:
  // Simple 2D Kalman for X and Y separately
  class Kalman2D
  {
  public:
    Kalman2D(double q, double r) : filter_x_(q, r), filter_y_(q, r) {}

    void init(double x, double y)
    {
      filter_x_.init(x, 0.0);
      filter_y_.init(y, 0.0);
    }

    void predict(double dt)
    {
      filter_x_.predict(dt);
      filter_y_.predict(dt);
    }

    void update(double x, double y)
    {
      filter_x_.update(x);
      filter_y_.update(y);
    }

    double state_x() const { return filter_x_.state(); }
    double state_y() const { return filter_y_.state(); }

  private:
    KalmanFilter1D filter_x_;
    KalmanFilter1D filter_y_;
  };

  double q_;
  double r_;
  bool initialized_{false};
  Kalman2D filter_{q_, r_};
};

} // namespace af
