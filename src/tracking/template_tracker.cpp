// Native multi-target tracker: Kalman prediction + multi-scale template matching + ORB
// feature matching, fused by confidence. Runs on CPU with core OpenCV modules only.
//
// Moved out of apps/track/aeroforge-track.cpp so that it can be linked and tested.

#include "aeroforge/modules.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <spdlog/spdlog.h>

namespace af
{
namespace
{

// Performance Logging System
class PerformanceLogger
{
public:
  explicit PerformanceLogger(const std::string& filename)
      : log_file_(filename, std::ios::out | std::ios::trunc)
  {
    if (log_file_.is_open())
    {
      // Write CSV header
      log_file_ << "timestamp,obj_id,event,quality,template_score,feature_score,features,lost_frames,"
                   "processing_ms,reason\n";
      log_file_.flush();
      spdlog::info("[PerformanceLogger] Logging to: {}", filename);
    }
    else
    {
      spdlog::error("[PerformanceLogger] Failed to open log file: {}", filename);
    }
  }

  ~PerformanceLogger()
  {
    if (log_file_.is_open())
    {
      log_file_.close();
      spdlog::info("[PerformanceLogger] Closed log file");
    }
  }

  void log_track_start(int obj_id, double timestamp)
  {
    if (!log_file_.is_open())
      return;
    log_file_ << std::fixed << std::setprecision(3) << timestamp << "," << obj_id
              << ",START,1.0,0.0,0.0,0,0,0.0,track_initialized\n";
    log_file_.flush();
  }

  void log_frame(int obj_id, double timestamp, float quality, float template_score, float feature_score,
                 int features, int lost_frames, double processing_ms)
  {
    if (!log_file_.is_open())
      return;
    log_file_ << std::fixed << std::setprecision(3) << timestamp << "," << obj_id << ",TRACK," << quality
              << "," << template_score << "," << feature_score << "," << features << "," << lost_frames << ","
              << processing_ms << ",tracking\n";
  }

  void log_loss(int obj_id, double timestamp, float quality, const std::string& reason)
  {
    if (!log_file_.is_open())
      return;
    log_file_ << std::fixed << std::setprecision(3) << timestamp << "," << obj_id << ",LOST," << quality
              << ",0.0,0.0,0,0,0.0," << reason << "\n";
    log_file_.flush();
    spdlog::warn("[PerformanceLogger] Object {} lost - {}", obj_id, reason);
  }

  void log_occlusion(int obj_id, double timestamp, int lost_frames)
  {
    if (!log_file_.is_open())
      return;
    log_file_ << std::fixed << std::setprecision(3) << timestamp << "," << obj_id
              << ",OCCLUDED,0.3,0.0,0.0,0," << lost_frames << ",0.0,occluded\n";
  }

private:
  std::ofstream log_file_;
};

// Multi-Object Tracker - tracks up to 5 objects simultaneously
class InteractiveTrackerDetector : public ITargetTracker
{
public:
  static constexpr int MAX_OBJECTS = 5;

  struct TrackedObject
  {
    // Identity
    int id;
    cv::Scalar color;
    bool active{false};

    // Visual templates
    cv::Mat template_gray;
    cv::Mat template_rgb;

    // Bounding box and scale
    cv::Rect bbox;
    float scale{1.0f};

    // Kalman filter for smooth position/velocity tracking (6-state: x, y, vx, vy, ax, ay)
    cv::KalmanFilter kalman{6, 2, 0};
    bool kalman_initialized{false};

    // Feature-based tracking (ORB keypoints)
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Ptr<cv::ORB> orb_detector;

    // Motion model (with acceleration)
    float pos_x{0.0f}, pos_y{0.0f};
    float velocity_x{0.0f}, velocity_y{0.0f};
    float accel_x{0.0f}, accel_y{0.0f};

    // Track quality metrics
    float template_match_score{0.0f};
    float feature_match_score{0.0f};
    float overall_quality{0.0f};
    int good_feature_matches{0};

    // State tracking
    int lost_frames{0};
    int tracked_frames{0};
    bool occluded{false};

    // Performance tracking
    double last_processing_ms{0.0};
    int template_area{0};

    TrackedObject()
    {
      orb_detector = cv::ORB::create(500); // Max 500 features
      initialize_kalman();
    }

    void initialize_kalman()
    {
      // State: [x, y, vx, vy, ax, ay]
      // Measurement: [x, y]
      kalman.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5, 0, // x = x + vx*dt + 0.5*ax*dt^2
                                 0, 1, 0, 1, 0, 0.5,                          // y = y + vy*dt + 0.5*ay*dt^2
                                 0, 0, 1, 0, 1, 0,                            // vx = vx + ax*dt
                                 0, 0, 0, 1, 0, 1,                            // vy = vy + ay*dt
                                 0, 0, 0, 0, 0.95, 0,                         // ax with damping
                                 0, 0, 0, 0, 0, 0.95);                        // ay with damping

      kalman.measurementMatrix = (cv::Mat_<float>(2, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);

      // Process noise covariance
      cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(0.03));

      // Measurement noise covariance
      cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(0.5));

      // Error covariance
      cv::setIdentity(kalman.errorCovPost, cv::Scalar::all(1.0));
    }

    void init_kalman(float x, float y, float vx, float vy)
    {
      kalman.statePost = (cv::Mat_<float>(6, 1) << x, y, vx, vy, 0, 0);
      kalman_initialized = true;
    }

    cv::Point2f predict_kalman()
    {
      if (!kalman_initialized)
        return cv::Point2f(pos_x, pos_y);

      cv::Mat prediction = kalman.predict();
      return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
    }

    void update_kalman(float x, float y)
    {
      if (!kalman_initialized)
      {
        init_kalman(x, y, velocity_x, velocity_y);
        return;
      }

      cv::Mat measurement = (cv::Mat_<float>(2, 1) << x, y);
      kalman.correct(measurement);

      // Update state from Kalman
      pos_x = kalman.statePost.at<float>(0);
      pos_y = kalman.statePost.at<float>(1);
      velocity_x = kalman.statePost.at<float>(2);
      velocity_y = kalman.statePost.at<float>(3);
      accel_x = kalman.statePost.at<float>(4);
      accel_y = kalman.statePost.at<float>(5);
    }
  };

  explicit InteractiveTrackerDetector(const std::string& perf_csv_path)
  {
    // Initialize color palette for different objects
    colors_ = {
        cv::Scalar(0, 255, 255), // Cyan
        cv::Scalar(255, 0, 255), // Magenta
        cv::Scalar(255, 255, 0), // Yellow
        cv::Scalar(0, 255, 0),   // Green
        cv::Scalar(255, 128, 0)  // Orange
    };
    if (!perf_csv_path.empty())
    {
      logger_ = std::make_unique<PerformanceLogger>(perf_csv_path);
    }
    spdlog::info("[MultiObjectTracker] Initialized - can track up to {} objects", MAX_OBJECTS);
  }

  void process(Frame& frame) override { (void)frame; }

  bool set_roi(const cv::Rect& roi, const cv::Mat& frame) override
  {
    if (frame.empty() || (frame.channels() != 3 && frame.channels() != 1))
    {
      spdlog::warn("[MultiObjectTracker] Cannot select a target: empty or unsupported frame");
      return false;
    }
    if (roi.width <= 0 || roi.height <= 0 || roi.area() < 100)
    {
      spdlog::warn("[MultiObjectTracker] ROI too small");
      return false;
    }

    cv::Rect safe_roi = roi & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_roi.area() < 100)
      return false;

    // Find an inactive slot
    int slot = -1;
    for (int i = 0; i < MAX_OBJECTS; ++i)
    {
      if (!objects_[i].active)
      {
        slot = i;
        break;
      }
    }
    if (slot == -1)
    {
      spdlog::warn("[MultiObjectTracker] Maximum {} objects already tracked", MAX_OBJECTS);
      return false;
    }

    // Extract and initialize tracking template
    cv::Mat template_rgb = frame(safe_roi).clone();
    cv::Mat template_gray;
    if (template_rgb.channels() == 3)
    {
      cv::cvtColor(template_rgb, template_gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
      template_gray = template_rgb;
    }

    auto& obj = objects_[slot];
    obj.id = slot;
    obj.template_gray = template_gray;
    obj.template_rgb = template_rgb;
    obj.bbox = safe_roi;
    obj.scale = 1.0f;
    obj.active = true;
    obj.lost_frames = 0;
    obj.tracked_frames = 0;
    obj.velocity_x = 0.0f;
    obj.velocity_y = 0.0f;
    obj.accel_x = 0.0f;
    obj.accel_y = 0.0f;
    obj.color = colors_[slot];
    obj.occluded = false;
    obj.template_area = safe_roi.area();

    // Initialize position at center of bbox
    obj.pos_x = safe_roi.x + safe_roi.width / 2.0f;
    obj.pos_y = safe_roi.y + safe_roi.height / 2.0f;

    // Extract ORB features from template
    obj.keypoints.clear();
    obj.orb_detector->detectAndCompute(template_gray, cv::noArray(), obj.keypoints, obj.descriptors);
    spdlog::debug("[MultiObjectTracker] Extracted {} ORB features for object {}", obj.keypoints.size(), slot);

    // Initialize Kalman filter
    obj.init_kalman(obj.pos_x, obj.pos_y, 0.0f, 0.0f);

    active_count_++;
    spdlog::info("[MultiObjectTracker] Object {} started at ({}, {}) {}x{} - Total: {}/{}", slot, safe_roi.x,
                 safe_roi.y, safe_roi.width, safe_roi.height, active_count_, MAX_OBJECTS);

    // Log tracking start
    if (logger_)
    {
      logger_->log_track_start(slot, af::now_seconds());
    }

    return true;
  }

  void reset() override
  {
    for (int i = 0; i < MAX_OBJECTS; ++i)
    {
      objects_[i].active = false;
    }
    active_count_ = 0;
    spdlog::info("[MultiObjectTracker] Reset all objects");
  }

  bool is_tracking() const override
  {
    for (int i = 0; i < MAX_OBJECTS; ++i)
    {
      if (objects_[i].active)
        return true;
    }
    return false;
  }

  int get_active_count() const { return active_count_; }

  std::vector<Detection> detect(const Frame& frame) override
  {
    std::vector<Detection> all_detections;

    if (frame.img.empty() || !is_tracking())
    {
      return all_detections;
    }

    cv::Mat gray_frame;
    if (frame.img.channels() == 3)
    {
      cv::cvtColor(frame.img, gray_frame, cv::COLOR_BGR2GRAY);
    }
    else if (frame.img.channels() == 1)
    {
      gray_frame = frame.img;
    }
    else
    {
      return all_detections;
    }

    // Track each active object using hybrid approach
    for (int obj_idx = 0; obj_idx < MAX_OBJECTS; ++obj_idx)
    {
      auto& obj = objects_[obj_idx];
      if (!obj.active)
        continue;

      // Start timing for performance measurement
      auto t_start = std::chrono::high_resolution_clock::now();

      // ===== STEP 1: Kalman Prediction =====
      cv::Point2f predicted_pos = obj.predict_kalman();

      // ===== STEP 2: Template Matching (OPTIMIZED) =====
      float template_score = 0.0f;
      cv::Point2f template_pos = predicted_pos;
      cv::Rect template_bbox = obj.bbox;
      float best_scale = obj.scale;

      {
        // OPTIMIZATION: Adaptive scales based on object size
        // Large objects (>15000px) use fewer scales to save processing time
        std::vector<float> scales;
        if (obj.template_area > 15000)
          scales = {0.9f, 1.0f, 1.1f}; // 3 scales for large objects
        else if (obj.template_area > 8000)
          scales = {0.8f, 0.9f, 1.0f, 1.1f, 1.2f}; // 5 scales for medium objects
        else
          scales = {0.7f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f, 1.3f}; // 7 scales for small objects

        float best_template_score = 0.0f;
        cv::Point best_loc;

        // OPTIMIZATION: Tighter search region (1.5x instead of 2.5x)
        float search_expansion =
            1.5f +
            std::min(2.0f,
                     std::sqrt(obj.velocity_x * obj.velocity_x + obj.velocity_y * obj.velocity_y) * 0.05f);
        int search_w = static_cast<int>(obj.bbox.width * search_expansion);
        int search_h = static_cast<int>(obj.bbox.height * search_expansion);
        int search_x = static_cast<int>(predicted_pos.x - search_w / 2);
        int search_y = static_cast<int>(predicted_pos.y - search_h / 2);

        // Clamp to frame bounds
        search_x = std::max(0, std::min(search_x, gray_frame.cols - 1));
        search_y = std::max(0, std::min(search_y, gray_frame.rows - 1));
        search_w = std::min(search_w, gray_frame.cols - search_x);
        search_h = std::min(search_h, gray_frame.rows - search_y);

        if (search_w >= obj.template_gray.cols + 10 && search_h >= obj.template_gray.rows + 10)
        {
          cv::Rect search_roi(search_x, search_y, search_w, search_h);
          cv::Mat search_region = gray_frame(search_roi);

          // Multi-scale template matching
          for (float scale : scales)
          {
            cv::Mat scaled_template;
            if (std::abs(scale - 1.0f) < 0.01f)
              scaled_template = obj.template_gray;
            else
              cv::resize(obj.template_gray, scaled_template, cv::Size(), scale, scale);

            if (scaled_template.cols > search_region.cols || scaled_template.rows > search_region.rows)
              continue;

            cv::Mat result;
            cv::matchTemplate(search_region, scaled_template, result, cv::TM_CCOEFF_NORMED);

            double min_val, max_val;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);

            if (max_val > best_template_score)
            {
              best_template_score = static_cast<float>(max_val);
              best_loc = max_loc;
              best_scale = scale;
            }
          }

          if (best_template_score > 0.3f)
          {
            template_score = best_template_score;
            int new_w = static_cast<int>(obj.template_gray.cols * best_scale);
            int new_h = static_cast<int>(obj.template_gray.rows * best_scale);
            int new_x = search_roi.x + best_loc.x;
            int new_y = search_roi.y + best_loc.y;
            template_bbox = cv::Rect(new_x, new_y, new_w, new_h);
            template_pos = cv::Point2f(new_x + new_w / 2.0f, new_y + new_h / 2.0f);
          }
        }
      }

      // ===== STEP 3: Feature Matching (ORB - OPTIMIZED) =====
      float feature_score = 0.0f;
      cv::Point2f feature_pos = predicted_pos;
      int good_matches = 0;

      // OPTIMIZATION: Skip feature matching on alternating frames for large objects (>10000px)
      bool skip_features = (obj.template_area > 10000) && (frame_count_ % 2 == 0);

      if (!skip_features && !obj.descriptors.empty() && template_score > 0.25f)
      {
        // Extract features from current search region
        cv::Rect feature_roi = template_bbox & cv::Rect(0, 0, gray_frame.cols, gray_frame.rows);
        if (feature_roi.area() > 100)
        {
          cv::Mat roi_gray = gray_frame(feature_roi);
          std::vector<cv::KeyPoint> current_keypoints;
          cv::Mat current_descriptors;
          obj.orb_detector->detectAndCompute(roi_gray, cv::noArray(), current_keypoints, current_descriptors);

          if (!current_descriptors.empty())
          {
            // Match features using BFMatcher with Hamming distance
            cv::BFMatcher matcher(cv::NORM_HAMMING, true);
            std::vector<cv::DMatch> matches;
            matcher.match(obj.descriptors, current_descriptors, matches);

            // Filter good matches (distance < threshold)
            std::vector<cv::DMatch> good_feature_matches;
            for (const auto& m : matches)
            {
              if (m.distance < 50.0f) // Hamming distance threshold
              {
                good_feature_matches.push_back(m);
              }
            }

            good_matches = static_cast<int>(good_feature_matches.size());

            if (good_matches >= 5)
            {
              // Compute center of mass of matched keypoints
              float sum_x = 0.0f, sum_y = 0.0f;
              for (const auto& m : good_feature_matches)
              {
                const auto& kp = current_keypoints[m.trainIdx];
                sum_x += kp.pt.x;
                sum_y += kp.pt.y;
              }
              feature_pos =
                  cv::Point2f(feature_roi.x + sum_x / good_matches, feature_roi.y + sum_y / good_matches);
              feature_score = std::min(1.0f, good_matches / 50.0f);
            }
          }
        }
      }

      // ===== STEP 4: Hybrid Fusion & Quality Scoring =====
      float overall_quality = 0.0f;
      cv::Point2f final_pos;
      cv::Rect final_bbox;

      // Weight the tracking methods based on their confidence
      float template_weight = template_score;
      float feature_weight = feature_score * 0.8f;                   // Features slightly less reliable
      float kalman_weight = 0.3f * (1.0f - obj.lost_frames / 30.0f); // Decay with lost frames

      float total_weight = template_weight + feature_weight + kalman_weight;

      if (total_weight > 0.5f)
      {
        // Weighted fusion of all tracking sources
        final_pos.x = (template_pos.x * template_weight + feature_pos.x * feature_weight +
                       predicted_pos.x * kalman_weight) /
                      total_weight;
        final_pos.y = (template_pos.y * template_weight + feature_pos.y * feature_weight +
                       predicted_pos.y * kalman_weight) /
                      total_weight;

        final_bbox = template_bbox;
        final_bbox.x = static_cast<int>(final_pos.x - final_bbox.width / 2.0f);
        final_bbox.y = static_cast<int>(final_pos.y - final_bbox.height / 2.0f);

        // Overall quality is combination of all scores
        overall_quality =
            (template_score * 0.5f + feature_score * 0.3f + std::min(1.0f, total_weight / 2.0f) * 0.2f);

        // Update Kalman filter with fused position
        obj.update_kalman(final_pos.x, final_pos.y);

        // Update object state
        obj.bbox = final_bbox;
        obj.scale = best_scale;
        obj.template_match_score = template_score;
        obj.feature_match_score = feature_score;
        obj.overall_quality = overall_quality;
        obj.good_feature_matches = good_matches;
        obj.lost_frames = 0;
        obj.tracked_frames++;
        obj.occluded = false;

        // Record processing time
        auto t_end = std::chrono::high_resolution_clock::now();
        obj.last_processing_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        // Log successful tracking
        if (logger_)
        {
          logger_->log_frame(obj.id, frame.timestamp_s, overall_quality, template_score, feature_score,
                             good_matches, 0, obj.last_processing_ms);
        }

        // Adaptive template update (only when tracking quality is high)
        if (overall_quality > 0.7f && frame_count_ % 10 == 0)
        {
          cv::Rect update_roi = final_bbox & cv::Rect(0, 0, gray_frame.cols, gray_frame.rows);
          if (update_roi.area() > 100)
          {
            cv::Mat new_template;
            cv::resize(gray_frame(update_roi), new_template, obj.template_gray.size());
            obj.template_gray = 0.85f * obj.template_gray + 0.15f * new_template;

            // Occasionally update ORB features too
            if (frame_count_ % 30 == 0)
            {
              obj.orb_detector->detectAndCompute(new_template, cv::noArray(), obj.keypoints, obj.descriptors);
            }
          }
        }

        // Create detection
        Detection det;
        det.id = obj.id;
        det.bbox = final_bbox;
        det.centroid = final_pos;
        det.score = overall_quality;
        all_detections.push_back(det);
      }
      else
      {
        // Track lost - handle gracefully
        obj.lost_frames++;
        obj.occluded = (obj.lost_frames > 5 && obj.lost_frames < 20);

        // Give up after too many lost frames
        if (obj.lost_frames > 45)
        {
          std::string loss_reason =
              template_score < 0.3f ? "low_template_score"
                                    : (feature_score < 0.2f ? "low_feature_score" : "prolonged_occlusion");

          if (logger_)
          {
            logger_->log_loss(obj.id, frame.timestamp_s, overall_quality, loss_reason);
          }

          spdlog::warn("[MultiObjectTracker] Lost object {} (quality: {:.2f})", obj.id, overall_quality);
          obj.active = false;
          active_count_--;
        }
        else if (obj.occluded)
        {
          // Log occlusion event
          if (logger_ && obj.lost_frames == 6) // Log once when first occluded
          {
            logger_->log_occlusion(obj.id, frame.timestamp_s, obj.lost_frames);
          }

          // During occlusion, continue with Kalman prediction only
          obj.bbox.x = static_cast<int>(predicted_pos.x - obj.bbox.width / 2.0f);
          obj.bbox.y = static_cast<int>(predicted_pos.y - obj.bbox.height / 2.0f);

          Detection det;
          det.id = obj.id;
          det.bbox = obj.bbox;
          det.centroid = predicted_pos;
          det.score = 0.3f; // Low confidence during occlusion
          det.validity = Validity::Predicted;
          all_detections.push_back(det);
        }
      }
    }

    frame_count_++;
    return all_detections;
  }

  const TrackedObject& get_object(int idx) const { return objects_[idx]; }

  const char* name() const override { return "MultiObjectTracker"; }

private:
  TrackedObject objects_[MAX_OBJECTS];
  std::vector<cv::Scalar> colors_;
  int active_count_{0};
  int frame_count_{0};
  std::unique_ptr<PerformanceLogger> logger_;
};

} // namespace

std::unique_ptr<ITargetTracker> make_template_tracker(const std::string& perf_csv_path)
{
  return std::make_unique<InteractiveTrackerDetector>(perf_csv_path);
}

} // namespace af
