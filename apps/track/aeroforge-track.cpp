#include "aeroforge/core.hpp"
#include "aeroforge/config.hpp"
#include "aeroforge/dji_iface.hpp"
#include "aeroforge/plugin.hpp"
#include "aeroforge/math.hpp"

#ifdef AEROFORGE_WITH_IMGUI
#include "aeroforge/ui.hpp"
#include <GLFW/glfw3.h>
#include <imgui.h>
#endif

#include <spdlog/spdlog.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <cmath>

// ColorBall Detector Implementation
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
    (void)frame;
  }

  std::vector<Detection> detect(const Frame& frame) override
  {
    if (frame.img.empty())
    {
      return {};
    }

    cv::Mat hsv;
    cv::cvtColor(frame.img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_lo_, hsv_hi_, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
      return {};
    }

    auto it = std::max_element(contours.begin(), contours.end(),
                               [](const auto& a, const auto& b)
                               { return cv::contourArea(a) < cv::contourArea(b); });

    double area = cv::contourArea(*it);
    if (area < min_area_px_)
    {
      return {};
    }

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*it, center, radius);

    Detection det;
    det.id = 0;
    det.centroid = center;
    det.bbox = cv::Rect(static_cast<int>(center.x - radius),
                        static_cast<int>(center.y - radius), static_cast<int>(2 * radius),
                        static_cast<int>(2 * radius));
    det.score = static_cast<float>(area / (CV_PI * radius * radius));

    return {det};
  }

  const char* name() const override { return "ColorBallDetector"; }

private:
  cv::Scalar hsv_lo_;
  cv::Scalar hsv_hi_;
  double min_area_px_;
};

// Kalman Tracker Implementation
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
      if (initialized_)
      {
        filter_.predict(dt);
      }
      return {};
    }

    const auto& det = detections[0];

    if (!initialized_)
    {
      filter_.init(det.centroid.x, det.centroid.y);
      initialized_ = true;
      spdlog::debug("[KalmanTracker] Initialized at ({:.1f}, {:.1f})", det.centroid.x,
                    det.centroid.y);
    }
    else
    {
      filter_.predict(dt);
      filter_.update(det.centroid.x, det.centroid.y);
    }

    Detection smoothed = det;
    smoothed.centroid.x = static_cast<float>(filter_.state_x());
    smoothed.centroid.y = static_cast<float>(filter_.state_y());

    int w = smoothed.bbox.width;
    int h = smoothed.bbox.height;
    smoothed.bbox.x = static_cast<int>(smoothed.centroid.x - w / 2.0f);
    smoothed.bbox.y = static_cast<int>(smoothed.centroid.y - h / 2.0f);

    return {smoothed};
  }

  const char* name() const override { return "KalmanTracker"; }

private:
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

// Size-Based Estimator Implementation
class SizeBasedEstimator : public IEstimator
{
public:
  SizeBasedEstimator(double object_diameter_m, double fx, double fy, double cx, double cy)
      : object_diameter_m_(object_diameter_m), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
    spdlog::info("[SizeBasedEstimator] object_diameter={:.2f}m, fx={:.1f}, fy={:.1f}",
                 object_diameter_m, fx, fy);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Pose3D> estimate(const Detection& det, const Frame& frame,
                                 const Telemetry& telemetry) override
  {
    (void)telemetry;

    if (det.bbox.width <= 0 || det.bbox.height <= 0)
    {
      return std::nullopt;
    }

    double d_pixels = (det.bbox.width + det.bbox.height) / 2.0;
    double range_m = (fx_ * object_diameter_m_) / d_pixels;

    double u = det.centroid.x;
    double v = det.centroid.y;

    double x_norm = (u - cx_) / fx_;
    double y_norm = (v - cy_) / fy_;

    Pose3D pose;
    pose.position_m.x() = x_norm * range_m;
    pose.position_m.y() = y_norm * range_m;
    pose.position_m.z() = range_m;
    pose.timestamp_s = frame.timestamp_s;

    return pose;
  }

  const char* name() const override { return "SizeBasedEstimator"; }

private:
  double object_diameter_m_;
  double fx_, fy_, cx_, cy_;
};

// PID Velocity Controller Implementation
class PIDVelocityController : public IController
{
public:
  PIDVelocityController(const Eigen::Vector3d& desired_offset_m, const Eigen::Vector3d& kp,
                        const Eigen::Vector3d& ki, const Eigen::Vector3d& kd,
                        double max_speed_mps)
      : desired_offset_m_(desired_offset_m),
        pid_(kp, ki, kd, max_speed_mps),
        max_speed_mps_(max_speed_mps)
  {
    spdlog::info("[PIDVelocityController] desired_offset=[{:.1f}, {:.1f}, {:.1f}], "
                 "kp=[{:.2f}, {:.2f}, {:.2f}], max_speed={:.1f}",
                 desired_offset_m.x(), desired_offset_m.y(), desired_offset_m.z(), kp.x(),
                 kp.y(), kp.z(), max_speed_mps);
  }

  void process(Frame& frame) override { (void)frame; }

  std::optional<Eigen::Vector3d> velocity_cmd(const Pose3D& target_rel,
                                               const Telemetry& drone_state) override
  {
    (void)drone_state;

    Eigen::Vector3d error = target_rel.position_m - desired_offset_m_;

    double dt = 0.033; // ~30 Hz
    Eigen::Vector3d cmd = pid_.compute(error, dt);

    double mag = cmd.norm();
    if (mag > max_speed_mps_)
    {
      cmd = cmd / mag * max_speed_mps_;
    }

    spdlog::debug("[PIDVelocityController] error=[{:.2f}, {:.2f}, {:.2f}], cmd=[{:.2f}, "
                  "{:.2f}, {:.2f}]",
                  error.x(), error.y(), error.z(), cmd.x(), cmd.y(), cmd.z());

    return cmd;
  }

  const char* name() const override { return "PIDVelocityController"; }

private:
  Eigen::Vector3d desired_offset_m_;
  PIDController pid_;
  double max_speed_mps_;
};

// Interactive Template Matching Tracker
class InteractiveTrackerDetector : public IDetector
{
public:
  InteractiveTrackerDetector()
  {
    spdlog::info("[InteractiveTrackerDetector] Initialized. Click and drag to select target.");
  }

  void process(Frame& frame) override { (void)frame; }

  bool set_roi(const cv::Rect& roi, const cv::Mat& frame)
  {
    if (roi.width <= 0 || roi.height <= 0 || roi.area() < 100)
    {
      spdlog::warn("[InteractiveTrackerDetector] ROI too small");
      return false;
    }

    // Ensure ROI is within frame bounds
    cv::Rect safe_roi = roi & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_roi.area() < 100)
    {
      return false;
    }

    // Extract template
    template_ = frame(safe_roi).clone();
    current_bbox_ = safe_roi;
    is_tracking_ = true;
    search_window_scale_ = 2.0f;

    spdlog::info("[InteractiveTrackerDetector] Tracking initialized at ({}, {}) {}x{}",
                 safe_roi.x, safe_roi.y, safe_roi.width, safe_roi.height);
    return true;
  }

  void reset()
  {
    template_ = cv::Mat();
    is_tracking_ = false;
    spdlog::info("[InteractiveTrackerDetector] Tracking reset");
  }

  bool is_tracking() const { return is_tracking_; }

  std::vector<Detection> detect(const Frame& frame) override
  {
    if (!is_tracking_ || template_.empty())
    {
      return {};
    }

    // Define search region around last known position with proper bounds checking
    int search_w = static_cast<int>(current_bbox_.width * search_window_scale_);
    int search_h = static_cast<int>(current_bbox_.height * search_window_scale_);

    int center_x = current_bbox_.x + current_bbox_.width / 2;
    int center_y = current_bbox_.y + current_bbox_.height / 2;

    int search_x = center_x - search_w / 2;
    int search_y = center_y - search_h / 2;

    // Clamp to frame bounds
    search_x = std::max(0, std::min(search_x, frame.img.cols - 1));
    search_y = std::max(0, std::min(search_y, frame.img.rows - 1));

    // Adjust width and height to stay within bounds
    search_w = std::min(search_w, frame.img.cols - search_x);
    search_h = std::min(search_h, frame.img.rows - search_y);

    // Ensure minimum search area
    if (search_w < template_.cols + 10 || search_h < template_.rows + 10)
    {
      spdlog::warn("[InteractiveTrackerDetector] Search region too small, tracking lost");
      is_tracking_ = false;
      return {};
    }

    cv::Rect search_roi(search_x, search_y, search_w, search_h);
    cv::Mat search_region = frame.img(search_roi);

    // Template matching
    cv::Mat result;
    cv::matchTemplate(search_region, template_, result, cv::TM_CCOEFF_NORMED);

    double min_val, max_val;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);

    // Check if match quality is good enough
    if (max_val < 0.5)
    {
      spdlog::warn("[InteractiveTrackerDetector] Tracking lost (score: {:.2f})", max_val);
      is_tracking_ = false;
      return {};
    }

    // Update bbox in global coordinates
    current_bbox_.x = search_roi.x + max_loc.x;
    current_bbox_.y = search_roi.y + max_loc.y;
    current_bbox_.width = template_.cols;
    current_bbox_.height = template_.rows;

    Detection det;
    det.id = 0;
    det.bbox = current_bbox_;
    det.centroid = cv::Point2f(current_bbox_.x + current_bbox_.width / 2.0f,
                                current_bbox_.y + current_bbox_.height / 2.0f);
    det.score = static_cast<float>(max_val);

    return {det};
  }

  const char* name() const override { return "InteractiveTrackerDetector"; }

private:
  cv::Mat template_;
  cv::Rect current_bbox_;
  bool is_tracking_{false};
  float search_window_scale_{2.0f};
};

} // namespace af

// Mouse selection state
struct MouseSelection
{
  bool is_selecting{false};
  bool selection_complete{false};
  cv::Point start_point;
  cv::Point end_point;
  cv::Rect roi;
} g_mouse_selection;

void print_usage(const char* prog_name)
{
  spdlog::info("Usage: {} --config <path-to-yaml>", prog_name);
}

int main(int argc, char** argv)
{
  spdlog::set_level(spdlog::level::info);
  spdlog::info("AeroForge-Track v0.1.0");

  // Parse arguments
  std::string config_path;
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc)
    {
      config_path = argv[i + 1];
      ++i;
    }
    else if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      return 0;
    }
  }

  if (config_path.empty())
  {
    spdlog::error("No config file specified. Use --config <path>");
    print_usage(argv[0]);
    return 1;
  }

  // Load config
  spdlog::info("Loading config: {}", config_path);
  af::PipelineConfig config;
  try
  {
    config = af::load_config(config_path);
  }
  catch (const std::exception& e)
  {
    spdlog::error("Failed to load config: {}", e.what());
    return 1;
  }

  spdlog::info("Config loaded. Input type: {}, modules: {}, sinks: {}", config.input.type,
               config.modules.size(), config.sinks.size());

  // Build pipeline from config
  std::unique_ptr<af::InteractiveTrackerDetector> interactive_detector = std::make_unique<af::InteractiveTrackerDetector>();
  af::IDetector* detector = interactive_detector.get();

  std::unique_ptr<af::ITracker> tracker;
  std::unique_ptr<af::IEstimator> estimator;
  std::unique_ptr<af::IController> controller;

  for (const auto& mod : config.modules)
  {
    if (mod.type == "colorball" || mod.type == "interactive")
    {
      // Using interactive tracker - already created
      spdlog::info("Interactive tracker detector ready");
    }
    else if (mod.type == "kalman")
    {
      double q = af::get_param<double>(mod.params, "q", 0.02);
      double r = af::get_param<double>(mod.params, "r", 0.20);

      tracker = std::make_unique<af::KalmanTracker>(q, r);
      spdlog::info("Kalman tracker initialized");
    }
    else if (mod.type == "size_based")
    {
      double obj_diam = af::get_param<double>(mod.params, "object_diameter_m", 0.20);
      double fx = af::get_param<double>(mod.params, "fx", 920.0);
      double fy = af::get_param<double>(mod.params, "fy", 920.0);
      double cx = af::get_param<double>(mod.params, "cx", 640.0);
      double cy = af::get_param<double>(mod.params, "cy", 360.0);

      estimator = std::make_unique<af::SizeBasedEstimator>(obj_diam, fx, fy, cx, cy);
      spdlog::info("Size-based estimator initialized");
    }
    else if (mod.type == "pid_velocity")
    {
      auto offset_vec = af::get_param<std::vector<double>>(mod.params, "desired_offset_m", {-3.0, 0.0, 0.0});
      auto kp_vec = af::get_param<std::vector<double>>(mod.params, "kp", {0.8, 0.8, 0.6});
      auto ki_vec = af::get_param<std::vector<double>>(mod.params, "ki", {0.0, 0.0, 0.0});
      auto kd_vec = af::get_param<std::vector<double>>(mod.params, "kd", {0.1, 0.1, 0.08});
      double max_speed = af::get_param<double>(mod.params, "max_speed_mps", 3.0);

      Eigen::Vector3d offset(offset_vec[0], offset_vec[1], offset_vec[2]);
      Eigen::Vector3d kp(kp_vec[0], kp_vec[1], kp_vec[2]);
      Eigen::Vector3d ki(ki_vec[0], ki_vec[1], ki_vec[2]);
      Eigen::Vector3d kd(kd_vec[0], kd_vec[1], kd_vec[2]);

      controller = std::make_unique<af::PIDVelocityController>(offset, kp, ki, kd, max_speed);
      spdlog::info("PID velocity controller initialized");
    }
  }

  if (!detector)
  {
    spdlog::warn("No detector configured, tracking will not work");
  }
  if (!tracker)
  {
    spdlog::warn("No tracker configured");
  }
  if (!estimator)
  {
    spdlog::warn("No estimator configured");
  }
  if (!controller)
  {
    spdlog::warn("No controller configured");
  }

  if (config.input.type != "webcam")
  {
    spdlog::error("Only webcam input is currently supported");
    return 1;
  }

  cv::VideoCapture cap(config.input.device);
  if (!cap.isOpened())
  {
    spdlog::error("Failed to open webcam {}", config.input.device);
    return 1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, config.input.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.input.height);
  cap.set(cv::CAP_PROP_FPS, config.input.fps);

  spdlog::info("Webcam opened: {}x{} @ {} FPS",
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
               cap.get(cv::CAP_PROP_FPS));

  // Initialize mock drone
  auto drone = af::make_mock_drone();
  drone->connect();

#ifdef AEROFORGE_WITH_IMGUI
  // Initialize HUD
  af::HUDConfig hud_config;
  hud_config.show_fps = true;
  hud_config.show_telemetry = true;
  hud_config.show_timings = false;
  hud_config.draw_bbox = true;

  af::HUD hud(hud_config);
  if (!hud.init(config.input.width, config.input.height, "AeroForge-Track"))
  {
    spdlog::error("Failed to initialize HUD");
    return 1;
  }

  // FPS counter
  double fps = 0.0;
  auto last_time = std::chrono::high_resolution_clock::now();
  int frame_count = 0;

  bool safety_enabled = false;
  double last_frame_time_s = af::now_seconds();

  // Pipeline state
  std::optional<af::Pose3D> current_pose;
  std::optional<Eigen::Vector3d> current_velocity_cmd;

  // Main loop
  spdlog::info("===========================================");
  spdlog::info("  CONTROLS:");
  spdlog::info("  - Click and drag on camera to select target");
  spdlog::info("  - R: Reset tracking");
  spdlog::info("  - F: Toggle safety/control");
  spdlog::info("  - SPACE: Emergency stop");
  spdlog::info("  - ESC: Exit");
  spdlog::info("===========================================");

  while (!hud.should_close())
  {
    hud.begin_frame();

    // Check safety keys
    if (hud.is_key_pressed(GLFW_KEY_ESCAPE))
    {
      spdlog::info("ESC pressed, exiting...");
      break;
    }
    if (hud.is_key_pressed(GLFW_KEY_SPACE))
    {
      spdlog::warn("E-STOP pressed!");
      safety_enabled = false;
      drone->hold();
    }
    if (hud.is_key_pressed(GLFW_KEY_F))
    {
      safety_enabled = !safety_enabled;
      spdlog::info("Safety enabled: {}", safety_enabled);
    }
    if (hud.is_key_pressed(GLFW_KEY_R))
    {
      interactive_detector->reset();
      g_mouse_selection = MouseSelection();
      spdlog::info("Tracking reset");
    }

    // Capture frame
    af::Frame frame;
    bool success = cap.read(frame.img);
    if (!success)
    {
      spdlog::warn("Failed to read frame");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    frame.seq = frame_count++;
    frame.timestamp_s = af::now_seconds();

    // Update FPS
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - last_time;
    if (elapsed.count() >= 1.0)
    {
      fps = frame_count / elapsed.count();
      frame_count = 0;
      last_time = now;
    }

    // Read telemetry
    auto telemetry = drone->read_telemetry();

    // Compute frame dt
    double current_time_s = frame.timestamp_s;
    double dt = current_time_s - last_frame_time_s;
    last_frame_time_s = current_time_s;

    // Handle selection completion
    if (g_mouse_selection.selection_complete)
    {
      interactive_detector->set_roi(g_mouse_selection.roi, frame.img);
      g_mouse_selection.selection_complete = false;
      g_mouse_selection.is_selecting = false;
    }

    // Run full pipeline: detect → track → estimate → control
    std::vector<af::Detection> detections;
    std::vector<af::Detection> tracked_detections;

    // 1. Detection
    if (detector)
    {
      detections = detector->detect(frame);
    }

    // 2. Tracking
    if (tracker && dt > 0.0 && dt < 1.0)
    {
      tracked_detections = tracker->update(dt, detections);
    }
    else
    {
      tracked_detections = detections;
    }

    // 3. Pose Estimation
    current_pose.reset();
    if (estimator && !tracked_detections.empty())
    {
      current_pose = estimator->estimate(tracked_detections[0], frame, telemetry);
    }

    // 4. Control
    current_velocity_cmd.reset();
    if (controller && current_pose.has_value() && safety_enabled)
    {
      current_velocity_cmd = controller->velocity_cmd(current_pose.value(), telemetry);

      // Send velocity command to drone (only when safety is enabled)
      if (current_velocity_cmd.has_value())
      {
        auto cmd = current_velocity_cmd.value();
        drone->send_velocity_cmd(cmd);
      }
    }

    // Draw tracking visualization on frame
    if (!tracked_detections.empty())
    {
      const auto& det = tracked_detections[0];
      // Draw bounding box (cyan for active tracking)
      cv::rectangle(frame.img, det.bbox, cv::Scalar(255, 255, 0), 3);
      // Draw crosshair at centroid
      int cx = static_cast<int>(det.centroid.x);
      int cy = static_cast<int>(det.centroid.y);
      cv::line(frame.img, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy), cv::Scalar(0, 0, 255), 2);
      cv::line(frame.img, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20), cv::Scalar(0, 0, 255), 2);
    }

    // Draw selection rectangle if selecting
    if (g_mouse_selection.is_selecting)
    {
      cv::rectangle(frame.img, g_mouse_selection.start_point, g_mouse_selection.end_point,
                    cv::Scalar(0, 255, 0), 2);
    }

    // === IMPROVED UI LAYOUT ===

    // Main camera feed window (larger, positioned at top-left)
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(config.input.width + 20.0f, config.input.height + 70.0f), ImGuiCond_Always);
    ImGui::Begin("Camera Feed", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    // Mouse interaction for ROI selection
    ImVec2 img_pos = ImGui::GetCursorScreenPos();
    ImVec2 img_size(static_cast<float>(frame.img.cols), static_cast<float>(frame.img.rows));

    // Upload and display frame
    hud.render_frame(frame);

    // Handle mouse clicks on the image for ROI selection
    if (ImGui::IsWindowHovered() && !interactive_detector->is_tracking())
    {
      if (ImGui::IsMouseClicked(0))
      {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        g_mouse_selection.start_point = cv::Point(
          static_cast<int>(mouse_pos.x - img_pos.x),
          static_cast<int>(mouse_pos.y - img_pos.y)
        );
        g_mouse_selection.is_selecting = true;
      }
      else if (g_mouse_selection.is_selecting && ImGui::IsMouseDown(0))
      {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        g_mouse_selection.end_point = cv::Point(
          static_cast<int>(mouse_pos.x - img_pos.x),
          static_cast<int>(mouse_pos.y - img_pos.y)
        );
      }
      else if (g_mouse_selection.is_selecting && ImGui::IsMouseReleased(0))
      {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        g_mouse_selection.end_point = cv::Point(
          static_cast<int>(mouse_pos.x - img_pos.x),
          static_cast<int>(mouse_pos.y - img_pos.y)
        );

        int x = std::min(g_mouse_selection.start_point.x, g_mouse_selection.end_point.x);
        int y = std::min(g_mouse_selection.start_point.y, g_mouse_selection.end_point.y);
        int w = std::abs(g_mouse_selection.end_point.x - g_mouse_selection.start_point.x);
        int h = std::abs(g_mouse_selection.end_point.y - g_mouse_selection.start_point.y);

        g_mouse_selection.roi = cv::Rect(x, y, w, h);
        g_mouse_selection.selection_complete = true;
      }
    }

    // Status text
    if (!interactive_detector->is_tracking())
    {
      ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Click and drag to select target");
    }
    else
    {
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Tracking active");
    }
    ImGui::End();

    // FPS overlay (top-right)
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(config.input.width + 30), 10.0f), ImGuiCond_Always);
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "FPS: %.1f", fps);
    ImGui::End();

    // Safety status (below FPS)
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(config.input.width + 30), 80.0f), ImGuiCond_Always);
    ImGui::Begin("Safety", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
    if (safety_enabled)
    {
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "CONTROL: ENABLED");
    }
    else
    {
      ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "CONTROL: DISABLED");
    }
    ImGui::Text("Press F to toggle");
    ImGui::End();

    // Telemetry panel (right side, below safety)
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(config.input.width + 30), 170.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350.0f, 300.0f), ImGuiCond_Always);
    ImGui::Begin("Telemetry", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::Text("Altitude:  %.2f m", telemetry.altitude_m);
    ImGui::Text("Speed:     %.2f m/s", telemetry.speed_mps);
    ImGui::Text("Yaw:       %.1f deg", telemetry.yaw_deg);
    ImGui::Separator();
    ImGui::Text("Position:");
    ImGui::Text("  X: %7.2f m", telemetry.position_m.x());
    ImGui::Text("  Y: %7.2f m", telemetry.position_m.y());
    ImGui::Text("  Z: %7.2f m", telemetry.position_m.z());
    ImGui::Separator();
    ImGui::Text("Velocity:");
    ImGui::Text("  Vx: %6.2f m/s", telemetry.velocity_mps.x());
    ImGui::Text("  Vy: %6.2f m/s", telemetry.velocity_mps.y());
    ImGui::Text("  Vz: %6.2f m/s", telemetry.velocity_mps.z());
    ImGui::Separator();
    ImGui::Text("GPS: %s", telemetry.gps_valid ? "Valid" : "Invalid");
    ImGui::Text("Satellites: %d", telemetry.satellite_count);
    ImGui::End();

    // Pipeline Status (right side, below telemetry)
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(config.input.width + 30), 480.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350.0f, 260.0f), ImGuiCond_Always);
    ImGui::Begin("Tracking Pipeline", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Detections: %zu", detections.size());
    ImGui::Text("Tracked: %zu", tracked_detections.size());

    ImGui::Separator();
    if (current_pose.has_value())
    {
      auto& pose = current_pose.value();
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Target Position:");
      ImGui::Text("  X: %7.2f m (lateral)", pose.position_m.x());
      ImGui::Text("  Y: %7.2f m (vertical)", pose.position_m.y());
      ImGui::Text("  Z: %7.2f m (forward)", pose.position_m.z());

      double range = pose.position_m.z();
      double horiz_dist = std::sqrt(pose.position_m.x() * pose.position_m.x() +
                                     pose.position_m.z() * pose.position_m.z());
      ImGui::Separator();
      ImGui::Text("Range:     %.2f m", range);
      ImGui::Text("Horiz Dist: %.2f m", horiz_dist);
    }
    else
    {
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No target detected");
    }

    ImGui::Separator();
    if (current_velocity_cmd.has_value())
    {
      auto& cmd = current_velocity_cmd.value();
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Velocity Command:");
      ImGui::Text("  Vx: %6.2f m/s", cmd.x());
      ImGui::Text("  Vy: %6.2f m/s", cmd.y());
      ImGui::Text("  Vz: %6.2f m/s", cmd.z());
      ImGui::Text("  Mag: %.2f m/s", cmd.norm());
    }
    else if (safety_enabled)
    {
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Waiting for target...");
    }
    else
    {
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Control disabled");
    }
    ImGui::End();

    // Controls help (bottom)
    ImGui::SetNextWindowPos(ImVec2(10.0f, static_cast<float>(config.input.height + 90)), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(config.input.width + 20), 110.0f), ImGuiCond_Always);
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextColored(ImVec4(0.5f, 1.0f, 1.0f, 1.0f), "KEYBOARD CONTROLS:");
    ImGui::Columns(2, nullptr, false);
    ImGui::Text("R");       ImGui::NextColumn(); ImGui::Text("Reset tracking"); ImGui::NextColumn();
    ImGui::Text("F");       ImGui::NextColumn(); ImGui::Text("Toggle safety/control"); ImGui::NextColumn();
    ImGui::Text("SPACE");   ImGui::NextColumn(); ImGui::Text("Emergency stop"); ImGui::NextColumn();
    ImGui::Text("ESC");     ImGui::NextColumn(); ImGui::Text("Exit application"); ImGui::NextColumn();
    ImGui::Columns(1);
    ImGui::End();

    hud.end_frame();
  }

  hud.shutdown();
#else
  spdlog::error("ImGui HUD not enabled. Rebuild with AEROFORGE_WITH_IMGUI=ON");
  return 1;
#endif

  drone->disconnect();

  return 0;
}
