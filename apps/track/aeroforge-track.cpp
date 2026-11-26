// AeroForge-Track: webcam / recorded-video tracking with a simulated follow controller.
//
// Two front ends share one engine:
//   GUI      (default)    - ImGui window, click-and-drag target selection
//   headless (--headless) - no window, target from --roi, JSON report on stdout
//
// Velocity commands only ever reach the built-in simulator (see SafetyGate).

#include "aeroforge/config.hpp"
#include "aeroforge/core.hpp"
#include "aeroforge/dji_iface.hpp"
#include "aeroforge/io.hpp"
#include "aeroforge/math.hpp"
#include "aeroforge/pipeline.hpp"
#include "aeroforge/safety.hpp"

#ifdef AEROFORGE_WITH_IMGUI
#include "aeroforge/ui.hpp"
#include <GLFW/glfw3.h>
#include <imgui.h>
#endif

#include <opencv2/imgproc.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

// ──────────────────────────────────────────────────────────────────────────
// Command line
// ──────────────────────────────────────────────────────────────────────────

struct Options
{
  std::string config_path;
  std::string input_override; // video file replacing pipeline.input
  std::string report_path;    // headless: also write the JSON report here
  bool headless{false};
  uint64_t max_frames{0}; // 0 = until the source ends (headless) / window closes (GUI)
  std::optional<cv::Rect> roi;
};

void print_usage(const char* prog)
{
  std::printf(
      "AeroForge-Track %s\n"
      "Usage: %s --config <file.yml> [options]\n"
      "\n"
      "  --config <path>     pipeline configuration (required)\n"
      "  --input <video>     replay this video file instead of pipeline.input\n"
      "  --headless          run without a window and print a JSON report\n"
      "  --roi x,y,w,h       select this region as the target on the first frame\n"
      "  --max-frames <n>    stop after n frames\n"
      "  --report <path>     headless: also write the JSON report to this file\n"
      "  --help              show this text\n",
      AEROFORGE_VERSION, prog);
}

std::optional<cv::Rect> parse_roi(const std::string& text)
{
  // Exactly four comma-separated integers
  long v[4] = {0, 0, 0, 0};
  const char* p = text.c_str();
  for (int i = 0; i < 4; ++i)
  {
    char* end = nullptr;
    v[i] = std::strtol(p, &end, 10);
    if (end == p || v[i] < -100000 || v[i] > 100000)
    {
      return std::nullopt;
    }
    p = end;
    if (i < 3)
    {
      if (*p != ',')
      {
        return std::nullopt;
      }
      ++p;
    }
  }
  if (*p != '\0' || v[2] <= 0 || v[3] <= 0)
  {
    return std::nullopt;
  }
  return cv::Rect(static_cast<int>(v[0]), static_cast<int>(v[1]), static_cast<int>(v[2]),
                  static_cast<int>(v[3]));
}

// Returns false (after printing why) when the arguments are unusable.
bool parse_args(int argc, char** argv, Options& opt, bool& show_help)
{
  show_help = false;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    auto value = [&](const char* name) -> const char*
    {
      if (i + 1 >= argc)
      {
        spdlog::error("{} needs a value", name);
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h")
    {
      show_help = true;
      return true;
    }
    else if (arg == "--headless")
    {
      opt.headless = true;
    }
    else if (arg == "--config")
    {
      const char* v = value("--config");
      if (!v) return false;
      opt.config_path = v;
    }
    else if (arg == "--input")
    {
      const char* v = value("--input");
      if (!v) return false;
      opt.input_override = v;
    }
    else if (arg == "--report")
    {
      const char* v = value("--report");
      if (!v) return false;
      opt.report_path = v;
    }
    else if (arg == "--max-frames")
    {
      const char* v = value("--max-frames");
      if (!v) return false;
      char* end = nullptr;
      const long long n = std::strtoll(v, &end, 10);
      if (end == v || *end != '\0' || n <= 0)
      {
        spdlog::error("--max-frames must be a positive integer, got '{}'", v);
        return false;
      }
      opt.max_frames = static_cast<uint64_t>(n);
    }
    else if (arg == "--roi")
    {
      const char* v = value("--roi");
      if (!v) return false;
      opt.roi = parse_roi(v);
      if (!opt.roi)
      {
        spdlog::error("--roi must be x,y,w,h with w and h > 0, got '{}'", v);
        return false;
      }
    }
    else
    {
      spdlog::error("Unknown argument '{}'", arg);
      return false;
    }
  }

  if (opt.config_path.empty())
  {
    spdlog::error("No config file specified. Use --config <path>");
    return false;
  }
  return true;
}

// ──────────────────────────────────────────────────────────────────────────
// Latency statistics
// ──────────────────────────────────────────────────────────────────────────

class LatencyStats
{
public:
  void add(double ms) { samples_.push_back(ms); }

  // Nearest-rank percentile, p in [0, 100]. 0 when empty.
  double percentile(double p) const
  {
    if (samples_.empty())
    {
      return 0.0;
    }
    std::vector<double> sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    const double rank = std::ceil(p / 100.0 * static_cast<double>(sorted.size()));
    const size_t idx = static_cast<size_t>(std::max(1.0, rank)) - 1;
    return sorted[std::min(idx, sorted.size() - 1)];
  }

  std::string json() const
  {
    return fmt::format("{{\"p50\": {:.3f}, \"p95\": {:.3f}, \"p99\": {:.3f}, \"max\": {:.3f}}}",
                       percentile(50), percentile(95), percentile(99), percentile(100));
  }

private:
  std::vector<double> samples_;
};

// ──────────────────────────────────────────────────────────────────────────
// Engine: everything except the user interface
// ──────────────────────────────────────────────────────────────────────────

struct Counters
{
  uint64_t frames{0};
  uint64_t frames_with_target{0};    // any tracked detection
  uint64_t frames_fresh_target{0};   // measured this frame
  uint64_t frames_predicted{0};      // tracker coasting
  uint64_t frames_with_pose{0};
  uint64_t commands_sent{0};
};

struct Tick
{
  af::Frame frame;
  af::Telemetry telemetry;
  af::PipelineOutput out;
  af::SafetyGate::Result gate;
  std::optional<Eigen::Vector3d> cmd_ned;
  double frame_age_ms{0.0};
};

class Engine
{
public:
  explicit Engine(const af::PipelineConfig& config)
      : pipeline_(config),
        source_(af::make_frame_source(config.input)),
        drone_(af::make_mock_drone()),
        gate_(config.safety)
  {
  }

  bool start()
  {
    if (!source_->open())
    {
      return false;
    }
    drone_->connect();
    return true;
  }

  void stop()
  {
    drone_->hold();
    drone_->disconnect();
    source_->close();
  }

  // Reads and processes one frame. Returns false when no frame was available.
  bool tick(Tick& t, bool enable_key_down, bool estop_key_down,
            const std::optional<cv::Rect>& select_roi)
  {
    const af::SafetyGate::State before = gate_.state();
    gate_.update(enable_key_down, estop_key_down);
    if (gate_.state() != af::SafetyGate::State::Enabled &&
        before == af::SafetyGate::State::Enabled)
    {
      drone_->hold(); // control was just withdrawn: brake now, don't wait for the watchdog
    }

    if (!source_->read(t.frame))
    {
      return false;
    }

    // Advance the simulator by the time between frames
    if (have_last_frame_)
    {
      drone_->step(t.frame.timestamp_s - last_frame_time_s_);
    }
    last_frame_time_s_ = t.frame.timestamp_s;
    have_last_frame_ = true;

    t.telemetry = drone_->read_telemetry();

    if (select_roi)
    {
      if (!pipeline_.select_target(*select_roi, t.frame))
      {
        spdlog::warn("Target selection {}x{} at ({}, {}) was rejected", select_roi->width,
                     select_roi->height, select_roi->x, select_roi->y);
      }
    }

    const bool control_enabled = gate_.state() == af::SafetyGate::State::Enabled;
    t.out = pipeline_.step(t.frame, t.telemetry, control_enabled);

    t.gate = gate_.evaluate(t.out.velocity_cmd_body, t.out.pose, t.telemetry,
                            drone_->capabilities(), source_->now_s());
    t.cmd_ned.reset();
    if (t.gate.cmd)
    {
      t.cmd_ned = af::body_to_ned(*t.gate.cmd, t.telemetry.yaw_deg);
      if (drone_->send_velocity_cmd(*t.cmd_ned))
      {
        ++counters_.commands_sent;
      }
    }

    t.frame_age_ms = (source_->now_s() - t.frame.timestamp_s) * 1000.0;

    ++counters_.frames;
    if (!t.out.tracked.empty())
    {
      ++counters_.frames_with_target;
      if (t.out.tracked.front().validity == af::Validity::Fresh)
      {
        ++counters_.frames_fresh_target;
      }
      else
      {
        ++counters_.frames_predicted;
      }
    }
    if (t.out.pose)
    {
      ++counters_.frames_with_pose;
    }
    detect_ms_.add(t.out.timings.detect_ms);
    total_ms_.add(t.out.timings.total_ms());
    return true;
  }

  void reset_tracking() { pipeline_.reset(); }

  af::Pipeline& pipeline() { return pipeline_; }
  af::IFrameSource& source() { return *source_; }
  af::IDrone& drone() { return *drone_; }
  const af::SafetyGate& gate() const { return gate_; }
  const Counters& counters() const { return counters_; }
  const LatencyStats& detect_ms() const { return detect_ms_; }
  const LatencyStats& total_ms() const { return total_ms_; }

private:
  af::Pipeline pipeline_;
  std::unique_ptr<af::IFrameSource> source_;
  std::unique_ptr<af::IDrone> drone_;
  af::SafetyGate gate_;
  Counters counters_;
  LatencyStats detect_ms_;
  LatencyStats total_ms_;
  bool have_last_frame_{false};
  double last_frame_time_s_{0.0};
};

std::string json_escape(const std::string& in)
{
  std::string out;
  for (char c : in)
  {
    if (c == '"' || c == '\\')
    {
      out += '\\';
      out += c;
    }
    else if (static_cast<unsigned char>(c) < 0x20)
    {
      out += ' ';
    }
    else
    {
      out += c;
    }
  }
  return out;
}

std::string build_report(Engine& engine, double wall_s)
{
  const Counters& c = engine.counters();
  const double fps = wall_s > 0.0 ? static_cast<double>(c.frames) / wall_s : 0.0;
  return fmt::format(
      "{{\n"
      "  \"version\": \"{}\",\n"
      "  \"source\": \"{}\",\n"
      "  \"detector\": \"{}\",\n"
      "  \"frames\": {},\n"
      "  \"frames_with_target\": {},\n"
      "  \"frames_fresh_target\": {},\n"
      "  \"frames_predicted\": {},\n"
      "  \"frames_with_pose\": {},\n"
      "  \"commands_sent_to_simulator\": {},\n"
      "  \"failed_reads\": {},\n"
      "  \"wall_time_s\": {:.3f},\n"
      "  \"throughput_fps\": {:.2f},\n"
      "  \"detect_ms\": {},\n"
      "  \"pipeline_ms\": {}\n"
      "}}",
      AEROFORGE_VERSION, json_escape(engine.source().describe()),
      engine.pipeline().detector_name(), c.frames, c.frames_with_target, c.frames_fresh_target,
      c.frames_predicted, c.frames_with_pose, c.commands_sent, engine.source().failed_reads(),
      wall_s, fps, engine.detect_ms().json(), engine.total_ms().json());
}

// ──────────────────────────────────────────────────────────────────────────
// Headless front end
// ──────────────────────────────────────────────────────────────────────────

int run_headless(Engine& engine, const Options& opt)
{
  if (opt.roi && !engine.pipeline().target_tracker())
  {
    spdlog::warn("--roi ignored: detector {} does not take a selected region",
                 engine.pipeline().detector_name());
  }
  if (!opt.roi && engine.pipeline().target_tracker())
  {
    spdlog::warn("No --roi given: the interactive tracker has nothing to follow");
  }

  const double start = af::now_seconds();
  double last_frame_wall = start;
  std::optional<cv::Rect> pending_roi = opt.roi;

  while (opt.max_frames == 0 || engine.counters().frames < opt.max_frames)
  {
    Tick t;
    if (engine.tick(t, false, false, pending_roi))
    {
      pending_roi.reset();
      last_frame_wall = af::now_seconds();
      continue;
    }
    if (engine.source().at_end())
    {
      break;
    }
    // Live source without a frame: wait briefly, give up after 5 s of silence
    if (af::now_seconds() - last_frame_wall > 5.0)
    {
      spdlog::error("No frames from {} for 5 s, giving up", engine.source().describe());
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const std::string report = build_report(engine, af::now_seconds() - start);
  std::printf("%s\n", report.c_str());
  if (!opt.report_path.empty())
  {
    std::ofstream f(opt.report_path, std::ios::out | std::ios::trunc);
    if (!f)
    {
      spdlog::error("Cannot write report to '{}'", opt.report_path);
      return 1;
    }
    f << report << "\n";
  }

  if (engine.counters().frames == 0)
  {
    spdlog::error("No frames were processed");
    return 2;
  }
  return 0;
}

// ──────────────────────────────────────────────────────────────────────────
// GUI front end
// ──────────────────────────────────────────────────────────────────────────

#ifdef AEROFORGE_WITH_IMGUI

struct MouseSelection
{
  bool is_selecting{false};
  cv::Point start_point;
  cv::Point end_point;
};

cv::Scalar color_for(af::Validity v)
{
  return v == af::Validity::Fresh ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
}

void draw_overlays(cv::Mat& img, const std::vector<af::Detection>& tracked,
                   const MouseSelection& mouse)
{
  for (const auto& det : tracked)
  {
    const cv::Scalar color = color_for(det.validity);
    const int thickness = det.score > 0.7f ? 3 : (det.score > 0.4f ? 2 : 1);
    cv::rectangle(img, det.bbox, color, thickness);

    const int cx = static_cast<int>(det.centroid.x);
    const int cy = static_cast<int>(det.centroid.y);
    cv::line(img, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy), color, 2);
    cv::line(img, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20), color, 2);

    char label[64];
    std::snprintf(label, sizeof(label), "ID:%d %.0f%% %s", det.id, det.score * 100.0f,
                  af::to_string(det.validity));
    int baseline = 0;
    const cv::Size text = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const cv::Point pos(det.bbox.x, det.bbox.y - 12);
    cv::rectangle(img, cv::Point(pos.x - 2, pos.y - text.height - 4),
                  cv::Point(pos.x + text.width + 2, pos.y + 4), cv::Scalar(0, 0, 0), -1);
    cv::putText(img, label, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
  }

  if (mouse.is_selecting)
  {
    cv::rectangle(img, mouse.start_point, mouse.end_point, cv::Scalar(0, 255, 0), 2);
  }
}

int run_gui(Engine& engine, const Options& opt, const af::PipelineConfig& config)
{
  cv::Size view = engine.source().frame_size();
  if (view.width <= 0 || view.height <= 0)
  {
    view = cv::Size(config.input.width, config.input.height);
  }

  const float side_w = 370.0f;
  af::HUDConfig hud_config;
  af::HUD hud(hud_config);
  if (!hud.init(view.width + 40 + static_cast<int>(side_w), view.height + 230,
                "AeroForge-Track [SIMULATION]"))
  {
    spdlog::error("Failed to initialize HUD");
    return 1;
  }

  spdlog::info("Controls: drag on the image = select target, R = reset tracking, "
               "F = {} simulated control, SPACE = e-stop, ESC = exit",
               engine.gate().hold_to_enable() ? "hold to enable" : "toggle");

  MouseSelection mouse;
  std::optional<cv::Rect> pending_roi = opt.roi;
  bool prev_reset_down = false;
  int consecutive_failures = 0;

  double fps = 0.0;
  int fps_frames = 0;
  double fps_window_start = af::now_seconds();
  const double wall_start = af::now_seconds();

  Tick t;                 // last successfully processed frame (kept for display)
  bool have_tick = false;
  bool finished = false;  // recorded source reached its end

  while (!hud.should_close())
  {
    hud.begin_frame();

    if (hud.is_key_down(GLFW_KEY_ESCAPE))
    {
      break;
    }
    const bool reset_down = hud.is_key_down(GLFW_KEY_R);
    if (reset_down && !prev_reset_down)
    {
      engine.reset_tracking();
      mouse = MouseSelection();
      spdlog::info("Tracking reset");
    }
    prev_reset_down = reset_down;

    if (!finished && (opt.max_frames == 0 || engine.counters().frames < opt.max_frames))
    {
      // Recorded sources are paced to their own frame rate
      if (!engine.source().is_live() && have_tick)
      {
        const double due = wall_start + t.frame.timestamp_s + 1.0 / engine.source().nominal_fps();
        const double wait = due - af::now_seconds();
        if (wait > 0.0)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(std::min(wait, 0.1)));
        }
      }

      Tick next;
      if (engine.tick(next, hud.is_key_down(GLFW_KEY_F), hud.is_key_down(GLFW_KEY_SPACE),
                      pending_roi))
      {
        pending_roi.reset();
        t = std::move(next);
        have_tick = true;
        consecutive_failures = 0;
        ++fps_frames;
      }
      else if (engine.source().at_end())
      {
        finished = true;
        spdlog::info("End of {}", engine.source().describe());
      }
      else
      {
        ++consecutive_failures;
      }
    }
    else if (opt.max_frames != 0 && engine.counters().frames >= opt.max_frames)
    {
      break;
    }

    const double now = af::now_seconds();
    if (now - fps_window_start >= 1.0)
    {
      fps = fps_frames / (now - fps_window_start);
      fps_frames = 0;
      fps_window_start = now;
    }

    // ── Camera feed ──
    const float img_w = static_cast<float>(have_tick ? t.frame.img.cols : view.width);
    const float img_h = static_cast<float>(have_tick ? t.frame.img.rows : view.height);
    const float right_x = img_w + 30.0f;

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(img_w + 20.0f, img_h + 90.0f), ImGuiCond_Always);
    ImGui::Begin("Camera Feed", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    const ImVec2 img_pos = ImGui::GetCursorScreenPos();
    if (have_tick)
    {
      af::Frame shown;
      shown.seq = t.frame.seq;
      shown.timestamp_s = t.frame.timestamp_s;
      shown.img = t.frame.img.clone(); // overlays never touch the frame the tracker saw
      draw_overlays(shown.img, t.out.tracked, mouse);
      hud.render_frame(shown);
    }

    const bool selectable = engine.pipeline().target_tracker() != nullptr;
    if (selectable && have_tick && ImGui::IsWindowHovered())
    {
      const ImVec2 m = ImGui::GetMousePos();
      const cv::Point p(static_cast<int>(m.x - img_pos.x), static_cast<int>(m.y - img_pos.y));
      if (ImGui::IsMouseClicked(0))
      {
        mouse.start_point = p;
        mouse.end_point = p;
        mouse.is_selecting = true;
      }
      else if (mouse.is_selecting && ImGui::IsMouseDown(0))
      {
        mouse.end_point = p;
      }
    }
    if (mouse.is_selecting && ImGui::IsMouseReleased(0))
    {
      mouse.is_selecting = false;
      const cv::Rect roi(std::min(mouse.start_point.x, mouse.end_point.x),
                         std::min(mouse.start_point.y, mouse.end_point.y),
                         std::abs(mouse.end_point.x - mouse.start_point.x),
                         std::abs(mouse.end_point.y - mouse.start_point.y));
      if (roi.area() > 0)
      {
        pending_roi = roi;
      }
    }

    if (finished)
    {
      ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "End of recording");
    }
    else if (consecutive_failures > 0)
    {
      ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "NO FRAMES from %s (%d failed reads)",
                         engine.source().describe().c_str(), consecutive_failures);
    }
    else if (!selectable)
    {
      ImGui::Text("Detector: %s (automatic)", engine.pipeline().detector_name());
    }
    else if (engine.pipeline().target_tracker()->is_tracking())
    {
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Tracking active (%s) - R to reset",
                         engine.pipeline().detector_name());
    }
    else
    {
      ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Click and drag to select target");
    }
    ImGui::End();

    // ── Performance ──
    ImGui::SetNextWindowPos(ImVec2(right_x, 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(side_w, 150.0f), ImGuiCond_Always);
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "FPS: %.1f", fps);
    ImGui::Text("Source: %s", engine.source().describe().c_str());
    if (have_tick)
    {
      ImGui::Text("Frame %llu, age %.1f ms", static_cast<unsigned long long>(t.frame.seq),
                  t.frame_age_ms);
      ImGui::Text("Detect %.2f  Track %.2f ms", t.out.timings.detect_ms, t.out.timings.track_ms);
      ImGui::Text("Estimate %.2f  Control %.2f ms", t.out.timings.estimate_ms,
                  t.out.timings.control_ms);
    }
    ImGui::Text("Failed reads: %llu",
                static_cast<unsigned long long>(engine.source().failed_reads()));
    ImGui::End();

    // ── Safety ──
    const af::SafetyGate::State state = engine.gate().state();
    ImGui::SetNextWindowPos(ImVec2(right_x, 170.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(side_w, 110.0f), ImGuiCond_Always);
    ImGui::Begin("Safety", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "SIMULATION ONLY - no aircraft");
    const ImVec4 state_color = state == af::SafetyGate::State::Enabled
                                   ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f)
                                   : ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    ImGui::TextColored(state_color, "CONTROL: %s", af::to_string(state));
    ImGui::TextUnformatted(engine.gate().hold_to_enable() ? "Hold F to enable, SPACE = e-stop"
                                                          : "Press F to toggle, SPACE = e-stop");
    if (have_tick && t.gate.reason[0] != '\0')
    {
      ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No command: %s", t.gate.reason);
    }
    ImGui::End();

    // ── Telemetry ──
    ImGui::SetNextWindowPos(ImVec2(right_x, 290.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(side_w, 230.0f), ImGuiCond_Always);
    ImGui::Begin("Telemetry (simulated)", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    if (have_tick)
    {
      const af::Telemetry& tel = t.telemetry;
      ImGui::Text("State:     %s%s", af::to_string(tel.validity), tel.simulated ? " (sim)" : "");
      ImGui::Text("Altitude:  %.2f m", tel.altitude_m);
      ImGui::Text("Speed:     %.2f m/s", tel.speed_mps);
      ImGui::Text("Yaw:       %.1f deg", tel.yaw_deg);
      ImGui::Separator();
      ImGui::Text("Position NED: %6.2f %6.2f %6.2f m", tel.position_m.x(), tel.position_m.y(),
                  tel.position_m.z());
      ImGui::Text("Velocity NED: %6.2f %6.2f %6.2f m/s", tel.velocity_mps.x(),
                  tel.velocity_mps.y(), tel.velocity_mps.z());
      ImGui::Separator();
      ImGui::Text("Geofence: %s", engine.gate().geofence_ok(tel) ? "inside" : "OUTSIDE");
    }
    ImGui::End();

    // ── Tracking pipeline ──
    ImGui::SetNextWindowPos(ImVec2(right_x, 530.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(side_w, 260.0f), ImGuiCond_Always);
    ImGui::Begin("Tracking Pipeline", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    if (have_tick)
    {
      ImGui::Text("Detections: %zu  Tracked: %zu", t.out.detections.size(), t.out.tracked.size());
      if (!t.out.tracked.empty())
      {
        const af::Detection& d = t.out.tracked.front();
        ImGui::Text("Target: %s, confidence %.0f%%", af::to_string(d.validity), d.score * 100.0f);
      }
      else
      {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No target");
      }

      ImGui::Separator();
      if (t.out.pose)
      {
        const af::Pose3D& pose = *t.out.pose;
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Target (camera frame, %s):",
                           af::to_string(pose.validity));
        ImGui::Text("  X: %7.2f m (right)", pose.position_m.x());
        ImGui::Text("  Y: %7.2f m (down)", pose.position_m.y());
        ImGui::Text("  Z: %7.2f m (forward)", pose.position_m.z());
        ImGui::Text("  Range: %.2f m", pose.position_m.norm());
      }
      else if (!engine.pipeline().has_estimator())
      {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No estimator configured");
      }
      else
      {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No position estimate");
      }

      ImGui::Separator();
      if (t.gate.cmd)
      {
        const Eigen::Vector3d& cmd = *t.gate.cmd;
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Simulated velocity command (body):");
        ImGui::Text("  fwd %5.2f  right %5.2f  down %5.2f m/s", cmd.x(), cmd.y(), cmd.z());
        ImGui::Text("  |v| = %.2f m/s", cmd.norm());
      }
      else
      {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No command");
      }
    }
    ImGui::End();

    // ── Controls help ──
    ImGui::SetNextWindowPos(ImVec2(10.0f, img_h + 110.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(img_w + 20.0f, 100.0f), ImGuiCond_Always);
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::Columns(2, nullptr, false);
    ImGui::Text("Drag on image"); ImGui::NextColumn(); ImGui::Text("Select target"); ImGui::NextColumn();
    ImGui::Text("R");             ImGui::NextColumn(); ImGui::Text("Reset tracking"); ImGui::NextColumn();
    ImGui::Text("F / SPACE");     ImGui::NextColumn(); ImGui::Text("Enable simulated control / e-stop"); ImGui::NextColumn();
    ImGui::Text("ESC");           ImGui::NextColumn(); ImGui::Text("Exit"); ImGui::NextColumn();
    ImGui::Columns(1);
    ImGui::End();

    hud.end_frame();
  }

  hud.shutdown();
  return 0;
}

#endif // AEROFORGE_WITH_IMGUI

} // namespace

int main(int argc, char** argv)
{
  // Logs go to stderr so that stdout carries only the headless JSON report
  spdlog::set_default_logger(spdlog::stderr_color_mt("aeroforge"));
  spdlog::set_level(spdlog::level::info);

  Options opt;
  bool show_help = false;
  if (!parse_args(argc, argv, opt, show_help))
  {
    print_usage(argv[0]);
    return 1;
  }
  if (show_help)
  {
    print_usage(argv[0]);
    return 0;
  }

  spdlog::info("AeroForge-Track v{}", AEROFORGE_VERSION);

  int exit_code = 1;
  try
  {
    af::PipelineConfig config = af::load_config(opt.config_path);
    if (!opt.input_override.empty())
    {
      config.input.type = "file";
      config.input.path = opt.input_override;
    }

    Engine engine(config);
    if (!engine.start())
    {
      return 1;
    }

    if (opt.headless)
    {
      exit_code = run_headless(engine, opt);
    }
    else
    {
#ifdef AEROFORGE_WITH_IMGUI
      exit_code = run_gui(engine, opt, config);
#else
      spdlog::error("This build has no GUI (AEROFORGE_WITH_IMGUI=OFF). Use --headless.");
      exit_code = 1;
#endif
    }

    engine.stop();
  }
  catch (const std::exception& e)
  {
    spdlog::error("{}", e.what());
    return 1;
  }

  return exit_code;
}
