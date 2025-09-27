#pragma once

#include "core.hpp"
#include "telemetry.hpp"
#include <memory>
#include <string>
#include <vector>

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// HUD Overlay
// ──────────────────────────────────────────────────────────────────────────

struct HUDConfig
{
  bool draw_bbox{true};
  bool show_fps{true};
  bool show_telemetry{true};
  bool show_timings{true};
};

class HUD
{
public:
  explicit HUD(const HUDConfig& config);
  ~HUD();

  bool init(int window_width, int window_height, const std::string& title);
  void shutdown();

  bool should_close() const;
  void begin_frame();
  void end_frame();

  void render_frame(const Frame& frame);
  void render_detections(const std::vector<Detection>& detections);
  void render_fps(double fps);
  void render_telemetry(const Telemetry& tel);
  void render_timings(const std::vector<std::pair<std::string, double>>& timings);
  void render_safety_status(bool enabled, bool geofence_ok);

  bool is_key_pressed(int key) const;
  bool is_key_down(int key) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace af
