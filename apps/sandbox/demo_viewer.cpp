#include "aeroforge/core.hpp"

#ifdef AEROFORGE_WITH_IMGUI
#include "aeroforge/ui.hpp"
#endif

#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
  spdlog::set_level(spdlog::level::info);
  spdlog::info("AeroForge Demo Viewer");

  int device_id = 0;
  if (argc > 1)
  {
    device_id = std::atoi(argv[1]);
  }

  // Open webcam
  cv::VideoCapture cap(device_id);
  if (!cap.isOpened())
  {
    spdlog::error("Failed to open webcam {}", device_id);
    return 1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  cap.set(cv::CAP_PROP_FPS, 30);

  spdlog::info("Webcam opened: {}x{} @ {} FPS",
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
               static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
               cap.get(cv::CAP_PROP_FPS));

#ifdef AEROFORGE_WITH_IMGUI
  // Initialize HUD
  af::HUDConfig hud_config;
  hud_config.show_fps = true;
  hud_config.show_telemetry = false;
  hud_config.show_timings = false;
  hud_config.draw_bbox = false;

  af::HUD hud(hud_config);
  if (!hud.init(1280, 720, "AeroForge - Demo Viewer"))
  {
    spdlog::error("Failed to initialize HUD");
    return 1;
  }

  // FPS counter
  double fps = 0.0;
  auto last_time = std::chrono::high_resolution_clock::now();
  int frame_count = 0;

  // Main loop
  while (!hud.should_close())
  {
    hud.begin_frame();

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

    // Render
    hud.render_frame(frame);
    hud.render_fps(fps);

    hud.end_frame();
  }

  hud.shutdown();
#else
  spdlog::error("ImGui HUD not enabled. Rebuild with AEROFORGE_WITH_IMGUI=ON");
  return 1;
#endif

  return 0;
}
