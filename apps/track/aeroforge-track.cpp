#include "aeroforge/core.hpp"
#include "aeroforge/config.hpp"
#include "aeroforge/dji_iface.hpp"

#ifdef AEROFORGE_WITH_IMGUI
#include "aeroforge/ui.hpp"
#endif

#include <spdlog/spdlog.h>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <thread>

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

  // TODO: Build pipeline from config
  // For now, just open webcam and display

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

  // Main loop
  spdlog::info("Entering main loop. Press ESC to exit, SPACE for e-stop, F to enable.");
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

    // TODO: Run pipeline (detect, track, estimate, control)

    // Render
    hud.render_frame(frame);
    hud.render_fps(fps);
    hud.render_telemetry(telemetry);
    hud.render_safety_status(safety_enabled, true);

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
