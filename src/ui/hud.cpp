#include "aeroforge/ui.hpp"

#ifdef AEROFORGE_WITH_IMGUI

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace af
{

struct HUD::Impl
{
  HUDConfig config;
  GLFWwindow* window{nullptr};
  GLuint texture_id{0};
  int window_width{1280};
  int window_height{720};

  // Frame texture
  cv::Mat display_frame;
};

HUD::HUD(const HUDConfig& config) : impl_(std::make_unique<Impl>())
{
  impl_->config = config;
}

HUD::~HUD()
{
  shutdown();
}

bool HUD::init(int window_width, int window_height, const std::string& title)
{
  impl_->window_width = window_width;
  impl_->window_height = window_height;

  // Initialize GLFW
  if (!glfwInit())
  {
    spdlog::error("[HUD] Failed to initialize GLFW");
    return false;
  }

  // GL 3.3 + GLSL 330
  const char* glsl_version = "#version 330";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Create window
  impl_->window = glfwCreateWindow(window_width, window_height, title.c_str(), nullptr, nullptr);
  if (!impl_->window)
  {
    spdlog::error("[HUD] Failed to create GLFW window");
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(impl_->window);
  glfwSwapInterval(1); // Enable vsync

  // Setup ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(impl_->window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Create OpenGL texture for video frame
  glGenTextures(1, &impl_->texture_id);
  glBindTexture(GL_TEXTURE_2D, impl_->texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  spdlog::info("[HUD] Initialized {}x{}: {}", window_width, window_height, title);
  return true;
}

void HUD::shutdown()
{
  if (impl_->window)
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (impl_->texture_id)
    {
      glDeleteTextures(1, &impl_->texture_id);
      impl_->texture_id = 0;
    }

    glfwDestroyWindow(impl_->window);
    glfwTerminate();
    impl_->window = nullptr;
    spdlog::info("[HUD] Shutdown");
  }
}

bool HUD::should_close() const
{
  return impl_->window && glfwWindowShouldClose(impl_->window);
}

void HUD::begin_frame()
{
  if (!impl_->window)
  {
    return;
  }

  glfwPollEvents();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void HUD::end_frame()
{
  if (!impl_->window)
  {
    return;
  }

  // Rendering
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(impl_->window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(impl_->window);
}

void HUD::render_frame(const Frame& frame)
{
  if (frame.img.empty())
  {
    return;
  }

  // Convert BGR to RGB
  cv::Mat rgb;
  cv::cvtColor(frame.img, rgb, cv::COLOR_BGR2RGB);

  // Upload texture
  glBindTexture(GL_TEXTURE_2D, impl_->texture_id);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows, 0, GL_RGB, GL_UNSIGNED_BYTE,
               rgb.data);

  // Display as ImGui image
  ImGui::Begin("Camera Feed", nullptr,
               ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
  ImVec2 size(static_cast<float>(rgb.cols), static_cast<float>(rgb.rows));
  ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(impl_->texture_id)), size);
  ImGui::End();

  impl_->display_frame = rgb;
}

void HUD::render_detections(const std::vector<Detection>& detections)
{
  if (!impl_->config.draw_bbox || detections.empty())
  {
    return;
  }

  ImGui::Begin("Detections", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("Count: %zu", detections.size());

  for (size_t i = 0; i < detections.size(); ++i)
  {
    const auto& det = detections[i];
    ImGui::Text("[%zu] ID=%d, centroid=(%.1f, %.1f), score=%.2f", i, det.id, det.centroid.x,
                det.centroid.y, det.score);
  }

  ImGui::End();
}

void HUD::render_fps(double fps)
{
  if (!impl_->config.show_fps)
  {
    return;
  }

  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::Begin("FPS", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings);
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "FPS: %.1f", fps);
  ImGui::End();
}

void HUD::render_telemetry(const Telemetry& tel)
{
  if (!impl_->config.show_telemetry)
  {
    return;
  }

  ImGui::Begin("Telemetry", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("Altitude: %.2f m", tel.altitude_m);
  ImGui::Text("Speed: %.2f m/s", tel.speed_mps);
  ImGui::Text("Yaw: %.1f deg", tel.yaw_deg);
  ImGui::Text("Position: (%.2f, %.2f, %.2f)", tel.position_m.x(), tel.position_m.y(),
              tel.position_m.z());
  ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", tel.velocity_mps.x(), tel.velocity_mps.y(),
              tel.velocity_mps.z());
  ImGui::Text("GPS: %s, Sats: %d", tel.gps_valid ? "Valid" : "Invalid", tel.satellite_count);
  ImGui::End();
}

void HUD::render_timings(const std::vector<std::pair<std::string, double>>& timings)
{
  if (!impl_->config.show_timings || timings.empty())
  {
    return;
  }

  ImGui::Begin("Pipeline Timings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  for (const auto& [name, time_ms] : timings)
  {
    ImGui::Text("%s: %.2f ms", name.c_str(), time_ms);
  }
  ImGui::End();
}

void HUD::render_safety_status(bool enabled, bool geofence_ok)
{
  ImGui::SetNextWindowPos(ImVec2(10, 50), ImGuiCond_Always);
  ImGui::Begin("Safety", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings);

  if (enabled)
  {
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "ENABLED");
  }
  else
  {
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "DISABLED");
  }

  if (!geofence_ok)
  {
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "[GEOFENCE!]");
  }

  ImGui::End();
}

bool HUD::is_key_pressed(int key) const
{
  if (!impl_->window)
  {
    return false;
  }
  return glfwGetKey(impl_->window, key) == GLFW_PRESS;
}

bool HUD::is_key_down(int key) const
{
  return is_key_pressed(key);
}

} // namespace af

#endif // AEROFORGE_WITH_IMGUI
