#include "aeroforge/pytracking_iface.hpp"
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <cstring>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <unistd.h>
#include <signal.h>
#define POPEN popen
#define PCLOSE pclose
#endif

using json = nlohmann::json;

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// Base64 encoding/decoding utilities
// ──────────────────────────────────────────────────────────────────────────

namespace base64
{

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string encode(const std::vector<uchar>& data)
{
  std::string ret;
  int i = 0;
  int j = 0;
  uchar char_array_3[3];
  uchar char_array_4[4];
  size_t in_len = data.size();
  const uchar* bytes_to_encode = data.data();

  while (in_len--)
  {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3)
    {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; i < 4; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for (j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

    for (j = 0; j < i + 1; j++)
      ret += base64_chars[char_array_4[j]];

    while (i++ < 3)
      ret += '=';
  }

  return ret;
}

} // namespace base64

// ──────────────────────────────────────────────────────────────────────────
// Implementation
// ──────────────────────────────────────────────────────────────────────────

struct PyTrackingDetector::Impl
{
  std::string wrapper_path;
  std::string tracker_name;
  std::string python_cmd;
  // Model loading on first use can take tens of seconds; a tracked frame must not.
  static constexpr int kInitTimeoutMs = 60000;
  static constexpr int kTrackTimeoutMs = 1000;

  bool initialized{false};
  float last_confidence{0.0f};
  cv::Rect last_bbox;

#ifdef _WIN32
  HANDLE h_child_stdin_rd{nullptr};
  HANDLE h_child_stdin_wr{nullptr};
  HANDLE h_child_stdout_rd{nullptr};
  HANDLE h_child_stdout_wr{nullptr};
  HANDLE h_child_stderr_rd{nullptr};
  HANDLE h_child_stderr_wr{nullptr};
  HANDLE h_process{nullptr};
  HANDLE h_thread{nullptr};
  std::thread stderr_drain_thread; // drains child's stderr to avoid blocking and mixing with stdout
#else
  FILE* pipe_in{nullptr};
  FILE* pipe_out{nullptr};
  pid_t process_pid{-1};
#endif

  Impl(std::string wrapper, std::string tracker, std::string python)
      : wrapper_path(std::move(wrapper)),
        tracker_name(std::move(tracker)),
        python_cmd(std::move(python))
  {
  }

  ~Impl() { shutdown(); }

  bool start_subprocess()
  {
#ifdef _WIN32
    if (h_process != nullptr)
    {
      spdlog::warn("[PyTrackingDetector] Subprocess already running");
      return true;
    }
#else
    if (pipe_in != nullptr || pipe_out != nullptr)
    {
      spdlog::warn("[PyTrackingDetector] Subprocess already running");
      return true;
    }
#endif

    spdlog::info("[PyTrackingDetector] Starting Python subprocess: {} {}",
                 python_cmd, wrapper_path);

#ifdef _WIN32
    // Windows: Use CreateProcess with pipes for stdin/stdout
    SECURITY_ATTRIBUTES sa_attr;
    sa_attr.nLength = sizeof(SECURITY_ATTRIBUTES);
    sa_attr.bInheritHandle = TRUE;
    sa_attr.lpSecurityDescriptor = nullptr;

    // Create pipes for child process stdin
    if (!CreatePipe(&h_child_stdin_rd, &h_child_stdin_wr, &sa_attr, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to create stdin pipe: {}", GetLastError());
      return false;
    }
    // Ensure write handle is not inherited
    if (!SetHandleInformation(h_child_stdin_wr, HANDLE_FLAG_INHERIT, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to set stdin handle info: {}", GetLastError());
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      return false;
    }

    // Create pipes for child process stdout
    if (!CreatePipe(&h_child_stdout_rd, &h_child_stdout_wr, &sa_attr, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to create stdout pipe: {}", GetLastError());
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      return false;
    }
    // Ensure read handle is not inherited
    if (!SetHandleInformation(h_child_stdout_rd, HANDLE_FLAG_INHERIT, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to set stdout handle info: {}", GetLastError());
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      CloseHandle(h_child_stdout_rd);
      CloseHandle(h_child_stdout_wr);
      return false;
    }

    // Create pipes for child process stderr (separate from stdout)
    if (!CreatePipe(&h_child_stderr_rd, &h_child_stderr_wr, &sa_attr, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to create stderr pipe: {}", GetLastError());
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      CloseHandle(h_child_stdout_rd);
      CloseHandle(h_child_stdout_wr);
      return false;
    }
    // Ensure stderr read handle is not inherited
    if (!SetHandleInformation(h_child_stderr_rd, HANDLE_FLAG_INHERIT, 0))
    {
      spdlog::error("[PyTrackingDetector] Failed to set stderr handle info: {}", GetLastError());
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      CloseHandle(h_child_stdout_rd);
      CloseHandle(h_child_stdout_wr);
      CloseHandle(h_child_stderr_rd);
      CloseHandle(h_child_stderr_wr);
      return false;
    }

    // Set up process info
    PROCESS_INFORMATION proc_info;
    ZeroMemory(&proc_info, sizeof(PROCESS_INFORMATION));

    STARTUPINFOA startup_info;
    ZeroMemory(&startup_info, sizeof(STARTUPINFOA));
    startup_info.cb = sizeof(STARTUPINFOA);
    // Keep stdout dedicated for JSON. Route stderr to a separate pipe.
    startup_info.hStdError = h_child_stderr_wr;
    startup_info.hStdOutput = h_child_stdout_wr;
    startup_info.hStdInput = h_child_stdin_rd;
    startup_info.dwFlags |= STARTF_USESTDHANDLES;

    // Build command line: python wrapper_path
    // Quote the script path so that directories with spaces survive tokenisation
    std::string cmd_line = python_cmd + " \"" + wrapper_path + "\"";
    spdlog::debug("[PyTrackingDetector] Command line: {}", cmd_line);

    // Create child process
    BOOL success = CreateProcessA(
        nullptr,                          // Application name
        const_cast<char*>(cmd_line.c_str()), // Command line
        nullptr,                          // Process security attributes
        nullptr,                          // Thread security attributes
        TRUE,                             // Inherit handles
        CREATE_NO_WINDOW,                 // Creation flags (no console window)
        nullptr,                          // Environment
        nullptr,                          // Current directory
        &startup_info,                    // Startup info
        &proc_info);                      // Process info

    if (!success)
    {
      DWORD error = GetLastError();
      spdlog::error("[PyTrackingDetector] Failed to create process: {}", error);
      CloseHandle(h_child_stdin_rd);
      CloseHandle(h_child_stdin_wr);
      CloseHandle(h_child_stdout_rd);
      CloseHandle(h_child_stdout_wr);
      CloseHandle(h_child_stderr_rd);
      CloseHandle(h_child_stderr_wr);
      return false;
    }

    // Close unused pipe handles (child ends) in parent
    CloseHandle(h_child_stdin_rd);
    CloseHandle(h_child_stdout_wr);
    CloseHandle(h_child_stderr_wr);

    // Store process handles
    h_process = proc_info.hProcess;
    h_thread = proc_info.hThread;

    // Start a background thread to drain stderr so child doesn't block
    try
    {
      stderr_drain_thread = std::thread([this]() {
        std::string line;
        char ch;
        DWORD bytes_read;
        while (true)
        {
          BOOL ok = ReadFile(h_child_stderr_rd, &ch, 1, &bytes_read, nullptr);
          if (!ok || bytes_read == 0) break;
          if (ch == '\n')
          {
            if (!line.empty())
            {
              spdlog::info("[PyTrackingDetector][stderr] {}", line);
              line.clear();
            }
          }
          else if (ch != '\r')
          {
            line += ch;
          }
        }
        if (!line.empty())
        {
          spdlog::info("[PyTrackingDetector][stderr] {}", line);
        }
      });
    }
    catch (...)
    {
      spdlog::warn("[PyTrackingDetector] Failed to start stderr drain thread");
    }

    // Give Python time to start up
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    spdlog::info("[PyTrackingDetector] Windows subprocess started successfully");
    return true;

#else
    // Unix: Use popen with bidirectional pipe
    std::string cmd = python_cmd + " " + wrapper_path + " 2>&1";
    pipe_in = POPEN(cmd.c_str(), "w");
    pipe_out = pipe_in; // Same pipe for bidirectional communication

    if (!pipe_in)
    {
      spdlog::error("[PyTrackingDetector] Failed to start subprocess: {}", strerror(errno));
      return false;
    }

    // Give Python time to start up
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    spdlog::info("[PyTrackingDetector] Subprocess started successfully");
    return true;
#endif
  }

  void shutdown()
  {
#ifdef _WIN32
    if (h_process != nullptr)
    {
      // Send quit command
      try
      {
        json quit_msg;
        quit_msg["type"] = "quit";
        send_message(quit_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      catch (...)
      {
        // Ignore errors during shutdown
      }

      // Close pipe handles
      if (h_child_stdin_wr) CloseHandle(h_child_stdin_wr);
      if (h_child_stdout_rd) CloseHandle(h_child_stdout_rd);
      if (h_child_stderr_rd) CloseHandle(h_child_stderr_rd);

      // Wait for process to exit (with timeout)
      WaitForSingleObject(h_process, 2000);

      // Force terminate if still running
      DWORD exit_code;
      if (GetExitCodeProcess(h_process, &exit_code) && exit_code == STILL_ACTIVE)
      {
        spdlog::warn("[PyTrackingDetector] Force terminating subprocess");
        TerminateProcess(h_process, 1);
      }

      // Clean up handles
      CloseHandle(h_process);
      CloseHandle(h_thread);

      // Join stderr drain thread if running
      if (stderr_drain_thread.joinable())
      {
        try { stderr_drain_thread.join(); } catch (...) {}
      }

      h_child_stdin_wr = nullptr;
      h_child_stdout_rd = nullptr;
      h_child_stderr_rd = nullptr;
      h_process = nullptr;
      h_thread = nullptr;

      spdlog::info("[PyTrackingDetector] Windows subprocess terminated");
    }
#else
    if (pipe_in || pipe_out)
    {
      // Send quit command
      try
      {
        json quit_msg;
        quit_msg["type"] = "quit";
        send_message(quit_msg);
      }
      catch (...)
      {
        // Ignore errors during shutdown
      }

      // pipe_out aliases pipe_in: close the stream exactly once
      FILE* stream = pipe_in;
      pipe_in = nullptr;
      pipe_out = nullptr;
      if (stream)
      {
        PCLOSE(stream);
      }

      spdlog::info("[PyTrackingDetector] Subprocess terminated");
    }
#endif
  }

  void send_message(const json& msg)
  {
#ifdef _WIN32
    if (!h_child_stdin_wr)
    {
      throw std::runtime_error("Subprocess not running");
    }

    std::string msg_str = msg.dump() + "\n";
    DWORD written;
    BOOL success = WriteFile(h_child_stdin_wr, msg_str.c_str(),
                             static_cast<DWORD>(msg_str.size()), &written, nullptr);

    if (!success || written != msg_str.size())
    {
      throw std::runtime_error("Failed to write to subprocess");
    }
#else
    if (!pipe_in)
    {
      throw std::runtime_error("Subprocess not running");
    }

    std::string msg_str = msg.dump() + "\n";
    size_t written = fwrite(msg_str.c_str(), 1, msg_str.size(), pipe_in);
    fflush(pipe_in);

    if (written != msg_str.size())
    {
      throw std::runtime_error("Failed to write to subprocess");
    }
#endif
  }

  // Waits at most `timeout_ms` for a reply (Windows). After a timeout the stream is out of
  // step with the requests, so callers must shut the subprocess down.
  json receive_message(int timeout_ms)
  {
#ifdef _WIN32
    if (!h_child_stdout_rd)
    {
      throw std::runtime_error("Subprocess not running");
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (true)
    {
      std::string line;
      char ch;
      DWORD bytes_read;

      // Read character by character until newline
      while (true)
      {
        DWORD available = 0;
        while (true)
        {
          if (!PeekNamedPipe(h_child_stdout_rd, nullptr, 0, nullptr, &available, nullptr))
          {
            throw std::runtime_error("Subprocess closed its output (it may have exited)");
          }
          if (available > 0)
          {
            break;
          }
          if (std::chrono::steady_clock::now() >= deadline)
          {
            throw std::runtime_error("Timed out after " + std::to_string(timeout_ms) +
                                     " ms waiting for the Python tracker");
          }
          Sleep(1);
        }

        BOOL success = ReadFile(h_child_stdout_rd, &ch, 1, &bytes_read, nullptr);

        if (!success || bytes_read == 0)
        {
          if (line.empty())
          {
            throw std::runtime_error("Failed to read from subprocess");
          }
          break;
        }

        if (ch == '\n')
        {
          break;
        }

        if (ch != '\r')  // Skip carriage returns
        {
          line += ch;
        }
      }

      // Expect protocol prefix to mark JSON lines
      constexpr const char* kPrefix = "AFJSON ";
      if (line.rfind(kPrefix, 0) != 0) // not starting with prefix
      {
        spdlog::warn("[PyTrackingDetector] Skipping non-protocol stdout: {}", line);
        continue;
      }

      const std::string payload = line.substr(strlen(kPrefix));
      try
      {
        return json::parse(payload);
      }
      catch (const std::exception&)
      {
        spdlog::warn("[PyTrackingDetector] JSON parse failed, skipping payload: {}", payload);
        continue; // try next line
      }
    }

#else
    (void)timeout_ms; // not implemented for the POSIX path
    if (!pipe_out)
    {
      throw std::runtime_error("Subprocess not running");
    }

    while (true)
    {
      std::string line;
      char buffer[4096];

      if (fgets(buffer, sizeof(buffer), pipe_out) == nullptr)
      {
        throw std::runtime_error("Failed to read from subprocess");
      }

      line = buffer;

      // Remove trailing newline
      if (!line.empty() && line.back() == '\n')
      {
        line.pop_back();
      }

      constexpr const char* kPrefix = "AFJSON ";
      if (line.rfind(kPrefix, 0) != 0)
      {
        spdlog::warn("[PyTrackingDetector] Skipping non-protocol stdout: {}", line);
        continue;
      }
      const std::string payload = line.substr(strlen(kPrefix));
      try
      {
        return json::parse(payload);
      }
      catch (const std::exception&)
      {
        spdlog::warn("[PyTrackingDetector] JSON parse failed, skipping payload: {}", payload);
        continue;
      }
    }
#endif
  }

  std::string encode_image(const cv::Mat& img)
  {
    // Encode image as JPEG
    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85}; // Balance quality and speed
    cv::imencode(".jpg", img, buf, params);

    // Base64 encode
    return base64::encode(buf);
  }

  bool init_tracker(const cv::Rect& roi, const cv::Mat& frame)
  {
#ifdef _WIN32
    bool subprocess_running = (h_process != nullptr);
#else
    bool subprocess_running = (pipe_in != nullptr);
#endif

    if (!subprocess_running)
    {
      if (!start_subprocess())
      {
        return false;
      }
    }

    try
    {
      // Encode frame
      std::string frame_b64 = encode_image(frame);

      // Create init message
      json init_msg;
      init_msg["type"] = "init";
      init_msg["frame"] = frame_b64;
      init_msg["bbox"] = {roi.x, roi.y, roi.width, roi.height};
      init_msg["tracker_name"] = tracker_name;

      spdlog::debug("[PyTrackingDetector] Sending init message (frame size: {} bytes)",
                    frame_b64.size());

      // Send message
      send_message(init_msg);

      // Receive response
      json response = receive_message(kInitTimeoutMs);

      if (response["status"] != "ok")
      {
        std::string error = response.value("message", "Unknown error");
        spdlog::error("[PyTrackingDetector] Init failed: {}", error);
        return false;
      }

      initialized = true;
      last_bbox = roi;
      last_confidence = 1.0f;

      spdlog::info("[PyTrackingDetector] Initialized with tracker: {}",
                   response.value("tracker", tracker_name));

      return true;
    }
    catch (const std::exception& e)
    {
      spdlog::error("[PyTrackingDetector] Init exception: {}", e.what());
      initialized = false;
      shutdown(); // replies can no longer be matched to requests; restart on next select
      return false;
    }
  }

  std::vector<Detection> track_frame(const cv::Mat& frame)
  {
    if (!initialized)
    {
      return {};
    }

    try
    {
      // Encode frame
      std::string frame_b64 = encode_image(frame);

      // Create track message
      json track_msg;
      track_msg["type"] = "track";
      track_msg["frame"] = frame_b64;

      // Send message
      send_message(track_msg);

      // Receive response
      json response = receive_message(kTrackTimeoutMs);

      if (response["status"] != "ok")
      {
        std::string error = response.value("message", "Unknown error");
        spdlog::warn("[PyTrackingDetector] Track failed: {}", error);
        return {};
      }

      // Parse bbox
      auto bbox_arr = response["bbox"];
      cv::Rect bbox(
          static_cast<int>(bbox_arr[0]),
          static_cast<int>(bbox_arr[1]),
          static_cast<int>(bbox_arr[2]),
          static_cast<int>(bbox_arr[3]));

      last_bbox = bbox;
      last_confidence = response.value("confidence", 1.0f);

      // Create detection
      Detection det;
      det.id = 0;
      det.bbox = bbox;
      det.centroid = cv::Point2f(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
      det.score = last_confidence;

      return {det};
    }
    catch (const std::exception& e)
    {
      spdlog::error("[PyTrackingDetector] Track exception: {}", e.what());
      initialized = false; // Mark as uninitialized on error
      shutdown();          // see init_tracker
      return {};
    }
  }

  void reset_tracker()
  {
#ifdef _WIN32
    if (!h_process)
    {
      return;
    }
#else
    if (!pipe_in)
    {
      return;
    }
#endif

    try
    {
      json reset_msg;
      reset_msg["type"] = "reset";

      send_message(reset_msg);
      receive_message(kTrackTimeoutMs); // Wait for acknowledgment

      initialized = false;
      last_confidence = 0.0f;

      spdlog::info("[PyTrackingDetector] Tracker reset");
    }
    catch (const std::exception& e)
    {
      spdlog::error("[PyTrackingDetector] Reset exception: {}", e.what());
      initialized = false;
      shutdown();
    }
  }
};

// ──────────────────────────────────────────────────────────────────────────
// PyTrackingDetector Implementation
// ──────────────────────────────────────────────────────────────────────────

PyTrackingDetector::PyTrackingDetector(const std::string& wrapper_path,
                                       const std::string& tracker_name,
                                       const std::string& python_cmd)
    : impl_(std::make_unique<Impl>(wrapper_path, tracker_name, python_cmd))
{
  spdlog::info("[PyTrackingDetector] Created with tracker: {}", tracker_name);
}

PyTrackingDetector::~PyTrackingDetector() = default;

PyTrackingDetector::PyTrackingDetector(PyTrackingDetector&&) noexcept = default;
PyTrackingDetector& PyTrackingDetector::operator=(PyTrackingDetector&&) noexcept = default;

bool PyTrackingDetector::set_roi(const cv::Rect& roi, const cv::Mat& frame)
{
  return impl_->init_tracker(roi, frame);
}

bool PyTrackingDetector::is_tracking() const { return impl_->initialized; }

void PyTrackingDetector::reset() { impl_->reset_tracker(); }

float PyTrackingDetector::get_confidence() const { return impl_->last_confidence; }

const std::string& PyTrackingDetector::get_tracker_name() const
{
  return impl_->tracker_name;
}

void PyTrackingDetector::process(Frame& frame) { (void)frame; }

std::vector<Detection> PyTrackingDetector::detect(const Frame& frame)
{
  if (frame.img.empty())
  {
    return {};
  }

  return impl_->track_frame(frame.img);
}

const char* PyTrackingDetector::name() const { return "PyTrackingDetector"; }

std::unique_ptr<PyTrackingDetector> make_pytracking_detector(const std::string& wrapper_path,
                                                               const std::string& tracker_name,
                                                               const std::string& python_cmd)
{
  return std::make_unique<PyTrackingDetector>(wrapper_path, tracker_name, python_cmd);
}

} // namespace af
