#pragma once

#include "aeroforge/core.hpp"
#include "aeroforge/plugin.hpp"
#include <memory>
#include <string>
#include <vector>

namespace af
{

/**
 * PyTracking Detector Interface
 *
 * This class provides integration with the pytracking library via a Python subprocess.
 * It communicates with the Python wrapper using JSON over stdin/stdout.
 *
 * Features:
 * - State-of-the-art tracking algorithms (DiMP, PrDiMP, ToMP, KeepTrack, etc.)
 * - Automatic subprocess management
 * - Base64 image encoding for IPC
 * - Supports interactive ROI selection
 *
 * Usage:
 *   PyTrackingDetector detector("python/pytracking_wrapper.py", "dimp");
 *   detector.set_roi(roi, frame);
 *   auto detections = detector.detect(frame);
 */
class PyTrackingDetector : public ITargetTracker
{
public:
  /**
   * Constructor
   *
   * @param wrapper_path Path to pytracking_wrapper.py script
   * @param tracker_name Name of tracker ("dimp", "prdimp", "tomp", "keep_track", etc.)
   * @param python_cmd Python command to use (default: "python3")
   */
  explicit PyTrackingDetector(const std::string& wrapper_path,
                               const std::string& tracker_name = "dimp",
                               const std::string& python_cmd = "python3");

  /**
   * Destructor - terminates Python subprocess
   */
  ~PyTrackingDetector() override;

  // Disable copy (subprocess handles are not copyable)
  PyTrackingDetector(const PyTrackingDetector&) = delete;
  PyTrackingDetector& operator=(const PyTrackingDetector&) = delete;

  // Allow move
  PyTrackingDetector(PyTrackingDetector&&) noexcept;
  PyTrackingDetector& operator=(PyTrackingDetector&&) noexcept;

  /**
   * Set ROI to initialize tracking
   *
   * @param roi Bounding box for the object to track
   * @param frame Frame containing the object
   * @return true if initialization succeeded, false otherwise
   */
  bool set_roi(const cv::Rect& roi, const cv::Mat& frame) override;

  /**
   * Check if tracker is currently tracking an object
   */
  bool is_tracking() const override;

  /**
   * Reset tracker (stop tracking)
   */
  void reset() override;

  /**
   * Get last tracking confidence score
   */
  float get_confidence() const;

  /**
   * Get tracker name
   */
  const std::string& get_tracker_name() const;

  // IDetector interface
  void process(Frame& frame) override;
  std::vector<Detection> detect(const Frame& frame) override;
  const char* name() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * Create a PyTrackingDetector from config parameters
 *
 * Expected parameters:
 * - wrapper_path: Path to Python wrapper script
 * - tracker_name: Name of tracker (default: "dimp")
 * - python_cmd: Python command (default: "python3")
 */
std::unique_ptr<PyTrackingDetector> make_pytracking_detector(
    const std::string& wrapper_path,
    const std::string& tracker_name = "dimp",
    const std::string& python_cmd = "python3");

} // namespace af
