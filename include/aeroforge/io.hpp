#pragma once

#include "config.hpp"
#include "core.hpp"
#include <memory>
#include <string>

namespace af
{

// A source of frames: webcam, recorded video file, or network stream.
//
// Timestamps:
//   live sources     - Frame::timestamp_s is af::now_seconds() when the frame was read
//   recorded sources - Frame::timestamp_s is media time (frame index / fps), so a replay
//                      produces identical timestamps on every run
// now_s() returns the current time on the same clock as the frames, so "age of the last
// frame" is always now_s() - frame.timestamp_s.
struct IFrameSource
{
  virtual ~IFrameSource() = default;

  // Returns false and logs the reason if the source cannot be opened.
  virtual bool open() = 0;
  virtual void close() = 0;

  // Reads the next frame. Returns false when no frame is available right now: check
  // at_end() to tell "recording finished" from "live source hiccup". A live source that
  // keeps failing tries to reopen itself at most once per second.
  virtual bool read(Frame& out) = 0;

  virtual bool at_end() const = 0;
  virtual bool is_live() const = 0;
  virtual double now_s() const = 0;
  virtual double nominal_fps() const = 0;
  virtual cv::Size frame_size() const = 0; // as reported by the source after open()
  virtual uint64_t failed_reads() const = 0;
  virtual std::string describe() const = 0;
};

// input.type: webcam | file | stream. Throws std::invalid_argument for anything else.
std::unique_ptr<IFrameSource> make_frame_source(const InputConfig& input);

} // namespace af
