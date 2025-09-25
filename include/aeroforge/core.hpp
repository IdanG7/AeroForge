#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace af
{

// ──────────────────────────────────────────────────────────────────────────
// Frame & Detection types
// ──────────────────────────────────────────────────────────────────────────

struct Frame
{
  uint64_t seq{};
  double timestamp_s{};
  cv::Mat img; // BGR/RGB; prealloc pool used by capture
};

struct Detection
{
  int id{-1};
  cv::Rect bbox;
  cv::Point2f centroid;
  float score{1.0f};
  std::optional<cv::Mat> rvec; // ArUco pose (optional)
  std::optional<cv::Mat> tvec;
};

// ──────────────────────────────────────────────────────────────────────────
// Single-Producer Single-Consumer Lock-Free Ring Buffer
// ──────────────────────────────────────────────────────────────────────────

template <typename T, size_t N>
class SpscRingBuffer
{
public:
  SpscRingBuffer() : head_(0), tail_(0) {}

  // Push by single producer thread
  bool push(const T& v)
  {
    const size_t current_tail = tail_.load(std::memory_order_relaxed);
    const size_t next_tail = increment(current_tail);
    if (next_tail == head_.load(std::memory_order_acquire))
    {
      return false; // full
    }
    buf_[current_tail] = v;
    tail_.store(next_tail, std::memory_order_release);
    return true;
  }

  // Pop by single consumer thread
  bool pop(T& out)
  {
    const size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire))
    {
      return false; // empty
    }
    out = buf_[current_head];
    head_.store(increment(current_head), std::memory_order_release);
    return true;
  }

  size_t size() const
  {
    const size_t h = head_.load(std::memory_order_acquire);
    const size_t t = tail_.load(std::memory_order_acquire);
    if (t >= h)
    {
      return t - h;
    }
    return N - h + t;
  }

  bool empty() const { return size() == 0; }

  size_t capacity() const { return N - 1; }

private:
  size_t increment(size_t idx) const { return (idx + 1) % N; }

  std::array<T, N> buf_;
  std::atomic<size_t> head_;
  std::atomic<size_t> tail_;
};

// ──────────────────────────────────────────────────────────────────────────
// Frame Pool for pre-allocation
// ──────────────────────────────────────────────────────────────────────────

class FramePool
{
public:
  FramePool(size_t count, int width, int height, int type = CV_8UC3);
  std::optional<Frame> acquire();
  void release(Frame&& frame);
  size_t available() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// ──────────────────────────────────────────────────────────────────────────
// Timing utilities
// ──────────────────────────────────────────────────────────────────────────

double now_seconds();

class ScopedTimer
{
public:
  explicit ScopedTimer(double& out_ms);
  ~ScopedTimer();

private:
  double& out_ms_;
  double start_;
};

} // namespace af
