#include "aeroforge/core.hpp"
#include <mutex>
#include <queue>

namespace af
{

struct FramePool::Impl
{
  std::queue<Frame> available_;
  std::mutex mutex_;
  size_t capacity_;

  Impl(size_t count, int width, int height, int type) : capacity_(count)
  {
    for (size_t i = 0; i < count; ++i)
    {
      Frame f;
      f.seq = 0;
      f.timestamp_s = 0.0;
      f.img = cv::Mat(height, width, type);
      available_.push(std::move(f));
    }
  }
};

FramePool::FramePool(size_t count, int width, int height, int type)
    : impl_(std::make_unique<Impl>(count, width, height, type))
{
}

std::optional<Frame> FramePool::acquire()
{
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  if (impl_->available_.empty())
  {
    return std::nullopt;
  }
  Frame f = std::move(impl_->available_.front());
  impl_->available_.pop();
  return f;
}

void FramePool::release(Frame&& frame)
{
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  // Reset metadata but keep Mat buffer allocated
  frame.seq = 0;
  frame.timestamp_s = 0.0;
  impl_->available_.push(std::move(frame));
}

size_t FramePool::available() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex_);
  return impl_->available_.size();
}

} // namespace af
