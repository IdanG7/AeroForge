#include "aeroforge/core.hpp"
#include <chrono>

namespace af
{

double now_seconds()
{
  using namespace std::chrono;
  auto now = high_resolution_clock::now();
  auto duration = now.time_since_epoch();
  return duration_cast<duration<double>>(duration).count();
}

ScopedTimer::ScopedTimer(double& out_ms) : out_ms_(out_ms), start_(now_seconds()) {}

ScopedTimer::~ScopedTimer()
{
  double end = now_seconds();
  out_ms_ = (end - start_) * 1000.0; // convert to milliseconds
}

} // namespace af
