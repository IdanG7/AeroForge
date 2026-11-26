#pragma once

#include <cstdint>

namespace af
{

// Validity of an observation (detection, pose, telemetry sample).
//
// Downstream stages must check this before using a value. Only `Fresh` values may drive
// control; everything else is for display and diagnostics.
enum class Validity : uint8_t
{
  Unavailable, // no value was produced (source missing or not connected)
  Invalid,     // a value was produced but failed validation (non-finite, out of range)
  Stale,       // a measured value that is older than the consumer's freshness limit
  Predicted,   // extrapolated by a model, not measured in this frame
  Fresh,       // measured in the current frame / sample
};

constexpr const char* to_string(Validity v)
{
  switch (v)
  {
  case Validity::Unavailable:
    return "unavailable";
  case Validity::Invalid:
    return "invalid";
  case Validity::Stale:
    return "stale";
  case Validity::Predicted:
    return "predicted";
  case Validity::Fresh:
    return "fresh";
  }
  return "unknown";
}

} // namespace af
