#pragma once
#include <cstddef>
#include <array>
#include <numeric>

#include "utility.hpp"

template <size_t N>
class DelayLine
{
  static_assert((N > 1) && ((N & (N - 1)) == 0), "N must be power of 2");

public:
  void write(float sample)
  {
    ++current_pos;
    samples[current_pos & mask] = sample;
  }

  void clear()
  {
    samples.fill(0.f);
  }

  float read_linear(float delay) const
  {
    size_t d = static_cast<size_t>(delay);
    float frac = delay - static_cast<float>(d);
    return lerpf(tap(d), tap(d + 1), frac);
  }
private:
  float tap(size_t delay) const
  {
    return samples[(current_pos - delay) & mask];
    // if delay > current_pos, underflow happens but (& mask) makes it semantically correct
  }

  static constexpr size_t mask = N - 1;
  std::array<float, N> samples = {};
  size_t current_pos = 0;
};
