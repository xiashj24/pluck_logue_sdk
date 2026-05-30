#pragma once
/*
    BSD 3-Clause License

    Copyright (c) 2026, KORG INC.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "processor.h"
#include "unit_osc.h"

/**
 * @brief karplus-strong wavegudie synthesis
 * @author Shijie Xia (xiashj@korg.co.jp)
 */

#include "dsp/utility.hpp"

class Osc : public Processor
{
public:
  uint32_t getBufferSize() const override final { return 0; } // ununsed
  void setPitch(float w0) {}                                  // ununsed
  void setShapeLfo(float lfo) {}                              // ununsed

  // audio parameters
  enum
  {
    SHAPE = 0U,
    ALT,
    NUM_PARAMS
  };

  // Note: Make sure that default param values correspond to declarations in header.c
  struct Params
  {
    float shape;
    float alt;

    void reset()
    {
      shape = 0.f;
      alt = 0.f;
    }

    Params() { reset(); }
  };

  void setParameter(uint8_t index, int32_t value) override final
  {
    switch (index)
    {
    case SHAPE:
      params.shape = param_10bit_to_f32(value); // 0 .. 1023 -> 0.0 .. 1.0
      break;

    case ALT:
      params.alt = param_10bit_to_f32(value); // 0 .. 1023 -> 0.0 .. 1.0
      break;
    default:
      break;
    }
  }

  void init(float *) override final
  {
    params.reset();
  }

  void noteOn(uint8_t note, uint8_t velocity) override final
  {
    pitch = note_to_hz(note);
  }

  void process(const float *__restrict in, float *__restrict out, uint32_t frames) override final
  {
    // Caching current parameter values. Consider smoothing sensitive parameters in audio loop
    const Params p = params;

    for (const float *out_end = out + frames; out != out_end; in += 2, out += 1)
    {
      // Process/generate samples here

      phase += pitch / getSampleRate();
      if (phase >= 1.f)
      {
        phase -= 1.f;
      }

      out[0] = osc_sinf(phase);
    }
  }

private:
  Params params;
  float phase = 0.f; // phase ramp up from 0 to 1
  float pitch = 440.f;
};
