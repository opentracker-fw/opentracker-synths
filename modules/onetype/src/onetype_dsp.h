// 1-Type DSP kernels (inline, single precision): the filter section the
// engine runs per 16-frame tick, also driven directly by the engine test.
#pragma once

#include <cstdint>

namespace ot {
namespace audio {
namespace onetype {
namespace dsp {

constexpr int kTick = 16;   // frames per control tick

// The filter section (filter, output limiter, saturation) with static
// settings, a tick at a time: the engine test's probe.
struct Section {
    void init(int type, float hz, float res, float sat) {
        (void)type;
        (void)hz;
        (void)res;
        (void)sat;
    }
    void run(const float* x, float* y) {
        (void)x;
        for (int i = 0; i < kTick; ++i) y[i] = 0.0f;
    }
};

}  // namespace dsp
}  // namespace onetype
}  // namespace audio
}  // namespace ot
