// SPDX-License-Identifier: Apache-2.0
// 6-Type engine lookup tables (generated: modules/sixtype/tools/make_sixtype_tables.py) [OT].
#pragma once

#include <cstdint>

namespace ot {
namespace audio {
namespace sixtype {

constexpr int kTabLg = 10;                // 1024 entries per table
constexpr int kTabN = 1 << kTabLg;

extern int32_t gSixtypeSin[kTabN + 1];          // Q24 sine (SRAM, built by initSinTable)
extern const uint32_t kSixtypeExp2[kTabN + 1];  // Q30 2^x mantissa
extern const int32_t kSixtypeFreqLut[kTabN + 1];  // log2 Hz (fraction) -> phase increment

// [I] Builds gSixtypeSin (idempotent): sixtype::engine() calls it, before any render.
void initSinTable();

// Sine of a Q24 phase (1 << 24 = one cycle; it wraps, so unsigned), Q24
// result (msfa Sin::lookup). Neighbour differences are < 2^17, so the
// interpolation fits 32 bits.
inline int32_t sinLookup(uint32_t phase) {
    constexpr int kShift = 24 - kTabLg;
    const int ix = int((phase >> kShift) & (kTabN - 1));
    const int lowbits = int(phase & ((1u << kShift) - 1));
    const int32_t y0 = gSixtypeSin[ix];
    return y0 + (((gSixtypeSin[ix + 1] - y0) * lowbits) >> kShift);
}

// 2^(x / 2^24) in Q24, for x in [-14 << 24, 6 << 24) (msfa Exp2::lookup).
inline int32_t exp2Lookup(int32_t x) {
    constexpr int kShift = 24 - kTabLg;
    const int ix = (x >> kShift) & (kTabN - 1);
    const int lowbits = x & ((1 << kShift) - 1);
    const uint32_t y0 = kSixtypeExp2[ix];
    const uint32_t y = y0 + uint32_t((uint64_t(kSixtypeExp2[ix + 1] - y0) * uint32_t(lowbits)) >> kShift);
    return int32_t(y >> (6 - (x >> 24)));
}

// log2(Hz) in Q24 (0 .. 20 << 24) -> phase increment per sample (Q24 cycles).
inline int32_t freqLookup(int32_t logfreq) {
    constexpr int kShift = 24 - kTabLg;
    const int ix = (logfreq & 0xffffff) >> kShift;
    const int32_t y0 = kSixtypeFreqLut[ix], y1 = kSixtypeFreqLut[ix + 1];
    const int lowbits = logfreq & ((1 << kShift) - 1);
    const int32_t y = y0 + int32_t((int64_t(y1 - y0) * lowbits) >> kShift);
    return y >> (20 - (logfreq >> 24));
}

}  // namespace sixtype
}  // namespace audio
}  // namespace ot
