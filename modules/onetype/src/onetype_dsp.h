// 1-Type DSP kernels (inline, single precision): the oscillator frames in
// the source engine's conventions, the envelope steps, the small math the
// engine runs per 16-frame tick, and the filter section the engine runs per
// tick (also driven directly by the engine test).
//
// Derived from Oxo Synth2 (https://github.com/chab/oxo_synth2, commit 6bb4435):
//
//   MIT License
//   Copyright (c) 2026 Francois Chabloz and contributors
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to deal
//   in the Software without restriction, including without limitation the rights
//   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//   copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.
#pragma once

#include <cstdint>
#include <cstring>

#include "audio/va_dsp.h"
#include "onetype_params.h"

namespace ot {
namespace audio {
namespace onetype {
namespace dsp {

using va::clampf;
using va::fastExp2;

constexpr int kTick = 16;                        // frames per control tick
constexpr int kTicks = int(kBlock) / kTick;      // 8 per block
constexpr float kFs = kSampleRate;
constexpr float kInvFs = 1.0f / kSampleRate;

// --- math ---------------------------------------------------------------------------

// log2(x) for x > 0 (exponent + a quartic on the mantissa, |error| < 3e-4).
inline float fastLog2(float x) {
    uint32_t b;
    std::memcpy(&b, &x, 4);
    const float e = float(int((b >> 23) & 0xFFu) - 127);
    b = (b & 0x007FFFFFu) | 0x3F800000u;
    float m;
    std::memcpy(&m, &b, 4);   // 1 .. 2
    return e - 2.4968460f + m * (4.0285479f + m * (-2.0812141f + m * (0.6288736f - 0.0791582f * m)));
}
// Phase increment (cycles per frame) of a pitch in semitones (69 = 440 Hz).
inline float incOf(float pitch) { return clampf(440.0f * kInvFs * fastExp2((pitch - 69.0f) * (1.0f / 12.0f)), 0.0f, va::kMaxDt); }

// --- oscillator frames ---------------------------------------------------------------
//
// Each frame takes the phase after this frame's step (p, 0..1), the step d
// (> 0) and whether the phase wrapped, and returns the PREVIOUS frame's
// band-limited sample (z, held): an edge crossed during this frame corrects
// the frame before it and the frame now (two-frame polyBLEP / polyBLAMP,
// audio/va_dsp.h). The shapes follow the source: the saw falls (+1 at phase
// 0), the pulse is w for the first 1 - w of the cycle and w - 1 for the rest
// (half amplitude, no DC), the triangle is in sine phase (+1 at 0.25).

inline void blamp(float c, float x, float& before, float& now) {
    const float y = 1.0f - x;
    before += c * x * x * x * (1.0f / 6.0f);
    now += c * y * y * y * (1.0f / 6.0f);
}

inline float sawFrame(float p, float d, bool wrap, float& z) {
    float out = z, cur = 1.0f - 2.0f * p;
    if (wrap) va::stepResidual(2.0f, p / d, out, cur);
    z = cur;
    return out;
}
// hi: the frame before was in the first (high) part.
inline float pulseFrame(float p, float d, bool wrap, float w, float& z, uint8_t& hi) {
    const float on = 1.0f - w;
    const bool h = p < on;
    float out = z, cur = h ? w : w - 1.0f;
    if (wrap) va::stepResidual(1.0f, p / d, out, cur);
    if (!h && (hi || wrap)) va::stepResidual(-1.0f, clampf((p - on) / d, 0.0f, 1.0f), out, cur);
    hi = h;
    z = cur;
    return out;
}
inline float triNaive(float p) { return p < 0.25f ? 4.0f * p : (p < 0.75f ? 2.0f - 4.0f * p : 4.0f * p - 4.0f); }
// hi: the frame before was on the falling side (0.25 .. 0.75).
inline float triFrame(float p, float d, float& z, uint8_t& hi) {
    const bool h = p >= 0.25f && p < 0.75f;
    float out = z, cur = triNaive(p);
    if (h != bool(hi)) {
        if (h) {
            blamp(-8.0f * d, clampf((p - 0.25f) / d, 0.0f, 1.0f), out, cur);
        } else {
            const float q = p < 0.5f ? p + 1.0f : p;
            blamp(8.0f * d, clampf((q - 0.75f) / d, 0.0f, 1.0f), out, cur);
        }
    }
    hi = h;
    z = cur;
    return out;
}
inline float sineFrame(float p, float& z) {
    const float out = z;
    z = va::sinePar(p);
    return out;
}
// The hi state of a phase (a fresh oscillator, a reset).
inline uint8_t pulseHi(float p, float w) { return p < 1.0f - w; }
inline uint8_t triHi(float p) { return p >= 0.25f && p < 0.75f; }

// The source's osc 2 noise: a 1-bit sequence s[n] = s[n-13] ^ s[n-18] ^
// s[n-19] ^ s[n-20] played as +-kNoiseLevel, from kNoiseSeed (a synced
// osc 1 wrap restarts it there).
constexpr uint32_t kNoiseSeed = 0x70A6E;
constexpr float kNoiseLevel = 0.65039f;
inline float noiseStep(uint32_t& n) {
    const uint32_t bit = (n >> 12 ^ n >> 17 ^ n >> 18 ^ n >> 19) & 1u;
    n = ((n << 1) | bit) & 0xFFFFFu;
    return bit ? kNoiseLevel : -kNoiseLevel;
}

// Hard sync for the saw and the pulse (audio/va_dsp.h vcoSync in these
// shapes): the master wrapped x frames ago (0 <= x < 1), so the phase
// restarts at 0 then; the jump from where it was gets the step residual like
// any edge. kP: 0 saw, 1 pulse of low part w.
template <int kP>
inline float syncNaive(float p, float w) { return kP ? (p < 1.0f - w ? w : w - 1.0f) : 1.0f - 2.0f * p; }
// The phase from p over dp (<= one frame) ending `age` frames before now:
// each edge crossed adds its residual. Returns the end phase.
template <int kP>
inline float syncRun(float p, float dp, float d, float age, float w, float& before, float& now) {
    float e = p + dp;
    const bool wrap = e >= 1.0f;
    if (!kP) {
        if (wrap) {
            e -= 1.0f;
            va::stepResidual(2.0f, age + e / d, before, now);
        }
    } else {
        const float on = 1.0f - w;
        if (p < on && e >= on) va::stepResidual(-1.0f, age + (e - on) / d, before, now);
        if (wrap) {
            e -= 1.0f;
            va::stepResidual(1.0f, age + e / d, before, now);
            if (e >= on) va::stepResidual(-1.0f, age + (e - on) / d, before, now);
        }
    }
    return e;
}
template <int kP>
inline float syncFrame(float& p, float& z, float d, float w, float x) {
    float out = z, cur = 0.0f;
    const float q = syncRun<kP>(p, (1.0f - x) * d, d, x, w, out, cur);
    va::stepResidual(syncNaive<kP>(0.0f, w) - syncNaive<kP>(q, w), x, out, cur);
    p = syncRun<kP>(0.0f, x * d, d, 0.0f, w, out, cur);
    z = cur + syncNaive<kP>(p, w);
    return out;
}

// --- LFOs ----------------------------------------------------------------------------

// Tempo sync: the cycle in beats for Sync 1..30 (16 bars .. 1/64T).
constexpr float kSyncBeats[30] = {64.0f, 48.0f, 36.0f, 32.0f,  24.0f,        18.0f, 16.0f,         12.0f, 9.0f,  8.0f,
                                  6.0f,  4.0f,  3.5f,  3.0f,   2.0f,         1.75f, 1.5f,          4.0f / 3.0f, 1.0f, 0.875f,
                                  0.75f, 2.0f / 3.0f, 0.5f, 0.375f, 1.0f / 3.0f, 0.25f, 1.0f / 6.0f, 0.125f, 1.0f / 12.0f, 1.0f / 24.0f};
// An LFO on pitch: cents at full swing for Depth u = i / 64 (the source's law).
constexpr float kLfoCents[65] = {
    0.0f,      18.65008f, 46.33075f, 74.16143f, 102.0032f, 129.8409f, 157.6734f, 185.4967f, 214.5106f, 250.5557f, 287.7542f,
    315.5823f, 352.7851f, 390.0063f, 434.531f,  473.8332f, 511.0213f, 557.6022f, 604.191f,  647.9848f, 697.362f,  750.3214f,
    796.7277f, 852.5493f, 908.2979f, 964.1663f, 1029.408f, 1090.608f, 1155.638f, 1220.687f, 1290.172f, 1360.148f, 1434.649f,
    1513.547f, 1592.803f, 1676.691f, 1764.59f,  1853.594f, 1941.4f,   2043.657f, 2140.138f, 2242.395f, 2350.807f, 2459.523f,
    2577.397f, 2691.858f, 2819.454f, 2942.871f, 3075.3f,   3214.846f, 3354.179f, 3502.95f,  3661.212f, 3819.27f,  3978.982f,
    4147.698f, 4324.925f, 4510.254f, 4697.168f, 4901.074f, 5097.656f, 5311.523f, 5534.766f, 5758.155f, 6000.0f};
inline float lfoCents(float u) {
    const float x = clampf(u, 0.0f, 1.0f) * 64.0f;
    const int i = x < 64.0f ? int(x) : 63;
    return kLfoCents[i] + (kLfoCents[i + 1] - kLfoCents[i]) * (x - float(i));
}

// --- envelopes ------------------------------------------------------------------------
//
// One step per tick. The attack is linear (its time to full scale); the
// decay and the release keep the DISTANCE to their target and multiply it
// (the additive form stalls in float for long times), and land on it
// within 1/2048 (where the source's envelopes stop).
enum Stage : uint8_t { kStIdle = 0, kStAttack, kStDecay, kStRelease };
constexpr float kEnvStop = 1.0f / 2048.0f;

struct EnvCoef {
    float att;   // level per tick in the attack
    float dec;   // distance factor per tick in the decay
    float rel;   // ... in the release
    float sus;
};
// times: attack to full scale; decay / release to 1 % of the distance (4.6 time constants).
inline float tickFactor(float toOnePct) {
    const float t = toOnePct < 0.0001f ? 0.0001f : toOnePct;
    return fastExp2(-4.6f * va::kLog2e * float(kTick) * kInvFs / t);
}
inline void setEnv(EnvCoef& c, float att, float dec, float sus, float rel) {
    const float frames = att * kFs;
    c.att = float(kTick) / (frames < 1.0f ? 1.0f : frames);
    c.dec = tickFactor(dec);
    c.rel = tickFactor(rel);
    c.sus = clampf(sus, 0.0f, 1.0f);
}
inline float envStep(uint8_t& st, float e, const EnvCoef& c) {
    switch (st) {
        case kStAttack:
            e += c.att;
            if (e >= 1.0f) {
                e = 1.0f;
                st = kStDecay;
            }
            return e;
        case kStDecay: {
            const float d = (e - c.sus) * c.dec;
            return (d < kEnvStop && d > -kEnvStop) ? c.sus : c.sus + d;
        }
        case kStRelease:
            e *= c.rel;
            if (e < kEnvStop) {
                e = 0.0f;
                st = kStIdle;
            }
            return e;
        default: return 0.0f;
    }
}
// The filter and mod envelopes: the decay and release aim 1/2048 under
// their target and stop on it (the source's: they reach it in finite time,
// smoothly; the mod envelope's FM law reads the tail in dB).
inline float envStepAim(uint8_t& st, float e, const EnvCoef& c) {
    switch (st) {
        case kStDecay: {
            const float x = c.sus + (e - c.sus + kEnvStop) * c.dec - kEnvStop;
            return x <= c.sus ? c.sus : x;
        }
        case kStRelease: {
            const float x = (e + kEnvStop) * c.rel - kEnvStop;
            if (x <= 0.0f) {
                st = kStIdle;
                return 0.0f;
            }
            return x;
        }
        default: return envStep(st, e, c);
    }
}

// --- the filter section -------------------------------------------------------------
//
// LP12 / HP12 / BP12: the source's Chamberlin state-variable filter run twice
// per frame with the input held; f = 2 sin(pi fc / 2 fs), damping 1 - r,
// input gain 1 - 0.7 r. LP24: four bilinear one-poles with the feedback
// through a one-frame delay, k = (0.0297 x 127 r - 0.076) x comp(cutoff), and
// an AGC on the feedback (a peak follower, 1.2 ms / 0.2 s, (T / e)^0.883
// above T = 5.5) that bounds the self-oscillation. Both end in the output
// limiter: a gain on a follower of |y| (3.7 ms up, 6.9 ms down), 1 below
// 0.39. Then the saturation y = (1 + a) x / (1 + a |x|). The followers run
// per frame, the limiter gain per tick (from its follower at the tick's
// start), the AGC gain per frame but only above its threshold (a gain held
// for a tick beats with a self-oscillation near the tick rate); the cutoff
// coefficient ramps across the tick.
//
// The cutoff is driven on the source's knob axes: the SVF's 16.7 Hz x
// 2^(k / 12.624) (12.624 units per octave), the ladder's measured table.

constexpr float kKnobPerOct = 12.624f;
constexpr float kKnobHz = 16.7f;
constexpr float kKnobTop = 125.0f;           // the envelope target's top (both axes; SVF: 15.98 kHz, LP24: 20.1 kHz)
constexpr float kLimAtt = 0.0060844f, kLimRel = 0.0032844f;   // 3.714 ms / 6.89 ms at the device rate
constexpr float kAgcAtt = 0.0187116f, kAgcRel = 0.000112372f; // 1.2 ms / 0.2017 s
constexpr float kLimFloor = 0.39f;
constexpr float kAgcT = 5.5f, kAgcLog2T = 2.4594316f, kAgcExp = 0.883f;
constexpr float kSatMax = 103.489f;          // the saturation's a at 100 %

// The limiter's log2 gain on log2 of its envelope, -1.5 .. 2.5 in 1/16 octave
// ((1 + (e / 1.038)^11.54)^(-0.856 / 11.54)); past the top: slope -0.856.
constexpr float kLimitLog2[65] = {
    -4.282476e-07f, -7.060123e-07f, -1.163936e-06f, -1.91887e-06f, -3.16345e-06f, -5.215244e-06f, -8.597767e-06f, -1.417399e-05f,
    -2.336638e-05f, -3.851932e-05f, -6.349592e-05f, -0.0001046599f, -0.0001724886f, -0.0002842179f, -0.0004681617f, -0.0007707246f,
    -0.001267673f,  -0.002081941f,  -0.003410971f,  -0.005566615f,  -0.009028281f,  -0.01450152f,   -0.02295504f,   -0.03558042f,
    -0.05360663f,   -0.07796229f,   -0.1089283f,    -0.1460179f,    -0.18818f,      -0.2341677f,    -0.2828399f,    -0.3333017f,
    -0.3849159f,    -0.4372556f,    -0.4900456f,    -0.5431127f,    -0.5963492f,    -0.649689f,     -0.7030918f,    -0.7565328f,
    -0.8099969f,    -0.8634752f,    -0.916962f,     -0.970454f,     -1.023949f,     -1.077446f,     -1.130944f,     -1.184443f,
    -1.237943f,     -1.291442f,     -1.344942f,     -1.398442f,     -1.451942f,     -1.505442f,     -1.558942f,     -1.612442f,
    -1.665942f,     -1.719442f,     -1.772942f,     -1.826442f,     -1.879942f,     -1.933442f,     -1.986942f,     -2.040442f,
    -2.093942f};
// LP24 runs on the source ladder's own knob axis (its measured cutoff table,
// about 11.8 knob units per octave, compressing above 110): log2 of the
// stage cutoff and the feedback compensation at knob 2 i (i = 64: knob 127's).
constexpr float kLadderLog2Hz[65] = {
    4.004856f, 4.175295f, 4.345735f, 4.516174f, 4.686613f, 4.82554f,  5.002265f, 5.176557f, 5.348955f, 5.563809f, 5.739959f,
    5.904547f, 6.074631f, 6.240508f, 6.402346f, 6.563309f, 6.72629f,  6.889874f, 7.053081f, 7.225953f, 7.388372f, 7.55057f,
    7.712356f, 7.874323f, 8.036238f, 8.198272f, 8.36011f,  8.522332f, 8.694881f, 8.857607f, 9.020608f, 9.183769f, 9.347582f,
    9.511863f, 9.676703f, 9.842143f, 10.00808f, 10.1854f,  10.35324f, 10.52203f, 10.69173f, 10.86273f, 11.035f,   11.20853f,
    11.38364f, 11.56017f, 11.74945f, 11.92922f, 12.11048f, 12.29292f, 12.47637f, 12.66007f, 12.84324f, 13.02466f, 13.20265f,
    13.38589f, 13.55026f, 13.70437f, 13.84594f, 13.97315f, 14.08496f, 14.18099f, 14.26146f, 14.30008f, 14.30008f};
constexpr float kLadderComp[65] = {
    1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f,
    1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.061999f,
    1.052514f, 1.049024f, 1.04311f,  1.038266f, 1.034161f, 1.03016f,  1.025417f, 1.019255f, 1.014816f, 1.008378f, 1.001446f,
    0.9940841f, 0.9859948f, 0.9765442f, 0.9661619f, 0.9539916f, 0.9414855f, 0.9275236f, 0.9121978f, 0.8954675f, 0.8772041f,
    0.8573293f, 0.8359052f, 0.8128956f, 0.7866801f, 0.7603873f, 0.7325484f, 0.7032394f, 0.6727081f, 0.6411332f, 0.6086937f,
    0.5756683f, 0.5424138f, 0.5072785f, 0.4748396f, 0.4434731f, 0.4136966f, 0.3860546f, 0.3611333f, 0.3395972f, 0.3222306f,
    0.3146622f, 0.3146622f};

// (out of line: per tick, and per frame only above the AGC's threshold)
__attribute__((noinline)) inline float limitGainSlow(float e) {
    const float x = (fastLog2(e) + 1.5f) * 16.0f;
    float lg;
    if (x >= 64.0f) {
        lg = kLimitLog2[64] - 0.856f * (x - 64.0f) * (1.0f / 16.0f);
    } else {
        const int i = x > 0.0f ? int(x) : 0;
        lg = kLimitLog2[i] + (kLimitLog2[i + 1] - kLimitLog2[i]) * (x - float(i));
    }
    return fastExp2(lg);
}
inline float limitGain(float e) { return e <= kLimFloor ? 1.0f : limitGainSlow(e); }
__attribute__((noinline)) inline float agcGainSlow(float e) { return fastExp2(kAgcExp * (kAgcLog2T - fastLog2(e))); }
inline float agcGain(float e) { return e > kAgcT ? agcGainSlow(e) : 1.0f; }
inline float satA(float s) { return s <= 0.0f ? 0.0f : kSatMax * (fastExp2(10.0f * clampf(s, 0.0f, 1.0f)) - 1.0f) * (1.0f / 1023.0f); }
inline float knobOfHz(float hz) { return kKnobPerOct * fastLog2((hz < 1.0f ? 1.0f : hz) * (1.0f / kKnobHz)); }
inline float hzOfKnob(float k) { return kKnobHz * fastExp2(k * (1.0f / kKnobPerOct)); }
// sin(x), 0 <= x <= pi / 2 (Taylor to x^9).
inline float sinPoly(float x) {
    const float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f + x2 * (1.0f / 362880.0f)))));
}
// SVF f for knob k (clamped to 0 .. 125).
inline float svfF(float k) { return 2.0f * sinPoly(va::kPi * hzOfKnob(clampf(k, 0.0f, kKnobTop)) * (0.5f * kInvFs)); }
// LP24: the stage cutoff of ladder knob k (0..127), the knob of a cutoff.
inline float ladderHz(float k) {
    const float x = clampf(k, 0.0f, 127.0f) * 0.5f;
    const int i = x < 63.0f ? int(x) : 63;
    return fastExp2(kLadderLog2Hz[i] + (kLadderLog2Hz[i + 1] - kLadderLog2Hz[i]) * (x - float(i)));
}
inline float ladderKnobOfHz(float hz) {
    const float l = fastLog2(hz < 1.0f ? 1.0f : hz);
    if (l <= kLadderLog2Hz[0]) return 0.0f;
    int lo = 0, hi = 63;   // kLadderLog2Hz[lo] < l <= kLadderLog2Hz[hi]
    if (l >= kLadderLog2Hz[63]) return 126.0f;
    while (hi - lo > 1) {
        const int m = (lo + hi) >> 1;
        if (kLadderLog2Hz[m] < l) lo = m;
        else hi = m;
    }
    return 2.0f * (float(lo) + (l - kLadderLog2Hz[lo]) / (kLadderLog2Hz[hi] - kLadderLog2Hz[lo]));
}
// LP24 stage coefficient a = g / (1 + g), g = tan(pi f / fs), and the feedback k, for ladder knob k.
inline float ladderA(float k) {
    const float hz = ladderHz(k);
    const float g = va::tanPade(va::kPi * (hz < 0.49f * kFs ? hz : 0.49f * kFs) * kInvFs);
    return g / (1.0f + g);
}
inline float ladderK(float k, float res) {
    const float x = clampf(k, 0.0f, 127.0f) * 0.5f;
    const int i = x < 63.0f ? int(x) : 63;
    const float comp = kLadderComp[i] + (kLadderComp[i + 1] - kLadderComp[i]) * (x - float(i));
    const float r = 3.7719f * res - 0.076f;
    return (r > 0.0f ? r : 0.0f) * comp;
}

// SVF damping 1 - r, with a small floor (undamped, its state would grow
// without bound under a tone at the cutoff).
inline float svfDamp(float r) { return 1.0f - r > 0.004f ? 1.0f - r : 0.004f; }

struct Filter {
    float s[6];   // SVF: low, band; LP24: four stages, the delayed output, the AGC follower
    float lim;    // the limiter's follower
};
enum FilterKind : uint8_t { kKindSvf = 0, kKindLadder };
inline uint8_t kindOf(int type) { return type == kLp24 || type == kLpdl ? kKindLadder : kKindSvf; }

// One tick of the filter section on x (in place). c0 -> c1: the cutoff
// coefficient (SVF f, LP24 a) ramped across the tick; k: the LP24 feedback
// (k0 -> k1); svfType: which SVF output; d, g: the SVF damping and input
// gain; sat: the saturation's a (0 = off).
template <int kKind>
inline void filterTick(Filter& F, float* x, float c0, float c1, float k0, float k1, int svfType, float d, float g, float sat) {
    const float dc = (c1 - c0) * (1.0f / float(kTick)), dk = (k1 - k0) * (1.0f / float(kTick));
    const float lg = limitGain(F.lim);
    float c = c0, k = k0, lim = F.lim;
    if (kKind == kKindSvf) {
        float low = F.s[0], band = F.s[1];
        for (int i = 0; i < kTick; ++i) {
            const float in = g * x[i];
            low += c * band;
            float high = in - low - d * band;
            band += c * high;
            low += c * band;
            high = in - low - d * band;
            band += c * high;
            const float y = svfType == kLp12 ? low : (svfType == kHp12 ? high : band);
            const float a = y < 0.0f ? -y : y;
            lim += (a > lim ? kLimAtt : kLimRel) * (a - lim);
            float o = y * lg;
            if (sat > 0.0f) o = (1.0f + sat) * o / (1.0f + sat * (o < 0.0f ? -o : o));
            x[i] = o;
            c += dc;
        }
        F.s[0] = va::flush(low);
        F.s[1] = va::flush(band);
    } else {
        float s0 = F.s[0], s1 = F.s[1], s2 = F.s[2], s3 = F.s[3], fb = F.s[4], agc = F.s[5];
        for (int i = 0; i < kTick; ++i) {
            float u = x[i] - k * agcGain(agc) * fb, v, y;   // the AGC per frame (held per tick, it beats with the tone)
            v = (u - s0) * c; y = v + s0; s0 = y + v; u = y;
            v = (u - s1) * c; y = v + s1; s1 = y + v; u = y;
            v = (u - s2) * c; y = v + s2; s2 = y + v; u = y;
            v = (u - s3) * c; y = v + s3; s3 = y + v; u = y;
            fb = u;
            const float a = u < 0.0f ? -u : u;
            agc += (a > agc ? kAgcAtt : kAgcRel) * (a - agc);
            lim += (a > lim ? kLimAtt : kLimRel) * (a - lim);
            float o = u * lg;
            if (sat > 0.0f) o = (1.0f + sat) * o / (1.0f + sat * (o < 0.0f ? -o : o));
            x[i] = o;
            c += dc;
            k += dk;
        }
        F.s[0] = va::flush(s0);
        F.s[1] = va::flush(s1);
        F.s[2] = va::flush(s2);
        F.s[3] = va::flush(s3);
        F.s[4] = va::flush(fb);
        F.s[5] = agc;
    }
    F.lim = lim;
}

// The filter section with static settings, a tick at a time: the engine
// test's probe (type, cutoff Hz as the engine's Cutoff, resonance 0..1,
// saturation 0..1).
struct Section {
    Filter f;
    int type = 0;
    float c = 0, k = 0, d = 1, g = 1, a = 0;
    void init(int t, float hz, float res, float sat) {
        std::memset(&f, 0, sizeof(f));
        type = t;
        const float r = clampf(res, 0.0f, 1.0f);
        if (kindOf(t) == kKindSvf) {
            c = svfF(knobOfHz(hz));
            d = svfDamp(r);
            g = 1.0f - 0.7f * r;
        } else {
            const float knob = ladderKnobOfHz(hz);
            c = ladderA(knob);
            k = ladderK(knob, r);
        }
        a = satA(sat);
    }
    void run(const float* x, float* y) {
        for (int i = 0; i < kTick; ++i) y[i] = x[i];
        if (kindOf(type) == kKindSvf) filterTick<kKindSvf>(f, y, c, c, 0.0f, 0.0f, type, d, g, a);
        else filterTick<kKindLadder>(f, y, c, c, k, k, type, d, g, a);
    }
};

}  // namespace dsp
}  // namespace onetype
}  // namespace audio
}  // namespace ot
