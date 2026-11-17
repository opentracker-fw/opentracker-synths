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
// The cutoff is driven on the knob axis of the source's SVF: 16.7 Hz x
// 2^(k / 12.624) (12.624 units per octave).

constexpr float kKnobPerOct = 12.624f;
constexpr float kKnobHz = 16.7f;
constexpr float kSvfTop = 125.0f;            // the SVF's cutoff stops at knob 125 (15.98 kHz)
constexpr float kLadderTop = 129.1966f;      // the LP24 target's top (its own knob 125, 20.1 kHz)
constexpr float kLadderMinHz = 16.054f, kLadderMaxHz = 20172.19f;
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
// LP24 feedback compensation on the knob axis, k = 2 i.
constexpr float kLadderComp[66] = {
    1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f,
    1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.081027f, 1.080977f,
    1.062369f, 1.052794f, 1.049257f, 1.043637f, 1.038686f, 1.034657f, 1.030689f, 1.026436f, 1.020885f, 1.016055f, 1.01061f,
    1.003815f, 0.9970397f, 0.9895982f, 0.9809957f, 0.9715737f, 0.9611846f, 0.9499861f, 0.9378017f, 0.9244014f, 0.9098924f,
    0.8942767f, 0.8774613f, 0.8593651f, 0.8401484f, 0.8198121f, 0.798378f, 0.7758111f, 0.7521923f, 0.7275937f, 0.7020628f,
    0.6757112f, 0.6486089f, 0.6208066f, 0.592321f, 0.5631359f, 0.533273f, 0.5027391f, 0.4713648f, 0.4389812f, 0.4054153f,
    0.3705836f, 0.3351587f, 0.3146622f};

inline float limitGain(float e) {
    if (e <= kLimFloor) return 1.0f;
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
inline float agcGain(float e) { return e > kAgcT ? fastExp2(kAgcExp * (kAgcLog2T - fastLog2(e))) : 1.0f; }
inline float satA(float s) { return s <= 0.0f ? 0.0f : kSatMax * (fastExp2(10.0f * clampf(s, 0.0f, 1.0f)) - 1.0f) * (1.0f / 1023.0f); }
inline float knobOfHz(float hz) { return kKnobPerOct * fastLog2((hz < 1.0f ? 1.0f : hz) * (1.0f / kKnobHz)); }
inline float hzOfKnob(float k) { return kKnobHz * fastExp2(k * (1.0f / kKnobPerOct)); }
// sin(x), 0 <= x <= pi / 2 (Taylor to x^9).
inline float sinPoly(float x) {
    const float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f + x2 * (1.0f / 362880.0f)))));
}
// SVF f for knob k (clamped to 0 .. 125).
inline float svfF(float k) { return 2.0f * sinPoly(va::kPi * hzOfKnob(clampf(k, 0.0f, kSvfTop)) * (0.5f * kInvFs)); }
// LP24 stage coefficient a = g / (1 + g), g = tan(pi f / fs), and the feedback k, for knob k.
inline float ladderA(float k) {
    const float hz = clampf(hzOfKnob(k), kLadderMinHz, kLadderMaxHz);
    const float g = va::tanPade(va::kPi * (hz < 0.49f * kFs ? hz : 0.49f * kFs) * kInvFs);
    return g / (1.0f + g);
}
inline float ladderK(float k, float res) {
    const float x = clampf(k * 0.5f, 0.0f, 65.0f);
    const int i = x < 65.0f ? int(x) : 64;
    const float comp = kLadderComp[i] + (kLadderComp[i + 1] - kLadderComp[i]) * (x - float(i));
    const float r = 3.7719f * res - 0.076f;
    return (r > 0.0f ? r : 0.0f) * comp;
}

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
        const float knob = knobOfHz(hz);
        const float r = clampf(res, 0.0f, 1.0f);
        if (kindOf(t) == kKindSvf) {
            c = svfF(knob);
            d = 1.0f - r;
            g = 1.0f - 0.7f * r;
        } else {
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
