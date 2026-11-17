// 1-Type engine (see onetype.h): a single-precision port of the voice of
// Oxo Synth2.
//
// A block is 8 control ticks of 16 frames. At the 9 tick points the voice
// computes its oscillator increments, the FM deviation, the pulse width,
// the filter coefficients and the amp gain (from its envelopes, glide, the
// slot's two LFOs at those points, and the parameters ramped from the
// block's start to its end); every per-frame value ramps linearly from one
// point to the next. Then the passes over 128-frame buffers: the
// oscillators (polyBLEP, one frame late), osc 1's two detuned copies, the
// sub (its phase derived from osc 1's plus a wrap parity bit: locked
// exactly), the mix with the ring modulator, the filter section
// (onetype_dsp.h), the amp into the slot bus. With FM osc 2 runs first and
// steps osc 1; with sync osc 1 runs first and resets osc 2 (band-limited for
// the saw and pulse); with both, one loop runs them together. The LFO on
// Pan acts on the slot bus (slotPost).
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
#include "onetype.h"

#include <cstring>

#include "audio/va_dsp.h"

#pragma GCC diagnostic error "-Wdouble-promotion"

#include "onetype_dsp.h"

#if defined(__GNUC__) && !defined(__clang__)
#define ONETYPE_PER_BLOCK __attribute__((optimize("Os")))
#else
#define ONETYPE_PER_BLOCK
#endif

namespace ot {
namespace audio {
namespace onetype {

namespace {

using namespace dsp;

constexpr float kOut = 0.76f;   // osc full scale -> the slot bus: the Init patch at the level of the other synths
constexpr float kWidthMin = 1.25f / 254.0f;             // the final pulse duty never reaches 0 or 1
constexpr float kWidthMax = 1.0f - 1.375f / 254.0f;
constexpr float kTickFrac = 1.0f / float(kTick);
constexpr float kFmLog2 = 9.9672262f;                   // log2(1001): FM deviation (1001^u - 1) / 2000 per frame
constexpr float kNoiseFm = 0.65f / kNoiseLevel;         // noise as an FM source is +-0.65
constexpr float kGlideAim = 0.0135f;                    // a glide aims 1.35 % of its distance past the note
constexpr float kGlideLand = 4.3184f;                   // ... and lands after 4.318 time constants
constexpr float kDetuneGain = 0.5145f, kDetuneGainSine = 0.4988f;   // osc 1 + its copies, the source's 9-voice level
constexpr float kSubDetune = 0.891f, kSubDetuneRing = 3.0f;         // the sub with detune on (ring: the source's louder sub)
constexpr float kLog2Of1e4 = 13.287712f;

// Continuous parameters at the block start (0) and end (1): the tick points lerp them.
struct Cont {
    float shift, tune;   // osc 1 + 2 key shift, fine tune (semitones)
    float osc2;          // osc 2 pitch + fine (semitones)
    float gA, gS, gB;    // mix weights: osc 1, sub, osc 2
    float width;         // pulse duty of the low part, 0 .. 0.5
    float cut;           // cutoff on the knob axis (dsp::knobOfHz)
    float env;           // filter envelope depth (knob units at envelope 1)
    float track;         // key track (knob units per semitone)
    float res;           // resonance 0..1
    float fm;            // FM 0..1
};

// One slot LFO: free running per slot (key sync restarts it), its value at
// the block's 9 tick points.
struct Lfo {
    float ph, prev, cur;     // phase; the random values of the last and this cycle
    uint32_t rng;
    float v[kTicks + 1];
    float inc;               // phase per tick
    float depth, cents;      // 0..1; pitch: cents at full swing
    uint8_t dest, wave, keySync, on;
};

struct SlotState {
    uint32_t serial;
    uint32_t seed;           // free-running start phases, the noise of later voices
    uint8_t valid, hasLast, notes, lfoInit;
    uint8_t shape1, shape2, subShape, subOn;
    uint8_t subDown, track2, phaseOn, glideMode;
    uint8_t ftype, kind, sync, ring;
    uint8_t detune, modDest, fmOn, ringFm;
    float phase2;            // osc 2's start phase (Phase > 0)
    float copyPhase;         // the lower copy's start phase (Phase > 0)
    float lastPitch;
    float sat;               // the saturation's a
    float svfD, svfG;        // SVF damping and input gain (block end)
    float ratio[2];          // the copies' pitch ratios
    float mod;               // mod envelope amount -1..1
    float glideA;            // glide: distance factor per tick (0 = no glide)
    float ringGain;          // the ring product's osc 2 gain
    EnvCoef amp, fenv, menv;
    Cont c[2];
    Lfo lfo[2];
};
static_assert(sizeof(SlotState) <= kSynthSlotScratchBytes, "1-Type slot state");

struct VoiceState {
    float ph1, ph2, pc[2];   // oscillator phases 0..1 (osc 1, osc 2, osc 1's copies)
    float z1, z2, zs, zc[2]; // samples held one frame (osc 1, osc 2, sub, copies)
    float amp, fenv, menv;   // amp, filter and mod envelopes
    float pitch, target, aim;   // glide: the played pitch, the note, the aim past it
    uint32_t noise;
    Filter F;
    uint8_t hi1, hi2, his, parity;   // frame states (pulse high / triangle falling); the sub's wrap parity
    uint8_t hic[2], init, pad0;
    uint8_t aSt, fSt, mSt, started;
    uint8_t gate, pad1, pad2, pad3;
};
static_assert(sizeof(VoiceState) <= kSynthVoiceStateBytes, "1-Type voice state");

// One control point.
struct Point {
    float incA, incB, w, amp;
    float c, k;              // filter: SVF f or LP24 a; LP24 feedback
    float fm;                // FM deviation (cycles per frame at full osc 2)
};

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float unit(uint32_t& s) {   // the source LFO's random values: xorshift, -1 .. 1
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return float(s) * (2.0f / 4294967296.0f) - 1.0f;
}
inline float rnd01(uint32_t& seed) {
    seed = seed * 1664525u + 1013904223u;
    return float(seed >> 8) * (1.0f / 16777216.0f);
}

ONETYPE_PER_BLOCK void contOf(Cont& c, const float* E) {
    c.shift = clampf(E[kKeyShift], -24.0f, 24.0f);
    c.tune = 0.01f * clampf(E[kFineTune], -100.0f, 100.0f);
    c.osc2 = clampf(E[kOsc2Pitch], -60.0f, 60.0f) + 0.01f * clampf(E[kOsc2Fine], -100.0f, 100.0f);
    // The mix (the source's law): osc 1 (1 - w)(1 - m), sub w (1 - m), osc 2
    // (1 - w) m, all over 1 - w m; w = g / (g + 32), g the sub level in knob units.
    const float m = clampf(E[kMix], 0.0f, 1.0f), g = 127.0f * clampf(E[kSubLevel], 0.0f, 1.0f);
    const float w = g / (g + 32.0f), norm = 1.0f / (1.0f - m * w);
    c.gA = (1.0f - m) * (1.0f - w) * norm;
    c.gS = (1.0f - m) * w * norm;
    c.gB = m * (1.0f - w) * norm;
    c.width = 0.5f * clampf(E[kPulseWidth], 0.0f, 1.0f);
    c.cut = knobOfHz(clampf(E[kCutoff], 16.0f, 20000.0f));
    c.env = 125.0f * clampf(E[kEnvAmt], -1.0f, 1.0f);
    c.track = clampf(E[kKeyTrack], 0.0f, 1.0f) * (kKnobPerOct / 12.0f);
    c.res = clampf(E[kResonance], 0.0f, 1.0f);
    c.fm = clampf(E[kOsc1Fm], 0.0f, 1.0f);
}

void lfoInit(Lfo& l) {
    l.ph = 0.0f;
    l.rng = 0x2545F491u;
    l.prev = unit(l.rng);
    l.cur = unit(l.rng);
}
void lfoNext(Lfo& l) {
    l.prev = l.cur;
    l.cur = unit(l.rng);
}
// The LFO waves (the source's: saw falling, triangle from -1, square + first,
// S&H, smooth random (on Amp from -r to minus the last r), sine).
inline float lfoWave(const Lfo& l) {
    const float p = l.ph;
    switch (l.wave) {
        case kLfoSaw: return 1.0f - 2.0f * p;
        case kLfoTri: return p < 0.5f ? -1.0f + 4.0f * p : 3.0f - 4.0f * p;
        case kLfoSquare: return p < 0.5f ? 1.0f : -1.0f;
        case kLfoSh: return l.cur;
        case kLfoSmooth: return l.dest == kToAmp ? -(l.cur + (l.prev - l.cur) * p) : l.prev + (l.cur - l.prev) * p;
        default: return va::sinePar(p);
    }
}

ONETYPE_PER_BLOCK void refresh(SlotState& S, const SynthParams& p) {
    const float* E = p.end;
    S.shape1 = uint8_t(va::enumOf(E[kOsc1Wave], kTri));
    S.shape2 = uint8_t(va::enumOf(E[kOsc2Wave], kOsc2Noise));
    S.subShape = uint8_t(va::enumOf(E[kSubWave], kSubSquare));
    S.subOn = E[kSubLevel] > 0.0f || p.start[kSubLevel] > 0.0f;
    S.subDown = E[kSubOctave] >= 0.5f;
    S.track2 = E[kOsc2Track] >= 0.5f;
    S.sync = E[kSync] >= 0.5f;
    S.ring = E[kRing] >= 0.5f;
    const float ph = clampf(E[kPhase], 0.0f, 1.0f), restart = clampf((127.0f * ph - 1.0f) * (1.0f / 126.0f), 0.0f, 1.0f);
    S.phaseOn = ph > 0.0f;
    S.phase2 = 0.5f * restart;
    S.copyPhase = (202.0f / 360.0f) * restart;
    S.glideMode = uint8_t(va::enumOf(E[kGlideMode], kLegatoOverlap));
    setEnv(S.amp, E[kAttack], E[kDecay], E[kSustain], E[kRelease]);
    setEnv(S.fenv, E[kFAttack], E[kFDecay], E[kFSustain], E[kFRelease]);
    setEnv(S.menv, E[kModAttack], E[kModDecay], 0.0f, E[kModDecay]);
    S.ftype = uint8_t(va::enumOf(E[kFilterType], kLpdl));
    S.kind = kindOf(S.ftype);
    const float r = clampf(E[kResonance], 0.0f, 1.0f);
    S.svfD = 1.0f - r;
    S.svfG = 1.0f - 0.7f * r;
    S.sat = satA(E[kSaturation]);
    // Osc 1's two copies at +-20 cents x Detune.
    const float det = clampf(E[kOsc1Detune], 0.0f, 1.0f);
    S.detune = det > 0.0f;
    S.ratio[0] = fastExp2(-20.0f * det * (1.0f / 1200.0f));
    S.ratio[1] = fastExp2(20.0f * det * (1.0f / 1200.0f));
    S.mod = clampf(E[kModAmount], -1.0f, 1.0f);
    S.modDest = uint8_t(va::enumOf(E[kModDest], kModPw));
    for (int k = 0; k < 2; ++k) {
        Lfo& l = S.lfo[k];
        const float* L = E + kLfo1Dest + k * kLfoStride;
        l.dest = uint8_t(va::enumOf(L[kLfoDest], kToPan));
        l.wave = uint8_t(va::enumOf(L[kLfoWave], kLfoSine));
        l.depth = clampf(L[kLfoDepth], 0.0f, 1.0f);
        l.on = l.depth > 0.0f;   // depth 0: no route at all
        l.cents = lfoCents(l.depth);
        l.keySync = L[kLfoKeySync] >= 0.5f;
    }
    // FM: the knob, an LFO on FM, the mod envelope on FM; none with ring on.
    const bool fmLfo = (S.lfo[0].on && S.lfo[0].dest == kToFm) || (S.lfo[1].on && S.lfo[1].dest == kToFm);
    const bool fmEnv = S.mod != 0.0f && S.modDest == kModFm;
    S.fmOn = !S.ring && (E[kOsc1Fm] > 0.0f || p.start[kOsc1Fm] > 0.0f || fmLfo || fmEnv);
    // Ring with an FM source in the patch: the product takes osc 2's pulse at
    // full scale (x 2) and the noise at +-1 (the source's rule).
    S.ringFm = E[kOsc1Fm] > 0.0f || fmLfo || (fmEnv && S.mod >= 3.0f / 63.0f);
    S.ringGain = S.ringFm ? (S.shape2 == kOsc2Pulse ? 2.0f : (S.shape2 == kOsc2Noise ? 1.0f / kNoiseLevel : 1.0f)) : 1.0f;
    const float gt = clampf(E[kGlideTime], 0.0f, 3.0f);
    S.glideA = gt < 0.0005f ? 0.0f : fastExp2(-va::kLog2e * float(kTick) * kInvFs * kGlideLand / gt);
    contOf(S.c[0], p.start);
    contOf(S.c[1], E);
    S.serial = p.serial;
    S.valid = 1;
}

// The oscillator frame of shape K (osc 1's numbering: sine, saw, pulse, tri).
template <int K>
inline float frame(float p, float d, bool wrap, float w, float& z, uint8_t& hi) {
    if (K == kSine) return sineFrame(p, z);
    if (K == kSaw) return sawFrame(p, d, wrap, z);
    if (K == kPulse) return pulseFrame(p, d, wrap, w, z, hi);
    return triFrame(p, d, z, hi);
}
// The naive wave (a backward FM step).
template <int K>
inline float naive(float p, float w) {
    if (K == kSine) return va::sinePar(p);
    if (K == kSaw) return 1.0f - 2.0f * p;
    if (K == kPulse) return p < 1.0f - w ? w : w - 1.0f;
    return triNaive(p);
}
// One step of osc 1 (or a copy) by d (FM: may be <= 0: then the naive wave).
template <int K>
inline float step1(float& p, float d, float w, float& z, uint8_t& hi) {
    if (d > 0.0f) {
        p += d;
        const bool wrap = p >= 1.0f;
        if (wrap) p -= 1.0f;
        return frame<K>(p, d, wrap, w, z, hi);
    }
    p += d;
    if (p < 0.0f) p += 1.0f;
    const float out = z;
    z = naive<K>(p, w);
    hi = K == kPulse ? pulseHi(p, w) : triHi(p);
    return out;
}

// Osc 1 into a; its phase after each frame into ph (the sub's and sync's);
// kFm: each step + the FM deviation x osc 2 (b).
template <int K, bool kPh, bool kFm>
__attribute__((noinline)) void osc1Pass(VoiceState& V, const Point* P, float* a, float* ph, const float* b) {
    float p = V.ph1, z = V.z1;
    uint8_t hi = V.hi1;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d = P[t].incA, w = P[t].w, f = P[t].fm;
        const float dd = (P[t + 1].incA - d) * kTickFrac, dw = (P[t + 1].w - w) * kTickFrac, df = (P[t + 1].fm - f) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            if (kFm) {
                a[n] = step1<K>(p, clampf(d + f * b[n], -va::kMaxDt, va::kMaxDt), w, z, hi);
                f += df;
            } else {
                p += d;
                const bool wrap = p >= 1.0f;
                if (wrap) p -= 1.0f;
                a[n] = frame<K>(p, d, wrap, w, z, hi);
            }
            if (kPh) ph[n] = p;
            d += dd;
            w += dw;
        }
    }
    V.ph1 = p;
    V.z1 = z;
    V.hi1 = hi;
}

// Osc 1's two copies (detune), added into a, then the sum scaled.
template <int K, bool kFm>
__attribute__((noinline)) void copiesPass(VoiceState& V, const SlotState& S, const Point* P, float* a, const float* b) {
    float p0 = V.pc[0], p1 = V.pc[1], z0 = V.zc[0], z1 = V.zc[1];
    uint8_t h0 = V.hic[0], h1 = V.hic[1];
    const float r0 = S.ratio[0], r1 = S.ratio[1], gain = K == kSine ? kDetuneGainSine : kDetuneGain;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d = P[t].incA, w = P[t].w, f = P[t].fm;
        const float dd = (P[t + 1].incA - d) * kTickFrac, dw = (P[t + 1].w - w) * kTickFrac, df = (P[t + 1].fm - f) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            // (the source: every copy takes osc 1's FM deviation as it is)
            const float dev = kFm ? f * b[n] : 0.0f;
            const float c0 = step1<K>(p0, clampf(d * r0 + dev, -va::kMaxDt, va::kMaxDt), w, z0, h0);
            const float c1 = step1<K>(p1, clampf(d * r1 + dev, -va::kMaxDt, va::kMaxDt), w, z1, h1);
            a[n] = (a[n] + c0 + c1) * gain;
            d += dd;
            w += dw;
            if (kFm) f += df;
        }
    }
    V.pc[0] = p0;
    V.pc[1] = p1;
    V.zc[0] = z0;
    V.zc[1] = z1;
    V.hic[0] = h0;
    V.hic[1] = h1;
}

// Osc 2 into b (osc 2's numbering: saw, pulse, tri, noise); kSync: reset at
// osc 1's forward wraps (osc 1's phases in ph, from: its phase before the
// block), band-limited for the saw and the pulse; the noise restarts.
template <int K, bool kSync>
__attribute__((noinline)) void osc2Pass(VoiceState& V, const Point* P, float* b, const float* ph, float from) {
    float p = V.ph2, z = V.z2, prev = from;
    uint8_t hi = V.hi2;
    uint32_t nz = V.noise;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d = P[t].incB, w = P[t].w;
        const float dd = (P[t + 1].incB - d) * kTickFrac, dw = (P[t + 1].w - w) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            float x = -1.0f;   // osc 1 wrapped x frames ago
            if (kSync) {
                const float q = ph[n], step = q - prev;
                if (step < -0.5f) x = clampf(q / (step + 1.0f), 0.0f, 0.999f);
                prev = q;
            }
            if (K == kOsc2Saw || K == kOsc2Pulse) {
                if (kSync && x >= 0.0f) {
                    b[n] = syncFrame<K == kOsc2Saw ? 0 : 1>(p, z, d, w, x);
                    hi = pulseHi(p, w);
                } else {
                    p += d;
                    const bool wrap = p >= 1.0f;
                    if (wrap) p -= 1.0f;
                    b[n] = K == kOsc2Saw ? sawFrame(p, d, wrap, z) : pulseFrame(p, d, wrap, w, z, hi);
                }
            } else if (K == kOsc2Tri) {
                if (kSync && x >= 0.0f) {
                    p = x * d;   // a naive reset
                    hi = triHi(p);
                } else {
                    p += d;
                    if (p >= 1.0f) p -= 1.0f;
                }
                b[n] = triFrame(p, d, z, hi);
            } else {
                if (kSync && x >= 0.0f) nz = kNoiseSeed;
                b[n] = noiseStep(nz);
            }
            d += dd;
            w += dw;
        }
    }
    V.ph2 = p;
    V.z2 = z;
    V.hi2 = hi;
    V.noise = nz;
}

// FM and sync at once: osc 2 (reset by osc 1's last wrap) steps osc 1 frame
// by frame. Rare; generic in the shapes.
__attribute__((noinline)) void crossPass(VoiceState& V, const SlotState& S, const Point* P, float* a, float* b, float* ph) {
    float p1 = V.ph1, z1 = V.z1, p2 = V.ph2, z2 = V.z2, x = -1.0f;
    uint8_t h1 = V.hi1, h2 = V.hi2;
    uint32_t nz = V.noise;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d1 = P[t].incA, d2 = P[t].incB, w = P[t].w, f = P[t].fm;
        const float dd1 = (P[t + 1].incA - d1) * kTickFrac, dd2 = (P[t + 1].incB - d2) * kTickFrac;
        const float dw = (P[t + 1].w - w) * kTickFrac, df = (P[t + 1].fm - f) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            float o2;
            switch (S.shape2) {
                case kOsc2Saw:
                    if (x >= 0.0f) o2 = syncFrame<0>(p2, z2, d2, w, x);
                    else o2 = step1<kSaw>(p2, d2, w, z2, h2);
                    break;
                case kOsc2Pulse:
                    if (x >= 0.0f) {
                        o2 = syncFrame<1>(p2, z2, d2, w, x);
                        h2 = pulseHi(p2, w);
                    } else {
                        o2 = step1<kPulse>(p2, d2, w, z2, h2);
                    }
                    break;
                case kOsc2Tri:
                    if (x >= 0.0f) {
                        p2 = x * d2;
                        h2 = triHi(p2);
                        o2 = triFrame(p2, d2, z2, h2);
                    } else {
                        o2 = step1<kTri>(p2, d2, w, z2, h2);
                    }
                    break;
                default:
                    if (x >= 0.0f) nz = kNoiseSeed;
                    o2 = noiseStep(nz);
                    break;
            }
            b[n] = o2;
            const float s1 = clampf(d1 + f * o2, -va::kMaxDt, va::kMaxDt), before = p1;
            switch (S.shape1) {
                case kSine: a[n] = step1<kSine>(p1, s1, w, z1, h1); break;
                case kSaw: a[n] = step1<kSaw>(p1, s1, w, z1, h1); break;
                case kPulse: a[n] = step1<kPulse>(p1, s1, w, z1, h1); break;
                default: a[n] = step1<kTri>(p1, s1, w, z1, h1); break;
            }
            x = s1 > 0.0f && p1 < before ? clampf(p1 / s1, 0.0f, 0.999f) : -1.0f;
            ph[n] = p1;
            d1 += dd1;
            d2 += dd2;
            w += dw;
            f += df;
        }
    }
    V.ph1 = p1;
    V.z1 = z1;
    V.hi1 = h1;
    V.ph2 = p2;
    V.z2 = z2;
    V.hi2 = h2;
    V.noise = nz;
}

// The sub (sub numbering: sine, tri, saw, square) from osc 1's phases in c,
// in place: its phase is osc 1's (octave 0) or half of osc 1's plus the
// parity of osc 1's wraps (octave -1). from: osc 1's phase before the block.
template <int K>
__attribute__((noinline)) void subPass(VoiceState& V, float from, bool down, float* c) {
    float prev = from, z = V.zs;
    uint8_t hi = V.his, par = V.parity;
    for (int n = 0; n < int(kBlock); ++n) {
        const float p = c[n];
        float d = p - prev;
        const bool fwd = d < -0.5f, back = d > 0.5f;   // a forward / backward wrap of osc 1
        if (fwd) d += 1.0f;
        if (back) d -= 1.0f;
        par = uint8_t(par ^ uint8_t(fwd || back));
        float sp = p, sd = d;
        bool wrap = fwd;
        if (down) {
            sp = 0.5f * (p + float(par));
            sd = 0.5f * d;
            wrap = fwd && !par;
        }
        float s;
        if (sd <= 0.0f) {   // osc 1 stepping backward (FM): the naive wave
            s = z;
            z = K == kSubSine ? va::sinePar(sp) : K == kSubTri ? triNaive(sp) : K == kSubSaw ? 1.0f - 2.0f * sp : (sp < 0.5f ? 0.5f : -0.5f);
            hi = K == kSubTri ? triHi(sp) : pulseHi(sp, 0.5f);
        } else if (K == kSubSine) {
            s = sineFrame(sp, z);
        } else if (K == kSubTri) {
            s = triFrame(sp, sd, z, hi);
        } else if (K == kSubSaw) {
            s = sawFrame(sp, sd, wrap, z);
        } else {
            s = pulseFrame(sp, sd, wrap, 0.5f, z, hi);
        }
        c[n] = s;
        prev = p;
    }
    V.zs = z;
    V.his = hi;
    V.parity = par;
}

using Osc1Fn = void (*)(VoiceState&, const Point*, float*, float*, const float*);
#define ONETYPE_OSC1(ph, fm) {osc1Pass<kSine, ph, fm>, osc1Pass<kSaw, ph, fm>, osc1Pass<kPulse, ph, fm>, osc1Pass<kTri, ph, fm>}
const Osc1Fn kOsc1Fn[2][2][4] = {{ONETYPE_OSC1(false, false), ONETYPE_OSC1(false, true)},
                                 {ONETYPE_OSC1(true, false), ONETYPE_OSC1(true, true)}};
#undef ONETYPE_OSC1
using CopiesFn = void (*)(VoiceState&, const SlotState&, const Point*, float*, const float*);
const CopiesFn kCopiesFn[2][4] = {{copiesPass<kSine, false>, copiesPass<kSaw, false>, copiesPass<kPulse, false>, copiesPass<kTri, false>},
                                  {copiesPass<kSine, true>, copiesPass<kSaw, true>, copiesPass<kPulse, true>, copiesPass<kTri, true>}};
using Osc2Fn = void (*)(VoiceState&, const Point*, float*, const float*, float);
const Osc2Fn kOsc2Fn[2][4] = {{osc2Pass<kOsc2Saw, false>, osc2Pass<kOsc2Pulse, false>, osc2Pass<kOsc2Tri, false>, osc2Pass<kOsc2Noise, false>},
                              {osc2Pass<kOsc2Saw, true>, osc2Pass<kOsc2Pulse, true>, osc2Pass<kOsc2Tri, true>, osc2Pass<kOsc2Noise, true>}};
using SubFn = void (*)(VoiceState&, float, bool, float*);
const SubFn kSubFn[4] = {subPass<kSubSine>, subPass<kSubTri>, subPass<kSubSaw>, subPass<kSubSquare>};

// The mix into a (in place); ring: osc 2 x osc 1 (before the sub).
__attribute__((noinline)) void mixPass(float* a, const float* b, const float* c, const SlotState& S, bool sub) {
    constexpr float n128 = 1.0f / float(kBlock);
    const float subScale = S.detune ? (S.ring ? kSubDetuneRing : kSubDetune) : 1.0f;
    float gA = S.c[0].gA, gS = S.c[0].gS * subScale, gB = S.c[0].gB;
    const float dA = (S.c[1].gA - gA) * n128, dS = (S.c[1].gS * subScale - gS) * n128, dB = (S.c[1].gB - gB) * n128;
    const float ring = S.ring ? S.ringGain : 0.0f;
    for (int n = 0; n < int(kBlock); ++n) {
        const float o2 = S.ring ? b[n] * a[n] * ring : b[n];
        a[n] = gA * a[n] + gB * o2 + (sub ? gS * c[n] : 0.0f);
        gA += dA;
        gS += dS;
        gB += dB;
    }
}

// The filter section on x, tick by tick.
template <int kKind>
__attribute__((noinline)) void filterPass(Filter& F, float* x, const Point* P, const SlotState& S) {
    for (int t = 0; t < kTicks; ++t)
        filterTick<kKind>(F, x + t * kTick, P[t].c, P[t + 1].c, P[t].k, P[t + 1].k, S.ftype, S.svfD, S.svfG, S.sat);
}

// The amp and the framework's gain into L / R.
__attribute__((noinline)) void outPass(const float* x, const Point* P, float g0, float gInc, float* L, float* R) {
    float g = g0 * kOut;
    const float dg = gInc * kOut;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float e = P[t].amp;
        const float de = (P[t + 1].amp - e) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            const float o = x[n] * e * g;
            L[n] += o;
            R[n] += o;
            e += de;
            g += dg;
        }
    }
}

// A voice's oscillators restart (Phase > 0): osc 1 at 0, osc 2 at Phase x
// 180 degrees, the copies at their own angles.
void restartPhases(VoiceState& V, const SlotState& S) {
    V.ph1 = 0.0f;
    V.ph2 = S.phase2;
    V.pc[0] = S.copyPhase;
    V.pc[1] = 0.0f;
    V.parity = 0;
    V.init = 1;
}

class OnetypeEngine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "1TYPE"; }

    void slotReset(SynthSlotScratch& sc) override { std::memset(sc.bytes, 0, sizeof(sc.bytes)); }
    void setTempo(float bpm) override { tempo_ = bpm > 1.0f ? bpm : 120.0f; }

    ONETYPE_PER_BLOCK void prepare(SynthSlotScratch& sc, const SynthParams& p) override {
        if (p.count < kParamCount) return;
        SlotState& S = sc.as<SlotState>();
        if (!S.valid || S.serial != p.serial || p.moving) refresh(S, p);
        if (!S.lfoInit) lfoStart(S);
        // The LFOs: their values at the block's 9 tick points.
        for (int k = 0; k < 2; ++k) {
            Lfo& l = S.lfo[k];
            const float* L = p.end + kLfo1Dest + k * kLfoStride;
            const int sync = va::enumOf(L[kLfoSync], kLfoSyncSteps);
            const float hz = sync ? tempo_ * (1.0f / 60.0f) / kSyncBeats[sync - 1] : clampf(L[kLfoRate], 0.08f, 130.0f);
            l.inc = hz * float(kTick) * kInvFs;
            l.v[0] = lfoWave(l);
            for (int j = 1; j <= kTicks; ++j) {
                l.ph += l.inc;
                if (l.ph >= 1.0f) {
                    l.ph -= float(int(l.ph));
                    lfoNext(l);
                }
                l.v[j] = lfoWave(l);
            }
        }
    }

    ONETYPE_PER_BLOCK void noteOn(SynthVoice& v, const SynthParams& p, const SynthNote& n) override {
        if (p.count < kParamCount) return;
        SlotState& S = v.slotScratch->as<SlotState>();
        VoiceState& V = v.as<VoiceState>();
        if (!S.valid || S.serial != p.serial) refresh(S, p);
        if (!S.lfoInit) lfoStart(S);
        const bool held = n.reused && V.started;
        const bool legato = (S.glideMode == kLegato || S.glideMode == kLegatoOverlap) && n.overlap && held;
        // Key sync: a note with no other held restarts the LFO (a new random value too).
        if (!n.overlap && p.held.count <= 1)
            for (Lfo& l : S.lfo)
                if (l.keySync) {
                    l.ph = 0.0f;
                    lfoNext(l);
                    l.v[0] = lfoWave(l);
                }
        if (!held) {
            std::memset(&V, 0, sizeof(V));
            if (!S.phaseOn) {   // free running: arbitrary start phases
                V.ph1 = rnd01(S.seed);
                V.ph2 = rnd01(S.seed);
                V.pc[0] = rnd01(S.seed);
                V.pc[1] = rnd01(S.seed);
            }
            // The first note after a reset plays the source's noise from its seed.
            V.noise = S.notes ? (((S.seed >> 4) & 0xFFFFFu) | 1u) : kNoiseSeed;
            if (S.notes < 255) ++S.notes;
            V.init = 1;
        }
        if (S.phaseOn && (!held || !legato)) restartPhases(V, S);   // a mono retrigger too
        // Glide (the A-Type rules): Always / Legato from the slot's last note,
        // Overlap / Legato Overlap only from a held one; a Glide FX owns the start.
        const bool always = S.glideMode == kAlways || S.glideMode == kLegato;
        const bool glide = S.glideA > 0.0f && (held || S.hasLast) && !n.fxGlide && (always || n.overlap);
        if (!glide) V.pitch = n.pitch;
        else if (!held) V.pitch = S.lastPitch;
        V.target = n.pitch;
        V.aim = kGlideAim * __builtin_fabsf(V.pitch - n.pitch);
        if (!legato || V.aSt == kStIdle) V.aSt = V.fSt = V.mSt = kStAttack;   // from the current levels
        S.lastPitch = n.pitch;
        S.hasLast = 1;
        V.gate = 1;
        V.started = 1;
        // A fresh voice reports full level (no steal victim before its first block).
        v.level = n.reused ? V.amp : 1.0f;
    }

    void noteOff(SynthVoice& v, const SynthParams&) override {
        VoiceState& V = v.as<VoiceState>();
        V.gate = 0;
        if (V.aSt != kStIdle) V.aSt = kStRelease;
        if (V.fSt != kStIdle) V.fSt = kStRelease;
    }

    ONETYPE_PER_BLOCK bool render(SynthVoice& v, const SynthParams& p, float g0, float gInc, float* L, float* R) override {
        VoiceState& V = v.as<VoiceState>();
        if (!V.started || p.count < kParamCount) {
            v.level = 0.0f;
            return false;
        }
        const SlotState& S = v.slotScratch->as<SlotState>();
        Point P[kTicks + 1];
        points(P, V, S, v);
        if (V.init) {   // a fresh or restarted oscillator: its frame states from its phase
            V.init = 0;
            const float w = P[0].w;
            V.hi1 = S.shape1 == kPulse ? pulseHi(V.ph1, w) : triHi(V.ph1);
            V.hi2 = S.shape2 == kOsc2Pulse ? pulseHi(V.ph2, w) : triHi(V.ph2);
            for (int k = 0; k < 2; ++k) V.hic[k] = S.shape1 == kPulse ? pulseHi(V.pc[k], w) : triHi(V.pc[k]);
            const float sp = S.subDown ? 0.5f * V.ph1 : V.ph1;
            V.his = S.subShape == kSubTri ? triHi(sp) : pulseHi(sp, 0.5f);
        }
        float a[kBlock], b[kBlock], c[kBlock];
        const float from = V.ph1;
        const bool sub = S.subOn, fm = S.fmOn, sync = S.sync, ph = sub || sync;
        if (fm && S.shape2 == kOsc2Noise)   // noise as an FM source is +-0.65
            for (Point& q : P) q.fm *= kNoiseFm;
        if (fm && sync) {
            crossPass(V, S, P, a, b, c);
        } else if (fm) {
            kOsc2Fn[0][S.shape2](V, P, b, nullptr, 0.0f);
            kOsc1Fn[ph][1][S.shape1](V, P, a, c, b);
        } else {
            kOsc1Fn[ph][0][S.shape1](V, P, a, c, nullptr);
            kOsc2Fn[sync][S.shape2](V, P, b, c, from);
        }
        if (S.detune) kCopiesFn[fm][S.shape1](V, S, P, a, b);
        if (sub) kSubFn[S.subShape](V, from, S.subDown, c);
        mixPass(a, b, c, S, sub);
        if (S.kind == kKindSvf) filterPass<kKindSvf>(V.F, a, P, S);
        else filterPass<kKindLadder>(V.F, a, P, S);
        outPass(a, P, g0, gInc, L, R);
        v.level = V.amp;
        // The amp envelope ended, or a held note decayed to Sustain 0.
        if (V.aSt == kStIdle || (V.aSt == kStDecay && V.amp == 0.0f && S.amp.sus == 0.0f)) {
            V.started = 0;
            V.aSt = kStIdle;
            V.amp = 0.0f;
            return false;
        }
        return true;
    }

    // [3.x] The LFOs on Pan: the slot bus' balance (the source's law, after the voices).
    bool slotPost(int, SynthSlotScratch& sc, const SynthParams& p, float* L, float* R) override {
        if (p.count < kParamCount) return false;
        const SlotState& S = sc.as<SlotState>();
        const bool pan0 = S.lfo[0].on && S.lfo[0].dest == kToPan, pan1 = S.lfo[1].on && S.lfo[1].dest == kToPan;
        if (!pan0 && !pan1) return false;
        float gl[kTicks + 1], gr[kTicks + 1];
        for (int j = 0; j <= kTicks; ++j) {
            float s = 0.0f;
            if (pan0) s += S.lfo[0].depth * S.lfo[0].v[j];
            if (pan1) s += S.lfo[1].depth * S.lfo[1].v[j];
            const float u = 1.0f - (s < 0.0f ? -s : s), cube = u * u * u, q = s >= 0.0f ? 1.0f - cube : cube - 1.0f;
            gl[j] = q >= 0.0f ? 1.0f + 0.5f * q : 1.0f + q;
            gr[j] = q >= 0.0f ? 1.0f - q : 1.0f - 0.5f * q;
        }
        int n = 0;
        for (int t = 0; t < kTicks; ++t) {
            float l = gl[t], r = gr[t];
            const float dl = (gl[t + 1] - l) * kTickFrac, dr = (gr[t + 1] - r) * kTickFrac;
            for (int i = 0; i < kTick; ++i, ++n) {
                L[n] *= l;
                R[n] *= r;
                l += dl;
                r += dr;
            }
        }
        return false;
    }

private:
    static void lfoStart(SlotState& S) {
        lfoInit(S.lfo[0]);
        lfoInit(S.lfo[1]);
        S.lfoInit = 1;
    }

    // The 9 control points (0 = now, then one tick each: the envelopes and the glide step).
    ONETYPE_PER_BLOCK static void points(Point* P, VoiceState& V, const SlotState& S, const SynthVoice& v) {
        const Cont &c0 = S.c[0], &c1 = S.c[1];
        const float top = S.kind == kKindSvf ? kSvfTop : kLadderTop;
        for (int j = 0; j <= kTicks; ++j) {
            if (j) {
                V.amp = envStep(V.aSt, V.amp, S.amp);
                V.fenv = envStep(V.fSt, V.fenv, S.fenv);
                if (S.mod != 0.0f) V.menv = envStep(V.mSt, V.menv, S.menv);
                if (V.pitch != V.target) {   // the source's glide: r -> r a - E (1 - a), lands
                    const float r = __builtin_fabsf(V.pitch - V.target);
                    const float rest = r * S.glideA - V.aim * (1.0f - S.glideA);
                    V.pitch = rest <= 0.0f || S.glideA <= 0.0f ? V.target : V.target + (V.pitch > V.target ? rest : -rest);
                }
            }
            const float t = float(j) * (1.0f / float(kTicks));
            // The LFOs at this point, per destination (both on one: the source's sums / product).
            float cents12 = 0.0f, cents2 = 0.0f, cut = 0.0f, amp = 1.0f, pw = 0.0f, fmu = 0.0f;
            bool pwOn = false;
            for (int k = 0; k < 2; ++k) {
                const Lfo& l = S.lfo[k];
                if (!l.on) continue;
                const float x = l.v[j];
                switch (l.dest) {
                    case kToOsc2: cents2 += l.cents * x; break;
                    case kToOsc12: cents12 += l.cents * x; break;
                    case kToFilter: cut += l.depth * 0.5f * (x + 1.0f); break;
                    case kToAmp: amp *= 1.0f - l.depth * 0.5f * (1.0f - x); break;
                    case kToPw: pw += l.depth * x, pwOn = true; break;
                    case kToFm: fmu += l.depth * 0.5f * (x + 1.0f); break;
                    default: break;
                }
            }
            const float bend = lerp(v.bendStart, v.bend, t), tune = lerp(c0.tune, c1.tune, t);
            const float base = V.pitch + lerp(c0.shift, c1.shift, t) + tune;
            const float lfoAll = bend + 0.01f * cents12;
            float p2 = (S.track2 ? base : 57.0f + tune) + lerp(c0.osc2, c1.osc2, t) + lfoAll + 0.01f * cents2;
            // The mod envelope: osc 2 pitch (+-60 semitones), PW and FM (room left).
            const float m = S.mod * V.menv;
            if (S.mod != 0.0f && S.modDest == kModOsc2Pitch) p2 += 60.0f * m;
            Point& q = P[j];
            q.incA = incOf(base + lfoAll);
            q.incB = incOf(p2);
            const float width = lerp(c0.width, c1.width, t);
            float w = width;
            if (pwOn) w += pw * (pw > 0.0f ? 1.0f - width : width);
            if (S.mod != 0.0f && S.modDest == kModPw) {
                const float mm = clampf(m, -1.0f, 1.0f);
                w += mm * (mm > 0.0f ? 1.0f - w : w);
            }
            q.w = clampf(w, kWidthMin, kWidthMax);
            q.amp = V.amp * amp;
            q.fm = 0.0f;
            if (S.fmOn) {
                float u = lerp(c0.fm, c1.fm, t);
                u += (1.0f - u) * fmu;
                if (S.mod != 0.0f && S.modDest == kModFm) {
                    // The decay reaches FM in dB: 0 dB -> 1, -80 dB -> 0 (linear in time).
                    const float e = V.menv;
                    const float g = V.mSt == kStAttack ? e : (e > 1.0e-4f ? 1.0f + fastLog2(e) * (1.0f / kLog2Of1e4) : 0.0f);
                    const float mm = clampf(S.mod * g, -1.0f, 1.0f);
                    u += mm * (mm > 0.0f ? 1.0f - u : u);
                }
                u = clampf(u, 0.0f, 1.0f);
                q.fm = u > 0.0f ? (fastExp2(kFmLog2 * u) - 1.0f) * (1.0f / 2000.0f) : 0.0f;
            }
            // The cutoff on the knob axis: key track (around key 48), the
            // Performance offset; the envelope sweeps from 2.1 under it (at 0)
            // to its target at 1, the target clamped (the source's law); an
            // LFO pulls it toward the top.
            const float fb = lerp(c0.cut, c1.cut, t) + lerp(c0.track, c1.track, t) * (V.target - 48.0f) + v.cutoffOct * kKnobPerOct;
            const float low = fb - 2.1f;
            float k = low + (clampf(fb + lerp(c0.env, c1.env, t), -2.0f, top) - low) * V.fenv;
            if (cut > 0.0f) k += (top - k > 0.0f ? top - k : 0.0f) * (cut < 1.0f ? cut : 1.0f);
            if (S.kind == kKindSvf) {
                q.c = svfF(k);
                q.k = 0.0f;
            } else {
                q.c = ladderA(k);
                q.k = ladderK(k, lerp(c0.res, c1.res, t));
            }
        }
    }

    float tempo_ = 120.0f;
};

OnetypeEngine gOnetype;

}  // namespace

ISynthEngine* engine() { return &gOnetype; }

}  // namespace onetype
}  // namespace audio
}  // namespace ot
