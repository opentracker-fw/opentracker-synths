// 1-Type engine (see onetype.h): a single-precision port of the voice of
// Oxo Synth2.
//
// A block is 8 control ticks of 16 frames. At the 9 tick points the voice
// computes its oscillator increments, the pulse width and the amp gain
// (from its envelopes, glide and the slot's LFOs at those points, and the
// parameters ramped from the block's start to its end); every per-frame
// value ramps linearly from one point to the next. Then the passes over
// 128-frame buffers: osc 1 (polyBLEP, one frame late; its phases kept for
// the sub), osc 2, the sub (its phase derived from osc 1's plus a wrap
// parity bit: locked exactly), the mix, and the amp into the slot bus.
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
};

struct SlotState {
    uint32_t serial;
    uint32_t seed;           // free-running start phases, the noise of later voices
    uint8_t valid, hasLast, notes, pad0;
    uint8_t shape1, shape2, subShape, subOn;
    uint8_t subDown, track2, phaseOn, glideMode;
    uint8_t ftype, kind, pad1, pad2;   // filter type, its kind (dsp::FilterKind)
    float phase2;            // osc 2's start phase (Phase > 0)
    float lastPitch;
    float sat;               // the saturation's a
    float svfD, svfG;        // SVF damping and input gain (block end)
    EnvCoef amp, fenv;
    Cont c[2];
};
static_assert(sizeof(SlotState) <= kSynthSlotScratchBytes, "1-Type slot state");

struct VoiceState {
    float ph1, ph2;          // oscillator phases 0..1
    float z1, z2, zs;        // samples held one frame (osc 1, osc 2, sub)
    float amp, fenv;         // amp and filter envelopes
    float pitch;             // the note (semitones)
    uint32_t noise;
    Filter F;
    uint8_t hi1, hi2, his, parity;   // frame states (pulse high / triangle falling); the sub's wrap parity
    uint8_t aSt, fSt, started, gate;
    uint8_t init, pad0, pad1, pad2;
};
static_assert(sizeof(VoiceState) <= kSynthVoiceStateBytes, "1-Type voice state");

// One control point.
struct Point {
    float incA, incB, w, amp;
    float c, k;              // filter: SVF f or LP24 a; LP24 feedback
};

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

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
}

ONETYPE_PER_BLOCK void refresh(SlotState& S, const SynthParams& p) {
    const float* E = p.end;
    S.shape1 = uint8_t(va::enumOf(E[kOsc1Wave], kTri));
    S.shape2 = uint8_t(va::enumOf(E[kOsc2Wave], kOsc2Noise));
    S.subShape = uint8_t(va::enumOf(E[kSubWave], kSubSquare));
    S.subOn = E[kSubLevel] > 0.0f || p.start[kSubLevel] > 0.0f;
    S.subDown = E[kSubOctave] >= 0.5f;
    S.track2 = E[kOsc2Track] >= 0.5f;
    const float ph = clampf(E[kPhase], 0.0f, 1.0f);
    S.phaseOn = ph > 0.0f;
    S.phase2 = 0.5f * clampf((127.0f * ph - 1.0f) * (1.0f / 126.0f), 0.0f, 1.0f);
    S.glideMode = uint8_t(va::enumOf(E[kGlideMode], kLegatoOverlap));
    setEnv(S.amp, E[kAttack], E[kDecay], E[kSustain], E[kRelease]);
    setEnv(S.fenv, E[kFAttack], E[kFDecay], E[kFSustain], E[kFRelease]);
    S.ftype = uint8_t(va::enumOf(E[kFilterType], kLpdl));
    S.kind = kindOf(S.ftype);
    const float r = clampf(E[kResonance], 0.0f, 1.0f);
    S.svfD = 1.0f - r;
    S.svfG = 1.0f - 0.7f * r;
    S.sat = satA(E[kSaturation]);
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

// Osc 1 into a; its phase after each frame into ph (the sub's, when on).
template <int K, bool kPh>
__attribute__((noinline)) void osc1Pass(VoiceState& V, const Point* P, float* a, float* ph) {
    float p = V.ph1, z = V.z1;
    uint8_t hi = V.hi1;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d = P[t].incA, w = P[t].w;
        const float dd = (P[t + 1].incA - d) * kTickFrac, dw = (P[t + 1].w - w) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            p += d;
            const bool wrap = p >= 1.0f;
            if (wrap) p -= 1.0f;
            a[n] = frame<K>(p, d, wrap, w, z, hi);
            if (kPh) ph[n] = p;
            d += dd;
            w += dw;
        }
    }
    V.ph1 = p;
    V.z1 = z;
    V.hi1 = hi;
}

// Osc 2 into b (osc 2's numbering: saw, pulse, tri, noise).
template <int K>
__attribute__((noinline)) void osc2Pass(VoiceState& V, const Point* P, float* b) {
    float p = V.ph2, z = V.z2;
    uint8_t hi = V.hi2;
    uint32_t nz = V.noise;
    int n = 0;
    for (int t = 0; t < kTicks; ++t) {
        float d = P[t].incB, w = P[t].w;
        const float dd = (P[t + 1].incB - d) * kTickFrac, dw = (P[t + 1].w - w) * kTickFrac;
        for (int i = 0; i < kTick; ++i, ++n) {
            p += d;
            const bool wrap = p >= 1.0f;
            if (wrap) p -= 1.0f;
            if (K == kOsc2Saw) b[n] = sawFrame(p, d, wrap, z);
            else if (K == kOsc2Pulse) b[n] = pulseFrame(p, d, wrap, w, z, hi);
            else if (K == kOsc2Tri) b[n] = triFrame(p, d, z, hi);
            else b[n] = noiseStep(nz);
            d += dd;
            w += dw;
        }
    }
    V.ph2 = p;
    V.z2 = z;
    V.hi2 = hi;
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

using Osc1Fn = void (*)(VoiceState&, const Point*, float*, float*);
const Osc1Fn kOsc1Fn[2][4] = {{osc1Pass<kSine, false>, osc1Pass<kSaw, false>, osc1Pass<kPulse, false>, osc1Pass<kTri, false>},
                              {osc1Pass<kSine, true>, osc1Pass<kSaw, true>, osc1Pass<kPulse, true>, osc1Pass<kTri, true>}};
using Osc2Fn = void (*)(VoiceState&, const Point*, float*);
const Osc2Fn kOsc2Fn[4] = {osc2Pass<kOsc2Saw>, osc2Pass<kOsc2Pulse>, osc2Pass<kOsc2Tri>, osc2Pass<kOsc2Noise>};
using SubFn = void (*)(VoiceState&, float, bool, float*);
const SubFn kSubFn[4] = {subPass<kSubSine>, subPass<kSubTri>, subPass<kSubSaw>, subPass<kSubSquare>};

// The mix into a (in place).
__attribute__((noinline)) void mixPass(float* a, const float* b, const float* c, const SlotState& S, bool sub) {
    constexpr float n128 = 1.0f / float(kBlock);
    float gA = S.c[0].gA, gS = S.c[0].gS, gB = S.c[0].gB;
    const float dA = (S.c[1].gA - gA) * n128, dS = (S.c[1].gS - gS) * n128, dB = (S.c[1].gB - gB) * n128;
    for (int n = 0; n < int(kBlock); ++n) {
        a[n] = gA * a[n] + gB * b[n] + (sub ? gS * c[n] : 0.0f);
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

class OnetypeEngine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "1TYPE"; }

    void slotReset(SynthSlotScratch& sc) override { std::memset(sc.bytes, 0, sizeof(sc.bytes)); }

    ONETYPE_PER_BLOCK void prepare(SynthSlotScratch& sc, const SynthParams& p) override {
        if (p.count < kParamCount) return;
        SlotState& S = sc.as<SlotState>();
        if (!S.valid || S.serial != p.serial || p.moving) refresh(S, p);
    }

    ONETYPE_PER_BLOCK void noteOn(SynthVoice& v, const SynthParams& p, const SynthNote& n) override {
        if (p.count < kParamCount) return;
        SlotState& S = v.slotScratch->as<SlotState>();
        VoiceState& V = v.as<VoiceState>();
        if (!S.valid || S.serial != p.serial) refresh(S, p);
        const bool held = n.reused && V.started;
        const bool legato = (S.glideMode == kLegato || S.glideMode == kLegatoOverlap) && n.overlap && held;
        if (!held) {
            std::memset(&V, 0, sizeof(V));
            if (!S.phaseOn) {   // free running: arbitrary start phases
                S.seed = S.seed * 1664525u + 1013904223u;
                V.ph1 = float(S.seed >> 8) * (1.0f / 16777216.0f);
                S.seed = S.seed * 1664525u + 1013904223u;
                V.ph2 = float(S.seed >> 8) * (1.0f / 16777216.0f);
            }
            // The first note after a reset plays the source's noise from its seed.
            V.noise = S.notes ? (((S.seed >> 4) & 0xFFFFFu) | 1u) : kNoiseSeed;
            if (S.notes < 255) ++S.notes;
        }
        if (S.phaseOn && (!held || !legato)) {   // Phase > 0: osc 1 at 0, osc 2 at Phase (a mono retrigger too)
            V.ph1 = 0.0f;
            V.ph2 = S.phase2;
            V.parity = 0;
            V.init = 1;
        }
        if (!held) V.init = 1;
        V.pitch = n.pitch;
        if (!legato || V.aSt == kStIdle) V.aSt = V.fSt = kStAttack;   // from the current levels
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
        // The control points: 0 = now, then one tick each.
        Point P[kTicks + 1];
        for (int j = 0; j <= kTicks; ++j) {
            if (j) {
                V.amp = envStep(V.aSt, V.amp, S.amp);
                V.fenv = envStep(V.fSt, V.fenv, S.fenv);
            }
            const float t = float(j) * (1.0f / float(kTicks));
            const Cont &c0 = S.c[0], &c1 = S.c[1];
            const float bend = lerp(v.bendStart, v.bend, t), tune = lerp(c0.tune, c1.tune, t);
            const float p1 = V.pitch + lerp(c0.shift, c1.shift, t) + tune + bend;
            const float p2 = (S.track2 ? p1 : 57.0f + tune + bend) + lerp(c0.osc2, c1.osc2, t);
            Point& q = P[j];
            q.incA = incOf(p1);
            q.incB = incOf(p2);
            q.w = clampf(lerp(c0.width, c1.width, t), kWidthMin, kWidthMax);
            q.amp = V.amp;
            // The cutoff on the knob axis: key track (around key 48), the
            // Performance offset; the envelope sweeps from 2.1 under it (at 0)
            // to its target at 1, the target clamped (the source's law).
            const float base = lerp(c0.cut, c1.cut, t) + lerp(c0.track, c1.track, t) * (V.pitch - 48.0f) + v.cutoffOct * kKnobPerOct;
            const float top = S.kind == kKindSvf ? kSvfTop : kLadderTop, low = base - 2.1f;
            const float k = low + (clampf(base + lerp(c0.env, c1.env, t), -2.0f, top) - low) * V.fenv;
            if (S.kind == kKindSvf) {
                q.c = svfF(k);
                q.k = 0.0f;
            } else {
                q.c = ladderA(k);
                q.k = ladderK(k, lerp(c0.res, c1.res, t));
            }
        }
        if (V.init) {   // a fresh or restarted oscillator: its frame states from its phase
            V.init = 0;
            V.hi1 = S.shape1 == kPulse ? pulseHi(V.ph1, P[0].w) : triHi(V.ph1);
            V.hi2 = S.shape2 == kOsc2Pulse ? pulseHi(V.ph2, P[0].w) : triHi(V.ph2);
            const float sp = S.subDown ? 0.5f * V.ph1 : V.ph1;
            V.his = S.subShape == kSubTri ? triHi(sp) : pulseHi(sp, 0.5f);
        }
        float a[kBlock], b[kBlock], c[kBlock];
        const float from = V.ph1;
        const bool sub = S.subOn;
        kOsc1Fn[sub][S.shape1](V, P, a, c);
        kOsc2Fn[S.shape2](V, P, b);
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
};

OnetypeEngine gOnetype;

}  // namespace

ISynthEngine* engine() { return &gOnetype; }

}  // namespace onetype
}  // namespace audio
}  // namespace ot
