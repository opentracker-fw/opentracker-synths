// V-Type engine (see vtype.h; the laws are in vtype_params.h).
//
// Per block a voice (1) evaluates its destinations at the block start from
// the envelope levels and glide pitch it kept, the slot's LFO values at the
// block start and p.start, (2) steps envelopes and glide by one block,
// (3) evaluates them again at the block end from p.end, and (4) renders the
// block as passes over stack buffers, every destination ramping linearly
// from the start to the end value. Nothing of the block start is stored: it
// is evaluated again from what the voice and the slot keep.
//
// Swarm: phases in [0, 1), one compare per saw and frame; a wrap is a step
// of -2 x the saw's level whose two-frame polyBLEP residual goes half into
// the frame before (written one frame back; the block's last frame takes the
// next block's first events) and half into the frame now. The square sub is
// the centre saw's wrap parity: its edges are the centre's wraps, so its step
// joins the centre's. The sine sub, the sync and the ring passes step their
// own copy of the centre phase from the same start and increment ramp.
//
// Signal path: A = Swarm + sub, B = osc 2 (+ ring) + noise, each at its
// Balance gain (noise unscaled). Single: A + B -> filter 1 -> Drive. Serial:
// ... -> filter 2, the tap after Drive crossfaded (linear) to filter 2's
// output by Flt Balance. Parallel: A + B through filter 1 -> Drive and through
// filter 2, crossfaded at equal power. Split: A -> filter 1 -> Drive and
// B -> filter 2 at equal power, panned -w / +w (linear) by Split Width.
#include "vtype.h"

#include <cstring>

#include "audio/va_dsp.h"

#pragma GCC diagnostic error "-Wdouble-promotion"

// Per-block (not per-sample) code is built for size.
#if defined(__GNUC__) && !defined(__clang__)
#define VT_PER_BLOCK __attribute__((optimize("Os")))
#else
#define VT_PER_BLOCK
#endif

namespace ot {
namespace audio {
namespace vtype {

namespace {

using va::clampf;
using va::enumOf;
using va::fastExp2;

constexpr float kInvBlock = 1.0f / float(kBlock);
constexpr float kMaxDtC = 0.42f;      // centre increment ceiling: the widest saw (+60 cents) stays under va::kMaxDt
constexpr float kDoneLevel = 1.0e-4f; // -80 dB: the voice is finished
constexpr float kOutTrim = 0.32f;     // voice output level (Init near ACD's Init at the slot gain)

inline float sqrtf_(float x) { return __builtin_sqrtf(x > 0.0f ? x : 0.0f); }
inline float absf(float x) { return __builtin_fabsf(x); }

// ---------------------------------------------------------------------------
// State.

struct SlotState {
    uint32_t serial;
    uint8_t valid, hasLast;
    uint8_t count, phase, subWave, glideMode;
    uint8_t o2Wave, sync, t1, t2, routing, curve;
    uint32_t rng;
    va::Contour env[3];   // Amp, Env 1, Env 2 (p.end)
    float glideBlocks;    // glide time in blocks (0 = off)
    float lastPitch;      // the slot's previous note (glide source)
    // Patch-only values at the block start [0] and end [1].
    float ring[2], noise[2], colour[2], width[2];
};
static_assert(sizeof(SlotState) <= kSynthSlotScratchBytes, "V-Type slot scratch");

enum VoiceFlag : uint8_t { kStarted = 1, kSubPar = 2, kHi2 = 4 };

struct VoiceState {
    float ph[kMaxSaws];   // Swarm phases 0..1 (centre first)
    float p2, z2;         // osc 2 phase and held frame
    float f[8];           // filter states: filter 1 stage 1 s1 s2, stage 2 s1 s2; filter 2 the same
    uint32_t rng;         // noise
    float nz;             // noise colour state
    float env[3];         // Amp, Env 1, Env 2 levels
    float pitch, target, glide;   // glided note pitch, its target, semitones per block
    uint8_t st[3];        // envelope stages
    uint8_t flags;
};
static_assert(sizeof(VoiceState) <= kSynthVoiceStateBytes, "V-Type voice state");

// Destinations at one end of the block.
struct Dest {
    float dtc, dt2;   // centre / osc 2 phase increments
    float spread;     // Swarm spread (cents)
    float wc, ws;     // centre / side saw levels (Balance included)
    float sub;        // sub level (Balance included)
    float g2;         // osc 2 gain (Balance)
    float pw;         // osc 2 pulse width
    float fg[2], fk[2];   // filters: tan(pi fc / fs), 2 x damping
    float drive;
    float fb;         // Flt Balance
    float amp;        // amplitude (envelope, Volume destination, trim)
};

inline float incOf(float pitch, float maxDt) {
    return clampf(440.0f * va::kInvFs * fastExp2((clampf(pitch, -48.0f, 150.0f) - 69.0f) * (1.0f / 12.0f)), 1.0e-7f, maxDt);
}

VT_PER_BLOCK void evalDest(const float* P, const SlotState& S, const float* src, float pitch, float bend, float cutOct, Dest& d) {
    float m[kDstCount] = {};
    const float pc = pitch + P[kKeyShift] + 0.01f * P[kFineTune] + bend + m[kDstPitch];
    d.dtc = incOf(pc, kMaxDtC);
    d.dt2 = incOf(pc + P[kO2Semi] + 0.01f * P[kO2Fine] + m[kDstO2Pitch], va::kMaxDt);
    d.spread = spreadCents(clampf(P[kDetune] + m[kDstDetune], 0.0f, 1.0f));
    const float b = clampf(P[kBalance] + m[kDstOscBalance], -1.0f, 1.0f);
    const float ga = sqrtf_(0.5f - 0.5f * b);
    d.g2 = sqrtf_(0.5f + 0.5f * b);
    const int pairs = (sawsOf(S.count) - 1) >> 1;
    if (pairs == 0) {
        d.wc = ga;
        d.ws = 0.0f;
    } else {
        const float M = clampf(P[kSwarmMix] + m[kDstSwarmMix], 0.0f, 1.0f);
        d.wc = ga * sqrtf_(1.0f - kMixShare * M);
        d.ws = ga * sqrtf_(kMixShare * M / float(2 * pairs));
    }
    d.sub = ga * clampf(P[kSubLevel], 0.0f, 1.0f);
    d.pw = clampf(P[kO2Pw] + m[kDstPw], 0.05f, 0.95f);
    const float e1 = src[kSrcEnv1], kt = (pitch - 60.0f) * (1.0f / 12.0f);
    const float reso = m[kDstReso];
    for (int f = 0; f < 2; ++f) {
        const int base = f == 0 ? kF1Type : kF2Type;
        const float oct = kEnvOct * P[base + 3] * e1 + P[base + 4] * kt + m[kDstCutoff1 + f] + cutOct;
        d.fg[f] = va::svfG(P[base + 1], clampf(oct, -24.0f, 24.0f));
        d.fk[f] = resoK(clampf(P[base + 2] + reso, 0.0f, 1.0f));
    }
    d.drive = clampf(P[kDrive] + m[kDstDrive], 0.0f, 1.0f);
    d.fb = clampf(P[kFltBalance] + m[kDstFltBalance], -1.0f, 1.0f);
    const float vol = 1.0f + m[kDstVolume];
    d.amp = vol > 0.0f ? src[kSrcAmp] * vol * kOutTrim : 0.0f;
}

// ---------------------------------------------------------------------------
// Swarm.

struct SwarmRamp {
    float dt, ddt;          // centre increment and its per-frame step
    float r[kMaxSaws];      // ratios to the centre
    float ir[kMaxSaws];     // 1 / (mean increment x ratio): frames per unit of phase
    float wc, dwc, ws, dws; // levels
    float sq, dsq;          // square sub level
};

template <int kN>
void swarmKernel(float* out, float* phase, uint8_t& flags, const SwarmRamp& o) {
    constexpr float kSides = float(kN - 1);
    float h[kN], r[kN], ir[kN];
    for (int k = 0; k < kN; ++k) {
        h[k] = phase[k];
        r[k] = o.r[k];
        ir[k] = o.ir[k];
    }
    float sgn = (flags & kSubPar) ? -1.0f : 1.0f;   // the square sub's sign
    float dt = o.dt, wc = o.wc, ws = o.ws, sq = o.sq;
    float sink = 0.0f;
    float* prev = &sink;
    for (uint32_t i = 0; i < kBlock; ++i) {
        float y = 0.0f, side = 0.0f;
#pragma GCC unroll 7
        for (int k = 0; k < kN; ++k) {
            h[k] += dt * r[k];
            if (h[k] >= 1.0f) {
                h[k] -= 1.0f;
                const float t = h[k] * ir[k], u = 1.0f - t;
                float a;
                if (k == 0) {
                    a = wc + sq * sgn;   // the saw's step and the sub's together
                    sgn = -sgn;
                } else {
                    a = ws;
                }
                y += a * u * u;
                *prev -= a * t * t;
            }
            if (k > 0) side += h[k];
        }
        y += wc * (h[0] + h[0] - 1.0f) + sq * sgn;
        if (kN > 1) y += ws * (side + side - kSides);
        out[i] = y;
        prev = out + i;
        dt += o.ddt;
        wc += o.dwc;
        ws += o.dws;
        sq += o.dsq;
    }
    // Events of the next frame (the next block's first): their first halves.
    float fix = 0.0f;
    for (int k = 0; k < kN; ++k) {
        if (h[k] + dt * r[k] >= 1.0f) {
            const float t = (h[k] + dt * r[k] - 1.0f) * ir[k];
            fix += (k == 0 ? wc + sq * sgn : ws) * t * t;
        }
        phase[k] = h[k];
    }
    out[kBlock - 1] -= fix;
    flags = uint8_t(sgn < 0.0f ? (flags | kSubPar) : (flags & ~kSubPar));
}

using SwarmFn = void (*)(float*, float*, uint8_t&, const SwarmRamp&);
constexpr SwarmFn kSwarmFns[4] = {swarmKernel<1>, swarmKernel<3>, swarmKernel<5>, swarmKernel<7>};

// The centre saw's phase, stepped again from the block start (the sine
// sub, sync and ring passes).
struct Centre {
    float h, dt, ddt;
};

// The sine sub: sine of the centre's half phase (h + parity) / 2.
void subSinePass(float* out, Centre c, bool par, float s, float ds) {
    float u0 = par ? 0.5f : 0.0f;
    for (uint32_t i = 0; i < kBlock; ++i) {
        c.h += c.dt;
        if (c.h >= 1.0f) {
            c.h -= 1.0f;
            u0 = 0.5f - u0;
        }
        out[i] += s * va::sinePar(u0 + 0.5f * c.h);
        c.dt += c.ddt;
        s += ds;
    }
}

// ---------------------------------------------------------------------------
// Osc 2: saw / pulse / triangle (va::vcoFrame, one frame late), sine.

struct OscRamp {
    float dt, ddt, w, dw, g, dg;
};

template <int kW>
void osc2Pass(float* out, VoiceState& V, OscRamp o) {
    float p = V.p2, z = V.z2;
    bool hi = V.flags & kHi2;
    for (uint32_t i = 0; i < kBlock; ++i) {
        float y;
        if (kW == kO2Saw) y = va::vcoFrame<va::kShSaw>(p, z, hi, o.dt, o.w);
        else if (kW == kO2Pulse) y = va::vcoFrame<va::kShPulse>(p, z, hi, o.dt, o.w);
        else if (kW == kO2Tri) y = va::vcoFrame<va::kShTri>(p, z, hi, o.dt, o.w);
        else {
            p += o.dt;
            if (p >= 1.0f) p -= 1.0f;
            y = va::sinePar(p);
        }
        out[i] = o.g * y;
        o.dt += o.ddt;
        o.w += o.dw;
        o.g += o.dg;
    }
    V.p2 = p;
    V.z2 = z;
    V.flags = uint8_t(hi ? (V.flags | kHi2) : (V.flags & ~kHi2));
}

// The naive triangle / sine at phase p (the synced triangle and sine).
template <int kW>
inline float naive2(float p) {
    return kW == kO2Tri ? 4.0f * absf(p - 0.5f) - 1.0f : va::sinePar(p);
}

// Osc 2 restarted at the centre saw's wraps: saw / pulse through
// va::vcoSync, triangle / sine with the jump's step residual.
template <int kW>
void osc2SyncPass(float* out, VoiceState& V, OscRamp o, Centre c) {
    float p = V.p2, z = V.z2;
    for (uint32_t i = 0; i < kBlock; ++i) {
        c.h += c.dt;
        float x = -1.0f;
        if (c.h >= 1.0f) {
            c.h -= 1.0f;
            x = clampf(c.h / c.dt, 0.0f, 0.999f);
        }
        float y;
        if (kW == kO2Saw) y = va::vcoSync<va::kShSaw>(p, z, o.dt, o.w, x);
        else if (kW == kO2Pulse) y = va::vcoSync<va::kShPulse>(p, z, o.dt, o.w, x);
        else {
            y = z;
            float cur = 0.0f;
            if (x < 0.0f) {
                p += o.dt;
                if (p >= 1.0f) p -= 1.0f;
            } else {
                float q = p + (1.0f - x) * o.dt;
                if (q >= 1.0f) q -= 1.0f;
                va::stepResidual(naive2<kW>(0.0f) - naive2<kW>(q), x, y, cur);
                p = x * o.dt;
            }
            z = cur + naive2<kW>(p);
        }
        out[i] = o.g * y;
        c.dt += c.ddt;
        o.dt += o.ddt;
        o.w += o.dw;
        o.g += o.dg;
    }
    V.p2 = p;
    V.z2 = z;
}

using OscFn = void (*)(float*, VoiceState&, OscRamp);
using SyncFn = void (*)(float*, VoiceState&, OscRamp, Centre);
constexpr OscFn kOscFns[4] = {osc2Pass<kO2Saw>, osc2Pass<kO2Pulse>, osc2Pass<kO2Tri>, osc2Pass<kO2Sine>};
constexpr SyncFn kSyncFns[4] = {osc2SyncPass<kO2Saw>, osc2SyncPass<kO2Pulse>, osc2SyncPass<kO2Tri>, osc2SyncPass<kO2Sine>};

// Ring: B += R x centre saw x B.
void ringPass(float* b, Centre c, float r, float dr) {
    for (uint32_t i = 0; i < kBlock; ++i) {
        c.h += c.dt;
        if (c.h >= 1.0f) c.h -= 1.0f;
        b[i] += r * (c.h + c.h - 1.0f) * b[i];
        c.dt += c.ddt;
        r += dr;
    }
}

// Noise: white (0.35 N), Colour < 0 its one-pole low pass, > 0 white minus it.
void noisePass(float* b, VoiceState& V, int mode, float n, float dn, float a, float da) {
    uint32_t s = V.rng;
    float lp = V.nz;
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float w = va::lcg(s);
        float y = w;
        if (mode != 0) {
            lp += a * (w - lp);
            y = mode < 0 ? lp : w - lp;
            a += da;
        }
        b[i] += n * y;
        n += dn;
    }
    V.rng = s;
    V.nz = va::flush(lp);
}

// ---------------------------------------------------------------------------
// Filters: TPT state-variable stages (Simper's form), coefficients ramping
// from (g0, k0) to (g1, k1) across the block.

struct SvfRamp {
    float a1, d1, a2, d2, a3, d3;
};
inline SvfRamp svfRamp(float g0, float k0, float g1, float k1) {
    const float a10 = 1.0f / (1.0f + g0 * (g0 + k0)), a11 = 1.0f / (1.0f + g1 * (g1 + k1));
    const float a20 = g0 * a10, a21 = g1 * a11;
    const float a30 = g0 * a20, a31 = g1 * a21;
    return SvfRamp{a10, (a11 - a10) * kInvBlock, a20, (a21 - a20) * kInvBlock, a30, (a31 - a30) * kInvBlock};
}

// LP24's first stage (low-pass), in place.
void svfLowPass(float* x, float* s, SvfRamp c) {
    float s1 = s[0], s2 = s[1];
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float v3 = x[i] - s2;
        const float v1 = c.a1 * s1 + c.a2 * v3;
        const float v2 = s2 + c.a2 * s1 + c.a3 * v3;
        s1 = v1 + v1 - s1;
        s2 = v2 + v2 - s2;
        x[i] = v2;
        c.a1 += c.d1;
        c.a2 += c.d2;
        c.a3 += c.d3;
    }
    s[0] = va::flush(s1);
    s[1] = va::flush(s2);
}

// The resonant stage, any type: y = m0 x + m1 band + m2 low (high-pass:
// x - k band - low, m1 = -k ramping), in place.
void svfStage(float* x, float* s, SvfRamp c, float m0, float m1, float dm1, float m2) {
    float s1 = s[0], s2 = s[1];
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float in = x[i], v3 = in - s2;
        const float v1 = c.a1 * s1 + c.a2 * v3;
        const float v2 = s2 + c.a2 * s1 + c.a3 * v3;
        s1 = v1 + v1 - s1;
        s2 = v2 + v2 - s2;
        x[i] = m0 * in + m1 * v1 + m2 * v2;
        m1 += dm1;
        c.a1 += c.d1;
        c.a2 += c.d2;
        c.a3 += c.d3;
    }
    s[0] = va::flush(s1);
    s[1] = va::flush(s2);
}

void filterPass(float* x, float* s, int type, float g0, float k0, float g1, float k1) {
    if (type == kLp24) svfLowPass(x, s, svfRamp(g0, kStage1K, g1, kStage1K));
    float m0 = 0.0f, m1 = 0.0f, dm1 = 0.0f, m2 = 1.0f;
    if (type == kBp12) {
        m1 = 1.0f;
        m2 = 0.0f;
    } else if (type == kHp12) {
        m0 = 1.0f;
        m1 = -k0;
        dm1 = (k0 - k1) * kInvBlock;
        m2 = -1.0f;
    }
    svfStage(x, s + 2, svfRamp(g0, k0, g1, k1), m0, m1, dm1, m2);
}

// ---------------------------------------------------------------------------
// Drive: 2^(4 d) into the curve, 2^(-2.4 d) after it.

inline float fold(float x) {
    const float t0 = (clampf(x, -250.0f, 250.0f) + 1.0f) * 0.25f + 64.0f;
    const float t = t0 - float(int(t0));
    return 1.0f - 4.0f * absf(t - 0.5f);
}

template <int kC>
void drivePass(float* x, float d0, float d1) {
    float gi = fastExp2(kDriveIn * d0), go = fastExp2(kDriveOut * d0);
    const float dgi = (fastExp2(kDriveIn * d1) - gi) * kInvBlock, dgo = (fastExp2(kDriveOut * d1) - go) * kInvBlock;
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float u = gi * x[i];
        const float y = kC == kSoft ? va::tanhR(u) : (kC == kHard ? clampf(u, -1.0f, 1.0f) : fold(u));
        x[i] = go * y;
        gi += dgi;
        go += dgo;
    }
}
using DriveFn = void (*)(float*, float, float);
constexpr DriveFn kDriveFns[3] = {drivePass<kSoft>, drivePass<kHard>, drivePass<kFold>};

// ---------------------------------------------------------------------------
// Output: amplitude a (ramped) into L and R.

void outMono(const float* x, float a, float da, float* L, float* R) {
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float y = x[i] * a;
        L[i] += y;
        R[i] += y;
        a += da;
    }
}

struct MixRamp {
    float wa, dwa, wb, dwb, a, da;
};

void outMix(const float* x, const float* y, MixRamp m, float* L, float* R) {
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float o = (m.wa * x[i] + m.wb * y[i]) * m.a;
        L[i] += o;
        R[i] += o;
        m.wa += m.dwa;
        m.wb += m.dwb;
        m.a += m.da;
    }
}

// Split: x (filter 1) at pan -w, y (filter 2, may be absent) at +w.
void outSplit(const float* x, const float* y, MixRamp m, float w, float dw, float* L, float* R) {
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float hp = 0.5f + 0.5f * w, hm = 0.5f - 0.5f * w;
        const float xa = m.wa * m.a * x[i], yb = y ? m.wb * m.a * y[i] : 0.0f;
        L[i] += xa * hp + yb * hm;
        R[i] += xa * hm + yb * hp;
        m.wa += m.dwa;
        m.wb += m.dwb;
        m.a += m.da;
        w += dw;
    }
}

// ---------------------------------------------------------------------------
// The colour one-pole's coefficient and mode (-1 low pass, +1 high pass, 0 white).
inline float colourCoef(float c, int& mode) {
    mode = c < 0.0f ? -1 : (c > 0.0f ? 1 : 0);
    const float a = absf(c);
    const float oct = c < 0.0f ? -a * 4.7369655f : a * 4.7369655f;   // log2(8000 / 300)
    const float hz = (c < 0.0f ? kColourHi : kColourLo) * fastExp2(oct);
    return va::onePole(hz);
}

// ---------------------------------------------------------------------------

class VtypeEngine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "VTYPE"; }

    void slotReset(SynthSlotScratch& sc) override {
        std::memset(sc.bytes, 0, sizeof(sc.bytes));
        sc.as<SlotState>().rng = 0x6C8E9CF5u;
    }

    VT_PER_BLOCK void prepare(SynthSlotScratch& sc, const SynthParams& p) override {
        if (p.count < kParamCount) return;
        SlotState& S = sc.as<SlotState>();
        if (!S.valid || S.serial != p.serial) refresh(S, p);
        for (int e = 0; e < 2; ++e) {
            const float* P = e ? p.end : p.start;
            S.ring[e] = clampf(P[kRing], 0.0f, 1.0f);
            S.noise[e] = kNoiseLevel * clampf(P[kNoise], 0.0f, 1.0f);
            S.colour[e] = clampf(P[kColour], -1.0f, 1.0f);
            S.width[e] = p.count > kSplitWidth ? clampf(P[kSplitWidth], 0.0f, 1.0f) : 0.0f;
        }
    }

    VT_PER_BLOCK void noteOn(SynthVoice& v, const SynthParams& p, const SynthNote& n) override {
        if (p.count < kParamCount) return;
        SlotState& S = v.slotScratch->as<SlotState>();
        VoiceState& V = v.as<VoiceState>();
        if (!S.valid || S.serial != p.serial) refresh(S, p);
        const bool fresh = !(V.flags & kStarted);
        if (fresh) {
            va::lcg(S.rng);
            V.rng = (S.rng ^ 0x9E3779B9u) | 1u;
            V.flags |= kStarted;
        }
        // Glide (Always / Legato: from the slot's previous note; Overlap
        // modes: only from a held note). A Glide FX owns the start pitch.
        const int mode = S.glideMode;
        bool hasFrom = false;
        float from = n.pitch;
        if (n.reused) {
            hasFrom = true;
            from = V.pitch;
        } else if (S.hasLast) {
            hasFrom = true;
            from = S.lastPitch;
        }
        V.target = n.pitch;
        const bool always = mode == kAlways || mode == kLegato;
        if (S.glideBlocks > 0.0f && hasFrom && !n.fxGlide && (always || n.overlap) && from != n.pitch) {
            V.pitch = from;
            V.glide = absf(n.pitch - from) / S.glideBlocks;
        } else {
            V.pitch = n.pitch;
            V.glide = 0.0f;
        }
        const bool legato = (mode == kLegato || mode == kLegatoOverlap) && n.overlap;
        if (!legato || V.st[0] == va::kIdle) {
            for (uint8_t& s : V.st) s = va::kAtt;
            if (S.phase == kPhaseReset) {
                for (int k = 0; k < kMaxSaws; ++k) V.ph[k] = float(k) * (1.0f / float(kMaxSaws));
                V.flags = uint8_t(V.flags & ~(kSubPar | kHi2));
                V.p2 = V.z2 = 0.0f;
            } else if (fresh || V.env[0] < 1.0e-3f) {   // a sounding voice keeps running (no click)
                for (float& ph : V.ph) ph = 0.5f + 0.5f * va::lcg(S.rng);
                for (float& ph : V.ph)
                    if (ph >= 1.0f) ph = 0.0f;
                V.p2 = 0.5f + 0.5f * va::lcg(S.rng);
                if (V.p2 >= 1.0f) V.p2 = 0.0f;
            }
        }
        S.lastPitch = n.pitch;
        S.hasLast = 1;
        v.level = V.env[0];
    }

    void noteOff(SynthVoice& v, const SynthParams& p) override {
        (void)p;
        VoiceState& V = v.as<VoiceState>();
        for (uint8_t& s : V.st)
            if (s != va::kIdle) s = va::kRel;
    }

    bool render(SynthVoice& v, const SynthParams& p, float g0, float gInc, float* L, float* R) override;

private:
    static VT_PER_BLOCK void refresh(SlotState& S, const SynthParams& p) {
        const float* E = p.end;
        S.serial = p.serial;
        S.valid = 1;
        S.count = uint8_t(enumOf(E[kCount], 3));
        S.phase = uint8_t(enumOf(E[kPhase], 1));
        S.subWave = uint8_t(enumOf(E[kSubWave], 1));
        S.glideMode = uint8_t(enumOf(E[kGlideMode], 3));
        S.o2Wave = uint8_t(enumOf(E[kO2Wave], 3));
        S.sync = E[kO2Sync] >= 0.5f;
        S.t1 = uint8_t(enumOf(E[kF1Type], 3));
        S.t2 = uint8_t(enumOf(E[kF2Type], 3));
        S.routing = uint8_t(enumOf(E[kRouting], 3));
        S.curve = uint8_t(enumOf(E[kCurve], 2));
        va::setContour(S.env[0], E[kAmpA], E[kAmpD], E[kAmpS], E[kAmpR]);
        va::setContour(S.env[1], E[kE1A], E[kE1D], E[kE1S], E[kE1R]);
        va::setContour(S.env[2], E[kE2A], E[kE2D], E[kE2S], E[kE2R]);
        const float gt = clampf(E[kGlideTime], 0.0f, 3.0f);
        S.glideBlocks = gt < 0.0005f ? 0.0f : (gt < va::kBlockT ? 1.0f : gt / va::kBlockT);
    }
};

VT_PER_BLOCK bool VtypeEngine::render(SynthVoice& v, const SynthParams& p, float g0, float gInc, float* L, float* R) {
    VoiceState& V = v.as<VoiceState>();
    if (V.st[0] == va::kIdle || p.count < kParamCount) {
        v.level = 0.0f;
        return false;
    }
    const SlotState& S = v.slotScratch->as<SlotState>();

    // 1. destinations at the block start, 2. one block of envelopes and glide, 3. at the end.
    Dest d0, d1;
    {
        const float src[kSrcCount] = {V.env[0], V.env[1], V.env[2], 0.0f, 0.0f};
        evalDest(p.start, S, src, V.pitch, v.bendStart, v.cutoffOct, d0);
    }
    for (int e = 0; e < 3; ++e) V.env[e] = va::step(V.st[e], V.env[e], S.env[e]);
    if (V.glide > 0.0f) {
        const float d = V.target - V.pitch;
        if (absf(d) <= V.glide) {
            V.pitch = V.target;
            V.glide = 0.0f;
        } else {
            V.pitch += d > 0.0f ? V.glide : -V.glide;
        }
    }
    {
        const float src[kSrcCount] = {V.env[0], V.env[1], V.env[2], 0.0f, 0.0f};
        evalDest(p.end, S, src, V.pitch, v.bend, v.cutoffOct, d1);
    }

    // 4. the passes: A = Swarm + sub, B = osc 2 + ring + noise.
    float A[kBlock], B[kBlock], T[kBlock];
    const Centre centre{V.ph[0], d0.dtc, (d1.dtc - d0.dtc) * kInvBlock};
    const bool par = V.flags & kSubPar;
    const bool hasA = d0.wc > 0.0f || d1.wc > 0.0f;
    if (hasA) {
        SwarmRamp o;
        o.dt = d0.dtc;
        o.ddt = centre.ddt;
        const float idt = 2.0f / (d0.dtc + d1.dtc);
        const int c = S.count;
        o.r[0] = 1.0f;
        o.ir[0] = idt;
        for (int k = 1; k < sawsOf(c); ++k) {
            o.r[k] = ratioOf(d1.spread * kOffsets[c][k - 1]);
            o.ir[k] = idt / o.r[k];
        }
        o.wc = d0.wc;
        o.dwc = (d1.wc - d0.wc) * kInvBlock;
        o.ws = d0.ws;
        o.dws = (d1.ws - d0.ws) * kInvBlock;
        const bool square = S.subWave == kSubSquare;
        o.sq = square ? d0.sub : 0.0f;
        o.dsq = square ? (d1.sub - d0.sub) * kInvBlock : 0.0f;
        kSwarmFns[c](A, V.ph, V.flags, o);
        if (!square && (d0.sub > 0.0f || d1.sub > 0.0f)) subSinePass(A, centre, par, d0.sub, (d1.sub - d0.sub) * kInvBlock);
    } else {
        // Silent Swarm: the centre still runs as the sync and ring passes step it.
        Centre c = centre;
        for (uint32_t i = 0; i < kBlock; ++i) {
            c.h += c.dt;
            if (c.h >= 1.0f) {
                c.h -= 1.0f;
                V.flags ^= kSubPar;
            }
            c.dt += c.ddt;
        }
        V.ph[0] = c.h;
        std::memset(A, 0, sizeof A);
    }
    bool hasB = d0.g2 > 0.0f || d1.g2 > 0.0f;
    if (hasB) {
        const OscRamp o{d0.dt2, (d1.dt2 - d0.dt2) * kInvBlock, d0.pw, (d1.pw - d0.pw) * kInvBlock, d0.g2, (d1.g2 - d0.g2) * kInvBlock};
        if (S.sync) kSyncFns[S.o2Wave](B, V, o, centre);
        else kOscFns[S.o2Wave](B, V, o);
        if (S.ring[0] > 0.0f || S.ring[1] > 0.0f) ringPass(B, centre, S.ring[0], (S.ring[1] - S.ring[0]) * kInvBlock);
    }
    if (S.noise[0] > 0.0f || S.noise[1] > 0.0f) {
        if (!hasB) std::memset(B, 0, sizeof B);
        hasB = true;
        int m0, m1;
        const float a0 = colourCoef(S.colour[0], m0), a1 = colourCoef(S.colour[1], m1);
        noisePass(B, V, m1, S.noise[0], (S.noise[1] - S.noise[0]) * kInvBlock, a0, (a1 - a0) * kInvBlock);
    }

    // 5. routing and output.
    const float ga = d0.amp * g0, gb = d1.amp * (g0 + gInc * float(kBlock));
    MixRamp m{0.0f, 0.0f, 0.0f, 0.0f, ga, (gb - ga) * kInvBlock};
    const bool drive = d0.drive > 0.0f || d1.drive > 0.0f;
    float* f1 = V.f;
    float* f2 = V.f + 4;
    const int route = S.routing;
    if (route != kSplit && hasB)
        for (uint32_t i = 0; i < kBlock; ++i) A[i] += B[i];
    // Crossfade weights at both block ends.
    float wa0, wb0, wa1, wb1;
    if (route == kSerial) {
        wa0 = 0.5f - 0.5f * d0.fb;
        wb0 = 0.5f + 0.5f * d0.fb;
        wa1 = 0.5f - 0.5f * d1.fb;
        wb1 = 0.5f + 0.5f * d1.fb;
    } else {
        wa0 = sqrtf_(0.5f - 0.5f * d0.fb);
        wb0 = sqrtf_(0.5f + 0.5f * d0.fb);
        wa1 = sqrtf_(0.5f - 0.5f * d1.fb);
        wb1 = sqrtf_(0.5f + 0.5f * d1.fb);
    }
    m.wa = wa0;
    m.dwa = (wa1 - wa0) * kInvBlock;
    m.wb = wb0;
    m.dwb = (wb1 - wb0) * kInvBlock;
    const bool useA = wa0 > 0.0f || wa1 > 0.0f, useB = wb0 > 0.0f || wb1 > 0.0f;
    switch (route) {
        case kSingle:
            filterPass(A, f1, S.t1, d0.fg[0], d0.fk[0], d1.fg[0], d1.fk[0]);
            if (drive) kDriveFns[S.curve](A, d0.drive, d1.drive);
            outMono(A, m.a, m.da, L, R);
            break;
        case kSerial:
            filterPass(A, f1, S.t1, d0.fg[0], d0.fk[0], d1.fg[0], d1.fk[0]);
            if (drive) kDriveFns[S.curve](A, d0.drive, d1.drive);
            if (useA) std::memcpy(T, A, sizeof T);
            filterPass(A, f2, S.t2, d0.fg[1], d0.fk[1], d1.fg[1], d1.fk[1]);
            if (useA) outMix(T, A, m, L, R);
            else outMono(A, m.a, m.da, L, R);
            break;
        case kParallel:
            if (useB) std::memcpy(T, A, sizeof T);
            else std::memset(T, 0, sizeof T);
            if (useA) {
                filterPass(A, f1, S.t1, d0.fg[0], d0.fk[0], d1.fg[0], d1.fk[0]);
                if (drive) kDriveFns[S.curve](A, d0.drive, d1.drive);
            }
            if (useB) filterPass(T, f2, S.t2, d0.fg[1], d0.fk[1], d1.fg[1], d1.fk[1]);
            outMix(A, T, m, L, R);
            break;
        default:   // Split
            filterPass(A, f1, S.t1, d0.fg[0], d0.fk[0], d1.fg[0], d1.fk[0]);
            if (drive) kDriveFns[S.curve](A, d0.drive, d1.drive);
            if (hasB) filterPass(B, f2, S.t2, d0.fg[1], d0.fk[1], d1.fg[1], d1.fk[1]);
            outSplit(A, hasB ? B : nullptr, m, S.width[0], (S.width[1] - S.width[0]) * kInvBlock, L, R);
            break;
    }

    v.level = V.env[0];
    if (V.env[0] < kDoneLevel && (V.st[0] == va::kRel || V.st[0] == va::kIdle || (V.st[0] == va::kDec && S.env[0].sustain <= 0.0f))) {
        V.st[0] = va::kIdle;
        return false;
    }
    return true;
}

VtypeEngine gVtype;

}  // namespace

ISynthEngine* engine() { return &gVtype; }

}  // namespace vtype
}  // namespace audio
}  // namespace ot
