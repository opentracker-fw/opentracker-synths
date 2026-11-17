// V-Type engine (see vtype.h; the laws are in vtype_params.h).
//
// Per block a voice (1) evaluates its destinations at the block start from
// the envelope levels and glide pitch it kept, the slot's LFO values at the
// block start and p.start, (2) steps envelopes and glide by one block,
// (3) evaluates them again at the block end from p.end, and (4) renders the
// block as passes over stack buffers, every destination ramping linearly
// from the start to the end value.
//
// Swarm: phases in [0, 1), one compare per saw and frame; a wrap is a step
// of -2 x the saw's level whose two-frame polyBLEP residual goes half into
// the frame before (written one frame back; the block's last frame takes the
// next block's first events) and half into the frame now. The square sub is
// the centre saw's wrap parity: its edges are the centre's wraps, so its step
// joins the centre's. The sine sub, the sync and the ring passes step their
// own copy of the centre phase from the same start and increment ramp.
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
constexpr float kOutTrim = 0.33f;     // voice output level (Init near ACD's Init at the slot gain)

inline float sqrtf_(float x) { return __builtin_sqrtf(x > 0.0f ? x : 0.0f); }

// ---------------------------------------------------------------------------
// State.

struct SlotState {
    uint32_t serial;
    uint8_t valid, hasLast;
    uint8_t count, phase, subWave, glideMode;
    uint8_t pad[2];
    uint32_t rng;
    va::Contour env[3];   // Amp, Env 1, Env 2 (p.end)
    float glideBlocks;    // glide time in blocks (0 = off)
    float lastPitch;      // the slot's previous note (glide source)
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
    float dtc;      // centre phase increment
    float spread;   // Swarm spread (cents)
    float wc, ws;   // centre / side saw levels
    float sub;      // sub level
    float amp;      // amplitude (envelope, trim)
};

inline float incOf(float pitch, float maxDt) {
    return clampf(440.0f * va::kInvFs * fastExp2((clampf(pitch, -48.0f, 150.0f) - 69.0f) * (1.0f / 12.0f)), 1.0e-7f, maxDt);
}

VT_PER_BLOCK void evalDest(const float* P, const SlotState& S, const float* src, float pitch, float bend, Dest& d) {
    const float pc = pitch + P[kKeyShift] + 0.01f * P[kFineTune] + bend;
    d.dtc = incOf(pc, kMaxDtC);
    const float D = clampf(P[kDetune], 0.0f, 1.0f);
    d.spread = spreadCents(D);
    const int pairs = (sawsOf(S.count) - 1) >> 1;
    if (pairs == 0) {
        d.wc = 1.0f;
        d.ws = 0.0f;
    } else {
        const float M = clampf(P[kSwarmMix], 0.0f, 1.0f);
        d.wc = sqrtf_(1.0f - kMixShare * M);
        d.ws = sqrtf_(kMixShare * M / float(2 * pairs));
    }
    d.sub = clampf(P[kSubLevel], 0.0f, 1.0f);
    d.amp = src[kSrcAmp] * kOutTrim;
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

// The sine sub: sine of the centre's half phase (h + parity) / 2.
void subSinePass(float* out, float h, bool par, float dt, float ddt, float s, float ds) {
    float u0 = par ? 0.5f : 0.0f;
    for (uint32_t i = 0; i < kBlock; ++i) {
        h += dt;
        if (h >= 1.0f) {
            h -= 1.0f;
            u0 = 0.5f - u0;
        }
        out[i] += s * va::sinePar(u0 + 0.5f * h);
        dt += ddt;
        s += ds;
    }
}

// ---------------------------------------------------------------------------
// Output.

void outMono(const float* x, float a, float da, float* L, float* R) {
    for (uint32_t i = 0; i < kBlock; ++i) {
        const float y = x[i] * a;
        L[i] += y;
        R[i] += y;
        a += da;
    }
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

    void setTempo(float bpm) override {
        if (bpm > 0.0f) bpm_ = bpm;
    }

    VT_PER_BLOCK void prepare(SynthSlotScratch& sc, const SynthParams& p) override {
        if (p.count < kParamCount) return;
        SlotState& S = sc.as<SlotState>();
        if (!S.valid || S.serial != p.serial) refresh(S, p);
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
            V.glide = __builtin_fabsf(n.pitch - from) / S.glideBlocks;
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
        va::setContour(S.env[0], E[kAmpA], E[kAmpD], E[kAmpS], E[kAmpR]);
        va::setContour(S.env[1], E[kE1A], E[kE1D], E[kE1S], E[kE1R]);
        va::setContour(S.env[2], E[kE2A], E[kE2D], E[kE2S], E[kE2R]);
        const float gt = clampf(E[kGlideTime], 0.0f, 3.0f);
        S.glideBlocks = gt < 0.0005f ? 0.0f : (gt < va::kBlockT ? 1.0f : gt / va::kBlockT);
    }

    float bpm_ = 120.0f;
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
        evalDest(p.start, S, src, V.pitch, v.bendStart, d0);
    }
    for (int e = 0; e < 3; ++e) V.env[e] = va::step(V.st[e], V.env[e], S.env[e]);
    if (V.glide > 0.0f) {
        const float d = V.target - V.pitch;
        if (__builtin_fabsf(d) <= V.glide) {
            V.pitch = V.target;
            V.glide = 0.0f;
        } else {
            V.pitch += d > 0.0f ? V.glide : -V.glide;
        }
    }
    {
        const float src[kSrcCount] = {V.env[0], V.env[1], V.env[2], 0.0f, 0.0f};
        evalDest(p.end, S, src, V.pitch, v.bend, d1);
    }

    // 4. the passes.
    float A[kBlock];
    const float hc = V.ph[0];
    const bool par = V.flags & kSubPar;
    {
        SwarmRamp o;
        o.dt = d0.dtc;
        o.ddt = (d1.dtc - d0.dtc) * kInvBlock;
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
    }
    if (S.subWave == kSubSine && (d0.sub > 0.0f || d1.sub > 0.0f))
        subSinePass(A, hc, par, d0.dtc, (d1.dtc - d0.dtc) * kInvBlock, d0.sub, (d1.sub - d0.sub) * kInvBlock);

    const float ga = d0.amp * g0, gb = d1.amp * (g0 + gInc * float(kBlock));
    outMono(A, ga, (gb - ga) * kInvBlock, L, R);

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
