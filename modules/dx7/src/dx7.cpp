// DX7 6-operator FM engine (see dx7.h) [OT].
//
// Derived from msfa (Music Synthesizer for Android), Copyright 2012-2013
// Google Inc., Licensed under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0 (modules/dx7/msfa.txt).
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// Changed from msfa (dx7note.cc, env.cc, pitchenv.cc, lfo.cc, fm_core.cc,
// fm_op_kernel.cc, patch.cc): merged into one engine for the synth framework,
// compact state, flash tables, float pitch + bend, transpose, key sync,
// amplitude modulation, the algorithm 4 / 6 feedback loops, voice end (dx7.h).
//
// Units (msfa's): phases Q24 (1 << 24 = one cycle); pitches log2(Hz) in Q24;
// envelope levels log2 gain in Q24 ("doublings"); operator outputs Q24 sines
// x linear gain (full output level: 2^25). Envelopes, the pitch envelope, the
// LFO and the pitch run per kN = 64 frames (msfa's N; two per block); gains
// ramp linearly across each 64 frames.
#include "dx7.h"

#include <cmath>
#include <cstring>

#include "dx7_patch.h"
#include "dx7_tables.h"

namespace ot {
namespace audio {
namespace dx7 {

namespace {

constexpr int kLgN = 6;
constexpr int kN = 1 << kLgN;   // frames per control step
constexpr int kSteps = int(kBlock) / kN;
static_assert(int(kBlock) % kN == 0, "block must be a multiple of 64 frames");

// Mono voice output (Q24 x gain) -> float: one carrier at full level peaks at
// 0.25 (typical voices 0.25..1 per note, -9 dB default slot gain on top).
constexpr float kOutScale = 1.0f / float(1 << 27);
// Operator gains below this are skipped (msfa kLevelThresh, ~-90 dB).
constexpr int32_t kLevelThresh = 1120;

// VCED layout: operators 6..1 (21 bytes each), then the voice parameters.
enum : int {
    kOpBytes = 21,
    kOpRate = 0, kOpLevel = 4, kOpBreak = 8, kOpLDepth = 9, kOpRDepth = 10, kOpLCurve = 11,
    kOpRCurve = 12, kOpRateScale = 13, kOpAms = 14, kOpVelSens = 15, kOpOut = 16, kOpMode = 17,
    kOpCoarse = 18, kOpFine = 19, kOpDetune = 20,
    kPitchRate = 126, kPitchLevel = 130, kAlgorithm = 134, kFeedback = 135, kKeySync = 136,
    kLfoSpeed = 137, kLfoDelay = 138, kLfoPmd = 139, kLfoAmd = 140, kLfoSync = 141, kLfoWave = 142,
    kPitchModSens = 143, kTranspose = 144, kName = 145,
};

// Largest valid value of each VCED byte.
constexpr uint8_t kOpMax[kOpBytes] = {99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
                                      3,  3,  7,  3,  7,  99, 1,  31, 99, 14};
constexpr uint8_t kVoiceMax[kVoiceBytes - 6 * kOpBytes] = {
    99, 99, 99, 99, 99, 99, 99, 99, 31, 7, 1, 99, 99, 99, 99, 1, 5, 7, 48,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127};

// ---- msfa tables (dx7note.cc, env.cc, pitchenv.cc) ----------------------------

const int32_t kCoarseMul[32] = {
    -16777216, 0, 16777216, 26591258, 33554432, 38955489, 43368474, 47099600,
    50331648, 53182516, 55732705, 58039632, 60145690, 62083076, 63876816,
    65546747, 67108864, 68576247, 69959732, 71268397, 72509921, 73690858,
    74816848, 75892776, 76922906, 77910978, 78860292, 79773775, 80654032,
    81503396, 82323963, 83117622};

const uint8_t kVelocityData[64] = {
    0, 70, 86, 97, 106, 114, 121, 126, 132, 138, 142, 148, 152, 156, 160, 163,
    166, 170, 173, 174, 178, 181, 184, 186, 189, 190, 194, 196, 198, 200, 202,
    205, 206, 209, 211, 214, 216, 218, 220, 222, 224, 225, 227, 229, 230, 232,
    233, 235, 237, 238, 240, 241, 242, 243, 244, 246, 246, 248, 249, 250, 251,
    252, 253, 254};

const uint8_t kExpScaleData[33] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 14, 16, 19, 23, 27, 33, 39, 47, 56, 66,
    80, 94, 110, 126, 142, 158, 174, 190, 206, 222, 238, 250};

const uint8_t kPitchModSensTab[8] = {0, 10, 20, 33, 55, 92, 153, 255};

const uint8_t kLevelLut[20] = {0, 5, 9, 13, 17, 20, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 42, 43, 45, 46};

const uint8_t kPitchRateTab[100] = {
    1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12,
    12, 13, 13, 14, 14, 15, 16, 16, 17, 18, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 30, 31, 33, 34, 36, 37, 38, 39, 41, 42, 44, 46, 47,
    49, 51, 53, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 79, 82,
    85, 88, 91, 94, 98, 102, 106, 110, 115, 120, 125, 130, 135, 141, 147,
    153, 159, 165, 171, 178, 185, 193, 202, 211, 232, 243, 254, 255};

const int8_t kPitchTab[100] = {
    -128, -116, -104, -95, -85, -76, -68, -61, -56, -52, -49, -46, -43,
    -41, -39, -37, -35, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24,
    -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10,
    -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
    28, 29, 30, 31, 32, 33, 34, 35, 38, 40, 43, 46, 49, 53, 58, 65, 73,
    82, 92, 103, 115, 127};

// Algorithms (fm_core.cc): per operator 6..1, bits 0-1 output bus (0 = the
// voice output, 1/2 = scratch buses), bit 2 add to the bus, bits 4-5 input
// bus, bit 6 feedback in, bit 7 feedback out.
enum : uint8_t { kOutAdd = 1 << 2, kFbIn = 1 << 6, kFbOut = 1 << 7 };
const uint8_t kAlgorithms[32][6] = {
    {0xc1, 0x11, 0x11, 0x14, 0x01, 0x14}, {0x01, 0x11, 0x11, 0x14, 0xc1, 0x14},
    {0xc1, 0x11, 0x14, 0x01, 0x11, 0x14}, {0x41, 0x11, 0x94, 0x01, 0x11, 0x14},
    {0xc1, 0x14, 0x01, 0x14, 0x01, 0x14}, {0x41, 0x94, 0x01, 0x14, 0x01, 0x14},
    {0xc1, 0x11, 0x05, 0x14, 0x01, 0x14}, {0x01, 0x11, 0xc5, 0x14, 0x01, 0x14},
    {0x01, 0x11, 0x05, 0x14, 0xc1, 0x14}, {0x01, 0x05, 0x14, 0xc1, 0x11, 0x14},
    {0xc1, 0x05, 0x14, 0x01, 0x11, 0x14}, {0x01, 0x05, 0x05, 0x14, 0xc1, 0x14},
    {0xc1, 0x05, 0x05, 0x14, 0x01, 0x14}, {0xc1, 0x05, 0x11, 0x14, 0x01, 0x14},
    {0x01, 0x05, 0x11, 0x14, 0xc1, 0x14}, {0xc1, 0x11, 0x02, 0x25, 0x05, 0x14},
    {0x01, 0x11, 0x02, 0x25, 0xc5, 0x14}, {0x01, 0x11, 0x11, 0xc5, 0x05, 0x14},
    {0xc1, 0x14, 0x14, 0x01, 0x11, 0x14}, {0x01, 0x05, 0x14, 0xc1, 0x14, 0x14},
    {0x01, 0x14, 0x14, 0xc1, 0x14, 0x14}, {0xc1, 0x14, 0x14, 0x14, 0x01, 0x14},
    {0xc1, 0x14, 0x14, 0x01, 0x14, 0x04}, {0xc1, 0x14, 0x14, 0x14, 0x04, 0x04},
    {0xc1, 0x14, 0x14, 0x04, 0x04, 0x04}, {0xc1, 0x05, 0x14, 0x01, 0x14, 0x04},
    {0x01, 0x05, 0x14, 0xc1, 0x14, 0x04}, {0x04, 0xc1, 0x11, 0x14, 0x01, 0x14},
    {0xc1, 0x14, 0x01, 0x14, 0x04, 0x04}, {0x04, 0xc1, 0x11, 0x14, 0x04, 0x04},
    {0xc1, 0x14, 0x04, 0x04, 0x04, 0x04}, {0xc4, 0x04, 0x04, 0x04, 0x04, 0x04},
};

// Amplitude modulation (msfa has none). The modulation amount
// x = AMD x (1 - LFO) (0..1) x LFO delay x the operator's AMS scale, on the 0..99
// AMD scale, attenuates the operator by kAmCurve(x) doublings (6 dB):
// exponential in x, near silence at AMS 3 / AMD 99 / LFO peak. Fitted to
// Dexed's tremolo depths (the core's tools/dx7_render compare probes), as data, not
// code: per AMS 1/2 the depth equals AMS 3 at 0.26 / 0.44 of the AMD.
constexpr float kAmsScale[4] = {0.0f, 0.26f, 0.44f, 1.0f};
constexpr float kAmCurve[11] = {0.0f, 0.1f, 0.22f, 0.47f, 0.86f, 1.43f, 2.3f, 3.75f, 6.0f, 9.6f, 12.6f};

// Attenuation (Q24 doublings) for AM amount x in 0..99 (x 10 per table step).
int32_t amAttenuation(float x) {
    float t = x * 0.1f;
    if (t <= 0.0f) return 0;
    if (t >= 10.0f) t = 10.0f - 1e-4f;
    const int i = int(t);
    const float f = t - float(i);
    return int32_t((kAmCurve[i] + (kAmCurve[i + 1] - kAmCurve[i]) * f) * float(1 << 24));
}

// msfa unit constants at our sample rate (PitchEnv::init, Lfo::init).
constexpr int32_t kPitchUnit = int32_t(float(kN) * float(1 << 24) / (21.3f * kSampleRate) + 0.5f);
constexpr uint32_t kLfoUnit = uint32_t(float(kN) * 25190424.0f / kSampleRate + 0.5f);

template <class T>
inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

int scaleOutLevel(int ol) { return ol >= 20 ? 28 + ol : kLevelLut[ol]; }

int scaleVelocity(int velocity, int sens) {
    const int vel = clampT(velocity, 0, 127);
    const int value = int(kVelocityData[vel >> 1]) - 239;
    return ((sens * value + 7) >> 3) * 16;
}

int scaleRate(int note, int sens) {
    const int x = clampT(note / 3 - 7, 0, 31);
    return (sens * x) >> 3;
}

int scaleCurve(int group, int depth, int curve) {
    int scale;
    if (curve == 0 || curve == 3) {
        scale = (group * depth * 329) >> 12;   // linear
    } else {
        const int raw = kExpScaleData[group < 32 ? group : 32];
        scale = (raw * depth * 329) >> 15;   // exponential
    }
    return curve < 2 ? -scale : scale;
}

int scaleLevel(int note, const uint8_t* op) {
    // Key groups of 3 notes from the break point; (|offset| + 1) / 3 puts the
    // group edges one note nearer the break point than msfa's |offset| / 3,
    // matching Dexed's level scaling note for note (compare probes).
    const int offset = note - op[kOpBreak] - 17;
    if (offset >= 0) return scaleCurve((offset + 1) / 3, op[kOpRDepth], op[kOpRCurve]);
    return scaleCurve((1 - offset) / 3, op[kOpLDepth], op[kOpLCurve]);
}

// Ratio-mode detune per step (DET - 7) in cents, at notes 24, 36, .., 96:
// larger at low notes (msfa: a constant ~0.9 c). Measured from Dexed
// (compare probes); linear in DET, interpolated per note, clamped outside.
constexpr float kDetuneCents[7] = {2.455f, 1.983f, 1.556f, 1.197f, 0.906f, 0.677f, 0.501f};

// Log-frequency (Q24 octaves) per detune step at a note.
int32_t detuneStep(int note) {
    float t = (float(note) - 24.0f) * (1.0f / 12.0f);
    t = clampT(t, 0.0f, 5.999f);
    const int i = int(t);
    const float c = kDetuneCents[i] + (kDetuneCents[i + 1] - kDetuneCents[i]) * (t - float(i));
    return int32_t(c * (float(1 << 24) / 1200.0f) + 0.5f);
}

// Operator pitch offset: ratio mode relative to the note (without detune),
// fixed mode absolute.
int32_t opPitch(const uint8_t* op) {
    const int coarse = op[kOpCoarse], fine = op[kOpFine], detune = op[kOpDetune];
    if (op[kOpMode] == 0) {
        int32_t lf = kCoarseMul[coarse & 31];
        // (1 << 24) / ln 2 x ln(1 + fine / 100).
        if (fine) lf += int32_t(24204406.323123f * std::log(1.0f + 0.01f * float(fine)) + 0.5f);
        return lf;   // detune: per note (detuneStep), added in render
    }
    int32_t lf = (4458616 * ((coarse & 3) * 100 + fine)) >> 3;
    return lf + (detune > 7 ? 13457 * (detune - 7) : 0);
}

// Engine controls on top of the voice (kDx7*, dx7_patch.h).
constexpr float kBrightDoublings = 2.0f;   // Brightness +-1: modulators +-12 dB
constexpr int32_t kLevelMax = 16 << 24;    // envelope level cap (exp2Lookup: <= 4x full gain)
constexpr float kRateRange = 50.0f;        // Attack / Release +-1: EG rate -+50 steps (of 0..99)
constexpr float kSpreadCents = 7.0f;       // Detune 1: operators up to +-7 cents apart
// Detune spread pattern of operators 6..1 (alternating, the outer pair widest).
constexpr float kSpreadPattern[6] = {1.0f, -1.0f, 0.6f, -0.6f, 0.3f, -0.3f};

// ---- state ---------------------------------------------------------------------

struct Slot {
    uint8_t v[kVoiceBytes];
    bool loaded;
    uint8_t lfoWave;
    uint8_t lfoRand;
    bool lfoSync;
    bool keyed;                // a note started since the last prepare (LFO sync)
    uint8_t algorithm;
    uint8_t fbShift;           // 16 = no feedback (voice + Feedback control, per block)
    uint8_t pitchModSens;
    uint8_t fixedMask;         // bit op: fixed frequency
    bool fbCarrier;            // the feedback loop ends on a carrier (algorithms 4, 6, 32)
    bool legato;               // Mono control: Legato (else Retrigger)
    int32_t pitchModDepth;     // voice PMD x LFO Depth (per block)
    float amDepth;             // AMD (0..99) x LFO Depth (per block)
    int32_t bright;            // Brightness: modulator level offset (Q24 doublings)
    int8_t attack, release;    // EG rate 1 / rate 4 offsets (rate steps)
    int32_t spread[6];         // Detune control: extra log-frequency per operator (Q24)
    uint32_t lfoPhase, lfoDelta, delayInc, delayInc2;
    int32_t lfo[kSteps];       // LFO value (0..1 Q24) per control step of the block
    int32_t opPitch[6];
    int8_t detune[6];          // ratio mode: DET - 7 (fixed mode: 0, in opPitch)
};

struct Env {
    int32_t level;
    int32_t target;
    int32_t inc;
    int16_t outlevel;          // microsteps (incl. key / velocity scaling)
    uint8_t ix;                // stage 0..3, 4 = done
    bool rising;
    int8_t rateScale;
    uint8_t rate[4];
    uint8_t lvl[4];
};

struct Op {
    uint32_t phase;            // wraps (unsigned: no signed overflow)
    int32_t gain;              // gain at the end of the last control step
    Env env;
};

struct Voice {
    Op op[6];
    int32_t peLevel, peTarget, peInc;
    uint8_t peIx;
    bool peRising;
    bool down;
    bool alive;
    int32_t fb[2];
    uint32_t delayState;
    float pitch;               // note pitch incl. transpose (semitones)
    int32_t detuneStep;        // log-frequency per detune step at this note
};
// Six operators do not fit SynthVoice::state: the voices live here, one per
// pool record (SynthVoice::index), in the module's own memory.
Voice gVoices[kSynthMaxVoices];
Voice& voiceOf(SynthVoice& v) { return gVoices[v.index < kSynthMaxVoices ? v.index : 0]; }

// A control's value this block (its default when the patch list is short).
inline float control(const SynthParams& p, int i, float def) { return i < p.count ? p.end[i] : def; }

// Feedback shift (msfa: 8 - fb). Where the loop's last operator is a carrier
// (algorithms 4, 6, 32), two steps (x 1/4) weaker: matches Dexed's spectra
// for those algorithms (the core's tools/dx7_render compare probes).
uint8_t feedbackShift(int fb, bool carrier) { return uint8_t(fb > 0 ? 8 - fb + (carrier ? 2 : 0) : 16); }

// EG rate with a control offset (rate steps), inside 0..99.
uint8_t offsetRate(int rate, float amount) {
    return uint8_t(clampT(rate - int(std::lround(amount * kRateRange)), 0, 99));
}

void envAdvance(Env& e, int ix) {
    e.ix = uint8_t(ix);
    if (ix >= 4) return;
    int actual = scaleOutLevel(e.lvl[ix]) >> 1;
    actual = (actual << 6) + e.outlevel - 4256;
    if (actual < 16) actual = 16;
    e.target = actual << 16;
    e.rising = e.target > e.level;
    int qrate = ((e.rate[ix] * 41) >> 6) + e.rateScale;
    if (qrate > 63) qrate = 63;
    e.inc = (4 + (qrate & 3)) << (2 + kLgN + (qrate >> 2));
}

int32_t envStep(Env& e, bool down) {
    if (e.ix < 3 || (e.ix < 4 && !down)) {
        if (e.rising) {
            constexpr int32_t kJump = 1716 << 16;
            if (e.level < kJump) e.level = kJump;
            e.level += ((int32_t(17 << 24) - e.level) >> 24) * e.inc;
            if (e.level >= e.target) {
                e.level = e.target;
                envAdvance(e, e.ix + 1);
            }
        } else {
            e.level -= e.inc;
            if (e.level <= e.target) {
                e.level = e.target;
                envAdvance(e, e.ix + 1);
            }
        }
    }
    return e.level;
}

void peAdvance(Voice& x, const Slot& s, int ix) {
    x.peIx = uint8_t(ix);
    if (ix >= 4) return;
    x.peTarget = int32_t(kPitchTab[s.v[kPitchLevel + ix]]) * (1 << 19);
    x.peRising = x.peTarget > x.peLevel;
    x.peInc = kPitchRateTab[s.v[kPitchRate + ix]] * kPitchUnit;
}

int32_t peStep(Voice& x, const Slot& s) {
    if (x.peIx < 3 || (x.peIx < 4 && !x.down)) {
        if (x.peRising) {
            x.peLevel += x.peInc;
            if (x.peLevel >= x.peTarget) {
                x.peLevel = x.peTarget;
                peAdvance(x, s, x.peIx + 1);
            }
        } else {
            x.peLevel -= x.peInc;
            if (x.peLevel <= x.peTarget) {
                x.peLevel = x.peTarget;
                peAdvance(x, s, x.peIx + 1);
            }
        }
    }
    return x.peLevel;
}

// LFO (lfo.cc), one step of kN frames; 0..1 in Q24.
int32_t lfoStep(Slot& s) {
    s.lfoPhase += s.lfoDelta;
    const uint32_t ph = s.lfoPhase;
    switch (s.lfoWave) {
        case 0: {   // triangle
            int32_t x = int32_t(ph >> 7);
            x ^= -int32_t(ph >> 31);
            return x & ((1 << 24) - 1);
        }
        case 1: return int32_t((~ph ^ (1u << 31)) >> 8);   // saw down
        case 2: return int32_t((ph ^ (1u << 31)) >> 8);    // saw up
        case 3: return int32_t(((~ph) >> 7) & (1u << 24)); // square
        case 4: return (1 << 23) + (sinLookup(ph >> 8) >> 1);
        case 5:   // sample and hold
            if (ph < s.lfoDelta) s.lfoRand = uint8_t(s.lfoRand * 179 + 17);
            return ((s.lfoRand ^ 0x80) + 1) << 16;
        default: return 1 << 23;
    }
}

// LFO delay of one voice, one step; 0..1 in Q24.
int32_t delayStep(Voice& x, const Slot& s) {
    const uint32_t delta = x.delayState < (1u << 31) ? s.delayInc : s.delayInc2;
    const uint32_t d = x.delayState + delta;
    if (d < s.delayInc) return 1 << 24;   // wrapped: delay over
    x.delayState = d;
    return d < (1u << 31) ? 0 : int32_t((d >> 7) & ((1u << 24) - 1));
}

// ---- operator kernels (fm_op_kernel.cc) -------------------------------------------

// The per-sample loops run unrolled (loop overhead is ~20 % of a kernel).
#if defined(__GNUC__) && !defined(__clang__)
#define DX7_UNROLL _Pragma("GCC unroll 4")
#else
#define DX7_UNROLL
#endif

// Operator output: Q24 sine x gain >> 24, as the top word of (y << 6) x
// (gain << 2) (one SMMUL). Sines are within +-2^24 and gains below 2^29,
// so neither shifted factor overflows. Kernels keep gains as gain << 2.
inline int32_t opOut(int32_t y, int32_t gain4) { return int32_t((int64_t(y * 64) * gain4) >> 32); }

inline int32_t gainStep4(int32_t g1, int32_t g2) { return ((g2 - g1 + (kN >> 1)) >> kLgN) * 4; }

void opKernel(int32_t* out, const int32_t* in, uint32_t phase, uint32_t freq, int32_t g1, int32_t g2, bool add) {
    const int32_t dgain = gainStep4(g1, g2);
    int32_t gain = g1 * 4;
    if (in) {
        if (add) {
            DX7_UNROLL
            for (int i = 0; i < kN; ++i) {
                gain += dgain;
                out[i] += opOut(sinLookup(phase + uint32_t(in[i])), gain);
                phase += freq;
            }
        } else {
            DX7_UNROLL
            for (int i = 0; i < kN; ++i) {
                gain += dgain;
                out[i] = opOut(sinLookup(phase + uint32_t(in[i])), gain);
                phase += freq;
            }
        }
    } else if (add) {
        DX7_UNROLL
        for (int i = 0; i < kN; ++i) {
            gain += dgain;
            out[i] += opOut(sinLookup(phase), gain);
            phase += freq;
        }
    } else {
        DX7_UNROLL
        for (int i = 0; i < kN; ++i) {
            gain += dgain;
            out[i] = opOut(sinLookup(phase), gain);
            phase += freq;
        }
    }
}

void fbKernel(int32_t* out, uint32_t phase, uint32_t freq, int32_t g1, int32_t g2, int32_t* fb, int shift, bool add) {
    const int32_t dgain = gainStep4(g1, g2);
    int32_t gain = g1 * 4;
    int32_t y0 = fb[0], y = fb[1];
    const int sh = shift + 1;
    if (add) {
        DX7_UNROLL
        for (int i = 0; i < kN; ++i) {
            gain += dgain;
            const int32_t m = (y0 + y) >> sh;
            y0 = y;
            y = opOut(sinLookup(phase + uint32_t(m)), gain);
            out[i] += y;
            phase += freq;
        }
    } else {
        DX7_UNROLL
        for (int i = 0; i < kN; ++i) {
            gain += dgain;
            const int32_t m = (y0 + y) >> sh;
            y0 = y;
            y = opOut(sinLookup(phase + uint32_t(m)), gain);
            out[i] = y;
            phase += freq;
        }
    }
    fb[0] = y0;
    fb[1] = y;
}

// Algorithms 4 and 6 (ours): a feedback loop through a chain of n = 3 / 2
// operators (6 -> 5 -> 4 / 6 -> 5), the last one's output fed back into the
// first, sample by sample. With feedback 0 it is a plain chain. Always the
// first writer of the voice output.
template <int n>
void chainKernel(int32_t* out, const Op* ops, const int32_t* freq, const int32_t* gnew, int32_t* fb, int shift) {
    uint32_t ph[n];
    int32_t g[n], dg[n];
    for (int k = 0; k < n; ++k) {
        ph[k] = ops[k].phase;
        g[k] = ops[k].gain * 4;
        dg[k] = gainStep4(ops[k].gain, gnew[k]);
    }
    int32_t y0 = fb[0], y = fb[1];
    const int sh = shift < 16 ? shift + 1 : 31;
    for (int i = 0; i < kN; ++i) {
        int32_t m = (y0 + y) >> sh;
        for (int k = 0; k < n; ++k) {
            g[k] += dg[k];
            m = opOut(sinLookup(ph[k] + uint32_t(m)), g[k]);
            ph[k] += uint32_t(freq[k]);
        }
        y0 = y;
        y = m;
        out[i] = m;
    }
    fb[0] = y0;
    fb[1] = y;
}

class Dx7Engine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "DX7"; }

    void slotReset(SynthSlotScratch& slot) override { std::memset(&slot.as<Slot>(), 0, sizeof(Slot)); }

    // The voice of the patch (field kPatchFieldDx7Voice), else INIT VOICE.
    static_assert(kVoiceBytes == kDx7VoiceBytes, "VCED size");
    void loadPatch(SynthSlotScratch& slot, const SynthPatch& patch) override {
        uint8_t v[kVoiceBytes];
        patchDx7Voice(patch, v);
        setVoice(slot, v);
    }

    // The continuous controls (block rate): Feedback, LFO Depth, Brightness,
    // Detune. Mono, Attack and Release act at note on / off (from p there:
    // prepare only runs while the slot sounds).
    void prepare(SynthSlotScratch& slot, const SynthParams& p) override {
        Slot& s = slot.as<Slot>();
        const int fb = clampT(int(s.v[kFeedback]) + int(std::lround(control(p, kDx7Feedback, 0.0f))), 0, 7);
        s.fbShift = feedbackShift(fb, s.fbCarrier);
        const float depth = clampT(control(p, kDx7LfoDepth, 1.0f), 0.0f, 2.0f);
        // At most the DX7's full depth (PMD 99 = 255): render's 32-bit
        // product PMD x LFO delay has no room above it.
        s.pitchModDepth = clampT(int32_t(float((s.v[kLfoPmd] * 165) >> 6) * depth), int32_t(0), int32_t(255));
        s.amDepth = float(s.v[kLfoAmd]) * depth;
        s.bright = int32_t(control(p, kDx7Brightness, 0.0f) * kBrightDoublings * float(1 << 24));
        const float spread = control(p, kDx7Detune, 0.0f) * kSpreadCents * (float(1 << 24) / 1200.0f);
        for (int k = 0; k < 6; ++k) s.spread[k] = (s.fixedMask >> k) & 1 ? 0 : int32_t(spread * kSpreadPattern[k]);
        for (int k = 0; k < kSteps; ++k) s.lfo[k] = lfoStep(s);
        s.keyed = false;
    }

    // Polyphony 1 reuses the slot's voice for the next note (n.reused).
    // Mono Retrigger: every note starts like a new voice (envelopes from 0,
    // feedback cleared; gains ramp over one control step). Legato: a note
    // that overlaps the held one only moves the pitch (envelopes, phases and
    // feedback run on); one after a release restarts from where the
    // envelopes are.
    void noteOn(SynthVoice& v, const SynthParams& p, const SynthNote& n) override {
        Slot& s = v.slotScratch->as<Slot>();
        Voice& x = voiceOf(v);
        if (!n.reused) std::memset(&x, 0, sizeof(Voice));   // a fresh voice starts zeroed
        if (!s.loaded) {
            x.alive = false;
            return;
        }
        const int transpose = int(s.v[kTranspose]) - 24;
        const int note = clampT(int(n.note) + transpose, 0, 127);
        const bool legato = control(p, kDx7Mono, 0.0f) >= 0.5f;
        if (legato && x.alive && n.reused && n.overlap && x.down) {
            x.pitch = n.pitch + float(transpose);
            x.detuneStep = detuneStep(note);
            return;
        }
        const int velocity = clampT(int(n.velocity * 127.0f + 0.5f), 0, 127);
        const bool newVoice = !x.alive || !n.reused;
        const bool fresh = newVoice || !legato;   // Retrigger: a reused voice restarts too
        const float attack = control(p, kDx7Attack, 0.0f);
        for (int k = 0; k < 6; ++k) {
            const uint8_t* od = s.v + k * kOpBytes;
            Op& o = x.op[k];
            Env& e = o.env;
            for (int i = 0; i < 4; ++i) {
                e.rate[i] = od[kOpRate + i];
                e.lvl[i] = od[kOpLevel + i];
            }
            if (attack != 0.0f) e.rate[0] = offsetRate(od[kOpRate], attack);
            int ol = scaleOutLevel(od[kOpOut]) + scaleLevel(note, od);
            if (ol > 127) ol = 127;
            ol = ol * 32 + scaleVelocity(velocity, od[kOpVelSens]);
            e.outlevel = int16_t(ol < 0 ? 0 : ol);
            e.rateScale = int8_t(scaleRate(note, od[kOpRateScale]));
            if (newVoice) {
                e.level = 0;
                o.gain = 0;
                o.phase = 0;
            } else if (fresh) {
                // Retrigger on the sounding voice: the envelopes restart from
                // 0 but the gains keep their last value, so each operator
                // ramps down over the next control step instead of clicking.
                // The phase runs on even with key sync (resetting it under a
                // loud operator is the click itself).
                e.level = 0;
            } else if (s.v[kKeySync]) {
                o.phase = 0;
            }
            envAdvance(e, 0);
        }
        if (fresh) x.peLevel = int32_t(kPitchTab[s.v[kPitchLevel + 3]]) * (1 << 19);
        if (newVoice) x.fb[0] = x.fb[1] = 0;
        x.down = true;
        x.alive = true;
        peAdvance(x, s, 0);
        x.delayState = 0;
        x.pitch = n.pitch + float(transpose);
        x.detuneStep = detuneStep(note);
        if (s.lfoSync && !s.keyed) {
            s.lfoPhase = (1u << 31) - 1;
            s.keyed = true;
        }
        v.level = 0.0f;
    }

    void noteOff(SynthVoice& v, const SynthParams& p) override {
        Voice& x = voiceOf(v);
        if (!x.alive || !x.down) return;
        x.down = false;
        const float release = control(p, kDx7Release, 0.0f);
        const uint8_t* vd = v.slotScratch->as<Slot>().v;
        for (int k = 0; k < 6; ++k) {
            Env& e = x.op[k].env;
            e.rate[3] = offsetRate(vd[k * kOpBytes + kOpRate + 3], release);
            envAdvance(e, 3);
        }
        peAdvance(x, v.slotScratch->as<Slot>(), 3);
    }

    bool render(SynthVoice& v, const SynthParams& p, float g0, float gInc, float* L, float* R) override {
        (void)p;
        Voice& x = voiceOf(v);
        if (!x.alive) return false;
        Slot& s = v.slotScratch->as<Slot>();
        const uint8_t* alg = kAlgorithms[s.algorithm];
        int32_t out[kN], bus[2][kN];
        int32_t gnew[6], freq[6];
        float g = g0;
        int32_t peak = 0;
        const float pitchStep = (v.bend - v.bendStart) * (1.0f / float(kSteps));
        for (int st = 0; st < kSteps; ++st) {
            // Pitch: note + bend (ramped per step), pitch envelope, LFO.
            const float pitch = x.pitch + v.bendStart + pitchStep * float(st + 1);
            const int32_t noteLog = int32_t(50857777.0f + pitch * (float(1 << 24) / 12.0f));
            int32_t pitchMod = peStep(x, s);
            const int32_t delay = delayStep(x, s);
            const int32_t lfo = s.lfo[st];
            const uint32_t pmd = uint32_t(s.pitchModDepth) * uint32_t(delay);
            const int32_t sensLfo = int32_t(s.pitchModSens) * (lfo - (1 << 23));
            pitchMod += int32_t((int64_t(pmd) * int64_t(sensLfo)) >> 39);
            // AM follows the inverted LFO (1 - lfo), as Dexed's tremolo does.
            const float am = s.amDepth * float((1 << 24) - lfo) * float(delay) * (1.0f / float(1 << 24)) * (1.0f / float(1 << 24));   // 0..99
            for (int k = 0; k < 6; ++k) {
                Op& o = x.op[k];
                int32_t level = envStep(o.env, x.down);
                const int ams = s.v[k * kOpBytes + kOpAms];
                if (ams) {
                    level -= amAttenuation(am * kAmsScale[ams]);
                    if (level < 0) level = 0;
                }
                gnew[k] = exp2Lookup(level - (14 << 24));
                // Brightness: modulators only, and only while audible (a
                // silent operator stays silent).
                if (s.bright && (alg[k] & 3) && gnew[k] >= kLevelThresh)
                    gnew[k] = exp2Lookup(clampT(level + s.bright, int32_t(0), kLevelMax) - (14 << 24));
                // Fixed-frequency operators ignore the note, the pitch envelope and
                // the LFO (as in Dexed; msfa adds the modulation to them).
                int32_t lf = s.opPitch[k] + ((s.fixedMask >> k) & 1 ? 0 : noteLog + pitchMod + s.detune[k] * x.detuneStep + s.spread[k]);
                freq[k] = freqLookup(clampT<int32_t>(lf, 0, (20 << 24) - 1));
            }
            // Operators 6..1 through the algorithm's buses (fm_core.cc).
            bool has[3] = {false, false, false};
            int k = 0;
            if (alg[0] == (kFbIn | 1)) {   // algorithms 4 / 6: the feedback chain
                const int n = alg[1] & kFbOut ? 2 : 3;
                bool loud = false;
                for (int j = 0; j < n; ++j) loud |= x.op[j].gain >= kLevelThresh || gnew[j] >= kLevelThresh;
                if (loud) {
                    if (n == 3)
                        chainKernel<3>(out, x.op, freq, gnew, x.fb, s.fbShift);
                    else
                        chainKernel<2>(out, x.op, freq, gnew, x.fb, s.fbShift);
                    has[0] = true;
                } else {
                    x.fb[0] = x.fb[1] = 0;
                }
                for (int j = 0; j < n; ++j) {
                    x.op[j].phase += uint32_t(freq[j]) << kLgN;
                    x.op[j].gain = gnew[j];
                }
                k = n;
            }
            for (; k < 6; ++k) {
                const int flags = alg[k];
                Op& o = x.op[k];
                bool add = (flags & kOutAdd) != 0;
                const int inbus = (flags >> 4) & 3;
                const int outbus = flags & 3;
                int32_t* dst = outbus == 0 ? out : bus[outbus - 1];
                if (o.gain >= kLevelThresh || gnew[k] >= kLevelThresh) {
                    if (!has[outbus]) add = false;
                    if (inbus == 0 || !has[inbus]) {
                        if ((flags & 0xc0) == 0xc0 && s.fbShift < 16)
                            fbKernel(dst, o.phase, freq[k], o.gain, gnew[k], x.fb, s.fbShift, add);
                        else
                            opKernel(dst, nullptr, o.phase, freq[k], o.gain, gnew[k], add);
                    } else {
                        opKernel(dst, bus[inbus - 1], o.phase, freq[k], o.gain, gnew[k], add);
                    }
                    has[outbus] = true;
                } else if (!add) {
                    has[outbus] = false;
                }
                o.phase += uint32_t(freq[k]) << kLgN;
                o.gain = gnew[k];
            }
            // Carrier level (voice stealing / end).
            for (int j = 0; j < 6; ++j)
                if ((alg[j] & 3) == 0 && gnew[j] > peak) peak = gnew[j];
            if (has[0]) {
                float* l = L + st * kN;
                float* r = R + st * kN;
                float gs = g * kOutScale;
                const float gsInc = gInc * kOutScale;
                for (int i = 0; i < kN; ++i) {
                    const float y = float(out[i]) * gs;
                    gs += gsInc;
                    l[i] += y;
                    r[i] += y;
                }
            }
            g += gInc * float(kN);
        }
        v.level = clampT(float(peak) * (1.0f / float(1 << 25)), 0.0f, 1.0f);
        if (!x.down && peak < kLevelThresh) {
            // Released and every carrier silent: done once no envelope can
            // rise again (all past their release stage).
            bool done = true;
            for (int j = 0; j < 6; ++j)
                if ((alg[j] & 3) == 0 && x.op[j].env.ix < 4) done = false;
            if (done) {
                x.alive = false;
                return false;
            }
        }
        return true;
    }
};

Dx7Engine gEngine;

}  // namespace

void unpackVoice(const uint8_t b[kPackedBytes], uint8_t v[kVoiceBytes]) {
    for (int op = 0; op < 6; ++op) {
        const uint8_t* s = b + op * 17;
        uint8_t* d = v + op * kOpBytes;
        std::memcpy(d, s, 11);   // rates, levels, break point, depths
        d[kOpLCurve] = s[11] & 3;
        d[kOpRCurve] = (s[11] >> 2) & 3;
        d[kOpRateScale] = s[12] & 7;
        d[kOpDetune] = (s[12] >> 3) & 15;
        d[kOpAms] = s[13] & 3;
        d[kOpVelSens] = (s[13] >> 2) & 7;
        d[kOpOut] = s[14];
        d[kOpMode] = s[15] & 1;
        d[kOpCoarse] = (s[15] >> 1) & 31;
        d[kOpFine] = s[16];
    }
    std::memcpy(v + kPitchRate, b + 102, 9);   // pitch envelope, algorithm
    v[kFeedback] = b[111] & 7;
    v[kKeySync] = (b[111] >> 3) & 1;
    std::memcpy(v + kLfoSpeed, b + 112, 4);   // speed, delay, PMD, AMD
    v[kLfoSync] = b[116] & 1;
    v[kLfoWave] = (b[116] >> 1) & 7;
    v[kPitchModSens] = (b[116] >> 4) & 7;
    std::memcpy(v + kTranspose, b + 117, 11);   // transpose, name
}

int sanitizeVoice(uint8_t v[kVoiceBytes]) {
    int changed = 0;
    for (int i = 0; i < kVoiceBytes; ++i) {
        const uint8_t mx = i < 6 * kOpBytes ? kOpMax[i % kOpBytes] : kVoiceMax[i - 6 * kOpBytes];
        if (v[i] > mx) {
            v[i] = mx;
            ++changed;
        }
    }
    for (int i = kName; i < kName + kNameLen; ++i)
        if (v[i] < 32) {
            v[i] = ' ';
            ++changed;
        }
    return changed;
}

int parseSysex(const uint8_t* d, size_t len, uint8_t out[][kVoiceBytes], int maxVoices, bool* checksumOk) {
    auto checksum = [](const uint8_t* p, size_t n) {
        unsigned sum = 0;
        for (size_t i = 0; i < n; ++i) sum += p[i];
        return uint8_t((128 - (sum & 0x7f)) & 0x7f);
    };
    if (checksumOk) *checksumOk = true;
    if (len < 6 || d[0] != 0xF0 || d[1] != 0x43 || (d[2] & 0xF0) != 0) return 0;
    if (d[3] == 0x00 && d[4] == 0x01 && d[5] == 0x1B && len >= 6 + kVoiceBytes + 2) {
        if (maxVoices < 1) return 0;
        std::memcpy(out[0], d + 6, kVoiceBytes);
        if (checksumOk) *checksumOk = checksum(d + 6, kVoiceBytes) == d[6 + kVoiceBytes];
        sanitizeVoice(out[0]);
        return 1;
    }
    if (d[3] == 0x09 && d[4] == 0x20 && d[5] == 0x00 && len >= 6 + 4096 + 2) {
        if (checksumOk) *checksumOk = checksum(d + 6, 4096) == d[6 + 4096];
        const int n = maxVoices < kBankVoices ? maxVoices : kBankVoices;
        for (int i = 0; i < n; ++i) {
            unpackVoice(d + 6 + i * kPackedBytes, out[i]);
            sanitizeVoice(out[i]);
        }
        return n;
    }
    return 0;
}

int sysexVoiceCount(const uint8_t* d, size_t len) {
    if (len < 6 || d[0] != 0xF0 || d[1] != 0x43 || (d[2] & 0xF0) != 0) return 0;
    if (d[3] == 0x00 && d[4] == 0x01 && d[5] == 0x1B && len >= 6 + kVoiceBytes + 2) return 1;
    if (d[3] == 0x09 && d[4] == 0x20 && d[5] == 0x00 && len >= 6 + 4096 + 2) return kBankVoices;
    return 0;
}

bool sysexVoice(const uint8_t* d, size_t len, int k, uint8_t out[kVoiceBytes]) {
    const int n = sysexVoiceCount(d, len);
    if (k < 0 || k >= n) return false;
    if (n == 1) std::memcpy(out, d + 6, kVoiceBytes);
    else unpackVoice(d + 6 + k * kPackedBytes, out);
    sanitizeVoice(out);
    return true;
}

void setVoice(SynthSlotScratch& slot, const uint8_t v[kVoiceBytes]) {
    static_assert(sizeof(Slot) <= kSynthSlotScratchBytes, "DX7 slot state too large");
    Slot& s = slot.as<Slot>();
    std::memcpy(s.v, v, kVoiceBytes);
    sanitizeVoice(s.v);
    s.algorithm = s.v[kAlgorithm];
    s.fbCarrier = false;
    for (int k = 0; k < 6; ++k)
        if ((kAlgorithms[s.algorithm][k] & kFbOut) && (kAlgorithms[s.algorithm][k] & 3) == 0) s.fbCarrier = true;
    // The voice as stored; prepare() applies the controls every block.
    s.fbShift = feedbackShift(s.v[kFeedback], s.fbCarrier);
    s.pitchModDepth = (s.v[kLfoPmd] * 165) >> 6;
    s.pitchModSens = kPitchModSensTab[s.v[kPitchModSens] & 7];
    s.amDepth = float(s.v[kLfoAmd]);
    s.bright = 0;
    for (int32_t& d : s.spread) d = 0;
    s.fixedMask = 0;
    for (int k = 0; k < 6; ++k) {
        const uint8_t* od = s.v + k * kOpBytes;
        s.opPitch[k] = opPitch(od);
        s.detune[k] = int8_t(od[kOpMode] ? 0 : int(od[kOpDetune]) - 7);
        if (od[kOpMode]) s.fixedMask |= uint8_t(1 << k);
    }
    // LFO (Lfo::reset).
    const int rate = s.v[kLfoSpeed];
    int sr = rate == 0 ? 1 : (165 * rate) >> 6;
    sr *= sr < 160 ? 11 : (11 + ((sr - 160) >> 4));
    s.lfoDelta = kLfoUnit * uint32_t(sr);
    const int a = 99 - s.v[kLfoDelay];
    if (a == 99) {
        s.delayInc = s.delayInc2 = ~0u;
    } else {
        int b = (16 + (a & 15)) << (1 + (a >> 4));
        s.delayInc = kLfoUnit * uint32_t(b);
        b &= 0xff80;
        if (b < 0x80) b = 0x80;
        s.delayInc2 = kLfoUnit * uint32_t(b);
    }
    s.lfoWave = s.v[kLfoWave];
    s.lfoSync = s.v[kLfoSync] != 0;
    s.loaded = true;
}

ISynthEngine* engine() {
    initSinTable();   // normally done by the module's attach(); direct users (tests) too
    return &gEngine;
}

}  // namespace dx7
}  // namespace audio
}  // namespace ot
