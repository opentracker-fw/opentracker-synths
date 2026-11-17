// V-Type parameter indices, enum values and the closed-form laws the engine
// and its tests share (modules/vtype). Header only, no state.
#pragma once

#include <cstdint>

namespace ot {
namespace audio {
namespace vtype {

constexpr int32_t kEngineId = 0x8B;

// Stored parameter indices (append only after the first release).
enum Param : int {
    // OSC1 "Swarm"
    kCount = 0, kDetune, kSwarmMix, kPhase, kSubLevel, kSubWave,
    // OSC2
    kO2Wave = 6, kO2Semi, kO2Fine, kO2Pw, kO2Sync, kRing,
    // MIX
    kBalance = 12, kNoise, kColour, kKeyShift, kFineTune,
    // FILT1
    kF1Type = 17, kF1Cutoff, kF1Reso, kF1Env, kF1Key,
    // FILT2 (and the routing)
    kF2Type = 22, kF2Cutoff, kF2Reso, kF2Env, kF2Key, kRouting, kFltBalance, kDrive, kCurve, kSplitWidth,
    // AMP
    kAmpA = 32, kAmpD, kAmpS, kAmpR, kGlideMode, kGlideTime, kVolume,
    // ENV (Env 1: filter, Env 2: matrix only)
    kE1A = 39, kE1D, kE1S, kE1R, kE2A, kE2D, kE2S, kE2R,
    // LFO: per LFO Rate, Wave, Sync, Key Sync
    kL1Rate = 47, kL1Wave, kL1Sync, kL1Key, kL2Rate, kL2Wave, kL2Sync, kL2Key,
    // MOD: slot k at kModBase + 3k: Src, Dest, Amt
    kModBase = 55,
    kParamCount = 67,
};
constexpr int kLfoStride = kL2Rate - kL1Rate;
constexpr int kModSlots = 4;
constexpr int modSrc(int k) { return kModBase + 3 * k; }
constexpr int modDest(int k) { return kModBase + 3 * k + 1; }
constexpr int modAmt(int k) { return kModBase + 3 * k + 2; }
// Extension parameters (patch field 16): the slot's Width and Spread (SynthBank).
constexpr int kWidthParam = kParamCount;
constexpr int kSpreadParam = kParamCount + 1;

enum SubWave : int { kSubSquare = 0, kSubSine };
enum PhaseMode : int { kPhaseFree = 0, kPhaseReset };
enum O2Wave : int { kO2Saw = 0, kO2Pulse, kO2Tri, kO2Sine };
enum FType : int { kLp12 = 0, kLp24, kBp12, kHp12 };
enum Routing : int { kSingle = 0, kSerial, kParallel, kSplit };
enum Curve : int { kSoft = 0, kHard, kFold };
enum GlideMode : int { kAlways = 0, kOverlap, kLegato, kLegatoOverlap };
enum LfoWave : int { kTri = 0, kSine, kRampUp, kRampDown, kSquare, kSampleHold };
// Matrix sources (names::kModSrc) and destinations.
enum ModSrc : int { kSrcAmp = 0, kSrcEnv1, kSrcEnv2, kSrcLfo1, kSrcLfo2, kSrcCount };
enum ModDest : int {
    kDstPitch = 0, kDstO2Pitch, kDstDetune, kDstSwarmMix, kDstOscBalance, kDstPw, kDstCutoff1, kDstCutoff2,
    kDstCutoff12, kDstReso, kDstDrive, kDstFltBalance, kDstVolume, kDstCount
};

// --- laws ------------------------------------------------------------------------

constexpr int kMaxSaws = 7;
// Saws per Count index 0..3.
constexpr int sawsOf(int count) { return 1 + 2 * (count < 0 ? 0 : (count > 3 ? 3 : count)); }
// Swarm spread in cents at Detune d: 60 d (0.2 + 0.8 d).
constexpr float spreadCents(float d) { return 60.0f * d * (0.2f + 0.8f * d); }
// Pair offsets (x the spread) per Count index: saw 1..6 after the centre.
constexpr float kOffsets[4][6] = {
    {0, 0, 0, 0, 0, 0},
    {1.00f, -0.97f, 0, 0, 0, 0},
    {0.52f, -0.55f, 1.00f, -0.97f, 0, 0},
    {0.29f, -0.31f, 0.62f, -0.65f, 1.00f, -0.97f},
};
// Frequency ratio of c cents (2^(c / 1200) to the second order: no exp per saw).
constexpr float ratioOf(float c) { return 1.0f + c * (5.7762e-4f + 1.668e-7f * c); }
constexpr float kMixShare = 0.85f;   // the side saws' power share at Swarm Mix 1
constexpr float kNoiseLevel = 0.35f;
// Noise colour: the one-pole's corner (Hz) at |Colour| 0 .. 1 for the low pass
// (Colour < 0: 8 kHz .. 300 Hz) and the high pass (> 0: 300 Hz .. 8 kHz,
// the white signal minus the low pass).
constexpr float kColourHi = 8000.0f;
constexpr float kColourLo = 300.0f;
constexpr float kColourWet = 10.0f;   // the colour fully in from |Colour| 0.1
constexpr float kEnvOct = 8.0f;      // Env Amt 1 = 8 octaves at full Env 1
constexpr float kModOct = 8.0f;      // matrix Cutoff: 8 octaves
constexpr float kModSemi = 24.0f;    // matrix Pitch: 24 st x Amt |Amt|
constexpr float kModPw = 0.45f;
constexpr float kStage1K = 1.848f;   // LP24's first stage (2 x damping)
// Resonance r -> k (2 x damping): Q 0.5 .. 50, no self-oscillation.
constexpr float resoK(float r) { return 2.0f * (1.0f - r) * (1.0f - r) + 0.02f; }
// The filter input's gain at k: 1 down to k = 1 (Resonance 0.29), then k,
// so the resonant peak (1 / k) stays within 6 dB of the open filter's.
constexpr float resoGain(float k) { return k < 1.0f ? k : 1.0f; }
// Drive d: 2^(4 d) into the curve, 2^(-2.4 d) after it.
constexpr float kDriveIn = 4.0f;
constexpr float kDriveOut = -2.4f;
constexpr float kDriveWet = 20.0f;   // the curve fully in from Drive 0.05

// LFO tempo divisions (Sync 1..15): quarter notes per cycle.
constexpr float kSyncBeats[16] = {0.0f,  32.0f, 16.0f, 8.0f, 4.0f, 3.0f, 2.0f, 1.5f,
                                  4.0f / 3.0f, 1.0f, 0.75f, 2.0f / 3.0f, 0.5f, 1.0f / 3.0f, 0.25f, 0.125f};

}  // namespace vtype
}  // namespace audio
}  // namespace ot
