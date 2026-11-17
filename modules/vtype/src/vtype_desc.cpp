// V-Type descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/VTYPE/, "V-Type" on the Synth
// page (displayName).
#include "modules/synth_descriptor.h"
#include "vtype.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::vtype;

#define ENV(a, d, s, r) {a, 0, 10}, {d, 0, 10}, {s, 0, 1}, {r, 0, 10}
#define LFO(rate) {rate, 0.05f, 50}, {0, 0, 5}, {0, 0, 15}, {0, 0, 1}
#define MODSLOT {0, 0, 4}, {0, 0, 12}, {0, -1, 1}
// Init: five saws at Detune 0.35, Single routing through filter 1 (LP24 at
// 1.2 kHz, a little resonance, Env 1 and key track at 30 %), no osc 2, no
// noise, no Drive, an organ-like amp envelope.
const PatchParamRange kParams[kParamCount] = {
    {2, 0, 3},            // Count: 1 / 3 / 5 / 7
    {0.35f, 0, 1},        // Detune
    {0.6f, 0, 1},         // Swarm Mix
    {0, 0, 1},            // Phase: Free / Reset
    {0, 0, 1},            // Sub Level
    {0, 0, 1},            // Sub Wave: Square / Sine
    {0, 0, 3},            // Osc 2 Wave
    {0, -36, 36},         // Semi
    {0, -100, 100},       // Fine (cents)
    {0.5f, 0.05f, 0.95f}, // Pulse Width
    {0, 0, 1},            // Sync
    {0, 0, 1},            // Ring
    {-1, -1, 1},          // Balance: -1 Swarm only .. +1 osc 2 only
    {0, 0, 1},            // Noise
    {0, -1, 1},           // Colour
    {0, -24, 24},         // Key Shift
    {0, -100, 100},       // Fine Tune (cents)
    {1, 0, 3},            // F1 Type: LP24
    {1200, 20, 20000},    // F1 Cutoff
    {0.2f, 0, 1},         // F1 Resonance
    {0.3f, -1, 1},        // F1 Env Amt
    {0.3f, 0, 1},         // F1 Key Track
    {3, 0, 3},            // F2 Type: HP12
    {60, 20, 20000},      // F2 Cutoff
    {0, 0, 1},            // F2 Resonance
    {0, -1, 1},           // F2 Env Amt
    {0, 0, 1},            // F2 Key Track
    {0, 0, 3},            // Routing: Single
    {0, -1, 1},           // Flt Balance
    {0, 0, 1},            // Drive
    {0, 0, 2},            // Curve: Soft / Hard / Fold
    {0.7f, 0, 1},         // Split Width
    {0.002f, 0, 10}, {0.4f, 0, 10}, {0.8f, 0, 1}, {0.3f, 0, 10},   // Amp ADSR
    {0, 0, 3},            // Glide Mode (names::kGlide)
    {0, 0, 3},            // Glide Time
    {1, 0, 2},            // Volume
    ENV(0, 0.5f, 0.3f, 0.4f),   // Env 1
    ENV(0, 0.3f, 0, 0.3f),      // Env 2
    LFO(5),               // LFO 1
    LFO(0.3f),            // LFO 2
    MODSLOT, MODSLOT, MODSLOT, MODSLOT,
};
#undef ENV
#undef LFO
#undef MODSLOT

// Extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const char* const kCountName[4] = {"1", "3", "5", "7"};
const char* const kPhaseName[2] = {"Free", "Reset"};
const char* const kSubName[2] = {"Square", "Sine"};
const char* const kWaveName[4] = {"Saw", "Pulse", "Triangle", "Sine"};
const char* const kTypeName[4] = {"LP12", "LP24", "BP12", "HP12"};
const char* const kRouteName[4] = {"Single", "Serial", "Parallel", "Split"};
const char* const kCurveName[3] = {"Soft", "Hard", "Fold"};
const char* const kSyncName[16] = {"Off",  "8/1", "4/1",  "2/1", "1/1",  "1/2D", "1/2",  "1/4D",
                                   "1/2T", "1/4", "1/8D", "1/4T", "1/8", "1/8T", "1/16", "1/32"};
const char* const kDestName[kDstCount] = {"Pitch",    "Osc2 Pitch", "Detune",     "Swarm Mix", "Osc Balance",
                                          "PW",       "Cutoff 1",   "Cutoff 2",   "Cutoff 1+2", "Reso 1+2",
                                          "Drive",    "Flt Balance", "Volume"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
#define ENVUI(p) {p " Attack", F(kFmtSec)}, {p " Decay", F(kFmtSec)}, {p " Sustain", F(kFmtPct)}, {p " Release", F(kFmtSec)}
#define LFOUI(p) {p " Rate", F(kFmtLfoHz)}, {p " Wave", E(names::kWave)}, {p " Sync", E(kSyncName)}, {p " Key Sync", F(kFmtOnOff)}
#define MODUI(k) {"Src " #k, E(names::kModSrc)}, {"Dest " #k, E(kDestName)}, {"Amt " #k, F(kFmtPctMax)}
const ParamUi kUi[kParamCount + 2] = {
    {"Count", E(kCountName)},     {"Detune", F(kFmtPct)},       {"Swarm Mix", F(kFmtPct)},
    {"Phase", E(kPhaseName)},     {"Sub Level", F(kFmtPct)},    {"Sub Wave", E(kSubName)},
    {"Wave", E(kWaveName)},       {"Semi", F(kFmtSemi)},        {"Fine", F(kFmtCents)},
    {"Pulse Width", F(kFmtPct)},  {"Sync", F(kFmtOnOff)},       {"Ring", F(kFmtPct)},
    {"Balance", F(kFmtPctMax)},   {"Noise", F(kFmtPct)},        {"Colour", F(kFmtPctMax)},
    {"Key Shift", F(kFmtSemi)},   {"Fine Tune", F(kFmtCents)},
    {"F1 Type", E(kTypeName)},    {"F1 Cutoff", F(kFmtHz)},     {"F1 Reso", F(kFmtPct)},
    {"F1 Env Amt", F(kFmtPctMax)}, {"F1 Key Track", F(kFmtPct)},
    {"F2 Type", E(kTypeName)},    {"F2 Cutoff", F(kFmtHz)},     {"F2 Reso", F(kFmtPct)},
    {"F2 Env Amt", F(kFmtPctMax)}, {"F2 Key Track", F(kFmtPct)}, {"Routing", E(kRouteName)},
    {"Flt Balance", F(kFmtPctMax)}, {"Drive", F(kFmtPct)},      {"Curve", E(kCurveName)},
    {"Split Width", F(kFmtPct)},
    {"Attack", F(kFmtSec)},       {"Decay", F(kFmtSec)},        {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},      {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)},
    {"Volume", F(kFmtPct)},
    ENVUI("E1"), ENVUI("E2"),
    LFOUI("L1"), LFOUI("L2"),
    MODUI(1), MODUI(2), MODUI(3), MODUI(4),
    {"Width", F(kFmtPct)},        {"Spread", F(kFmtPct)},
};
#undef ENVUI
#undef LFOUI
#undef MODUI
#undef E
#undef F

const uint8_t kSwarm[] = {kCount, kDetune, kSwarmMix, kPhase, kSubLevel, kSubWave};
const uint8_t kOsc2[] = {kO2Wave, kO2Semi, kO2Fine, kO2Pw, kO2Sync, kRing};
const uint8_t kMix[] = {kBalance, kNoise, kColour, kKeyShift, kFineTune};
const uint8_t kFilt1[] = {kF1Type, kF1Cutoff, kF1Reso, kF1Env, kF1Key};
const uint8_t kFilt2[] = {kF2Type, kF2Cutoff, kF2Reso, kF2Env, kF2Key, kRouting, kFltBalance, kDrive, kCurve, kSplitWidth};
const uint8_t kAmp[] = {kAmpA, kAmpD, kAmpS, kAmpR, kGlideMode, kGlideTime, kVolume, uint8_t(kWidthParam), uint8_t(kSpreadParam)};
const uint8_t kEnv[] = {kE1A, kE1D, kE1S, kE1R, kE2A, kE2D, kE2S, kE2R};
const uint8_t kLfo[] = {kL1Rate, kL1Wave, kL1Sync, kL1Key, kL2Rate, kL2Wave, kL2Sync, kL2Key};
const uint8_t kMod[] = {kModBase,      kModBase + 1, kModBase + 2, kModBase + 3, kModBase + 4,  kModBase + 5,
                        kModBase + 6,  kModBase + 7, kModBase + 8, kModBase + 9, kModBase + 10, kModBase + 11};

const Section kSections[] = {
    OT_SYNTH_SECTION("OSC1", "Swarm", kSwarm),        OT_SYNTH_SECTION("OSC2", "Oscillator 2", kOsc2),
    OT_SYNTH_SECTION("MIX", "Mixer", kMix),           OT_SYNTH_SECTION("FILT1", "Filter 1", kFilt1),
    OT_SYNTH_SECTION("FILT2", "Filter 2 / Route", kFilt2), OT_SYNTH_SECTION("AMP", "Amplifier", kAmp),
    OT_SYNTH_SECTION("ENV", "Envelopes", kEnv),       OT_SYNTH_SECTION("LFO", "LFOs", kLfo),
    OT_SYNTH_SECTION("MOD", "Mod Matrix", kMod),      OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kVtypeDescriptor;
const SynthDescriptor kVtypeDescriptor = {kEngineId, "VTYPE", 0, int8_t(kVolume), uint8_t(kParamCount), kParams, kUi, kSections,
                                         uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "V-Type"};

}  // namespace mod
}  // namespace ot
