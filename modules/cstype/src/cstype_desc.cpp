// CS-Type synth descriptor (synth_descriptor.h): stored ranges, Edit Patch
// labels, formats, enum names and sections. Patches/CSTYPE/, "CS-Type" on the
// Synth page (displayName).
#include "cstype.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::cstype;

// Init: a two-layer brass / pad: both layers a saw at 8' (layer II 6 cents
// up, a slower attack, quieter, a darker low-pass), each low-pass opened by
// its envelope (initial level 0, attack level 50 %); the velocity scales the
// attack level (Init -> Bril 30 %), the Touch vibrato ready on After -> VCO.
#define LAYER(lpf, fatt, att, lvl)                                                                                      \
    {1, 0, 3},            /* Feet: 16' 8' 4' 2' */                                                                   \
    {1, 0, 1},            /* Saw: Off / On */                                                                        \
    {0, 0, 1},            /* Pulse: Off / On */                                                                      \
    {0, 0, 1},            /* PW: 50 % .. 5 % */                                                                      \
    {0, 0, 1},            /* PWM (the layer's PWM LFO) */                                                            \
    {4, 0.1f, 20},        /* PWM Speed (Hz) */                                                                       \
    {0, 0, 1},            /* Sine level */                                                                           \
    {0, 0, 1},            /* Noise level */                                                                          \
    {20, 20, 20000},      /* HPF Cutoff (Hz) */                                                                      \
    {0, 0, 1},            /* HPF Res */                                                                              \
    {lpf, 20, 20000},     /* LPF Cutoff (Hz) */                                                                      \
    {0.2f, 0, 1},         /* LPF Res */                                                                              \
    {0, -1, 1},           /* Initial Level (+-4 octaves) */                                                          \
    {0.5f, -1, 1},        /* Attack Level */                                                                         \
    {fatt, 0, 10},        /* F Attack */                                                                             \
    {0.6f, 0, 10},        /* F Decay */                                                                              \
    {0.4f, 0, 10},        /* F Release */                                                                            \
    {att, 0, 10},         /* Attack */                                                                               \
    {0.6f, 0, 10},        /* Decay */                                                                                \
    {0.8f, 0, 1},         /* Sustain */                                                                              \
    {0.4f, 0, 10},        /* Release */                                                                              \
    {lvl, 0, 1}           /* Level */
const PatchParamRange kParams[kParamCount] = {
    LAYER(2000, 0.03f, 0.01f, 0.8f),
    LAYER(1500, 0.1f, 0.1f, 0.6f),
    {0, -1, 1},          // Brilliance (both layers: LPF x 2^(2 b), HPF x 2^b)
    {0, -1, 1},          // Resonance (both layers: +-0.5 on every Res)
    {6, -50, 50},        // II Detune (cents)
    {0, 0, 1},           // RM Depth (wet)
    {100, 0.1f, 2000},   // RM Speed (Hz)
    {0, -1, 1},          // RM Sweep (the start: Speed x 2^(3 x Sweep))
    {1, 0, 10},          // RM Decay (the sweep's time)
    {0, 0, 4},           // Function: Sine / Saw / Ramp / Square / Noise (sub-osc LFO)
    {5, 0.1f, 20},       // Speed (Hz)
    {0, 0, 1},           // -> VCO (+-1 semitone)
    {0, 0, 1},           // -> VCF (+-2 octaves)
    {0, 0, 1},           // -> VCA (tremolo)
    {0, 0, 1},           // Touch (after-touch: a macro target)
    {0.3f, 0, 1},        // Init -> Bril (velocity scales the attack levels)
    {0, 0, 1},           // Init -> Level (velocity to the VCA)
    {0.5f, 0, 1},        // After -> VCO (Touch vibrato)
    {0, 0, 1},           // After -> VCF (Touch +2 octaves)
    {0, 0, 1},           // After -> VCA (Touch up to +6 dB)
    {0, -1, 1},          // Ribbon (pitch: a macro target)
    {2, 0, 24},          // Ribbon Range (semitones)
    {0, 0, 3},           // Glide Mode (names::kGlide): Always
    {0, 0, 3},           // Glide Time
    {1, 0, 2},           // Volume
};
#undef LAYER

const char* const kFeetName[4] = {"16'", "8'", "4'", "2'"};
const char* const kFnName[5] = {"Sine", "Saw", "Ramp", "Square", "Noise"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
#define LAYER_UI                                                                                         \
    {"Feet", E(kFeetName)},      {"Saw", F(kFmtOnOff)},       {"Pulse", F(kFmtOnOff)},                   \
    {"PW", F(kFmtPct)},          {"PWM", F(kFmtPct)},         {"PWM Speed", F(kFmtLfoHz)},               \
    {"Sine", F(kFmtPct)},        {"Noise", F(kFmtPct)},       {"HPF", F(kFmtHz)},                        \
    {"HPF Res", F(kFmtPct)},     {"LPF", F(kFmtHz)},          {"LPF Res", F(kFmtPct)},                   \
    {"Initial Lvl", F(kFmtPct)}, {"Attack Lvl", F(kFmtPct)},  {"F Attack", F(kFmtSec)},                  \
    {"F Decay", F(kFmtSec)},     {"F Release", F(kFmtSec)},   {"Attack", F(kFmtSec)},                    \
    {"Decay", F(kFmtSec)},       {"Sustain", F(kFmtPct)},     {"Release", F(kFmtSec)},                   \
    {"Level", F(kFmtPct)}
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    LAYER_UI,
    LAYER_UI,
    {"Brilliance", F(kFmtPct)},  {"Resonance", F(kFmtPct)},   {"II Detune", F(kFmtCents)},
    {"RM Depth", F(kFmtPct)},    {"RM Speed", F(kFmtLfoHz)},  {"RM Sweep", F(kFmtPct)},
    {"RM Decay", F(kFmtSec)},    {"Function", E(kFnName)},    {"Speed", F(kFmtLfoHz)},
    {"-> VCO", F(kFmtPct)},      {"-> VCF", F(kFmtPct)},      {"-> VCA", F(kFmtPct)},
    {"Touch", F(kFmtPct)},       {"Init->Bril", F(kFmtPct)},  {"Init->Level", F(kFmtPct)},
    {"After->VCO", F(kFmtPct)},  {"After->VCF", F(kFmtPct)},  {"After->VCA", F(kFmtPct)},
    {"Ribbon", F(kFmtPct)},      {"Ribbon Rng", F(kFmtSemi)}, {"Glide Mode", E(names::kGlide)},
    {"Glide Time", F(kFmtSec)},  {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef LAYER_UI
#undef E
#undef F

#define OSC(o) {uint8_t(o + kFeet), uint8_t(o + kSaw), uint8_t(o + kPulse), uint8_t(o + kPw), uint8_t(o + kPwm), \
                uint8_t(o + kPwmSpeed), uint8_t(o + kSine), uint8_t(o + kNoise)}
#define VCF(o) {uint8_t(o + kHpfCutoff), uint8_t(o + kHpfRes), uint8_t(o + kLpfCutoff), uint8_t(o + kLpfRes),   \
                uint8_t(o + kInitLevel), uint8_t(o + kAttackLevel), uint8_t(o + kFAttack), uint8_t(o + kFDecay), \
                uint8_t(o + kFRelease)}
#define VCA(o) {uint8_t(o + kAttack), uint8_t(o + kDecay), uint8_t(o + kSustain), uint8_t(o + kRelease), uint8_t(o + kLevel)}
const uint8_t kOsc1[] = OSC(0);
const uint8_t kVcf1[] = VCF(0);
const uint8_t kVca1[] = VCA(0);
const uint8_t kOsc2[] = OSC(kLayer2);
const uint8_t kVcf2[] = VCF(kLayer2);
const uint8_t kVca2[] = VCA(kLayer2);
#undef OSC
#undef VCF
#undef VCA
const uint8_t kMix[] = {kBrilliance, kResonance, kDetune, kRmDepth, kRmSpeed, kRmSweep, kRmDecay};
const uint8_t kMod[] = {kFunction, kSpeed, kToVco, kToVcf, kToVca, kTouch, kInitBril, kInitLevelVel, kAfterVco, kAfterVcf, kAfterVca};
const uint8_t kCtrl[] = {kRibbon, kRibbonRange, kGlideMode, kGlideTime, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};

const Section kSections[] = {
    OT_SYNTH_SECTION("I.OSC", "Layer I Oscillator", kOsc1),   OT_SYNTH_SECTION("I.VCF", "Layer I Filters", kVcf1),
    OT_SYNTH_SECTION("I.VCA", "Layer I Amplifier", kVca1),    OT_SYNTH_SECTION("II.OSC", "Layer II Oscillator", kOsc2),
    OT_SYNTH_SECTION("II.VCF", "Layer II Filters", kVcf2),    OT_SYNTH_SECTION("II.VCA", "Layer II Amplifier", kVca2),
    OT_SYNTH_SECTION("MIX", "Mix and Ring Mod", kMix),        OT_SYNTH_SECTION("MOD", "Modulation and Touch", kMod),
    OT_SYNTH_SECTION("CTRL", "Controllers", kCtrl),           OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kCstypeDescriptor;
const SynthDescriptor kCstypeDescriptor = {audio::cstype::kEngineId, "CSTYPE", 0, int8_t(audio::cstype::kVolume),
                                          uint8_t(audio::cstype::kParamCount), kParams, kUi, kSections,
                                          uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "CS-Type"};

}  // namespace mod
}  // namespace ot
