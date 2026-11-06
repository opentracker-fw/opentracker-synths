// A-Type synth descriptor (synth_descriptor.h): stored ranges, Edit Patch
// labels, formats, enum names and sections. Patches/ATYPE/, "A-Type" on the
// Synth page (displayName); the Poly 1 note priority of the framework
// (kSynthFlagNotePriority) with Priority (index 42) choosing Last / Low / High.
#include "atype.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::atype;

// Init: two saws at 8' (VCO 2 7 cents up) through the half-open 4-pole with
// a positive envelope, ADSR on the VCA, Keyboard Mono, Priority Last, glide
// at 0 (mode Always: every note of a tracker line restarts the envelopes).
const PatchParamRange kParams[kParamCount] = {
    {2, 0, 3},          // VCO1 Range: 32' 16' 8' 4'
    {0, -100, 100},     // VCO1 Fine (cents)
    {0, 0, 1},          // VCO1 Wave: Saw / Pulse
    {0, 0, 1},          // VCO1 PW: 50 % .. 5 %
    {0, 0, 1},          // VCO1 PWM (LFO)
    {0, 0, 1},          // VCO1 LFO Mod (vibrato)
    {2, 0, 3},          // VCO2 Range
    {0, -24, 24},       // VCO2 Tune (semitones)
    {7, -100, 100},     // VCO2 Fine (cents)
    {0, 0, 1},          // VCO2 Wave
    {0, 0, 1},          // VCO2 PW
    {0, 0, 1},          // VCO2 PWM
    {0, 0, 1},          // VCO2 Mod
    {0, 0, 2},          // VCO2 Mod Src: LFO / S&H / ADSR
    {0, 0, 1},          // Sync: Off / On (VCO 2 to VCO 1)
    {0.8f, 0, 1},       // VCO1 Level
    {0.6f, 0, 1},       // VCO2 Level
    {0, 0, 1},          // Source: Noise / Ring
    {0, 0, 1},          // Source Level
    {0, 0, 3},          // S&H Input: Noise / VCO1 / VCO2 / Mix
    {0, 0, 1},          // S&H Clock: Rate / Key
    {4, 0.1f, 50},      // S&H Rate (Hz)
    {0, 0, 1},          // S&H Lag (s)
    {1500, 20, 20000},  // Cutoff (Hz)
    {0.2f, 0, 1},       // Resonance
    {0, 0, 1},          // Mode: 4-pole / 2-pole
    {0.66f, 0, 1},      // Key Track
    {0.4f, -1, 1},      // Env Amt (+-7 octaves)
    {0, 0, 1},          // Env Src: ADSR / AR
    {0, 0, 1},          // LFO Amt (+-3 octaves)
    {0, 0, 1},          // S&H Amt (+-3 octaves)
    {20, 20, 20000},    // HPF (Hz)
    {0, 0, 1},          // VCA Env: ADSR / AR
    {0.005f, 0, 10},    // Attack
    {0.4f, 0, 10},      // Decay
    {0.6f, 0, 1},       // Sustain
    {0.3f, 0, 10},      // Release
    {0.01f, 0, 10},     // AR Attack
    {0.5f, 0, 10},      // AR Release
    {5, 0.1f, 30},      // LFO Rate (Hz)
    {0, 0, 3},          // LFO Shape: Sine / Tri / Square / Saw
    {0, 0, 1},          // Keyboard: Mono / Duo
    {0, 0, 2},          // Priority: Last / Low / High
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {1, 0, 2},          // Volume
};

const char* const kRangeName[4] = {"32'", "16'", "8'", "4'"};
const char* const kWaveName[2] = {"Saw", "Pulse"};
const char* const kModSrcName[3] = {"LFO", "S&H", "ADSR"};
const char* const kSourceName[2] = {"Noise", "Ring"};
const char* const kShInName[4] = {"Noise", "VCO1", "VCO2", "Mix"};
const char* const kClockName[2] = {"Rate", "Key"};
const char* const kModeName[2] = {"4-pole", "2-pole"};
const char* const kEnvName[2] = {"ADSR", "AR"};
const char* const kLfoName[4] = {"Sine", "Tri", "Square", "Saw"};
const char* const kKeyName[2] = {"Mono", "Duo"};
const char* const kPrioName[3] = {"Last", "Low", "High"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Range", E(kRangeName)},      {"Fine", F(kFmtCents)},        {"Wave", E(kWaveName)},
    {"PW", F(kFmtPct)},            {"PWM", F(kFmtPct)},           {"LFO Mod", F(kFmtPct)},
    {"Range", E(kRangeName)},      {"Tune", F(kFmtSemi)},         {"Fine", F(kFmtCents)},
    {"Wave", E(kWaveName)},        {"PW", F(kFmtPct)},            {"PWM", F(kFmtPct)},
    {"Mod", F(kFmtPct)},           {"Mod Src", E(kModSrcName)},   {"Sync", F(kFmtOnOff)},
    {"VCO1", F(kFmtPct)},          {"VCO2", F(kFmtPct)},          {"Source", E(kSourceName)},
    {"Source Lvl", F(kFmtPct)},    {"Input", E(kShInName)},       {"Clock", E(kClockName)},
    {"Rate", F(kFmtLfoHz)},        {"Lag", F(kFmtSec)},           {"Cutoff", F(kFmtHz)},
    {"Resonance", F(kFmtPct)},     {"Mode", E(kModeName)},        {"Key Track", F(kFmtPct)},
    {"Env Amt", F(kFmtPct)},       {"Env Src", E(kEnvName)},      {"LFO Amt", F(kFmtPct)},
    {"S&H Amt", F(kFmtPct)},       {"HPF", F(kFmtHz)},            {"VCA Env", E(kEnvName)},
    {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},       {"AR Attack", F(kFmtSec)},     {"AR Release", F(kFmtSec)},
    {"Rate", F(kFmtLfoHz)},        {"Shape", E(kLfoName)},        {"Keyboard", E(kKeyName)},
    {"Priority", E(kPrioName)},    {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)},
    {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kOsc1[] = {kVco1Range, kVco1Fine, kVco1Wave, kVco1Pw, kVco1Pwm, kVco1LfoMod};
const uint8_t kOsc2[] = {kVco2Range, kVco2Tune, kVco2Fine, kVco2Wave, kVco2Pw, kVco2Pwm, kVco2Mod, kVco2ModSrc, kSync};
const uint8_t kMix[] = {kVco1Level, kVco2Level, kSource, kSourceLevel};
const uint8_t kSh[] = {kShInput, kShClock, kShRate, kShLag};
const uint8_t kVcf[] = {kCutoff, kResonance, kFilterMode, kKeyTrack, kEnvAmt, kEnvSrc, kLfoAmt, kShAmt, kHpf};
const uint8_t kEnv[] = {kVcaEnv, kAttack, kDecay, kSustain, kRelease, kArAttack, kArRelease};
const uint8_t kLfo[] = {kLfoRate, kLfoShape};
const uint8_t kCtrl[] = {kKeyboard, kPriority, kGlideMode, kGlideTime, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};

const Section kSections[] = {
    OT_SYNTH_SECTION("VCO1", "Oscillator 1", kOsc1), OT_SYNTH_SECTION("VCO2", "Oscillator 2", kOsc2),
    OT_SYNTH_SECTION("MIX", "Mixer", kMix),          OT_SYNTH_SECTION("S&H", "Sample & Hold", kSh),
    OT_SYNTH_SECTION("VCF", "Filter", kVcf),         OT_SYNTH_SECTION("ENV", "Envelopes", kEnv),
    OT_SYNTH_SECTION("LFO", "LFO", kLfo),            OT_SYNTH_SECTION("CTRL", "Controllers", kCtrl),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kAtypeDescriptor;
const SynthDescriptor kAtypeDescriptor = {audio::atype::kEngineId, "ATYPE", kSynthFlagNotePriority, int8_t(audio::atype::kVolume),
                                         uint8_t(audio::atype::kParamCount), kParams, kUi, kSections,
                                         uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "A-Type",
                                         int8_t(audio::atype::kPriority)};

}  // namespace mod
}  // namespace ot
