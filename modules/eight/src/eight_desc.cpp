// 8-Type descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/8TYPE/, "8-Type" on the Synth page
// (displayName).
#include "eight.h"

#include "audio/chip_dsp.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::eight;
namespace chip = ot::audio::chip;

#define OSC(wave, level, filt)                                                                                   \
    {wave, 0, 7}, {0, 0, 1}, {0, 0, 1}, {level, 0, 1}, {0, -24, 24}, {0, -100, 100}, {0, 0, 1}, {0, 0, 1}, \
        {filt, 0, 1}, {0, 0, 1}
// Init: oscillator 1 a 50 % pulse at 0.8 into the filter (low-pass at
// 1,200 Hz, a little resonance, Key Track 50 %, a short positive envelope),
// oscillators 2 and 3 off; ADSR 8 ms / 283 ms / 10 / 172 ms; Grit 12 (clean).
const PatchParamRange kParams[kParamCount] = {
    OSC(2, 0.8f, 1),    // OSC1: Wave, PW, PWM Depth, Level, Semi, Fine, Ring, Sync, Filt, Clean
    OSC(7, 0.0f, 1),    // OSC2
    OSC(7, 0.0f, 1),    // OSC3
    {1200, 30, 12000},  // Cutoff (Hz)
    {2, 0, 15},         // Resonance (16 steps)
    {0, 0, 6},          // Mode: LP BP HP LP+BP Notch BP+HP All
    {0, 0, 1},          // Curve: Bright / Dark
    {0, 0, 1},          // Dirt
    {0.5f, 0, 1},       // Key Track
    {0.3f, -1, 1},      // Env Amt (+-4 octaves)
    {0.25f, 0.01f, 5},  // Env Decay (s)
    {1, 0, 15},         // Attack (table)
    {6, 0, 15},         // Decay
    {10, 0, 15},        // Sustain (4-bit)
    {5, 0, 15},         // Release
    {0.5f, 0.05f, 20},  // PWM Rate (Hz)
    {12, 4, 12},        // Grit: the DAC bits of the oscillator mix (12 = clean)
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {1, 0, 2},          // Volume
    {0, 0, 11},         // Arp Mode
    {1, 0, 8},          // Arp Rate
    {4, 0, 24},         // Arp Note 2
    {7, 0, 24},         // Arp Note 3
    {5, 0.5f, 15},      // Vib Rate
    {0, 0, 2},          // Vib Depth
    {0.2f, 0, 1},       // Vib Delay
    {0, -24, 24},       // Slide Start
    {0.05f, 0, 1},      // Slide Time
};
#undef OSC

const char* const kWaveName[8] = {"Tri", "Saw", "Pulse", "Noise", "Tri+Pulse", "Saw+Pulse", "Tri+Saw", "Off"};
const char* const kModeName[7] = {"LP", "BP", "HP", "LP+BP", "Notch", "BP+HP", "All"};
const char* const kCurveName[2] = {"Bright", "Dark"};
const char* const kAttName[16] = {"2 ms",   "8 ms",   "13 ms", "22 ms", "35 ms", "58 ms", "94 ms", "155 ms",
                                  "253 ms", "414 ms", "679 ms", "1.1 s", "1.8 s", "3.0 s", "4.9 s", "8 s"};
const char* const kDecName[16] = {"6 ms",   "24 ms", "39 ms", "64 ms", "106 ms", "173 ms", "283 ms", "463 ms",
                                  "759 ms", "1.2 s", "2.0 s", "3.3 s", "5.5 s",  "8.9 s",  "15 s",   "24 s"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
#define OSCUI                                                                                                      \
    {"Wave", E(kWaveName)}, {"PW", F(kFmtPct)}, {"PWM Depth", F(kFmtPct)}, {"Level", F(kFmtPct)},           \
        {"Semi", F(kFmtSemi)}, {"Fine", F(kFmtCents)}, {"Ring", F(kFmtOnOff)}, {"Sync", F(kFmtOnOff)},          \
        {"Filt", F(kFmtOnOff)}, {"Clean", F(kFmtOnOff)}
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    OSCUI, OSCUI, OSCUI,
    {"Cutoff", F(kFmtHz)},         {"Resonance", F(kFmtInt)},     {"Mode", E(kModeName)},
    {"Curve", E(kCurveName)},      {"Dirt", F(kFmtPct)},          {"Key Track", F(kFmtPct)},
    {"Env Amt", F(kFmtPct)},       {"Env Decay", F(kFmtSec)},     {"Attack", E(kAttName)},
    {"Decay", E(kDecName)},        {"Sustain", F(kFmtInt)},       {"Release", E(kDecName)},
    {"PWM Rate", F(kFmtLfoHz)},    {"Grit", F(kFmtInt)},          {"Glide Mode", E(names::kGlide)},
    {"Glide Time", F(kFmtSec)},    {"Volume", F(kFmtPct)},        {"Arp Mode", E(chip::kArpNames)},
    {"Arp Rate", E(chip::kRateNames)}, {"Arp Note 2", F(kFmtSemi)}, {"Arp Note 3", F(kFmtSemi)},
    {"Vib Rate", F(kFmtLfoHz)},    {"Vib Depth", F(kFmtSemi)},    {"Vib Delay", F(kFmtSec)},
    {"Slide Start", F(kFmtSemi)},  {"Slide Time", F(kFmtSec)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef OSCUI
#undef E
#undef F

#define OSCS(k) {osc(k, 0), osc(k, 1), osc(k, 2), osc(k, 3), osc(k, 4), osc(k, 5), osc(k, 6), osc(k, 7), osc(k, 8), osc(k, 9)}
const uint8_t kOsc1[] = OSCS(0);
const uint8_t kOsc2[] = OSCS(1);
const uint8_t kOsc3[] = OSCS(2);
#undef OSCS
const uint8_t kFilter[] = {kCutoff, kResonance, kMode, kCurve, kDirt, kKeyTrack, kEnvAmt, kEnvDecay};
const uint8_t kEnv[] = {kAttack, kDecay, kSustain, kRelease};
const uint8_t kVoice[] = {kPwmRate, kGrit, kGlideMode, kGlideTime, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};
const uint8_t kChip[] = {kArpMode, kArpRate, kArpNote2, kArpNote3, kVibRate, kVibDepth, kVibDelay, kSlideStart, kSlideTime};

const Section kSections[] = {
    OT_SYNTH_SECTION("OSC1", "Oscillator 1", kOsc1), OT_SYNTH_SECTION("OSC2", "Oscillator 2", kOsc2),
    OT_SYNTH_SECTION("OSC3", "Oscillator 3", kOsc3), OT_SYNTH_SECTION("FILTER", "Filter", kFilter),
    OT_SYNTH_SECTION("ENV", "Envelope", kEnv),       OT_SYNTH_SECTION("VOICE", "Voice", kVoice),
    OT_SYNTH_SECTION("CHIP", "Chip Tricks", kChip),  OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kEightDescriptor;
const SynthDescriptor kEightDescriptor = {audio::eight::kEngineId, "8TYPE", 0, int8_t(audio::eight::kVolume),
                                         uint8_t(audio::eight::kParamCount), kParams, kUi, kSections,
                                         uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "8-Type"};

}  // namespace mod
}  // namespace ot
