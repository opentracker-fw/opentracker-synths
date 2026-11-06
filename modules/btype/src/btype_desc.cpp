// B-Type descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/BTYPE/, "B-Type" on the Synth
// page (displayName).
#include "btype.h"

#include "audio/chip_dsp.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::btype;
namespace chip = ot::audio::chip;

// Init: a pure square from the 64 kHz clock in tune (Free), the counters
// reset at each note, no high-pass; a 4-bit ADSR (Decay 6 frames a level to
// Sustain 10, Release 2), a 4-bit DAC; the CHIP tricks off.
const PatchParamRange kParams[kParamCount] = {
    {1, 0, 2},          // Clock: 15.6 kHz / 64 kHz / 1.79 MHz
    {0, 0, 7},          // Distortion
    {0, 0, 2},          // Tuning: Free / Coarse / Fine
    {1, 0, 1},          // Poly Reset
    {0, 0, 1},          // Hi-Pass
    {1000, 50, 8000},   // HP Freq (Hz)
    {0, 0, 1},          // Osc 2 Level
    {1, -8, 8},         // Osc 2 Detune (divider steps)
    {0, 0, 7},          // Osc 2 Distortion
    {0, 0, 15},         // Attack (1/60 s frames a level)
    {6, 0, 15},         // Decay
    {10, 0, 15},        // Sustain
    {2, 0, 15},         // Release
    {1, 0, 1},          // Env Bits: Off (smooth) / On (4-bit steps)
    {0, 0, 11},         // Arp Mode
    {1, 0, 8},          // Arp Rate
    {4, 0, 24},         // Arp Note 2
    {7, 0, 24},         // Arp Note 3
    {5, 0.5f, 15},      // Vib Rate
    {0, 0, 2},          // Vib Depth
    {0.2f, 0, 1},       // Vib Delay
    {0, -24, 24},       // Slide Start
    {0.05f, 0, 1},      // Slide Time
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {4, 4, 8},          // DAC Bits (8 = clean)
    {1, 0, 2},          // Volume
};

const char* const kClockName[3] = {"15.6 kHz", "64 kHz", "1.79 MHz"};
const char* const kDistName[8] = {"Pure", "Poly4", "Poly5", "Poly9", "Poly17", "Poly5x4", "Poly5x17", "Poly5x9"};
const char* const kTuneName[3] = {"Free", "Coarse", "Fine"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Clock", E(kClockName)},      {"Distortion", E(kDistName)},  {"Tuning", E(kTuneName)},
    {"Poly Reset", F(kFmtOnOff)},  {"Hi-Pass", F(kFmtOnOff)},     {"HP Freq", F(kFmtHz)},
    {"Level", F(kFmtPct)},         {"Detune", F(kFmtInt)},        {"Distortion", E(kDistName)},
    {"Attack", F(kFmtInt)},        {"Decay", F(kFmtInt)},         {"Sustain", F(kFmtInt)},
    {"Release", F(kFmtInt)},       {"Bits", F(kFmtOnOff)},        {"Arp Mode", E(chip::kArpNames)},
    {"Arp Rate", E(chip::kRateNames)}, {"Arp Note 2", F(kFmtSemi)}, {"Arp Note 3", F(kFmtSemi)},
    {"Vib Rate", F(kFmtLfoHz)},    {"Vib Depth", F(kFmtSemi)},    {"Vib Delay", F(kFmtSec)},
    {"Slide Start", F(kFmtSemi)},  {"Slide Time", F(kFmtSec)},    {"Glide Mode", E(names::kGlide)},
    {"Glide Time", F(kFmtSec)},    {"DAC Bits", F(kFmtInt)},      {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kChan[] = {kClock, kDistortion, kTuning, kPolyReset, kHiPass, kHpFreq};
const uint8_t kOsc2[] = {kOsc2Level, kOsc2Detune, kOsc2Distortion};
const uint8_t kEnv[] = {kAttack, kDecay, kSustain, kRelease, kEnvBits};
const uint8_t kChip[] = {kArpMode, kArpRate, kArpNote2, kArpNote3, kVibRate, kVibDepth, kVibDelay, kSlideStart, kSlideTime};
const uint8_t kCtrl[] = {kGlideMode, kGlideTime, kDacBits, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};

const Section kSections[] = {
    OT_SYNTH_SECTION("CHAN", "Channel", kChan), OT_SYNTH_SECTION("OSC2", "Channel 2", kOsc2),
    OT_SYNTH_SECTION("ENV", "Envelope", kEnv),  OT_SYNTH_SECTION("CHIP", "Chip Tricks", kChip),
    OT_SYNTH_SECTION("CTRL", "Controllers", kCtrl), OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kBtypeDescriptor;
const SynthDescriptor kBtypeDescriptor = {audio::btype::kEngineId, "BTYPE", 0, int8_t(audio::btype::kVolume),
                                         uint8_t(audio::btype::kParamCount), kParams, kUi, kSections,
                                         uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "B-Type"};

}  // namespace mod
}  // namespace ot
