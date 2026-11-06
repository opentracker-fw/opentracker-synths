// Hand-Type descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/HANDTYPE/, "Hand-Type" on the
// Synth page (displayName).
#include "hand.h"

#include "audio/chip_dsp.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::hand;
namespace chip = ot::audio::chip;

// Init: a 50 % pulse, the envelope from 15 down a level per 62 ms, Release
// Cut, a 4-bit DAC; the CHIP tricks off; glide 0 (mode Always).
const PatchParamRange kParams[kParamCount] = {
    {0, 0, 3},          // Channel: Pulse / Wave / Noise / Drums
    {2, 0, 3},          // Duty: 12.5 / 25 / 50 / 75 %
    {0, 0, 2},          // Sweep Dir: Off / Up / Down
    {3, 1, 7},          // Sweep Time (ticks of 1/128 s)
    {3, 1, 7},          // Sweep Shift
    {1, 0, 1},          // Sweep Cut
    {0, 0, 1},          // Pulse 2 Level
    {7, -50, 50},       // Pulse 2 Detune (cents)
    {1, 0, 3},          // Pulse 2 Duty
    {1, 0, 11},         // Wave: the built-in tables, User
    {0, 0, 15},         // User 1 .. 8: a ramp
    {2, 0, 15},
    {4, 0, 15},
    {6, 0, 15},
    {9, 0, 15},
    {11, 0, 15},
    {13, 0, 15},
    {15, 0, 15},
    {1, 0, 1},          // Levels: 4 / 16
    {0, 0, 1},          // Width: 15 / 7 bits
    {6, 0, 13},         // Noise Shift
    {0, 0, 7},          // Divider
    {0, 0, 1},          // Track
    {15, 0, 15},        // Env Init
    {0, 0, 1},          // Env Dir: Down / Up
    {3, 0, 6},          // Env Step: 1..7 ticks of 1/64 s
    {0, 0, 1},          // Release: Cut / Fade
    {0, 0, 7},          // Length: Off, 1..7 x 1/16 s
    {0, -24, 24},       // Kick Tune (semitones)
    {5, 1, 7},          // Kick Sweep (speed)
    {3, 0, 7},          // Kick Decay
    {0, 0, 1},          // Snare Width: 15 / 7 bits
    {3, 0, 7},          // Snare Decay
    {3, 0, 13},         // Hat Shift
    {1, 0, 7},          // Hat Closed Decay
    {4, 0, 7},          // Hat Open Decay
    {6, 0, 7},          // Crash Decay
    {3, 0, 7},          // Tom Decay
    {1, 0, 7},          // Blip Decay
    {0, 0, 11},         // Arp Mode
    {1, 0, 8},          // Arp Rate: 50 / 60 / 100 / 120 Hz, 1/32 .. 1/128
    {4, 0, 24},         // Arp Note 2 (semitones)
    {7, 0, 24},         // Arp Note 3
    {5, 0.5f, 15},      // Vib Rate (Hz)
    {0, 0, 2},          // Vib Depth (semitones)
    {0.2f, 0, 1},       // Vib Delay (s)
    {0, -24, 24},       // Slide Start (semitones)
    {0.05f, 0, 1},      // Slide Time (s)
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {4, 4, 8},          // Bits: the DAC (8 = clean)
    {1, 0, 2},          // Volume
};

const char* const kChanName[4] = {"Pulse", "Wave", "Noise", "Drums"};
const char* const kDutyName[4] = {"12.5 %", "25 %", "50 %", "75 %"};
const char* const kSweepName[3] = {"Off", "Up", "Down"};
const char* const kWaveName[12] = {"Sine", "Triangle", "Saw",    "Square 50", "Square 25", "Organ",
                                   "Bass", "Buzz",     "Nasal",  "Hollow",    "Ramp 2",    "User"};
const char* const kLevelsName[2] = {"4", "16"};
const char* const kWidthName[2] = {"15 bit", "7 bit"};
const char* const kDirName[2] = {"Down", "Up"};
const char* const kStepName[7] = {"16 ms", "31 ms", "47 ms", "62 ms", "78 ms", "94 ms", "109 ms"};
const char* const kRelName[2] = {"Cut", "Fade"};
const char* const kLenName[8] = {"Off", "62 ms", "125 ms", "188 ms", "250 ms", "312 ms", "375 ms", "438 ms"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Channel", E(kChanName)},     {"Duty", E(kDutyName)},        {"Sweep", E(kSweepName)},
    {"Sweep Time", F(kFmtInt)},    {"Sweep Shift", F(kFmtInt)},   {"Sweep Cut", F(kFmtOnOff)},
    {"Pulse 2", F(kFmtPct)},       {"P2 Detune", F(kFmtCents)},   {"P2 Duty", E(kDutyName)},
    {"Wave", E(kWaveName)},        {"User 1", F(kFmtInt)},        {"User 2", F(kFmtInt)},
    {"User 3", F(kFmtInt)},        {"User 4", F(kFmtInt)},        {"User 5", F(kFmtInt)},
    {"User 6", F(kFmtInt)},        {"User 7", F(kFmtInt)},        {"User 8", F(kFmtInt)},
    {"Levels", E(kLevelsName)},    {"Width", E(kWidthName)},      {"Shift", F(kFmtInt)},
    {"Divider", F(kFmtInt)},       {"Track", F(kFmtOnOff)},       {"Init", F(kFmtInt)},
    {"Dir", E(kDirName)},          {"Step", E(kStepName)},        {"Release", E(kRelName)},
    {"Length", E(kLenName)},       {"Kick Tune", F(kFmtSemi)},    {"Kick Sweep", F(kFmtInt)},
    {"Kick Decay", F(kFmtInt)},    {"Snare Width", E(kWidthName)}, {"Snare Decay", F(kFmtInt)},
    {"Hat Shift", F(kFmtInt)},     {"Hat Closed", F(kFmtInt)},    {"Hat Open", F(kFmtInt)},
    {"Crash Decay", F(kFmtInt)},   {"Tom Decay", F(kFmtInt)},     {"Blip Decay", F(kFmtInt)},
    {"Arp Mode", E(chip::kArpNames)}, {"Arp Rate", E(chip::kRateNames)}, {"Arp Note 2", F(kFmtSemi)},
    {"Arp Note 3", F(kFmtSemi)},   {"Vib Rate", F(kFmtLfoHz)},    {"Vib Depth", F(kFmtSemi)},
    {"Vib Delay", F(kFmtSec)},     {"Slide Start", F(kFmtSemi)},  {"Slide Time", F(kFmtSec)},
    {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)}, {"Bits", F(kFmtInt)},
    {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kChan[] = {kChannel};
const uint8_t kPulseS[] = {kDuty, kSweepDir, kSweepTime, kSweepShift, kSweepCut, kP2Level, kP2Detune, kP2Duty};
const uint8_t kWaveS[] = {kWave, kUser1, kUser2, kUser3, kUser4, kUser5, kUser6, kUser7, kUser8, kLevels};
const uint8_t kNoiseS[] = {kWidth, kNoiseShift, kDivider, kTrack};
const uint8_t kEnvS[] = {kEnvInit, kEnvDir, kEnvStep, kRelease, kLength};
const uint8_t kDrumS[] = {kKickTune, kKickSweep, kKickDecay, kSnareWidth, kSnareDecay, kHatShift,
                          kHatClosedDecay, kHatOpenDecay, kCrashDecay, kTomDecay, kBlipDecay};
const uint8_t kChipS[] = {kArpMode, kArpRate, kArpNote2, kArpNote3, kVibRate, kVibDepth, kVibDelay, kSlideStart, kSlideTime};
const uint8_t kCtrlS[] = {kGlideMode, kGlideTime, kBits, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};

const Section kSections[] = {
    OT_SYNTH_SECTION("CHAN", "Channel", kChan),   OT_SYNTH_SECTION("PULSE", "Pulse", kPulseS),
    OT_SYNTH_SECTION("WAVE", "Wave", kWaveS),     OT_SYNTH_SECTION("NOISE", "Noise", kNoiseS),
    OT_SYNTH_SECTION("ENV", "Envelope", kEnvS),   OT_SYNTH_SECTION("DRUMS", "Drums", kDrumS),
    OT_SYNTH_SECTION("CHIP", "Chip Tricks", kChipS), OT_SYNTH_SECTION("CTRL", "Controllers", kCtrlS),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kHandDescriptor;
const SynthDescriptor kHandDescriptor = {audio::hand::kEngineId, "HANDTYPE", 0, int8_t(audio::hand::kVolume),
                                        uint8_t(audio::hand::kParamCount), kParams, kUi, kSections,
                                        uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "Hand-Type"};

}  // namespace mod
}  // namespace ot
