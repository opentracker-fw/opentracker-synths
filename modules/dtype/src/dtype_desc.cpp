// D-Type synth descriptor (synth_descriptor.h): stored ranges, Edit Patch
// labels, formats, enum names and sections. Patches/DTYPE/, "D-Type" on the
// Synth page (displayName).
#include "dtype.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::dtype;

// Init: two saws at 8' (VCO 2 7 cents up) through a half-open ladder with a
// positive contour, Decay On, glide at 0 (mode Always, as ACD: every note of
// a tracker line restarts the contours; Legato only when chosen). VCO 3 is
// a slow triangle LFO (Lo, off the keyboard), silent in the mix, ready for
// the Wheel.
const PatchParamRange kParams[kParamCount] = {
    {3, 0, 5},          // VCO1 Range: Lo 32' 16' 8' 4' 2'
    {2, 0, 5},          // VCO1 Wave: Tri Tri-Saw Saw Square Wide Narrow
    {0.8f, 0, 1},       // VCO1 Level
    {3, 0, 5},          // VCO2 Range
    {2, 0, 5},          // VCO2 Wave
    {0, -24, 24},       // VCO2 Tune (semitones)
    {7, -100, 100},     // VCO2 Fine (cents)
    {0.6f, 0, 1},       // VCO2 Level
    {0, 0, 5},          // VCO3 Range
    {0, 0, 5},          // VCO3 Wave: Tri Rev-Saw Saw Square Wide Narrow
    {0, -24, 24},       // VCO3 Tune
    {0, -100, 100},     // VCO3 Fine
    {0, 0, 1},          // VCO3 Level
    {0, 0, 1},          // VCO3 Kb: Off (fixed from C3) / On
    {0, 0, 1},          // Noise Level
    {0, 0, 1},          // Noise: White / Pink
    {0, 0, 1},          // Feedback
    {1200, 20, 20000},  // Cutoff (Hz)
    {0.3f, 0, 1},       // Emphasis
    {0.4f, 0, 1},       // Contour amount (0..+6 octaves)
    {2, 0, 3},          // Key Track: Off 1/3 2/3 Full
    {0.005f, 0, 10},    // Filter Attack
    {0.5f, 0, 10},      // Filter Decay
    {0.3f, 0, 1},       // Filter Sustain
    {0.005f, 0, 10},    // Loud Attack
    {0.6f, 0, 10},      // Loud Decay
    {0.8f, 0, 1},       // Loud Sustain
    {0, 0, 1},          // Mod Mix: VCO 3 .. Noise
    {0.15f, 0, 1},      // Osc Mod
    {0, 0, 1},          // Filter Mod
    {0, 0, 1},          // Wheel
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {1, 0, 1},          // Decay switch: Off (short release) / On (release = Decay)
    {1, 0, 2},          // Volume
};

const char* const kRangeName[6] = {"Lo", "32'", "16'", "8'", "4'", "2'"};
const char* const kWave12Name[6] = {"Tri", "Tri-Saw", "Saw", "Square", "Wide", "Narrow"};
const char* const kWave3Name[6] = {"Tri", "Rev Saw", "Saw", "Square", "Wide", "Narrow"};
const char* const kColourName[2] = {"White", "Pink"};
const char* const kKeyName[4] = {"Off", "1/3", "2/3", "Full"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Range", E(kRangeName)},     {"Wave", E(kWave12Name)},      {"Level", F(kFmtPct)},
    {"Range", E(kRangeName)},     {"Wave", E(kWave12Name)},      {"Tune", F(kFmtSemi)},
    {"Fine", F(kFmtCents)},       {"Level", F(kFmtPct)},         {"Range", E(kRangeName)},
    {"Wave", E(kWave3Name)},      {"Tune", F(kFmtSemi)},         {"Fine", F(kFmtCents)},
    {"Level", F(kFmtPct)},        {"Keyboard", F(kFmtOnOff)},    {"Noise", F(kFmtPct)},
    {"Colour", E(kColourName)},   {"Feedback", F(kFmtPct)},      {"Cutoff", F(kFmtHz)},
    {"Emphasis", F(kFmtPct)},     {"Contour", F(kFmtPct)},       {"Key Track", E(kKeyName)},
    {"Attack", F(kFmtSec)},       {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Attack", F(kFmtSec)},       {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Mod Mix", F(kFmtPct)},      {"Osc Mod", F(kFmtPct)},       {"Filter Mod", F(kFmtPct)},
    {"Wheel", F(kFmtPct)},        {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)},
    {"Decay Sw", F(kFmtOnOff)},   {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kOsc1[] = {kVco1Range, kVco1Wave, kVco1Level};
const uint8_t kOsc2[] = {kVco2Range, kVco2Wave, kVco2Tune, kVco2Fine, kVco2Level};
const uint8_t kOsc3[] = {kVco3Range, kVco3Wave, kVco3Tune, kVco3Fine, kVco3Level, kVco3Kb};
const uint8_t kMix[] = {kNoiseLevel, kNoiseColour, kFeedback};
const uint8_t kVcf[] = {kCutoff, kEmphasis, kContour, kKeyTrack};
const uint8_t kFenv[] = {kFAttack, kFDecay, kFSustain};
const uint8_t kLenv[] = {kLAttack, kLDecay, kLSustain};
const uint8_t kMod[] = {kModMix, kOscMod, kFilterMod, kWheel};
const uint8_t kCtrl[] = {kGlideMode, kGlideTime, kDecaySw, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};

const Section kSections[] = {
    OT_SYNTH_SECTION("VCO1", "Oscillator 1", kOsc1),   OT_SYNTH_SECTION("VCO2", "Oscillator 2", kOsc2),
    OT_SYNTH_SECTION("VCO3", "Oscillator 3", kOsc3),   OT_SYNTH_SECTION("MIX", "Mixer", kMix),
    OT_SYNTH_SECTION("VCF", "Filter", kVcf),           OT_SYNTH_SECTION("F.Env", "Filter Contour", kFenv),
    OT_SYNTH_SECTION("L.Env", "Loudness Contour", kLenv), OT_SYNTH_SECTION("MOD", "Modulation", kMod),
    OT_SYNTH_SECTION("CTRL", "Controllers", kCtrl),    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kDtypeDescriptor;
const SynthDescriptor kDtypeDescriptor = {audio::dtype::kEngineId, "DTYPE", 0, int8_t(audio::dtype::kVolume),
                                         uint8_t(audio::dtype::kParamCount), kParams, kUi, kSections,
                                         uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "D-Type"};

}  // namespace mod
}  // namespace ot
