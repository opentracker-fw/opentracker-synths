// 1-Type synth descriptor (synth_descriptor.h): stored ranges, Edit Patch
// labels, formats, enum names and sections. Patches/1TYPE/, "1-Type" on the
// Synth page (displayName). The Init patch is the classic softsynth's own
// built-in init sound in engineering units (pulse + saw an octave down,
// LP24, an LFO on the cutoff, one on the pulse width).
//
// Parameter set and defaults derived from Oxo Synth2
// (https://github.com/chab/oxo_synth2, commit 6bb4435):
//
//   MIT License
//   Copyright (c) 2026 Francois Chabloz and contributors
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to deal
//   in the Software without restriction, including without limitation the rights
//   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//   copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.
#include "modules/synth_descriptor.h"
#include "onetype.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::onetype;

const PatchParamRange kParams[kParamCount] = {
    {2, 0, 3},              // Osc1 Wave: Sine / Saw / Pulse / Tri
    {0, 0, 1},              // Osc1 Detune
    {0, 0, 1},              // Osc1 FM (from osc 2)
    {0, 0, 1},              // Sub Level
    {1, 0, 3},              // Sub Wave: Sine / Tri / Saw / Square
    {1, 0, 1},              // Sub Octave: 0 / -1
    {0, 0, 3},              // Osc2 Wave: Saw / Pulse / Tri / Noise
    {-12, -60, 60},         // Osc2 Pitch (semitones)
    {15, -100, 100},        // Osc2 Fine (cents)
    {1, 0, 1},              // Osc2 Key Track
    {0, 0, 1},              // Sync (osc 2 to osc 1)
    {0, 0, 1},              // Ring
    {0.5433071f, 0, 1},     // Mix: osc 1 .. osc 2
    {0.503937f, 0, 1},      // Pulse Width
    {0, -24, 24},           // Key Shift (semitones)
    {0, -100, 100},         // Fine Tune (cents)
    {0, 0, 1},              // Phase: 0 = free running
    {0, -1, 1},             // Mod Env Amount (0 = off)
    {0.0005f, 0, 30},       // Mod Env Attack (s)
    {0.00506f, 0, 30},      // Mod Env Decay (s, to 1 %)
    {0, 0, 2},              // Mod Env Dest: Osc2 Pitch / FM / PW
    {1, 0, 4},              // Filter Type: LP12 / LP24 / HP12 / BP12 / LPDL
    {105.88f, 16, 20000},   // Cutoff (Hz)
    {0.1732283f, 0, 1},     // Resonance
    {0.3015681f, -1, 1},    // Env Amt (+-9.9 octaves)
    {0.5868999f, 0, 1},     // Key Track (octaves per octave)
    {0, 0, 1},              // Saturation
    {0.0005f, 0, 30},       // Filter Attack (s)
    {0.0774456f, 0, 30},    // Filter Decay (s, to 1 %)
    {0.1653543f, 0, 1},     // Filter Sustain
    {0.5632916f, 0, 30},    // Filter Release (s, to 1 %)
    {0.0005f, 0, 30},       // Attack (s)
    {0.3999088f, 0, 30},    // Decay (s, to 1 %)
    {0.6821289f, 0, 1},     // Sustain
    {1.0437998f, 0, 30},    // Release (s, to 1 %)
    {2, 0, 6},              // LFO1 Dest: Osc2 / Osc1+2 / Filter / Amp / PW / FM / Pan
    {5, 0, 5},              // LFO1 Wave: Saw / Tri / Square / S&H / Smooth / Sine
    {1.32248f, 0.08f, 130}, // LFO1 Rate (Hz)
    {0.1496063f, 0, 1},     // LFO1 Depth (0 = off)
    {0, 0, 30},             // LFO1 Sync: Off, 16/1 .. 1/64T
    {0, 0, 1},              // LFO1 Key Sync
    {4, 0, 6},              // LFO2 Dest
    {1, 0, 5},              // LFO2 Wave
    {3.32259f, 0.08f, 130}, // LFO2 Rate (Hz)
    {0.503937f, 0, 1},      // LFO2 Depth
    {0, 0, 30},             // LFO2 Sync
    {0, 0, 1},              // LFO2 Key Sync
    {0, 0, 3},              // Glide Mode (names::kGlide): Always
    {0, 0, 3},              // Glide Time (s, to land)
    {1.120679f, 0, 2},      // Volume
};

const char* const kOsc1Name[4] = {"Sine", "Saw", "Pulse", "Tri"};
const char* const kSubName[4] = {"Sine", "Tri", "Saw", "Square"};
const char* const kOctName[2] = {"0", "-1"};
const char* const kOsc2Name[4] = {"Saw", "Pulse", "Tri", "Noise"};
const char* const kModDestName[3] = {"Osc2 Pitch", "FM", "PW"};
const char* const kTypeName[5] = {"LP12", "LP24", "HP12", "BP12", "LPDL"};
const char* const kLfoDestName[7] = {"Osc2", "Osc1+2", "Filter", "Amp", "PW", "FM", "Pan"};
const char* const kLfoWaveName[6] = {"Saw", "Tri", "Square", "S&H", "Smooth", "Sine"};
const char* const kSyncName[kLfoSyncSteps + 1] = {
    "Off",  "16/1", "12/1", "9/1",  "8/1",   "6/1",   "9/2",   "4/1",  "3/1",   "9/4",   "2/1",
    "3/2",  "1/1",  "7/8",  "1/2D", "1/2",   "7/16",  "1/4D",  "1/2T", "1/4",   "7/32",  "1/8D",
    "1/4T", "1/8",  "1/16D", "1/8T", "1/16", "1/16T", "1/32",  "1/32T", "1/64T"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Wave", E(kOsc1Name)},        {"Detune", F(kFmtPct)},        {"FM", F(kFmtPct)},
    {"Sub Level", F(kFmtPct)},     {"Sub Wave", E(kSubName)},     {"Sub Octave", E(kOctName)},
    {"Wave", E(kOsc2Name)},        {"Pitch", F(kFmtSemi)},        {"Fine", F(kFmtCents)},
    {"Key Track", F(kFmtOnOff)},   {"Sync", F(kFmtOnOff)},        {"Ring", F(kFmtOnOff)},
    {"Mix", F(kFmtPct)},           {"Pulse Width", F(kFmtPct)},   {"Key Shift", F(kFmtSemi)},
    {"Fine Tune", F(kFmtCents)},   {"Phase", F(kFmtPct)},
    {"Amount", F(kFmtPct)},        {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},
    {"Dest", E(kModDestName)},
    {"Type", E(kTypeName)},        {"Cutoff", F(kFmtHz)},         {"Resonance", F(kFmtPct)},
    {"Env Amt", F(kFmtPct)},       {"Key Track", F(kFmtPct)},     {"Saturation", F(kFmtPct)},
    {"F Attack", F(kFmtSec)},      {"F Decay", F(kFmtSec)},       {"F Sustain", F(kFmtPct)},
    {"F Release", F(kFmtSec)},
    {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},
    {"Dest", E(kLfoDestName)},     {"Wave", E(kLfoWaveName)},     {"Rate", F(kFmtLfoHz)},
    {"Depth", F(kFmtPct)},         {"Sync", E(kSyncName)},        {"Key Sync", F(kFmtOnOff)},
    {"Dest", E(kLfoDestName)},     {"Wave", E(kLfoWaveName)},     {"Rate", F(kFmtLfoHz)},
    {"Depth", F(kFmtPct)},         {"Sync", E(kSyncName)},        {"Key Sync", F(kFmtOnOff)},
    {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)}, {"Volume", F(kFmtPct)},
    {"Width", F(kFmtPct)},         {"Spread", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kOsc1[] = {kOsc1Wave, kOsc1Detune, kOsc1Fm, kSubLevel, kSubWave, kSubOctave};
const uint8_t kOsc2[] = {kOsc2Wave, kOsc2Pitch, kOsc2Fine, kOsc2Track, kSync, kRing};
const uint8_t kMixS[] = {kMix, kPulseWidth, kKeyShift, kFineTune, kPhase};
const uint8_t kModS[] = {kModAmount, kModAttack, kModDecay, kModDest};
const uint8_t kVcf[] = {kFilterType, kCutoff, kResonance, kEnvAmt, kKeyTrack, kSaturation, kFAttack, kFDecay, kFSustain, kFRelease};
const uint8_t kAmp[] = {kAttack,    kDecay,  kSustain, kRelease, kGlideMode, kGlideTime, kVolume, uint8_t(kParamCount),
                        uint8_t(kParamCount + 1)};
const uint8_t kLfo1[] = {kLfo1Dest, kLfo1Wave, kLfo1Rate, kLfo1Depth, kLfo1Sync, kLfo1KeySync};
const uint8_t kLfo2[] = {kLfo2Dest, kLfo2Wave, kLfo2Rate, kLfo2Depth, kLfo2Sync, kLfo2KeySync};

const Section kSections[] = {
    OT_SYNTH_SECTION("OSC1", "Oscillator 1", kOsc1), OT_SYNTH_SECTION("OSC2", "Oscillator 2", kOsc2),
    OT_SYNTH_SECTION("MIX", "Mixer", kMixS),         OT_SYNTH_SECTION("MOD ENV", "Mod Envelope", kModS),
    OT_SYNTH_SECTION("VCF", "Filter", kVcf),         OT_SYNTH_SECTION("AMP", "Amplifier", kAmp),
    OT_SYNTH_SECTION("LFO1", "LFO 1", kLfo1),        OT_SYNTH_SECTION("LFO2", "LFO 2", kLfo2),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kOnetypeDescriptor;
const SynthDescriptor kOnetypeDescriptor = {audio::onetype::kEngineId, "1TYPE", 0, int8_t(audio::onetype::kVolume),
                                           uint8_t(audio::onetype::kParamCount), kParams, kUi, kSections,
                                           uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "1-Type"};

}  // namespace mod
}  // namespace ot
