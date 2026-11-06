// FAT synth descriptor (synth_descriptor.h, docs/PLAN_3X_M2.md M-A): the stored
// parameter table (spec 21 §8.3 with the MINI_PLAN corrections) and the Edit Patch
// labels, value formats, enum names and sections (Tracker Mini manual), moved
// from the OpenTracker core.
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::synthui::names;

// --- stored ranges -----------------------------------------------------------
const PatchParamRange kParams[25] = {
    {0.1f, 0, 1},  {1, -1, 1},      {0.05f, 0, 1},   {0, 0, 1},     {0, 0, 1},     {1, 0, 1},
    {0, 0, 2},     {400, 20, 20000}, {0.5f, 0, 1.5f}, {0, -1, 1},   {0.14f, -1, 1}, {0, 0, 10},
    {0.5f, 0, 10}, {0, 0, 1},       {0.5f, 0, 10},   {0, 0, 10},    {0.5f, 0, 10}, {0, 0, 1},
    {0.5f, 0, 10}, {1, 0.1f, 100},  {1, 0, 5},       {2, 0, 2},     {0, 0, 3},     {0, 0, 3},
    {1, 0, 2},
};

// [OT] extension parameters (2.x stereo, patch field 16): Width (slot level)
// and Stereo (the stack's pairs split between L and R), 0..1, 0 = mono.
const PatchParamRange kExt[3] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent},
                                  {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

// --- enum names --------------------------------------------------------------
const char* const kFatFilter[3] = {"LP MG 24dB", "LP OB 24dB", "LP OB 12dB"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0

// --- Edit Patch --------------------------------------------------------------
// [OT] the table ends with the extension parameters (Width, Stereo), not macro targets.
const ParamUi kUi[28] = {
    {"Fatness", F(kFmtVal100)},    {"Brightness", F(kFmtVal100)}, {"LFO Amt", F(kFmtPct)},
    {"Timbre", F(kFmtVal100)},     {"Noise", F(kFmtPct)},         {"Reset", F(kFmtOnOff)},
    {"Type", E(kFatFilter)},       {"Cutoff", F(kFmtHz)},         {"Resonance", F(kFmtPctMax)},
    {"LFO Amt", F(kFmtPct)},       {"Env Amt", F(kFmtPct)},       {"Attack", F(kFmtSec)},
    {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},       {"Release", F(kFmtSec)},
    {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},       {"Frequency", F(kFmtLfoHz)},   {"Waveform", E(kWave)},
    {"Retrigger", E(kRetrig)},     {"Glide Mode", E(kGlide)},     {"Glide Time", F(kFmtSec)},
    {"Volume", F(kFmtPct)},        {"Width", F(kFmtPct)},         {"Stereo", F(kFmtPct)},
    {"Spread", F(kFmtPct)},
};
const uint8_t kFatOsc[] = {0, 1, 3, 2, 4, 5, 26};
const uint8_t kFatFlt[] = {7, 8, 6, 10, 9};
const uint8_t kFatAmp[] = {11, 12, 13, 14};
const uint8_t kFatEnv1[] = {15, 16, 17, 18};
const uint8_t kFatLfo[] = {19, 20, 21};
const uint8_t kFatVoice[] = {22, 23, 24, 25, 27};

#undef E
#undef F

const Section kSections[] = {OT_SYNTH_SECTION("Osc", "Oscillator", kFatOsc), OT_SYNTH_SECTION("Filter", "Filter", kFatFlt),
                           OT_SYNTH_SECTION("Amp Env", "Amp Env", kFatAmp), OT_SYNTH_SECTION("Env 1", "Filter Env", kFatEnv1),
                           OT_SYNTH_SECTION("LFO 1", "LFO", kFatLfo),       OT_SYNTH_SECTION("Voice", "Voice", kFatVoice), OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kFatDescriptor;
const SynthDescriptor kFatDescriptor = {kSynthEngineFat, "FAT", 0, 24, 25, kParams, kUi, kSections,
                                     uint8_t(sizeof(kSections) / sizeof(kSections[0])), 3, kExt};

}  // namespace mod
}  // namespace ot
