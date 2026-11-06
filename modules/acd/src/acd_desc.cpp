// ACD synth descriptor (synth_descriptor.h, docs/PLAN_3X_M2.md M-A): the stored
// parameter table (spec 21 §8.2 with the MINI_PLAN corrections) and the Edit Patch
// labels, value formats, enum names and sections (Tracker Mini manual), moved
// from the OpenTracker core.
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::synthui::names;

// --- stored ranges -----------------------------------------------------------
const PatchParamRange kParams[29] = {
    {1, 0, 1},     {0, 0, 1},       {0, 0, 1},       {0, -1, 1},    {0.5f, 0, 1},  {0, 0, 5},
    {0, -1, 1},    {0, -1, 1},      {0, -3, 3},      {400, 20, 20000}, {0.5f, 0, 1.5f}, {1, 0, 2},
    {1, -2, 2},    {0, -1, 1},      {0.14f, -1, 1},  {0, 0, 10},    {0.5f, 0, 10}, {0, 0, 1},
    {0.5f, 0, 10}, {0, 0, 10},      {0.5f, 0, 10},   {0, 0, 1},     {0.5f, 0, 10}, {1, 0.1f, 100},
    {1, 0, 5},     {2, 0, 2},       {0, 0, 3},       {0, 0, 3},     {1, 0, 2},
};

// [OT] extension parameters (2.x stereo, patch field 16): Width, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

// --- enum names --------------------------------------------------------------
const char* const kSubMode[6] = {"-1 Oct Square", "-1 Oct Pulse", "-1 Oct Triangle",
                                 "-2 Oct Square", "-2 Oct Pulse", "-2 Oct Triangle"};
const char* const kAcdFilter[3] = {"LP SVF 12dB", "LP SVF 24dB", "LP RD3"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0

// --- Edit Patch --------------------------------------------------------------
// [OT] the table ends with the extension parameters (Width), not macro targets.
const ParamUi kUi[31] = {
    {"Saw Mix", F(kFmtPct)},       {"Square Mix", F(kFmtPct)},   {"Sub Mix", F(kFmtPct)},
    {"Noise Mix", F(kFmtPct)},     {"PW", F(kFmtPct)},           {"Sub Mode", E(kSubMode)},
    {"PW LFO", F(kFmtPct)},        {"PW Env", F(kFmtPct)},       {"Pitch LFO", F(kFmtPctMax)},
    {"Cutoff", F(kFmtHz)},         {"Resonance", F(kFmtPctMax)}, {"Type", E(kAcdFilter)},
    {"Note Track", F(kFmtPct)},    {"LFO Amt", F(kFmtPct)},      {"Env Amt", F(kFmtPct)},
    {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},        {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},       {"Attack", F(kFmtSec)},       {"Decay", F(kFmtSec)},
    {"Sustain", F(kFmtPct)},       {"Release", F(kFmtSec)},      {"Frequency", F(kFmtLfoHz)},
    {"Waveform", E(kWave)},        {"Retrigger", E(kRetrig)},    {"Glide Mode", E(kGlide)},
    {"Glide Time", F(kFmtSec)},    {"Volume", F(kFmtPct)},       {"Width", F(kFmtPct)},
    {"Spread", F(kFmtPct)},
};
const uint8_t kAcdOsc[] = {0, 1, 2, 3, 4, 5, 8, 7, 6};
const uint8_t kAcdFlt[] = {9, 10, 11, 14, 12, 13};
const uint8_t kAcdAmp[] = {15, 16, 17, 18};
const uint8_t kAcdEnv1[] = {19, 20, 21, 22};
const uint8_t kAcdLfo[] = {23, 24, 25};
const uint8_t kAcdVoice[] = {26, 27, 28, 29, 30};

#undef E
#undef F

const Section kSections[] = {OT_SYNTH_SECTION("Osc", "Oscillator", kAcdOsc), OT_SYNTH_SECTION("Filter", "Filter", kAcdFlt),
                           OT_SYNTH_SECTION("Amp Env", "Amp Env", kAcdAmp), OT_SYNTH_SECTION("Env 1", "Mod Env", kAcdEnv1),
                           OT_SYNTH_SECTION("LFO 1", "LFO", kAcdLfo),       OT_SYNTH_SECTION("Voice", "Voice", kAcdVoice), OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kAcdDescriptor;
const SynthDescriptor kAcdDescriptor = {kSynthEngineAcd, "ACD", 0, 28, 29, kParams, kUi, kSections,
                                     uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt};

}  // namespace mod
}  // namespace ot
