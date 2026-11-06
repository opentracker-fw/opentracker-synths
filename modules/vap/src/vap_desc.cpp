// VAP synth descriptor (synth_descriptor.h, docs/PLAN_3X_M2.md M-A): the stored
// parameter table (spec 21 §8.4 with the MINI_PLAN corrections) and the Edit Patch
// labels, value formats, enum names and sections (Tracker Mini manual), moved
// from the OpenTracker core.
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::synthui::names;

// --- stored ranges -----------------------------------------------------------
#define MODSLOT {0, 0, 4}, {0, 0, 13}, {0, -4, 4}
const PatchParamRange kParams[60] = {
    {0.5f, 0, 1},  {0.5f, 0, 1},    {0, -0.5f, 0.5f}, {0, -0.5f, 0.5f}, {0, -1, 1},  {0, -36, 36},
    {0, -36, 36},  {0, 0, 100},     {0, -100, 100},  {0, 0, 1},     {0, 0, 1},     {0, 0, 14},
    {400, 20, 20000}, {0.5f, 0, 1.5f}, {0.14f, -1, 1}, {0, 0, 10},   {0.5f, 0, 10}, {1, 0, 1},
    {0.3f, 0, 10}, {0, 0, 10},      {0.5f, 0, 10},   {0, 0, 1},     {0.5f, 0, 10}, {0, 0, 10},
    {0.5f, 0, 10}, {0, 0, 1},       {0.3f, 0, 10},
    {0, 0, 100},   {1, 0, 5},       {2, 0, 2},       {0, 0, 1},  // 27-30 LFO 1
    {0, 0, 100},   {1, 0, 5},       {2, 0, 2},       {0, 0, 1},  // 31-34 LFO 2
    {0, 0, 3},     {0, 0, 3},       {1, -2, 2},      {0, -2, 2},    {1, 0, 2},     {0, 0, 1},
    {0, 0, 1},     // 41 Noise
    MODSLOT,       MODSLOT,         MODSLOT,         MODSLOT,       MODSLOT,       MODSLOT,
};
#undef MODSLOT

// [OT] extension parameters (2.x stereo, patch field 16): Width, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

// --- enum names --------------------------------------------------------------
const char* const kVapDest[14] = {"Osc1 PWM",   "Osc2 PWM",         "Osc Mix",    "Filter Cutoff", "Amp Env Sustain",
                                  "Flt Env Sustain", "Wave Env Sustain", "Volume", "Osc1 Pitch", "Osc2 Pitch",
                                  "Flt Env Decay", "Noise Volume",  "Osc1 Shape", "Osc2 Shape"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0

// --- Edit Patch --------------------------------------------------------------
#define MODROW(k) {"Src " #k, E(kModSrc)}, {"Dest " #k, E(kVapDest)}, {"Amt " #k, F(kFmtPctMax)}
// [OT] the table ends with the extension parameters (Width), not macro targets.
const ParamUi kUi[62] = {
    {"Shape 1", F(kFmtVal100)},   {"Shape 2", F(kFmtVal100)},  {"PW 1", F(kFmtVal100)},
    {"PW 2", F(kFmtVal100)},      {"Mix", F(kFmtPct)},         {"Tune 1", F(kFmtSemi)},
    {"Tune 2", F(kFmtSemi)},      {"Detune", F(kFmtCents)},    {"Finetune", F(kFmtCents)},
    {"Reset 1", F(kFmtOnOff)},    {"Reset 2", F(kFmtOnOff)},   {"Type", E(kFilter15)},
    {"Cutoff", F(kFmtHz)},        {"Resonance", F(kFmtPctMax)}, {"Env Amt", F(kFmtPct)},
    {"Attack", F(kFmtSec)},       {"Decay", F(kFmtSec)},       {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},      {"Attack", F(kFmtSec)},      {"Decay", F(kFmtSec)},
    {"Sustain", F(kFmtPct)},      {"Release", F(kFmtSec)},     {"Attack", F(kFmtSec)},
    {"Decay", F(kFmtSec)},        {"Sustain", F(kFmtPct)},     {"Release", F(kFmtSec)},
    {"Ratio", F(kFmtLfoHz)},      {"Waveform", E(kWave)},      {"Retrigger", E(kRetrig)},
    {"Sync", F(kFmtOnOff)},       {"Ratio", F(kFmtLfoHz)},     {"Waveform", E(kWave)},
    {"Retrigger", E(kRetrig)},    {"Sync", F(kFmtOnOff)},      {"Glide Mode", E(kGlide)},
    {"Glide Time", F(kFmtSec)},   {"Note Track", F(kFmtPct)},  {"Mix Env Amt", F(kFmtPct)},
    {"Volume", F(kFmtPct)},       {"Sync", F(kFmtOnOff)},      {"Noise", F(kFmtPct)},
    MODROW(1), MODROW(2), MODROW(3), MODROW(4), MODROW(5), MODROW(6),
    {"Width", F(kFmtPct)},
    {"Spread", F(kFmtPct)},
};
#undef MODROW
const uint8_t kVapOsc[] = {4, 0, 1, 2, 3, 5, 6, 8, 7, 41, 40, 9, 10, 38};
const uint8_t kVapFlt[] = {12, 13, 11, 14, 37};
const uint8_t kVapAmp[] = {15, 16, 17, 18};
const uint8_t kVapEnv1[] = {19, 20, 21, 22};
const uint8_t kVapEnv2[] = {23, 24, 25, 26};
const uint8_t kVapLfo1[] = {27, 28, 29, 30};
const uint8_t kVapLfo2[] = {31, 32, 33, 34};
const uint8_t kVapMod[] = {42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
const uint8_t kVapVoice[] = {35, 36, 39, 60, 61};

#undef E
#undef F

const Section kSections[] = {OT_SYNTH_SECTION("Osc", "Oscillators", kVapOsc), OT_SYNTH_SECTION("Filter", "Filter", kVapFlt),
                           OT_SYNTH_SECTION("Amp Env", "Amp Env", kVapAmp),  OT_SYNTH_SECTION("Env 1", "Envelope 1", kVapEnv1),
                           OT_SYNTH_SECTION("Env 2", "Envelope 2", kVapEnv2), OT_SYNTH_SECTION("LFO 1", "LFO 1", kVapLfo1),
                           OT_SYNTH_SECTION("LFO 2", "LFO 2", kVapLfo2),     OT_SYNTH_SECTION("Mod", "Modulation", kVapMod),
                           OT_SYNTH_SECTION("Voice", "Voice", kVapVoice),    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kVapDescriptor;
const SynthDescriptor kVapDescriptor = {kSynthEngineVap, "VAP", 0, 39, 60, kParams, kUi, kSections,
                                     uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt};

}  // namespace mod
}  // namespace ot
