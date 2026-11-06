// WTFM synth descriptor (synth_descriptor.h, docs/PLAN_3X_M2.md M-A): the stored
// parameter table (spec 21 §8.5 with the MINI_PLAN corrections) and the Edit Patch
// labels, value formats, enum names and sections (Tracker Mini manual), moved
// from the OpenTracker core.
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::synthui::names;

// --- stored ranges -----------------------------------------------------------
constexpr uint8_t NR = kParamFlagNoRange | kParamFlagInitAbsent;
#define MODSLOT {0, 0, 4}, {0, 0, 13}, {0, -4, 4}
const PatchParamRange kParams[63] = {
    {3, 0, 47},    {3, 0, 47},      {0, -100, 100},  {0, -100, 100}, {0, 0, 1},    {0, 0, 1},
    {0, 0, 0, NR}, {0, 0, 0, NR},   {0, 0, 1},       {0, 0, 8},     {0, 0, 8},     {0, 0, 10},
    {0, 0, 1},     {0, 0, 1},       {0, 0, 1},       {0, 0, 14},    {400, 20, 20000}, {0.5f, 0, 1.5f},
    {0.14f, -1, 1}, {1, -2, 2},
    {0, 0, 10},    {0.5f, 0, 10},   {1, 0, 1},       {0.3f, 0, 10},  // 20-23 Amp Env
    {0, 0, 10},    {0.5f, 0, 10},   {0, 0, 1},       {0.5f, 0, 10},  // 24-27 Env 1
    {0, 0, 10},    {0.5f, 0, 10},   {0, 0, 1},       {0.3f, 0, 10},  // 28-31 Env 2
    {0, 0, 100},   {1, 0, 5},       {2, 0, 2},       {0, 0, 1},      // 32-35 LFO 1
    {0, 0, 100},   {1, 0, 5},       {2, 0, 2},       {0, 0, 1},      // 36-39 LFO 2
    {0, 0, 3},     {0, 0, 3},       {1, 0, 2},
    MODSLOT,       MODSLOT,         MODSLOT,         MODSLOT,       MODSLOT,       MODSLOT,
    {0, 0, 1},     {0, 0, 1},
};
#undef MODSLOT

// [OT] extension parameters (2.x stereo, patch field 16): Width, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

// --- enum names --------------------------------------------------------------
const char* const kWtfmDest[14] = {"Osc1 Pitch",  "Osc2 Pitch",   "Op1 Shape",       "Op2 Shape",       "Op1 Feedback",
                                   "Op2 Feedback", "Op21 Feedback", "FM",             "Filter Cutoff",   "Amp Env Sustain",
                                   "Flt Env Sustain", "Wave Env Sustain", "Volume",   "Flt Env Decay"};
const char* const kCharacter[9] = {"Smoother", "Sharper", "Wilder", "Add 1", "Add 2", "Add 3", "Add 5", "Add 7", "Add 11"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0

// --- Edit Patch --------------------------------------------------------------
#define MODROW(k) {"Src " #k, E(kModSrc)}, {"Dest " #k, E(kWtfmDest)}, {"Amt " #k, F(kFmtPctMax)}
// [OT] the table ends with the extension parameters (Width), not macro targets.
const ParamUi kUi[65] = {
    {"Ratio 1", F(kFmtRatio)},     {"Ratio 2", F(kFmtRatio)},     {"Finetune 1", F(kFmtCents)},
    {"Finetune 2", F(kFmtCents)},  {"Shape 1", F(kFmtVal100)},    {"Shape 2", F(kFmtVal100)},
    {nullptr, F(kFmtVal100)},      {nullptr, F(kFmtVal100)},      {"Oversampling", F(kFmtOnOff)},
    {"Character 1", E(kCharacter)}, {"Character 2", E(kCharacter)}, {"FM", F(kFmtPct)},
    {"Feedback 1", F(kFmtPct)},    {"Feedback 2", F(kFmtPct)},    {"Feedback 2>1", F(kFmtPct)},
    {"Type", E(kFilter15)},        {"Cutoff", F(kFmtHz)},         {"Resonance", F(kFmtPctMax)},
    {"Env Amt", F(kFmtPct)},       {"Note Track", F(kFmtPct)},    {"Attack", F(kFmtSec)},
    {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},       {"Release", F(kFmtSec)},
    {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},         {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},       {"Attack", F(kFmtSec)},        {"Decay", F(kFmtSec)},
    {"Sustain", F(kFmtPct)},       {"Release", F(kFmtSec)},       {"Ratio", F(kFmtLfoHz)},
    {"Waveform", E(kWave)},        {"Retrigger", E(kRetrig)},     {"Sync", F(kFmtOnOff)},
    {"Ratio", F(kFmtLfoHz)},       {"Waveform", E(kWave)},        {"Retrigger", E(kRetrig)},
    {"Sync", F(kFmtOnOff)},        {"Glide Mode", E(kGlide)},     {"Glide Time", F(kFmtSec)},
    {"Volume", F(kFmtPct)},
    MODROW(1), MODROW(2), MODROW(3), MODROW(4), MODROW(5), MODROW(6),
    {"Reset 1", F(kFmtOnOff)},     {"Reset 2", F(kFmtOnOff)},     {"Width", F(kFmtPct)},
    {"Spread", F(kFmtPct)},
};
#undef MODROW
const uint8_t kWtfmOsc[] = {11, 0, 1, 4, 5, 9, 10, 12, 13, 14, 2, 3, 61, 62, 8};
const uint8_t kWtfmFlt[] = {16, 17, 15, 18, 19};
const uint8_t kWtfmAmp[] = {20, 21, 22, 23};
const uint8_t kWtfmEnv1[] = {24, 25, 26, 27};
const uint8_t kWtfmEnv2[] = {28, 29, 30, 31};
const uint8_t kWtfmLfo1[] = {32, 33, 34, 35};
const uint8_t kWtfmLfo2[] = {36, 37, 38, 39};
const uint8_t kWtfmMod[] = {43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60};
const uint8_t kWtfmVoice[] = {40, 41, 42, 63, 64};

#undef E
#undef F

const Section kSections[] = {OT_SYNTH_SECTION("Osc", "Oscillator", kWtfmOsc), OT_SYNTH_SECTION("Filter", "Filter", kWtfmFlt),
                            OT_SYNTH_SECTION("Amp Env", "Amp Env", kWtfmAmp), OT_SYNTH_SECTION("Env 1", "Envelope 1", kWtfmEnv1),
                            OT_SYNTH_SECTION("Env 2", "Envelope 2", kWtfmEnv2), OT_SYNTH_SECTION("LFO 1", "LFO 1", kWtfmLfo1),
                            OT_SYNTH_SECTION("LFO 2", "LFO 2", kWtfmLfo2),   OT_SYNTH_SECTION("Mod", "Modulation", kWtfmMod),
                            OT_SYNTH_SECTION("Voice", "Voice", kWtfmVoice),  OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kWtfmDescriptor;
const SynthDescriptor kWtfmDescriptor = {kSynthEngineWtfm, "WTFM", 0, 42, 63, kParams, kUi, kSections,
                                     uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt};

}  // namespace mod
}  // namespace ot
