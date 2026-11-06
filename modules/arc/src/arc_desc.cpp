// Arc-Type descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/ARCTYPE/, "Arc-Type" on the
// Synth page (displayName).
#include "arc.h"

#include "audio/chip_dsp.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::arc;
namespace chip = ot::audio::chip;

#define OP(level) \
    {1, 0, 15}, {0, -3, 3}, {level, 0, 1}, {1, 0, 3}, {31, 0, 31}, {8, 0, 31}, {4, 0, 15}, {0, 0, 31}, {6, 0, 15}, {0, 0, 4}, {0, 0, 3}
// Init: algorithm 1 > 2 > 3 > 4 with operator 3 at 25 % into the carrier 4
// (a soft, bright sine), no feedback, the 9-bit DAC; every operator Multiple
// 1, Rate Scaling 1, attack 31, first decay 8 to -12 dB, no second decay,
// release 6; no LFO, no Aux.
const PatchParamRange kParams[kParamCount] = {
    {0, 0, 7},          // Algorithm
    {0, 0, 7},          // Feedback
    {1, 0, 2},          // DAC: Off / 9-bit / Ladder
    {0, 0, 3},          // Glide Mode (names::kGlide): Always
    {0, 0, 3},          // Glide Time
    {1, 0, 2},          // Volume
    {0, 0, 8},          // LFO Rate: Off, 8 speeds
    {0, 0, 7},          // PMS
    OP(0.0f),           // OP1: MUL, DT, Level, RS, AR, D1R, D1L, D2R, RR, Loop, AMS
    OP(0.0f),           // OP2
    OP(0.25f),          // OP3
    OP(1.0f),           // OP4
    {0, 0, 2},          // Aux Wave: Off / Square / Noise
    {0.5f, 0, 1},       // Aux Level
    {0, -2, 2},         // Aux Octave
    {4, 0, 15},         // Aux Decay (1/60 s frames a level; 0 holds)
    {0, 0, 11},         // Arp Mode
    {1, 0, 8},          // Arp Rate
    {4, 0, 24},         // Arp Note 2
    {7, 0, 24},         // Arp Note 3
    {0, -24, 24},       // Slide Start
    {0.05f, 0, 1},      // Slide Time
};
#undef OP

const char* const kAlgName[8] = {"1>2>3>4", "(1+2)>3>4", "(1+2>3)>4", "(1>2+3)>4", "1>2, 3>4", "1>(2,3,4)", "1>2, 3, 4", "1, 2, 3, 4"};
const char* const kDacName[3] = {"Off", "9-bit", "Ladder"};
const char* const kMulName[16] = {"0.5", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};
const char* const kLoopName[5] = {"Off", "Repeat", "Mirror", "Hold Inv", "Repeat Inv"};
const char* const kLfoName[9] = {"Off", "4 Hz", "5.5 Hz", "7 Hz", "9 Hz", "12 Hz", "20 Hz", "40 Hz", "72 Hz"};
const char* const kAuxName[3] = {"Off", "Square", "Noise"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
#define OPUI                                                                                                     \
    {"MUL", E(kMulName)}, {"DT", F(kFmtInt)}, {"Level", F(kFmtPct)}, {"RS", F(kFmtInt)}, {"AR", F(kFmtInt)},  \
        {"D1R", F(kFmtInt)}, {"D1L", F(kFmtInt)}, {"D2R", F(kFmtInt)}, {"RR", F(kFmtInt)}, {"Loop", E(kLoopName)}, \
        {"AMS", F(kFmtInt)}
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[kParamCount + 2] = {
    {"Algorithm", E(kAlgName)},   {"Feedback", F(kFmtInt)},      {"DAC", E(kDacName)},
    {"Glide Mode", E(names::kGlide)}, {"Glide Time", F(kFmtSec)}, {"Volume", F(kFmtPct)},
    {"LFO Rate", E(kLfoName)},    {"PMS", F(kFmtInt)},
    OPUI, OPUI, OPUI, OPUI,
    {"Aux", E(kAuxName)},         {"Aux Level", F(kFmtPct)},     {"Aux Octave", F(kFmtInt)},
    {"Aux Decay", F(kFmtInt)},    {"Arp Mode", E(chip::kArpNames)}, {"Arp Rate", E(chip::kRateNames)},
    {"Arp Note 2", F(kFmtSemi)},  {"Arp Note 3", F(kFmtSemi)},   {"Slide Start", F(kFmtSemi)},
    {"Slide Time", F(kFmtSec)},
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef OPUI
#undef E
#undef F

#define OPS(k) {op(k, 0), op(k, 1), op(k, 2), op(k, 3), op(k, 4), op(k, 5), op(k, 6), op(k, 7), op(k, 8), op(k, 9), op(k, 10)}
const uint8_t kOp1[] = OPS(0);
const uint8_t kOp2[] = OPS(1);
const uint8_t kOp3[] = OPS(2);
const uint8_t kOp4[] = OPS(3);
#undef OPS
const uint8_t kGlobal[] = {kAlgorithm, kFeedback, kDac, kGlideMode, kGlideTime, kVolume, uint8_t(kParamCount), uint8_t(kParamCount + 1)};
const uint8_t kLfo[] = {kLfoRate, kPms};
const uint8_t kAux[] = {kAuxWave, kAuxLevel, kAuxOctave, kAuxDecay};
const uint8_t kChip[] = {kArpMode, kArpRate, kArpNote2, kArpNote3, kSlideStart, kSlideTime};

const Section kSections[] = {
    OT_SYNTH_SECTION("GLOBAL", "Global", kGlobal), OT_SYNTH_SECTION("LFO", "LFO", kLfo),
    OT_SYNTH_SECTION("OP1", "Operator 1", kOp1),   OT_SYNTH_SECTION("OP2", "Operator 2", kOp2),
    OT_SYNTH_SECTION("OP3", "Operator 3", kOp3),   OT_SYNTH_SECTION("OP4", "Operator 4", kOp4),
    OT_SYNTH_SECTION("AUX", "Aux Voice", kAux),    OT_SYNTH_SECTION("CHIP", "Chip Tricks", kChip),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kArcDescriptor;
const SynthDescriptor kArcDescriptor = {audio::arc::kEngineId, "ARCTYPE", 0, int8_t(audio::arc::kVolume),
                                       uint8_t(audio::arc::kParamCount), kParams, kUi, kSections,
                                       uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt, "Arc-Type"};

}  // namespace mod
}  // namespace ot
