// FM4 synth descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/FM4/.
#include "fm4.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::synthui::names;

// Init: algorithm 1 (a 4-stack) with only operator 3 modulating the carrier
// operator 4 a little: a soft, slightly bright sine.
#define OP(level, sus) {1, 0, 15}, {0, -3, 3}, {level, 0, 1}, {0, 0, 10}, {0.5f, 0, 10}, {sus, 0, 1}, \
                       {0, 0, 10}, {0.3f, 0, 10}, {0, 0, 1}
const PatchParamRange kParams[audio::fm4::kParamCount] = {
    {0, 0, 7}, {0, 0, 7}, {5, 0, 20}, {0, 0, 1}, {0, 0, 1}, {0, 0, 3}, {1, 0, 2},
    OP(0.0f, 1.0f), OP(0.0f, 1.0f), OP(0.25f, 0.6f), OP(1.0f, 1.0f),
};
#undef OP

const char* const kAlgName[8] = {"1>2>3>4", "(1+2)>3>4", "(1+2>3)>4", "(1>2+3)>4",
                                 "1>2 + 3>4", "1>(2,3,4)", "1>2 + 3 + 4", "1 + 2 + 3 + 4"};
const char* const kMultName[16] = {"0.5", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
#define OPUI {"Mult", E(kMultName)}, {"Detune", F(kFmtInt)}, {"Level", F(kFmtPct)}, {"Attack", F(kFmtSec)}, \
             {"Decay", F(kFmtSec)}, {"Sustain", F(kFmtPct)}, {"Sus Rate", F(kFmtSec)}, {"Release", F(kFmtSec)}, \
             {"Tremolo", F(kFmtOnOff)}
// [3.x] extension parameters (patch field 16): Width and Spread, 0..1, 0 = mono.
const PatchParamRange kExt[2] = {{0.0f, 0.0f, 1.0f, kParamFlagInitAbsent}, {0.0f, 0.0f, 1.0f, kParamFlagInitAbsent | kParamFlagSpread}};

const ParamUi kUi[audio::fm4::kParamCount + 2] = {
    {"Algorithm", E(kAlgName)}, {"Feedback", F(kFmtInt)}, {"LFO Rate", F(kFmtLfoHz)}, {"Vibrato", F(kFmtPct)},
    {"Tremolo", F(kFmtPct)},    {"Glide Time", F(kFmtSec)}, {"Volume", F(kFmtPct)},
    OPUI, OPUI, OPUI, OPUI,
    {"Width", F(kFmtPct)},          {"Spread", F(kFmtPct)},
};
#undef OPUI
#undef E
#undef F

const uint8_t kGlobal[] = {0, 1, 6, 5, uint8_t(audio::fm4::kParamCount), uint8_t(audio::fm4::kParamCount + 1)};
const uint8_t kLfo[] = {2, 3, 4};
#define OPSEC(k) {7 + 9 * k, 8 + 9 * k, 9 + 9 * k, 10 + 9 * k, 11 + 9 * k, 12 + 9 * k, 13 + 9 * k, 14 + 9 * k, 15 + 9 * k}
const uint8_t kOp1[] = OPSEC(0);
const uint8_t kOp2[] = OPSEC(1);
const uint8_t kOp3[] = OPSEC(2);
const uint8_t kOp4[] = OPSEC(3);
#undef OPSEC

const Section kSections[] = {
    OT_SYNTH_SECTION("Global", "Global", kGlobal), OT_SYNTH_SECTION("LFO", "LFO", kLfo),
    OT_SYNTH_SECTION("Op 1", "Operator 1", kOp1),  OT_SYNTH_SECTION("Op 2", "Operator 2", kOp2),
    OT_SYNTH_SECTION("Op 3", "Operator 3", kOp3),  OT_SYNTH_SECTION("Op 4", "Operator 4", kOp4),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kFm4Descriptor;
const SynthDescriptor kFm4Descriptor = {audio::fm4::kEngineId, "FM4", 0, int8_t(audio::fm4::kVolume),
                                       uint8_t(audio::fm4::kParamCount), kParams, kUi, kSections,
                                       uint8_t(sizeof(kSections) / sizeof(kSections[0])), 2, kExt};

}  // namespace mod
}  // namespace ot
