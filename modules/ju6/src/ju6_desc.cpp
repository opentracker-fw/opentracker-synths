// JU6 synth descriptor (synth_descriptor.h): stored ranges, Edit Patch labels,
// formats, enum names and sections. Patches/JU6/.
#include "ju6.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;
using namespace ot::audio::ju6;

// Init: saw + a little sub through a half-open filter with a positive
// envelope, slow-ish attack, LFO pulse width ready for the pulse: a soft
// brass pad.
const PatchParamRange kParams[kParamCount] = {
    {1, 0, 1},          // Saw
    {0, 0, 1},          // Pulse
    {0.4f, 0, 1},       // PW (manual width, or PWM depth in LFO mode)
    {1, 0, 1},          // PWM: Manual / LFO
    {0.35f, 0, 1},      // Sub
    {0, 0, 1},          // Noise
    {0, 0, 1},          // Vibrato (DCO LFO)
    {1, 0, 3},          // HPF 0..3
    {1400, 20, 20000},  // Cutoff (Hz)
    {0.15f, 0, 1},      // Resonance
    {0.4f, -1, 1},      // Env (bipolar: negative = inverted)
    {0, 0, 1},          // VCF LFO
    {0.5f, 0, 1},       // Key follow
    {0, 0, 1},          // Velocity -> VCF
    {0, 0, 1},          // VCA: Env / Gate
    {0.06f, 0, 10},     // Attack
    {1.0f, 0, 10},      // Decay
    {0.65f, 0, 1},      // Sustain
    {0.45f, 0, 10},     // Release
    {3.5f, 0.1f, 20},   // LFO Rate (Hz)
    {0.6f, 0, 5},       // LFO Delay
    {1, 0, 2},          // Volume
    {0, 0, 3},          // Chorus: Off / I / II / I+II (absent in older patches: Off)
};

const char* const kPwmName[2] = {"Manual", "LFO"};
const char* const kHpfName[4] = {"0 Boost", "1 Flat", "2", "3"};
const char* const kVcaName[2] = {"Env", "Gate"};
const char* const kChorusName[4] = {"Off", "I", "II", "I+II"};

#define E(n) OT_SYNTH_ENUM(n)
#define F(f) f, nullptr, 0
const ParamUi kUi[kParamCount] = {
    {"Saw", F(kFmtOnOff)},       {"Pulse", F(kFmtOnOff)},      {"PW", F(kFmtPct)},
    {"PWM", E(kPwmName)},        {"Sub", F(kFmtPct)},          {"Noise", F(kFmtPct)},
    {"Vibrato", F(kFmtPct)},     {"HPF", E(kHpfName)},         {"Cutoff", F(kFmtHz)},
    {"Resonance", F(kFmtPct)},   {"Env", F(kFmtPctMax)},       {"LFO", F(kFmtPct)},
    {"Key Follow", F(kFmtPct)},  {"Velocity", F(kFmtPct)},     {"VCA", E(kVcaName)},
    {"Attack", F(kFmtSec)},      {"Decay", F(kFmtSec)},        {"Sustain", F(kFmtPct)},
    {"Release", F(kFmtSec)},     {"LFO Rate", F(kFmtLfoHz)},   {"LFO Delay", F(kFmtSec)},
    {"Volume", F(kFmtPct)},      {"Chorus", E(kChorusName)},
};
#undef E
#undef F

const uint8_t kDco[] = {kSaw, kPulse, kPw, kPwMode, kSub, kNoise, kVibrato};
const uint8_t kVcf[] = {kHpf, kCutoff, kResonance, kEnvAmt, kVcfLfo, kKeyFollow, kVelVcf};
const uint8_t kVca[] = {kVcaMode, kVolume};
const uint8_t kEnv[] = {kAttack, kDecay, kSustain, kRelease};
const uint8_t kLfo[] = {kLfoRate, kLfoDelay};
const uint8_t kCho[] = {kChorus};

const Section kSections[] = {
    OT_SYNTH_SECTION("DCO", "Oscillator", kDco), OT_SYNTH_SECTION("VCF", "HPF / VCF", kVcf),
    OT_SYNTH_SECTION("VCA", "Amplifier", kVca),  OT_SYNTH_SECTION("ENV", "Envelope", kEnv),
    OT_SYNTH_SECTION("LFO", "LFO", kLfo),        OT_SYNTH_SECTION("CHO", "Chorus", kCho),
    OT_SYNTH_MACRO_SECTIONS};

}  // namespace

extern const SynthDescriptor kJu6Descriptor;
const SynthDescriptor kJu6Descriptor = {audio::ju6::kEngineId, "JU6", 0, int8_t(audio::ju6::kVolume),
                                       uint8_t(audio::ju6::kParamCount), kParams, kUi, kSections,
                                       uint8_t(sizeof(kSections) / sizeof(kSections[0])), 0, nullptr};

}  // namespace mod
}  // namespace ot
