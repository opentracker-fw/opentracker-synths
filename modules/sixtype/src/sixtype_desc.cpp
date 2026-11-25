// 6-Type synth descriptor (modules/synth_descriptor.h) [OT]: the controls on top
// of the loaded voice, the Init patch's voice and macros, and the .syx banks
// in Load Patch. Patches/6TYPE/, "6-Type" on the Synth page.
#include <cstring>

#include "sixtype.h"
#include "sixtype_patch.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

namespace {

using namespace ot::synthui;

const PatchParamRange kParams[kSixtypeParamCount] = {
    {1, 0, 2},     // 0 Volume
    {0, 0, 1},     // 1 Mono: Retrigger / Legato
    {0, -1, 1},    // 2 Brightness
    {0, -7, 7},    // 3 Feedback
    {0, -1, 1},    // 4 Attack
    {0, -1, 1},    // 5 Release
    {1, 0, 2},     // 6 LFO Depth
    {0, 0, 1},     // 7 Detune
};

const char* const kMonoNames[2] = {"Retrigger", "Legato"};
#define E(n) OT_SYNTH_ENUM(n)
#define F(f) OT_SYNTH_FMT(f)
const ParamUi kUi[kSixtypeParamCount] = {
    {"Volume", F(kFmtPct)},  {"Mono", E(kMonoNames)},  {"Brightness", F(kFmtPct)}, {"Feedback", F(kFmtInt)},
    {"Attack", F(kFmtPct)},  {"Release", F(kFmtPct)}, {"LFO Depth", F(kFmtPct)},  {"Detune", F(kFmtPct)},
};
#undef E
#undef F

const uint8_t kVoice[] = {kSixtypeVolume, kSixtypeMono};
const uint8_t kTone[] = {kSixtypeBrightness, kSixtypeFeedback, kSixtypeAttack, kSixtypeRelease, kSixtypeLfoDepth, kSixtypeDetune};
const Section kSections[] = {OT_SYNTH_SECTION("Voice", "Voice", kVoice), OT_SYNTH_SECTION("Tone", "Tone", kTone),
                             OT_SYNTH_MACRO_SECTIONS};

void setName(PatchMacro& m, const char* s) {
    const size_t l = std::strlen(s);
    m.nameLen = uint8_t(l);
    std::memcpy(m.name, s, l + 1);
}

// The init voice; no operator editor, so the macros come aimed at the
// engine's controls. Amount 0.5 x the parameter's span = its full range for
// a bipolar macro (-1..1); 1 for the unipolar Detune (0..1).
void initPatch(SynthPatch& p) {
    patchSetSixtypeVoice(p, kSixtypeInitVoice);
    struct M {
        const char* name;
        int param;
        bool bipolar;
        float amount;
    };
    // Names fit the Synth Macros F-key cells with their "Mn " prefix
    // (12 characters: "M1 Brightness" was cut).
    static const M kM[6] = {{"Bright", kSixtypeBrightness, true, 0.5f}, {"Feedback", kSixtypeFeedback, true, 0.5f},
                            {"Attack", kSixtypeAttack, true, 0.5f},     {"Release", kSixtypeRelease, true, 0.5f},
                            {"LFO Depth", kSixtypeLfoDepth, true, 0.5f}, {"Detune", kSixtypeDetune, false, 1.0f}};
    for (int k = 0; k < 6; ++k) {
        PatchMacro& m = p.macros[k];
        setName(m, kM[k].name);
        m.bipolar = kM[k].bipolar;
        m.count = 1;
        m.targets[0].param = kM[k].param;
        std::memcpy(&m.targets[0].amountBits, &kM[k].amount, 4);
    }
    // Velocity works only through each operator's velocity
    // sensitivity (the engine gets the note velocity). M7 -> Volume on top
    // counted it twice (-22 dB instead of -16 dB at Volume FX 50 on a
    // sensitivity-7 voice). M7 stays named "Velocity", unassigned.
    p.macros[kPatchMacroVelocity].count = 0;
    p.macros[kPatchMacroVelocity].targets[0] = PatchTarget{0, 0u};
}

// .syx files: a 32-voice bank (4104 bytes) or a single voice (163 bytes).
int importCount(const uint8_t* file, uint32_t len) { return audio::sixtype::sysexVoiceCount(file, len); }

void importName(const uint8_t* file, uint32_t len, int k, char* out, int cap) {
    out[0] = 0;
    uint8_t v[audio::sixtype::kVoiceBytes];
    if (!audio::sixtype::sysexVoice(file, len, k, v)) return;
    const uint8_t* nm = v + audio::sixtype::kVoiceBytes - audio::sixtype::kNameLen;
    int n = 0;
    for (int i = 0; i < audio::sixtype::kNameLen && n < cap - 1; ++i) out[n++] = char(nm[i]);
    while (n > 0 && out[n - 1] == ' ') --n;
    out[n] = 0;
}

bool importPatch(const uint8_t* file, uint32_t len, int k, SynthPatch& p) {
    uint8_t v[audio::sixtype::kVoiceBytes];
    return audio::sixtype::sysexVoice(file, len, k, v) && patchSetSixtypeVoice(p, v);
}

// A bank dump: F0 43 0n 09 20 00, 32 x 128, checksum, F7. Past 8 KB a file
// is no voice dump.
const PatchImport kImport = {".syx", 8192, 6 + 4096 + 2, importCount, importName, importPatch};

}  // namespace

extern const SynthDescriptor kSixtypeDescriptor;
const SynthDescriptor kSixtypeDescriptor = {audio::sixtype::kEngineId, "6TYPE", 0, int8_t(kSixtypeVolume), uint8_t(kSixtypeParamCount),
                                        kParams, kUi, kSections, uint8_t(sizeof(kSections) / sizeof(kSections[0])),
                                        0, nullptr, "6-Type", -1, initPatch, &kImport};

}  // namespace mod
}  // namespace ot
