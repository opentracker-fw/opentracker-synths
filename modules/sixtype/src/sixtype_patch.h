// SPDX-License-Identifier: Apache-2.0
// 6-Type patch data [OT]: the voice a 6-Type patch carries and the engine's own
// controls (indices into the patch's params).
//
// A 6-Type patch keeps its 155-byte single voice (sixtype.h) in top-level
// field 100, past every Tracker Mini field (a Mini, which has no such model,
// would skip it). The params are the engine's controls on top of the voice.
#pragma once

#include <cstdint>

#include "model/synth_patch.h"

namespace ot {

constexpr uint32_t kPatchFieldSixtypeVoice = 100;
constexpr int kSixtypeVoiceBytes = 155;
extern const uint8_t kSixtypeInitVoice[kSixtypeVoiceBytes];   // "INIT VOICE"
// The patch's voice into out; false (out = the init voice) when it has none.
bool patchSixtypeVoice(const SynthPatch& p, uint8_t out[kSixtypeVoiceBytes]);
bool patchSetSixtypeVoice(SynthPatch& p, const uint8_t v[kSixtypeVoiceBytes]);

// 6-Type parameters (new ones are appended). All but Volume act on top of the
// voice without changing its data (sixtype.cpp); the Init patch puts M1-M6 on
// Brightness .. Detune (sixtype_desc.cpp).
enum SixtypeParam : int {
    kSixtypeVolume = 0,
    kSixtypeMono,         // polyphony 1: 0 Retrigger (each note restarts), 1 Legato
    kSixtypeBrightness,   // -1..1: modulator levels -/+ 2 doublings (12 dB)
    kSixtypeFeedback,     // -7..7: added to the voice's feedback (0..7)
    kSixtypeAttack,       // -1..1: EG rate 1 of every operator, + = slower
    kSixtypeRelease,      // -1..1: EG rate 4 of every operator, + = longer
    kSixtypeLfoDepth,     // 0..2: x the voice's PMD and AMD
    kSixtypeDetune,       // 0..1: extra detune spread across the operators
    kSixtypeParamCount
};

}  // namespace ot
