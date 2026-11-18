// DX7 patch data [OT]: the voice a DX7 patch carries and the engine's own
// controls (indices into the patch's params).
//
// A DX7 patch keeps its 155-byte single voice (VCED, dx7.h) in top-level
// field 100, past every Tracker Mini field (a Mini, which has no DX7 model,
// would skip it). The params are the engine's controls on top of the voice.
#pragma once

#include <cstdint>

#include "model/synth_patch.h"

namespace ot {

constexpr uint32_t kPatchFieldDx7Voice = 100;
constexpr int kDx7VoiceBytes = 155;
extern const uint8_t kDx7InitVoice[kDx7VoiceBytes];   // the DX7 "INIT VOICE"
// The patch's voice into out; false (out = the init voice) when it has none.
bool patchDx7Voice(const SynthPatch& p, uint8_t out[kDx7VoiceBytes]);
bool patchSetDx7Voice(SynthPatch& p, const uint8_t v[kDx7VoiceBytes]);

// DX7 parameters (new ones are appended). All but Volume act on top of the
// voice without changing its data (dx7.cpp); the Init patch puts M1-M6 on
// Brightness .. Detune (dx7_desc.cpp).
enum Dx7Param : int {
    kDx7Volume = 0,
    kDx7Mono,         // polyphony 1: 0 Retrigger (each note restarts), 1 Legato
    kDx7Brightness,   // -1..1: modulator levels -/+ 2 doublings (12 dB)
    kDx7Feedback,     // -7..7: added to the voice's feedback (0..7)
    kDx7Attack,       // -1..1: EG rate 1 of every operator, + = slower
    kDx7Release,      // -1..1: EG rate 4 of every operator, + = longer
    kDx7LfoDepth,     // 0..2: x the voice's PMD and AMD
    kDx7Detune,       // 0..1: extra detune spread across the operators
    kDx7ParamCount
};

}  // namespace ot
