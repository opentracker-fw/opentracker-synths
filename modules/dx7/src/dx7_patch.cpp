// DX7 patch data (dx7_patch.h) [OT].
#include "dx7_patch.h"

#include <cstring>

#include "modules/synth_descriptor.h"

namespace ot {

// DX7 "INIT VOICE" (VCED, operators 6..1): algorithm 1, only operator 1 sounds
// (a sine at full output level), every envelope at rate 99 / level 99 99 99 0.
const uint8_t kDx7InitVoice[kDx7VoiceBytes] = {
#define OP(out) 99, 99, 99, 99, 99, 99, 99, 0, 39, 0, 0, 0, 0, 0, 0, 0, out, 0, 1, 0, 7
    OP(0), OP(0), OP(0), OP(0), OP(0), OP(99),
#undef OP
    99, 99, 99, 99, 50, 50, 50, 50,      // pitch EG rates, levels
    0, 0, 1,                             // algorithm 1, feedback 0, key sync on
    35, 0, 0, 0, 1, 0, 3,                // LFO speed, delay, PMD, AMD, sync, wave, PMS
    24,                                  // transpose (C3)
    'I', 'N', 'I', 'T', ' ', 'V', 'O', 'I', 'C', 'E',
};

bool patchDx7Voice(const SynthPatch& p, uint8_t out[kDx7VoiceBytes]) {
    const uint8_t* d = nullptr;
    size_t n = 0;
    if (patchOpaqueBytes(p, kPatchFieldDx7Voice, &d, &n) && n == size_t(kDx7VoiceBytes)) {
        std::memcpy(out, d, n);
        return true;
    }
    std::memcpy(out, kDx7InitVoice, kDx7VoiceBytes);
    return false;
}

bool patchSetDx7Voice(SynthPatch& p, const uint8_t v[kDx7VoiceBytes]) {
    return patchSetOpaqueBytes(p, kPatchFieldDx7Voice, v, kDx7VoiceBytes);
}

}  // namespace ot
