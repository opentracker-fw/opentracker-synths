// 1-Type: a two-oscillator virtual analog voice in the style of a classic
// freeware softsynth (OpenTracker 3.x module engine, id 0x8A, module 1TY1,
// Patches/1TYPE/, "1-Type" on the Synth page). A single-precision port of
// the voice of Oxo Synth2 (MIT, see onetype.cpp).
//
// Osc 1 (sine / saw / pulse / triangle, two detuned copies, FM from osc 2,
// a sub oscillator locked to it), osc 2 (saw / pulse / triangle / noise,
// pitch, fine, key track, sync, ring), the mix, the pulse width, a mod
// envelope (osc 2 pitch, FM or pulse width), the filter (LP12, LP24, HP12,
// BP12, LPDL) with its envelope, key track and saturation, the amp
// envelope, two LFOs per slot (seven destinations, tempo and key sync) and
// glide. Engine contract (audio/synth_engine.h): Volume (index 49) is never
// applied here; the voice is mono (L = R), the LFO on Pan acts on the slot
// bus (slotPost).
#pragma once

#include "audio/synth_engine.h"
#include "onetype_params.h"

namespace ot {
namespace audio {
namespace onetype {

constexpr int32_t kEngineId = 0x8A;

ISynthEngine* engine();

}  // namespace onetype
}  // namespace audio
}  // namespace ot
