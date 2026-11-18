// V-Type: a swarm-oscillator virtual analog voice in the style of the late-90s
// desktop VA synths (OpenTracker 3.x module engine 0x8B, module VTY1,
// Patches/VTYPE/, "V-Type" on the Synth page). Our own design:
//
//  - Swarm: 1 / 3 / 5 / 7 detuned polyBLEP saws (Count, Detune, Swarm Mix at
//    constant power), Free or Reset start phases, a sub one octave down
//    (square or sine, locked to the centre saw);
//  - osc 2 (saw / pulse / triangle / sine, Semi, Fine, PW), hard sync to the
//    centre saw, ring (centre saw x osc 2), white noise with a Colour tilt;
//  - two TPT state-variable filters (LP12 / LP24 / BP12 / HP12 each, own
//    cutoff, resonance, Env 1 amount and key track), a Drive stage (Soft /
//    Hard / Fold) after filter 1, four routings: Single, Serial, Parallel and
//    Split (Swarm + sub through filter 1 to the left, osc 2 + noise through
//    filter 2 to the right, Split Width);
//  - three envelopes (Amp, Env 1, Env 2), two LFOs per slot (Hz or a tempo
//    division, Key Sync), a 4-slot modulation matrix, glide.
//
// Engine contract (audio/synth_engine.h): Volume (index 38) is never applied
// here; the voice is mono (L = R) except in Split with a Split Width above 0.
#pragma once

#include "audio/synth_engine.h"
#include "vtype_params.h"

namespace ot {
namespace audio {
namespace vtype {

ISynthEngine* engine();

}  // namespace vtype
}  // namespace audio
}  // namespace ot
