// 1-Type parameter indices and enum values (no includes: the host-only
// reference tool and the .sy1 mapping use it too). Stored values are
// engineering units (Hz, seconds, semitones, cents, levels 0..1); the knob
// laws of the classic softsynth this engine is styled after live in the
// .sy1 mapping, not here.
//
// Derived from Oxo Synth2 (https://github.com/chab/oxo_synth2, commit 6bb4435):
//
//   MIT License
//   Copyright (c) 2026 Francois Chabloz and contributors
//
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files (the "Software"), to deal
//   in the Software without restriction, including without limitation the rights
//   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//   copies of the Software, and to permit persons to whom the Software is
//   furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//   SOFTWARE.
#pragma once

namespace ot {
namespace audio {
namespace onetype {

enum Param : int {
    kOsc1Wave = 0, kOsc1Detune, kOsc1Fm, kSubLevel, kSubWave, kSubOctave,         // OSC1
    kOsc2Wave, kOsc2Pitch, kOsc2Fine, kOsc2Track, kSync, kRing,                    // OSC2
    kMix, kPulseWidth, kKeyShift, kFineTune, kPhase,                               // MIX
    kModAmount, kModAttack, kModDecay, kModDest,                                   // MOD ENV
    kFilterType, kCutoff, kResonance, kEnvAmt, kKeyTrack, kSaturation,             // VCF
    kFAttack, kFDecay, kFSustain, kFRelease,
    kAttack, kDecay, kSustain, kRelease,                                           // AMP
    kLfo1Dest, kLfo1Wave, kLfo1Rate, kLfo1Depth, kLfo1Sync, kLfo1KeySync,          // LFO1
    kLfo2Dest, kLfo2Wave, kLfo2Rate, kLfo2Depth, kLfo2Sync, kLfo2KeySync,          // LFO2
    kGlideMode, kGlideTime, kVolume,                                               // AMP (cont.)
    kParamCount,   // 50
};
// LFO k's parameters: kLfo1Dest + k * kLfoStride + (field).
constexpr int kLfoStride = kLfo2Dest - kLfo1Dest;
enum LfoField : int { kLfoDest = 0, kLfoWave, kLfoRate, kLfoDepth, kLfoSync, kLfoKeySync };

enum Osc1Wave : int { kSine = 0, kSaw, kPulse, kTri };
enum SubWave : int { kSubSine = 0, kSubTri, kSubSaw, kSubSquare };
enum Osc2Wave : int { kOsc2Saw = 0, kOsc2Pulse, kOsc2Tri, kOsc2Noise };
enum ModDest : int { kModOsc2Pitch = 0, kModFm, kModPw };
enum FilterType : int { kLp12 = 0, kLp24, kHp12, kBp12, kLpdl };
enum LfoDest : int { kToOsc2 = 0, kToOsc12, kToFilter, kToAmp, kToPw, kToFm, kToPan };
enum LfoWave : int { kLfoSaw = 0, kLfoTri, kLfoSquare, kLfoSh, kLfoSmooth, kLfoSine };
enum GlideMode : int { kAlways = 0, kOverlap, kLegato, kLegatoOverlap };

constexpr int kLfoSyncSteps = 30;   // tempo divisions (Sync 1..30; 0 = Off)

}  // namespace onetype
}  // namespace audio
}  // namespace ot
