// 1-Type bench (core tests/bench_api.h): one voice per block incl. prepare.
// onetype.unit: the costliest kept settings: both oscillators pulse, the
// pulse width from LFO 1, sync, ring, FM, the sub, detune, LP24 at
// Resonance 90 %, saturation, the mod envelope on FM, LFO 2 on the filter,
// a glide in progress, note 96.
// onetype.unit.init: the Init patch (LP24, no cross terms).
#include <cstdio>

#include "audio/synth_engine.h"
#include "bench_api.h"
#include "formats/patch_pb.h"
#include "onetype.h"

namespace {

double runBlocks(ot::audio::SynthParams& p, int blocks, bool worst) {
    using namespace ot;
    using namespace ot::audio;
    ISynthEngine* e = onetype::engine();
    static SynthSlotScratch sc;
    static SynthVoice v;
    e->slotReset(sc);
    e->setTempo(120.0f);
    v = SynthVoice{};
    v.slotScratch = &sc;
    v.gate = true;
    SynthNote n;
    n.pitch = worst ? 72.0f : 60.0f;
    e->noteOn(v, p, n);
    n.pitch = 96.0f;   // polyBLEP corrections grow with the pitch; reused: a glide up
    n.reused = true;
    n.overlap = true;
    if (worst) e->noteOn(v, p, n);
    static float l[kBlock], r[kBlock];
    const double t0 = otbench::nowUnits();
    for (int b = 0; b < blocks; ++b) {
        e->prepare(sc, p);
        e->render(v, p, 0.5f, 0.0f, l, r);
    }
    return (otbench::nowUnits() - t0) / blocks;
}

}  // namespace

void onetypeBench() {
    using namespace ot;
    using namespace ot::audio;
    namespace o = ot::audio::onetype;
    static SynthParams p;
    int count = 0;
    const PatchParamRange* t = patchEngineParams(o::kEngineId, &count);
    if (!t) return;
    p.engine = o::kEngineId;
    p.count = count;
    for (int i = 0; i < count; ++i) p.start[i] = p.end[i] = t[i].def;
    p.serial = 1;
    const int blocks = 800 / otbench::scale();
    const double init = runBlocks(p, blocks, false);
    auto set = [](int i, float v) { p.start[i] = p.end[i] = v; };
    set(o::kOsc1Wave, float(o::kPulse));
    set(o::kOsc2Wave, float(o::kOsc2Pulse));
    set(o::kPulseWidth, 0.4f);
    set(o::kLfo1Dest, float(o::kToPw));
    set(o::kLfo1Wave, float(o::kLfoTri));
    set(o::kLfo1Rate, 3.0f);
    set(o::kLfo1Depth, 0.5f);
    set(o::kOsc2Pitch, 7.0f);
    set(o::kSync, 1.0f);
    set(o::kRing, 1.0f);
    set(o::kOsc1Fm, 0.4f);
    set(o::kSubLevel, 0.6f);
    set(o::kSubWave, float(o::kSubSquare));
    set(o::kOsc1Detune, 0.5f);
    set(o::kMix, 0.5f);
    set(o::kFilterType, float(o::kLp24));
    set(o::kResonance, 0.9f);
    set(o::kSaturation, 0.5f);
    set(o::kModAmount, 0.6f);
    set(o::kModDest, float(o::kModFm));
    set(o::kModDecay, 2.0f);
    set(o::kLfo2Dest, float(o::kToFilter));
    set(o::kLfo2Wave, float(o::kLfoSine));
    set(o::kLfo2Rate, 2.0f);
    set(o::kLfo2Depth, 0.5f);
    set(o::kGlideMode, float(o::kAlways));
    set(o::kGlideTime, 3.0f);
    ++p.serial;
    const double worst = runBlocks(p, blocks, true);
    std::printf("    onetype.unit %9.1f  (one voice, 2 pulse oscs, PW LFO, sync, ring, FM, sub, detune, LP24, glide, limit 36000 M4 insn)\n",
                worst);
    std::printf("    onetype.unit.init %9.1f  (one voice, the Init patch)\n", init);
}
