// 1-Type bench (core tests/bench_api.h): one voice per block incl. prepare.
// onetype.unit: the costliest kept settings: FM and sync on triangles, the
// pulse width from LFO 1, the sub, detune, LP24 at Resonance 90 %,
// saturation, the mod envelope on FM, LFO 2 on the filter, a glide in
// progress, note 96. .pulse: the same on pulses; .ring: sync + ring (no FM).
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
    const auto reset = [&] {
        for (int i = 0; i < count; ++i) p.start[i] = p.end[i] = t[i].def;
        ++p.serial;
    };
    auto set = [](int i, float v) { p.start[i] = p.end[i] = v; };
    const int blocks = 800 / otbench::scale();
    reset();
    const double init = runBlocks(p, blocks, false);
    // The costliest kept settings: FM and sync (one frame loop), triangles
    // (polyBLAMP corners), the sub, detune, LP24, the mod envelope on FM, both
    // LFOs routed, a glide.
    const auto worst = [&](int shape1, int shape2, bool ring, int filter) {
        reset();
        set(o::kOsc1Wave, float(shape1));
        set(o::kOsc2Wave, float(shape2));
        set(o::kPulseWidth, 0.4f);
        set(o::kLfo1Dest, float(o::kToPw));
        set(o::kLfo1Wave, float(o::kLfoTri));
        set(o::kLfo1Rate, 3.0f);
        set(o::kLfo1Depth, 0.5f);
        set(o::kOsc2Pitch, 7.0f);
        set(o::kSync, 1.0f);
        set(o::kRing, ring ? 1.0f : 0.0f);
        set(o::kOsc1Fm, 0.4f);
        set(o::kSubLevel, 0.6f);
        set(o::kSubWave, float(o::kSubTri));
        set(o::kOsc1Detune, 0.5f);
        set(o::kMix, 0.5f);
        set(o::kFilterType, float(filter));
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
        return runBlocks(p, blocks, true);
    };
    const double unit = worst(o::kTri, o::kOsc2Tri, false, o::kLp24);
    const double pulse = worst(o::kPulse, o::kOsc2Pulse, false, o::kLp24);
    const double ring = worst(o::kTri, o::kOsc2Tri, true, o::kLp24);
    const double lpdl = worst(o::kTri, o::kOsc2Tri, false, o::kLpdl);
    std::printf("    onetype.unit %9.1f  (one voice, FM + sync on triangles, sub, detune, LP24, mod env, 2 LFOs, glide, limit 45000 M4 insn)\n",
                unit);
    std::printf("    onetype.unit.pulse %9.1f  (the same on pulses)\n", pulse);
    std::printf("    onetype.unit.ring %9.1f  (sync + ring on triangles: no FM)\n", ring);
    std::printf("    onetype.unit.lpdl %9.1f  (the worst case on LPDL: it plays as LP24)\n", lpdl);
    std::printf("    onetype.unit.init %9.1f  (one voice, the Init patch)\n", init);
}
