// V-Type bench (core tests/bench_api.h): one voice per block incl. prepare,
// a glide in progress, at note 96 (the Swarm's wraps grow with the pitch).
#include <cstdio>

#include "audio/synth_engine.h"
#include "bench_api.h"
#include "formats/patch_pb.h"
#include "vtype.h"

namespace {

using namespace ot;
using namespace ot::audio;
namespace vt = ot::audio::vtype;

SynthParams g_p;

void set(int i, float v) { g_p.start[i] = g_p.end[i] = v; }

double runBlocks(int blocks, bool worst) {
    ISynthEngine* e = vt::engine();
    static SynthSlotScratch sc;
    static SynthVoice v;
    static float l[kBlock], r[kBlock];
    ++g_p.serial;
    e->slotReset(sc);
    e->setTempo(120.0f);
    v = SynthVoice{};
    v.slotScratch = &sc;
    v.gate = true;
    SynthNote n;
    n.pitch = worst ? 84.0f : 60.0f;
    e->noteOn(v, g_p, n);
    if (worst) {
        n.pitch = 96.0f;
        n.reused = n.overlap = true;
        e->noteOn(v, g_p, n);
    }
    for (int b = 0; b < 2; ++b) {   // the envelopes past their first block
        e->prepare(sc, g_p);
        e->render(v, g_p, 0.5f, 0.0f, l, r);
    }
    const double t0 = otbench::nowUnits();
    for (int b = 0; b < blocks; ++b) {
        e->prepare(sc, g_p);
        e->render(v, g_p, 0.5f, 0.0f, l, r);
    }
    return (otbench::nowUnits() - t0) / blocks;
}

void init() {
    int count = 0;
    const PatchParamRange* t = patchEngineParams(vt::kEngineId, &count);
    g_p = SynthParams{};
    g_p.engine = vt::kEngineId;
    g_p.count = count + 2;
    for (int i = 0; i < count; ++i) g_p.start[i] = g_p.end[i] = t[i].def;
}

}  // namespace

void vtypeBench() {
    if (!patchEngineParams(vt::kEngineId, nullptr)) return;
    const int blocks = 400 / otbench::scale();
    init();
    set(vt::kCount, 3.0f);
    set(vt::kDetune, 0.7f);
    set(vt::kSubLevel, 0.4f);
    set(vt::kSubWave, float(vt::kSubSine));
    set(vt::kGlideTime, 3.0f);
    const double swarm = runBlocks(blocks, true);
    init();
    const double initPatch = runBlocks(blocks, false);
    std::printf("    vtype.unit.swarm %9.1f  (one voice, 7 saws, sine sub, glide, note 96)\n", swarm);
    std::printf("    vtype.unit.init %9.1f  (the Init patch, limit 16000 M4 insn)\n", initPatch);
}
