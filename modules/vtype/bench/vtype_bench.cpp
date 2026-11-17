// V-Type bench (core tests/bench_api.h): one voice per block incl. prepare,
// a glide in progress, at note 96 (the Swarm's wraps grow with the pitch).
// vtype.unit: the costliest setting (vtype.scan with OT_VTYPE_SCAN: every
// routing x filter type pair x curve x osc 2 wave x Sync x sub wave on top
// of the worst modulation); .parallel, .split, .fold, .pad, .init beside it.
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

// The worst modulation and sources (everything but the path's switches).
void worstBase() {
    init();
    set(vt::kCount, 3.0f);
    set(vt::kDetune, 0.7f);
    set(vt::kSubLevel, 0.4f);
    set(vt::kSubWave, float(vt::kSubSine));
    set(vt::kO2Wave, float(vt::kO2Pulse));
    set(vt::kO2Sync, 1.0f);
    set(vt::kO2Semi, 7.0f);
    set(vt::kRing, 0.3f);
    set(vt::kBalance, 0.0f);
    set(vt::kNoise, 0.3f);
    set(vt::kColour, -0.4f);
    set(vt::kRouting, float(vt::kSerial));
    set(vt::kFltBalance, 0.3f);
    set(vt::kF1Type, float(vt::kLp24));
    set(vt::kF2Type, float(vt::kLp24));
    set(vt::kF1Reso, 0.9f);
    set(vt::kF2Reso, 0.9f);
    set(vt::kF2Cutoff, 3000.0f);
    set(vt::kF2Env, 0.3f);
    set(vt::kDrive, 0.5f);
    set(vt::kCurve, float(vt::kSoft));
    set(vt::kGlideTime, 3.0f);
    set(vt::kSplitWidth, 0.7f);
    set(vt::kL1Rate, 7.0f);
    set(vt::kL2Rate, 0.5f);
    set(vt::kL2Wave, float(vt::kSampleHold));
    const int src[4] = {vt::kSrcLfo1, vt::kSrcEnv2, vt::kSrcLfo2, vt::kSrcEnv1};
    const int dst[4] = {vt::kDstPw, vt::kDstO2Pitch, vt::kDstCutoff12, vt::kDstDrive};
    for (int k = 0; k < 4; ++k) {
        set(vt::modSrc(k), float(src[k]));
        set(vt::modDest(k), float(dst[k]));
        set(vt::modAmt(k), 0.3f);
    }
}

// The path's switches.
struct Path {
    int route, t1, t2, curve, wave, sync, sub;
};
void path(const Path& s) {
    set(vt::kRouting, float(s.route));
    set(vt::kF1Type, float(s.t1));
    set(vt::kF2Type, float(s.t2));
    set(vt::kCurve, float(s.curve));
    set(vt::kO2Wave, float(s.wave));
    set(vt::kO2Sync, float(s.sync));
    set(vt::kSubWave, float(s.sub));
    set(vt::kFltBalance, s.route == vt::kSerial ? 0.3f : 0.0f);
}

// vtype.unit's path: the costliest of vtype.scan (38,780 over 8 blocks):
// Split, LP24 and LP24, Fold, a synced pulse, the sine sub.
constexpr Path kWorst = {vt::kSplit, vt::kLp24, vt::kLp24, vt::kFold, vt::kO2Pulse, 1, vt::kSubSine};

}  // namespace

void vtypeBench() {
    if (!patchEngineParams(vt::kEngineId, nullptr)) return;
    const int blocks = 400 / otbench::scale();
#if defined(OT_VTYPE_SCAN)
    {
        double most = 0.0;
        Path at{};
        for (int route = 0; route < 4; ++route)
            for (int t1 = 0; t1 < 4; ++t1)
                for (int t2 = 0; t2 < 4; ++t2)
                    for (int curve = 0; curve < 3; ++curve)
                        for (int wave = 0; wave < 4; ++wave)
                            for (int sync = 0; sync < 2; ++sync)
                                for (int sub = 0; sub < 2; ++sub) {
                                    const Path s{route, t1, t2, curve, wave, sync, sub};
                                    worstBase();
                                    path(s);
                                    const double c = runBlocks(8, true);
                                    if (c > most) {
                                        most = c;
                                        at = s;
                                    }
                                }
        std::printf("    vtype.scan %9.1f  (costliest: routing %d, types %d / %d, curve %d, osc 2 wave %d, sync %d, sub %d; not gated)\n",
                    most, at.route, at.t1, at.t2, at.curve, at.wave, at.sync, at.sub);
    }
#endif
    worstBase();
    path(kWorst);
    const double worst = runBlocks(blocks, true);
    worstBase();
    path(Path{vt::kParallel, vt::kLp24, vt::kLp24, vt::kFold, vt::kO2Pulse, 1, vt::kSubSine});
    const double par = runBlocks(blocks, true);
    worstBase();
    path(Path{vt::kSplit, vt::kLp24, vt::kLp24, vt::kSoft, vt::kO2Pulse, 1, vt::kSubSine});
    const double split = runBlocks(blocks, true);
    worstBase();
    path(Path{vt::kSerial, vt::kLp24, vt::kLp24, vt::kFold, vt::kO2Pulse, 1, vt::kSubSine});
    const double fold = runBlocks(blocks, true);
    // A pad: 7 saws, osc 2 saw, Serial LP24 -> HP12, Soft drive, one LFO.
    init();
    set(vt::kCount, 3.0f);
    set(vt::kO2Wave, float(vt::kO2Saw));
    set(vt::kBalance, -0.3f);
    set(vt::kRouting, float(vt::kSerial));
    set(vt::kF2Type, float(vt::kHp12));
    set(vt::kDrive, 0.3f);
    set(vt::modSrc(0), float(vt::kSrcLfo2));
    set(vt::modDest(0), float(vt::kDstCutoff1));
    set(vt::modAmt(0), 0.2f);
    const double pad = runBlocks(blocks, false);
    init();
    const double initPatch = runBlocks(blocks, false);
    std::printf("    vtype.unit %9.1f  (one voice, 7 saws, sine sub, synced pulse with PW from LFO 1, ring, coloured noise, Split LP24 / LP24 at 0.9, Fold drive, 4 matrix slots, glide, note 96, limit 40000 M4 insn)\n",
                worst);
    std::printf("    vtype.unit.parallel %9.1f  (as vtype.unit, Parallel, limit 40000 M4 insn)\n", par);
    std::printf("    vtype.unit.split %9.1f  (as vtype.unit, Soft drive, limit 40000 M4 insn)\n", split);
    std::printf("    vtype.unit.fold %9.1f  (as vtype.unit, Serial LP24 -> LP24, limit 40000 M4 insn)\n", fold);
    std::printf("    vtype.unit.pad %9.1f  (7 saws, osc 2 saw, Serial LP24 -> HP12, Soft drive, one LFO, limit 28000 M4 insn)\n", pad);
    std::printf("    vtype.unit.init %9.1f  (the Init patch: 5 saws, Single LP24, limit 16000 M4 insn)\n", initPatch);
}
