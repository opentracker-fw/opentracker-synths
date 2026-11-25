// SPDX-License-Identifier: Apache-2.0
// 6-Type bench (core tests/bench_api.h): one voice per block incl. prepare,
// holding a note. sixtype.unit: the worst-case voice (six operators at
// output level 99, feedback 7, every operator's AMS on with AMD 99, PMD 99,
// PMS 7, the LFO without delay; Brightness, Feedback, LFO Depth and Detune
// at their maximum) on the costliest algorithm (sixtype.scan with
// OT_SIXTYPE_SCAN: all 32 algorithms, then the pitch). Algorithm 4 (the
// three-operator feedback chain, run sample by sample) costs most; the cost
// does not depend on the pitch (flat within 0.1 % from note 12 to 127).
// .alg6 (the two-operator chain), .alg1, .alg32 and .init beside it.
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "audio/synth_engine.h"
#include "bench_api.h"
#include "formats/patch_pb.h"
#include "sixtype.h"
#include "sixtype_patch.h"

namespace {

using namespace ot;
using namespace ot::audio;
namespace st = ot::audio::sixtype;

SynthParams g_p;
uint8_t g_voice[st::kVoiceBytes];

void set(int i, float v) { g_p.start[i] = g_p.end[i] = v; }

void init() {
    int count = 0;
    const PatchParamRange* t = patchEngineParams(st::kEngineId, &count);
    g_p = SynthParams{};
    g_p.engine = st::kEngineId;
    g_p.count = count;
    for (int i = 0; i < count; ++i) g_p.start[i] = g_p.end[i] = t[i].def;
}

// The worst-case voice on `alg` (0..31). AMS 1: the attenuation is computed
// for every operator, but it never takes one below the skip threshold (AMS 3
// at the LFO's peak would, which is cheaper).
void worstVoice(int alg) {
    uint8_t* v = g_voice;
    std::memset(v, 0, st::kVoiceBytes);
    for (int k = 0; k < 6; ++k) {
        uint8_t* o = v + k * 21;
        o[0] = 90; o[1] = 60; o[2] = 50; o[3] = 40;   // rates
        o[4] = 99; o[5] = 95; o[6] = 90; o[7] = 0;    // levels
        o[8] = 39;                                     // break point
        o[14] = 1;                                     // AMS
        o[15] = 0;                                     // velocity sensitivity
        o[16] = 99;                                    // output level
        o[18] = uint8_t(1 + k);                        // ratio 1..6
        o[19] = uint8_t(7 * k);                        // fine
        o[20] = uint8_t(k < 3 ? 10 : 4);               // detune
    }
    for (int i = 0; i < 4; ++i) {
        v[126 + i] = 60;
        v[130 + i] = 50;
    }
    v[126] = 30;      // pitch envelope rate 1: still moving
    v[130] = 70;
    v[134] = uint8_t(alg);
    v[135] = 7;       // feedback
    v[136] = 1;       // key sync
    v[137] = 60;      // LFO speed
    v[138] = 0;       // no delay
    v[139] = 99;      // PMD
    v[140] = 99;      // AMD
    v[141] = 0;
    v[142] = 4;       // sine
    v[143] = 7;       // PMS
    v[144] = 24;
    std::memcpy(v + 145, "BENCH     ", 10);
    init();
    set(kSixtypeBrightness, 1.0f);
    set(kSixtypeFeedback, 7.0f);
    set(kSixtypeLfoDepth, 2.0f);
    set(kSixtypeDetune, 1.0f);
}

// One voice holding `pitch` with g_voice loaded.
double runBlocks(int blocks, float pitch) {
    ISynthEngine* e = st::engine();
    static SynthSlotScratch sc;
    static SynthVoice v;
    static float l[kBlock], r[kBlock];
    ++g_p.serial;
    e->slotReset(sc);
    e->setTempo(120.0f);
    st::setVoice(sc, g_voice);
    v = SynthVoice{};
    v.slotScratch = &sc;
    v.gate = true;
    SynthNote n;
    n.pitch = pitch;
    n.note = uint8_t(pitch);
    n.velocity = 1.0f;
    e->noteOn(v, g_p, n);
    for (int b = 0; b < 4; ++b) {   // the envelopes past their first steps
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

// sixtype.unit's algorithm and pitch: the costliest of sixtype.scan.
constexpr int kWorstAlg = 3;   // algorithm 4
constexpr float kWorstNote = 60.0f;

}  // namespace

void sixtypeBench() {
    if (!patchEngineParams(st::kEngineId, nullptr)) return;
    const int blocks = 400 / otbench::scale();
#if defined(OT_SIXTYPE_SCAN)
    {
        double most = 0.0, least = 1e18;
        int at = 0, cheapest = 0;
        for (int alg = 0; alg < 32; ++alg) {
            worstVoice(alg);
            const double c = runBlocks(16, 60.0f);
            std::printf("    sixtype.scan.alg%02d %9.1f  (not gated)\n", alg + 1, c);
            if (c > most) {
                most = c;
                at = alg;
            }
            if (c < least) {
                least = c;
                cheapest = alg;
            }
        }
        float note = 60.0f;
        for (float n : {12.0f, 24.0f, 36.0f, 48.0f, 72.0f, 84.0f, 96.0f, 108.0f, 120.0f, 127.0f}) {
            worstVoice(at);
            const double c = runBlocks(16, n);
            std::printf("    sixtype.scan.note%.0f %9.1f  (algorithm %d, not gated)\n", double(n), c, at + 1);
            if (c > most) {
                most = c;
                note = n;
            }
        }
        std::printf("    sixtype.scan %9.1f  (costliest: algorithm %d, note %.0f; cheapest algorithm %d %.1f; not gated)\n",
                    most, at + 1, double(note), cheapest + 1, least);
    }
#endif
    worstVoice(kWorstAlg);
    const double worst = runBlocks(blocks, kWorstNote);
    worstVoice(5);
    const double alg6 = runBlocks(blocks, kWorstNote);
    worstVoice(0);
    const double alg1 = runBlocks(blocks, kWorstNote);
    worstVoice(31);
    const double alg32 = runBlocks(blocks, kWorstNote);
    std::memcpy(g_voice, kSixtypeInitVoice, st::kVoiceBytes);
    init();
    const double initPatch = runBlocks(blocks, 60.0f);
    std::printf("    sixtype.unit %9.1f  (one voice, six operators at level 99, algorithm %d, feedback 7, AMS + AMD 99, PMD 99, "
                "Brightness / Feedback / LFO Depth / Detune at max, note %.0f, limit 23000 M4 insn)\n",
                worst, kWorstAlg + 1, double(kWorstNote));
    std::printf("    sixtype.unit.alg6 %9.1f  (as sixtype.unit on algorithm 6: the two-operator chain, limit 23000 M4 insn)\n", alg6);
    std::printf("    sixtype.unit.alg1 %9.1f  (as sixtype.unit on algorithm 1, limit 19000 M4 insn)\n", alg1);
    std::printf("    sixtype.unit.alg32 %9.1f  (as sixtype.unit on algorithm 32: six carriers, limit 19000 M4 insn)\n", alg32);
    std::printf("    sixtype.unit.init %9.1f  (the Init patch: one carrier, note 60, limit 6000 M4 insn)\n", initPatch);
}
