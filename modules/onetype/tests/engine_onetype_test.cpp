// 1-Type engine (opentracker-synths modules/onetype, engine 0x8A): registered
// with a valid descriptor ("1-Type" on screen, Patches/1TYPE/), its patch
// file round trip, silent when idle; then the reference features
// (onetype_ref.inc: the source engine's measurements, tools/ref) against
// the same features of this engine: pitch, harmonics, the mix law, noise,
// the amp envelope; the filter responses, the LP24 self-oscillation, the
// limiter and saturation, the filter envelope depth and key track; FM, ring
// and sync, detune, the mod envelope, the LFOs (waves, destinations, both on
// one, rates and tempo sync), glide. And the engine cases kept from the
// source's own tests (portamento, mono retrigger, a stolen voice, the voice
// end, the random LFO restart), the Volume contract, the voice level, no
// NaN or denormals at the extremes, every parameter audible.
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
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "audio/governor.h"
#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"
#include "onetype.h"
#include "onetype_dsp.h"
#include "onetype_features.h"
#include "onetype_ref.inc"

using namespace ot;
using namespace ot::audio;
namespace rf = onetype_ref;

namespace {

constexpr float kFs = ot::audio::kSampleRate;   // (model/project.h has a kSampleRate too)
constexpr int kN = onetype::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;

const PatchParamRange* range(int i) { return synthParamRange(onetype::kEngineId, i); }
bool verbose() { return std::getenv("ONETYPE_VERBOSE") != nullptr; }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[size_t(i)] = range(i)->def;
    return v;
}

// One engine voice pool driven directly (no framework).
struct Rig {
    ISynthEngine* e = synthEngineFor(onetype::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    std::vector<float> L, R;

    explicit Rig(const std::vector<float>& vals, float tempo = 120.0f) {
        p.engine = onetype::kEngineId;
        p.count = kN;
        set(vals);
        e->slotReset(scratch);
        e->setTempo(tempo);
        for (auto& x : v) {
            x = SynthVoice{};
            x.slotScratch = &scratch;
        }
    }
    void set(const std::vector<float>& vals) {
        for (int i = 0; i < kN; ++i) p.start[i] = p.end[i] = vals[size_t(i)];
        ++p.serial;
    }
    void on(int k, float pitch, bool reused = false, bool overlap = false) {
        if (!reused) {
            std::memset(v[k].state, 0, sizeof(v[k].state));
            v[k].level = 0.0f;
        }
        v[k].gate = true;
        SynthNote n;
        n.pitch = pitch;
        n.note = uint8_t(std::clamp(int(pitch), 0, 127));
        n.reused = reused;
        n.overlap = overlap;
        int held = 0;
        for (int j = 0; j < kSynthMaxVoices; ++j) held += (alive[j] && v[j].gate) || j == k;
        p.held.count = uint8_t(held);
        p.held.mono = 1;
        e->noteOn(v[k], p, n);
        alive[k] = true;
    }
    void off(int k) {
        v[k].gate = false;
        e->noteOff(v[k], p);
    }
    // Render n blocks; appends to L / R. Returns the number of live voices.
    int run(int blocks, float gain = 1.0f) {
        int live = 0;
        for (int b = 0; b < blocks; ++b) {
            float l[kBlock] = {}, r[kBlock] = {};
            e->prepare(scratch, p);
            live = 0;
            for (int k = 0; k < kSynthMaxVoices; ++k) {
                if (!alive[k]) continue;
                alive[k] = e->render(v[k], p, gain, 0.0f, l, r);
                live += alive[k];
            }
            e->slotPost(0, scratch, p, l, r);
            L.insert(L.end(), l, l + kBlock);
            R.insert(R.end(), r, r + kBlock);
        }
        return live;
    }
};

using Buf = std::vector<float>;

float peak(const Buf& x) {
    float m = 0.0f;
    for (float s : x) m = std::max(m, std::fabs(s));
    return m;
}
double rms(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    b = std::min(b, x.size());
    double s = 0.0;
    for (size_t i = a; i < b; ++i) s += double(x[i]) * x[i];
    return b > a ? std::sqrt(s / double(b - a)) : 0.0;
}
double hz(double pitch) { return 440.0 * std::pow(2.0, (pitch - 69.0) / 12.0); }
double cents(double f, double ref) { return f > 0 && ref > 0 ? 1200.0 * std::log2(f / ref) : 1e9; }

// --- the reference cases -------------------------------------------------------------

std::vector<float> caseValues(const rf::Case& c) {
    std::vector<float> v(rf::kBase, rf::kBase + kN);
    for (int k = 0; k < c.nset; ++k) v[size_t(c.set[k].i)] = c.set[k].v;
    return v;
}

// The case through this engine: one voice, key at block 0, key 2 on the same
// voice (overlapping), the note off; returns the features.
int renderCase(const rf::Case& c, float* out) {
    Rig r(caseValues(c), c.tempo);
    r.on(0, float(c.key));
    for (int b = 0; b < c.blocks; ++b) {
        if (b == c.b2) r.on(0, float(c.key2), true, r.v[0].gate);
        if (b == c.bOff) r.off(0);
        r.run(1);
    }
    return rf::voiceFeatures(c, r.L, r.R, kFs, out);
}

// The filter section as a per-sample step (one tick of latency: the section
// runs a tick at a time).
struct SectionStep {
    onetype::dsp::Section s;
    float in[onetype::dsp::kTick] = {}, out[onetype::dsp::kTick] = {};
    int n = 0;
    SectionStep(int type, float hzv, float res, float sat) { s.init(type, hzv, res, sat); }
    double operator()(double x) {
        const double y = out[n];
        in[n++] = float(x);
        if (n == onetype::dsp::kTick) {
            s.run(in, out);
            n = 0;
        }
        return y;
    }
};
int kernelCase(const rf::Case& c, float* out) {
    const float* a = c.arg;
    switch (c.kind) {
        case rf::kFiltIr: {
            SectionStep s(int(a[0]), a[1], a[2], 0.0f);
            return rf::filterIrFeatures(c, s, kFs, out);
        }
        case rf::kSelfOsc: {
            SectionStep s(int(a[0]), a[1], a[2], 0.0f);
            return rf::selfOscFeatures(s, kFs, out);
        }
        case rf::kLimSat:
            return rf::limSatFeatures(c, [&] { return SectionStep(onetype::kLp12, 16.7f * std::exp2(127.0f / 12.624f), 0.0f, a[0]); }, kFs,
                                      out);
        default: return 0;
    }
}

const rf::Case* findCase(const char* name) {
    for (const rf::Case& c : rf::kCases)
        if (std::strcmp(c.name, name) == 0) return &c;
    return nullptr;
}

// Ours minus the source's level of the reference case (osc 1 sine, key 60, h1).
double levelOffset() {
    static double off = 1e9;
    if (off < 1e8) return off;
    const rf::Case* c = findCase("harm_osc1_sine_k60");
    float o[rf::kMaxExp] = {};
    renderCase(*c, o);
    off = double(o[0]) - double(c->exp[0]);
    return off;
}

bool startsWith(const char* s, const char* p) { return std::strncmp(s, p, std::strlen(p)) == 0; }

// Compares every case whose name starts with `prefix` by `cmp(case, ours, k)`
// -> error in units of its tolerance (|e| <= 1 passes). Prints the worst.
template <class Cmp>
void compareCases(const char* prefix, Cmp cmp, const char* skip = nullptr) {
    double worst = 0;
    const char* worstName = "";
    int worstK = -1, cases = 0;
    for (const rf::Case& c : rf::kCases) {
        if (!startsWith(c.name, prefix) || (skip && startsWith(c.name, skip))) continue;
        float o[rf::kMaxExp] = {};
        const int n = c.kind >= rf::kFiltIr ? kernelCase(c, o) : renderCase(c, o);
        CHECK_EQ(n, int(c.nexp));
        ++cases;
        for (int k = 0; k < std::min(n, int(c.nexp)); ++k) {
            const double e = cmp(c, o, k);
            if (verbose()) std::printf("    %s[%d]: ref %.6g ours %.6g (%.2f)\n", c.name, k, double(c.exp[k]), double(o[k]), e);
            CHECK(std::isfinite(e));
            if (!(std::fabs(e) <= worst) || !std::isfinite(e)) {
                worst = std::isfinite(e) ? std::fabs(e) : 1e9;
                worstName = c.name;
                worstK = k;
            }
        }
    }
    CHECK(cases > 0);
    std::printf("  %s*: %d cases, worst %.2f of its tolerance (%s[%d])\n", prefix, cases, worst, worstName, worstK);
    CHECK(worst <= 1.0);
}

// Levels in dB after the reference offset; values more than `floor` dB under
// the case's loudest are ignored (the source's alias or numerical floor).
double levelErr(const rf::Case& c, const float* o, int k, double tol, double floor = 50.0, int group = 0, int stride = 0) {
    double top = -1e9;
    const int a = stride ? (k / stride) * stride : 0, b = stride ? a + stride : int(c.nexp);
    for (int j = a; j < b; ++j) top = std::max(top, double(c.exp[j]));
    (void)group;
    if (c.exp[k] < top - floor) return 0.0;
    return (double(o[k]) - levelOffset() - double(c.exp[k])) / tol;
}

}  // namespace

// --- the descriptor and the patch file ----------------------------------------------------

TEST(onetype_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(onetype::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "1TYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(onetype::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(onetype::kEngineId));
    CHECK_STR(d->name, "1TYPE");
    CHECK_STR(d->displayName, "1-Type");
    CHECK_EQ(int(d->paramCount), 50);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(onetype::kVolume));
    CHECK_EQ(patchVolumeParam(onetype::kEngineId), int(onetype::kVolume));
    CHECK_STR(patchEngineName(onetype::kEngineId), "1TYPE");
    CHECK_EQ(int(d->sectionCount), 15);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    CHECK(d->ext[0].flags == kParamFlagInitAbsent);
    CHECK(d->ext[1].flags & kParamFlagSpread);
    CHECK_STR(synthEngineFolder(onetype::kEngineId), "1TYPE");
    CHECK_STR(synthEngineDisplayName(onetype::kEngineId), "1-Type");
    const int k = synthModelListIndex(onetype::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "1-Type");
    std::vector<int> seen(kN, 0);
    int macros = 0;
    for (int s = 0; s < d->sectionCount; ++s) {
        const synthui::Section& sec = d->sections[s];
        if (sec.macro >= 0) {
            CHECK_EQ(int(sec.macro), macros);
            ++macros;
            continue;
        }
        CHECK_EQ(macros, 0);
        for (int j = 0; j < sec.count; ++j) {
            CHECK(sec.params[j] < kN + 2);   // + Width, Spread
            if (sec.params[j] < kN) ++seen[sec.params[j]];
        }
    }
    CHECK_EQ(macros, 7);
    for (int i = 0; i < kN; ++i) {
        const PatchParamRange& r = d->params[i];
        CHECK(r.min < r.max && r.def >= r.min && r.def <= r.max);
        CHECK(d->ui[i].label != nullptr && std::strlen(d->ui[i].label) > 0);
        if (d->ui[i].fmt == synthui::kFmtEnum) CHECK_EQ(int(d->ui[i].nameCount), int(r.max - r.min) + 1);
        if (d->ui[i].fmt == synthui::kFmtOnOff) CHECK(r.min == 0 && r.max == 1);
        CHECK_EQ(seen[size_t(i)], 1);
    }
    SynthPatch p;
    CHECK(patchInit(onetype::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(onetype::kVolume));
    // The reference's neutral patch is in range.
    for (int i = 0; i < kN; ++i) CHECK(patchParamInRange(onetype::kEngineId, i, rf::kBase[i]));
}

TEST(onetype_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(onetype::kEngineId, &p));
    const float vals[][2] = {{onetype::kOsc1Wave, 3},    {onetype::kOsc2Pitch, -7},   {onetype::kOsc2Fine, 33.5f},
                             {onetype::kSync, 1},        {onetype::kCutoff, 432.5f},  {onetype::kLfo1Rate, 17.25f},
                             {onetype::kLfo2Sync, 19},   {onetype::kModAmount, -0.4f}, {onetype::kGlideTime, 0.25f},
                             {onetype::kFilterType, 4}, {onetype::kSubOctave, 0}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(onetype::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(onetype::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    CHECK(!patchParamInRange(onetype::kEngineId, onetype::kOsc1Wave, 4.0f));
    CHECK(!patchParamInRange(onetype::kEngineId, onetype::kOsc2Pitch, 61.0f));
    CHECK(!patchParamInRange(onetype::kEngineId, onetype::kLfo1Sync, 31.0f));
}

TEST(onetype_silent_when_idle) {
    Rig r(initValues());
    r.run(20);   // no note
    CHECK_EQ(peak(r.L), 0.0f);
    CHECK_EQ(peak(r.R), 0.0f);
    float l[kBlock] = {}, rr[kBlock] = {};
    r.v[3].gate = false;
    CHECK(!r.e->render(r.v[3], r.p, 1.0f, 0.0f, l, rr));
    CHECK_EQ(peak(Buf(l, l + kBlock)), 0.0f);
    CHECK_EQ(peak(Buf(rr, rr + kBlock)), 0.0f);
}

// --- M1: oscillators, sub, mix, noise, the amp envelope --------------------------------------

TEST(onetype_ref_pitch) {
    compareCases("pitch_", [](const rf::Case& c, const float* o, int k) { return cents(o[k], c.exp[k]) / 1.0; });
}

TEST(onetype_ref_harmonics) {
    // 0.5 dB below 5 kHz, 3 dB to 12 kHz (polyBLEP rolls the top octave off), above: not compared.
    compareCases("harm_", [](const rf::Case& c, const float* o, int k) {
        const double f = c.arg[0] * double(k + 1);
        if (f > 12000.0) return 0.0;
        return levelErr(c, o, k, f < 5000.0 ? 0.5 : 3.0, 45.0);
    });
}

TEST(onetype_ref_mix_law) {
    compareCases("mix_", [](const rf::Case& c, const float* o, int k) { return levelErr(c, o, k, 0.3, 40.0); });
}

TEST(onetype_ref_noise) {
    compareCases("noise_", [](const rf::Case& c, const float* o, int k) {
        return k == 0 ? levelErr(c, o, k, 0.5, 50.0, 0, 1) : (double(o[k]) / c.exp[k] - 1.0) / 0.05;
    });
}

TEST(onetype_ref_amp_envelope) {
    // Attack time to 90 %, decay / release to 1/e, sustain level: 3 %.
    compareCases("env_", [](const rf::Case& c, const float* o, int k) {
        const bool used = startsWith(c.name, "env_attack") ? k == 0
                          : startsWith(c.name, "env_decay") ? k == 1
                          : startsWith(c.name, "env_sustain") ? k == 2
                                                               : k == 3;
        if (!used) return 0.0;
        return (double(o[k]) / c.exp[k] - 1.0) / 0.03;
    });
}

// --- M2: the filter section --------------------------------------------------------------

TEST(onetype_ref_filter_response) {
    // Small-signal responses (impulse response, 4 types x 12 cutoffs x 4
    // resonances): 1 dB at the probes (down to -60 dB), the peak 2 %.
    compareCases(
        "filter_",
        [](const rf::Case& c, const float* o, int k) {
            const bool peakFeature = c.arg[3] > 0.5f && k == c.nexp - 1;
            if (peakFeature) return (double(o[k]) / c.exp[k] - 1.0) / 0.02;
            if (c.exp[k] < -60.0f) return 0.0;
            return (double(o[k]) - c.exp[k]) / 1.0;
        },
        "filter_lpdl");
}

TEST(onetype_ref_lp24_self_oscillation) {
    compareCases("selfosc_", [](const rf::Case& c, const float* o, int k) {
        return k == 0 ? (double(o[k]) - c.exp[k]) / 1.0 : (double(o[k]) / c.exp[k] - 1.0) / 0.02;
    });
}

TEST(onetype_ref_limiter_and_saturation) {
    compareCases("limsat_", [](const rf::Case& c, const float* o, int k) { return (double(o[k]) - c.exp[k]) / 0.5; });
}

TEST(onetype_ref_filter_env_and_key_track) {
    // The cutoff (a resonant BP12's peak on noise) within 3 % (0.043 octave).
    const auto cmp = [](const rf::Case& c, const float* o, int k) { return std::log2(double(o[k]) / c.exp[k]) / 0.043; };
    compareCases("fenv_", cmp);
    compareCases("keytrack_", cmp);
    compareCases("fenv24_", cmp);   // LP24: the source's ladder knob axis
    compareCases("keytrack24_", cmp);
}

// The Init patch (LP24) through a SynthBank slot at the default slot gain:
// one voice about -9 dBFS like the other synths, 4-voice chords below full
// scale, no clipping.
namespace {
struct BankSlot {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
    float pk = 0.0f;
    long clips = 0;
    BankSlot(int32_t engine, int poly) {
        mod::ModuleHost::instance().init();
        gov.setBudget(8);
        bank.setGovernor(&gov);
        bank.setStartsPerBlock(0);
        patchInit(engine, &patch);
        bank.bindPatch(0, &patch);
        SynthSlotRecord r = SynthSlotRecord::defaults(0);
        r.model = uint32_t(SynthModel::ACD);
        r.polyphony = uint32_t(poly);
        r.delaySend = 0.0f;
        r.reverbSend = 0.0f;
        bank.setSlotRecord(0, r);
        bank.setSlotEngine(0, engine >= 0x80 ? engine : 0);
        block();
    }
    void block() {
        for (uint32_t i = 0; i < kBlock; ++i) L[i] = R[i] = D[i] = V[i] = 0.0f;
        bank.render(in, L, R, D, V, nullptr);
    }
    void on(int track, int note) {
        SynthNoteOn n;
        n.slot = 0;
        n.note = uint8_t(note);
        n.volume = -1;
        bank.noteOn(track, n);
    }
    void off(int track) { bank.noteOff(track, NoteOffMode::Off); }
    void run(int n) {
        for (int b = 0; b < n; ++b) {
            block();
            for (uint32_t i = 0; i < kBlock; ++i) {
                const float a = std::max(std::fabs(L[i]), std::fabs(R[i]));
                pk = std::max(pk, a);
                if (a >= 1.0f) ++clips;
            }
        }
    }
};
const int kChords[4][4] = {{60, 64, 67, 71}, {57, 60, 64, 67}, {48, 55, 60, 64}, {62, 65, 69, 72}};
float chordPeak(int32_t engine, int voices, long& clips) {
    BankSlot s(engine, 4);
    for (int trial = 0; trial < 40; ++trial) {
        for (int k = 0; k < voices; ++k) s.on(k, kChords[trial % 4][k]);
        s.run(120);
        for (int k = 0; k < voices; ++k) s.off(k);
        s.run(250);
    }
    clips = s.clips;
    return s.pk;
}
double dbfs(float x) { return 20.0 * std::log10(x > 1e-9f ? double(x) : 1e-9); }
}  // namespace

TEST(onetype_level_matches_the_synth_conventions) {
    long c1 = 0, c4 = 0;
    const float p1 = chordPeak(onetype::kEngineId, 1, c1);
    const float p4 = chordPeak(onetype::kEngineId, 4, c4);
#if defined(OT_MOD_ACD1)
    long ca = 0;
    const float pa = chordPeak(kSynthEngineAcd, 1, ca);
    std::printf("  1-Type Init at -9 dB: 1 voice peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(pa), dbfs(p4));
    CHECK(std::fabs(dbfs(p1) - dbfs(pa)) < 1.5);
#else
    std::printf("  1-Type Init at -9 dB: 1 voice peak %.1f dBFS, 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(p4));
#endif
    CHECK(dbfs(p1) < -6.0);
    CHECK(dbfs(p1) > -20.0);
    CHECK(dbfs(p4) < -1.0);
    CHECK_EQ(c1, 0L);
    CHECK_EQ(c4, 0L);
}

// SynthVoice::cutoffOct (the track's Performance filter offset, octaves)
// moves the cutoff inside the engine's clamps; 0 leaves the render as it is.
namespace {
std::vector<float> cutRender(int type, float cutHz, float oct) {
    std::vector<float> v = initValues();
    v[onetype::kFilterType] = float(type);
    v[onetype::kCutoff] = cutHz;
    v[onetype::kEnvAmt] = 0.0f;
    v[onetype::kLfo1Depth] = 0.0f;
    v[onetype::kLfo2Depth] = 0.0f;
    v[onetype::kPhase] = 1.0f / 127.0f;
    Rig r(v);
    r.v[0].cutoffOct = oct;
    r.on(0, 48.0f);
    r.run(40);
    return r.L;
}
double relDiff(const Buf& a, const Buf& b) {
    double d = 0, e = 0;
    for (size_t i = 0; i < a.size(); ++i) d += double(a[i] - b[i]) * (a[i] - b[i]), e += double(a[i]) * a[i];
    return std::sqrt(d / (e + 1e-30));
}
}  // namespace

TEST(onetype_perf_cutoff_offset_moves_the_cutoff) {
    for (int type : {int(onetype::kLp12), int(onetype::kLp24)}) {
        const Buf ref = cutRender(type, 400.0f, 0.0f);
        CHECK(rms(ref) > 1e-4);
        CHECK(relDiff(ref, cutRender(type, 1600.0f, -2.0f)) < 0.05);
        CHECK(relDiff(cutRender(type, 3200.0f, 0.0f), cutRender(type, 400.0f, 3.0f)) < 0.05);
        CHECK(relDiff(ref, cutRender(type, 400.0f, -3.0f)) > 0.1);
        CHECK(relDiff(ref, cutRender(type, 400.0f, 0.0f)) == 0.0);
        for (float o : {-9.54f, 9.54f}) {
            const Buf x = cutRender(type, o > 0 ? 20000.0f : 16.0f, o);
            bool fin = true;
            for (float f : x) fin = fin && std::isfinite(f);
            CHECK(fin);
        }
    }
}

// --- M3: cross modulation, the mod envelope, the LFOs, glide -----------------------------------

TEST(onetype_ref_fm) {
    compareCases("fm_", [](const rf::Case& c, const float* o, int k) { return levelErr(c, o, k, 1.0, 40.0); });
}

TEST(onetype_ref_ring_and_sync) {
    const auto cmp = [](const rf::Case& c, const float* o, int k) {
        return k == 0 ? levelErr(c, o, k, 1.5, 50.0, 0, 1) : (double(o[k]) / c.exp[k] - 1.0) / 0.10;
    };
    compareCases("ring_", cmp);
    compareCases("sync_", cmp);
    compareCases("syncfm_", cmp);
}

TEST(onetype_ref_detune) {
    // The level (2 copies here for the source's 8; its copies' table images
    // move the centroid, not compared).
    // 3 dB: two copies beat more than eight over the window.
    compareCases("detune_", [](const rf::Case& c, const float* o, int k) { return k == 0 ? levelErr(c, o, k, 3.0, 50.0, 0, 1) : 0.0; });
}

TEST(onetype_ref_mod_env) {
    // 3 %: of the pitch offset in cents, of the duty (+0.015); the FM sidebands 1.5 dB.
    compareCases("modenv_pitch", [](const rf::Case& c, const float* o, int k) {
        if (c.exp[k] <= 0.0f) return 0.0;   // too low to measure in its window
        const double base = hz(c.key), dev = cents(c.exp[k], base);
        return cents(o[k], c.exp[k]) / (0.03 * std::fabs(dev) + 2.0);
    });
    compareCases("modenv_fm", [](const rf::Case& c, const float* o, int k) { return levelErr(c, o, k, 1.5, 40.0, 0, 2); });
    compareCases("modenv_pw", [](const rf::Case& c, const float* o, int k) { return (double(o[k]) - c.exp[k]) / (0.03 * c.exp[k] + 0.015); });
}

TEST(onetype_ref_lfo) {
    // 2 %: of the pitch offset in cents (+1 cent), of the level, the duty, the
    // cutoff in octaves (0.03 octave), the period, the pan gains; FM 1 dB.
    const auto pitch = [](const rf::Case& c, const float* o, int k) {
        const double base = hz(c.key), dev = cents(c.exp[k], base);
        return cents(o[k], c.exp[k]) / (0.02 * std::fabs(dev) + 1.0);
    };
    compareCases("lfo_osc", pitch);
    compareCases("lfo_both_osc12", pitch);
    const auto cutoff = [](const rf::Case& c, const float* o, int k) { return std::log2(double(o[k]) / c.exp[k]) / 0.03; };
    compareCases("lfo_filter", cutoff);
    compareCases("lfo_both_filter", cutoff);
    const auto level = [](const rf::Case& c, const float* o, int k) { return (double(o[k]) - c.exp[k]) / 0.02; };
    compareCases("lfo_wave_", level);
    compareCases("lfo_amp", level);
    compareCases("lfo_both_amp", level);
    const auto duty = [](const rf::Case& c, const float* o, int k) { return (double(o[k]) - c.exp[k]) / (0.02 * c.exp[k] + 0.01); };
    compareCases("lfo_pw", duty);
    compareCases("lfo_both_pw", duty);
    compareCases("lfo_fm", [](const rf::Case& c, const float* o, int k) { return levelErr(c, o, k, 1.0, 40.0, 0, 2); });
    // Pan: each channel's gain within 2 % of the louder channel's.
    const auto pan = [](const rf::Case& c, const float* o, int k) {
        const int w = k & ~1;
        const double off = levelOffset();
        const double top = std::pow(10.0, std::max(c.exp[w], c.exp[w + 1]) / 20.0);
        return (std::pow(10.0, (double(o[k]) - off) / 20.0) - std::pow(10.0, c.exp[k] / 20.0)) / (0.02 * top);
    };
    compareCases("lfo_pan", pan);
    compareCases("lfo_both_pan", pan);
    const auto period = [](const rf::Case& c, const float* o, int k) { return (double(o[k]) / c.exp[k] - 1.0) / 0.02; };
    compareCases("lfo_rate_", period);
    compareCases("lfo_sync_", period);
}

TEST(onetype_ref_glide) {
    compareCases("glide_", [](const rf::Case& c, const float* o, int k) { return (double(o[k]) / c.exp[k] - 1.0) / 0.05; });
}

// --- engine cases (re-expressed from the source's own tests) -------------------------------------

namespace {
// A flat patch: osc 1 sine alone, the filter open, the envelopes flat, no LFO.
std::vector<float> flat() {
    std::vector<float> v(rf::kBase, rf::kBase + kN);
    v[onetype::kOsc1Wave] = float(onetype::kSine);
    return v;
}
// Hz of x[a, b) by upward zero crossings.
double crossHz(const Buf& x, size_t a, size_t b) {
    const Buf part(x.begin() + long(a), x.begin() + long(std::min(b, x.size())));
    const std::vector<double> z = rf::upCrossings(part, kFs);
    return z.size() >= 2 ? double(z.size() - 1) / (z.back() - z.front()) : 0.0;
}
}  // namespace

// The source's PortamentoLandsAndStartsFromNoteZero: a glide lands on its
// note in finite time. A fresh voice here has no last pitch, so the slot's
// first note starts on its own pitch (the source's first note sweeps up
// from note 0).
TEST(onetype_glide_lands_and_the_first_note_starts_on_pitch) {
    std::vector<float> v = flat();
    v[onetype::kGlideMode] = float(onetype::kLegato);
    v[onetype::kGlideTime] = 0.31f;
    Rig r(v);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 2);
    CHECK(std::fabs(cents(crossHz(r.L, size_t(0.005 * kFs), size_t(0.04 * kFs)), hz(60))) < 2.0);
    const size_t at = r.L.size();
    r.on(0, 72.0f, true, true);   // legato glide up an octave
    r.run(kBlocksPerSec);
    const double target = hz(72);
    CHECK(crossHz(r.L, at + size_t(0.200 * kFs), at + size_t(0.225 * kFs)) < target * std::exp2(-20 / 1200.0));   // still gliding
    CHECK(std::fabs(crossHz(r.L, at + size_t(0.35 * kFs), at + size_t(0.5 * kFs)) - target) < 0.05);           // landed
}

// The source's PolyPortamentoGlidesFromTheVoicesLastPitch: here a fresh voice
// glides from the slot's last note (the framework zeroes a fresh voice).
TEST(onetype_poly_glide_starts_from_the_slots_last_note) {
    std::vector<float> v = flat();
    v[onetype::kGlideMode] = float(onetype::kAlways);
    v[onetype::kGlideTime] = 0.3f;
    Rig r(v);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec);
    r.off(0);
    const size_t at = r.L.size();
    r.on(1, 64.0f);   // a fresh voice: from 60, upward
    r.run(kBlocksPerSec / 10);
    const double f = crossHz(r.L, at + size_t(0.005 * kFs), at + size_t(0.02 * kFs));
    CHECK(f > hz(60) && f < hz(64));
}

// The source's MonoRetriggerRestartsPhases (Phase > 0).
TEST(onetype_mono_retrigger_restarts_phases) {
    std::vector<float> v = flat();
    v[onetype::kOsc1Wave] = float(onetype::kSaw);
    v[onetype::kPhase] = 1.0f / 127.0f;
    v[onetype::kGlideMode] = float(onetype::kAlways);
    Rig held(v), fresh(v);
    held.on(0, 60.0f);
    held.run(6);
    const size_t at = held.L.size();
    held.on(0, 64.0f, true, true);
    held.run(8);
    fresh.on(0, 64.0f);
    fresh.run(8);
    float diff = 0;
    for (size_t i = 64; i < fresh.L.size(); ++i) diff = std::max(diff, std::fabs(held.L[at + i] - fresh.L[i]));
    std::printf("  retrigger vs fresh: max difference %.4f (peak %.3f)\n", double(diff), double(peak(fresh.L)));
    CHECK(peak(fresh.L) > 0.01f);
    CHECK(diff < 0.01f * peak(fresh.L) / 0.2f);
}

// The source's StealingASoundingVoiceDoesNotClick: a Poly 1 slot's sounding
// voice takes the next note (a sine with its square sub an octave down).
TEST(onetype_a_reused_sounding_voice_does_not_click) {
    std::vector<float> v = flat();
    v[onetype::kPhase] = 0.0f;   // free running (the source's default)
    v[onetype::kSubLevel] = 1.0f;
    v[onetype::kSubOctave] = 1.0f;
    v[onetype::kAttack] = 0.0156f;
    Rig r(v);
    r.on(0, 48.0f);
    r.run(kBlocksPerSec / 2);
    const size_t at = r.L.size();
    r.on(0, 50.0f, true, true);
    r.run(kBlocksPerSec / 2);
    const auto kink = [&](size_t from, size_t to) {
        float k = 0;
        for (size_t i = from; i < to; ++i) k = std::max(k, std::fabs(r.L[i] - 2 * r.L[i - 1] + r.L[i - 2]));
        return k;
    };
    CHECK(peak(r.L) > 0.01f);
    CHECK(kink(at, at + 256) < 2 * kink(at / 2, at));
}

// The source's ReleasedVoicesEndWithShortHostBlocks: a released voice ends
// and the output is exact silence after it.
TEST(onetype_a_released_voice_ends) {
    std::vector<float> v = flat();
    v[onetype::kRelease] = 0.005f;
    Rig r(v);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 10);
    CHECK(rms(r.L) > 0.01);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < kBlocksPerSec) ++blocks;
    CHECK(blocks < 10);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

// The source's LfoRandomRestartDrawsANewValue: a key-synced S&H LFO draws a
// new value at every restart (a note with no other held).
TEST(onetype_random_lfo_restart_draws_a_new_value) {
    std::vector<float> v = flat();
    v[onetype::kLfo1Dest] = float(onetype::kToAmp);
    v[onetype::kLfo1Wave] = float(onetype::kLfoSh);
    v[onetype::kLfo1Rate] = 0.1f;
    v[onetype::kLfo1Depth] = 1.0f;
    v[onetype::kLfo1KeySync] = 1.0f;
    Rig r(v);
    double lv[3];
    for (double& x : lv) {
        r.L.clear();
        r.on(0, 60.0f);
        r.run(20);
        x = rms(r.L, r.L.size() / 2, r.L.size());
        r.off(0);
        r.run(40);
    }
    std::printf("  S&H levels at three restarts: %.4f %.4f %.4f\n", lv[0], lv[1], lv[2]);
    CHECK(lv[0] != lv[1] && lv[1] != lv[2]);
}

// --- the engine contract ----------------------------------------------------------------------

TEST(onetype_volume_is_never_applied) {
    std::vector<float> a = initValues(), b = initValues();
    a[onetype::kVolume] = 0.0f;
    b[onetype::kVolume] = 2.0f;
    Rig ra(a), rb(b);
    ra.on(0, 60.0f);
    rb.on(0, 60.0f);
    ra.run(40);
    rb.run(40);
    CHECK(rms(ra.L) > 1e-3);
    CHECK(ra.L == rb.L);
}

TEST(onetype_voice_level) {
    std::vector<float> c = flat();
    c[onetype::kAttack] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);   // a fresh voice is no steal victim before its first block
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    const float lv = r.v[0].level;
    r.on(0, 62.0f, true, false);
    CHECK_EQ(r.v[0].level, lv);
}

TEST(onetype_no_nan_or_denormals_at_the_extremes) {
    for (int side = 0; side < 2; ++side) {
        std::vector<float> v = initValues();
        for (int i = 0; i < kN; ++i) v[size_t(i)] = side ? range(i)->max : range(i)->min;
        v[onetype::kAttack] = 0.0f;
        v[onetype::kSustain] = 1.0f;
        for (int type = 0; type < 5; ++type) {
            v[onetype::kFilterType] = float(type);
            Rig r(v);
            r.on(0, 100.0f);
            r.on(1, 10.0f);
            r.run(60);
            bool finite = true;
            long sub = 0;
            for (float s : r.L) finite = finite && std::isfinite(s), sub += std::fpclassify(s) == FP_SUBNORMAL;
            CHECK(finite);
            CHECK_EQ(sub, 0L);
        }
    }
}

// Every parameter (Volume aside: the framework's) changes the sound of a rich
// patch playing a short phrase (a legato note, a fresh one after a release).
namespace {
std::vector<float> richContext() {
    std::vector<float> v = initValues();
    const float set[][2] = {
        {onetype::kOsc1Wave, onetype::kSaw},   {onetype::kOsc2Wave, onetype::kOsc2Pulse}, {onetype::kOsc2Pitch, 7},
        {onetype::kMix, 0.5f},                 {onetype::kSubLevel, 0.5f},                {onetype::kSubWave, onetype::kSubSquare},
        {onetype::kPulseWidth, 0.3f},          {onetype::kModAmount, 0.5f},               {onetype::kModDest, onetype::kModFm},
        {onetype::kModDecay, 0.3f},            {onetype::kFilterType, onetype::kLp12},    {onetype::kCutoff, 2000},
        {onetype::kResonance, 0.5f},           {onetype::kEnvAmt, 0.5f},                  {onetype::kKeyTrack, 0.5f},
        {onetype::kSaturation, 0.3f},          {onetype::kFAttack, 0.01f},                {onetype::kFDecay, 0.3f},
        {onetype::kFSustain, 0.5f},            {onetype::kFRelease, 0.3f},                {onetype::kAttack, 0.01f},
        {onetype::kDecay, 0.3f},               {onetype::kSustain, 0.7f},                 {onetype::kRelease, 0.2f},
        {onetype::kLfo1Dest, onetype::kToFilter}, {onetype::kLfo1Wave, onetype::kLfoTri}, {onetype::kLfo1Rate, 3},
        {onetype::kLfo1Depth, 0.3f},           {onetype::kLfo2Dest, onetype::kToPw},      {onetype::kLfo2Wave, onetype::kLfoSine},
        {onetype::kLfo2Rate, 5},               {onetype::kLfo2Depth, 0.3f},               {onetype::kGlideTime, 0.1f},
        {onetype::kPhase, 0.5f}};
    for (const auto& x : set) v[size_t(x[0])] = x[1];
    return v;
}
Buf phrase(const std::vector<float>& v) {
    Rig r(v);
    r.on(0, 48.0f);
    r.run(kBlocksPerSec / 4);
    r.on(0, 55.0f, true, true);
    r.run(kBlocksPerSec / 4);
    r.off(0);
    r.run(kBlocksPerSec / 3);
    r.on(1, 60.0f);
    r.run(kBlocksPerSec / 5);
    r.off(1);
    r.run(kBlocksPerSec / 5);
    Buf out = r.L;
    out.insert(out.end(), r.R.begin(), r.R.end());
    return out;
}
double rmsDiff(const Buf& a, const Buf& b) {
    double s = 0.0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i) s += double(a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s / double(std::min(a.size(), b.size())));
}
}  // namespace

TEST(onetype_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == onetype::kVolume) continue;
        std::vector<float> a = ctx, b = ctx;
        a[size_t(i)] = range(i)->min;
        b[size_t(i)] = range(i)->max;
        if (i == onetype::kLfo1Wave || i == onetype::kLfo1Sync || i == onetype::kLfo1KeySync || i == onetype::kLfo1Rate)
            a[onetype::kLfo1Dest] = b[onetype::kLfo1Dest] = float(onetype::kToOsc12);
        if (i == onetype::kLfo2KeySync) a[onetype::kLfo2Wave] = b[onetype::kLfo2Wave] = float(onetype::kLfoSh);
        const double d = rmsDiff(phrase(a), phrase(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

// A parameter that moved (start != end for one block) holds its new value in
// the blocks after it (the framework clears `moving` without a new serial).
TEST(onetype_a_moved_parameter_holds_its_new_value) {
    std::vector<float> v = flat();
    v[onetype::kOsc1Wave] = float(onetype::kSaw);
    Rig r(v);
    r.on(0, 60.0f);
    r.run(10);
    r.p.end[onetype::kKeyShift] = 12.0f;
    r.p.end[onetype::kCutoff] = 3000.0f;
    r.p.moving = true;
    ++r.p.serial;
    r.run(1);
    r.p.start[onetype::kKeyShift] = 12.0f;
    r.p.start[onetype::kCutoff] = 3000.0f;
    r.p.moving = false;   // the same serial
    const size_t at = r.L.size();
    r.run(30);
    const double f = crossHz(r.L, at + 4 * kBlock, r.L.size());
    std::printf("  after Key Shift 0 -> 12: %.2f Hz (wants %.2f)\n", f, hz(72));
    CHECK(std::fabs(cents(f, hz(72))) < 5.0);
    // Block to block the same (no ramp from the old value again).
    CHECK(std::fabs(rms(r.L, at + 10 * kBlock, at + 20 * kBlock) - rms(r.L, at + 20 * kBlock, at + 30 * kBlock)) < 0.02 * rms(r.L, at, r.L.size()));
}

// Osc 1's detune spreads its copies as wide as the source's eight (+-1, 3, 5,
// 7 x 20 cents: 92 cents rms): at Detune 100 % the copies sit +-92 cents out.
TEST(onetype_detune_is_as_wide_as_the_source) {
    std::vector<float> v = flat();
    v[onetype::kOsc1Detune] = 1.0f;
    v[onetype::kPhase] = 0.0f;
    Rig r(v);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec);
    const size_t a = size_t(0.2 * kFs), b = r.L.size();
    const double main = rf::toneAmp(r.L, a, b, hz(60), kFs);
    const double up = rf::toneAmp(r.L, a, b, hz(60) * std::exp2(92.0 / 1200.0), kFs);
    const double down = rf::toneAmp(r.L, a, b, hz(60) * std::exp2(-92.0 / 1200.0), kFs);
    std::printf("  detune 100 %%: main %.4f, +92 cents %.4f, -92 cents %.4f\n", main, up, down);
    CHECK(up > 0.7 * main && down > 0.7 * main);
}

// LPDL (the diode ladder) does not fit the CPU budget next to the worst
// kept case (FM + sync on triangles through LP24 already takes 44,300 of
// 45,000 instructions): it plays as LP24 with its resonance at 85 %.
TEST(onetype_lpdl_plays_as_lp24_with_its_resonance_scaled) {
    std::vector<float> a = richContext(), b = richContext();
    a[onetype::kFilterType] = float(onetype::kLpdl);
    a[onetype::kResonance] = 0.8f;
    b[onetype::kFilterType] = float(onetype::kLp24);
    b[onetype::kResonance] = 0.8f * 0.85f;
    CHECK(phrase(a) == phrase(b));
}

// Golden hashes: the Init patch and the rich context's phrase, bit for bit
// (a change of the sound must change these on purpose).
namespace {
uint64_t hashOf(const Buf& x) {
    uint64_t h = 1469598103934665603ull;
    for (float f : x) {
        uint32_t b;
        std::memcpy(&b, &f, 4);
        for (int k = 0; k < 4; ++k) h = (h ^ ((b >> (8 * k)) & 0xFFu)) * 1099511628211ull;
    }
    return h;
}
}  // namespace

TEST(onetype_golden_hashes) {
    const uint64_t init = hashOf(phrase(initValues())), rich = hashOf(phrase(richContext()));
    std::printf("  1-Type golden: init %016llx, rich %016llx\n", (unsigned long long)init, (unsigned long long)rich);
    CHECK_EQ(init, uint64_t(0x924ecb15e2ea84e7ull));
    CHECK_EQ(rich, uint64_t(0x6266afadd1a3f44bull));
}
