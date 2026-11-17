// V-Type engine (opentracker-synths modules/vtype, engine 0x8B): the
// descriptor ("V-Type" on screen, Patches/VTYPE/), the patch round trip,
// silence when idle; then every law of the voice against its closed form,
// computed here in double: pitch, the Swarm's lines and level, the saw's
// harmonics and its aliasing, Phase, the sub, osc 2 (waves, PW, sync, ring),
// Balance, noise and Colour, both filters' responses, the four routings and
// Split Width, Drive, Env Amt / Key Track / the Performance cutoff, the three
// envelopes, the LFOs (rates, shapes, tempo divisions, Key Sync), the matrix
// against its scale table, glide; and the engine contract: Volume never
// applied, the voice level, a reused voice and moved parameters without a
// click, chords as the sum of their voices, no NaN or denormals over random
// sets, levels at the synth conventions, every parameter audible, golden
// hashes.
#include "test.h"

#include <cstdio>
#include <set>

#include "vtype_probe.h"

using namespace vprobe;

namespace {

// Milestones whose laws the engine implements (tests of later ones report
// themselves as pending).
constexpr int kDone = 4;
bool pending(int milestone) {
    if (milestone <= kDone) return false;
    std::printf("  pending (M%d)\n", milestone);
    return true;
}

// A plain held tone: one saw (Phase Reset), filter 1 open (LP12 at the top,
// no resonance, no envelope, no key track), full sustain.
Vals plain() {
    Vals c = initValues();
    c[vt::kCount] = 0.0f;
    c[vt::kPhase] = float(vt::kPhaseReset);
    c[vt::kF1Type] = float(vt::kLp12);
    c[vt::kF1Cutoff] = 20000.0f;
    c[vt::kF1Reso] = 0.0f;
    c[vt::kF1Env] = 0.0f;
    c[vt::kF1Key] = 0.0f;
    c[vt::kAmpA] = 0.0f;
    c[vt::kAmpS] = 1.0f;
    return c;
}
// Osc 2 alone (Balance +1) as a sine probe at `pitch` through the routing.
Vals probe(int wave = vt::kO2Sine) {
    Vals c = plain();
    c[vt::kBalance] = 1.0f;
    c[vt::kO2Wave] = float(wave);
    return c;
}
Buf play(const Vals& vals, float pitch = 60.0f, double seconds = 0.5) {
    Rig r(vals);
    r.on(0, pitch);
    r.seconds(seconds);
    return r.L;
}
// The steady part of a note (after 0.1 s).
Buf held(const Vals& vals, float pitch, double seconds = 1.0) { return slice(play(vals, pitch, seconds + 0.1), 0.1, seconds + 0.1); }
// The voice's output gain: the plain saw's fundamental against 2 / pi.
double outGain() {
    static double g = 0.0;
    if (g == 0.0) {
        const double f0 = hzOf(48.0);
        g = level(held(plain(), 48.0f, 1.0), f0) / (2.0 / kPi * std::abs(typeH(vt::kLp12, 20000.0, 0.0, f0)));
    }
    return g;
}

}  // namespace

// --- descriptor, patch, idle (M0) -------------------------------------------------------

TEST(vtype_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(vt::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "VTYPE");
    CHECK_EQ(e->id(), vt::kEngineId);
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(vt::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_STR(d->name, "VTYPE");
    CHECK_STR(d->displayName, "V-Type");
    CHECK_EQ(int(d->paramCount), 67);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK(int(d->paramCount) <= kPatchMaxParams);
    CHECK_EQ(int(d->volumeParam), int(vt::kVolume));
    CHECK_EQ(int(d->sectionCount), 16);
    CHECK_EQ(int(d->extCount), 2);
    CHECK(d->ext[1].flags & kParamFlagSpread);
    CHECK_STR(synthEngineFolder(vt::kEngineId), "VTYPE");
    const int k = synthModelListIndex(vt::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "V-Type");
    std::vector<int> seen(size_t(kN) + 2, 0);
    int macros = 0;
    for (int s = 0; s < d->sectionCount; ++s) {
        const synthui::Section& sec = d->sections[s];
        if (sec.macro >= 0) {
            CHECK_EQ(int(sec.macro), macros);
            ++macros;
            continue;
        }
        CHECK(sec.count <= 12);
        for (int j = 0; j < sec.count; ++j)
            if (sec.params[j] < kN + 2) ++seen[sec.params[j]];
    }
    CHECK_EQ(macros, 7);
    for (int i = 0; i < kN + 2; ++i) {
        const PatchParamRange& r = i < kN ? d->params[i] : d->ext[i - kN];
        CHECK(r.min < r.max && r.def >= r.min && r.def <= r.max);
        CHECK(d->ui[i].label != nullptr && std::strlen(d->ui[i].label) > 0);
        if (d->ui[i].fmt == synthui::kFmtEnum) CHECK_EQ(int(d->ui[i].nameCount), int(r.max - r.min) + 1);
        CHECK_EQ(seen[size_t(i)], 1);
    }
    SynthPatch p;
    CHECK(patchInit(vt::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(vt::kVolume));
}

TEST(vtype_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(vt::kEngineId, &p));
    const float vals[][2] = {{vt::kCount, 3}, {vt::kDetune, 0.8f}, {vt::kO2Wave, 1}, {vt::kO2Semi, -12},
                             {vt::kRouting, 3}, {vt::kCurve, 2}, {vt::kL1Sync, 9}, {float(vt::modDest(3)), 12},
                             {float(vt::modAmt(2)), -0.4f}, {vt::kF2Cutoff, 3500}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(vt::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    CHECK_EQ(q.engine, vt::kEngineId);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
}

TEST(vtype_silent_when_idle) {
    Rig r(initValues());
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
    CHECK_EQ(peak(r.R), 0.0f);
}

// --- M1: Swarm, sub, Phase, amp envelope, glide, levels ----------------------------------

TEST(vtype_sounds_and_ends) {
    if (pending(1)) return;
    Rig r(initValues());
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 4);
    CHECK(rms(r.L) > 0.01);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    const size_t at = r.L.size();
    r.run(20);
    CHECK_EQ(peak(r.L, at), 0.0f);
}

TEST(vtype_pitch_of_the_centre_saw) {
    if (pending(1)) return;
    // Count 1, open filter: the saw's zero crossings; Key Shift, Fine Tune and bend move it.
    for (float key : {24.0f, 48.0f, 72.0f, 96.0f}) {
        const Buf x = held(plain(), key, 1.0);
        const double c = cents(freqOf(x), hzOf(key));
        if (!(std::fabs(c) < 1.0)) std::printf("       key %.0f: %.3f cents\n", key, c);
        CHECK(std::fabs(c) < 1.0);
    }
    Vals s = plain();
    s[vt::kKeyShift] = 7.0f;
    s[vt::kFineTune] = -30.0f;
    CHECK(std::fabs(cents(freqOf(held(s, 48.0f)), hzOf(48.0 + 7.0 - 0.3))) < 1.0);
    Rig r(plain());
    r.on(0, 48.0f);
    r.v[0].bend = r.v[0].bendStart = 2.5f;
    r.seconds(1.1);
    CHECK(std::fabs(cents(freqOf(slice(r.L, 0.1, 1.1)), hzOf(50.5))) < 1.0);
}

TEST(vtype_swarm_lines_follow_the_detune_and_mix_laws) {
    if (pending(1)) return;
    // Harmonic 8 of every saw at its ratio, at the Mix law's level against
    // the centre saw alone (Count 1), key 60, Phase Reset.
    const double h = 8.0, f0 = hzOf(60.0);
    Vals one = plain();
    const double ref = level(held(one, 60.0f, 4.0), h * f0);
    int bad = 0;
    for (int c : {1, 2, 3}) {
        for (double d : {0.2, 0.5, 1.0}) {
            for (double m : {0.2, 0.6, 1.0}) {
                Vals v = plain();
                v[vt::kCount] = float(c);
                v[vt::kDetune] = float(d);
                v[vt::kSwarmMix] = float(m);
                const Buf x = held(v, 60.0f, 4.0);
                double wc, ws;
                swarmLevels(c, m, wc, ws);
                for (int k = 0; k < vt::sawsOf(c); ++k) {
                    const double f = h * f0 * std::pow(2.0, sawCents(c, d, k) / 1200.0);
                    const double got = db(level(x, f) / ref), want = db(k == 0 ? wc : ws);
                    if (std::fabs(got - want) > 0.5) {
                        ++bad;
                        std::printf("       count %d detune %.1f mix %.1f saw %d: %.2f dB, want %.2f\n", c, d, m, k, got, want);
                    }
                }
            }
        }
    }
    CHECK_EQ(bad, 0);
}

TEST(vtype_swarm_level_holds_against_count_and_mix) {
    if (pending(1)) return;
    Vals one = plain();
    one[vt::kPhase] = float(vt::kPhaseFree);
    const double ref = rms(held(one, 60.0f, 4.0));
    for (int c = 0; c < 4; ++c)
        for (double m : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            Vals v = one;
            v[vt::kCount] = float(c);
            v[vt::kSwarmMix] = float(m);
            v[vt::kDetune] = 0.6f;
            const double d = db(rms(held(v, 60.0f, 4.0)) / ref);
            if (!(std::fabs(d) < 0.7)) std::printf("       count %d mix %.2f: %.2f dB\n", c, m, d);
            CHECK(std::fabs(d) < 0.7);
        }
}

TEST(vtype_saw_harmonics_and_aliasing) {
    if (pending(1)) return;
    // h1..h16 at 2 / (pi n) (to 5 kHz within 0.5 dB, to 12 kHz within 3 dB).
    for (float key : {36.0f, 60.0f, 84.0f}) {
        const Buf x = held(plain(), key, 1.0);
        const double f0 = hzOf(key);
        for (int n = 1; n <= 16; ++n) {
            const double f = n * f0;
            if (f > 12000.0) break;
            const double d = db(level(x, f) / (outGain() * 2.0 / (kPi * n) * std::abs(typeH(vt::kLp12, 20000.0, 0.0, f))));
            const double tol = f <= 5000.0 ? 0.5 : 3.0;
            if (!(std::fabs(d) < tol)) std::printf("       key %.0f h%d (%.0f Hz): %.2f dB\n", key, n, f, d);
            CHECK(std::fabs(d) < tol);
        }
    }
    // Aliases (folded harmonics) below 5 kHz, against the fundamental.
    const double pin[2] = {-67.0, -61.0};   // measured -70.3 / -64.2 dB, + 3 dB
    int i = 0;
    for (float key : {84.0f, 96.0f}) {
        const Buf x = held(plain(), key, 1.0);
        const double f0 = hzOf(key), fund = level(x, f0);
        double worst = -200.0;
        for (int n = 2; n < 200; ++n) {
            double f = std::fmod(n * f0, kFs);
            if (f > 0.5 * kFs) f = kFs - f;
            if (n * f0 < 0.5 * kFs || f > 5000.0 || f < 40.0) continue;
            const double near = std::fmod(f, f0);
            if (near < 20.0 || f0 - near < 20.0) continue;   // on a true harmonic
            worst = std::max(worst, db(level(x, f) / fund));
        }
        std::printf("  alias at key %.0f: %.1f dB\n", key, worst);
        CHECK(worst < -40.0);
        CHECK(worst < pin[i++]);
    }
}

TEST(vtype_phase_reset_repeats_free_does_not) {
    if (pending(1)) return;
    Vals r = initValues();
    r[vt::kPhase] = float(vt::kPhaseReset);
    CHECK(play(r, 57.0f) == play(r, 57.0f));
    Vals f = initValues();
    Rig a(f);
    a.on(0, 57.0f);
    a.seconds(0.3);
    a.on(1, 57.0f);
    a.L.clear();
    a.alive[0] = false;
    a.seconds(0.3);
    const Buf second = a.L;
    const Buf first = play(f, 57.0f, 0.3);
    CHECK(rmsDiff(first, second) > 1e-3);
}

TEST(vtype_sub_is_locked_one_octave_down) {
    if (pending(1)) return;
    for (int wave : {vt::kSubSquare, vt::kSubSine})
        for (float key : {36.0f, 60.0f}) {
            Vals v = plain();
            v[vt::kSubLevel] = 0.5f;
            v[vt::kSubWave] = float(wave);
            const Buf x = play(v, key, 30.0);
            const double f0 = hzOf(key);
            // Level: square 4 s / pi, sine s, against the saw's 2 / pi.
            const Buf a = slice(x, 0.2, 2.2), b = slice(x, 28.0, 30.0);
            const double want = outGain() * (wave == vt::kSubSquare ? 4.0 * 0.5 / kPi : 0.5);
            const double d = db(level(a, 0.5 * f0) / want);
            if (!(std::fabs(d) < 0.5)) std::printf("       sub %d key %.0f: level %.2f dB\n", wave, key, d);
            CHECK(std::fabs(d) < 0.5);
            // Locked: the sub's phase against the centre's does not drift over 30 s.
            const double pa = std::arg(line(a, f0)) - 2.0 * std::arg(line(a, 0.5 * f0));
            const double pb = std::arg(line(b, f0)) - 2.0 * std::arg(line(b, 0.5 * f0));
            const double t = (28.0 - 0.2);
            double drift = std::remainder((pb - 2.0 * kPi * f0 * t + 2.0 * 2.0 * kPi * 0.5 * f0 * t) - pa, 2.0 * kPi);
            if (!(std::fabs(drift) < 0.01)) std::printf("       sub %d key %.0f: drift %.4f rad\n", wave, key, drift);
            CHECK(std::fabs(drift) < 0.01);
        }
}

TEST(vtype_glide_modes_and_legato) {
    if (pending(1)) return;
    for (int mode = 0; mode < 4; ++mode)
        for (float to : {67.0f, 36.0f}) {
            Vals v = plain();
            v[vt::kGlideMode] = float(mode);
            v[vt::kGlideTime] = 0.4f;
            Rig r(v);
            r.on(0, 48.0f);
            r.seconds(0.3);
            const bool overlapping = true;
            r.on(0, to, true, overlapping);
            const size_t at = r.L.size();
            r.seconds(0.8);
            // Half way through the glide (0.2 s): half way in semitones.
            const double mid = pitchOf(freqOf(r.L, at + size_t(0.18 * kFs), at + size_t(0.22 * kFs)));
            const double end = pitchOf(freqOf(r.L, at + size_t(0.5 * kFs), at + size_t(0.8 * kFs)));
            const double want = 0.5 * (48.0 + to);
            if (!(std::fabs(mid - want) < 0.05 * std::fabs(to - 48.0))) std::printf("       mode %d to %.0f: mid %.2f want %.2f\n", mode, to, mid, want);
            CHECK(std::fabs(mid - want) < 0.05 * std::fabs(to - 48.0));
            CHECK(std::fabs(end - to) < 0.05);
        }
    // No glide without an overlap in the Overlap modes; legato keeps the envelope.
    Vals v = plain();
    v[vt::kGlideMode] = float(vt::kOverlap);
    v[vt::kGlideTime] = 0.4f;
    Rig r(v);
    r.on(0, 48.0f);
    r.seconds(0.3);
    r.off(0);
    r.on(0, 60.0f, true, false);
    const size_t at = r.L.size();
    r.seconds(0.3);
    CHECK(std::fabs(pitchOf(freqOf(r.L, at + size_t(0.05 * kFs), at + size_t(0.25 * kFs))) - 60.0) < 0.05);
    Vals l = plain();
    l[vt::kGlideMode] = float(vt::kLegato);
    l[vt::kAmpA] = 0.5f;
    Rig g(l);
    g.on(0, 48.0f);
    g.seconds(1.0);
    const float before = g.v[0].level;
    g.on(0, 55.0f, true, true);
    g.run(1);
    CHECK(g.v[0].level >= before - 1e-3f);   // no new attack from 0
}

TEST(vtype_reused_sounding_voice_does_not_click) {
    if (pending(1)) return;
    Vals v = initValues();
    v[vt::kAmpA] = 0.05f;   // a gentle attack: the test is about the voice's state, not the patch's snap
    v[vt::kAmpR] = 1.0f;
    v[vt::kF1Env] = 0.0f;
    Rig r(v);
    r.on(0, 57.0f);
    r.seconds(0.4);
    r.off(0);
    r.seconds(0.1);
    const size_t at = r.L.size();
    auto maxStep = [&r](size_t a, size_t b) {
        double m = 0;
        for (size_t i = a + 1; i < b; ++i) m = std::max(m, double(std::fabs(r.L[i] - r.L[i - 1])));
        return m;
    };
    const double before = maxStep(at - 20 * kBlock, at);
    r.on(0, 52.0f, true, false);
    r.run(4);
    const double jump = maxStep(at - 1, at + 2 * kBlock);
    r.seconds(0.3);
    const double after = maxStep(r.L.size() - 20 * kBlock, r.L.size());
    if (!(jump < 2.0 * std::max(before, after) + 1e-4)) std::printf("       step %.4f (steady %.4f / %.4f)\n", jump, before, after);
    CHECK(jump < 2.0 * std::max(before, after) + 1e-4);
}

TEST(vtype_voice_level_and_volume) {
    if (pending(1)) return;
    Vals v = plain();
    Rig r(v);
    r.on(0, 60.0f);
    r.run(4);
    CHECK(r.v[0].level > 0.99f && r.v[0].level <= 1.0f);
    Vals a = initValues(), b = initValues();
    a[vt::kVolume] = 0.0f;
    b[vt::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

TEST(vtype_level_matches_the_synth_conventions) {
    if (pending(1)) return;
    BankSlot s(1);
    for (int k = 0; k < 20; ++k) {
        s.on(0, 45 + 3 * (k % 8));
        s.run(100);
    }
    BankSlot c(4);
    for (int rep = 0; rep < 6; ++rep) {
        for (int k = 0; k < 4; ++k) c.on(k, 57 + 4 * k - 12 * (rep & 1));
        c.run(200);
    }
#if defined(OT_MOD_ACD1)
    BankSlot a(1, nullptr, kSynthEngineAcd);
    for (int j = 0; j < 20; ++j) {
        a.on(0, 45 + 3 * (j % 8));
        a.run(100);
    }
    std::printf("  V-Type Init at the slot gain: peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", db(s.pk), db(a.pk), db(c.pk));
    CHECK(std::fabs(db(s.pk) - db(a.pk)) < 3.0);
#else
    std::printf("  V-Type Init at the slot gain: peak %.1f dBFS, 4-voice chords %.1f dBFS\n", db(s.pk), db(c.pk));
#endif
    CHECK(db(s.pk) < -6.0 && db(s.pk) > -20.0);
    CHECK(db(c.pk) < -1.0);
    CHECK_EQ(s.clips + c.clips, 0L);
}

// --- M2: filters, routings, Drive, Env Amt, Key Track ----------------------------------

namespace {
// Osc 2's sine at hz through filter `which` (1: Single, 2: Parallel at Flt
// Balance +1) of type t, against the same probe through an open LP12.
double measuredGain(int which, int t, double fc, double reso, double hz) {
    Vals v = probe();
    const int base = which == 1 ? vt::kF1Type : vt::kF2Type;
    v[size_t(base)] = float(t);
    v[size_t(base + 1)] = float(fc);
    v[size_t(base + 2)] = float(reso);
    v[size_t(base + 3)] = 0.0f;
    v[size_t(base + 4)] = 0.0f;
    if (which == 2) {
        v[vt::kRouting] = float(vt::kParallel);
        v[vt::kFltBalance] = 1.0f;
    }
    const float pitch = float(pitchOf(hz));
    return level(held(v, pitch, 0.5), hz);
}
double openGain(double hz) {
    return level(held(probe(), float(pitchOf(hz)), 0.5), hz) / std::abs(typeH(vt::kLp12, 20000.0, 0.0, hz));
}
}  // namespace

TEST(vtype_filter_responses_both_filters) {
    if (pending(2)) return;
    int bad = 0, n = 0;
    const double cutoffs[12] = {80, 150, 300, 500, 800, 1200, 2000, 3000, 5000, 8000, 12000, 16000};
    for (int which = 1; which <= 2; ++which)
        for (int t = 0; t < 4; ++t)
            for (double reso : {0.0, 0.3, 0.6, 0.9})
                for (double fc : cutoffs) {
                    // Probes an octave under, at and an octave over the cutoff (inside the band).
                    for (double r : {0.5, 1.0, 2.0}) {
                        const double hz = std::clamp(fc * r, 40.0, 15000.0);
                        const double want = std::abs(typeH(t, fc, reso, hz)) * openGain(hz);
                        const double got = measuredGain(which, t, fc, reso, hz);
                        ++n;
                        if (std::fabs(db(got / want)) > 1.0) {
                            ++bad;
                            if (bad < 20)
                                std::printf("       filter %d type %d fc %.0f reso %.1f at %.0f Hz: %.2f dB\n", which, t, fc, reso, hz,
                                            db(got / want));
                        }
                    }
                }
    std::printf("  filter probes: %d of %d off\n", bad, n);
    CHECK_EQ(bad, 0);
}

TEST(vtype_filter_peak_frequency) {
    if (pending(2)) return;
    // A resonant LP12 / BP12 peaks at the cutoff (2 %): the loudest of a sweep of probes.
    for (int t : {int(vt::kLp12), int(vt::kBp12)})
        for (double fc : {300.0, 1000.0, 3000.0}) {
            double best = 0.0, at = 0.0;
            for (int i = -20; i <= 20; ++i) {
                const double hz = fc * std::pow(2.0, double(i) / 240.0);
                const double g = measuredGain(1, t, fc, 0.9, hz);
                if (g > best) {
                    best = g;
                    at = hz;
                }
            }
            CHECK(std::fabs(at / fc - 1.0) < 0.02);
        }
}

TEST(vtype_routings) {
    if (pending(2)) return;
    struct Pair {
        int t1, t2;
        double f1, f2;
    };
    const Pair pairs[3] = {{vt::kLp24, vt::kHp12, 2000.0, 300.0}, {vt::kLp12, vt::kBp12, 800.0, 1600.0}, {vt::kBp12, vt::kLp12, 1200.0, 5000.0}};
    for (const Pair& pr : pairs) {
        for (int route = 0; route < 4; ++route) {
            for (double hz : {250.0, 1000.0, 3000.0}) {
                Vals v = probe();
                v[vt::kF1Type] = float(pr.t1);
                v[vt::kF1Cutoff] = float(pr.f1);
                v[vt::kF2Type] = float(pr.t2);
                v[vt::kF2Cutoff] = float(pr.f2);
                v[vt::kF2Reso] = 0.2f;
                v[vt::kF1Reso] = 0.2f;
                v[vt::kRouting] = float(route);
                v[vt::kFltBalance] = route == vt::kSerial ? 1.0f : 0.0f;
                v[vt::kSplitWidth] = 0.0f;
                const std::complex<double> h1 = typeH(pr.t1, pr.f1, 0.2, hz), h2 = typeH(pr.t2, pr.f2, 0.2, hz);
                std::complex<double> want;
                const double eq = std::sqrt(0.5);
                switch (route) {
                    case vt::kSingle: want = h1; break;
                    case vt::kSerial: want = h1 * h2; break;
                    case vt::kParallel: want = eq * h1 + eq * h2; break;
                    default: want = 0.5 * eq * h2; break;   // Split: osc 2 is on filter 2's side only, Width 0
                }
                const double got = level(held(v, float(pitchOf(hz)), 0.5), hz) / openGain(hz);
                const double d = db(got / std::abs(want));
                if (!(std::fabs(d) < 1.0)) std::printf("       route %d pair %d/%d at %.0f Hz: %.2f dB\n", route, pr.t1, pr.t2, hz, d);
                CHECK(std::fabs(d) < 1.0);
            }
        }
    }
    // Balance ends: Serial -1 = the tap after Drive (filter 1 only); Parallel -1 / +1 = one filter.
    const double hz = 1000.0;
    Vals v = probe();
    v[vt::kF1Type] = float(vt::kLp12);
    v[vt::kF1Cutoff] = 500.0f;
    v[vt::kF2Type] = float(vt::kHp12);
    v[vt::kF2Cutoff] = 4000.0f;
    const double h1 = std::abs(typeH(vt::kLp12, 500.0, 0.0, hz)), h2 = std::abs(typeH(vt::kHp12, 4000.0, 0.0, hz));
    const double og = openGain(hz);
    for (int route : {int(vt::kSerial), int(vt::kParallel)})
        for (float b : {-1.0f, 1.0f}) {
            v[vt::kRouting] = float(route);
            v[vt::kFltBalance] = b;
            const double got = level(held(v, float(pitchOf(hz)), 0.5), hz) / og;
            const double want = b < 0.0f ? h1 : (route == vt::kSerial ? h1 * h2 : h2);
            CHECK(std::fabs(db(got / want)) < 1.0);
        }
}

TEST(vtype_drive_curves) {
    if (pending(2)) return;
    // The fundamental after Drive against 2^(-2.4 d) curve(2^(4 d) x) for a
    // sine through an open filter 1; Drive 0 is bit-identical to no Drive.
    const double hz = 220.0;
    Vals v = probe();
    const double g0 = level(held(v, float(pitchOf(hz)), 0.5), hz);
    int bad = 0;
    for (int c = 0; c < 3; ++c) {
        const double levels[8][2] = {{1.0, 0.1}, {1.0, 0.3}, {1.0, 0.5}, {1.0, 0.8}, {1.0, 1.0}, {0.5, 0.6}, {0.25, 1.0}, {0.7, 0.4}};
        for (const auto& lv : levels) {
            // The level into Drive: filter 1 (LP12) lowered until |H| at the probe is lv[0].
            double lo = 20.0, hi = 20000.0;
            for (int it = 0; it < 60; ++it) {
                const double mid = std::sqrt(lo * hi);
                (std::abs(typeH(vt::kLp12, mid, 0.0, hz)) < lv[0] ? lo : hi) = mid;
            }
            Vals d = v;
            d[vt::kF1Cutoff] = float(hi);
            d[vt::kDrive] = float(lv[1]);
            d[vt::kCurve] = float(c);
            const double got = level(held(d, float(pitchOf(hz)), 0.5), hz) / g0;
            const double want = driveGain(c, lv[1], std::abs(typeH(vt::kLp12, double(float(hi)), 0.0, hz)));
            if (std::fabs(db(got / want)) > 0.5) {
                ++bad;
                std::printf("       curve %d level %.2f drive %.1f: %.2f dB\n", c, lv[0], lv[1], db(got / want));
            }
        }
    }
    CHECK_EQ(bad, 0);
    Vals a = initValues();
    a[vt::kO2Wave] = float(vt::kO2Pulse);
    a[vt::kBalance] = 0.0f;
    a[vt::kRouting] = float(vt::kSerial);
    Vals b = a;
    b[vt::kCurve] = float(vt::kFold);   // the curve does nothing at Drive 0
    CHECK(play(a) == play(b));
}

namespace {
// The cutoff (Hz) filter 1 (LP12, no resonance) has, from a probe at hz.
double cutoffFrom(double gain, double hz) {
    const double k = double(vt::resoK(0.0f)), w = std::tan(kPi * hz / kFs);
    const double inv = 1.0 / (gain * gain), b = k * k - 2.0;
    const double u = 0.5 * (-b + std::sqrt(b * b - 4.0 * (1.0 - inv)));
    const double W = std::sqrt(std::max(u, 1e-12));
    return std::atan(w / W) * kFs / kPi;
}
}  // namespace

TEST(vtype_env_amt_key_track_and_performance_cutoff) {
    if (pending(2)) return;
    int bad = 0;
    auto check = [&bad](const char* what, double want, double got) {
        if (std::fabs(got / want - 1.0) > 0.03) {
            ++bad;
            std::printf("       %s: cutoff %.1f Hz, want %.1f\n", what, got, want);
        }
    };
    for (int which = 1; which <= 2; ++which) {
        const int base = which == 1 ? vt::kF1Type : vt::kF2Type;
        for (double amt : {-0.25, -0.1, 0.0, 0.1, 0.3}) {
            // Env 1 held at its sustain 0.5: 8 x amt x 0.5 octaves.
            Vals v = probe();
            v[vt::kE1A] = 0.0f;
            v[vt::kE1D] = 0.0f;
            v[vt::kE1S] = 0.5f;
            v[size_t(base)] = float(vt::kLp12);
            v[size_t(base + 1)] = 1000.0f;
            v[size_t(base + 2)] = 0.0f;
            v[size_t(base + 3)] = float(amt);
            v[size_t(base + 4)] = 0.0f;
            if (which == 2) {
                v[vt::kRouting] = float(vt::kParallel);
                v[vt::kFltBalance] = 1.0f;
            }
            const double want = 1000.0 * std::pow(2.0, 8.0 * amt * 0.5);
            const double got = cutoffFrom(level(held(v, float(pitchOf(want)), 0.4), want) / openGain(want), want);
            check("env amt", want, got);
        }
        for (int key : {24, 48, 60, 84, 96}) {
            Vals v = probe();
            v[size_t(base)] = float(vt::kLp12);
            v[size_t(base + 1)] = 1000.0f;
            v[size_t(base + 2)] = 0.0f;
            v[size_t(base + 3)] = 0.0f;
            v[size_t(base + 4)] = 0.5f;
            if (which == 2) {
                v[vt::kRouting] = float(vt::kParallel);
                v[vt::kFltBalance] = 1.0f;
            }
            const double want = 1000.0 * std::pow(2.0, 0.5 * (key - 60) / 12.0);
            v[vt::kO2Semi] = float(std::round(pitchOf(want) - key));
            const double hz = hzOf(key + double(v[vt::kO2Semi]));
            const double got = cutoffFrom(level(held(v, float(key), 0.4), hz) / openGain(hz), hz);
            check("key track", want, got);
        }
        for (float oct : {-1.5f, 1.0f}) {
            Vals v = probe();
            v[size_t(base)] = float(vt::kLp12);
            v[size_t(base + 1)] = 1000.0f;
            v[size_t(base + 2)] = 0.0f;
            v[size_t(base + 3)] = 0.0f;
            v[size_t(base + 4)] = 0.0f;
            if (which == 2) {
                v[vt::kRouting] = float(vt::kParallel);
                v[vt::kFltBalance] = 1.0f;
            }
            const double want = 1000.0 * std::pow(2.0, double(oct));
            Rig r(v);
            r.v[0].cutoffOct = oct;
            r.on(0, float(pitchOf(want)));
            r.seconds(0.5);
            const double got = cutoffFrom(level(slice(r.L, 0.1, 0.5), want) / openGain(want), want);
            check("performance cutoff", want, got);
        }
    }
    CHECK_EQ(bad, 0);
}

// --- M3: osc 2, sync, ring, noise, Balance, Split -------------------------------------

TEST(vtype_osc2_waves_and_pulse_width) {
    if (pending(3)) return;
    const double hz = 220.0;
    for (int w = 0; w < 4; ++w) {
        const Buf x = held(probe(w), float(pitchOf(hz)), 1.0);
        for (int n = 1; n <= 5; ++n) {
            double want;
            switch (w) {
                case vt::kO2Saw: want = 2.0 / (kPi * n); break;
                case vt::kO2Pulse: want = 4.0 / (kPi * n) * std::fabs(std::sin(kPi * n * 0.5)); break;
                case vt::kO2Tri: want = (n & 1) ? 8.0 / (kPi * kPi * n * n) : 0.0; break;
                default: want = n == 1 ? 1.0 : 0.0; break;
            }
            want *= outGain() * std::abs(typeH(vt::kLp12, 20000.0, 0.0, n * hz));
            const double got = level(x, n * hz);
            if (want < 1e-3 * outGain()) {
                if (!(got < 0.01 * outGain())) std::printf("       wave %d h%d: %.4f, want none\n", w, n, got);
                CHECK(got < 0.01 * outGain());
            } else {
                if (!(std::fabs(db(got / want)) < 0.5)) std::printf("       wave %d h%d: %.2f dB\n", w, n, db(got / want));
                CHECK(std::fabs(db(got / want)) < 0.5);
            }
        }
    }
    for (double pw : {0.1, 0.5, 0.9}) {
        Vals v = probe(vt::kO2Pulse);
        v[vt::kO2Pw] = float(pw);
        const Buf x = held(v, float(pitchOf(hz)), 1.0);
        for (int n = 1; n <= 4; ++n) {
            const double want = outGain() * 4.0 / (kPi * n) * std::fabs(std::sin(kPi * n * pw));
            if (want < 0.05 * outGain()) continue;
            CHECK(std::fabs(db(level(x, n * hz) / want)) < 0.5);
        }
    }
    // Semi and Fine against the note.
    Vals s = probe(vt::kO2Saw);
    s[vt::kO2Semi] = -12.0f;
    s[vt::kO2Fine] = 25.0f;
    CHECK(std::fabs(cents(freqOf(held(s, 72.0f)), hzOf(60.25))) < 1.0);
}

TEST(vtype_sync_and_ring) {
    if (pending(3)) return;
    // Sync: osc 2 (a non-harmonic ratio) restarts on the centre saw's wraps:
    // its output repeats at the centre's period (a line at f0, none without
    // Sync), and its spectrum moves up with Semi (centroid of 20 lines).
    const float key = 48.0f;
    const double f0 = hzOf(key);
    double first = 0.0, last = 0.0;
    for (float semi : {5.0f, 10.0f, 17.0f}) {
        Vals v = probe(vt::kO2Saw);
        v[vt::kO2Semi] = semi;
        const double free = level(held(v, key, 1.0), f0);
        v[vt::kO2Sync] = 1.0f;
        const Buf x = held(v, key, 1.0);
        const double c = centroid(x, f0, 20);
        if (!(level(x, f0) > 0.05 * outGain() && free < 0.01 * outGain()))
            std::printf("       sync semi %.0f: line %.4f (free %.4f)\n", semi, level(x, f0), free);
        CHECK(level(x, f0) > 0.05 * outGain());
        CHECK(free < 0.01 * outGain());
        CHECK(c > last);
        if (first == 0.0) first = c;
        last = c;
    }
    std::printf("  sync centroid: %.2f .. %.2f lines\n", first, last);
    CHECK(last > 1.1 * first);
    // Ring: R x centre saw x osc 2 sine: sidebands at f2 +- f1 at R / pi (1 dB).
    Vals r = probe(vt::kO2Sine);
    r[vt::kRing] = 0.8f;
    r[vt::kO2Semi] = 18.0f;   // not a harmonic ratio: every sideband on its own line
    const double f2 = hzOf(key + 18.0);
    const Buf x = held(r, key, 1.0);
    for (double f : {f2 + f0, f2 - f0}) {
        const double want = outGain() * 0.8 / kPi * std::abs(typeH(vt::kLp12, 20000.0, 0.0, f));
        const double d = db(level(x, f) / want);
        if (!(std::fabs(d) < 1.0)) std::printf("       ring sideband %.0f Hz: %.2f dB\n", f, d);
        CHECK(std::fabs(d) < 1.0);
    }
}

TEST(vtype_balance_noise_and_colour) {
    if (pending(3)) return;
    // Balance b: Swarm at sqrt((1 - b) / 2), osc 2 at sqrt((1 + b) / 2).
    const float key = 48.0f;
    Vals base = plain();
    base[vt::kO2Wave] = float(vt::kO2Sine);
    base[vt::kO2Semi] = 7.0f;   // osc 2 a fifth up: its own line
    const double f1 = hzOf(key), f2 = hzOf(key + 7.0);
    for (double b : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        Vals v = base;
        v[vt::kBalance] = float(b);
        const Buf x = held(v, key, 1.0);
        const double ws = std::sqrt((1.0 - b) / 2.0), w2 = std::sqrt((1.0 + b) / 2.0);
        const double hs = std::abs(typeH(vt::kLp12, 20000.0, 0.0, f1)), h2 = std::abs(typeH(vt::kLp12, 20000.0, 0.0, f2));
        if (ws > 0.01) CHECK(std::fabs(db(level(x, f1) / (outGain() * ws * 2.0 / kPi * hs))) < 0.3);
        if (w2 > 0.01) CHECK(std::fabs(db(level(x, f2) / (outGain() * w2 * h2))) < 0.3);
        if (ws < 0.01) CHECK(level(x, f1) < 1e-4);
        if (w2 < 0.01) CHECK(level(x, f2) < 1e-4);
    }
    // Noise N: white at 0.35 N (RMS 0.35 N / sqrt 3 before the filter).
    auto noise = [](double n, double colour) {
        Vals v = plain();
        v[vt::kBalance] = 1.0f;   // the Swarm off, osc 2 at 0 below
        v[vt::kO2Wave] = float(vt::kO2Sine);
        v[vt::kNoise] = float(n);
        v[vt::kColour] = float(colour);
        v[vt::kF1Type] = float(vt::kLp12);
        v[vt::kF1Cutoff] = 20000.0f;
        return v;
    };
    for (double n : {0.25, 0.5, 1.0}) {
        Vals v = noise(n, 0.0);
        v[vt::kO2Semi] = -36.0f;
        Rig r(v);
        r.on(0, 0.0f);   // osc 2 at about 1 Hz: negligible
        r.seconds(2.1);
        const Buf x = slice(r.L, 0.1, 2.1);
        const double band = bandPower(x, 500.0, 4000.0);   // flat, under the filter's knee
        const double want = std::pow(outGain() * 0.35 * n, 2.0) / 3.0 * 6.0 / double(x.size());   // per-line power of white noise (Hann)
        const double d = 10.0 * std::log10(band / want);
        if (!(std::fabs(d) < 1.0)) std::printf("       noise %.2f: %.2f dB\n", n, d);
        CHECK(std::fabs(d) < 1.0);
    }
    // Colour: the low / high band energy ratio follows the one-pole.
    for (double c : {-1.0, -0.5, 0.5, 1.0}) {
        Vals v = noise(1.0, c);
        v[vt::kO2Semi] = -36.0f;
        Rig r(v);
        r.on(0, 0.0f);
        r.seconds(2.1);
        const Buf x = slice(r.L, 0.1, 2.1);
        const double lo = bandPower(x, 150.0, 400.0), hi = bandPower(x, 6000.0, 9000.0);
        const double fc = c < 0.0 ? double(vt::kColourHi) * std::pow(double(vt::kColourLo / vt::kColourHi), -c)
                                  : double(vt::kColourLo) * std::pow(double(vt::kColourHi / vt::kColourLo), c);
        const double a = 1.0 - std::exp(-2.0 * kPi * fc / kFs);
        auto h2 = [&](double f) {
            const std::complex<double> z = std::polar(1.0, -2.0 * kPi * f / kFs);
            const std::complex<double> lp = a / (1.0 - (1.0 - a) * z);
            return std::norm(c < 0.0 ? lp : 1.0 - lp);
        };
        double wl = 0.0, wh = 0.0;
        for (int k = 0; k < 160; ++k) {
            wl += h2(150.0 + 250.0 * (k + 0.5) / 160.0);
            wh += h2(6000.0 + 3000.0 * (k + 0.5) / 160.0) * std::norm(typeH(vt::kLp12, 20000.0, 0.0, 6000.0 + 3000.0 * (k + 0.5) / 160.0));
        }
        const double d = 10.0 * std::log10((lo / hi) / (wl / wh));
        if (!(std::fabs(d) < 1.0)) std::printf("       colour %.1f: tilt off by %.2f dB\n", c, d);
        CHECK(std::fabs(d) < 1.0);
    }
}

TEST(vtype_split_and_width) {
    if (pending(3)) return;
    // Split: Swarm + sub through filter 1 on the left, osc 2 + noise through
    // filter 2 on the right; at Width 1 neither leaks (below -60 dB).
    const float key = 48.0f;
    Vals v = plain();
    v[vt::kRouting] = float(vt::kSplit);
    v[vt::kBalance] = 0.0f;
    v[vt::kFltBalance] = 0.0f;
    v[vt::kO2Wave] = float(vt::kO2Sine);
    v[vt::kO2Semi] = 7.0f;
    v[vt::kF2Type] = float(vt::kLp12);
    v[vt::kF2Cutoff] = 20000.0f;
    v[vt::kSplitWidth] = 1.0f;
    const double f1 = hzOf(key), f2 = hzOf(key + 7.0);
    Rig r(v);
    r.on(0, key);
    r.seconds(1.1);
    const Buf l = slice(r.L, 0.1, 1.1), rr = slice(r.R, 0.1, 1.1);
    CHECK(db(level(l, f2) / level(rr, f2)) < -60.0);
    CHECK(db(level(rr, f1) / level(l, f1)) < -60.0);
    // Width w: filter 1 at pan -w, filter 2 at +w (linear): L / R = (1 + w) / (1 - w).
    for (double w : {0.25, 0.5, 0.75}) {
        v[vt::kSplitWidth] = float(w);
        Rig s(v);
        s.on(0, key);
        s.seconds(1.1);
        const Buf a = slice(s.L, 0.1, 1.1), b = slice(s.R, 0.1, 1.1);
        CHECK(std::fabs(db(level(a, f1) / level(b, f1)) - db((1.0 + w) / (1.0 - w))) < 0.3);
        CHECK(std::fabs(db(level(b, f2) / level(a, f2)) - db((1.0 + w) / (1.0 - w))) < 0.3);
    }
    // Width 0 is the mono sum, bit for bit; every other routing is mono.
    v[vt::kSplitWidth] = 0.0f;
    Rig m(v);
    m.on(0, key);
    m.seconds(0.5);
    CHECK(m.L == m.R);
    CHECK(rms(m.L) > 0.01);
    for (int route : {int(vt::kSingle), int(vt::kSerial), int(vt::kParallel)}) {
        Vals u = v;
        u[vt::kRouting] = float(route);
        u[vt::kSplitWidth] = 1.0f;
        Rig q(u);
        q.on(0, key);
        q.seconds(0.3);
        CHECK(q.L == q.R);
    }
}

// --- M4: envelopes, LFOs, matrix, ramps, random sets ----------------------------------

namespace {
// The level of a held probe per block, with envelope `which` (0 Amp, 1 Env 1,
// 2 Env 2) at A / D / S / R; Env 1 and 2 through the matrix's Volume at Amt -1
// (gain 1 - env), so the measured envelope is 1 - level.
struct EnvRun {
    std::vector<double> e;   // envelope per block
    size_t offAt;            // block of the note off
};
EnvRun envRun(int which, double a, double d, double s, double rel, double hold) {
    Vals v = probe();
    v[vt::kO2Semi] = 0.0f;
    const int base = which == 0 ? vt::kAmpA : (which == 1 ? vt::kE1A : vt::kE2A);
    v[size_t(base)] = float(a);
    v[size_t(base + 1)] = float(d);
    v[size_t(base + 2)] = float(s);
    v[size_t(base + 3)] = float(rel);
    if (which > 0) {
        v[vt::kAmpA] = 0.0f;
        v[vt::kAmpS] = 1.0f;
        v[vt::kAmpR] = 10.0f;
        v[size_t(vt::modSrc(0))] = float(which == 1 ? vt::kSrcEnv1 : vt::kSrcEnv2);
        v[size_t(vt::modDest(0))] = float(vt::kDstVolume);
        v[size_t(vt::modAmt(0))] = -1.0f;
    }
    Rig r(v);
    r.on(0, 81.0f);   // 880 Hz: 50 frames a period, a block's peak is the level
    r.seconds(hold);
    EnvRun out;
    out.offAt = r.L.size() / kBlock;
    r.off(0);
    r.seconds(rel * 1.3 + 0.05);
    const std::vector<double> pk = blockPeaks(r.L);
    double top = 0.0;
    if (which == 0) {
        for (double x : pk) top = std::max(top, x);
        for (double x : pk) out.e.push_back(x / top);
    } else {
        Vals ref = probe();
        ref[vt::kAmpA] = 0.0f;
        ref[vt::kAmpS] = 1.0f;
        const double g = peak(held(ref, 81.0f, 0.2));
        // After the note off the amp envelope releases too (10 s): divided out.
        const double amp = std::exp(-4.6 * double(kBlock) / kFs / 10.0);
        for (size_t b = 0; b < pk.size(); ++b)
            out.e.push_back(1.0 - pk[b] / (g * (b > out.offAt ? std::pow(amp, double(b - out.offAt)) : 1.0)));
    }
    return out;
}
double timeTo(const std::vector<double>& e, size_t from, double lvl, bool falling) {
    for (size_t i = from; i < e.size(); ++i)
        if (falling ? e[i] <= lvl : e[i] >= lvl) return double(i - from + 1) * kBlock / kFs;
    return 1e9;
}
}  // namespace

TEST(vtype_envelopes) {
    if (pending(4)) return;
    int bad = 0;
    auto near = [&bad](const char* what, int which, double want, double got) {
        // Block-rate envelopes read by block peaks: 3 % and a few blocks.
        if (std::fabs(got - want) > 0.03 * want + 6.0 * kBlock / kFs) {
            ++bad;
            std::printf("       env %d %s: %.3f s, want %.3f\n", which, what, got, want);
        }
    };
    for (int which = 0; which < 3; ++which)
        for (double t : {0.2, 0.5, 1.0, 2.0, 4.0}) {
            // Attack: reaches the top after t.
            EnvRun a = envRun(which, t, 10.0, 1.0, 0.05, t + 0.2);
            near("attack", which, t, timeTo(a.e, 0, 0.999, false));
            // Decay: 1 % of the way to the sustain 0.4 after t.
            EnvRun d = envRun(which, 0.0, t, 0.4, 0.05, t + 0.3);
            near("decay", which, t, timeTo(d.e, 0, 0.4 + 0.006, true));
            const double sus = d.e[d.offAt - 2];
            if (std::fabs(sus - 0.4) > 0.01) {
                ++bad;
                std::printf("       env %d sustain %.3f\n", which, sus);
            }
            // Release: 1 % of the sustain after t.
            EnvRun r = envRun(which, 0.0, 0.0, 0.8, t, 0.2);
            near("release", which, t, timeTo(r.e, r.offAt, 0.008, true));
        }
    CHECK_EQ(bad, 0);
}

namespace {
// LFO 1 (or 2) through the matrix's Volume at Amt 0.5 on a held probe: the
// LFO value per block, 2 (level - 1).
std::vector<double> lfoRun(int lfo, int wave, double rate, int sync, double bpm, double seconds, bool key = false) {
    Vals v = probe();
    const int base = vt::kL1Rate + vt::kLfoStride * lfo;
    v[size_t(base)] = float(rate);
    v[size_t(base + 1)] = float(wave);
    v[size_t(base + 2)] = float(sync);
    v[size_t(base + 3)] = key ? 1.0f : 0.0f;
    v[size_t(vt::modSrc(1))] = float(vt::kSrcLfo1 + lfo);
    v[size_t(vt::modDest(1))] = float(vt::kDstVolume);
    v[size_t(vt::modAmt(1))] = 0.5f;
    Rig r(v);
    r.bpm = float(bpm);
    r.on(0, 81.0f);
    r.seconds(seconds);
    Vals ref = probe();
    const double g = peak(held(ref, 81.0f, 0.2));
    std::vector<double> out;
    for (double x : blockPeaks(r.L, 4 * kBlock)) out.push_back(2.0 * (x / g - 1.0));
    return out;
}
// Cycles per second of a ramp-up LFO trace (phase unwrapped, least squares).
double rampRate(const std::vector<double>& y) {
    std::vector<double> ph;
    double turns = 0.0, last = -1.0;
    for (double v : y) {
        const double p = 0.5 * (v + 1.0);
        if (last >= 0.0 && p < last - 0.5) turns += 1.0;
        last = p;
        ph.push_back(p + turns);
    }
    const double n = double(ph.size());
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < ph.size(); ++i) {
        const double x = double(i) * kBlock / kFs;
        sx += x;
        sy += ph[i];
        sxx += x * x;
        sxy += x * ph[i];
    }
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}
}  // namespace

TEST(vtype_lfo_rates_shapes_divisions_and_key_sync) {
    if (pending(4)) return;
    for (int lfo = 0; lfo < 2; ++lfo)
        for (double rate : {0.5, 2.0, 7.0}) {
            const double got = rampRate(lfoRun(lfo, vt::kRampUp, rate, 0, 120.0, 4.0));
            if (!(std::fabs(got / rate - 1.0) < 0.02)) std::printf("       lfo %d rate %.2f: %.3f Hz\n", lfo, rate, got);
            CHECK(std::fabs(got / rate - 1.0) < 0.02);
        }
    // Shapes against their laws (phase 0 at the note: Key Sync).
    for (int w = 0; w < 6; ++w) {
        const double rate = 1.0;
        const std::vector<double> y = lfoRun(0, w, rate, 0, 120.0, 2.0, true);
        double err = 0.0, sum = 0.0;
        for (size_t i = 2; i + 2 < y.size(); ++i) {
            const double ph = std::fmod((double(i) + 4.5) * kBlock / kFs * rate, 1.0);
            double want;
            switch (w) {
                case vt::kTri: want = ph < 0.25 ? 4 * ph : (ph < 0.75 ? 2 - 4 * ph : 4 * ph - 4); break;
                case vt::kSine: want = std::sin(2 * kPi * ph); break;
                case vt::kRampUp: want = 2 * ph - 1; break;
                case vt::kRampDown: want = 1 - 2 * ph; break;
                case vt::kSquare: want = ph < 0.5 ? 1 : -1; break;
                default: want = y[i]; break;
            }
            if (w == vt::kSquare && (std::fabs(ph - 0.5) < 0.01 || ph < 0.01 || ph > 0.99)) continue;
            if ((w == vt::kRampUp || w == vt::kRampDown) && (ph < 0.01 || ph > 0.99)) continue;
            err += (y[i] - want) * (y[i] - want);
            sum += want * want;
        }
        if (w == vt::kSampleHold) {
            // Steps: one value per cycle, each within -1..1, not all equal.
            std::set<int> vals;
            for (double v : y) vals.insert(int(std::round(v * 1000.0)));
            CHECK(vals.size() >= 2);
            continue;
        }
        const double rel = std::sqrt(err / sum);
        if (!(rel < 0.02)) std::printf("       wave %d: shape error %.3f\n", w, rel);
        CHECK(rel < 0.02);
    }
    // Tempo divisions at 90 / 120 / 174 BPM.
    int bad = 0;
    for (double bpm : {90.0, 120.0, 174.0})
        for (int s = 1; s <= 15; ++s) {
            const double want = bpm / 60.0 / double(vt::kSyncBeats[s]);
            const double secs = std::max(2.0, 0.6 / want);
            const double got = rampRate(lfoRun(0, vt::kRampUp, 5.0, s, bpm, secs));
            if (std::fabs(got / want - 1.0) > 0.02) {
                ++bad;
                std::printf("       bpm %.0f sync %d: %.4f Hz, want %.4f\n", bpm, s, got, want);
            }
        }
    CHECK_EQ(bad, 0);
    // Key Sync: two notes 0.37 s apart start the LFO alike; free running they don't.
    for (int key = 0; key < 2; ++key) {
        Vals v = probe();
        v[vt::kL1Rate] = 1.3f;
        v[vt::kL1Wave] = float(vt::kRampUp);
        v[vt::kL1Key] = float(key);
        v[size_t(vt::modSrc(0))] = float(vt::kSrcLfo1);
        v[size_t(vt::modDest(0))] = float(vt::kDstVolume);
        v[size_t(vt::modAmt(0))] = 0.5f;
        Rig r(v);
        r.on(0, 81.0f);
        r.seconds(0.6);
        const Buf a = slice(r.L, 0.0, 0.3);
        r.off(0);
        r.alive[0] = false;
        r.L.clear();
        r.run(int(0.37 * kFs / kBlock));
        r.L.clear();
        r.on(1, 81.0f);
        r.seconds(0.3);
        const Buf b = slice(r.L, 0.0, 0.3);
        const double d = rmsDiff(a, b) / rms(a);
        if (key) CHECK(d < 0.01);
        else CHECK(d > 0.05);
    }
}

namespace {
// The matrix value Amt x source on a dest against the patch with that
// parameter moved by the scale table (Amp Env / Env 1 / Env 2 held at their
// sustain 0.6, LFOs a slow square at +1 from the note).
Vals matrixContext() {
    Vals c = initValues();
    c[vt::kPhase] = float(vt::kPhaseReset);
    c[vt::kCount] = 2.0f;
    c[vt::kDetune] = 0.4f;
    c[vt::kSwarmMix] = 0.5f;
    c[vt::kBalance] = -0.2f;
    c[vt::kO2Wave] = float(vt::kO2Pulse);
    c[vt::kO2Pw] = 0.4f;
    c[vt::kO2Semi] = 7.0f;
    c[vt::kRouting] = float(vt::kParallel);
    c[vt::kFltBalance] = 0.0f;
    c[vt::kF1Type] = float(vt::kLp12);
    c[vt::kF1Cutoff] = 900.0f;
    c[vt::kF1Reso] = 0.3f;
    c[vt::kF1Env] = 0.0f;
    c[vt::kF1Key] = 0.0f;
    c[vt::kF2Type] = float(vt::kBp12);
    c[vt::kF2Cutoff] = 2500.0f;
    c[vt::kF2Reso] = 0.3f;
    c[vt::kDrive] = 0.35f;
    for (int base : {int(vt::kAmpA), int(vt::kE1A), int(vt::kE2A)}) {
        c[size_t(base)] = 0.0f;
        c[size_t(base + 1)] = 0.0f;
        c[size_t(base + 2)] = 0.6f;
    }
    for (int l = 0; l < 2; ++l) {
        const int b = vt::kL1Rate + vt::kLfoStride * l;
        c[size_t(b)] = 0.05f;
        c[size_t(b + 1)] = float(vt::kSquare);
        c[size_t(b + 3)] = 1.0f;
    }
    return c;
}
// Relative distance of two magnitude spectra (80 log-spaced lines, 60 Hz .. 6 kHz).
double specDiff(const Buf& a, const Buf& b) {
    double num = 0.0, den = 0.0;
    for (int k = 0; k < 80; ++k) {
        const double f = 60.0 * std::pow(100.0, k / 79.0);
        const double x = level(a, f), y = level(b, f);
        num += (x - y) * (x - y);
        den += y * y;
    }
    return std::sqrt(num / std::max(den, 1e-12));
}
double srcValue(int src) { return src <= vt::kSrcEnv2 ? 0.6 : 1.0; }
// The static patch for dest += value (false: Volume, compared by level).
void applyStatic(Vals& c, int dest, double v) {
    auto add = [&c](int i, double x) { c[size_t(i)] = float(std::clamp(double(c[size_t(i)]) + x, double(range(i)->min), double(range(i)->max))); };
    switch (dest) {
        case vt::kDstPitch: c[vt::kKeyShift] += float(v); break;
        case vt::kDstO2Pitch: c[vt::kO2Semi] += float(v); break;
        case vt::kDstDetune: add(vt::kDetune, v); break;
        case vt::kDstSwarmMix: add(vt::kSwarmMix, v); break;
        case vt::kDstOscBalance: add(vt::kBalance, v); break;
        case vt::kDstPw: add(vt::kO2Pw, v); break;
        case vt::kDstCutoff1: c[vt::kF1Cutoff] *= float(std::pow(2.0, v)); break;
        case vt::kDstCutoff2: c[vt::kF2Cutoff] *= float(std::pow(2.0, v)); break;
        case vt::kDstCutoff12:
            c[vt::kF1Cutoff] *= float(std::pow(2.0, v));
            c[vt::kF2Cutoff] *= float(std::pow(2.0, v));
            break;
        case vt::kDstReso:
            add(vt::kF1Reso, v);
            add(vt::kF2Reso, v);
            break;
        case vt::kDstDrive: add(vt::kDrive, v); break;
        case vt::kDstFltBalance: add(vt::kFltBalance, v); break;
        default: break;
    }
}
double scaled(int dest, double amt, double src) {
    switch (dest) {
        case vt::kDstPitch:
        case vt::kDstO2Pitch: return double(vt::kModSemi) * amt * std::fabs(amt) * src;
        case vt::kDstCutoff1:
        case vt::kDstCutoff2:
        case vt::kDstCutoff12: return double(vt::kModOct) * amt * src;
        case vt::kDstOscBalance:
        case vt::kDstFltBalance: return 2.0 * amt * src;
        case vt::kDstPw: return double(vt::kModPw) * amt * src;
        default: return amt * src;
    }
}
}  // namespace

TEST(vtype_matrix_against_the_scale_table) {
    if (pending(4)) return;
    int bad = 0;
    for (int src = 0; src < vt::kSrcCount; ++src)
        for (int dest = 0; dest < vt::kDstCount; ++dest)
            for (double amt : {-1.0, -0.5, 0.5, 1.0}) {
                Vals m = matrixContext();
                m[size_t(vt::modSrc(2))] = float(src);
                m[size_t(vt::modDest(2))] = float(dest);
                m[size_t(vt::modAmt(2))] = float(amt);
                const double v = scaled(dest, amt, srcValue(src));
                const Buf got = held(m, 57.0f, 0.4);
                if (dest == vt::kDstVolume) {
                    const Buf want = held(matrixContext(), 57.0f, 0.4);
                    const double g = std::max(0.0, 1.0 + v);
                    const double d = g > 0.0 ? db(rms(got) / (g * rms(want))) : db(rms(got) + 1e-9) + 200.0;
                    if (g > 0.0 ? std::fabs(d) > 0.26 : rms(got) > 1e-6) {
                        ++bad;
                        std::printf("       src %d dest %d amt %.1f: volume off by %.2f dB\n", src, dest, amt, d);
                    }
                    continue;
                }
                Vals s = matrixContext();
                applyStatic(s, dest, v);
                const Buf want = held(s, 57.0f, 0.4);
                // An envelope's attack moves the phases on the way (Pitch, Detune): compare spectra there.
                const bool phase = src <= vt::kSrcEnv2 && (dest == vt::kDstPitch || dest == vt::kDstO2Pitch || dest == vt::kDstDetune);
                const double d = phase ? specDiff(got, want) : rmsDiff(got, want) / std::max(rms(want), 1e-6);
                if (d > (phase ? 0.1 : 0.03)) {
                    ++bad;
                    std::printf("       src %d dest %d amt %.1f: %.3f off\n", src, dest, amt, d);
                }
            }
    // Two slots on one dest add.
    Vals two = matrixContext();
    two[size_t(vt::modSrc(0))] = float(vt::kSrcEnv1);
    two[size_t(vt::modDest(0))] = float(vt::kDstCutoff1);
    two[size_t(vt::modAmt(0))] = 0.2f;
    two[size_t(vt::modSrc(3))] = float(vt::kSrcLfo2);
    two[size_t(vt::modDest(3))] = float(vt::kDstCutoff1);
    two[size_t(vt::modAmt(3))] = -0.1f;
    Vals st = matrixContext();
    applyStatic(st, vt::kDstCutoff1, 8.0 * (0.2 * 0.6 - 0.1));
    CHECK(rmsDiff(held(two, 57.0f, 0.4), held(st, 57.0f, 0.4)) / rms(held(st, 57.0f, 0.4)) < 0.03);
    CHECK_EQ(bad, 0);
}

TEST(vtype_moved_parameters_ramp_without_a_click) {
    if (pending(4)) return;
    struct Jump {
        int param;
        float a, b;
    };
    const Jump jumps[] = {{vt::kF1Cutoff, 300.0f, 6000.0f}, {vt::kF1Reso, 0.0f, 0.9f},     {vt::kDetune, 0.0f, 1.0f},
                          {vt::kSwarmMix, 0.0f, 1.0f},     {vt::kBalance, -1.0f, 1.0f},   {vt::kDrive, 0.0f, 1.0f},
                          {vt::kFltBalance, -1.0f, 1.0f},  {vt::kSplitWidth, 0.0f, 1.0f}, {vt::kSubLevel, 0.0f, 1.0f},
                          {vt::kNoise, 0.0f, 0.5f},        {vt::kO2Pw, 0.1f, 0.9f},       {vt::kRing, 0.0f, 1.0f},
                          {vt::kF2Cutoff, 200.0f, 5000.0f}};
    for (const Jump& j : jumps) {
        Vals c = initValues();
        c[vt::kRouting] = float(j.param == vt::kSplitWidth ? vt::kSplit : vt::kSerial);
        c[vt::kFltBalance] = 0.0f;
        c[vt::kBalance] = 0.0f;
        c[vt::kO2Wave] = float(vt::kO2Saw);
        c[vt::kF2Type] = float(vt::kLp12);
        c[vt::kF2Cutoff] = 3000.0f;
        c[vt::kAmpD] = 10.0f;
        c[size_t(j.param)] = j.a;
        Rig r(c);
        r.on(0, 45.0f);
        r.seconds(0.25);
        auto maxStep = [&r](size_t a, size_t b) {
            double m = 0;
            for (size_t i = a + 1; i < b; ++i) m = std::max(m, double(std::fabs(r.L[i] - r.L[i - 1])));
            return m;
        };
        for (float v : {j.b, j.a}) {
            const size_t at = r.L.size();
            const double before = maxStep(at - 20 * kBlock, at);
            c[size_t(j.param)] = v;
            r.move(c);
            r.run(1);
            const double jump = maxStep(at - 1, at + 2 * kBlock);
            r.seconds(0.25);
            const double after = maxStep(r.L.size() - 20 * kBlock, r.L.size());
            const double lim = 2.0 * std::max(before, after) + 3e-3;   // -50 dB
            if (!(jump < lim)) std::printf("       param %d -> %.2f: step %.4f (steady %.4f / %.4f)\n", j.param, v, jump, before, after);
            CHECK(jump < lim);
            CHECK(std::fabs(r.p.end[j.param] - v) < 1e-6f);
        }
    }
}

TEST(vtype_chord_is_the_sum_of_its_voices) {
    if (pending(4)) return;
    Vals v = initValues();
    v[vt::kPhase] = float(vt::kPhaseReset);
    v[vt::kO2Wave] = float(vt::kO2Pulse);
    v[vt::kBalance] = 0.0f;
    v[vt::kRouting] = float(vt::kParallel);
    v[vt::kFltBalance] = 0.0f;
    const float notes[3] = {48.0f, 55.0f, 63.0f};
    Rig chord(v);
    for (int k = 0; k < 3; ++k) chord.on(k, notes[k]);
    chord.seconds(0.5);
    Buf sum(chord.L.size(), 0.0f);
    for (int k = 0; k < 3; ++k) {
        const Buf one = play(v, notes[k], 0.5);
        for (size_t i = 0; i < sum.size() && i < one.size(); ++i) sum[i] += one[i];
    }
    CHECK(rmsDiff(chord.L, sum) < 1e-5 * rms(sum) + 1e-7);
}

TEST(vtype_no_nan_or_denormals) {
    if (pending(4)) return;
    uint32_t seed = 139;
    auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) * (1.0f / 16777216.0f);
    };
    int bad = 0;
    long sub = 0;
    for (int set = 0; set < 2000; ++set) {
        Vals a(size_t(kN) + 2, 0.0f);
        for (int i = 0; i < kN; ++i) a[size_t(i)] = range(i)->min + rnd() * (range(i)->max - range(i)->min);
        a[size_t(kN)] = rnd();
        a[size_t(kN) + 1] = rnd();
        Rig r(a);
        r.bpm = 20.0f + rnd() * 400.0f;
        r.on(0, float(int(rnd() * 127.0f)));
        r.on(1, float(int(rnd() * 127.0f)));
        r.run(16);
        r.on(1, float(int(rnd() * 127.0f)), true, rnd() < 0.5f);
        r.run(8);
        r.off(0);
        r.off(1);
        r.run(8);
        if (!finite(r.L) || !finite(r.R) || peak(r.L) > 4.0f || peak(r.R) > 4.0f) {
            ++bad;
            if (bad < 10) std::printf("       set %d: peak %.3f%s\n", set, peak(r.L), finite(r.L) ? "" : " (not finite)");
        }
        sub += subnormals(r.L) + subnormals(r.R);
    }
    CHECK_EQ(bad, 0);
    CHECK_EQ(sub, 0L);
    for (int ext = 0; ext < 2; ++ext) {
        Vals a(size_t(kN) + 2, 0.0f);
        for (int i = 0; i < kN; ++i) a[size_t(i)] = ext ? range(i)->max : range(i)->min;
        Rig r(a);
        r.on(0, 127.0f);
        r.on(1, 0.0f);
        r.seconds(1.0);
        r.off(0);
        r.off(1);
        r.seconds(1.0);
        CHECK(finite(r.L) && finite(r.R));
        CHECK(peak(r.L) < 4.0f);
        CHECK_EQ(subnormals(r.L) + subnormals(r.R), 0L);
    }
}

namespace {
Vals richContext() {
    Vals c = initValues();
    c[vt::kCount] = 3.0f;
    c[vt::kDetune] = 0.5f;
    c[vt::kSubLevel] = 0.3f;
    c[vt::kO2Wave] = float(vt::kO2Pulse);
    c[vt::kO2Semi] = 7.0f;
    c[vt::kO2Sync] = 1.0f;
    c[vt::kRing] = 0.2f;
    c[vt::kBalance] = 0.0f;
    c[vt::kNoise] = 0.2f;
    c[vt::kColour] = -0.3f;
    c[vt::kRouting] = float(vt::kSerial);
    c[vt::kFltBalance] = 0.5f;
    c[vt::kF1Reso] = 0.5f;
    c[vt::kF2Type] = float(vt::kLp12);
    c[vt::kF2Cutoff] = 3000.0f;
    c[vt::kF2Reso] = 0.3f;
    c[vt::kF2Env] = 0.2f;
    c[vt::kF2Key] = 0.2f;
    c[vt::kDrive] = 0.3f;
    c[vt::kE2D] = 0.4f;
    c[vt::kL1Rate] = 3.0f;
    c[vt::kL2Rate] = 0.7f;
    c[vt::kL2Wave] = float(vt::kSine);
    c[size_t(vt::modSrc(0))] = float(vt::kSrcLfo1);
    c[size_t(vt::modDest(0))] = float(vt::kDstPw);
    c[size_t(vt::modAmt(0))] = 0.3f;
    c[size_t(vt::modSrc(1))] = float(vt::kSrcEnv2);
    c[size_t(vt::modDest(1))] = float(vt::kDstO2Pitch);
    c[size_t(vt::modAmt(1))] = 0.4f;
    c[size_t(vt::modSrc(2))] = float(vt::kSrcLfo2);
    c[size_t(vt::modDest(2))] = float(vt::kDstCutoff12);
    c[size_t(vt::modAmt(2))] = 0.1f;
    c[size_t(vt::modSrc(3))] = float(vt::kSrcAmp);
    c[size_t(vt::modDest(3))] = float(vt::kDstDetune);
    c[size_t(vt::modAmt(3))] = 0.2f;
    c[vt::kGlideTime] = 0.1f;
    c[vt::kSplitWidth] = 0.5f;
    c[size_t(kN)] = 0.0f;
    c[size_t(kN) + 1] = 0.0f;
    return c;
}
Buf sequence(const Vals& vals, Buf* right = nullptr) {
    Rig r(vals);
    r.on(0, 57.0f);
    r.seconds(1.0 / 3.0);
    r.on(0, 64.0f, true, true);
    r.seconds(1.0 / 3.0);
    r.off(0);
    r.seconds(1.0 / 3.0);
    if (right) *right = r.R;
    return r.L;
}
}  // namespace

TEST(vtype_every_parameter_has_an_effect) {
    if (pending(4)) return;
    const Vals ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == vt::kVolume) continue;
        Vals a = ctx, b = ctx;
        a[size_t(i)] = range(i)->min;
        b[size_t(i)] = range(i)->max;
        if (i == vt::kGlideMode) {
            a[size_t(i)] = float(vt::kAlways);
            b[size_t(i)] = float(vt::kLegato);
        }
        if (i == vt::kSplitWidth || i == vt::kRouting) {
            a[vt::kRouting] = b[vt::kRouting] = float(vt::kSplit);
            if (i == vt::kRouting) a[vt::kRouting] = float(vt::kSingle);
        }
        if (i == vt::kCurve) a[vt::kDrive] = b[vt::kDrive] = 0.8f;
        if (i == vt::kPhase) {   // Free against Reset: a second voice
            Rig ra(a), rb(b);
            for (Rig* r : {&ra, &rb}) {
                r->on(0, 57.0f);
                r->seconds(0.2);
                r->on(1, 57.0f);
                r->seconds(0.2);
            }
            CHECK(rmsDiff(ra.L, rb.L) > 1e-4);
            continue;
        }
        if (i == vt::kL1Key || i == vt::kL2Key) {   // a second note restarts the LFO
            Rig ra(a), rb(b);
            for (Rig* r : {&ra, &rb}) {
                r->on(0, 57.0f);
                r->seconds(0.31);
                r->on(1, 60.0f);
                r->seconds(0.3);
            }
            CHECK(rmsDiff(ra.L, rb.L) > 1e-4);
            continue;
        }
        if (i == vt::kL1Sync || i == vt::kL2Sync) b[size_t(i)] = 12.0f;   // 1/8 at 120 BPM against 0.05 Hz
        if (i == vt::kL1Rate || i == vt::kL2Rate) b[size_t(i)] = 9.0f;
        Buf ra, rb;
        const double d = std::max(rmsDiff(sequence(a, &ra), sequence(b, &rb)), rmsDiff(ra, rb));
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
        CHECK(d > 1e-4);
    }
}

// Golden hashes (host build, -ffp-contract=off): a rich phrase on the engine
// alone and an Init line through a SynthBank slot. Any change of the sound
// shows here; update the values only for an intended change.
namespace {
uint64_t hashOf(const Buf& x) {
    uint64_t h = 1469598103934665603ull;
    for (float s : x) {
        uint32_t b;
        std::memcpy(&b, &s, 4);
        for (int k = 0; k < 4; ++k) {
            h ^= (b >> (8 * k)) & 0xFFu;
            h *= 1099511628211ull;
        }
    }
    return h;
}
uint64_t bankHash() {
    BankSlot s(1);
    const int line[6] = {36, 48, 43, 36, 55, 31};
    for (int k = 0; k < 6; ++k) {
        s.on(0, line[k] + 12);
        s.run(40);
    }
    s.off(0);
    s.run(200);
    return hashOf(s.out);
}
}  // namespace

TEST(vtype_golden_hashes) {
    if (pending(5)) return;
    Buf right;
    const Buf left = sequence(richContext(), &right);
    const uint64_t e = hashOf(left) ^ (hashOf(right) * 3), b = bankHash();
    std::printf("  V-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x0ull));
    CHECK_EQ(b, uint64_t(0x0ull));
}
