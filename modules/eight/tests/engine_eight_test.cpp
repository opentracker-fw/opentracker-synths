// 8-Type chip-style engine (opentracker-synths modules/eight, engine 0x86):
// registered with a valid descriptor ("8-Type" on screen, Patches/8TYPE/),
// silent when idle; the pitch with Semi / Fine and a three-layer chord; every
// wave's shape (the pulse duty and PW / PWM, the triangle's odd 1 / n^2
// harmonics, the saw's 1 / n, the combined waves thinner than the pulse,
// Tri+Saw), the noise's shift rate following the pitch, Ring (sum and
// difference tones), Sync (the master's pitch, no aliasing burst), Grit's
// level counts; the filter's seven modes (slopes, the notch), its cutoff on
// both Curves, the 16 resonance steps (monotonic, never self-oscillating),
// Dirt bounded, Key Track, the filter envelope's sign and decay; the ADSR
// table times, the attack from the current level, the 4-bit sustain; glide,
// the CHIP tricks; through a SynthBank slot at Poly 1 and 3; no click on
// parameter jumps, no NaN or denormals, Volume never applied, the slot level,
// every parameter audible, golden hashes.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <vector>

#include "audio/chip_dsp.h"
#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "eight.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"

using namespace ot;
using namespace ot::audio;
namespace chip = ot::audio::chip;
using eight::osc;

namespace {

constexpr float kFs = ot::audio::kSampleRate;
constexpr int kN = eight::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(eight::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

using Buf = std::vector<float>;

// The engine's voices driven directly (no framework).
struct Rig {
    ISynthEngine* e = synthEngineFor(eight::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    Buf L, R;
    float bpm = 120.0f;

    explicit Rig(const std::vector<float>& vals) {
        p.engine = eight::kEngineId;
        p.count = kN;
        set(vals);
        e->slotReset(scratch);
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
        v[k].note = uint8_t(std::clamp(int(pitch), 0, 127));
        SynthNote n;
        n.pitch = pitch;
        n.note = uint8_t(std::clamp(int(pitch), 0, 127));
        n.reused = reused;
        n.overlap = overlap;
        e->setTempo(bpm);
        e->noteOn(v[k], p, n);
        alive[k] = true;
    }
    void off(int k) {
        v[k].gate = false;
        e->noteOff(v[k], p);
    }
    int run(int blocks) {
        int live = 0;
        for (int b = 0; b < blocks; ++b) {
            float l[kBlock] = {}, r[kBlock] = {};
            e->setTempo(bpm);
            e->prepare(scratch, p);
            live = 0;
            for (int k = 0; k < kSynthMaxVoices; ++k) {
                if (!alive[k]) continue;
                alive[k] = e->render(v[k], p, 1.0f, 0.0f, l, r);
                live += alive[k];
            }
            L.insert(L.end(), l, l + kBlock);
            R.insert(R.end(), r, r + kBlock);
        }
        return live;
    }
};

bool finite(const Buf& x) {
    for (float s : x)
        if (!std::isfinite(s)) return false;
    return true;
}
long subnormals(const Buf& x) {
    long n = 0;
    for (float s : x) n += std::fpclassify(s) == FP_SUBNORMAL;
    return n;
}
float peak(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    float m = 0.0f;
    for (size_t i = a; i < std::min(b, x.size()); ++i) m = std::max(m, std::fabs(x[i]));
    return m;
}
double rms(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    b = std::min(b, x.size());
    double s = 0.0;
    for (size_t i = a; i < b; ++i) s += double(x[i]) * x[i];
    return b > a ? std::sqrt(s / double(b - a)) : 0.0;
}
double rmsDiff(const Buf& a, const Buf& b) {
    const size_t n = std::min(a.size(), b.size());
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) s += double(a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s / double(n ? n : 1));
}
double hz(float pitch) { return 440.0 * std::pow(2.0, (double(pitch) - 69.0) / 12.0); }
double semis(double f, double ref) { return 12.0 * std::log2(f / ref); }
// Upward zero crossings (interpolated, in frames) of x around its mean.
std::vector<double> crossings(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    b = std::min(b, x.size());
    double mean = 0;
    for (size_t i = a; i < b; ++i) mean += x[i];
    mean /= double(b > a ? b - a : 1);
    std::vector<double> t;
    for (size_t i = a + 1; i < b; ++i) {
        const double y0 = x[i - 1] - mean, y1 = x[i] - mean;
        if (y0 < 0.0 && y1 >= 0.0) t.push_back(double(i - 1) + y0 / (y0 - y1));
    }
    return t;
}
// Mean frequency of x over [a, b) from its upward crossings.
double freqOf(const Buf& x, size_t a, size_t b) {
    const std::vector<double> t = crossings(x, a, b);
    if (t.size() < 3) return 0.0;
    return kFs * double(t.size() - 1) / (t.back() - t.front());
}
// Goertzel magnitude (Hann) at hz.
double tone(const Buf& b, double f) {
    const double w = 2.0 * kPi * f / kFs, c = 2.0 * std::cos(w), wn = 2.0 * kPi / double(b.size());
    double s1 = 0, s2 = 0;
    for (size_t i = 0; i < b.size(); ++i) {
        const double s0 = b[i] * (0.5 - 0.5 * std::cos(wn * double(i))) + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - c * s1 * s2)) / double(b.size());
}

// Power spectrum magnitudes at harmonics: Goertzel of x at h x f0, h = 1..n.
std::vector<double> harmonics(const Buf& x, double f0, int n) {
    std::vector<double> h;
    for (int k = 1; k <= n; ++k) h.push_back(tone(x, f0 * k));
    return h;
}
// Power off the harmonics of f0 below maxHz over the harmonic power (DFT, Hann; the mean removed).
double aliasRatio(const Buf& in, double f0, double maxHz) {
    const size_t n = in.size();
    double mean = 0;   // a synced saw has an offset
    for (float v : in) mean += v;
    mean /= double(n);
    Buf x(in);
    for (float& v : x) v = float(v - mean);
    double harm = 0, alias = 0;
    const double binHz = kFs / double(n);
    for (size_t k = 1; k < n / 2 && double(k) * binHz < maxHz; ++k) {
        double re = 0, im = 0;
        const double dw = 2.0 * kPi * double(k) / double(n);
        for (size_t i = 0; i < n; ++i) {
            const double w = x[i] * (0.5 - 0.5 * std::cos(2.0 * kPi * double(i) / double(n)));
            re += w * std::cos(dw * double(i));
            im -= w * std::sin(dw * double(i));
        }
        const double p = re * re + im * im, f = double(k) * binHz, hh = std::round(f / f0);
        if (hh >= 1 && std::fabs(f - hh * f0) <= 4 * binHz) harm += p;
        else alias += p;
    }
    return alias / harm;
}

// One oscillator alone, held: the filter out of the way (Filt off), the
// ADSR at full sustain, Grit clean, no envelope on the cutoff.
std::vector<float> only(int wave, float level = 0.5f) {
    std::vector<float> c = initValues();
    c[osc(0, eight::kWave)] = float(wave);
    c[osc(0, eight::kLevel)] = level;
    c[osc(0, eight::kFilt)] = 0.0f;
    c[eight::kAttack] = 0.0f;
    c[eight::kSustain] = 15.0f;
    c[eight::kEnvAmt] = 0.0f;
    c[eight::kKeyTrack] = 0.0f;
    return c;
}
// Oscillator 1 (a triangle at the note) through the filter at cutoff hz.
std::vector<float> filtered(float hz, int mode, int res = 0) {
    std::vector<float> c = only(eight::kTri, 0.3f);
    c[osc(0, eight::kFilt)] = 1.0f;
    c[eight::kCutoff] = hz;
    c[eight::kMode] = float(mode);
    c[eight::kResonance] = float(res);
    return c;
}
Buf play(const std::vector<float>& vals, float pitch = 57.0f, float seconds = 0.5f) {
    Rig r(vals);
    r.on(0, pitch);
    r.run(int(seconds * kFs / float(kBlock)) + 1);
    return r.L;
}
Buf mid(const Buf& x, double t0 = 0.15, double t1 = 0.45) {
    return Buf(x.begin() + long(t0 * kFs), x.begin() + long(std::min(double(x.size()), t1 * kFs)));
}
double db(double x) { return 20.0 * std::log10(std::max(x, 1e-12)); }
// The filter's gain (dB) for the triangle's fundamental at the note.
double gainAt(float pitch, float cutoff, int mode, int res = 0, int curve = eight::kBright) {
    std::vector<float> f = filtered(cutoff, mode, res), d = filtered(cutoff, mode, res);
    f[eight::kCurve] = float(curve);
    d[osc(0, eight::kFilt)] = 0.0f;
    return db(tone(mid(play(f, pitch)), hz(pitch)) / tone(mid(play(d, pitch)), hz(pitch)));
}
// The Init patch (or an edited one) through a real SynthBank slot at the
// default slot gain (-9 dB).
struct BankSlot {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
    float pk = 0.0f;
    long clips = 0;
    Buf out;
    BankSlot(int poly, const std::vector<float>* vals = nullptr, int32_t engine = eight::kEngineId) {
        mod::ModuleHost::instance().init();
        gov.setBudget(8);
        bank.setGovernor(&gov);
        bank.setStartsPerBlock(0);
        patchInit(engine, &patch);
        if (vals)
            for (int i = 0; i < kN; ++i) synthParamSet(patch, i, (*vals)[size_t(i)]);
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
            out.insert(out.end(), L, L + kBlock);
            for (uint32_t i = 0; i < kBlock; ++i) {
                const float a = std::max(std::fabs(L[i]), std::fabs(R[i]));
                pk = std::max(pk, a);
                if (a >= 1.0f) ++clips;
            }
        }
    }
};
double dbfs(float x) { return 20.0 * std::log10(x > 1e-9f ? double(x) : 1e-9); }


}  // namespace

TEST(eight_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(eight::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "8TYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(eight::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(eight::kEngineId));
    CHECK_STR(d->name, "8TYPE");
    CHECK_STR(d->displayName, "8-Type");
    CHECK_EQ(int(d->paramCount), 56);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(eight::kVolume));
    CHECK_EQ(int(d->sectionCount), 14);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    CHECK_STR(synthEngineFolder(eight::kEngineId), "8TYPE");   // the leading digit survives
    CHECK_STR(synthEngineDisplayName(eight::kEngineId), "8-Type");
    const int k = synthModelListIndex(eight::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "8-Type");
#if defined(OT_MOD_HND1)
    CHECK(k < synthModelListIndex(0x87));   // before Hand-Type (the profile's order)
#endif
    std::vector<int> seen(kN, 0);
    int macros = 0;
    for (int s = 0; s < d->sectionCount; ++s) {
        const synthui::Section& sec = d->sections[s];
        if (sec.macro >= 0) {
            CHECK_EQ(int(sec.macro), macros);
            ++macros;
            continue;
        }
        for (int j = 0; j < sec.count; ++j)
            if (sec.params[j] < kN) ++seen[sec.params[j]];
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
    CHECK(patchInit(eight::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(eight::kVolume));
}

TEST(eight_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(eight::kEngineId, &p));
    const float vals[][2] = {{float(osc(1, eight::kWave)), 4}, {float(osc(2, eight::kSemi)), -12}, {float(osc(0, eight::kFine)), 33.5f},
                             {eight::kCutoff, 432.5f}, {eight::kMode, 4}, {eight::kGrit, 7}, {eight::kArpMode, 3}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(eight::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(eight::kEngineId));
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    CHECK(!patchParamInRange(eight::kEngineId, eight::kMode, 7.0f));
    CHECK(!patchParamInRange(eight::kEngineId, eight::kGrit, 13.0f));
}

TEST(eight_silent_when_idle) {
    Rig r(initValues());
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 4);
    CHECK(rms(r.L) > 0.01);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

TEST(eight_pitch_semi_fine_and_chord) {
    const Buf a = play(only(eight::kSaw), 69.0f);
    CHECK(std::fabs(freqOf(a, 5000, 20000) - 440.0) < 0.5);
    std::vector<float> c = only(eight::kSaw);
    c[osc(0, eight::kSemi)] = 7.0f;
    c[osc(0, eight::kFine)] = 50.0f;
    CHECK(std::fabs(semis(freqOf(play(c, 57.0f), 5000, 20000), hz(57.0f)) - 7.5) < 0.02);
    // Three layers at 0 / +4 / +7: a chord from one voice.
    std::vector<float> ch = only(eight::kTri, 0.3f);
    for (int k = 1; k < 3; ++k) {
        ch[osc(k, eight::kWave)] = float(eight::kTri);
        ch[osc(k, eight::kLevel)] = 0.3f;
        ch[osc(k, eight::kFilt)] = 0.0f;
        ch[osc(k, eight::kSemi)] = k == 1 ? 4.0f : 7.0f;
    }
    const Buf x = mid(play(ch, 57.0f));
    for (float st : {0.0f, 4.0f, 7.0f}) CHECK(tone(x, hz(57.0f + st)) > 0.01);
    CHECK(tone(x, hz(59.0f)) < 0.002);
}

TEST(eight_waves) {
    const double f0 = hz(45.0f);   // 110 Hz
    // Triangle: odd harmonics at 1 / n^2; saw: every harmonic at 1 / n.
    const std::vector<double> t = harmonics(mid(play(only(eight::kTri), 45.0f)), f0, 5);
    CHECK(t[1] < 0.02 * t[0] && t[3] < 0.02 * t[0]);
    CHECK(std::fabs(t[2] / t[0] - 1.0 / 9.0) < 0.02);
    CHECK(std::fabs(t[4] / t[0] - 1.0 / 25.0) < 0.01);
    const std::vector<double> s = harmonics(mid(play(only(eight::kSaw), 45.0f)), f0, 5);
    for (int k = 1; k < 5; ++k) CHECK(std::fabs(s[size_t(k)] / s[0] - 1.0 / (k + 1)) < 0.03);
    // Pulse: PW 0 = 50 % (no even harmonics), PW 1 = 2 %.
    auto duty = [](const Buf& x) {
        float lo = 1e9f, hi = -1e9f;
        for (float v : x) lo = std::min(lo, v), hi = std::max(hi, v);
        long up = 0;
        for (float v : x) up += v > 0.5f * (lo + hi);
        return double(up) / double(x.size());
    };
    std::vector<float> p = only(eight::kPulse);
    const Buf p0 = mid(play(p, 33.0f));
    CHECK(std::fabs(duty(p0) - 0.5) < 0.02);
    p[osc(0, eight::kPw)] = 1.0f;
    CHECK(std::fabs(duty(mid(play(p, 21.0f))) - 0.02) < 0.01);
    p[osc(0, eight::kPw)] = 0.5f;
    CHECK(std::fabs(duty(mid(play(p, 33.0f))) - 0.26) < 0.02);
    // PWM: the duty moves with the LFO.
    p[osc(0, eight::kPw)] = 0.5f;
    p[osc(0, eight::kPwmDepth)] = 0.8f;
    p[eight::kPwmRate] = 2.0f;
    const Buf m = play(p, 33.0f, 1.2f);
    double dl = 1, dh = 0;
    for (int k = 0; k < 8; ++k) {
        const double dd = duty(Buf(m.begin() + long(k * 0.12 * kFs + 1000), m.begin() + long(k * 0.12 * kFs + 4000)));
        dl = std::min(dl, dd);
        dh = std::max(dh, dd);
    }
    std::printf("  PWM: duty %.3f .. %.3f\n", dl, dh);
    CHECK(dh - dl > 0.2);
    // Combined waves: the pulse gates the other wave: thinner (quieter) than the pulse.
    const double rp = rms(mid(play(only(eight::kPulse), 45.0f)));
    for (int w : {eight::kTriPulse, eight::kSawPulse}) {
        const Buf x = mid(play(only(w), 45.0f));
        std::printf("  wave %d: rms %.4f (pulse %.4f)\n", w, rms(x), rp);
        CHECK(rms(x) < 0.7 * rp);
        CHECK(rms(x) > 0.05 * rp);
        CHECK(std::fabs(freqOf(x, 0, x.size()) - f0) < 1.0 || tone(x, f0) > 0.005);
    }
    // Tri+Saw: their mean.
    const Buf ts = mid(play(only(eight::kTriSaw), 45.0f)), tr = mid(play(only(eight::kTri), 45.0f)),
              sw = mid(play(only(eight::kSaw), 45.0f));
    Buf mean(ts.size());
    for (size_t i = 0; i < ts.size(); ++i) mean[i] = 0.5f * (tr[i] + sw[i]);
    CHECK(rmsDiff(ts, mean) < 0.05 * rms(ts));
    // Off: silent.
    CHECK_EQ(peak(play(only(eight::kOff))), 0.0f);
}

TEST(eight_noise_follows_the_pitch) {
    // About 16 register shifts per oscillator cycle: the stepped level changes at ~16 f.
    auto changes = [](float pitch) {
        const Buf x = play(only(eight::kNoise), pitch, 0.6f);
        long n = 0;
        for (size_t i = 5000; i < 25000; ++i) n += x[i] != x[i - 1];
        return double(n) * kFs / 20000.0;
    };
    const double a = changes(36.0f), b = changes(48.0f);
    std::printf("  noise: %.0f changes/s at note 36, %.0f at 48 (16 f = %.0f, %.0f)\n", a, b, 16 * hz(36), 16 * hz(48));
    CHECK(std::fabs(a / (16.0 * hz(36.0f)) - 1.0) < 0.15);
    CHECK(std::fabs(b / a - 2.0) < 0.25);
    const Buf x = play(only(eight::kNoise), 100.0f, 0.4f);
    CHECK(rms(x) > 0.05);
}

TEST(eight_ring_and_sync) {
    // Ring: oscillator 1's triangle flips with oscillator 3's square: sum and difference tones.
    std::vector<float> c = only(eight::kTri);
    c[osc(0, eight::kRing)] = 1.0f;
    c[osc(2, eight::kWave)] = float(eight::kPulse);   // the modulator, silent
    c[osc(2, eight::kSemi)] = 7.0f;
    const float n = 45.0f;
    const Buf x = mid(play(c, n));
    const double f1 = hz(n), f3 = hz(n + 7.0f);
    std::printf("  ring: sum %.4f, difference %.4f, carrier %.4f\n", tone(x, f1 + f3), tone(x, f3 - f1), tone(x, f1));
    CHECK(tone(x, f1 + f3) > 0.01);
    CHECK(tone(x, f3 - f1) > 0.01);
    CHECK(tone(x, f1) < 0.3 * tone(x, f1 + f3));
    c[osc(0, eight::kRing)] = 0.0f;
    CHECK(rmsDiff(mid(play(c, n)), x) > 0.02);
    // Sync: oscillator 2 (a saw 7.3 semitones up) reset by oscillator 1 (silent): the master's pitch.
    std::vector<float> s = only(eight::kSaw, 0.0f);
    s[osc(1, eight::kWave)] = float(eight::kSaw);
    s[osc(1, eight::kLevel)] = 0.5f;
    s[osc(1, eight::kFilt)] = 0.0f;
    s[osc(1, eight::kSemi)] = 7.0f;
    s[osc(1, eight::kFine)] = 30.0f;
    s[osc(1, eight::kSync)] = 1.0f;
    const float mn = 72.0f;
    const Buf y = mid(play(s, mn), 0.15, 0.15 + 4096.0 / kFs);
    CHECK(tone(y, hz(mn)) > 0.01);
    // Band-limited resets: far less aliasing below 8 kHz than the naive reset.
    Buf naive(y.size());
    double pm = 0.0, ps = 0.0;
    const double dm = hz(mn) / kFs, ds = hz(mn + 7.3f) / kFs;
    for (float& v : naive) {
        v = float(2.0 * ps - 1.0);
        pm += dm;
        ps += ds;
        if (ps >= 1.0) ps -= 1.0;
        if (pm >= 1.0) {
            pm -= 1.0;
            ps = pm / dm * ds;
        }
    }
    const double rn = aliasRatio(naive, hz(mn), 8000.0), rj = aliasRatio(y, hz(mn), 8000.0);
    std::printf("  sync: fundamental %.4f; alias / harmonic power below 8 kHz naive %.1f dB, 8-Type %.1f dB\n", tone(y, hz(mn)),
                10 * std::log10(rn), 10 * std::log10(rj));
    CHECK(10 * std::log10(rj) < 10 * std::log10(rn) - 10.0);
    s[osc(1, eight::kSync)] = 0.0f;
    const Buf z = mid(play(s, mn), 0.15, 0.15 + 4096.0 / kFs);
    CHECK(tone(z, hz(mn)) < 0.3 * tone(y, hz(mn)));
}

TEST(eight_grit_levels) {
    for (int bits : {4, 6, 8, 12}) {
        std::vector<float> c = only(eight::kTri, 1.0f);
        c[eight::kGrit] = float(bits);
        const Buf x = mid(play(c, 33.0f));
        std::set<float> lv(x.begin(), x.end());
        std::printf("  grit %d: %zu levels\n", bits, lv.size());
        if (bits < 12) CHECK(lv.size() <= size_t(1u << bits) && lv.size() >= size_t(1u << (bits - 2)));
        else CHECK(lv.size() > 4096);
    }
    // Clean: an oscillator that skips Grit.
    std::vector<float> c = only(eight::kTri, 1.0f);
    c[eight::kGrit] = 4.0f;
    c[osc(0, eight::kClean)] = 1.0f;
    const Buf x = mid(play(c, 33.0f));
    std::set<float> lv(x.begin(), x.end());
    CHECK(lv.size() > 4096);
}

TEST(eight_filter_modes_cutoff_and_resonance) {
    // LP at the cutoff, Resonance 0 (k 2): -6 dB; two octaves up -24 dB (12 dB / octave).
    const float fc = 440.0f;
    const double at = gainAt(69.0f, fc, eight::kLP), up = gainAt(93.0f, fc, eight::kLP);
    std::printf("  LP: %.2f dB at the cutoff, %.2f dB two octaves up\n", at, up);
    CHECK(std::fabs(at + 6.0) < 1.0);
    CHECK(up < -21.0 && up > -27.0);
    const double hp = gainAt(45.0f, fc, eight::kHP), hpTop = gainAt(93.0f, fc, eight::kHP);
    std::printf("  HP: %.2f dB two octaves down, %.2f dB two octaves up\n", hp, hpTop);
    CHECK(hp < -21.0);
    CHECK(hpTop > -1.0);
    const double bp = gainAt(69.0f, fc, eight::kBP), bpLo = gainAt(45.0f, fc, eight::kBP);
    CHECK(bp > bpLo + 5.0);   // Resonance 0 (Q 0.5): a wide band
    const double notch = gainAt(69.0f, fc, eight::kNotch, 6);
    std::printf("  BP %.2f / %.2f dB, notch %.2f dB\n", bp, bpLo, notch);
    CHECK(notch < -20.0);
    CHECK(gainAt(45.0f, fc, eight::kNotch, 6) > -3.0);
    CHECK(std::fabs(gainAt(69.0f, fc, eight::kLPBP) - gainAt(69.0f, fc, eight::kLP)) > 1.0);
    CHECK(gainAt(45.0f, fc, eight::kBPHP) < -9.0);
    CHECK(std::fabs(gainAt(45.0f, fc, eight::kAll)) < 2.0);   // lp + bp + hp: flat away from the cutoff
    CHECK(std::fabs(gainAt(93.0f, fc, eight::kAll)) < 2.0);
    // The Dark curve: the same at its own mapped cutoff.
    const float dark = eight::curveHz(1200.0f, eight::kDark);
    CHECK(dark < 0.6f * 1200.0f);
    CHECK(std::fabs(eight::curveHz(1200.0f, eight::kBright) - 1200.0f) < 1.0f);
    const float pd = float(69.0 + 12.0 * std::log2(dark / 440.0));
    const double g = gainAt(pd, 1200.0f, eight::kLP, 0, eight::kDark);
    std::printf("  Dark: cutoff 1200 -> %.0f Hz, %.2f dB there\n", dark, g);
    CHECK(std::fabs(g + 6.0) < 1.0);
    // Resonance: the peak at the cutoff over the pass band (two octaves down)
    // grows with every step to ~1 / 0.1 (+20 dB); the top steps thin the pass
    // band instead of raising the peak above ~+6 dB (bounded), and it never sings.
    double last = -100.0;
    for (int r = 0; r <= 15; ++r) {
        const double at = gainAt(69.0f, fc, eight::kLP, r), pass = gainAt(45.0f, fc, eight::kLP, r);
        CHECK(at - pass > last);
        CHECK(at < 7.0);
        last = at - pass;
    }
    std::printf("  resonance 15: %.2f dB over the pass band at the cutoff\n", last);
    CHECK(last > 17.0 && last < 23.0);
    CHECK(std::fabs(eight::resonanceK(0) - 2.0f) < 1e-6f && std::fabs(eight::resonanceK(15) - 0.1f) < 1e-6f);
    std::vector<float> q = filtered(fc, eight::kLP, 15);
    q[osc(0, eight::kLevel)] = 0.0f;
    CHECK_EQ(peak(play(q, 69.0f)), 0.0f);   // nothing in, nothing out
}

TEST(eight_dirt_key_track_and_filter_envelope) {
    // Dirt: bounded, and it changes the sound.
    std::vector<float> c = filtered(800.0f, eight::kLP, 12);
    c[osc(0, eight::kLevel)] = 1.0f;
    c[osc(0, eight::kWave)] = float(eight::kSaw);
    const Buf clean = play(c, 45.0f);
    c[eight::kDirt] = 1.0f;
    const Buf dirty = play(c, 45.0f);
    CHECK(rmsDiff(clean, dirty) > 0.01);
    CHECK(peak(dirty) < 3.0f);
    // Key Track 100 %: the cutoff follows the note (the same gain an octave up).
    std::vector<float> k = filtered(440.0f, eight::kLP);
    k[eight::kKeyTrack] = 1.0f;
    const double g1 = db(tone(mid(play(k, 81.0f)), hz(81.0f))), g0 = db(tone(mid(play(k, 69.0f)), hz(69.0f)));
    k[eight::kKeyTrack] = 0.0f;
    const double g2 = db(tone(mid(play(k, 81.0f)), hz(81.0f)));
    std::printf("  key track: %.2f dB at C4+21 vs %.2f at C4+9 (%.2f without)\n", g1, g0, g2);
    CHECK(std::fabs(g1 - g0) < 2.0);
    CHECK(g2 < g1 - 6.0);
    // Filter envelope: + opens then decays, - closes.
    std::vector<float> e = filtered(300.0f, eight::kLP);
    e[osc(0, eight::kWave)] = float(eight::kSaw);
    e[eight::kEnvAmt] = 1.0f;
    e[eight::kEnvDecay] = 0.4f;
    const Buf ex = play(e, 45.0f, 0.6f);
    auto bright = [](const Buf& b, double t0, double t1) {
        const Buf s = Buf(b.begin() + long(t0 * kFs), b.begin() + long(t1 * kFs));
        double d = 0;
        for (size_t i = 1; i < s.size(); ++i) d += double(s[i] - s[i - 1]) * (s[i] - s[i - 1]);
        return std::sqrt(d / double(s.size())) / std::max(rms(s), 1e-9);
    };
    const double early = bright(ex, 0.005, 0.03), late = bright(ex, 0.5, 0.58);
    std::printf("  filter env +: %.4f early, %.4f late\n", early, late);
    CHECK(early > 2.0 * late);
    e[eight::kEnvAmt] = -1.0f;
    e[eight::kCutoff] = 3000.0f;
    const Buf en = play(e, 45.0f, 0.6f);
    CHECK(bright(en, 0.005, 0.03) < 0.7 * bright(en, 0.5, 0.58));
}

TEST(eight_adsr_tables_and_rules) {
    CHECK(std::fabs(eight::attackSeconds(1) - 0.008f) < 1e-6f);
    CHECK(std::fabs(eight::attackSeconds(15) - 8.0f) < 1e-3f);
    CHECK(std::fabs(eight::decaySeconds(15) - 24.0f) < 1e-3f);
    for (int i = 1; i < 16; ++i) CHECK(eight::attackSeconds(i) > eight::attackSeconds(i - 1));
    // Attack 8 (253 ms): full level at 253 ms +-20 %.
    std::vector<float> c = only(eight::kSaw, 0.5f);
    c[eight::kAttack] = 8.0f;
    const Buf a = play(c, 69.0f, 0.6f);
    const float top = peak(a, size_t(0.45 * kFs), size_t(0.55 * kFs));
    size_t t = 0;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::fabs(a[i]) >= 0.97f * top) {
            t = i;
            break;
        }
    std::printf("  attack 8: full after %.1f ms (table %.1f ms)\n", 1000.0 * t / kFs, 1000.0 * eight::attackSeconds(8));
    CHECK(std::fabs(t / kFs - eight::attackSeconds(8)) < 0.2 * eight::attackSeconds(8));
    // Linear attack: half way at half the time.
    const float half = peak(a, size_t(0.5 * eight::attackSeconds(8) * kFs) - 200, size_t(0.5 * eight::attackSeconds(8) * kFs) + 200);
    CHECK(std::fabs(half / top - 0.5f) < 0.1f);
    // Decay 7 (463 ms) to Sustain 8: at the decay time the level is within 2 % of 8/15.
    c[eight::kAttack] = 0.0f;
    c[eight::kDecay] = 7.0f;
    c[eight::kSustain] = 8.0f;
    const Buf d = play(c, 69.0f, 1.2f);
    const float top0 = peak(d, 0, 2000), late = peak(d, size_t(1.0 * kFs), size_t(1.1 * kFs));
    CHECK(std::fabs(late / top0 - 8.0f / 15.0f) < 0.02f);
    const float atT = peak(d, size_t(eight::decaySeconds(7) * kFs) - 300, size_t(eight::decaySeconds(7) * kFs) + 300);
    CHECK(std::fabs(atT / top0 - 8.0f / 15.0f) < 0.05f);
    // A note during the release resumes from the current level (no restart from 0).
    c[eight::kAttack] = 6.0f;
    c[eight::kRelease] = 9.0f;
    Rig r(c);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec / 2);
    r.off(0);
    r.run(kBlocksPerSec / 4);
    const float before = peak(r.L, r.L.size() - 600, r.L.size());
    const size_t at2 = r.L.size();
    r.on(0, 57.0f, true, false);
    r.run(2);
    CHECK(peak(r.L, at2, at2 + 300) > 0.8f * before);
    // Release ends the voice.
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 20 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 20 * kBlocksPerSec);
    // Sustain is 4-bit: 0..15 = 16 levels.
    c[eight::kAttack] = 0.0f;
    c[eight::kDecay] = 0.0f;
    std::set<int> lv;
    for (int s2 = 0; s2 <= 15; ++s2) {
        c[eight::kSustain] = float(s2) + 0.3f;
        lv.insert(int(std::lround(1000 * peak(play(c, 69.0f, 0.3f), size_t(0.2 * kFs), size_t(0.3 * kFs)))));
    }
    CHECK_EQ(lv.size(), size_t(16));
}

TEST(eight_glide_and_tricks) {
    std::vector<float> c = only(eight::kSaw);
    c[eight::kGlideTime] = 0.3f;
    Rig r(c);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec / 4);
    const size_t at = r.L.size();
    r.on(0, 69.0f, true, true);
    r.run(kBlocksPerSec / 2);
    const double m = semis(freqOf(r.L, at + 2000, at + 4000), hz(57.0f)), e = semis(freqOf(r.L, r.L.size() - 3000, r.L.size()), hz(57.0f));
    CHECK(m > 1.0 && m < 11.0);
    CHECK(std::fabs(e - 12.0) < 0.15);
    // Arp Maj at 60 Hz: 0 4 7 at the ticks.
    std::vector<float> a = only(eight::kSaw);
    a[eight::kArpMode] = float(chip::kArpMaj);
    a[eight::kArpRate] = float(chip::k60Hz);
    const Buf x = play(a, 69.0f, 0.2f);
    const double per = kFs / 60.0;
    const int want[3] = {0, 4, 7};
    for (int k = 0; k < 6; ++k) {
        const double st = semis(freqOf(x, size_t(k * per + 0.2 * per), size_t((k + 1) * per - 0.1 * per)), 440.0);
        CHECK(std::fabs(st - want[k % 3]) < 0.2);
    }
    // Vibrato and slide.
    std::vector<float> v = only(eight::kSaw);
    v[eight::kVibDepth] = 1.0f;
    v[eight::kVibDelay] = 0.0f;
    v[eight::kArpRate] = float(chip::k120Hz);
    const Buf y = play(v, 69.0f, 0.8f);
    double lo = 0, hi = 0;
    for (size_t s = size_t(0.1 * kFs); s + 420 < y.size(); s += 147) {
        const double st = semis(freqOf(y, s, s + 420), 440.0);
        lo = std::min(lo, st), hi = std::max(hi, st);
    }
    CHECK(hi > 0.75 && lo < -0.75);
    std::vector<float> sl = only(eight::kSaw);
    sl[eight::kSlideStart] = 12.0f;
    sl[eight::kSlideTime] = 0.2f;
    const Buf z = play(sl, 57.0f, 0.4f);
    CHECK(semis(freqOf(z, 0, 800), hz(57.0f)) > 10.5);
    CHECK(std::fabs(semis(freqOf(z, size_t(0.25 * kFs), size_t(0.35 * kFs)), hz(57.0f))) < 0.1);
}

TEST(eight_through_the_bank_poly1_and_3) {
    BankSlot s(1);
    s.on(0, 57);
    s.run(kBlocksPerSec / 4);
    s.on(1, 64);
    s.run(kBlocksPerSec / 4);
    CHECK_EQ(s.bank.voicesOf(0), 1);
    BankSlot c(3);
    for (int k = 0; k < 3; ++k) c.on(k, 57 + 4 * k);
    c.run(kBlocksPerSec / 2);
    CHECK_EQ(c.bank.voicesOf(0), 3);
    CHECK(c.pk > 0.05f);
    CHECK_EQ(c.clips, 0L);
}

TEST(eight_parameter_jumps_do_not_click) {
    struct Jump {
        int param;
        float a, b;
    };
    const Jump jumps[] = {{eight::kCutoff, 300, 5000}, {eight::kResonance, 0, 15}, {osc(0, eight::kLevel), 0.1f, 1.0f},
                          {eight::kMode, 0, 2},        {osc(0, eight::kPw), 0, 0.9f}, {eight::kDirt, 0, 1},
                          {eight::kEnvAmt, 0, 1},      {eight::kKeyTrack, 0, 1}};
    for (const Jump& j : jumps) {
        std::vector<float> c = filtered(600.0f, eight::kLP, 4);
        c[osc(0, eight::kWave)] = float(j.param == osc(0, eight::kPw) ? eight::kPulse : eight::kSaw);
        c[osc(0, eight::kLevel)] = 0.4f;
        c[size_t(j.param)] = j.a;
        Rig r(c);
        r.on(0, 45.0f);
        r.run(kBlocksPerSec / 2);
        auto maxStep = [&r](size_t a, size_t b) {
            double m = 0;
            for (size_t i = a + 1; i < b; ++i) m = std::max(m, double(std::fabs(r.L[i] - r.L[i - 1])));
            return m;
        };
        for (float v : {j.b, j.a}) {
            const size_t at = r.L.size();
            const double before = maxStep(at - 20 * kBlock, at);
            c[size_t(j.param)] = v;
            r.set(c);
            r.run(2);
            const double jump = maxStep(at - 1, at + 2 * kBlock);
            r.run(kBlocksPerSec / 4);
            const double after = maxStep(r.L.size() - 20 * kBlock, r.L.size());
            const double lim = 2.0 * std::max(before, after) + 1e-4;
            if (!(jump < lim)) std::printf("       param %d -> %.2f: step %.4f (steady %.4f / %.4f)\n", j.param, v, jump, before, after);
            CHECK(jump < lim);
        }
    }
}

TEST(eight_no_nan_or_denormals) {
    uint32_t seed = 86;
    auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) * (1.0f / 16777216.0f);
    };
    int bad = 0;
    long sub = 0;
    for (int set = 0; set < 150; ++set) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[size_t(i)] = range(i)->min + rnd() * (range(i)->max - range(i)->min);
        Rig r(a);
        r.bpm = 20.0f + rnd() * 400.0f;
        r.on(0, float(int(rnd() * 127.0f)));
        r.on(1, float(int(rnd() * 127.0f)));
        r.run(kBlocksPerSec / 2);
        r.on(1, float(int(rnd() * 127.0f)), true, rnd() < 0.5f);
        r.run(kBlocksPerSec / 4);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec / 4);
        if (!finite(r.L) || peak(r.L) > 4.4f) {   // the bound: (2 + 1.5) x the trim
            ++bad;
            std::printf("       set %d: peak %.3f%s\n", set, peak(r.L), finite(r.L) ? "" : " (not finite)");
        }
        sub += subnormals(r.L);
    }
    CHECK_EQ(bad, 0);
    CHECK_EQ(sub, 0L);
    for (int ext = 0; ext < 2; ++ext) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[size_t(i)] = ext ? range(i)->max : range(i)->min;
        for (int w = 0; w < eight::kWaveCount; ++w) {
            for (int k = 0; k < 3; ++k) a[size_t(osc(k, eight::kWave))] = float(w);
            Rig r(a);
            r.on(0, 127.0f);
            r.on(1, 0.0f);
            r.run(kBlocksPerSec / 2);
            r.off(0);
            r.off(1);
            r.run(kBlocksPerSec / 2);
            CHECK(finite(r.L));
            CHECK(peak(r.L) < 4.4f);
            CHECK_EQ(subnormals(r.L), 0L);
        }
    }
    // A long release tail never goes subnormal.
    std::vector<float> c = initValues();
    c[eight::kRelease] = 15.0f;
    c[eight::kSustain] = 15.0f;
    Rig t(c);
    t.on(0, 40.0f);
    t.run(50);
    t.off(0);
    int blocks = 0;
    while (t.run(1) && blocks < 60 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 60 * kBlocksPerSec);
    CHECK_EQ(subnormals(t.L), 0L);
}

TEST(eight_voice_level_and_volume) {
    std::vector<float> c = only(eight::kSaw);
    c[eight::kAttack] = 10.0f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    std::vector<float> a = initValues(), b = initValues();
    a[eight::kVolume] = 0.0f;
    b[eight::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

TEST(eight_level_matches_the_synth_conventions) {
    BankSlot s(1);
    for (int k = 0; k < 20; ++k) {
        s.on(0, 45 + 3 * (k % 8));
        s.run(100);
    }
    BankSlot c(3);
    for (int rep = 0; rep < 8; ++rep) {
        for (int k = 0; k < 3; ++k) c.on(k, 57 + 4 * k - 12 * (rep & 1));
        c.run(200);
    }
#if defined(OT_MOD_ACD1)
    BankSlot a(1, nullptr, kSynthEngineAcd);
    for (int j = 0; j < 20; ++j) {
        a.on(0, 45 + 3 * (j % 8));
        a.run(100);
    }
    std::printf("  8-Type Init at -9 dB: peak %.1f dBFS (ACD %.1f), 3-voice chords %.1f dBFS\n", dbfs(s.pk), dbfs(a.pk), dbfs(c.pk));
    CHECK(std::fabs(dbfs(s.pk) - dbfs(a.pk)) < 3.0);
#else
    std::printf("  8-Type Init at -9 dB: peak %.1f dBFS, 3-voice chords %.1f dBFS\n", dbfs(s.pk), dbfs(c.pk));
#endif
    CHECK(dbfs(s.pk) < -6.0 && dbfs(s.pk) > -20.0);
    CHECK(dbfs(c.pk) < -1.0);
    CHECK_EQ(s.clips + c.clips, 0L);
}

namespace {
// Every section in use: three oscillators (pulse with PWM, a synced saw, a
// ringed triangle), the filter (BP+HP, resonance, Dirt, envelope), Grit, an
// arp, the vibrato, a slide, glide.
std::vector<float> richContext() {
    std::vector<float> c = initValues();
    c[osc(0, eight::kWave)] = float(eight::kPulse);
    c[osc(0, eight::kPw)] = 0.4f;
    c[osc(0, eight::kPwmDepth)] = 0.3f;
    c[osc(0, eight::kLevel)] = 0.5f;
    c[osc(1, eight::kWave)] = float(eight::kSaw);
    c[osc(1, eight::kLevel)] = 0.4f;
    c[osc(1, eight::kSemi)] = 7.0f;
    c[osc(1, eight::kFine)] = 9.0f;
    c[osc(1, eight::kSync)] = 1.0f;
    c[osc(1, eight::kPw)] = 0.3f;
    c[osc(1, eight::kPwmDepth)] = 0.3f;
    c[osc(2, eight::kWave)] = float(eight::kTri);
    c[osc(2, eight::kLevel)] = 0.4f;
    c[osc(2, eight::kSemi)] = -12.0f;
    c[osc(2, eight::kRing)] = 1.0f;
    c[osc(2, eight::kFilt)] = 0.0f;
    c[osc(2, eight::kPw)] = 0.3f;
    c[osc(2, eight::kPwmDepth)] = 0.3f;
    c[eight::kCutoff] = 900.0f;
    c[eight::kResonance] = 8.0f;
    c[eight::kDirt] = 0.4f;
    c[eight::kEnvAmt] = 0.5f;
    c[eight::kEnvDecay] = 0.15f;
    c[eight::kGrit] = 8.0f;
    c[eight::kArpMode] = float(chip::kArpCustom);
    c[eight::kVibDepth] = 0.5f;
    c[eight::kVibDelay] = 0.05f;
    c[eight::kSlideStart] = 5.0f;
    c[eight::kGlideTime] = 0.1f;
    c[eight::kAttack] = 2.0f;
    c[eight::kDecay] = 5.0f;
    c[eight::kSustain] = 8.0f;
    c[eight::kRelease] = 4.0f;
    return c;
}
Buf sequence(const std::vector<float>& vals) {
    Rig r(vals);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec / 3);
    r.on(0, 64.0f, true, true);
    r.run(kBlocksPerSec / 3);
    r.off(0);
    r.run(kBlocksPerSec / 3);
    return r.L;
}
}  // namespace

TEST(eight_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == eight::kVolume) continue;
        std::vector<float> a = ctx, b = ctx;
        a[size_t(i)] = range(i)->min;
        b[size_t(i)] = range(i)->max;
        const int k = i / eight::kOscStride, f = i % eight::kOscStride;
        if (i < 30 && f == eight::kWave) a[size_t(i)] = float(eight::kSaw);   // Tri vs Off would both be fine; Saw vs Off
        if (i < 30 && f == eight::kClean) a[size_t(osc(k, eight::kLevel))] = b[size_t(osc(k, eight::kLevel))] = 0.6f;
        if (i < 30 && (f == eight::kPw || f == eight::kPwmDepth))
            a[size_t(osc(k, eight::kWave))] = b[size_t(osc(k, eight::kWave))] = float(eight::kPulse);
        if (i < 30 && f == eight::kRing) a[size_t(osc(k, eight::kWave))] = b[size_t(osc(k, eight::kWave))] = float(eight::kTri);
        if (i == osc(0, eight::kRing) || i == osc(0, eight::kSync)) {
            // Oscillator 1's master is oscillator 3: break the chain 1 > 2 > 3 (a full cycle leaves 1 free).
            a[size_t(osc(1, eight::kSync))] = b[size_t(osc(1, eight::kSync))] = 0.0f;
            a[size_t(osc(2, eight::kRing))] = b[size_t(osc(2, eight::kRing))] = 0.0f;
            a[size_t(osc(2, eight::kWave))] = b[size_t(osc(2, eight::kWave))] = float(eight::kPulse);
        }
        if (i == eight::kGlideMode) {
            a[size_t(i)] = float(eight::kAlways);
            b[size_t(i)] = float(eight::kLegato);
        }
        const double d = rmsDiff(sequence(a), sequence(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

TEST(eight_pw_jumps_stay_bounded) {
    // A PW step (an Edit Patch turn, macro M3) at any note and block offset:
    // no spike above the pulse itself (the duty ramps, the edges are clamped).
    double worst = 0.0;
    for (float note : {12.0f, 24.0f, 30.0f, 36.0f, 42.0f, 48.0f}) {
        for (int offset = 0; offset < 12; ++offset) {
            std::vector<float> c = only(eight::kPulse, 0.5f);
            c[osc(1, eight::kWave)] = float(eight::kTriPulse);   // the gate of a combined wave too
            c[osc(1, eight::kLevel)] = 0.5f;
            c[osc(1, eight::kFilt)] = 0.0f;
            Rig r(c);
            r.on(0, note);
            r.run(40 + offset);
            const float steady = peak(r.L, r.L.size() - 4000, r.L.size());
            for (float pw : {0.9f, 0.0f, 1.0f}) {
                c[osc(0, eight::kPw)] = pw;
                c[osc(1, eight::kPw)] = pw;
                r.set(c);
                const size_t at = r.L.size();
                r.run(3);
                worst = std::max(worst, double(peak(r.L, at, r.L.size()) / steady));
            }
        }
    }
    std::printf("  PW jumps: peak %.2f x the steady pulse\n", worst);
    CHECK(worst < 1.6);
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

TEST(eight_golden_hashes) {
    const uint64_t e = hashOf(sequence(richContext())), b = bankHash();
    std::printf("  8-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x3bfc799125876b80ull));
    CHECK_EQ(b, uint64_t(0x8e01fc31536d01f9ull));
}

// --- Performance filter offset (issue #23) ---------------------------------------------
// SynthVoice::cutoffOct (octaves, filled by the framework from the track's
// Performance filter) moves the 8-Type cutoff: Cutoff x 2^offset, inside the
// engine's own clamps. A patch at 1600 Hz shifted by -2 octaves sounds like
// the same patch at 400 Hz; 0 leaves the render bit-identical.
// The offset applies to the curved cutoff (the Init patch's Bright curve is
// the plain setting).

namespace {
namespace perfcut {

// One note (pitch 48, 40 blocks) from the Init patch with its cutoff set by
// `cut(patch, hz)`, the voice's cutoffOct = `oct`.
template <class Cut>
std::vector<float> render(Cut cut, float hz, float oct) {
    const int32_t engine = eight::kEngineId;
    ISynthEngine* e = synthEngineFor(engine);
    SynthPatch patch;
    patchInit(engine, &patch);
    cut(patch, hz);
    SynthParams p;
    const uint8_t m[kSynthSeqMacros] = {};
    p.engine = engine;
    p.count = synthEffectiveParams(patch, m, 1.0f, p.end);
    std::memcpy(p.start, p.end, sizeof(p.end));
    p.serial = 1;
    static SynthSlotScratch scratch;
    static SynthVoice v;
    scratch = SynthSlotScratch{};
    e->slotReset(scratch);
    v = SynthVoice{};
    v.slotScratch = &scratch;
    v.cutoffOct = oct;
    SynthNote n;
    n.pitch = 48.0f;
    n.note = 48;
    v.note = 48;
    v.gate = true;
    e->noteOn(v, p, n);
    std::vector<float> out;
    float L[kBlock], R[kBlock];
    for (int b = 0; b < 40; ++b) {
        for (uint32_t i = 0; i < kBlock; ++i) L[i] = R[i] = 0.0f;
        e->prepare(scratch, p);
        e->render(v, p, 1.0f, 0.0f, L, R);
        out.insert(out.end(), L, L + kBlock);
    }
    return out;
}

double energy(const std::vector<float>& x) {
    double s = 0;
    for (float f : x) s += double(f) * f;
    return s;
}

// Relative difference of two renders (0 = identical).
double relDiff(const std::vector<float>& a, const std::vector<float>& b) {
    double d = 0;
    for (size_t i = 0; i < a.size(); ++i) d += double(a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(d / (energy(a) + 1e-30));
}

void cut(SynthPatch& p, float hz) {
    CHECK(synthParamSet(p, eight::kCutoff, hz));
}

}  // namespace perfcut
}  // namespace

TEST(eight_perf_cutoff_offset_moves_the_cutoff) {
    using namespace perfcut;
    const std::vector<float> ref = render(cut, 400.0f, 0.0f);
    CHECK(energy(ref) > 1e-6);
    // The same cutoff reached by the offset: close to the plain render.
    CHECK(relDiff(ref, render(cut, 1600.0f, -2.0f)) < 0.05);
    CHECK(relDiff(render(cut, 6400.0f, 0.0f), render(cut, 400.0f, 4.0f)) < 0.05);
    // The offset changes the sound, and 0 is the plain patch.
    CHECK(relDiff(ref, render(cut, 400.0f, -3.0f)) > 0.1);
    CHECK(relDiff(ref, render(cut, 400.0f, 0.0f)) == 0.0);
    // The full range stays finite (the engine's own clamps).
    for (float o : {-9.54f, 9.54f}) {
        const std::vector<float> x = render(cut, o > 0 ? 20000.0f : 20.0f, o);
        bool fin = true;
        for (float f : x) fin = fin && std::isfinite(f);
        CHECK(fin);
    }
}
