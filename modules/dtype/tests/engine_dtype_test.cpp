// D-Type three-oscillator mono engine (opentracker-synths modules/dtype,
// engine 0x82): registered with a valid descriptor ("D-Type" on screen,
// Patches/DTYPE/), silent when idle, the VCOs at A 440 with their ranges,
// tune, fine and the keyboard switch of VCO 3, the six waves and their
// duties, a small bounded drift, polyBLEP aliasing below a naive saw, white
// and pink noise, a bounded mixer drive and feedback, the ladder's cutoff,
// 24 dB slope, resonance peak and keyboard-tracked self-oscillation, Contour
// and Key Track, stability under fast modulation, the two contours and the
// Decay switch, glide time and modes, legato, last-note priority through a
// SynthBank slot, Wheel vibrato and filter mod, audio-rate FM sidebands, no
// click on parameter jumps, no NaN or denormals, the voice level for
// stealing, Volume never applied, every parameter audible, the slot level of
// the other engines.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "dtype.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"

using namespace ot;
using namespace ot::audio;

namespace {

constexpr float kFs = ot::audio::kSampleRate;   // (model/project.h has a kSampleRate too)
constexpr int kN = dtype::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(dtype::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

// One engine voice pool driven directly (no framework).
struct Rig {
    ISynthEngine* e = synthEngineFor(dtype::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    std::vector<float> L, R;

    explicit Rig(const std::vector<float>& vals) {
        p.engine = dtype::kEngineId;
        p.count = kN;
        set(vals);
        e->slotReset(scratch);
        for (auto& x : v) {
            x = SynthVoice{};
            x.slotScratch = &scratch;
        }
    }
    void set(const std::vector<float>& vals) {
        for (int i = 0; i < kN; ++i) p.start[i] = p.end[i] = vals[i];
        ++p.serial;
    }
    void on(int k, float pitch, float velocity = 1.0f, bool reused = false, bool overlap = false, bool fxGlide = false) {
        if (!reused) {
            std::memset(v[k].state, 0, sizeof(v[k].state));
            v[k].level = 0.0f;
        }
        v[k].gate = true;
        SynthNote n;
        n.pitch = pitch;
        n.velocity = velocity;
        n.note = uint8_t(std::clamp(int(pitch), 0, 127));
        n.reused = reused;
        n.overlap = overlap;
        n.fxGlide = fxGlide;
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
            L.insert(L.end(), l, l + kBlock);
            R.insert(R.end(), r, r + kBlock);
        }
        return live;
    }
};

using Buf = std::vector<float>;

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
double rmsDiff(const Buf& a, const Buf& b) {
    const size_t n = std::min(a.size(), b.size());
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) s += double(a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s / double(n));
}
Buf slice(const Buf& b, double t0, double t1) {
    const size_t a = size_t(t0 * kFs), e = std::min(b.size(), size_t(t1 * kFs));
    return Buf(b.begin() + long(std::min(a, e)), b.begin() + long(e));
}
// Fundamental (Hz) in [a, b) by autocorrelation over lags of 60..1500 Hz;
// the first lag within 90 % of the best one (no octave-down picks).
double pitchHz(const Buf& x, size_t a, size_t b) {
    const int lo = int(kFs / 1500.0f), hi = int(kFs / 60.0f);
    std::vector<double> ac(size_t(hi) + 2, 0.0);
    double best = 0.0;
    for (int lag = lo; lag <= hi + 1; ++lag) {
        double s = 0.0;
        for (size_t i = a; i + size_t(lag) < b; ++i) s += double(x[i]) * x[i + size_t(lag)];
        ac[size_t(lag)] = s;
        best = std::max(best, s);
    }
    for (int lag = lo + 1; lag <= hi; ++lag)
        if (ac[size_t(lag)] >= 0.9 * best && ac[size_t(lag)] >= ac[size_t(lag) - 1] && ac[size_t(lag)] >= ac[size_t(lag) + 1]) {
            const double y0 = ac[size_t(lag) - 1], y1 = ac[size_t(lag)], y2 = ac[size_t(lag) + 1];
            const double den = y0 - 2.0 * y1 + y2;
            const double off = den != 0.0 ? 0.5 * (y0 - y2) / den : 0.0;
            return kFs / (double(lag) + off);
        }
    return 0.0;
}
// High-frequency proxy: rms of the first difference over rms.
double bright(const Buf& b) {
    double s = 0;
    for (size_t i = 1; i < b.size(); ++i) s += double(b[i] - b[i - 1]) * (b[i] - b[i - 1]);
    const double r = rms(b);
    return r <= 0 ? 0.0 : std::sqrt(s / double(b.size())) / r;
}
// Goertzel magnitude (normalised, Hann window) at hz.
double tone(const Buf& b, double hz) {
    const double w = 2.0 * kPi * hz / kFs, c = 2.0 * std::cos(w);
    const double wn = 2.0 * kPi / double(b.size());
    double s1 = 0, s2 = 0;
    for (size_t i = 0; i < b.size(); ++i) {
        const double s0 = b[i] * (0.5 - 0.5 * std::cos(wn * double(i))) + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - c * s1 * s2)) / double(b.size());
}
double hz(float pitch) { return 440.0 * std::pow(2.0, (double(pitch) - 69.0) / 12.0); }
double cents(double f, double ref) { return 1200.0 * std::log2(f / ref); }
double db(double x) { return 20.0 * std::log10(std::max(x, 1e-12)); }

// Zero-crossing frequency track (upward crossings, interpolated).
std::vector<double> freqTrack(const Buf& x, std::vector<double>* at = nullptr) {
    std::vector<double> f;
    double last = -1;
    for (size_t i = 1; i < x.size(); ++i)
        if (x[i - 1] < 0.0f && x[i] >= 0.0f) {
            const double t = double(i - 1) + double(x[i - 1]) / double(x[i - 1] - x[i]);
            if (last >= 0) {
                f.push_back(kFs / (t - last));
                if (at) at->push_back(t / kFs);
            }
            last = t;
        }
    return f;
}
// The tracked frequency nearest to time t (seconds).
double freqAt(const Buf& x, double t) {
    std::vector<double> at;
    const std::vector<double> f = freqTrack(x, &at);
    double best = 1e9, out = 0.0;
    for (size_t i = 0; i < f.size(); ++i)
        if (std::fabs(at[i] - t) < best) best = std::fabs(at[i] - t), out = f[i];
    return out;
}

// A plain context: VCO 1 alone (saw, 8'), the ladder wide open, no
// emphasis, no contour, no key track, loudness held at full sustain, no
// modulation, no glide, Decay switch On.
std::vector<float> open() {
    std::vector<float> c = initValues();
    c[dtype::kVco1Level] = 1.0f;
    c[dtype::kVco2Level] = 0.0f;
    c[dtype::kVco3Level] = 0.0f;
    c[dtype::kNoiseLevel] = 0.0f;
    c[dtype::kFeedback] = 0.0f;
    c[dtype::kCutoff] = 20000.0f;
    c[dtype::kEmphasis] = 0.0f;
    c[dtype::kContour] = 0.0f;
    c[dtype::kKeyTrack] = float(dtype::kKeyOff);
    c[dtype::kLAttack] = 0.0f;
    c[dtype::kLSustain] = 1.0f;
    c[dtype::kLDecay] = 0.05f;
    c[dtype::kOscMod] = 0.0f;
    c[dtype::kFilterMod] = 0.0f;
    c[dtype::kWheel] = 0.0f;
    c[dtype::kGlideTime] = 0.0f;
    return c;
}
// One VCO alone at a low level (the input saturator stays near linear).
std::vector<float> only(int vco, int wave, float level = 0.25f) {
    std::vector<float> c = open();
    c[dtype::kVco1Level] = 0.0f;
    const int lv[3] = {dtype::kVco1Level, dtype::kVco2Level, dtype::kVco3Level};
    const int wv[3] = {dtype::kVco1Wave, dtype::kVco2Wave, dtype::kVco3Wave};
    c[size_t(lv[vco])] = level;
    c[size_t(wv[vco])] = float(wave);
    c[dtype::kVco2Fine] = 0.0f;
    c[dtype::kVco3Kb] = 1.0f;
    c[dtype::kVco3Range] = float(dtype::k8);
    return c;
}
// The ladder alone, self-oscillating: no VCO, no noise (the seed starts it).
std::vector<float> whistle(float cutoff = 440.0f, int keyTrack = dtype::kKeyFull) {
    std::vector<float> c = open();
    c[dtype::kVco1Level] = 0.0f;
    c[dtype::kCutoff] = cutoff;
    c[dtype::kEmphasis] = 1.0f;
    c[dtype::kKeyTrack] = float(keyTrack);
    return c;
}
Buf play(const std::vector<float>& vals, float pitch = 48.0f, float seconds = 0.5f) {
    Rig r(vals);
    r.on(0, pitch);
    r.run(int(seconds * kFs / float(kBlock)) + 1);
    return r.L;
}
const size_t kA = size_t(0.15 * 44100), kB = size_t(0.45 * 44100);

// Power spectrum (Hann) of x by a plain DFT, bins 0..n/2.
std::vector<double> spectrum(const Buf& x) {
    const size_t n = x.size();
    std::vector<double> w(n), out(n / 2 + 1);
    for (size_t i = 0; i < n; ++i) w[i] = x[i] * (0.5 - 0.5 * std::cos(2.0 * kPi * double(i) / double(n)));
    for (size_t k = 0; k <= n / 2; ++k) {
        double re = 0, im = 0;
        const double dw = 2.0 * kPi * double(k) / double(n);
        for (size_t i = 0; i < n; ++i) {
            re += w[i] * std::cos(dw * double(i));
            im -= w[i] * std::sin(dw * double(i));
        }
        out[k] = re * re + im * im;
    }
    return out;
}
// Power below maxHz that is not within 4 bins of a harmonic of f0, over the
// harmonic power below maxHz.
double aliasRatio(const Buf& x, double f0, double maxHz) {
    const std::vector<double> s = spectrum(x);
    const double binHz = kFs / double(x.size());
    double harm = 0, alias = 0;
    for (size_t k = 1; k < s.size() && double(k) * binHz < maxHz; ++k) {
        const double f = double(k) * binHz;
        const double h = std::round(f / f0);
        if (h >= 1 && std::fabs(f - h * f0) <= 4 * binHz) harm += s[k];
        else alias += s[k];
    }
    return alias / harm;
}

// Every section of the voice in use.
std::vector<float> richContext() {
    std::vector<float> c = initValues();
    c[dtype::kVco1Wave] = float(dtype::kTriSaw);
    c[dtype::kVco2Range] = float(dtype::k16);
    c[dtype::kVco2Wave] = float(dtype::kWide);
    c[dtype::kVco2Tune] = 7.0f;
    c[dtype::kVco3Range] = float(dtype::k4);
    c[dtype::kVco3Wave] = float(dtype::kSaw);
    c[dtype::kVco3Tune] = -5.0f;
    c[dtype::kVco3Fine] = 20.0f;
    c[dtype::kVco3Level] = 0.3f;
    c[dtype::kVco3Kb] = 1.0f;
    c[dtype::kNoiseLevel] = 0.2f;
    c[dtype::kNoiseColour] = 0.0f;
    c[dtype::kFeedback] = 0.3f;
    c[dtype::kCutoff] = 900.0f;
    c[dtype::kEmphasis] = 0.5f;
    c[dtype::kContour] = 0.5f;
    c[dtype::kKeyTrack] = float(dtype::kKey13);
    c[dtype::kFAttack] = 0.02f;
    c[dtype::kFDecay] = 0.05f;
    c[dtype::kFSustain] = 0.4f;
    c[dtype::kLAttack] = 0.01f;
    c[dtype::kLDecay] = 0.08f;
    c[dtype::kLSustain] = 0.5f;
    c[dtype::kModMix] = 0.5f;
    c[dtype::kOscMod] = 0.3f;
    c[dtype::kFilterMod] = 0.3f;
    c[dtype::kWheel] = 0.5f;
    c[dtype::kGlideTime] = 0.1f;
    return c;
}
Buf renderSequence(const std::vector<float>& vals) {
    Rig r(vals);
    r.run(10);
    r.on(0, 57.0f, 0.6f);
    r.run(kBlocksPerSec / 4);
    r.on(0, 64.0f, 0.6f, true, false);
    r.run(kBlocksPerSec / 4);
    r.off(0);
    r.run(kBlocksPerSec / 4);
    return r.L;
}

}  // namespace

TEST(dtype_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(dtype::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "DTYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(dtype::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(dtype::kEngineId));
    CHECK_STR(d->name, "DTYPE");
    CHECK_STR(d->displayName, "D-Type");
    CHECK_EQ(int(d->paramCount), 35);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(dtype::kVolume));
    CHECK_EQ(patchVolumeParam(dtype::kEngineId), int(dtype::kVolume));
    CHECK_STR(patchEngineName(dtype::kEngineId), "DTYPE");
    CHECK_EQ(int(d->sectionCount), synthui::kMaxSections);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    // Folder and screen name; in the model list after the Mini models (and JU6 when built).
    CHECK_STR(synthEngineFolder(dtype::kEngineId), "DTYPE");
    CHECK_STR(synthEngineDisplayName(dtype::kEngineId), "D-Type");
    const int k = synthModelListIndex(dtype::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "D-Type");
#if defined(OT_MOD_JU6S)
    CHECK(k > synthModelListIndex(0x81));
#endif
    // Ranges hold their defaults; enum names match the range; every
    // parameter sits in exactly one section; the macros come last.
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
    CHECK(patchInit(dtype::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(dtype::kVolume));
}

TEST(dtype_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(dtype::kEngineId, &p));
    const float vals[][2] = {{dtype::kVco1Range, 1}, {dtype::kVco2Tune, -12}, {dtype::kVco3Fine, 33.5f},
                             {dtype::kVco3Kb, 1},    {dtype::kCutoff, 432.5f}, {dtype::kKeyTrack, 3},
                             {dtype::kGlideTime, 0.25f}, {dtype::kDecaySw, 0}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(dtype::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(dtype::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    // Out of range values are refused.
    CHECK(!patchParamInRange(dtype::kEngineId, dtype::kVco1Range, 6.0f));
    CHECK(!patchParamInRange(dtype::kEngineId, dtype::kVco2Tune, 25.0f));
    CHECK(!patchParamInRange(dtype::kEngineId, dtype::kGlideTime, 3.5f));
}

TEST(dtype_silent_when_idle) {
    Rig r(initValues());
    r.run(20);   // no note
    CHECK_EQ(peak(r.L), 0.0f);
    // A voice that never started renders nothing and reports finished.
    float l[kBlock] = {}, rr[kBlock] = {};
    r.v[3].gate = false;
    CHECK(!r.e->render(r.v[3], r.p, 1.0f, 0.0f, l, rr));
    CHECK_EQ(peak(Buf(l, l + kBlock)), 0.0f);
    // After the release the voice ends and the output is exactly silent.
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 2);
    CHECK(rms(r.L) > 0.01);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

TEST(dtype_vco_pitch_and_ranges) {
    // VCO 1 at 8': the note (A4 = 440 Hz).
    for (float pitch : {69.0f, 57.0f, 45.0f}) {
        const Buf x = play(only(0, dtype::kSaw), pitch);
        CHECK(finite(x));
        const double f = pitchHz(x, kA, kB);
        CHECK_NEAR(f, hz(pitch), hz(pitch) * 0.005);
        if (pitch == 69.0f) std::printf("  A4: %.2f Hz\n", f);
    }
    // Ranges: 32' 16' 8' 4' 2' are -2..+2 octaves, Lo is 6 octaves down.
    const float oct[6] = {-6, -2, -1, 0, 1, 2};
    for (int rg = 0; rg < 6; ++rg) {
        std::vector<float> c = only(0, dtype::kSaw);
        c[dtype::kVco1Range] = float(rg);
        const float note = rg == dtype::kLo ? 112.0f : (rg <= dtype::k16 ? 69.0f : 57.0f);   // 60..1500 Hz (Lo: 104 Hz)
        const double want = hz(note) * std::pow(2.0, oct[rg]);
        const double f = pitchHz(play(c, note), kA, kB);
        CHECK_NEAR(f, want, want * 0.005);
    }
    // VCO 2 Tune (semitones) and Fine (cents); VCO 3 with the keyboard on likewise.
    for (int vco = 1; vco <= 2; ++vco) {
        const int tune = vco == 1 ? dtype::kVco2Tune : dtype::kVco3Tune;
        const int fine = vco == 1 ? dtype::kVco2Fine : dtype::kVco3Fine;
        std::vector<float> c = only(vco, dtype::kSaw);
        c[size_t(tune)] = 7.0f;
        CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(64.0f), hz(64.0f) * 0.003);
        c[size_t(tune)] = -12.0f;
        CHECK_NEAR(pitchHz(play(c, 69.0f), kA, kB), 220.0, 220.0 * 0.003);
        c[size_t(tune)] = 0.0f;
        c[size_t(fine)] = 50.0f;
        CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(57.5f), hz(57.5f) * 0.002);
        c[size_t(fine)] = -100.0f;
        CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(56.0f), hz(56.0f) * 0.002);
    }
}

TEST(dtype_vco3_keyboard_off_is_fixed) {
    std::vector<float> c = only(2, dtype::kSaw);
    c[dtype::kVco3Kb] = 0.0f;
    // Fixed from C3 (130.8 Hz) whatever the note; the range and tune still apply.
    for (float pitch : {69.0f, 45.0f, 84.0f}) CHECK_NEAR(pitchHz(play(c, pitch), kA, kB), hz(48.0f), hz(48.0f) * 0.004);
    c[dtype::kVco3Range] = float(dtype::k4);
    CHECK_NEAR(pitchHz(play(c, 60.0f), kA, kB), hz(60.0f), hz(60.0f) * 0.004);
    c[dtype::kVco3Tune] = 7.0f;
    CHECK_NEAR(pitchHz(play(c, 30.0f), kA, kB), hz(67.0f), hz(67.0f) * 0.004);
    // Keyboard on: the note.
    c = only(2, dtype::kSaw);
    CHECK_NEAR(pitchHz(play(c, 69.0f), kA, kB), 440.0, 440.0 * 0.004);
}

TEST(dtype_waveforms_and_duties) {
    // Pulse duties: the fraction of the period above the mean.
    auto highFraction = [](const Buf& x) {
        double mean = 0;
        for (float s : x) mean += s;
        mean /= double(x.size());
        size_t n = 0;
        for (float s : x) n += s > mean;
        return double(n) / double(x.size());
    };
    const double duty[3] = {0.5, 0.3, 0.1};
    for (int vco = 0; vco < 3; ++vco)
        for (int w = 0; w < 3; ++w) {
            const Buf x = slice(play(only(vco, dtype::kSquare + w), 45.0f), 0.1, 0.45);
            const double hf = highFraction(x);
            if (std::fabs(hf - duty[w]) >= 0.03) std::printf("       VCO%d wave %d: high %.3f\n", vco + 1, dtype::kSquare + w, hf);
            CHECK_NEAR(hf, duty[w], 0.03);
        }
    // Spectra at 110 Hz: the triangle has odd harmonics falling as 1/n^2, the
    // saw all harmonics as 1/n, Tri-Saw lies between, the square no even ones.
    auto h = [](const Buf& x, int n) { return tone(x, 110.0 * n) / tone(x, 110.0); };
    const Buf tri = slice(play(only(0, dtype::kTri), 45.0f), 0.1, 0.45);
    const Buf saw = slice(play(only(0, dtype::kSaw), 45.0f), 0.1, 0.45);
    const Buf ts = slice(play(only(0, dtype::kTriSaw), 45.0f), 0.1, 0.45);
    const Buf sq = slice(play(only(0, dtype::kSquare), 45.0f), 0.1, 0.45);
    CHECK(h(tri, 2) < 0.03);
    CHECK_NEAR(h(tri, 3), 1.0 / 9.0, 0.03);
    CHECK_NEAR(h(saw, 2), 0.5, 0.06);
    CHECK_NEAR(h(saw, 3), 1.0 / 3.0, 0.05);
    CHECK(h(ts, 2) > 0.1 && h(ts, 2) < 0.45);
    CHECK(h(sq, 2) < 0.03);
    CHECK_NEAR(h(sq, 3), 1.0 / 3.0, 0.05);
    // VCO 3: Rev Saw is the saw upside down (same phase: same seeds).
    const Buf s3 = play(only(2, dtype::kSaw), 45.0f), r3 = play(only(2, dtype::kRevSaw), 45.0f);
    Buf sum(s3.size());
    for (size_t i = 0; i < s3.size(); ++i) sum[i] = s3[i] + r3[i];
    CHECK(rms(sum) < 0.05 * rms(s3));
    // VCO 1 / 2 slot 1 is Tri-Saw, not Rev Saw.
    CHECK(rmsDiff(play(only(0, dtype::kTriSaw), 45.0f), play(only(0, dtype::kSaw), 45.0f)) > 0.1 * rms(saw));
}

TEST(dtype_drift_is_small) {
    // Each VCO drifts on its own, within +-3 cents of the note over 4 s.
    for (int vco = 0; vco < 3; ++vco) {
        const Buf x = play(only(vco, dtype::kSaw), 69.0f, 4.2f);
        double lo = 1e9, hi = -1e9;
        for (double t = 0.2; t + 0.25 < 4.2; t += 0.25) {
            const double c = cents(pitchHz(x, size_t(t * kFs), size_t((t + 0.25) * kFs)), 440.0);
            lo = std::min(lo, c);
            hi = std::max(hi, c);
        }
        std::printf("  VCO%d drift %.2f .. %.2f cents\n", vco + 1, lo, hi);
        CHECK(lo > -3.0 && hi < 3.0);
    }
}

TEST(dtype_polyblep_lowers_aliasing) {
    // Saw and narrow pulse at C7 (2093 Hz, note 96): the partials above
    // Nyquist of a naive waveform fold down between the harmonics. Compared
    // below 8 kHz, where the open ladder is flat.
    const float pitch = 96.0f;
    const double f0 = hz(pitch);
    for (int wave : {int(dtype::kSaw), int(dtype::kNarrow)}) {
        const Buf x = play(only(0, wave), pitch, 0.4f);
        const Buf seg = slice(x, 0.2, 0.2 + 4096.0 / kFs);
        Buf naive(seg.size());
        double ph = 0.37;
        const double d = wave == dtype::kSaw ? 0.0 : 0.1;
        for (float& s : naive) {
            s = float(wave == dtype::kSaw ? 2.0 * ph - 1.0 : (ph < d ? 1.0 : -1.0));
            ph += f0 / kFs;
            if (ph >= 1.0) ph -= 1.0;
        }
        const double rn = aliasRatio(naive, f0, 8000.0), rj = aliasRatio(seg, f0, 8000.0);
        std::printf("  wave %d alias/harmonic power below 8 kHz: naive %.1f dB, D-Type %.1f dB\n", wave, 10 * std::log10(rn),
                    10 * std::log10(rj));
        CHECK(10 * std::log10(rj) < 10 * std::log10(rn) - 10.0);
    }
}

TEST(dtype_white_and_pink_noise) {
    std::vector<float> c = open();
    c[dtype::kVco1Level] = 0.0f;
    c[dtype::kNoiseLevel] = 0.5f;
    const Buf w = slice(play(c, 60.0f, 1.0f), 0.1, 1.0);
    c[dtype::kNoiseColour] = float(dtype::kPink);
    const Buf p = slice(play(c, 60.0f, 1.0f), 0.1, 1.0);
    CHECK(rms(w) > 0.02 && rms(p) > 0.02);
    CHECK(rms(p) > 0.5 * rms(w) && rms(p) < 2.0 * rms(w));   // about the same loudness
    // Pink: -3 dB per octave, so far less in the highs.
    CHECK(bright(p) < 0.6 * bright(w));
    // Low (100-400 Hz) over high (3.2-6.4 kHz) band power per Hz: ~1 for white, ~16x for pink.
    auto band = [](const Buf& x, double a, double b) {
        double s = 0;
        int n = 0;
        for (double f = a; f < b; f += (b - a) / 16.0, ++n) s += tone(x, f) * tone(x, f);
        return s / n;
    };
    const double rw = band(w, 100, 400) / band(w, 3200, 6400), rp = band(p, 100, 400) / band(p, 3200, 6400);
    std::printf("  low / high band power: white %.2f, pink %.2f\n", rw, rp);
    CHECK(rw > 0.3 && rw < 3.0);
    CHECK(rp > 5.0);
}

TEST(dtype_mixer_drive_and_feedback_bounded) {
    // Three VCOs at full level saturate the filter input: louder than one,
    // far from three times as loud, never out of bounds.
    std::vector<float> one = open();
    std::vector<float> three = open();
    three[dtype::kVco2Level] = 1.0f;
    three[dtype::kVco3Level] = 1.0f;
    three[dtype::kVco3Kb] = 1.0f;
    three[dtype::kVco3Range] = float(dtype::k8);
    three[dtype::kVco2Fine] = 0.0f;   // in tune: their sum would be 3x without the saturation
    const Buf a = play(one, 45.0f), b = play(three, 45.0f);
    const double ra = rms(a, kA, kB), rb = rms(b, kA, kB);
    std::printf("  one VCO %.3f rms, three %.3f rms (peak %.3f)\n", ra, rb, peak(b));
    CHECK(rb > 1.1 * ra);
    CHECK(rb < 2.2 * ra);
    CHECK(peak(b) < 1.5f);
    // Feedback: the output back into the mixer (growl), bounded by the input
    // saturation, also with the emphasis high.
    for (float emph : {0.0f, 0.9f}) {
        std::vector<float> f = open();
        f[dtype::kCutoff] = 800.0f;
        f[dtype::kEmphasis] = emph;
        const Buf dry = play(f, 33.0f, 1.0f);
        f[dtype::kFeedback] = 1.0f;
        const Buf wet = play(f, 33.0f, 1.0f);
        CHECK(finite(wet));
        CHECK(peak(wet) < 1.5f);
        CHECK(rmsDiff(dry, wet) > 0.05 * rms(dry));
        CHECK_EQ(subnormals(wet), 0L);
    }
}

namespace {
// Ladder response at harmonic n of a 110 Hz saw (low level: linear): the
// filtered over the open output, same seeds (same phases).
double response(float cutoff, float emphasis, int n) {
    std::vector<float> c = only(0, dtype::kSaw, 0.1f);
    const Buf open = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    c[dtype::kCutoff] = cutoff;
    c[dtype::kEmphasis] = emphasis;
    const Buf f = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    return tone(f, 110.0 * n) / tone(open, 110.0 * n);
}
// The ideal 4-pole low-pass (k = 0) magnitude at f: four equal one-poles.
double ideal4(double f, double fc) {
    const double p = 1.0 / (1.0 + (f / fc) * (f / fc));   // one pole, squared
    return p * p;
}
}  // namespace

TEST(dtype_ladder_cutoff_and_slope) {
    // At the cutoff, four equal poles give -12 dB (no emphasis).
    for (float fc : {440.0f, 880.0f, 1760.0f}) {
        const int n = int(fc / 110.0f);
        const double g = db(response(fc, 0.0f, n));
        std::printf("  cutoff %.0f Hz: %.1f dB at the cutoff\n", fc, g);
        CHECK_NEAR(g, -12.0, 1.5);
        // The pass band below.
        CHECK_NEAR(db(response(fc, 0.0f, 1)), db(ideal4(110.0, fc)), 1.0);
    }
    // 24 dB per octave well above the cutoff: 440 -> 1760 Hz vs 3520 Hz.
    const double a = db(response(440.0f, 0.0f, 16)), b = db(response(440.0f, 0.0f, 32));
    const double want = db(ideal4(3520.0, 440.0)) - db(ideal4(1760.0, 440.0));
    std::printf("  slope %.1f dB per octave (ideal %.1f)\n", b - a, want);
    CHECK_NEAR(b - a, want, 2.5);
    CHECK(b - a < -21.0);
}

TEST(dtype_resonance_peak) {
    // Emphasis 60 % (below self-oscillation): a peak at the cutoff, less bass.
    const double at0 = response(880.0f, 0.0f, 8), at6 = response(880.0f, 0.6f, 8);
    const double lo0 = response(880.0f, 0.0f, 1), lo6 = response(880.0f, 0.6f, 1);
    std::printf("  emphasis 60 %%: %+.1f dB at the cutoff, %+.1f dB at 110 Hz\n", db(at6 / at0), db(lo6 / lo0));
    CHECK(at6 > 4.0 * at0);
    CHECK(lo6 < 0.9 * lo0);
    CHECK(lo6 > 0.3 * lo0);   // the half bass make-up
}

TEST(dtype_self_oscillation_tracks_the_keyboard) {
    // Emphasis 100 %, nothing in the mixer: the ladder sings at the cutoff
    // and follows the keys by the Key Track amount.
    const double want[4] = {1.0, std::pow(2.0, 1.0 / 3.0), std::pow(2.0, 2.0 / 3.0), 2.0};
    for (int kt = 0; kt < 4; ++kt) {
        double f[3];
        for (int i = 0; i < 3; ++i) {
            const Buf x = play(whistle(440.0f, kt), 48.0f + 12.0f * float(i), 1.0f);
            CHECK(finite(x));
            CHECK(peak(x) < 1.0f);
            if (kt == dtype::kKeyFull && i == 1) {
                std::printf("  self-oscillation at C4, cutoff 440 Hz: %.1f Hz, %.3f rms\n", pitchHz(x, size_t(0.5 * kFs), x.size()),
                            rms(x, size_t(0.5 * kFs)));
                CHECK(rms(x, size_t(0.5 * kFs)) > 0.03);
            }
            f[i] = pitchHz(x, size_t(0.5 * kFs), x.size());
        }
        CHECK_NEAR(f[1] / f[0], want[kt], want[kt] * 0.03);
        CHECK_NEAR(f[2] / f[1], want[kt], want[kt] * 0.03);
        CHECK_NEAR(f[1], 440.0, 4.4);   // C4: no key offset, the cutoff within 1 %
    }
}

TEST(dtype_contour_amount) {
    // Contour 1/6 = +1 octave at a held full contour (Attack 0, Sustain 1).
    std::vector<float> c = whistle(330.0f, dtype::kKeyOff);
    c[dtype::kFAttack] = 0.0f;
    c[dtype::kFSustain] = 1.0f;
    const double f0 = pitchHz(play(c, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    c[dtype::kContour] = 1.0f / 6.0f;
    const double f1 = pitchHz(play(c, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    std::printf("  contour 1/6: %.1f -> %.1f Hz\n", f0, f1);
    CHECK_NEAR(f1 / f0, 2.0, 0.08);
    // A decaying contour: bright at the start, dark at Sustain 0; none at Contour 0.
    std::vector<float> d = open();
    d[dtype::kCutoff] = 200.0f;
    d[dtype::kContour] = 0.8f;
    d[dtype::kFAttack] = 0.0f;
    d[dtype::kFDecay] = 0.6f;
    d[dtype::kFSustain] = 0.0f;
    const Buf x = play(d, 45.0f, 1.0f);
    std::printf("  contour 80 %%: brightness %.3f early, %.3f late\n", bright(slice(x, 0.01, 0.06)), bright(slice(x, 0.7, 0.95)));
    CHECK(bright(slice(x, 0.01, 0.06)) > 2.0 * bright(slice(x, 0.7, 0.95)));
    d[dtype::kContour] = 0.0f;
    const Buf y = play(d, 45.0f, 1.0f);
    CHECK_NEAR(bright(slice(y, 0.01, 0.06)) / bright(slice(y, 0.7, 0.95)), 1.0, 0.2);
}

TEST(dtype_stable_under_fast_modulation) {
    // Emphasis at the top, audio-rate VCO 3 into the oscillators and the
    // filter, full feedback, the cutoff slammed every block: finite, bounded;
    // after the release the voice ends and the output is exact zeros.
    std::vector<float> c = richContext();
    c[dtype::kEmphasis] = 1.0f;
    c[dtype::kFeedback] = 1.0f;
    c[dtype::kVco3Range] = float(dtype::k2);
    c[dtype::kOscMod] = 1.0f;
    c[dtype::kFilterMod] = 1.0f;
    c[dtype::kWheel] = 1.0f;
    c[dtype::kModMix] = 0.0f;
    c[dtype::kLDecay] = 0.3f;
    Rig r(c);
    r.on(0, 96.0f);
    uint32_t seed = 7;
    for (int b = 0; b < 3 * kBlocksPerSec; ++b) {
        seed = seed * 1664525u + 1013904223u;
        c[dtype::kCutoff] = 20.0f + float(seed >> 8) * (1.0f / 16777216.0f) * 19980.0f;
        c[dtype::kEmphasis] = (seed & 0x100u) ? 1.0f : 0.0f;
        r.set(c);
        r.run(1);
        if (b == kBlocksPerSec) r.on(0, 24.0f, 1.0f, true, true);
    }
    CHECK(finite(r.L));
    CHECK(peak(r.L) < 2.0f);
    CHECK_EQ(subnormals(r.L), 0L);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

TEST(dtype_contours_and_decay_switch) {
    // Loudness: attack, decay to sustain, release (Decay switch On: the release is the Decay time).
    std::vector<float> c = open();
    c[dtype::kLAttack] = 0.15f;
    c[dtype::kLDecay] = 0.3f;
    c[dtype::kLSustain] = 0.4f;
    Rig r(c);
    r.on(0, 45.0f);
    r.run(int(1.5f * kBlocksPerSec));
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    auto w = [&r](double t) { return rms(r.L, size_t(t * kFs), size_t((t + 0.02) * kFs)); };
    CHECK(w(0.02) < w(0.07) && w(0.07) < w(0.13));   // attack
    CHECK(w(0.15) > 1.5 * w(1.2));                   // decay
    CHECK(w(1.2) > 0.2 * w(0.15));                   // holds at sustain
    CHECK(w(1.55) > 0.3 * w(1.45));                  // a 0.3 s release (to 1 %): still sounding 50 ms in
    CHECK(w(1.85) < 0.3 * w(1.45));
    CHECK(blocks < kBlocksPerSec);
    // Decay switch Off: a short release (~5 ms) whatever the Decay.
    c[dtype::kDecaySw] = 0.0f;
    c[dtype::kLDecay] = 3.0f;
    Rig s(c);
    s.on(0, 45.0f);
    s.run(int(0.5f * kBlocksPerSec));
    s.off(0);
    blocks = 0;
    while (s.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks <= 6);   // ~17 ms
    const double held = rms(s.L, size_t(0.4 * kFs), size_t(0.5 * kFs));
    CHECK(rms(s.L, s.L.size() - kBlock, s.L.size()) < 0.05 * held);
    // Sustain 0 ends a held note after the decay.
    c[dtype::kDecaySw] = 1.0f;
    c[dtype::kLDecay] = 0.1f;
    c[dtype::kLSustain] = 0.0f;
    Rig h(c);
    h.on(0, 45.0f);
    blocks = 0;
    while (h.run(1) && blocks < 20 * kBlocksPerSec) ++blocks;
    CHECK(blocks < kBlocksPerSec);
    // The filter contour releases with its own Decay time (switch On): after
    // the gate a long filter Decay keeps the sound bright, a short one closes
    // it (the loudness release held long in both).
    std::vector<float> f = open();
    f[dtype::kCutoff] = 150.0f;
    f[dtype::kContour] = 0.8f;
    f[dtype::kFAttack] = 0.0f;
    f[dtype::kFSustain] = 1.0f;
    f[dtype::kLDecay] = 2.0f;
    auto relBright = [](const std::vector<float>& v) {
        Rig g(v);
        g.on(0, 45.0f);
        g.run(kBlocksPerSec / 2);
        g.off(0);
        g.run(kBlocksPerSec / 4);
        return bright(slice(g.L, 0.6, 0.7));
    };
    f[dtype::kFDecay] = 2.0f;
    const double slow = relBright(f);
    f[dtype::kFDecay] = 0.05f;
    const double fast = relBright(f);
    CHECK(slow > 1.5 * fast);
}

namespace {
// The played frequency of a triangle on VCO 1 (zero crossings) at t seconds.
std::vector<float> glideCtx(int mode, float seconds) {
    std::vector<float> c = only(0, dtype::kTri, 0.4f);
    c[dtype::kGlideMode] = float(mode);
    c[dtype::kGlideTime] = seconds;
    return c;
}
double semis(double f) { return 69.0 + 12.0 * std::log2(f / 440.0); }
}  // namespace

TEST(dtype_glide_time_and_modes) {
    // Always: from the slot's last note, exponential, 95 % of the way at Glide Time.
    {
        Rig r(glideCtx(dtype::kAlways, 0.5f));
        r.on(0, 57.0f);
        r.run(kBlocksPerSec / 4);
        r.off(0);
        r.run(kBlocksPerSec / 8);
        r.on(1, 69.0f);   // a fresh voice, not overlapping
        const size_t t0 = r.L.size();
        r.run(kBlocksPerSec);
        const Buf x(r.L.begin() + long(t0), r.L.end());
        const double s1 = semis(freqAt(x, 0.03)), s5 = semis(freqAt(x, 0.5)), s9 = semis(freqAt(x, 0.95));
        std::printf("  glide 0.5 s: %.2f, %.2f, %.2f semitones\n", s1, s5, s9);
        CHECK(s1 < 59.5);
        CHECK_NEAR(s5, 57.0 + 0.95 * 12.0, 0.35);
        CHECK_NEAR(s9, 69.0, 0.1);
    }
    // Overlap / Legato Overlap: only from a held note; Legato like Always.
    struct Case {
        int mode;
        bool overlap;
        bool glides;
    };
    const Case cases[] = {{dtype::kOverlap, false, false},       {dtype::kOverlap, true, true},
                          {dtype::kLegato, false, true},         {dtype::kLegato, true, true},
                          {dtype::kLegatoOverlap, false, false}, {dtype::kLegatoOverlap, true, true},
                          {dtype::kAlways, false, true}};
    for (const Case& k : cases) {
        Rig r(glideCtx(k.mode, 0.3f));
        r.on(0, 57.0f);
        r.run(kBlocksPerSec / 4);
        if (!k.overlap) r.off(0);
        r.run(4);
        r.on(0, 69.0f, 1.0f, true, k.overlap);
        const size_t t0 = r.L.size();
        r.run(kBlocksPerSec / 2);
        const double s = semis(freqAt(Buf(r.L.begin() + long(t0), r.L.end()), 0.05));
        if ((s < 67.0) != k.glides) std::printf("       mode %d overlap %d: %.2f semitones at 50 ms\n", k.mode, k.overlap, s);
        CHECK((s < 67.0) == k.glides);
    }
    // A Glide step FX owns the pitch start: no patch glide.
    Rig g(glideCtx(dtype::kAlways, 0.5f));
    g.on(0, 57.0f);
    g.run(kBlocksPerSec / 4);
    g.on(0, 69.0f, 1.0f, true, true, true);
    const size_t t0 = g.L.size();
    g.run(kBlocksPerSec / 4);
    CHECK_NEAR(semis(freqAt(Buf(g.L.begin() + long(t0), g.L.end()), 0.03)), 69.0, 0.1);
    // Glide Time 0: no glide in any mode.
    Rig z(glideCtx(dtype::kAlways, 0.0f));
    z.on(0, 57.0f);
    z.run(kBlocksPerSec / 4);
    z.on(0, 69.0f, 1.0f, true, true);
    const size_t z0 = z.L.size();
    z.run(kBlocksPerSec / 4);
    CHECK_NEAR(semis(freqAt(Buf(z.L.begin() + long(z0), z.L.end()), 0.02)), 69.0, 0.1);
}

TEST(dtype_legato_skips_retrigger) {
    // A held note decays to a low sustain; an overlapping note in a Legato
    // mode keeps the level, in Always / Overlap it attacks again.
    auto after = [](int mode) {
        std::vector<float> c = open();
        c[dtype::kLAttack] = 0.0f;
        c[dtype::kLDecay] = 0.1f;
        c[dtype::kLSustain] = 0.2f;
        c[dtype::kGlideMode] = float(mode);
        Rig r(c);
        r.on(0, 45.0f);
        r.run(kBlocksPerSec / 2);
        const double sus = rms(r.L, r.L.size() - 10 * kBlock, r.L.size());
        r.on(0, 52.0f, 1.0f, true, true);
        const size_t t0 = r.L.size();
        r.run(kBlocksPerSec / 10);
        return rms(r.L, t0 + kBlock, t0 + 6 * kBlock) / sus;
    };
    for (int m : {int(dtype::kLegato), int(dtype::kLegatoOverlap)}) CHECK(after(m) < 1.3);
    for (int m : {int(dtype::kAlways), int(dtype::kOverlap)}) CHECK(after(m) > 2.0);
}

namespace {
// The Init patch (or an edited one) through a real SynthBank slot at the
// default slot gain (-9 dB).
struct BankSlot {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
    float peak = 0.0f;
    long clips = 0;
    Buf out;
    BankSlot(int32_t engine, int poly, const std::vector<float>* vals = nullptr) {
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
                peak = std::max(peak, a);
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
    return s.peak;
}
double dbfs(float x) { return 20.0 * std::log10(x > 1e-9f ? double(x) : 1e-9); }
}  // namespace

TEST(dtype_last_note_priority_through_the_bank) {
    // Poly 1: a second key while the first is held takes the voice (last
    // note), legato by default (no new attack), one voice.
    std::vector<float> c = only(0, dtype::kTri, 0.4f);
    BankSlot s(dtype::kEngineId, 1, &c);
    s.on(0, 48);
    s.run(kBlocksPerSec / 2);
    s.on(1, 60);
    const size_t t0 = s.out.size();
    s.run(kBlocksPerSec / 2);
    CHECK_EQ(s.bank.voicesOf(0), 1);
    const Buf x(s.out.begin() + long(t0), s.out.end());
    CHECK_NEAR(pitchHz(x, size_t(0.1 * kFs), x.size()), hz(60.0f), hz(60.0f) * 0.005);
    s.on(2, 55);
    const size_t t1 = s.out.size();
    s.run(kBlocksPerSec / 2);
    const Buf y(s.out.begin() + long(t1), s.out.end());
    CHECK_NEAR(pitchHz(y, size_t(0.1 * kFs), y.size()), hz(55.0f), hz(55.0f) * 0.005);
    CHECK_EQ(s.bank.voicesOf(0), 1);
}

TEST(dtype_init_retriggers_sequenced_notes) {
    // A tracker line plays note after note on one track without OFF steps:
    // the framework hands each to the held Poly 1 voice as an overlap. Init
    // (Glide Mode Always) restarts the contours on every note.
    std::vector<float> c = initValues();
    c[dtype::kLDecay] = 0.1f;
    c[dtype::kLSustain] = 0.2f;
    c[dtype::kOscMod] = 0.0f;
    c[dtype::kVco2Level] = 0.0f;   // no beating between VCO 1 and 2 across the hits
    BankSlot s(dtype::kEngineId, 1, &c);
    double first = 0, last = 0;
    for (int k = 0; k < 4; ++k) {
        const size_t t0 = s.out.size();
        s.on(0, 45);
        s.run(kBlocksPerSec / 4);
        const double hit = rms(s.out, t0, t0 + 10 * kBlock);
        if (k == 0) first = hit;
        last = hit;
    }
    std::printf("  repeated notes: first hit %.4f, fourth %.4f rms\n", first, last);
    CHECK(last > 0.8 * first);
}

TEST(dtype_wheel_vibrato_and_filter_mod) {
    // VCO 3 at Lo, off the keyboard (~2 Hz triangle), into VCO 1 by Osc Mod x Wheel:
    // linear FM, the frequency x (1 +- 0.9 (Osc Mod x Wheel)^2), the pitch centre kept.
    auto spread = [](float wheel) {
        std::vector<float> c = only(0, dtype::kTri, 0.4f);
        c[dtype::kVco3Range] = float(dtype::kLo);
        c[dtype::kVco3Kb] = 0.0f;
        c[dtype::kVco3Wave] = float(dtype::kTri);
        c[dtype::kOscMod] = 1.0f;
        c[dtype::kWheel] = wheel;
        const Buf x = slice(play(c, 57.0f, 1.5f), 0.1, 1.5);
        std::vector<double> f = freqTrack(x);
        if (f.size() < 100) return -1.0;   // silent
        std::sort(f.begin(), f.end());
        const double lo = f[f.size() / 50], hi = f[f.size() - 1 - f.size() / 50];
        return 12.0 * std::log2(hi / lo);   // semitones, peak to peak
    };
    const double s0 = spread(0.0f), s5 = spread(0.5f), s1 = spread(1.0f);
    std::printf("  vibrato peak to peak: wheel 0 %.2f, 50 %% %.2f, 100 %% %.2f semitones\n", s0, s5, s1);
    CHECK(s0 >= 0.0 && s0 < 0.1);
    CHECK_NEAR(s5, 12.0 * std::log2(1.225 / 0.775), 1.0);   // +-22.5 %
    CHECK(s1 > 30.0);                                        // +-90 %: 1.9 / 0.1
    // The centre: cycles per second unchanged by the FM (VCO 3 a slow square
    // or narrow pulse, the sources that are not symmetric around their peak).
    for (int src : {int(dtype::kSquare), int(dtype::kNarrow)}) {
        std::vector<float> c = only(0, dtype::kTri, 0.4f);
        c[dtype::kVco3Range] = float(dtype::kLo);
        c[dtype::kVco3Kb] = 0.0f;
        c[dtype::kVco3Wave] = float(src);
        c[dtype::kOscMod] = 0.7f;
        c[dtype::kWheel] = 1.0f;
        const double span = 5.0 / (hz(48.0f) / 64.0);   // five periods of VCO 3 (C3 / 64)
        const double cycles = double(freqTrack(slice(play(c, 57.0f, 3.0f), 0.4, 0.4 + span)).size());
        std::printf("  vibrato centre (VCO 3 wave %d): %.1f Hz\n", src, cycles / span);
        CHECK_NEAR(cycles / span, 220.0, 220.0 * 0.01);
    }
    // Filter Mod x Wheel moves the self-oscillating ladder (+-4 octaves at full).
    std::vector<float> w = whistle(440.0f, dtype::kKeyOff);
    w[dtype::kVco3Range] = float(dtype::kLo);
    w[dtype::kVco3Kb] = 0.0f;
    w[dtype::kFilterMod] = 1.0f;
    w[dtype::kWheel] = 1.0f;
    const Buf x = slice(play(w, 60.0f, 1.5f), 0.3, 1.5);
    std::vector<double> f = freqTrack(x);
    std::sort(f.begin(), f.end());
    CHECK(f.size() > 100);
    if (f.size() > 100) {
        const double lo = f[f.size() / 20], hi = f[f.size() - 1 - f.size() / 20];
        std::printf("  filter mod: %.0f .. %.0f Hz\n", lo, hi);
        CHECK(hi / lo > 4.0);
    }
    // Noise as the source (Mod Mix 100 %): random pitch wobble.
    std::vector<float> n = only(0, dtype::kTri, 0.4f);
    n[dtype::kModMix] = 1.0f;
    n[dtype::kOscMod] = 0.7f;
    n[dtype::kWheel] = 1.0f;
    CHECK(rmsDiff(play(n, 57.0f), play(only(0, dtype::kTri, 0.4f), 57.0f)) > 0.01);
}

TEST(dtype_fm_keeps_the_waves_whole) {
    // Full Osc Mod x Wheel from the widest sources (a slow narrow pulse
    // holding its +1.8 peak, pink noise) on high square and triangle
    // carriers: no edge or corner may be missed (a missed one shifts the
    // wave: DC). The ladder at 20 Hz keeps only the slow part.
    for (int car : {int(dtype::kSquare), int(dtype::kTri)})
        for (int src = 0; src < 2; ++src) {
            std::vector<float> c = only(0, car, 0.3f);
            c[dtype::kVco1Range] = float(dtype::k2);
            c[dtype::kVco3Wave] = float(dtype::kNarrow);
            c[dtype::kVco3Range] = float(dtype::kLo);
            c[dtype::kVco3Kb] = 0.0f;
            c[dtype::kVco3Tune] = 12.0f;   // 4 Hz: the 10 % peak lasts 25 ms
            c[dtype::kModMix] = float(src);
            c[dtype::kNoiseColour] = float(dtype::kPink);
            c[dtype::kOscMod] = 1.0f;
            c[dtype::kWheel] = 1.0f;
            c[dtype::kCutoff] = 20.0f;
            const Buf x = slice(play(c, 103.0f, 1.5f), 0.3, 1.5);   // 2' of G7: at the increment ceiling
            double worst = 0;
            const size_t w = size_t(0.02 * kFs);
            for (size_t a = 0; a + w <= x.size(); a += w / 2) {
                double m = 0;
                for (size_t i = a; i < a + w; ++i) m += x[i];
                worst = std::max(worst, std::fabs(m / double(w)));
            }
            if (!(worst < 0.01)) std::printf("       carrier %d source %d: DC up to %.4f\n", car, src, worst);
            CHECK(worst < 0.01);
        }
}

TEST(dtype_audio_rate_fm_sidebands) {
    // VCO 3 at audio rate (keyboard on, a fifth up, silent in the mix) into
    // a triangle on VCO 1 at 220 Hz: sidebands at 220 +- 329.6 Hz.
    std::vector<float> c = only(0, dtype::kTri, 0.3f);
    c[dtype::kVco3Wave] = float(dtype::kTri);
    c[dtype::kVco3Tune] = 7.0f;
    c[dtype::kVco3Kb] = 1.0f;
    c[dtype::kVco3Range] = float(dtype::k8);
    c[dtype::kWheel] = 1.0f;
    c[dtype::kOscMod] = 0.0f;
    const Buf dry = slice(play(c, 57.0f, 0.6f), 0.2, 0.6);
    c[dtype::kOscMod] = 0.7f;
    const Buf fm = slice(play(c, 57.0f, 0.6f), 0.2, 0.6);
    const double fmod = hz(64.0f), car = 220.0;
    const double up = tone(fm, car + fmod) / tone(fm, car), dn = tone(fm, fmod - car) / tone(fm, car);
    const double up0 = tone(dry, car + fmod) / tone(dry, car);
    std::printf("  FM sidebands: %.1f dB, %.1f dB (dry %.1f dB)\n", db(up), db(dn), db(up0));
    CHECK(up > 0.05 && dn > 0.05);
    CHECK(up0 < 0.01);
}

TEST(dtype_parameter_jumps_do_not_click) {
    // A macro slammed between two blocks (no framework ramp): no step, the
    // output moves at most 2x as fast as the steady sound before or after it
    // (a cutoff slammed 4 octaves sweeps across one block).
    struct Jump {
        int param;
        float a, b;
    };
    const Jump jumps[] = {{dtype::kEmphasis, 0.1f, 0.9f}, {dtype::kCutoff, 300.0f, 5000.0f}, {dtype::kVco1Level, 0.1f, 1.0f},
                          {dtype::kContour, 0.0f, 0.6f},  {dtype::kFeedback, 0.0f, 1.0f},    {dtype::kVco2Level, 0.0f, 1.0f},
                          {dtype::kNoiseLevel, 0.0f, 1.0f}};
    for (const Jump& j : jumps) {
        std::vector<float> c = open();
        c[dtype::kVco1Wave] = float(dtype::kTri);
        c[dtype::kCutoff] = 600.0f;
        c[dtype::kFSustain] = 1.0f;
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

TEST(dtype_no_nan_or_denormals) {
    uint32_t seed = 61;
    auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) * (1.0f / 16777216.0f);
    };
    int bad = 0;
    long sub = 0;
    for (int set = 0; set < 120; ++set) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[i] = range(i)->min + rnd() * (range(i)->max - range(i)->min);
        Rig r(a);
        r.on(0, float(int(rnd() * 127.0f)), rnd());
        r.on(1, float(int(rnd() * 127.0f)), rnd());
        r.run(kBlocksPerSec);   // 1 s: slow instabilities show
        r.on(1, float(int(rnd() * 127.0f)), rnd(), true, rnd() < 0.5f);
        r.run(kBlocksPerSec / 4);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec / 4);
        // Two voices at unity gain; the emphasis make-up (up to x3.65) lifts edge transients.
        if (!finite(r.L) || peak(r.L) > 2.5f) {
            ++bad;
            std::printf("       set %d: peak %.3f%s\n", set, peak(r.L), finite(r.L) ? "" : " (not finite)");
        }
        sub += subnormals(r.L);
    }
    CHECK_EQ(bad, 0);
    CHECK_EQ(sub, 0L);
    // Extremes: every parameter at its max (and min), very high and low notes.
    for (int ext = 0; ext < 2; ++ext) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[i] = ext ? range(i)->max : range(i)->min;
        a[dtype::kVco1Level] = 1.0f;
        Rig r(a);
        r.on(0, 127.0f);
        r.on(1, 0.0f);
        r.run(kBlocksPerSec);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec);
        CHECK(finite(r.L));
        CHECK(peak(r.L) < 2.0f);
        CHECK_EQ(subnormals(r.L), 0L);
    }
    // A long, quiet release tail never goes subnormal.
    std::vector<float> c = initValues();
    c[dtype::kLDecay] = 10.0f;
    c[dtype::kLSustain] = 1.0f;
    Rig t(c);
    t.on(0, 40.0f);
    t.run(50);
    t.off(0);
    int blocks = 0;
    while (t.run(1) && blocks < 60 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 60 * kBlocksPerSec);
    CHECK_EQ(subnormals(t.L), 0L);
    // The self-oscillating ladder with nothing in it, cutoff at the bottom.
    std::vector<float> w = whistle(20.0f, dtype::kKeyOff);
    Rig q(w);
    q.on(0, 0.0f);
    q.run(5 * kBlocksPerSec);
    CHECK(finite(q.L));
    CHECK_EQ(subnormals(q.L), 0L);
}

// Fresh voices report full level at the note on (no steal before their
// first block); afterwards level follows the loudness contour.
TEST(dtype_voice_level) {
    std::vector<float> c = open();
    c[dtype::kLAttack] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    const float lv = r.v[0].level;
    r.on(0, 62.0f, 1.0f, true, false);   // a reused voice keeps its level
    CHECK_EQ(r.v[0].level, lv);
}

TEST(dtype_volume_is_never_applied) {
    std::vector<float> a = initValues(), b = initValues();
    a[dtype::kVolume] = 0.0f;
    b[dtype::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

// The Init patch through a real SynthBank slot at the default slot gain
// (-9 dB): one voice at ACD's Init level (within 1.5 dB, the FAT / WTFM rule
// of tests/synth_levels_test.cpp) and well below full scale; Poly 4 chords
// (the slot setting) stay 1 dB below full scale, nothing clipped.
TEST(dtype_level_matches_the_synth_conventions) {
    long c1 = 0, c4 = 0;
    const float p1 = chordPeak(dtype::kEngineId, 1, c1);
    const float p4 = chordPeak(dtype::kEngineId, 4, c4);
#if defined(OT_MOD_ACD1)
    long ca = 0;
    const float pa = chordPeak(kSynthEngineAcd, 1, ca);
    std::printf("  D-Type Init at -9 dB: 1 voice peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(pa),
                dbfs(p4));
    CHECK(std::fabs(dbfs(p1) - dbfs(pa)) < 1.5);
#else
    std::printf("  D-Type Init at -9 dB: 1 voice peak %.1f dBFS, 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(p4));
#endif
    CHECK(dbfs(p1) < -6.0);
    CHECK(dbfs(p1) > -20.0);
    CHECK(dbfs(p4) < -1.0);
    CHECK_EQ(c1, 0L);
    CHECK_EQ(c4, 0L);
}

TEST(dtype_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == dtype::kVolume) continue;
        const PatchParamRange* g = range(i);
        std::vector<float> a = ctx, b = ctx;
        a[i] = g->min;
        b[i] = g->max;
        const double d = rmsDiff(renderSequence(a), renderSequence(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

// Golden hashes (host build, -ffp-contract=off): a rich note sequence on the
// engine alone and Init lines through a SynthBank slot. Any change of the
// sound shows here; update the values only for an intended change.
namespace {
uint64_t hashOf(const Buf& l, const Buf& r) {
    uint64_t h = 1469598103934665603ull;
    auto eat = [&h](const Buf& x) {
        for (float s : x) {
            uint32_t b;
            std::memcpy(&b, &s, 4);
            for (int k = 0; k < 4; ++k) {
                h ^= (b >> (8 * k)) & 0xFFu;
                h *= 1099511628211ull;
            }
        }
    };
    eat(l);
    eat(r);
    return h;
}
uint64_t engineHash() {
    std::vector<float> c = richContext();
    c[dtype::kNoiseColour] = float(dtype::kPink);
    Rig r(c);
    r.run(3);
    r.on(0, 45.0f, 0.6f);
    r.run(kBlocksPerSec / 3);
    r.on(0, 57.0f, 0.9f, true, true);
    r.run(kBlocksPerSec / 3);
    c[dtype::kEmphasis] = 0.9f;
    c[dtype::kVco1Wave] = float(dtype::kNarrow);
    r.set(c);
    r.on(0, 52.0f, 0.9f, true, false);
    r.run(kBlocksPerSec / 3);
    r.off(0);
    r.run(kBlocksPerSec / 2);
    return hashOf(r.L, r.R);
}
uint64_t bankHash() {
    BankSlot s(dtype::kEngineId, 1);
    Buf L, R;
    const int line[6] = {36, 48, 43, 36, 55, 31};
    for (int k = 0; k < 6; ++k) {
        s.on(0, line[k]);
        for (int b = 0; b < 40; ++b) {
            s.block();
            L.insert(L.end(), s.L, s.L + kBlock);
            R.insert(R.end(), s.R, s.R + kBlock);
        }
    }
    s.off(0);
    for (int b = 0; b < 200; ++b) {
        s.block();
        L.insert(L.end(), s.L, s.L + kBlock);
        R.insert(R.end(), s.R, s.R + kBlock);
    }
    return hashOf(L, R);
}
}  // namespace

TEST(dtype_golden_hashes) {
    const uint64_t e = engineHash(), b = bankHash();
    std::printf("  D-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x6a3825513bfd0a67ull));
    CHECK_EQ(b, uint64_t(0xc251cb889e4c46bfull));
}

// --- Performance filter offset (issue #23) ---------------------------------------------
// SynthVoice::cutoffOct (octaves, filled by the framework from the track's
// Performance filter) moves the D-Type cutoff: Cutoff x 2^offset, inside the
// engine's own clamps. A patch at 1600 Hz shifted by -2 octaves sounds like
// the same patch at 400 Hz; 0 leaves the render bit-identical.

namespace {
namespace perfcut {

// One note (pitch 48, 40 blocks) from the Init patch with its cutoff set by
// `cut(patch, hz)`, the voice's cutoffOct = `oct`.
template <class Cut>
std::vector<float> render(Cut cut, float hz, float oct) {
    const int32_t engine = dtype::kEngineId;
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
    CHECK(synthParamSet(p, dtype::kCutoff, hz));
}

}  // namespace perfcut
}  // namespace

TEST(dtype_perf_cutoff_offset_moves_the_cutoff) {
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
