// JU6 one-DCO poly engine (opentracker-synths modules/ju6, engine 0x81):
// registered with a valid descriptor, silent when idle, the DCO at A 440,
// polyBLEP aliasing below a naive saw, the HPF positions in order, the LPF
// cutoff and resonance (self-oscillation at the top), the ADSR stages and the
// Gate VCA, LFO rate and delay, no NaN or denormals (also a held Sustain 0),
// the voice level for stealing, no click on a Resonance jump, the slot level of the
// other engines, the patch file round trip, Volume never applied, every
// parameter audible, random parameter sets stay finite.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "ju6.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"

using namespace ot;
using namespace ot::audio;

namespace {

constexpr float kFs = ot::audio::kSampleRate;   // (model/project.h has a kSampleRate too)
constexpr int kN = ju6::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(ju6::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

// One engine voice pool driven directly (no framework).
struct Rig {
    ISynthEngine* e = synthEngineFor(ju6::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    std::vector<float> L, R;
    int slot = 0;        // the slot index slotPost gets (its chorus line)
    bool tail = false;   // slotPost's last answer

    explicit Rig(const std::vector<float>& vals, int s = 0) : slot(s) {
        p.engine = ju6::kEngineId;
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
    void on(int k, float pitch, float velocity = 1.0f, bool reused = false, bool overlap = false) {
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
            tail = e->slotPost(slot, scratch, p, l, r);   // the chorus on the voices' sum
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

// A plain context: saw only, filter wide open, no envelope on the filter,
// VCA held at full sustain, no LFO, flat HPF.
std::vector<float> open() {
    std::vector<float> c = initValues();
    c[ju6::kSaw] = 1.0f;
    c[ju6::kPulse] = 0.0f;
    c[ju6::kSub] = 0.0f;
    c[ju6::kNoise] = 0.0f;
    c[ju6::kHpf] = 1.0f;
    c[ju6::kCutoff] = 20000.0f;
    c[ju6::kResonance] = 0.0f;
    c[ju6::kEnvAmt] = 0.0f;
    c[ju6::kKeyFollow] = 0.0f;
    c[ju6::kAttack] = 0.0f;
    c[ju6::kSustain] = 1.0f;
    c[ju6::kRelease] = 0.05f;
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

std::vector<float> richContext() {
    std::vector<float> c = initValues();
    c[ju6::kPulse] = 1.0f;
    c[ju6::kPw] = 0.5f;
    c[ju6::kSub] = 0.5f;
    c[ju6::kNoise] = 0.2f;
    c[ju6::kVibrato] = 0.3f;
    c[ju6::kHpf] = 2.0f;
    c[ju6::kCutoff] = 900.0f;
    c[ju6::kResonance] = 0.5f;
    c[ju6::kEnvAmt] = 0.5f;
    c[ju6::kVcfLfo] = 0.3f;
    c[ju6::kKeyFollow] = 0.5f;
    c[ju6::kVelVcf] = 0.5f;
    c[ju6::kAttack] = 0.02f;
    c[ju6::kDecay] = 0.05f;
    c[ju6::kSustain] = 0.5f;
    c[ju6::kRelease] = 0.2f;
    c[ju6::kLfoRate] = 6.0f;
    c[ju6::kLfoDelay] = 0.1f;
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

}  // namespace

TEST(ju6_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(ju6::kEngineId);
    CHECK(e != nullptr);
    CHECK_STR(e->name(), "JU6");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(ju6::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(ju6::kEngineId));
    CHECK_STR(d->name, "JU6");
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(ju6::kVolume));
    CHECK_EQ(patchVolumeParam(ju6::kEngineId), int(ju6::kVolume));
    CHECK_STR(patchEngineName(ju6::kEngineId), "JU6");
    CHECK(d->sectionCount <= synthui::kMaxSections);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    CHECK_EQ(synthWidthParam(ju6::kEngineId), kN);
    CHECK_EQ(synthSpreadParam(ju6::kEngineId), kN + 1);
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
        for (int k = 0; k < sec.count; ++k) {
            CHECK(sec.params[k] < kN + 2);   // + Width, Spread
            if (sec.params[k] < kN) ++seen[sec.params[k]];
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
    CHECK(patchInit(ju6::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(ju6::kVolume));
}

TEST(ju6_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(ju6::kEngineId, &p));
    const float vals[][2] = {{ju6::kPulse, 1}, {ju6::kPw, 0.7f}, {ju6::kHpf, 3}, {ju6::kCutoff, 432.5f},
                             {ju6::kEnvAmt, -0.6f}, {ju6::kVcaMode, 1}, {ju6::kLfoDelay, 2.5f}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(ju6::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(ju6::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    // Out of range values are refused.
    CHECK(!patchParamInRange(ju6::kEngineId, ju6::kHpf, 4.0f));
    CHECK(!patchParamInRange(ju6::kEngineId, ju6::kEnvAmt, -1.5f));
}

TEST(ju6_silent_when_idle) {
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
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

TEST(ju6_dco_pitch_a440) {
    for (float pitch : {69.0f, 57.0f, 45.0f}) {
        const Buf x = play(open(), pitch, 0.5f);
        CHECK(finite(x));
        const double f = pitchHz(x, kA, kB);
        CHECK_NEAR(f, hz(pitch), hz(pitch) * 0.002);
        if (pitch == 69.0f) std::printf("  A4: %.2f Hz\n", f);
    }
    // The sub oscillator alone is one octave down; the pulse alone keeps the pitch.
    std::vector<float> c = open();
    c[ju6::kSaw] = 0.0f;
    c[ju6::kSub] = 1.0f;
    CHECK_NEAR(pitchHz(play(c, 69.0f), kA, kB), 220.0, 220.0 * 0.003);
    c[ju6::kSub] = 0.0f;
    c[ju6::kPulse] = 1.0f;
    c[ju6::kPwMode] = float(ju6::kPwManual);
    c[ju6::kPw] = 0.3f;
    CHECK_NEAR(pitchHz(play(c, 69.0f), kA, kB), 440.0, 440.0 * 0.003);
}

TEST(ju6_polyblep_lowers_aliasing) {
    // Saw at 3520 Hz (A7): the partials above Nyquist of a naive saw fold
    // down between the harmonics. Compared below 8 kHz, where the open
    // filter is flat.
    const float pitch = 105.0f;
    const double f0 = hz(pitch);
    const Buf x = play(open(), pitch, 0.4f);
    const Buf seg = slice(x, 0.2, 0.2 + 4096.0 / kFs);
    Buf naive(seg.size());
    double ph = 0.37;
    for (float& s : naive) {
        s = float(2.0 * ph - 1.0);
        ph += f0 / kFs;
        if (ph >= 1.0) ph -= 1.0;
    }
    const double rj = aliasRatio(seg, f0, 8000.0), rn = aliasRatio(naive, f0, 8000.0);
    std::printf("  alias / harmonic power below 8 kHz: polyBLEP %.2f dB, naive %.2f dB\n", 10 * std::log10(rj),
                10 * std::log10(rn));
    CHECK(10 * std::log10(rj) < 10 * std::log10(rn) - 10.0);
}

TEST(ju6_hpf_positions_order) {
    // The fundamental of a low saw (65 Hz) and a mid partial (1.5 kHz).
    std::vector<float> c = open();
    double low[4], mid[4];
    for (int k = 0; k < 4; ++k) {
        c[ju6::kHpf] = float(k);
        const Buf x = slice(play(c, 36.0f), 0.15, 0.45);
        low[k] = tone(x, hz(36.0f));
        mid[k] = tone(x, 23.0 * hz(36.0f));
        std::printf("  HPF %d: 65 Hz %.1f dB, 1.5 kHz %.1f dB\n", k, 20 * std::log10(low[k]), 20 * std::log10(mid[k]));
    }
    CHECK(low[0] > low[1] * 1.15);   // 0: bass boost
    CHECK(low[1] > low[2] * 1.5);
    CHECK(low[2] > low[3] * 1.5);
    for (int k = 0; k < 4; ++k) CHECK(mid[k] > 0.7 * mid[1]);   // mids pass
}

TEST(ju6_lpf_cutoff_and_resonance) {
    std::vector<float> c = open();
    // Lower cutoff = darker.
    c[ju6::kCutoff] = 6000.0f;
    const double b6 = bright(slice(play(c), 0.15, 0.45));
    c[ju6::kCutoff] = 600.0f;
    const double b06 = bright(slice(play(c), 0.15, 0.45));
    CHECK(b06 < 0.6 * b6);
    // 24 dB/oct: two octaves above the cutoff a partial is well down.
    c[ju6::kCutoff] = 500.0f;
    const Buf x = slice(play(c, 36.0f), 0.15, 0.45);
    const double at = tone(x, 8 * hz(36.0f)), above = tone(x, 31 * hz(36.0f));   // 523 Hz, 2.03 kHz
    CHECK(20 * std::log10(above / at) < -30.0);   // a saw alone drops ~12 dB over 2 octaves
    // Resonance raises the peak near the cutoff.
    c[ju6::kCutoff] = 1000.0f;
    c[ju6::kResonance] = 0.0f;
    const Buf r0 = slice(play(c, 36.0f), 0.15, 0.45);
    c[ju6::kResonance] = 0.8f;
    const Buf r8 = slice(play(c, 36.0f), 0.15, 0.45);
    const double h = std::round(1000.0 / hz(36.0f)) * hz(36.0f);
    std::printf("  partial near the cutoff: Res 0 %.1f dB, Res 80 %% %.1f dB\n", 20 * std::log10(tone(r0, h)),
                20 * std::log10(tone(r8, h)));
    CHECK(tone(r8, h) > 2.0 * tone(r0, h));
    // Full resonance self-oscillates near the cutoff with every source off;
    // half resonance stays silent.
    std::vector<float> s = open();
    s[ju6::kSaw] = 0.0f;
    s[ju6::kCutoff] = 1000.0f;
    s[ju6::kResonance] = 1.0f;
    const Buf osc = play(s, 60.0f, 1.0f);
    CHECK(finite(osc));
    const double ro = rms(osc, size_t(0.5 * kFs), size_t(0.95 * kFs));
    const double fo = pitchHz(osc, size_t(0.5 * kFs), size_t(0.95 * kFs));
    std::printf("  self-oscillation at Cutoff 1 kHz: %.1f dBFS rms, %.0f Hz\n", 20 * std::log10(ro), fo);
    CHECK(ro > 0.03);
    CHECK(peak(osc) < 1.0f);
    CHECK(fo > 700.0 && fo < 1300.0);
    s[ju6::kResonance] = 0.5f;
    CHECK(rms(play(s, 60.0f, 1.0f), size_t(0.5 * kFs)) < 1e-3);
}

TEST(ju6_envelope_stages) {
    // VCA Env: attack rises, decay falls to the sustain level, release ends.
    std::vector<float> c = open();
    c[ju6::kAttack] = 0.2f;
    c[ju6::kDecay] = 0.2f;
    c[ju6::kSustain] = 0.5f;
    c[ju6::kRelease] = 0.3f;
    Rig r(c);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec * 2);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 5 * kBlocksPerSec) ++blocks;
    auto w = [&](double t) { return rms(r.L, size_t(t * kFs), size_t((t + 0.02) * kFs)); };
    std::printf("  env rms: 0.05 s %.3f, 0.19 s %.3f, 0.6 s %.3f, 1.9 s %.3f, release +0.15 s %.3f\n", w(0.05), w(0.18),
                w(0.6), w(1.9), w(2.15));
    CHECK(w(0.02) < w(0.1) && w(0.1) < w(0.18));   // attack
    CHECK(w(0.18) > 1.5 * w(0.6));                 // decay
    CHECK_NEAR(w(0.6) / w(0.18), 0.5, 0.12);       // sustain level relative to the peak
    CHECK_NEAR(w(1.9) / w(0.6), 1.0, 0.03);        // held
    CHECK(w(2.15) < 0.6 * w(1.9));                 // release
    CHECK(blocks < kBlocksPerSec);                 // ends within ~1 s
    // VCA Gate: full level at once regardless of the attack, ends right after the gate.
    c[ju6::kVcaMode] = float(ju6::kVcaGate);
    c[ju6::kAttack] = 2.0f;
    c[ju6::kRelease] = 5.0f;
    Rig g(c);
    g.on(0, 57.0f);
    g.run(kBlocksPerSec / 2);
    const double held = rms(g.L, size_t(0.02 * kFs), size_t(0.04 * kFs));
    CHECK(held > 0.8 * rms(g.L, size_t(0.4 * kFs)));
    g.off(0);
    int gb = 0;
    while (g.run(1) && gb < kBlocksPerSec) ++gb;
    CHECK(gb <= 3);
    // The envelope drives the filter too (VCA Gate, Env 100 %: a bright attack, a dark tail).
    std::vector<float> f = open();
    f[ju6::kVcaMode] = float(ju6::kVcaGate);
    f[ju6::kCutoff] = 200.0f;
    f[ju6::kEnvAmt] = 1.0f;
    f[ju6::kDecay] = 0.15f;
    f[ju6::kSustain] = 0.0f;
    const Buf fx = play(f, 48.0f, 1.0f);
    CHECK(bright(slice(fx, 0.01, 0.06)) > 2.0 * bright(slice(fx, 0.7, 0.95)));
    // Negative Env closes the filter at the attack instead.
    f[ju6::kCutoff] = 3000.0f;
    f[ju6::kEnvAmt] = -1.0f;
    const Buf nx = play(f, 48.0f, 1.0f);
    CHECK(bright(slice(nx, 0.01, 0.06)) < 0.7 * bright(slice(nx, 0.7, 0.95)));
}

TEST(ju6_lfo_rate_and_delay) {
    // Vibrato on a near-sine (saw through a low cutoff); the LFO rate shows
    // as the rate of the frequency track's swings, the delay as a quiet start.
    std::vector<float> c = open();
    c[ju6::kCutoff] = 400.0f;
    c[ju6::kVibrato] = 1.0f;
    for (float rate : {2.0f, 6.0f}) {
        c[ju6::kLfoRate] = rate;
        c[ju6::kLfoDelay] = 1.0f;
        const Buf x = play(c, 57.0f, 3.0f);
        std::vector<double> at;
        const std::vector<double> f = freqTrack(x, &at);
        double early = 0, late = 0, mean = 0;
        int ne = 0, nl = 0;
        for (size_t i = 0; i < f.size(); ++i)
            if (at[i] > 1.5) mean += f[i], ++nl;
        mean /= nl;
        int swings = 0;
        bool above = false;
        for (size_t i = 0; i < f.size(); ++i) {
            const double d = f[i] - hz(57.0f);
            if (at[i] > 0.05 && at[i] < 0.3) early = std::max(early, std::fabs(d)), ++ne;
            if (at[i] > 1.5) {
                late = std::max(late, std::fabs(d));
                if ((f[i] > mean) != above) {
                    above = !above;
                    swings += above;
                }
            }
        }
        const double measured = swings / (3.0 - 1.5);
        std::printf("  LFO %.0f Hz: %.2f Hz measured, deviation %.2f Hz in the delay, %.2f Hz after\n", rate, measured,
                    early, late);
        CHECK_NEAR(measured, double(rate), 0.15 * rate + 0.4);
        CHECK(late > 3.0);             // ~ +-2 semitones at 220 Hz is +-27 Hz
        CHECK(early < 0.35 * late);    // the delay holds the LFO back
        CHECK(ne > 0);
    }
    // No delay: the vibrato is there from the start.
    c[ju6::kLfoRate] = 6.0f;
    c[ju6::kLfoDelay] = 0.0f;
    const Buf x = play(c, 57.0f, 0.5f);
    std::vector<double> at;
    const std::vector<double> f = freqTrack(x, &at);
    double early = 0;
    for (size_t i = 0; i < f.size(); ++i)
        if (at[i] > 0.05 && at[i] < 0.3) early = std::max(early, std::fabs(f[i] - hz(57.0f)));
    CHECK(early > 3.0);
}

TEST(ju6_no_nan_or_denormals) {
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
        if (!finite(r.L) || peak(r.L) > 2.0f) ++bad;
        sub += subnormals(r.L);
    }
    CHECK_EQ(bad, 0);
    CHECK_EQ(sub, 0L);
    // Extremes: every parameter at its max (and min), full velocity, very high and low notes.
    for (int ext = 0; ext < 2; ++ext) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[i] = ext ? range(i)->max : range(i)->min;
        a[ju6::kSaw] = 1.0f;
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
    c[ju6::kRelease] = 10.0f;
    Rig t(c);
    t.on(0, 40.0f);
    t.run(50);
    t.off(0);
    int blocks = 0;
    while (t.run(1) && blocks < 60 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 60 * kBlocksPerSec);
    CHECK_EQ(subnormals(t.L), 0L);
    // A held note decaying to Sustain 0 (Env VCA) ends; nothing goes subnormal.
    std::vector<float> d = open();
    d[ju6::kDecay] = 0.05f;
    d[ju6::kSustain] = 0.0f;
    d[ju6::kRelease] = 10.0f;
    Rig h(d);
    h.on(0, 48.0f);
    int held = 0;
    while (h.run(1) && held < 20 * kBlocksPerSec) ++held;
    CHECK(held < kBlocksPerSec);
    CHECK_EQ(subnormals(h.L), 0L);
}

// Fresh voices report full level at the note on (no steal before their
// first block, as FM4); afterwards level follows the VCA.
TEST(ju6_voice_level) {
    std::vector<float> c = open();
    c[ju6::kAttack] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    const float lv = r.v[0].level;
    r.on(0, 62.0f, 1.0f, true, false);   // a reused voice keeps its level
    CHECK_EQ(r.v[0].level, lv);
}

// A Resonance jump between two blocks (a macro slammed) moves the output
// smoothly: the make-up gain and the loop gain glide across the block.
TEST(ju6_resonance_change_does_not_click) {
    std::vector<float> c = open();
    c[ju6::kCutoff] = 300.0f;
    c[ju6::kResonance] = 0.15f;
    Rig r(c);
    r.on(0, 45.0f);
    r.run(kBlocksPerSec / 2);
    auto maxStep = [&r](size_t a, size_t b) {
        double m = 0;
        for (size_t i = a + 1; i < b; ++i) m = std::max(m, double(std::fabs(r.L[i] - r.L[i - 1])));
        return m;
    };
    const size_t at = r.L.size();
    const double steady = maxStep(at - 20 * kBlock, at);
    for (float res : {0.6f, 0.15f, 0.9f}) {
        c[ju6::kResonance] = res;
        r.set(c);
        const size_t b = r.L.size();
        r.run(2);
        const double jump = maxStep(b - 1, b + 2 * kBlock);
        if (!(jump < 1.6 * steady)) std::printf("       Res -> %.2f: step %.4f (steady %.4f)\n", res, jump, steady);
        CHECK(jump < 1.6 * steady);
        r.run(kBlocksPerSec / 4);
    }
}

TEST(ju6_volume_is_never_applied) {
    std::vector<float> a = initValues(), b = initValues();
    a[ju6::kVolume] = 0.0f;
    b[ju6::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

// The Init patch through a real SynthBank slot at the default slot gain
// (-9 dB): one voice at ACD's Init level (within 1.5 dB, the FAT / WTFM rule
// of tests/synth_levels_test.cpp) and well below full scale, a 4-voice chord
// at least 1 dB below it, nothing clipped.
namespace {
struct BankSlot {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
    float peak = 0.0f;
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

TEST(ju6_level_matches_the_synth_conventions) {
    long c1 = 0, c4 = 0;
    const float p1 = chordPeak(ju6::kEngineId, 1, c1);
    const float p4 = chordPeak(ju6::kEngineId, 4, c4);
#if defined(OT_MOD_ACD1)
    long ca = 0;
    const float pa = chordPeak(kSynthEngineAcd, 1, ca);
    std::printf("  JU6 Init at -9 dB: 1 voice peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(pa),
                dbfs(p4));
    CHECK(std::fabs(dbfs(p1) - dbfs(pa)) < 1.5);
#else
    std::printf("  JU6 Init at -9 dB: 1 voice peak %.1f dBFS, 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(p4));
#endif
    CHECK(dbfs(p1) < -6.0);
    CHECK(dbfs(p1) > -20.0);
    CHECK(dbfs(p4) < -1.0);
    CHECK_EQ(c1, 0L);
    CHECK_EQ(c4, 0L);
}

TEST(ju6_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == ju6::kVolume) continue;
        const PatchParamRange* g = range(i);
        std::vector<float> a = ctx, b = ctx;
        a[i] = g->min;
        b[i] = g->max;
        const double d = rmsDiff(renderSequence(a), renderSequence(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

// Chorus Off is the JU6 of before the chorus, bit for bit: the hashes of a
// rich note sequence (engine alone) and of Init chords through a SynthBank
// slot, recorded with the engine before the chorus was added (host build,
// -ffp-contract=off).
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
    Rig r(richContext());
    r.run(3);
    r.on(0, 57.0f, 0.6f);
    r.on(1, 64.0f, 0.9f);
    r.run(kBlocksPerSec / 3);
    r.on(0, 60.0f, 0.6f, true, false);
    r.off(1);
    r.run(kBlocksPerSec / 3);
    r.off(0);
    r.run(kBlocksPerSec / 2);
    return hashOf(r.L, r.R);
}
uint64_t bankHash() {
    BankSlot s(ju6::kEngineId, 4);
    Buf L, R;
    for (int trial = 0; trial < 3; ++trial) {
        for (int k = 0; k < 4; ++k) s.on(k, kChords[trial][k]);
        for (int b = 0; b < 60; ++b) {
            s.block();
            L.insert(L.end(), s.L, s.L + kBlock);
            R.insert(R.end(), s.R, s.R + kBlock);
        }
        for (int k = 0; k < 4; ++k) s.off(k);
        for (int b = 0; b < 120; ++b) {
            s.block();
            L.insert(L.end(), s.L, s.L + kBlock);
            R.insert(R.end(), s.R, s.R + kBlock);
        }
    }
    return hashOf(L, R);
}
}  // namespace

TEST(ju6_chorus_off_is_the_engine_before_the_chorus) {
    const uint64_t e = engineHash(), b = bankHash();
    std::printf("  JU6 golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x58409e2fbba7ce9bull));
    CHECK_EQ(b, uint64_t(0xc791d9de914e9707ull));
}

// ---------------------------------------------------------------------------
// The built-in chorus (patch parameter Chorus: Off / I / II / I+II, the shared BBD
// chorus of core audio/bbd_chorus.h on the slot's voice sum).

namespace {
std::vector<float> chorusContext(int mode) {
    std::vector<float> c = open();
    c[ju6::kChorus] = float(mode);
    return c;
}
// The reference per mode (index = mode - 1): LFO rate and the tap delay range.
struct ChorusRef {
    double hz, minMs, maxMs;
};
constexpr int kChorusModes = 3;   // I, II, I+II
// I+II runs I and II at once: chorusDelay() reports I's pair, so its reference is I's.
constexpr ChorusRef kChorusRef[kChorusModes] = {{0.5, 2.0, 5.0}, {0.8, 1.5, 5.5}, {0.5, 2.0, 5.0}};
double corrLR(const Buf& a, const Buf& b, size_t from) {
    double ab = 0, aa = 0, bb = 0;
    for (size_t i = from; i < a.size() && i < b.size(); ++i) {
        ab += double(a[i]) * b[i];
        aa += double(a[i]) * a[i];
        bb += double(b[i]) * b[i];
    }
    return aa > 0 && bb > 0 ? ab / std::sqrt(aa * bb) : 1.0;
}
}  // namespace

TEST(ju6_chorus_parameter) {
    mod::ModuleHost::instance().init();
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(ju6::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK(int(ju6::kChorus) > int(ju6::kVolume));   // appended: the indices of older patches hold
    const PatchParamRange& r = d->params[ju6::kChorus];
    CHECK_EQ(r.def, 0.0f);   // Off (a patch without the field)
    CHECK_EQ(r.min, 0.0f);
    CHECK_EQ(r.max, 3.0f);
    CHECK_STR(d->ui[ju6::kChorus].label, "Chorus");
    CHECK_EQ(int(d->ui[ju6::kChorus].fmt), int(synthui::kFmtEnum));
    CHECK_EQ(int(d->ui[ju6::kChorus].nameCount), 4);
    if (d->ui[ju6::kChorus].nameCount == 4) {
        CHECK_STR(d->ui[ju6::kChorus].names[0], "Off");
        CHECK_STR(d->ui[ju6::kChorus].names[1], "I");
        CHECK_STR(d->ui[ju6::kChorus].names[2], "II");
        CHECK_STR(d->ui[ju6::kChorus].names[3], "I+II");
    }
}

TEST(ju6_patch_without_chorus_loads_off) {
    mod::ModuleHost::instance().init();
    // A patch as saved before the chorus: the 22 parameters up to Volume.
    SynthPatch old;
    CHECK(patchInit(ju6::kEngineId, &old));
    CHECK(synthParamSet(old, ju6::kCutoff, 900.0f));
    old.paramCount = uint16_t(ju6::kVolume + 1);
    std::vector<uint8_t> buf(kPatchMaxFileSize);
    size_t n = 0;
    CHECK(encodePatch(old, buf.data(), buf.size(), &n) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(buf.data(), n, &q) == PatchStatus::kOk);
    CHECK_EQ(int(q.paramCount), int(ju6::kVolume) + 1);
    CHECK_EQ(synthParamGet(q, ju6::kChorus), 0.0f);
    CHECK_EQ(synthParamGet(q, ju6::kCutoff), 900.0f);
    // Through a slot: the effective value is Off, and it plays as before.
    float out[kPatchMaxParams] = {};
    const uint8_t macros[kSynthSeqMacros] = {};
    CHECK_EQ(synthEffectiveParams(q, macros, 1.0f, out), kN);
    CHECK_EQ(out[ju6::kChorus], 0.0f);
    // Setting the chorus grows the patch to the engine's count; it round-trips.
    CHECK(synthParamSet(q, ju6::kChorus, 2.0f));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK(encodePatch(q, buf.data(), buf.size(), &n) == PatchStatus::kOk);
    SynthPatch q2;
    CHECK(decodePatch(buf.data(), n, &q2) == PatchStatus::kOk);
    CHECK_EQ(synthParamGet(q2, ju6::kChorus), 2.0f);
    CHECK_EQ(validatePatch(q2, nullptr, 0), 0);
}

TEST(ju6_chorus_off_is_mono_and_untouched) {
    Rig a(chorusContext(0));
    a.on(0, 48.0f);
    a.run(kBlocksPerSec / 2);
    CHECK(a.L == a.R);
    CHECK(!a.tail);
}

TEST(ju6_chorus_modes_rate_and_delay) {
    for (int mode = 1; mode <= kChorusModes; ++mode) {
        const ChorusRef& ref = kChorusRef[mode - 1];
        Rig g(chorusContext(mode));
        g.on(0, 48.0f);
        std::vector<double> dl, dr;
        const int blocks = 6 * kBlocksPerSec;
        for (int b = 0; b < blocks; ++b) {
            g.run(1);
            dl.push_back(ju6::chorusDelay(g.scratch, 0));
            dr.push_back(ju6::chorusDelay(g.scratch, 1));
        }
        double lo = 1e9, hi = -1e9;
        for (size_t i = 0; i < dl.size(); ++i) {
            lo = std::min(lo, std::min(dl[i], dr[i]));
            hi = std::max(hi, std::max(dl[i], dr[i]));
            // Opposite phases: the two taps sum to twice the centre.
            CHECK(std::fabs(dl[i] + dr[i] - 2.0 * 3.5 * 0.001 * kFs) < 1e-2);
        }
        const double ms = 1000.0 / kFs;
        std::printf("  JU6 chorus %d: taps %.2f .. %.2f ms\n", mode, lo * ms, hi * ms);
        CHECK(std::fabs(lo * ms - ref.minMs) < 0.05);
        CHECK(std::fabs(hi * ms - ref.maxMs) < 0.05);
        // Rate: upward crossings of the left tap through the centre.
        const double c = 3.5 * 0.001 * kFs;
        std::vector<double> ups;
        for (size_t i = 1; i < dl.size(); ++i)
            if (dl[i - 1] < c && dl[i] >= c) ups.push_back(double(i - 1) + (c - dl[i - 1]) / (dl[i] - dl[i - 1]));
        CHECK(ups.size() >= 2);
        if (ups.size() >= 2) {
            const double period = (ups.back() - ups.front()) / double(ups.size() - 1) * double(kBlock) / kFs;
            std::printf("  JU6 chorus %d: LFO %.3f Hz\n", mode, 1.0 / period);
            CHECK(std::fabs(1.0 / period - ref.hz) < 0.01);
        }
    }
}

TEST(ju6_chorus_decorrelates_the_sides) {
    for (int mode = 1; mode <= kChorusModes; ++mode) {
        Rig g(chorusContext(mode));
        g.on(0, 48.0f);
        g.on(1, 55.0f);
        g.run(2 * kBlocksPerSec);
        const double c = corrLR(g.L, g.R, size_t(kFs / 4));
        std::printf("  JU6 chorus %d: L/R correlation %.3f\n", mode, c);
        CHECK(c < 0.95);
        CHECK(c > 0.0);   // still one instrument, not two
        CHECK(rmsDiff(g.L, g.R) > 0.01 * rms(g.L));
    }
}

// I+II adds mode II's taps to I's: not the same sound as either, about the same level.
TEST(ju6_chorus_both_is_i_and_ii_together) {
    Buf out[kChorusModes];
    for (int mode = 1; mode <= kChorusModes; ++mode) {
        Rig g(chorusContext(mode));
        g.on(0, 48.0f);
        g.on(1, 55.0f);
        g.run(2 * kBlocksPerSec);
        out[mode - 1] = g.L;
    }
    CHECK(rmsDiff(out[2], out[0]) > 0.01 * rms(out[0]));
    CHECK(rmsDiff(out[2], out[1]) > 0.01 * rms(out[1]));
    const double db = 20.0 * std::log10(rms(out[2]) / rms(out[0]));
    std::printf("  JU6 chorus I+II vs I: level %+.2f dB\n", db);
    CHECK(std::fabs(db) < 1.5);
}

TEST(ju6_chorus_silence_in_silence_out) {
    for (int mode = 1; mode <= kChorusModes; ++mode) {
        Rig g(chorusContext(mode));
        g.run(50);   // no note
        CHECK_EQ(peak(g.L), 0.0f);
        CHECK_EQ(peak(g.R), 0.0f);
        CHECK(!g.tail);
        // After a note the line empties to exact zeros and the tail ends.
        g.on(0, 60.0f);
        g.run(kBlocksPerSec / 4);
        g.off(0);
        int blocks = 0;
        while ((g.run(1) || g.tail) && blocks < 10 * kBlocksPerSec) ++blocks;
        CHECK(blocks < 10 * kBlocksPerSec);
        CHECK(!g.tail);
        g.L.clear();
        g.R.clear();
        g.run(50);
        CHECK_EQ(peak(g.L), 0.0f);
        CHECK_EQ(peak(g.R), 0.0f);
    }
}

TEST(ju6_chorus_each_slot_has_its_own_line) {
    // Two slots rendered block by block in turn give exactly what each gives alone.
    auto sequence = [](Rig& g, int b) {
        if (b == 0) g.on(0, g.slot ? 64.0f : 45.0f);
        if (b == 3 * kBlocksPerSec / 4) g.off(0);
    };
    const int blocks = kBlocksPerSec;
    Rig a(chorusContext(2), 0), b(chorusContext(1), 1);
    for (int k = 0; k < blocks; ++k) {
        sequence(a, k);
        a.run(1);
        sequence(b, k);
        b.run(1);
    }
    Rig a1(chorusContext(2), 0), b1(chorusContext(1), 1);
    for (int k = 0; k < blocks; ++k) {
        sequence(a1, k);
        a1.run(1);
    }
    for (int k = 0; k < blocks; ++k) {
        sequence(b1, k);
        b1.run(1);
    }
    CHECK(a.L == a1.L && a.R == a1.R);
    CHECK(b.L == b1.L && b.R == b1.R);
    CHECK(rmsDiff(a.L, a.R) > 0.0);
}

TEST(ju6_chorus_no_nan_or_denormals) {
    for (int mode = 1; mode <= kChorusModes; ++mode) {
        std::vector<float> c = richContext();
        c[ju6::kChorus] = float(mode);
        c[ju6::kResonance] = 1.0f;   // self-oscillation into the line's soft clip
        c[ju6::kVolume] = 2.0f;
        Rig g(c);
        for (int k = 0; k < 6; ++k) g.on(k, 36.0f + 7.0f * float(k));
        g.run(kBlocksPerSec);
        for (int k = 0; k < 6; ++k) g.off(k);
        g.run(20 * kBlocksPerSec);   // the long decay into silence
        CHECK(finite(g.L) && finite(g.R));
        CHECK_EQ(subnormals(g.L), 0L);
        CHECK_EQ(subnormals(g.R), 0L);
    }
}

// Through a SynthBank slot: a Gate VCA note ends at once, the chorus tail
// plays on after the voice is freed (no cut), then the slot goes silent and
// the bank idle.
TEST(ju6_chorus_tail_after_the_last_voice) {
    BankSlot s(ju6::kEngineId, 4);
    synthParamSet(s.patch, ju6::kChorus, 2.0f);
    synthParamSet(s.patch, ju6::kVcaMode, float(ju6::kVcaGate));
    synthParamSet(s.patch, ju6::kSustain, 1.0f);
    s.bank.bindPatch(0, &s.patch);
    s.block();
    s.on(0, 60);
    s.run(100);
    s.off(0);
    int free = -1;
    float tailPeak = 0.0f;
    for (int b = 0; b < 40; ++b) {
        s.block();
        float pk = 0.0f;
        for (uint32_t i = 0; i < kBlock; ++i) pk = std::max(pk, std::fabs(s.L[i]));
        if (free < 0 && s.bank.voicesOf(0) == 0) free = b;
        else if (free >= 0) tailPeak = std::max(tailPeak, pk);
    }
    CHECK(free >= 0 && free < 3);
    CHECK(tailPeak > 0.0f);   // the wet tail after the voice
    CHECK(!s.bank.active());
    for (int b = 0; b < 5; ++b) {
        s.block();
        for (uint32_t i = 0; i < kBlock; ++i) CHECK(s.L[i] == 0.0f && s.R[i] == 0.0f);
    }
}

// --- Performance filter offset (issue #23) ---------------------------------------------
// SynthVoice::cutoffOct (octaves, filled by the framework from the track's
// Performance filter) moves the JU6 cutoff: Cutoff x 2^offset, inside the
// engine's own clamps. A patch at 1600 Hz shifted by -2 octaves sounds like
// the same patch at 400 Hz; 0 leaves the render bit-identical.

namespace {
namespace perfcut {

// One note (pitch 48, 40 blocks) from the Init patch with its cutoff set by
// `cut(patch, hz)`, the voice's cutoffOct = `oct`.
template <class Cut>
std::vector<float> render(Cut cut, float hz, float oct) {
    const int32_t engine = ju6::kEngineId;
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
    CHECK(synthParamSet(p, ju6::kCutoff, hz));
}

}  // namespace perfcut
}  // namespace

TEST(ju6_perf_cutoff_offset_moves_the_cutoff) {
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
