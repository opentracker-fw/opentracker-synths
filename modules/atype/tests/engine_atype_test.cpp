// A-Type duophonic engine (opentracker-synths modules/atype, engine 0x84):
// registered with a valid descriptor ("A-Type" on screen, Patches/ATYPE/, the
// framework's note priority), silent when idle, the VCOs at A 440 with their
// ranges, tune, fine and a small drift, the pulse widths and PWM, hard sync
// (the master's pitch, band-limited resets), the ring modulator's sum and
// difference tones, noise, the sample & hold (rate, key clock, lag, inputs),
// the LFO shapes and vibrato, the high-pass, the 4-pole and 2-pole low-pass
// (cutoff, slopes, resonance, keyboard-tracked self-oscillation), the
// envelope amount and sources, ADSR and AR on the VCA, glide, and through a
// SynthBank slot: Last / Low / High priority, return-to-held, Duo from two
// tracks (VCO 1 low, VCO 2 high) and the release of either, Mono, Duo ignored
// at Poly 2; no click on parameter jumps, no NaN or denormals, the voice
// level for stealing, Volume never applied, every parameter audible, the slot
// level of the other engines, golden hashes.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "atype.h"
#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"

using namespace ot;
using namespace ot::audio;

namespace {

constexpr float kFs = ot::audio::kSampleRate;   // (model/project.h has a kSampleRate too)
constexpr int kN = atype::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(atype::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

// One engine voice pool driven directly (no framework). held: what the
// framework would report for the slot (Duo).
struct Rig {
    ISynthEngine* e = synthEngineFor(atype::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    std::vector<float> L, R;

    explicit Rig(const std::vector<float>& vals) {
        p.engine = atype::kEngineId;
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
    void held(int count, int low = 0, int high = 0) {
        p.held.count = uint8_t(count);
        p.held.low = uint8_t(low);
        p.held.high = uint8_t(high);
        p.held.mono = 1;
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
// harmonic power below maxHz (the mean removed: a synced saw has an offset).
double aliasRatio(const Buf& x, double f0, double maxHz) {
    double mean = 0;
    for (float v : x) mean += v;
    mean /= double(x.size());
    Buf y(x);
    for (float& v : y) v = float(v - mean);
    const std::vector<double> s = spectrum(y);
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

// A plain context: VCO 1 alone (saw, 8'), the filter wide open, no
// resonance, envelope or key track, the VCA on the ADSR held at full
// sustain, no modulation, no glide.
std::vector<float> open() {
    std::vector<float> c = initValues();
    c[atype::kVco1Level] = 1.0f;
    c[atype::kVco2Level] = 0.0f;
    c[atype::kSourceLevel] = 0.0f;
    c[atype::kCutoff] = 20000.0f;
    c[atype::kResonance] = 0.0f;
    c[atype::kKeyTrack] = 0.0f;
    c[atype::kEnvAmt] = 0.0f;
    c[atype::kAttack] = 0.0f;
    c[atype::kDecay] = 0.05f;
    c[atype::kSustain] = 1.0f;
    c[atype::kGlideTime] = 0.0f;
    return c;
}
// One VCO alone at a low level (the filter input stays near linear).
std::vector<float> only(int vco, int wave, float level = 0.25f) {
    std::vector<float> c = open();
    c[atype::kVco1Level] = vco == 0 ? level : 0.0f;
    c[atype::kVco2Level] = vco == 1 ? level : 0.0f;
    c[vco == 0 ? atype::kVco1Wave : atype::kVco2Wave] = float(wave);
    c[atype::kVco2Fine] = 0.0f;
    return c;
}
// The filter alone, self-oscillating: no VCO, no noise (the seed starts it).
std::vector<float> whistle(float cutoff = 440.0f, float keyTrack = 1.0f, int mode = atype::k4Pole) {
    std::vector<float> c = open();
    c[atype::kVco1Level] = 0.0f;
    c[atype::kCutoff] = cutoff;
    c[atype::kResonance] = 1.0f;
    c[atype::kKeyTrack] = keyTrack;
    c[atype::kFilterMode] = float(mode);
    return c;
}
Buf play(const std::vector<float>& vals, float pitch = 48.0f, float seconds = 0.5f) {
    Rig r(vals);
    r.on(0, pitch);
    r.run(int(seconds * kFs / float(kBlock)) + 1);
    return r.L;
}
const size_t kA = size_t(0.15 * 44100), kB = size_t(0.45 * 44100);

// Every section of the voice in use.
std::vector<float> richContext() {
    std::vector<float> c = initValues();
    c[atype::kVco1Range] = float(atype::k16);
    c[atype::kVco1Fine] = -8.0f;
    c[atype::kVco1Wave] = float(atype::kPulse);
    c[atype::kVco1Pw] = 0.3f;
    c[atype::kVco1Pwm] = 0.3f;
    c[atype::kVco1LfoMod] = 0.2f;
    c[atype::kVco2Tune] = 7.0f;
    c[atype::kVco2Wave] = float(atype::kSaw);
    c[atype::kVco2Pw] = 0.4f;
    c[atype::kVco2Pwm] = 0.2f;
    c[atype::kVco2Mod] = 0.3f;
    c[atype::kVco2ModSrc] = float(atype::kModSh);
    c[atype::kSync] = 0.0f;
    c[atype::kSource] = float(atype::kNoise);
    c[atype::kSourceLevel] = 0.2f;
    c[atype::kShInput] = float(atype::kShNoise);
    c[atype::kShRate] = 12.0f;
    c[atype::kShLag] = 0.02f;
    c[atype::kCutoff] = 900.0f;
    c[atype::kResonance] = 0.5f;
    c[atype::kKeyTrack] = 0.5f;
    c[atype::kEnvAmt] = 0.5f;
    c[atype::kLfoAmt] = 0.2f;
    c[atype::kShAmt] = 0.2f;
    c[atype::kHpf] = 80.0f;
    c[atype::kAttack] = 0.01f;
    c[atype::kDecay] = 0.08f;
    c[atype::kSustain] = 0.5f;
    c[atype::kRelease] = 0.1f;
    c[atype::kArAttack] = 0.03f;
    c[atype::kArRelease] = 0.15f;
    c[atype::kLfoRate] = 6.0f;
    c[atype::kLfoShape] = float(atype::kTri);
    c[atype::kGlideTime] = 0.1f;
    return c;
}
// Two notes, the second reused without overlap, then the release; Duo
// sees two held notes during the second.
Buf renderSequence(const std::vector<float>& vals) {
    Rig r(vals);
    r.run(10);
    r.held(1, 57, 57);
    r.on(0, 57.0f, 0.6f);
    r.run(kBlocksPerSec / 4);
    r.held(2, 52, 64);
    r.on(0, 64.0f, 0.6f, true, false);
    r.run(kBlocksPerSec / 4);
    r.held(0);
    r.off(0);
    r.run(kBlocksPerSec / 4);
    return r.L;
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
                pk = std::max(pk, a);
                if (a >= 1.0f) ++clips;
            }
        }
    }
    // The last `seconds` of the output.
    Buf tail(double seconds) const { return Buf(out.end() - long(seconds * kFs), out.end()); }
};

}  // namespace

TEST(atype_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(atype::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "ATYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(atype::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(atype::kEngineId));
    CHECK_STR(d->name, "ATYPE");
    CHECK_STR(d->displayName, "A-Type");
    CHECK_EQ(int(d->paramCount), 46);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(atype::kVolume));
    CHECK(d->flags & mod::kSynthFlagNotePriority);
    CHECK_EQ(int(d->priorityParam), int(atype::kPriority));
    CHECK_EQ(patchVolumeParam(atype::kEngineId), int(atype::kVolume));
    CHECK_STR(patchEngineName(atype::kEngineId), "ATYPE");
    CHECK_EQ(int(d->sectionCount), 15);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    CHECK_STR(synthEngineFolder(atype::kEngineId), "ATYPE");
    CHECK_STR(synthEngineDisplayName(atype::kEngineId), "A-Type");
    // In the model list after the Mini models and the module engines before it.
    const int k = synthModelListIndex(atype::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "A-Type");
#if defined(OT_MOD_DTY1)
    CHECK(k > synthModelListIndex(0x82));
#endif
#if defined(OT_MOD_CST1)
    CHECK(k > synthModelListIndex(0x83));
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
    CHECK(patchInit(atype::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(atype::kVolume));
}

TEST(atype_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(atype::kEngineId, &p));
    const float vals[][2] = {{atype::kVco1Range, 0}, {atype::kVco2Tune, -12}, {atype::kVco2Fine, 33.5f},
                             {atype::kSync, 1},      {atype::kCutoff, 432.5f}, {atype::kShRate, 17.25f},
                             {atype::kKeyboard, 1},  {atype::kPriority, 2},   {atype::kGlideTime, 0.25f}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(atype::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(atype::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    CHECK(!patchParamInRange(atype::kEngineId, atype::kVco1Range, 4.0f));
    CHECK(!patchParamInRange(atype::kEngineId, atype::kVco2Tune, 25.0f));
    CHECK(!patchParamInRange(atype::kEngineId, atype::kPriority, 3.0f));
}

TEST(atype_silent_when_idle) {
    Rig r(initValues());
    r.run(20);   // no note
    CHECK_EQ(peak(r.L), 0.0f);
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

TEST(atype_vco_pitch_ranges_tune_and_drift) {
    for (float pitch : {69.0f, 57.0f, 45.0f}) {
        const Buf x = play(only(0, atype::kSaw), pitch);
        CHECK(finite(x));
        const double f = pitchHz(x, kA, kB);
        CHECK_NEAR(f, hz(pitch), hz(pitch) * 0.005);
        if (pitch == 69.0f) std::printf("  A4: %.2f Hz\n", f);
    }
    // Ranges 32' 16' 8' 4' = -2 .. +1 octaves, on both VCOs.
    const float oct[4] = {-2, -1, 0, 1};
    for (int vco = 0; vco < 2; ++vco)
        for (int rg = 0; rg < 4; ++rg) {
            std::vector<float> c = only(vco, atype::kSaw);
            c[vco == 0 ? atype::kVco1Range : atype::kVco2Range] = float(rg);
            const float note = rg <= atype::k16 ? 69.0f : 57.0f;
            const double want = hz(note) * std::pow(2.0, oct[rg]);
            CHECK_NEAR(pitchHz(play(c, note), kA, kB), want, want * 0.005);
        }
    // VCO 2 Tune and both Fines.
    std::vector<float> c = only(1, atype::kSaw);
    c[atype::kVco2Tune] = 7.0f;
    CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(64.0f), hz(64.0f) * 0.003);
    c[atype::kVco2Tune] = -12.0f;
    CHECK_NEAR(pitchHz(play(c, 69.0f), kA, kB), 220.0, 220.0 * 0.003);
    c[atype::kVco2Tune] = 0.0f;
    c[atype::kVco2Fine] = 50.0f;
    CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(57.5f), hz(57.5f) * 0.002);
    std::vector<float> d = only(0, atype::kSaw);
    d[atype::kVco1Fine] = -100.0f;
    CHECK_NEAR(pitchHz(play(d, 57.0f), kA, kB), hz(56.0f), hz(56.0f) * 0.002);
    // Drift: within +-3 cents over 4 s, each VCO.
    for (int vco = 0; vco < 2; ++vco) {
        const Buf x = play(only(vco, atype::kSaw), 69.0f, 4.2f);
        double lo = 1e9, hi = -1e9;
        for (double t = 0.2; t + 0.25 < 4.2; t += 0.25) {
            const double ct = cents(pitchHz(x, size_t(t * kFs), size_t((t + 0.25) * kFs)), 440.0);
            lo = std::min(lo, ct);
            hi = std::max(hi, ct);
        }
        std::printf("  VCO%d drift %.2f .. %.2f cents\n", vco + 1, lo, hi);
        CHECK(lo > -3.0 && hi < 3.0);
        CHECK(hi - lo > 0.05);   // it does drift
    }
}

namespace {
double highFraction(const Buf& x) {
    double mean = 0;
    for (float s : x) mean += s;
    mean /= double(x.size());
    size_t n = 0;
    for (float s : x) n += s > mean;
    return double(n) / double(x.size());
}
}  // namespace

TEST(atype_pulse_widths_and_pwm) {
    // PW 0 = 50 %, 0.5 = 27.5 %, 1 = 5 %, on both VCOs.
    const float pw[3] = {0.0f, 0.5f, 1.0f};
    const double duty[3] = {0.5, 0.275, 0.05};
    for (int vco = 0; vco < 2; ++vco)
        for (int k = 0; k < 3; ++k) {
            std::vector<float> c = only(vco, atype::kPulse);
            c[vco == 0 ? atype::kVco1Pw : atype::kVco2Pw] = pw[k];
            const double hf = highFraction(slice(play(c, 45.0f), 0.1, 0.45));
            if (std::fabs(hf - duty[k]) >= 0.03) std::printf("       VCO%d PW %.1f: high %.3f\n", vco + 1, pw[k], hf);
            CHECK_NEAR(hf, duty[k], 0.03);
        }
    // The saw ignores the width.
    std::vector<float> s = only(0, atype::kSaw), t = s;
    t[atype::kVco1Pw] = 1.0f;
    CHECK(play(s, 45.0f) == play(t, 45.0f));
    // PWM: the LFO sweeps the width (here 1 Hz); without PWM it stays.
    for (int vco = 0; vco < 2; ++vco) {
        std::vector<float> c = only(vco, atype::kPulse);
        c[atype::kLfoRate] = 1.0f;
        auto sweep = [&c]() {
            const Buf x = play(c, 45.0f, 2.0f);
            double lo = 1, hi = 0;
            for (double t0 = 0.1; t0 + 0.05 < 2.0; t0 += 0.05) {
                const double h = highFraction(slice(x, t0, t0 + 0.05));
                lo = std::min(lo, h);
                hi = std::max(hi, h);
            }
            return hi - lo;
        };
        const double still = sweep();
        c[vco == 0 ? atype::kVco1Pwm : atype::kVco2Pwm] = 1.0f;
        const double moving = sweep();
        std::printf("  VCO%d PWM: width swing %.3f (none %.3f)\n", vco + 1, moving, still);
        CHECK(still < 0.03);
        CHECK(moving > 0.3);
    }
}

TEST(atype_hard_sync) {
    // VCO 2 alone, synced to VCO 1 (silent in the mix): the output repeats at
    // VCO 1's pitch whatever VCO 2's tune; the tune moves the timbre.
    std::vector<float> c = only(1, atype::kSaw);
    c[atype::kSync] = 1.0f;
    c[atype::kVco2Tune] = 5.0f;
    const Buf a = play(c, 45.0f);
    CHECK_NEAR(pitchHz(a, kA, kB), 110.0, 110.0 * 0.01);
    c[atype::kVco2Tune] = 17.0f;
    const Buf b = play(c, 45.0f);
    CHECK_NEAR(pitchHz(b, kA, kB), 110.0, 110.0 * 0.01);
    CHECK(rmsDiff(slice(a, 0.15, 0.45), slice(b, 0.15, 0.45)) > 0.2 * rms(slice(a, 0.15, 0.45)));
    // Off: VCO 2's own pitch.
    c[atype::kSync] = 0.0f;
    CHECK_NEAR(pitchHz(play(c, 45.0f), kA, kB), hz(62.0f), hz(62.0f) * 0.005);
    // Band-limited resets: a synced saw at note 84 off a master at note 72
    // aliases far less (below 8 kHz) than the naive reset.
    c = only(1, atype::kSaw);
    c[atype::kSync] = 1.0f;
    c[atype::kVco2Tune] = 7.3f;
    c[atype::kVco2Fine] = 0.0f;
    const Buf x = play(c, 72.0f, 0.4f);
    const Buf seg = slice(x, 0.2, 0.2 + 4096.0 / kFs);
    Buf naive(seg.size());
    double pm = 0.0, ps = 0.0;
    const double dm = hz(72.0f) / kFs, ds = hz(79.3f) / kFs;
    for (float& s : naive) {
        s = float(2.0 * ps - 1.0);
        pm += dm;
        ps += ds;
        if (ps >= 1.0) ps -= 1.0;
        if (pm >= 1.0) {
            pm -= 1.0;
            ps = pm / dm * ds;
        }
    }
    const double rn = aliasRatio(naive, hz(72.0f), 8000.0), rj = aliasRatio(seg, hz(72.0f), 8000.0);
    std::printf("  synced saw alias/harmonic power below 8 kHz: naive %.1f dB, A-Type %.1f dB\n", 10 * std::log10(rn),
                10 * std::log10(rj));
    CHECK(10 * std::log10(rj) < 10 * std::log10(rn) - 10.0);
}

TEST(atype_ring_modulator_and_noise) {
    // Ring: VCO 1 x VCO 2 as the third source (the VCOs silent in the mix):
    // the sum and difference tones, not the inputs.
    std::vector<float> c = only(0, atype::kSaw, 0.0f);
    c[atype::kVco2Tune] = 4.0f;
    c[atype::kVco2Fine] = 0.0f;
    c[atype::kSource] = float(atype::kRing);
    c[atype::kSourceLevel] = 0.3f;
    const Buf x = slice(play(c, 57.0f), 0.1, 0.45);
    const double f1 = hz(57.0f), f2 = hz(61.0f);
    std::printf("  ring: sum %.4f, difference %.4f, inputs %.4f / %.4f\n", tone(x, f1 + f2), tone(x, f2 - f1), tone(x, f1), tone(x, f2));
    CHECK(rms(x) > 0.01);
    CHECK(tone(x, f1 + f2) > 5.0 * tone(x, f1));
    CHECK(tone(x, f1 + f2) > 5.0 * tone(x, f2));
    CHECK(tone(x, f2 - f1) > 5.0 * tone(x, f1));
    // Noise: broadband, about flat (white), audible.
    c[atype::kSource] = float(atype::kNoise);
    const Buf n = slice(play(c, 57.0f, 1.0f), 0.1, 1.0);
    CHECK(rms(n) > 0.02);
    auto band = [](const Buf& b, double lo, double hi) {
        double s = 0;
        int k = 0;
        for (double f = lo; f < hi; f += (hi - lo) / 16.0, ++k) s += tone(b, f) * tone(b, f);
        return s / k;
    };
    const double rw = band(n, 100, 400) / band(n, 3200, 6400);
    CHECK(rw > 0.3 && rw < 3.0);
    // Source Level 0: neither.
    c[atype::kSourceLevel] = 0.0f;
    CHECK(rms(slice(play(c, 57.0f), 0.1, 0.45)) < 1e-3);
}

namespace {
// VCO 2 alone, its pitch moved by the S&H (+-3 semitones at Mod 50 %): the
// pitch steps over time, in semitones off the note (per 10 ms window).
std::vector<double> shSteps(std::vector<float> c, float seconds, std::vector<int> notesAt = {}) {
    c[atype::kVco2Mod] = 0.5f;
    c[atype::kVco2ModSrc] = float(atype::kModSh);
    Rig r(c);
    r.on(0, 57.0f);
    const int blocks = int(seconds * kFs / float(kBlock));
    for (int b = 0; b < blocks; ++b) {
        for (int at : notesAt)
            if (b == at) r.on(0, 57.0f, 1.0f, true, true);
        r.run(1);
    }
    std::vector<double> at;
    const std::vector<double> f = freqTrack(r.L, &at);
    std::vector<double> semis;
    double t = 0.05;
    size_t i = 0;
    while (t < seconds) {
        double s = 0;
        int n = 0;
        for (; i < f.size() && at[i] < t + 0.01; ++i)
            if (at[i] >= t) s += f[i], ++n;
        if (n) semis.push_back(12.0 * std::log2(s / n / hz(57.0f)));
        t += 0.01;
    }
    return semis;
}
// Steps between 10 ms windows; a step spread over two windows counts once.
int jumps(const std::vector<double>& s, double minSemis) {
    int n = 0;
    bool last = false;
    for (size_t i = 1; i < s.size(); ++i) {
        const bool j = std::fabs(s[i] - s[i - 1]) > minSemis;
        n += j && !last;
        last = j;
    }
    return n;
}
}  // namespace

TEST(atype_sample_and_hold) {
    std::vector<float> c = only(1, atype::kSaw);
    // Rate: ~8 new values a second from noise.
    c[atype::kShRate] = 8.0f;
    const std::vector<double> a = shSteps(c, 2.0f);
    const int ja = jumps(a, 0.15);
    double lo = 1e9, hi = -1e9;
    for (double v : a) lo = std::min(lo, v), hi = std::max(hi, v);
    std::printf("  S&H 8 Hz: %d steps in 2 s, %.2f .. %.2f semitones\n", ja, lo, hi);
    CHECK(ja >= 10 && ja <= 18);
    CHECK(lo > -3.3 && hi < 3.3);
    CHECK(hi - lo > 1.5);
    // Key: a new value only at the notes.
    c[atype::kShClock] = float(atype::kClockKey);
    const std::vector<double> k = shSteps(c, 2.0f, {100, 300, 500});
    const int jk = jumps(k, 0.15);
    std::printf("  S&H Key: %d steps for 3 notes\n", jk);
    CHECK(jk >= 1 && jk <= 3);
    // Lag: the steps glide (intermediate values; fewer big 10 ms jumps).
    c[atype::kShClock] = float(atype::kClockRate);
    c[atype::kShLag] = 0.1f;
    const std::vector<double> g = shSteps(c, 2.0f);
    std::printf("  S&H lag 0.1 s: %d big steps\n", jumps(g, 0.6));
    CHECK(jumps(g, 0.6) < jumps(a, 0.6));
    // Inputs: VCO 1 (silent in the mix) is a source of its own, not the noise.
    std::vector<float> v1 = only(1, atype::kSaw);
    v1[atype::kShInput] = float(atype::kShVco1);
    v1[atype::kShRate] = 8.0f;
    v1[atype::kVco1Range] = float(atype::k16);
    v1[atype::kVco1Fine] = 37.0f;   // not a multiple of the clock: changing samples
    const std::vector<double> s1 = shSteps(v1, 2.0f);
    // (VCO 1 runs nearly whole cycles per clock here: small, steady steps.)
    std::printf("  S&H from VCO 1: %d steps\n", jumps(s1, 0.08));
    CHECK(jumps(s1, 0.08) >= 8);
    std::vector<float> nz = v1;
    nz[atype::kShInput] = float(atype::kShNoise);
    CHECK(rmsDiff(renderSequence(v1), renderSequence(nz)) > 1e-3);
    // The S&H into the filter: a whistle stepping in pitch.
    std::vector<float> w = whistle(600.0f, 0.0f);
    w[atype::kShAmt] = 0.3f;
    w[atype::kShRate] = 6.0f;
    const Buf x = play(w, 60.0f, 2.0f);
    std::vector<double> at;
    const std::vector<double> f = freqTrack(slice(x, 0.3, 2.0), &at);
    double flo = 1e9, fhi = 0;
    for (double v : f) flo = std::min(flo, v), fhi = std::max(fhi, v);
    std::printf("  S&H -> VCF: whistle %.0f .. %.0f Hz\n", flo, fhi);
    CHECK(fhi / flo > 1.3);
    CHECK(fhi / flo < 4.5);
}

TEST(atype_lfo_shapes_and_vibrato) {
    // VCO 1 LFO Mod 30 %: +-12 x 0.09 = +-1.08 semitones at 4 Hz.
    auto track = [](int shape) {
        std::vector<float> c = only(0, atype::kSaw);
        c[atype::kVco1LfoMod] = 0.3f;
        c[atype::kLfoRate] = 4.0f;
        c[atype::kLfoShape] = float(shape);
        const Buf x = play(c, 57.0f, 2.0f);
        std::vector<double> semis;
        for (double f : freqTrack(slice(x, 0.2, 2.0))) semis.push_back(12.0 * std::log2(f / hz(57.0f)));
        return semis;
    };
    for (int shape = 0; shape < 4; ++shape) {
        const std::vector<double> s = track(shape);
        double lo = 1e9, hi = -1e9;
        for (double v : s) lo = std::min(lo, v), hi = std::max(hi, v);
        int ext = 0, up = 0, down = 0;
        for (size_t i = 0; i < s.size(); ++i) {
            ext += std::fabs(s[i]) > 0.8 * 1.08;
            if (i) up += s[i] > s[i - 1] + 0.01, down += s[i] < s[i - 1] - 0.01;
        }
        const double fx = double(ext) / double(s.size());
        std::printf("  LFO shape %d: %.2f .. %.2f semitones, %.2f near the extremes, %d up / %d down\n", shape, lo, hi, fx, up, down);
        CHECK(hi > 0.9 && hi < 1.3);
        CHECK(lo < -0.9 && lo > -1.3);
        if (shape == atype::kSquare) CHECK(fx > 0.75);
        if (shape == atype::kTri) CHECK(fx < 0.35);
        if (shape == atype::kSine) CHECK(fx > 0.3 && fx < 0.65);
        if (shape == atype::kSawUp) CHECK(up > 3 * down);
    }
    // The rate.
    std::vector<float> c = only(0, atype::kSaw);
    c[atype::kVco1LfoMod] = 0.3f;
    c[atype::kLfoRate] = 3.0f;
    const Buf x = play(c, 57.0f, 2.2f);
    std::vector<double> at;
    const std::vector<double> f = freqTrack(slice(x, 0.2, 2.2), &at);
    int cross = 0;
    for (size_t i = 1; i < f.size(); ++i) cross += f[i - 1] < hz(57.0f) && f[i] >= hz(57.0f);
    CHECK(cross >= 5 && cross <= 7);   // 3 Hz over 2 s
}

namespace {
// Filter response at harmonic n of a 110 Hz saw (low level: linear): the
// filtered over the open output, same seeds (same phases).
double response(float cutoff, float res, int n, int mode = atype::k4Pole, float hpf = 20.0f) {
    std::vector<float> c = only(0, atype::kSaw, 0.1f);
    c[atype::kFilterMode] = float(mode);
    const Buf open = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    c[atype::kCutoff] = cutoff;
    c[atype::kResonance] = res;
    c[atype::kHpf] = hpf;
    const Buf f = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    return tone(f, 110.0 * n) / tone(open, 110.0 * n);
}
double ideal4(double f, double fc) {
    const double p = 1.0 / (1.0 + (f / fc) * (f / fc));
    return p * p;
}
}  // namespace

TEST(atype_hpf) {
    // 12 dB per octave below the HPF cutoff, flat well above; 20 Hz = off.
    const double h1 = db(response(20000.0f, 0.0f, 1, atype::k4Pole, 1760.0f));
    const double h2 = db(response(20000.0f, 0.0f, 2, atype::k4Pole, 1760.0f));
    const double h16 = db(response(20000.0f, 0.0f, 16, atype::k4Pole, 1760.0f));
    const double h64 = db(response(20000.0f, 0.0f, 64, atype::k4Pole, 1760.0f));
    std::printf("  HPF 1760 Hz: %.1f dB at 110, %.1f at 220, %.1f at the cutoff, %.1f at 7 kHz\n", h1, h2, h16, h64);
    CHECK_NEAR(h2 - h1, 12.0, 1.5);
    CHECK_NEAR(h16, -3.0, 1.0);
    CHECK(std::fabs(h64) < 1.0);
    CHECK(std::fabs(db(response(20000.0f, 0.0f, 1, atype::k4Pole, 20.0f))) < 0.5);
}

TEST(atype_filter_cutoff_slopes_and_resonance) {
    // 4-pole: -12 dB at the cutoff (no resonance), 24 dB per octave.
    for (float fc : {440.0f, 880.0f, 1760.0f}) {
        const int n = int(fc / 110.0f);
        const double g = db(response(fc, 0.0f, n));
        std::printf("  4-pole cutoff %.0f Hz: %.1f dB at the cutoff\n", fc, g);
        CHECK_NEAR(g, -12.0, 1.5);
        CHECK_NEAR(db(response(fc, 0.0f, 1)), db(ideal4(110.0, fc)), 1.0);
    }
    const double a = db(response(440.0f, 0.0f, 16)), b = db(response(440.0f, 0.0f, 32));
    std::printf("  4-pole slope %.1f dB per octave\n", b - a);
    CHECK(b - a < -21.0 && b - a > -27.0);
    // 2-pole: 12 dB per octave, flat pass band.
    const double c = db(response(440.0f, 0.0f, 16, atype::k2Pole)), d = db(response(440.0f, 0.0f, 32, atype::k2Pole));
    std::printf("  2-pole slope %.1f dB per octave, %.1f dB at the cutoff\n", d - c, db(response(880.0f, 0.0f, 8, atype::k2Pole)));
    CHECK(d - c < -10.5 && d - c > -14.0);
    CHECK(std::fabs(db(response(1760.0f, 0.0f, 1, atype::k2Pole))) < 0.5);
    CHECK(db(response(880.0f, 0.0f, 8, atype::k2Pole)) < -2.0);
    // Resonance 70 %: a peak at the cutoff in both modes.
    for (int mode = 0; mode < 2; ++mode) {
        const double at0 = response(880.0f, 0.0f, 8, mode), at7 = response(880.0f, 0.7f, 8, mode);
        std::printf("  mode %d resonance 70 %%: %+.1f dB at the cutoff\n", mode, db(at7 / at0));
        CHECK(at7 > 2.5 * at0);
    }
}

TEST(atype_2pole_steady_with_input_at_the_cutoff) {
    // A saw with the 2-pole's cutoff on its fundamental, Resonance up to the
    // top: the level is steady from block to block (no level jumps from the
    // singing control), bounded.
    for (float pitch : {84.0f, 96.0f, 108.0f})
        for (float res : {0.95f, 1.0f}) {
            std::vector<float> c = only(0, atype::kSaw, 0.8f);
            c[atype::kFilterMode] = float(atype::k2Pole);
            c[atype::kCutoff] = float(hz(pitch));
            c[atype::kResonance] = res;
            const Buf x = play(c, pitch, 1.0f);
            double lo = 1e9, hi = 0;
            for (size_t i = size_t(0.5 * kFs); i + kBlock <= x.size(); i += kBlock) {
                const double r = rms(x, i, i + kBlock);
                lo = std::min(lo, r);
                hi = std::max(hi, r);
            }
            std::printf("  2-pole at note %.0f, res %.2f: block rms %.3f .. %.3f, peak %.2f\n", pitch, res, lo, hi, peak(x));
            CHECK(hi < 1.3 * lo);
            CHECK(peak(x) < 1.5f);
        }
}

TEST(atype_self_oscillation_tracks_the_keyboard) {
    // Resonance 100 %, nothing in the mixer: the filter sings at the cutoff
    // (C4: no key offset) and follows the keys at Key Track 100 %, half as
    // much at 50 %; both modes.
    for (int mode = 0; mode < 2; ++mode)
        for (float kt : {1.0f, 0.5f}) {
            double f[3];
            for (int i = 0; i < 3; ++i) {
                const Buf x = play(whistle(440.0f, kt, mode), 48.0f + 12.0f * float(i), 1.0f);
                CHECK(finite(x));
                CHECK(peak(x) < 1.2f);
                if (i == 1) {
                    std::printf("  mode %d self-oscillation at C4, cutoff 440 Hz: %.1f Hz, %.3f rms\n", mode,
                                pitchHz(x, size_t(0.5 * kFs), x.size()), rms(x, size_t(0.5 * kFs)));
                    CHECK(rms(x, size_t(0.5 * kFs)) > 0.03);
                }
                f[i] = pitchHz(x, size_t(0.5 * kFs), x.size());
            }
            const double want = std::pow(2.0, double(kt));
            CHECK_NEAR(f[1] / f[0], want, want * 0.03);
            CHECK_NEAR(f[2] / f[1], want, want * 0.03);
            CHECK_NEAR(f[1], 440.0, 440.0 * 0.02);
        }
}

TEST(atype_env_amount_and_sources) {
    // Env Amt 1/7 = +1 octave at a held full envelope (Attack 0, Sustain 1).
    std::vector<float> c = whistle(330.0f, 0.0f);
    const double f0 = pitchHz(play(c, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    c[atype::kEnvAmt] = 1.0f / 7.0f;
    const double f1 = pitchHz(play(c, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    c[atype::kEnvAmt] = -1.0f / 7.0f;
    const double f2 = pitchHz(play(c, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    std::printf("  Env Amt +-1/7: %.1f -> %.1f / %.1f Hz\n", f0, f1, f2);
    CHECK_NEAR(f1 / f0, 2.0, 0.08);
    CHECK_NEAR(f0 / f2, 2.0, 0.08);
    // A decaying ADSR: bright at the start, dark at Sustain 0; negative the other way.
    std::vector<float> d = open();
    d[atype::kCutoff] = 200.0f;
    d[atype::kEnvAmt] = 0.8f;
    d[atype::kDecay] = 0.6f;
    d[atype::kSustain] = 0.0f;
    d[atype::kVcaEnv] = float(atype::kAr);   // the VCA held by the AR
    const Buf x = play(d, 45.0f, 1.0f);
    CHECK(bright(slice(x, 0.01, 0.06)) > 2.0 * bright(slice(x, 0.7, 0.95)));
    d[atype::kCutoff] = 3000.0f;
    d[atype::kEnvAmt] = -0.5f;
    const Buf y = play(d, 45.0f, 1.0f);
    CHECK(bright(slice(y, 0.01, 0.06)) < 0.8 * bright(slice(y, 0.7, 0.95)));
    // Env Src AR: the AR (held while the gate is on) instead of the ADSR:
    // the brightness stays.
    d[atype::kCutoff] = 200.0f;
    d[atype::kEnvAmt] = 0.8f;
    d[atype::kEnvSrc] = float(atype::kAr);
    d[atype::kArAttack] = 0.0f;
    const Buf z = play(d, 45.0f, 1.0f);
    CHECK_NEAR(bright(slice(z, 0.2, 0.3)) / bright(slice(z, 0.7, 0.95)), 1.0, 0.15);
    CHECK(bright(slice(z, 0.7, 0.95)) > 2.0 * bright(slice(x, 0.7, 0.95)));
}

TEST(atype_envelopes_and_the_vca) {
    constexpr double kBlockT = double(kBlock) / kFs;
    auto envOf = [](const Buf& x, double t) { return rms(x, size_t(t * kFs), size_t((t + 0.01) * kFs)); };
    // ADSR on the VCA: attack, decay to sustain, release.
    std::vector<float> c = open();
    c[atype::kVco1Level] = 0.4f;
    c[atype::kAttack] = 0.1f;
    c[atype::kDecay] = 0.2f;
    c[atype::kSustain] = 0.5f;
    c[atype::kRelease] = 0.2f;
    Rig r(c);
    r.on(0, 57.0f);
    r.run(int(1.0 / kBlockT));
    double full = 0;
    for (double t = 0.05; t < 0.2; t += 0.005) full = std::max(full, envOf(r.L, t));
    const double sus = envOf(r.L, 0.9);
    std::printf("  ADSR: %.3f at 5 ms, %.3f at the peak, %.3f at sustain\n", envOf(r.L, 0.005), full, sus);
    CHECK(envOf(r.L, 0.005) < 0.3 * full);
    CHECK_NEAR(sus / full, 0.5, 0.06);
    r.off(0);
    int blocks = 0;
    while (r.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    std::printf("  ADSR release 0.2 s: the voice ends after %.2f s\n", double(blocks) * kBlockT);
    CHECK(double(blocks) * kBlockT < 0.6);
    // AR on the VCA: full while held (no decay), its release at the gate off.
    c[atype::kVcaEnv] = float(atype::kAr);
    c[atype::kArAttack] = 0.05f;
    c[atype::kArRelease] = 0.5f;
    Rig a(c);
    a.on(0, 57.0f);
    a.run(int(1.0 / kBlockT));
    CHECK_NEAR(envOf(a.L, 0.9) / envOf(a.L, 0.2), 1.0, 0.05);
    a.off(0);
    const size_t at = a.L.size();
    a.run(int(0.2 / kBlockT));
    const double mid = rms(a.L, at + size_t(0.1 * kFs), at + size_t(0.11 * kFs));
    CHECK(mid > 0.2 * envOf(a.L, 0.9) && mid < 0.9 * envOf(a.L, 0.9));
    blocks = 0;
    while (a.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 10 * kBlocksPerSec);
    // ADSR at Sustain 0: the voice ends after the decay while still held.
    c = open();
    c[atype::kDecay] = 0.2f;
    c[atype::kSustain] = 0.0f;
    Rig d(c);
    d.on(0, 57.0f);
    blocks = 0;
    while (d.run(1) && blocks < 10 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 3 * kBlocksPerSec);
    // Attack 0: full at once; legato modes do not restart the envelope.
    c = open();
    c[atype::kDecay] = 0.1f;
    c[atype::kSustain] = 0.2f;
    for (int mode : {int(atype::kLegato), int(atype::kAlways)}) {
        c[atype::kGlideMode] = float(mode);
        Rig g(c);
        g.on(0, 45.0f);
        g.run(kBlocksPerSec / 2);
        const double s = rms(g.L, g.L.size() - 10 * kBlock, g.L.size());
        g.on(0, 52.0f, 1.0f, true, true);
        const size_t t0 = g.L.size();
        g.run(kBlocksPerSec / 10);
        const double after = rms(g.L, t0 + kBlock, t0 + 6 * kBlock) / s;
        if (mode == atype::kLegato) CHECK(after < 1.3);
        else CHECK(after > 2.0);
    }
}

TEST(atype_glide) {
    // Glide Time 0.3 s (95 %): from 45 up to 57 on a reused voice.
    std::vector<float> c = only(0, atype::kSaw);
    c[atype::kGlideTime] = 0.3f;
    Rig r(c);
    r.on(0, 45.0f);
    r.run(kBlocksPerSec / 2);
    const size_t t0 = r.L.size();
    r.on(0, 57.0f, 1.0f, true, true);
    r.run(kBlocksPerSec);
    const Buf x(r.L.begin() + long(t0), r.L.end());
    std::vector<double> at;
    const std::vector<double> f = freqTrack(x, &at);
    double t95 = -1;
    const double want = hz(45.0f) * std::pow(2.0, 0.95);   // 95 % of the way in pitch
    for (size_t i = 0; i < f.size(); ++i)
        if (f[i] >= want) {
            t95 = at[i];
            break;
        }
    std::printf("  glide 0.3 s: 95 %% after %.3f s\n", t95);
    CHECK(t95 > 0.22 && t95 < 0.4);
    CHECK_NEAR(pitchHz(x, size_t(0.7 * kFs), size_t(1.0 * kFs)), hz(57.0f), hz(57.0f) * 0.005);
    // Glide Mode Overlap: no glide without an overlap; a Glide FX owns the start.
    c[atype::kGlideMode] = float(atype::kOverlap);
    Rig o(c);
    o.on(0, 45.0f);
    o.run(kBlocksPerSec / 2);
    o.off(0);
    o.on(0, 57.0f, 1.0f, true, false);
    const size_t t1 = o.L.size();
    o.run(kBlocksPerSec / 4);
    const Buf y(o.L.begin() + long(t1), o.L.end());
    CHECK_NEAR(pitchHz(y, size_t(0.02 * kFs), size_t(0.12 * kFs)), hz(57.0f), hz(57.0f) * 0.01);
    c[atype::kGlideMode] = float(atype::kAlways);
    Rig g(c);
    g.on(0, 45.0f);
    g.run(kBlocksPerSec / 2);
    g.on(0, 57.0f, 1.0f, true, true, true);
    const size_t t2 = g.L.size();
    g.run(kBlocksPerSec / 4);
    const Buf z(g.L.begin() + long(t2), g.L.end());
    CHECK_NEAR(pitchHz(z, size_t(0.02 * kFs), size_t(0.12 * kFs)), hz(57.0f), hz(57.0f) * 0.01);
}

TEST(atype_duo_in_the_engine) {
    // Duo with two held notes: VCO 1 at the low one, VCO 2 at the high one;
    // one note: both on it; Mono: the note on both whatever is held.
    std::vector<float> c = only(0, atype::kSaw);
    c[atype::kVco2Level] = 0.25f;
    c[atype::kKeyboard] = float(atype::kDuo);
    Rig r(c);
    r.held(2, 45, 52);
    r.on(0, 52.0f);
    r.run(kBlocksPerSec / 2);
    const Buf x = slice(r.L, 0.15, 0.45);
    std::printf("  Duo: %.4f at the low note, %.4f at the high one\n", tone(x, hz(45.0f)), tone(x, hz(52.0f)));
    CHECK(tone(x, hz(45.0f)) > 0.01);
    CHECK(tone(x, hz(52.0f)) > 0.01);
    // The high note released: the voice gets the low one (return-to-held).
    r.held(1, 45, 45);
    r.on(0, 45.0f, 1.0f, true, true);
    const size_t t0 = r.L.size();
    r.run(kBlocksPerSec / 2);
    const Buf y(r.L.begin() + long(t0 + size_t(0.1 * kFs)), r.L.end());
    CHECK(tone(y, hz(52.0f)) < 0.1 * tone(y, hz(45.0f)));
    // Mono: two held notes, one pitch.
    c[atype::kKeyboard] = float(atype::kMono);
    Rig m(c);
    m.held(2, 45, 52);
    m.on(0, 52.0f);
    m.run(kBlocksPerSec / 2);
    const Buf z = slice(m.L, 0.15, 0.45);
    CHECK(tone(z, hz(45.0f)) < 0.1 * tone(z, hz(52.0f)));
    // Not on a Poly 2 slot (held.mono 0): each voice its own note.
    c[atype::kKeyboard] = float(atype::kDuo);
    Rig p(c);
    p.held(2, 45, 52);
    p.p.held.mono = 0;
    p.on(0, 52.0f);
    p.run(kBlocksPerSec / 2);
    const Buf w = slice(p.L, 0.15, 0.45);
    CHECK(tone(w, hz(45.0f)) < 0.1 * tone(w, hz(52.0f)));
}

TEST(atype_priority_and_duo_through_the_bank) {
    auto pitchOf = [](const BankSlot& s) { return pitchHz(s.tail(0.3), 0, size_t(0.3 * kFs)); };
    std::vector<float> c = only(0, atype::kSaw, 0.4f);
    // Last: the second key takes the voice; its release returns to the first.
    for (int prio = 0; prio < 3; ++prio) {
        c[atype::kPriority] = float(prio);
        BankSlot s(atype::kEngineId, 1, &c);
        s.on(0, 48);
        s.run(kBlocksPerSec / 2);
        s.on(1, 60);
        s.run(kBlocksPerSec / 2);
        const double both = pitchOf(s);
        s.off(prio == atype::kLow ? 0 : 1);   // release the sounding one
        s.run(kBlocksPerSec / 2);
        const double rest = pitchOf(s);
        std::printf("  priority %d: %.1f Hz with both held, %.1f Hz after the release\n", prio, both, rest);
        const double first = prio == atype::kLow ? hz(48.0f) : hz(60.0f);
        const double then = prio == atype::kLow ? hz(60.0f) : hz(48.0f);
        CHECK_NEAR(both, first, first * 0.01);
        CHECK_NEAR(rest, then, then * 0.01);
        CHECK_EQ(s.bank.voicesOf(0), 1);
        s.off(prio == atype::kLow ? 1 : 0);
        s.run(kBlocksPerSec);
    }
    // Duo from two tracks: both pitches; releasing either leaves the other.
    c[atype::kPriority] = float(atype::kLast);
    c[atype::kKeyboard] = float(atype::kDuo);
    c[atype::kVco2Level] = 0.4f;
    for (int rel = 0; rel < 2; ++rel) {
        BankSlot s(atype::kEngineId, 1, &c);
        s.on(0, 45);
        s.run(kBlocksPerSec / 4);
        s.on(1, 52);
        s.run(kBlocksPerSec / 2);
        const Buf both = s.tail(0.3);
        CHECK(tone(both, hz(45.0f)) > 0.01);
        CHECK(tone(both, hz(52.0f)) > 0.01);
        s.off(rel);
        s.run(kBlocksPerSec / 2);
        const Buf one = s.tail(0.3);
        const float keep = rel == 0 ? 52.0f : 45.0f, gone = rel == 0 ? 45.0f : 52.0f;
        std::printf("  Duo, release of track %d: %.4f kept, %.4f gone\n", rel, tone(one, hz(keep)), tone(one, hz(gone)));
        CHECK(tone(one, hz(keep)) > 0.01);
        CHECK(tone(one, hz(gone)) < 0.1 * tone(one, hz(keep)));
        CHECK_EQ(s.bank.voicesOf(0), 1);
        s.off(1 - rel);
        s.run(2 * kBlocksPerSec);
        CHECK_EQ(s.bank.voicesOf(0), 0);
    }
    // Mono through the bank: the second note alone.
    c[atype::kKeyboard] = float(atype::kMono);
    BankSlot m(atype::kEngineId, 1, &c);
    m.on(0, 45);
    m.on(1, 52);
    m.run(kBlocksPerSec / 2);
    const Buf z = m.tail(0.3);
    CHECK(tone(z, hz(45.0f)) < 0.1 * tone(z, hz(52.0f)));
    // Poly 2 with Duo: two voices, one note each.
    c[atype::kKeyboard] = float(atype::kDuo);
    BankSlot p(atype::kEngineId, 2, &c);
    p.on(0, 45);
    p.on(1, 52);
    p.run(kBlocksPerSec / 2);
    CHECK_EQ(p.bank.voicesOf(0), 2);
}

TEST(atype_init_retriggers_sequenced_notes) {
    // A tracker line plays note after note on one track without OFF steps:
    // Init (Glide Mode Always) restarts the envelopes on every note.
    std::vector<float> c = initValues();
    c[atype::kDecay] = 0.1f;
    c[atype::kSustain] = 0.2f;
    c[atype::kVco2Level] = 0.0f;
    BankSlot s(atype::kEngineId, 1, &c);
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

TEST(atype_parameter_jumps_do_not_click) {
    struct Jump {
        int param;
        float a, b;
    };
    const Jump jumps[] = {{atype::kResonance, 0.1f, 0.9f}, {atype::kCutoff, 300.0f, 5000.0f}, {atype::kVco1Level, 0.1f, 1.0f},
                          {atype::kEnvAmt, 0.0f, 0.6f},    {atype::kVco2Level, 0.0f, 1.0f},  {atype::kSourceLevel, 0.0f, 1.0f},
                          {atype::kHpf, 20.0f, 400.0f},    {atype::kVco1Pw, 0.0f, 0.9f},     {atype::kKeyTrack, 0.0f, 1.0f}};
    for (const Jump& j : jumps) {
        std::vector<float> c = open();
        c[atype::kCutoff] = 600.0f;
        c[atype::kVco1Wave] = float(j.param == atype::kVco1Pw ? atype::kPulse : atype::kSaw);
        c[atype::kVco1Level] = 0.4f;
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

TEST(atype_no_nan_or_denormals) {
    uint32_t seed = 71;
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
        if (rnd() < 0.5f) r.held(2, int(rnd() * 60.0f), 60 + int(rnd() * 67.0f));
        r.on(0, float(int(rnd() * 127.0f)), rnd());
        r.on(1, float(int(rnd() * 127.0f)), rnd());
        r.run(kBlocksPerSec);
        r.on(1, float(int(rnd() * 127.0f)), rnd(), true, rnd() < 0.5f);
        r.run(kBlocksPerSec / 4);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec / 4);
        if (!finite(r.L) || peak(r.L) > 2.5f) {
            ++bad;
            std::printf("       set %d: peak %.3f%s\n", set, peak(r.L), finite(r.L) ? "" : " (not finite)");
        }
        sub += subnormals(r.L);
    }
    CHECK_EQ(bad, 0);
    CHECK_EQ(sub, 0L);
    for (int ext = 0; ext < 2; ++ext) {
        std::vector<float> a(kN);
        for (int i = 0; i < kN; ++i) a[i] = ext ? range(i)->max : range(i)->min;
        a[atype::kVco1Level] = 1.0f;
        Rig r(a);
        r.held(2, 0, 127);
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
    c[atype::kRelease] = 10.0f;
    c[atype::kSustain] = 1.0f;
    Rig t(c);
    t.on(0, 40.0f);
    t.run(50);
    t.off(0);
    int blocks = 0;
    while (t.run(1) && blocks < 60 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 60 * kBlocksPerSec);
    CHECK_EQ(subnormals(t.L), 0L);
    // The self-oscillating filters with nothing in them, cutoff at the bottom.
    for (int mode = 0; mode < 2; ++mode) {
        Rig q(whistle(20.0f, 0.0f, mode));
        q.on(0, 0.0f);
        q.run(5 * kBlocksPerSec);
        CHECK(finite(q.L));
        CHECK_EQ(subnormals(q.L), 0L);
    }
    // Stable under fast modulation: the cutoff and resonance slammed every block.
    std::vector<float> m = richContext();
    m[atype::kSync] = 1.0f;
    m[atype::kSource] = float(atype::kRing);
    m[atype::kSourceLevel] = 1.0f;
    Rig f(m);
    f.on(0, 96.0f);
    for (int b = 0; b < 3 * kBlocksPerSec; ++b) {
        m[atype::kCutoff] = 20.0f + rnd() * 19980.0f;
        m[atype::kResonance] = rnd() < 0.5f ? 1.0f : 0.0f;
        m[atype::kFilterMode] = float(b / 50 % 2);
        f.set(m);
        f.run(1);
    }
    CHECK(finite(f.L));
    CHECK(peak(f.L) < 2.0f);
    CHECK_EQ(subnormals(f.L), 0L);
}

TEST(atype_voice_level) {
    std::vector<float> c = open();
    c[atype::kAttack] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    const float lv = r.v[0].level;
    r.on(0, 62.0f, 1.0f, true, false);
    CHECK_EQ(r.v[0].level, lv);
}

TEST(atype_volume_is_never_applied) {
    std::vector<float> a = initValues(), b = initValues();
    a[atype::kVolume] = 0.0f;
    b[atype::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

namespace {
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

TEST(atype_level_matches_the_synth_conventions) {
    long c1 = 0, c4 = 0;
    const float p1 = chordPeak(atype::kEngineId, 1, c1);
    const float p4 = chordPeak(atype::kEngineId, 4, c4);
#if defined(OT_MOD_ACD1)
    long ca = 0;
    const float pa = chordPeak(kSynthEngineAcd, 1, ca);
    std::printf("  A-Type Init at -9 dB: 1 voice peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(pa),
                dbfs(p4));
    CHECK(std::fabs(dbfs(p1) - dbfs(pa)) < 1.5);
#else
    std::printf("  A-Type Init at -9 dB: 1 voice peak %.1f dBFS, 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(p4));
#endif
    CHECK(dbfs(p1) < -6.0);
    CHECK(dbfs(p1) > -20.0);
    CHECK(dbfs(p4) < -1.0);
    CHECK_EQ(c1, 0L);
    CHECK_EQ(c4, 0L);
}

TEST(atype_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        // Volume is the framework's; Priority is the framework's note choice.
        if (i == atype::kVolume || i == atype::kPriority) continue;
        const PatchParamRange* g = range(i);
        std::vector<float> a = ctx, b = ctx;
        a[i] = g->min;
        b[i] = g->max;
        // Context for the switches whose other side needs something to act on.
        if (i == atype::kVco1Pw || i == atype::kVco1Pwm) a[atype::kVco1Wave] = b[atype::kVco1Wave] = float(atype::kPulse);
        if (i == atype::kVco2Pw || i == atype::kVco2Pwm) a[atype::kVco2Wave] = b[atype::kVco2Wave] = float(atype::kPulse);
        if (i == atype::kShInput || i == atype::kShClock || i == atype::kShLag) a[atype::kShAmt] = b[atype::kShAmt] = 0.5f;
        if (i == atype::kEnvSrc) a[atype::kEnvAmt] = b[atype::kEnvAmt] = 0.8f;
        if (i == atype::kArAttack || i == atype::kArRelease) a[atype::kVcaEnv] = b[atype::kVcaEnv] = float(atype::kAr);
        const double d = rmsDiff(renderSequence(a), renderSequence(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

// Golden hashes (host build, -ffp-contract=off): a rich note sequence on the
// engine alone (both filter modes) and an Init line plus a Duo pair through a
// SynthBank slot. Any change of the sound shows here; update the values only
// for an intended change.
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
    c[atype::kSync] = 1.0f;
    c[atype::kSource] = float(atype::kRing);
    Rig r(c);
    r.run(3);
    r.on(0, 45.0f, 0.6f);
    r.run(kBlocksPerSec / 3);
    r.held(2, 45, 57);
    c[atype::kKeyboard] = float(atype::kDuo);
    r.set(c);
    r.on(0, 57.0f, 0.9f, true, true);
    r.run(kBlocksPerSec / 3);
    c[atype::kFilterMode] = float(atype::k2Pole);
    c[atype::kResonance] = 0.95f;
    r.set(c);
    r.held(1, 52, 52);
    r.on(0, 52.0f, 0.9f, true, false);
    r.run(kBlocksPerSec / 3);
    r.held(0);
    r.off(0);
    r.run(kBlocksPerSec / 2);
    return hashOf(r.L, r.R);
}
uint64_t bankHash() {
    std::vector<float> c = initValues();
    c[atype::kKeyboard] = float(atype::kDuo);
    BankSlot s(atype::kEngineId, 1, &c);
    const int line[6] = {36, 48, 43, 36, 55, 31};
    for (int k = 0; k < 6; ++k) {
        s.on(0, line[k]);
        s.run(40);
    }
    s.on(1, 50);   // a Duo pair
    s.run(80);
    s.off(0);
    s.run(40);
    s.off(1);
    s.run(200);
    return hashOf(s.out, s.out);
}
}  // namespace

TEST(atype_golden_hashes) {
    const uint64_t e = engineHash(), b = bankHash();
    std::printf("  A-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x4726a385ffa76e27ull));
    CHECK_EQ(b, uint64_t(0x1c6e946460fd4883ull));
}

// --- Performance filter offset (issue #23) ---------------------------------------------
// SynthVoice::cutoffOct (octaves, filled by the framework from the track's
// Performance filter) moves the A-Type cutoff: Cutoff x 2^offset, inside the
// engine's own clamps. A patch at 1600 Hz shifted by -2 octaves sounds like
// the same patch at 400 Hz; 0 leaves the render bit-identical.
// The HPF is a separate stage and stays where the patch puts it.

namespace {
namespace perfcut {

// One note (pitch 48, 40 blocks) from the Init patch with its cutoff set by
// `cut(patch, hz)`, the voice's cutoffOct = `oct`.
template <class Cut>
std::vector<float> render(Cut cut, float hz, float oct) {
    const int32_t engine = atype::kEngineId;
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
    CHECK(synthParamSet(p, atype::kCutoff, hz));
}

}  // namespace perfcut
}  // namespace

TEST(atype_perf_cutoff_offset_moves_the_cutoff) {
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
