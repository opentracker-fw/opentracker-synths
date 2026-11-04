// CS-Type two-layer engine (opentracker-synths modules/cstype, engine 0x83):
// registered with a valid descriptor ("CS-Type" on screen, Patches/CSTYPE/),
// silent when idle, per layer the pitch, Feet and II Detune, the saw, pulse
// (widths, PWM at its own speed), sine and noise, the high-pass and low-pass
// (slopes, in series, resonance, self-oscillation with Key at half an octave
// per octave), the IL / AL contour, the per-layer VCAs (layer II at Level 0
// is layer I alone), the ring modulator (sidebands, Depth, Sweep over RM
// Decay), the sub-osc LFO into pitch / filters / VCAs, Brilliance and
// Resonance, Touch (after: VCO vibrato, VCF, VCA), velocity to the attack
// levels and the VCAs, the Ribbon, glide and legato, polyphony through a
// SynthBank slot; no click on parameter jumps, no NaN or denormals, the voice
// level for stealing, Volume never applied, every parameter audible, the
// slot level of the other engines, golden hashes.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "cstype.h"
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
constexpr int kN = cstype::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(cstype::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

// One engine voice pool driven directly (no framework). held: what the
// framework would report for the slot (Duo).
struct Rig {
    ISynthEngine* e = synthEngineFor(cstype::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    std::vector<float> L, R;

    explicit Rig(const std::vector<float>& vals) {
        p.engine = cstype::kEngineId;
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

constexpr int kII = cstype::kLayer2;

// A plain context: layer I a saw at 8' alone at full level, both filters
// open (HPF 20 Hz, LPF 20 kHz, no resonance, a flat contour), its VCA held
// at full sustain; layer II silent; no ring, sub-osc, Touch, Ribbon or glide.
std::vector<float> open() {
    std::vector<float> c = initValues();
    for (int o : {0, kII}) {
        c[size_t(o + cstype::kLpfCutoff)] = 20000.0f;
        c[size_t(o + cstype::kLpfRes)] = 0.0f;
        c[size_t(o + cstype::kInitLevel)] = 0.0f;
        c[size_t(o + cstype::kAttackLevel)] = 0.0f;
        c[size_t(o + cstype::kAttack)] = 0.0f;
        c[size_t(o + cstype::kSustain)] = 1.0f;
        c[size_t(o + cstype::kDecay)] = 0.05f;
        c[size_t(o + cstype::kRelease)] = 0.05f;
    }
    c[cstype::kLevel] = 1.0f;
    c[kII + cstype::kLevel] = 0.0f;
    c[cstype::kDetune] = 0.0f;
    c[cstype::kInitBril] = 0.0f;
    c[cstype::kAfterVco] = 0.0f;
    return c;
}
// One layer alone: wave 0 saw, 1 pulse, 2 sine; at a low level (linear).
std::vector<float> only(int layer, int wave, float level = 0.3f) {
    std::vector<float> c = open();
    const int o = layer ? kII : 0;
    c[cstype::kLevel] = 0.0f;
    c[size_t(o + cstype::kLevel)] = level;
    c[size_t(o + cstype::kSaw)] = wave == 0 ? 1.0f : 0.0f;
    c[size_t(o + cstype::kPulse)] = wave == 1 ? 1.0f : 0.0f;
    c[size_t(o + cstype::kSine)] = wave == 2 ? 1.0f : 0.0f;
    return c;
}
// Layer I's low-pass alone, self-oscillating: no wave (the seed starts it).
std::vector<float> whistle(float cutoff = 440.0f) {
    std::vector<float> c = open();
    c[cstype::kSaw] = 0.0f;
    c[cstype::kLpfCutoff] = cutoff;
    c[cstype::kLpfRes] = 1.0f;
    c[cstype::kLevel] = 0.5f;
    return c;
}
Buf play(const std::vector<float>& vals, float pitch = 48.0f, float seconds = 0.5f, float velocity = 1.0f) {
    Rig r(vals);
    r.on(0, pitch, velocity);
    r.run(int(seconds * kFs / float(kBlock)) + 1);
    return r.L;
}
const size_t kA = size_t(0.15 * 44100), kB = size_t(0.45 * 44100);
double highFraction(const Buf& x) {
    double mean = 0;
    for (float s : x) mean += s;
    mean /= double(x.size());
    size_t n = 0;
    for (float s : x) n += s > mean;
    return double(n) / double(x.size());
}

// Every section of the voice in use.
std::vector<float> richContext() {
    std::vector<float> c = initValues();
    c[cstype::kPulse] = 1.0f;
    c[cstype::kPw] = 0.3f;
    c[cstype::kPwm] = 0.3f;
    c[cstype::kPwmSpeed] = 3.0f;
    c[cstype::kSine] = 0.3f;
    c[cstype::kNoise] = 0.1f;
    c[cstype::kHpfCutoff] = 80.0f;
    c[cstype::kHpfRes] = 0.3f;
    c[cstype::kLpfCutoff] = 1200.0f;
    c[cstype::kLpfRes] = 0.4f;
    c[cstype::kInitLevel] = 0.1f;
    c[cstype::kAttackLevel] = 0.5f;
    c[cstype::kFAttack] = 0.05f;
    c[cstype::kFDecay] = 0.1f;
    c[cstype::kFRelease] = 0.1f;
    c[cstype::kAttack] = 0.01f;
    c[cstype::kDecay] = 0.1f;
    c[cstype::kSustain] = 0.6f;
    c[cstype::kRelease] = 0.1f;
    c[kII + cstype::kFeet] = float(cstype::k4);
    c[kII + cstype::kPulse] = 1.0f;
    c[kII + cstype::kPw] = 0.5f;
    c[kII + cstype::kPwm] = 0.4f;
    c[kII + cstype::kPwmSpeed] = 6.0f;
    c[kII + cstype::kSine] = 0.2f;
    c[kII + cstype::kNoise] = 0.1f;
    c[kII + cstype::kHpfCutoff] = 120.0f;
    c[kII + cstype::kHpfRes] = 0.3f;
    c[kII + cstype::kLpfRes] = 0.5f;
    c[kII + cstype::kInitLevel] = -0.1f;
    c[kII + cstype::kFAttack] = 0.04f;
    c[kII + cstype::kFDecay] = 0.08f;
    c[kII + cstype::kFRelease] = 0.1f;
    c[kII + cstype::kAttack] = 0.02f;
    c[kII + cstype::kDecay] = 0.1f;
    c[kII + cstype::kSustain] = 0.5f;
    c[kII + cstype::kRelease] = 0.1f;
    c[cstype::kBrilliance] = 0.1f;
    c[cstype::kResonance] = 0.1f;
    c[cstype::kDetune] = 9.0f;
    c[cstype::kRmDepth] = 0.3f;
    c[cstype::kRmSpeed] = 180.0f;
    c[cstype::kRmSweep] = 0.3f;
    c[cstype::kRmDecay] = 0.2f;
    c[cstype::kFunction] = float(cstype::kFnSine);
    c[cstype::kSpeed] = 6.0f;
    c[cstype::kToVco] = 0.2f;
    c[cstype::kToVcf] = 0.2f;
    c[cstype::kToVca] = 0.2f;
    c[cstype::kTouch] = 0.5f;
    c[cstype::kInitBril] = 0.5f;
    c[cstype::kInitLevelVel] = 0.5f;
    c[cstype::kAfterVco] = 0.5f;
    c[cstype::kAfterVcf] = 0.5f;
    c[cstype::kAfterVca] = 0.5f;
    c[cstype::kRibbon] = 0.3f;
    c[cstype::kRibbonRange] = 5.0f;
    c[cstype::kGlideTime] = 0.1f;
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

TEST(cstype_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(cstype::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "CSTYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(cstype::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(cstype::kEngineId));
    CHECK_STR(d->name, "CSTYPE");
    CHECK_STR(d->displayName, "CS-Type");
    CHECK_EQ(int(d->paramCount), 67);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(cstype::kVolume));
    CHECK_EQ(int(d->flags), 0);   // polyphonic: no note priority
    CHECK_EQ(patchVolumeParam(cstype::kEngineId), int(cstype::kVolume));
    CHECK_STR(patchEngineName(cstype::kEngineId), "CSTYPE");
    CHECK_EQ(int(d->sectionCount), synthui::kMaxSections);
    CHECK_EQ(int(d->extCount), 0);
    CHECK_STR(synthEngineFolder(cstype::kEngineId), "CSTYPE");
    CHECK_STR(synthEngineDisplayName(cstype::kEngineId), "CS-Type");
    const int k = synthModelListIndex(cstype::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "CS-Type");
#if defined(OT_MOD_DTY1)
    CHECK(k > synthModelListIndex(0x82));
#endif
#if defined(OT_MOD_ATY1)
    CHECK(k < synthModelListIndex(0x84));
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
        CHECK_EQ(macros, 0);
        for (int j = 0; j < sec.count; ++j) {
            CHECK(sec.params[j] < kN);
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
    // The two layers share one table, Init values apart.
    for (int i = 0; i < kII; ++i) {
        CHECK_STR(d->ui[i].label, d->ui[kII + i].label);
        CHECK(d->params[i].min == d->params[kII + i].min && d->params[i].max == d->params[kII + i].max);
    }
    SynthPatch p;
    CHECK(patchInit(cstype::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(cstype::kVolume));
}

TEST(cstype_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(cstype::kEngineId, &p));
    const float vals[][2] = {{cstype::kFeet, 3}, {kII + cstype::kLpfCutoff, 432.5f}, {cstype::kDetune, -12.5f},
                             {cstype::kRmSpeed, 1234.0f}, {cstype::kFunction, 4}, {cstype::kTouch, 0.75f},
                             {cstype::kRibbon, -0.5f}, {cstype::kGlideTime, 0.25f}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(cstype::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(cstype::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    CHECK(!patchParamInRange(cstype::kEngineId, cstype::kFeet, 4.0f));
    CHECK(!patchParamInRange(cstype::kEngineId, cstype::kRmSpeed, 2500.0f));
}

TEST(cstype_silent_when_idle) {
    Rig r(initValues());
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
    float l[kBlock] = {}, rr[kBlock] = {};
    CHECK(!r.e->render(r.v[3], r.p, 1.0f, 0.0f, l, rr));
    CHECK_EQ(peak(Buf(l, l + kBlock)), 0.0f);
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

TEST(cstype_pitch_feet_and_detune) {
    for (float pitch : {69.0f, 57.0f, 45.0f}) {
        const Buf x = play(only(0, 0), pitch);
        CHECK(finite(x));
        CHECK_NEAR(pitchHz(x, kA, kB), hz(pitch), hz(pitch) * 0.005);
    }
    // Feet 16' 8' 4' 2' = -1 .. +2 octaves, both layers.
    const float oct[4] = {-1, 0, 1, 2};
    for (int layer = 0; layer < 2; ++layer)
        for (int f = 0; f < 4; ++f) {
            std::vector<float> c = only(layer, 0);
            c[size_t((layer ? kII : 0) + cstype::kFeet)] = float(f);
            const float note = f <= cstype::k8 ? 69.0f : 45.0f;
            const double want = hz(note) * std::pow(2.0, oct[f]);
            CHECK_NEAR(pitchHz(play(c, note), kA, kB), want, want * 0.005);
        }
    // II Detune moves layer II only.
    std::vector<float> c = only(1, 0);
    c[cstype::kDetune] = 50.0f;
    CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(57.5f), hz(57.5f) * 0.002);
    c[cstype::kDetune] = -50.0f;
    CHECK_NEAR(pitchHz(play(c, 57.0f), kA, kB), hz(56.5f), hz(56.5f) * 0.002);
    std::vector<float> d = only(0, 0);
    d[cstype::kDetune] = 50.0f;
    CHECK_NEAR(pitchHz(play(d, 57.0f), kA, kB), hz(57.0f), hz(57.0f) * 0.002);
}

TEST(cstype_waves_widths_and_pwm) {
    auto h = [](const Buf& x, int n) { return tone(x, 110.0 * n) / tone(x, 110.0); };
    const Buf saw = slice(play(only(0, 0), 45.0f), 0.1, 0.45);
    const Buf sq = slice(play(only(0, 1), 45.0f), 0.1, 0.45);
    const Buf sine = slice(play(only(0, 2), 45.0f), 0.1, 0.45);
    CHECK_NEAR(h(saw, 2), 0.5, 0.06);
    CHECK_NEAR(h(saw, 3), 1.0 / 3.0, 0.05);
    CHECK(h(sq, 2) < 0.03);
    CHECK_NEAR(h(sq, 3), 1.0 / 3.0, 0.05);
    CHECK(h(sine, 2) < 0.01 && h(sine, 3) < 0.02);
    // Saw and pulse together: both.
    std::vector<float> both = only(0, 0);
    both[cstype::kPulse] = 1.0f;
    const Buf sp = slice(play(both, 45.0f), 0.1, 0.45);
    CHECK(rms(sp) > 1.2 * rms(saw));
    CHECK(h(sp, 2) < h(saw, 2) - 0.1);
    // PW 0 = 50 %, 1 = 5 %.
    for (int layer = 0; layer < 2; ++layer)
        for (float pw : {0.0f, 0.5f, 1.0f}) {
            std::vector<float> c = only(layer, 1);
            c[size_t((layer ? kII : 0) + cstype::kPw)] = pw;
            CHECK_NEAR(highFraction(slice(play(c, 45.0f), 0.1, 0.45)), 0.5 - 0.45 * pw, 0.03);
        }
    // PWM: the layer's own LFO at its speed sweeps the width.
    for (int layer = 0; layer < 2; ++layer) {
        std::vector<float> c = only(layer, 1);
        const int o = layer ? kII : 0;
        c[size_t(o + cstype::kPwmSpeed)] = 1.0f;
        auto swing = [&c]() {
            const Buf x = play(c, 45.0f, 2.0f);
            double lo = 1, hi = 0;
            for (double t0 = 0.1; t0 + 0.05 < 2.0; t0 += 0.05) {
                const double f = highFraction(slice(x, t0, t0 + 0.05));
                lo = std::min(lo, f);
                hi = std::max(hi, f);
            }
            return hi - lo;
        };
        CHECK(swing() < 0.03);
        c[size_t(o + cstype::kPwm)] = 1.0f;
        CHECK(swing() > 0.3);
    }
    // Noise: broadband, audible.
    std::vector<float> n = only(0, 3);
    n[cstype::kNoise] = 1.0f;
    const Buf nz = slice(play(n, 45.0f, 1.0f), 0.1, 1.0);
    CHECK(rms(nz) > 0.01);
    CHECK(bright(nz) > 3.0 * bright(saw));
}

namespace {
// Layer I's filter response at harmonic n of a 110 Hz saw (low level).
double response(float lpf, float lpRes, int n, float hpf = 20.0f, float hpRes = 0.0f) {
    std::vector<float> c = only(0, 0, 0.1f);
    const Buf open = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    c[cstype::kLpfCutoff] = lpf;
    c[cstype::kLpfRes] = lpRes;
    c[cstype::kHpfCutoff] = hpf;
    c[cstype::kHpfRes] = hpRes;
    const Buf f = slice(play(c, 45.0f, 0.6f), 0.2, 0.6);
    return tone(f, 110.0 * n) / tone(open, 110.0 * n);
}
}  // namespace

TEST(cstype_filters_cutoff_slopes_series_resonance) {
    // LPF at Key C4 offsets (note 45: 1/2 x -15 semitones): the low-pass cutoff
    // sits 0.625 octaves below its setting; 12 dB per octave above.
    const double a = db(response(440.0f, 0.0f, 16)), b = db(response(440.0f, 0.0f, 32));
    std::printf("  LPF slope %.1f dB per octave\n", b - a);
    CHECK(b - a < -10.5 && b - a > -14.0);
    CHECK(std::fabs(db(response(8000.0f, 0.0f, 1))) < 0.5);
    // HPF: 12 dB per octave below, flat above.
    const double h1 = db(response(20000.0f, 0.0f, 1, 1760.0f)), h2 = db(response(20000.0f, 0.0f, 2, 1760.0f));
    std::printf("  HPF 1760 Hz: %.1f dB at 110, %.1f at 220, %.1f at 7 kHz\n", h1, h2, db(response(20000.0f, 0.0f, 64, 1760.0f)));
    CHECK_NEAR(h2 - h1, 12.0, 1.5);
    CHECK(std::fabs(db(response(20000.0f, 0.0f, 64, 1760.0f))) < 1.5);
    // In series: a high-pass above the low-pass leaves little of either band.
    const double sb = response(300.0f, 0.0f, 8, 3000.0f);
    CHECK(sb < 0.25 * response(300.0f, 0.0f, 8) && sb < 0.25 * response(20000.0f, 0.0f, 8, 3000.0f));
    // Resonance 70 %: a peak at the cutoff, on both filters.
    const double fc = 880.0 * std::pow(2.0, -0.625);   // the LPF's cutoff at note 45
    const int nh = int(std::round(fc / 110.0));
    CHECK(response(880.0f, 0.7f, nh) > 2.5 * response(880.0f, 0.0f, nh));
    CHECK(response(20000.0f, 0.0f, 8, 880.0f, 0.7f) > 2.5 * response(20000.0f, 0.0f, 8, 880.0f, 0.0f));
}

TEST(cstype_self_oscillation_and_key) {
    // LPF Res 100 %, no wave: it sings at the cutoff (C4) and follows the keys
    // at half an octave per octave.
    double f[3];
    for (int i = 0; i < 3; ++i) {
        const Buf x = play(whistle(440.0f), 48.0f + 12.0f * float(i), 1.0f);
        CHECK(finite(x));
        CHECK(peak(x) < 1.2f);
        f[i] = pitchHz(x, size_t(0.5 * kFs), x.size());
        if (i == 1) {
            std::printf("  self-oscillation at C4, cutoff 440 Hz: %.1f Hz, %.3f rms\n", f[i], rms(x, size_t(0.5 * kFs)));
            CHECK(rms(x, size_t(0.5 * kFs)) > 0.02);
        }
    }
    CHECK_NEAR(f[1], 440.0, 440.0 * 0.02);
    CHECK_NEAR(f[1] / f[0], std::sqrt(2.0), 0.05);
    CHECK_NEAR(f[2] / f[1], std::sqrt(2.0), 0.05);
}

TEST(cstype_lpf_steady_with_input_at_the_cutoff) {
    // A saw with the low-pass on its fundamental (Key's offset taken out),
    // Res up to the top: the level is steady from block to block, bounded.
    for (float pitch : {72.0f, 96.0f})
        for (float res : {0.95f, 1.0f}) {
            std::vector<float> c = only(0, 0, 0.8f);
            c[cstype::kLpfCutoff] = float(hz(pitch) * std::pow(2.0, -0.5 * (double(pitch) - 60.0) / 12.0));
            c[cstype::kLpfRes] = res;
            const Buf x = play(c, pitch, 1.0f);
            double lo = 1e9, hi = 0;
            for (size_t i = size_t(0.5 * kFs); i + kBlock <= x.size(); i += kBlock) {
                const double r = rms(x, i, i + kBlock);
                lo = std::min(lo, r);
                hi = std::max(hi, r);
            }
            std::printf("  LPF at note %.0f, res %.2f: block rms %.3f .. %.3f, peak %.2f\n", pitch, res, lo, hi, peak(x));
            CHECK(hi < 1.3 * lo);
            CHECK(peak(x) < 2.0f);
        }
}

TEST(cstype_il_al_contour) {
    // IL 0, AL 1/4 (+1 octave): the whistle rises an octave in the attack and
    // falls back in the decay; IL 1/4 holds it up; the release goes to 0.
    std::vector<float> c = whistle(330.0f);
    c[cstype::kAttackLevel] = 0.25f;
    c[cstype::kFAttack] = 0.15f;
    c[cstype::kFDecay] = 0.3f;
    c[cstype::kRelease] = 2.0f;
    c[cstype::kInitBril] = 0.0f;
    Rig r(c);
    r.on(0, 60.0f);
    r.run(int(1.5 * kFs / kBlock));
    double atTop = 0;
    for (double t = 0.08; t < 0.3; t += 0.005) atTop = std::max(atTop, freqAt(r.L, t));
    const double late = freqAt(r.L, 1.4);
    std::printf("  IL/AL: %.1f Hz at the attack's end, %.1f Hz after the decay\n", atTop, late);
    CHECK_NEAR(atTop / late, 2.0, 0.15);
    CHECK_NEAR(late, 330.0, 330.0 * 0.03);
    c[cstype::kInitLevel] = 0.25f;
    Rig h(c);
    h.on(0, 60.0f);
    h.run(int(1.0 * kFs / kBlock));
    CHECK_NEAR(freqAt(h.L, 0.9), 660.0, 660.0 * 0.04);
    h.off(0);
    h.run(int(1.0 * kFs / kBlock));
    CHECK_NEAR(freqAt(h.L, 1.9), 330.0, 330.0 * 0.05);
    // A negative attack level closes it.
    c[cstype::kInitLevel] = 0.0f;
    c[cstype::kAttackLevel] = -0.25f;
    Rig n(c);
    n.on(0, 60.0f);
    n.run(int(0.5 * kFs / kBlock));
    CHECK(freqAt(n.L, 0.16) < 0.6 * 330.0);
}

TEST(cstype_il_al_starts_at_the_initial_level) {
    // A fresh note starts the contour at IL: IL -1/4 (an octave down) with a
    // slow attack: the whistle starts an octave below the cutoff.
    std::vector<float> c = whistle(660.0f);
    c[cstype::kInitLevel] = -0.25f;
    c[cstype::kAttackLevel] = 0.25f;   // the slow attack rises from IL towards +1 octave
    c[cstype::kFAttack] = 4.0f;
    c[cstype::kInitBril] = 0.0f;
    Rig r(c);
    r.on(0, 60.0f);
    r.run(int(0.4 * kFs / kBlock));
    const double f = freqAt(r.L, 0.3);
    std::printf("  IL start: %.1f Hz at 0.3 s (cutoff 660 Hz, IL -1 octave)\n", f);
    CHECK(f < 660.0 * 0.62);
}

TEST(cstype_sings_when_res_rises_mid_note) {
    // No wave, Res 50 % (silent), then Res 100 % during the note: it sings.
    std::vector<float> c = whistle(440.0f);
    c[cstype::kLpfRes] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    r.run(kBlocksPerSec);
    c[cstype::kLpfRes] = 1.0f;
    r.set(c);
    const size_t t0 = r.L.size();
    r.run(kBlocksPerSec);
    std::printf("  Res raised mid-note: %.3f rms\n", rms(r.L, t0 + size_t(0.5 * kFs)));
    CHECK(rms(r.L, t0 + size_t(0.5 * kFs)) > 0.02);
}

TEST(cstype_layers_and_their_vcas) {
    // Layer II at Level 0: exactly layer I alone, whatever layer II's settings.
    std::vector<float> a = initValues();
    a[kII + cstype::kLevel] = 0.0f;
    std::vector<float> b = a;
    b[kII + cstype::kPulse] = 1.0f;
    b[kII + cstype::kFeet] = 3.0f;
    b[kII + cstype::kLpfRes] = 0.9f;
    CHECK(play(a) == play(b));
    // Each layer's own ADSR: layer I quick, layer II slow.
    std::vector<float> c = open();
    c[cstype::kSaw] = 0.0f;   // layer I a sine at 110 Hz, layer II one at 220 Hz
    c[cstype::kSine] = 1.0f;
    c[cstype::kLevel] = 0.3f;
    c[kII + cstype::kLevel] = 0.3f;
    c[kII + cstype::kSaw] = 0.0f;
    c[kII + cstype::kSine] = 1.0f;
    c[kII + cstype::kFeet] = float(cstype::k4);
    c[kII + cstype::kAttack] = 0.5f;
    const Buf x = play(c, 45.0f, 1.0f);
    const Buf early = slice(x, 0.02, 0.07), lateB = slice(x, 0.8, 0.95);
    // Layer II's sine at 220 Hz: small early, full late; layer I steady.
    CHECK(tone(early, 220.0) < 0.3 * tone(lateB, 220.0));
    CHECK_NEAR(tone(early, 110.0) / tone(lateB, 110.0), 1.0, 0.1);
    // Levels.
    c[cstype::kLevel] = 0.15f;
    CHECK_NEAR(tone(slice(play(c, 45.0f, 1.0f), 0.8, 0.95), 110.0) / tone(lateB, 110.0), 0.5, 0.05);
}

TEST(cstype_ring_modulator) {
    // A 440 Hz sine x a 100 Hz sine: 340 and 540 Hz, not 440 (Depth 100 %).
    std::vector<float> c = only(0, 2);
    c[cstype::kRmDepth] = 1.0f;
    c[cstype::kRmSpeed] = 100.0f;
    const Buf x = slice(play(c, 69.0f), 0.1, 0.45);
    std::printf("  ring: %.4f at 340, %.4f at 540, %.4f at 440\n", tone(x, 340.0), tone(x, 540.0), tone(x, 440.0));
    CHECK(tone(x, 340.0) > 10.0 * tone(x, 440.0));
    CHECK(tone(x, 540.0) > 10.0 * tone(x, 440.0));
    // Depth 50 %: the carrier stays, the sidebands too.
    c[cstype::kRmDepth] = 0.5f;
    const Buf y = slice(play(c, 69.0f), 0.1, 0.45);
    CHECK(tone(y, 440.0) > 1.5 * tone(y, 340.0));   // x (0.5 + 0.5 sin): carrier 2x each sideband
    CHECK(tone(y, 340.0) > 0.05 * tone(y, 440.0));
    // Sweep 100 % over RM Decay 0.3 s: the rate starts 3 octaves up (800 Hz).
    c[cstype::kRmDepth] = 1.0f;
    c[cstype::kRmSweep] = 1.0f;
    c[cstype::kRmDecay] = 0.3f;
    const Buf z = play(c, 69.0f, 1.0f);
    const Buf z0 = slice(z, 0.0, 0.04), z1 = slice(z, 0.7, 0.95);
    CHECK(tone(z0, 1240.0) > 3.0 * tone(z0, 540.0));
    CHECK(tone(z1, 540.0) > 3.0 * tone(z1, 1240.0));
}

TEST(cstype_sub_osc_lfo) {
    // -> VCO 100 %: +-1 semitone at Speed; Square sits at the extremes.
    for (int fn : {int(cstype::kFnSine), int(cstype::kFnSquare), int(cstype::kFnNoise)}) {
        std::vector<float> c = only(0, 0);
        c[cstype::kFunction] = float(fn);
        c[cstype::kSpeed] = 4.0f;
        c[cstype::kToVco] = 1.0f;
        const Buf x = play(c, 57.0f, 2.0f);
        std::vector<double> s;
        for (double f : freqTrack(slice(x, 0.2, 2.0))) s.push_back(12.0 * std::log2(f / hz(57.0f)));
        double lo = 1e9, hi = -1e9;
        int ext = 0;
        for (double v : s) lo = std::min(lo, v), hi = std::max(hi, v), ext += std::fabs(v) > 0.8;
        std::printf("  sub-osc %d -> VCO: %.2f .. %.2f semitones\n", fn, lo, hi);
        CHECK(hi < 1.2 && lo > -1.2);
        if (fn != cstype::kFnNoise) CHECK(hi > 0.85 && lo < -0.85);
        else CHECK(hi - lo > 0.3);
        if (fn == cstype::kFnSquare) CHECK(double(ext) / double(s.size()) > 0.75);
    }
    // -> VCA: a tremolo (100 %: down to silence at the trough).
    std::vector<float> c = only(0, 0);
    c[cstype::kToVca] = 1.0f;
    c[cstype::kSpeed] = 4.0f;
    const Buf x = play(c, 45.0f, 2.0f);
    double lo = 1e9, hi = 0;
    for (double t = 0.2; t + 0.02 < 2.0; t += 0.02) {
        const double r = rms(x, size_t(t * kFs), size_t((t + 0.02) * kFs));
        lo = std::min(lo, r);
        hi = std::max(hi, r);
    }
    CHECK(lo < 0.2 * hi);
    // -> VCF: the brightness moves.
    std::vector<float> f = only(0, 0);
    f[cstype::kLpfCutoff] = 800.0f;
    f[cstype::kToVcf] = 1.0f;
    f[cstype::kSpeed] = 2.0f;
    f[cstype::kFunction] = float(cstype::kFnSquare);
    const Buf y = play(f, 45.0f, 2.0f);
    double blo = 1e9, bhi = 0;
    for (double t = 0.2; t + 0.05 < 2.0; t += 0.05) {
        const double v = bright(slice(y, t, t + 0.05));
        blo = std::min(blo, v);
        bhi = std::max(bhi, v);
    }
    CHECK(bhi > 2.0 * blo);
}

TEST(cstype_brilliance_and_resonance) {
    std::vector<float> c = only(0, 0);
    c[cstype::kLpfCutoff] = 800.0f;
    const double mid = bright(slice(play(c, 45.0f), 0.1, 0.45));
    c[cstype::kBrilliance] = 1.0f;
    const double up = bright(slice(play(c, 45.0f), 0.1, 0.45));
    c[cstype::kBrilliance] = -1.0f;
    const double down = bright(slice(play(c, 45.0f), 0.1, 0.45));
    std::printf("  Brilliance: %.3f / %.3f / %.3f\n", down, mid, up);
    CHECK(up > 1.3 * mid && mid > 1.3 * down);
    // Resonance +100 %: +0.5 on every Res: the whistle from Res 50 %.
    std::vector<float> w = whistle(440.0f);
    w[cstype::kLpfRes] = 0.5f;
    const Buf quiet = play(w, 60.0f, 1.0f);
    w[cstype::kResonance] = 1.0f;
    const Buf sing = play(w, 60.0f, 1.0f);
    CHECK(rms(sing, size_t(0.5 * kFs)) > 20.0 * rms(quiet, size_t(0.5 * kFs)));
    CHECK_NEAR(pitchHz(sing, size_t(0.5 * kFs), sing.size()), 440.0, 440.0 * 0.03);
}

TEST(cstype_touch_velocity_and_ribbon) {
    // After -> VCF 100 % at Touch 100 %: +2 octaves (the whistle).
    std::vector<float> w = whistle(330.0f);
    w[cstype::kAfterVcf] = 1.0f;
    const double f0 = pitchHz(play(w, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    w[cstype::kTouch] = 1.0f;
    const double f1 = pitchHz(play(w, 60.0f, 1.0f), size_t(0.5 * kFs), size_t(1.0 * kFs));
    std::printf("  Touch -> VCF: %.1f -> %.1f Hz\n", f0, f1);
    CHECK_NEAR(f1 / f0, 4.0, 0.2);
    // After -> VCA 100 % at full Touch: +6 dB (x2).
    std::vector<float> c = only(0, 0);
    const double r0 = rms(play(c, 45.0f), kA, kB);
    c[cstype::kAfterVca] = 1.0f;
    c[cstype::kTouch] = 1.0f;
    CHECK_NEAR(rms(play(c, 45.0f), kA, kB) / r0, 2.0, 0.1);
    // After -> VCO: a vibrato only with Touch (+-0.5 semitone at 100 %).
    std::vector<float> v = only(0, 0);
    v[cstype::kAfterVco] = 1.0f;
    auto swing = [](const Buf& x) {
        double lo = 1e9, hi = -1e9;
        for (double f : freqTrack(slice(x, 0.2, 1.5))) lo = std::min(lo, f), hi = std::max(hi, f);
        return 12.0 * std::log2(hi / lo);
    };
    CHECK(swing(play(v, 57.0f, 1.5f)) < 0.1);
    v[cstype::kTouch] = 1.0f;
    CHECK_NEAR(swing(play(v, 57.0f, 1.5f)), 1.0, 0.25);
    // Velocity: Init -> Level scales the VCAs; Init -> Bril the attack levels.
    std::vector<float> l = only(0, 0);
    l[cstype::kInitLevelVel] = 1.0f;
    CHECK_NEAR(rms(play(l, 45.0f, 0.5f, 0.5f), kA, kB) / rms(play(l, 45.0f, 0.5f, 1.0f), kA, kB), 0.5, 0.03);
    std::vector<float> b = whistle(330.0f);
    b[cstype::kInitLevel] = 0.25f;
    b[cstype::kAttackLevel] = 0.25f;   // IL = AL: a held +1 octave scaled by velocity? No: AL only
    b[cstype::kInitBril] = 1.0f;
    b[cstype::kInitLevel] = 0.0f;
    b[cstype::kFAttack] = 0.3f;
    b[cstype::kFDecay] = 5.0f;
    const double hard = freqAt(play(b, 60.0f, 0.5f, 1.0f), 0.32), soft = freqAt(play(b, 60.0f, 0.5f, 0.0f), 0.32);
    std::printf("  Init -> Bril: %.1f Hz hard, %.1f Hz soft\n", hard, soft);
    CHECK(hard > 1.7 * soft);
    // Ribbon: +-Range semitones.
    std::vector<float> r = only(0, 0);
    r[cstype::kRibbon] = 1.0f;
    CHECK_NEAR(pitchHz(play(r, 57.0f), kA, kB), hz(59.0f), hz(59.0f) * 0.003);
    r[cstype::kRibbon] = -0.5f;
    r[cstype::kRibbonRange] = 12.0f;
    CHECK_NEAR(pitchHz(play(r, 69.0f), kA, kB), hz(63.0f), hz(63.0f) * 0.003);
}

TEST(cstype_glide) {
    std::vector<float> c = only(0, 0);
    c[cstype::kGlideTime] = 0.3f;
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
    const double want = hz(45.0f) * std::pow(2.0, 0.95);
    for (size_t i = 0; i < f.size(); ++i)
        if (f[i] >= want) {
            t95 = at[i];
            break;
        }
    std::printf("  glide 0.3 s: 95 %% after %.3f s\n", t95);
    CHECK(t95 > 0.22 && t95 < 0.4);
    CHECK_NEAR(pitchHz(x, size_t(0.7 * kFs), size_t(1.0 * kFs)), hz(57.0f), hz(57.0f) * 0.005);
    // Legato: no new attack on an overlapping note.
    c = open();
    c[cstype::kDecay] = 0.1f;
    c[cstype::kSustain] = 0.2f;
    for (int mode : {int(cstype::kLegato), int(cstype::kAlways)}) {
        c[cstype::kGlideMode] = float(mode);
        Rig g(c);
        g.on(0, 45.0f);
        g.run(kBlocksPerSec / 2);
        const double s = rms(g.L, g.L.size() - 10 * kBlock, g.L.size());
        g.on(0, 52.0f, 1.0f, true, true);
        const size_t t1 = g.L.size();
        g.run(kBlocksPerSec / 10);
        const double after = rms(g.L, t1 + kBlock, t1 + 6 * kBlock) / s;
        if (mode == cstype::kLegato) CHECK(after < 1.3);
        else CHECK(after > 2.0);
    }
}

TEST(cstype_polyphony_through_the_bank) {
    BankSlot s(cstype::kEngineId, 4);
    const int chord[4] = {48, 55, 60, 64};
    for (int k = 0; k < 4; ++k) s.on(k, chord[k]);
    s.run(kBlocksPerSec / 2);
    CHECK_EQ(s.bank.voicesOf(0), 4);
    const Buf x = s.tail(0.3);
    for (int k = 0; k < 4; ++k) CHECK(tone(x, hz(float(chord[k]))) > 0.003);
    for (int k = 0; k < 4; ++k) s.off(k);
    s.run(3 * kBlocksPerSec);
    CHECK_EQ(s.bank.voicesOf(0), 0);
    // Repeated notes on one track at Poly 1 restart the envelopes (Glide Mode Always).
    std::vector<float> c = initValues();
    c[cstype::kDecay] = 0.1f;
    c[cstype::kSustain] = 0.2f;
    c[kII + cstype::kLevel] = 0.0f;
    BankSlot m(cstype::kEngineId, 1, &c);
    double first = 0, last = 0;
    for (int k = 0; k < 4; ++k) {
        const size_t t0 = m.out.size();
        m.on(0, 45);
        m.run(kBlocksPerSec / 4);
        const double hit = rms(m.out, t0, t0 + 10 * kBlock);
        if (k == 0) first = hit;
        last = hit;
    }
    CHECK(last > 0.8 * first);
}

TEST(cstype_parameter_jumps_do_not_click) {
    struct Jump {
        int param;
        float a, b;
    };
    const Jump jumps[] = {{cstype::kLpfRes, 0.1f, 0.9f},  {cstype::kLpfCutoff, 300.0f, 5000.0f}, {cstype::kLevel, 0.1f, 1.0f},
                          {cstype::kBrilliance, -1.0f, 1.0f}, {cstype::kResonance, -1.0f, 1.0f}, {cstype::kSine, 0.0f, 1.0f},
                          {cstype::kHpfCutoff, 20.0f, 400.0f}, {cstype::kTouch, 0.0f, 1.0f},     {cstype::kRmDepth, 0.0f, 1.0f},
                          {cstype::kPw, 0.0f, 0.9f}};
    for (const Jump& j : jumps) {
        std::vector<float> c = open();
        c[cstype::kLpfCutoff] = 600.0f;
        c[cstype::kLevel] = 0.4f;
        c[cstype::kPulse] = j.param == cstype::kPw ? 1.0f : 0.0f;
        c[cstype::kSaw] = j.param == cstype::kPw ? 0.0f : 1.0f;
        c[cstype::kAfterVcf] = 1.0f;
        c[cstype::kAfterVca] = 0.5f;
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

// Two voices at unity gain, two layers each clipped at 1, the VCA up to
// +6 dB with Touch: the bound of the random sets.
constexpr float kBound = 2.0f * 2.0f * 0.8f * 2.0f;
TEST(cstype_no_nan_or_denormals) {
    uint32_t seed = 83;
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
        r.run(kBlocksPerSec);
        r.on(1, float(int(rnd() * 127.0f)), rnd(), true, rnd() < 0.5f);
        r.run(kBlocksPerSec / 4);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec / 4);
        if (!finite(r.L) || peak(r.L) > kBound) {
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
        a[cstype::kLevel] = 1.0f;
        Rig r(a);
        r.on(0, 127.0f);
        r.on(1, 0.0f);
        r.run(kBlocksPerSec);
        r.off(0);
        r.off(1);
        r.run(kBlocksPerSec);
        CHECK(finite(r.L));
        CHECK(peak(r.L) < kBound);
        CHECK_EQ(subnormals(r.L), 0L);
    }
    std::vector<float> c = initValues();
    c[cstype::kRelease] = 10.0f;
    c[kII + cstype::kRelease] = 10.0f;
    c[cstype::kSustain] = 1.0f;
    Rig t(c);
    t.on(0, 40.0f);
    t.run(50);
    t.off(0);
    int blocks = 0;
    while (t.run(1) && blocks < 60 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 60 * kBlocksPerSec);
    CHECK_EQ(subnormals(t.L), 0L);
    Rig q(whistle(20.0f));
    q.on(0, 0.0f);
    q.run(5 * kBlocksPerSec);
    CHECK(finite(q.L));
    CHECK_EQ(subnormals(q.L), 0L);
    std::vector<float> m = richContext();
    Rig f(m);
    f.on(0, 96.0f);
    for (int b = 0; b < 3 * kBlocksPerSec; ++b) {
        m[cstype::kLpfCutoff] = 20.0f + rnd() * 19980.0f;
        m[cstype::kLpfRes] = rnd() < 0.5f ? 1.0f : 0.0f;
        m[cstype::kResonance] = rnd() < 0.5f ? 1.0f : -1.0f;
        m[cstype::kBrilliance] = rnd() * 2.0f - 1.0f;
        f.set(m);
        f.run(1);
    }
    CHECK(finite(f.L));
    CHECK(peak(f.L) < kBound);
    CHECK_EQ(subnormals(f.L), 0L);
}

TEST(cstype_voice_level) {
    std::vector<float> c = open();
    c[cstype::kAttack] = 0.5f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(10);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    const float lv = r.v[0].level;
    r.on(0, 62.0f, 1.0f, true, false);
    CHECK_EQ(r.v[0].level, lv);
}

TEST(cstype_volume_is_never_applied) {
    std::vector<float> a = initValues(), b = initValues();
    a[cstype::kVolume] = 0.0f;
    b[cstype::kVolume] = 2.0f;
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

TEST(cstype_level_matches_the_synth_conventions) {
    long c1 = 0, c4 = 0;
    const float p1 = chordPeak(cstype::kEngineId, 1, c1);
    const float p4 = chordPeak(cstype::kEngineId, 4, c4);
#if defined(OT_MOD_ACD1)
    long ca = 0;
    const float pa = chordPeak(kSynthEngineAcd, 1, ca);
    std::printf("  CS-Type Init at -9 dB: 1 voice peak %.1f dBFS (ACD %.1f), 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(pa),
                dbfs(p4));
    CHECK(std::fabs(dbfs(p1) - dbfs(pa)) < 1.5);
#else
    std::printf("  CS-Type Init at -9 dB: 1 voice peak %.1f dBFS, 4-voice chords %.1f dBFS\n", dbfs(p1), dbfs(p4));
#endif
    CHECK(dbfs(p1) < -6.0);
    CHECK(dbfs(p1) > -20.0);
    CHECK(dbfs(p4) < -1.0);
    CHECK_EQ(c1, 0L);
    CHECK_EQ(c4, 0L);
}

TEST(cstype_every_parameter_has_an_effect) {
    const std::vector<float> ctx = richContext();
    for (int i = 0; i < kN; ++i) {
        if (i == cstype::kVolume) continue;
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
// engine alone and Init chords through a SynthBank slot. Any change of the
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
    Rig r(c);
    r.run(3);
    r.on(0, 45.0f, 0.6f);
    r.on(1, 52.0f, 0.9f);
    r.run(kBlocksPerSec / 3);
    c[cstype::kLpfRes] = 1.0f;
    c[cstype::kFunction] = float(cstype::kFnNoise);
    r.set(c);
    r.on(0, 57.0f, 0.9f, true, true);
    r.run(kBlocksPerSec / 3);
    r.off(0);
    r.off(1);
    r.run(kBlocksPerSec / 2);
    return hashOf(r.L, r.R);
}
uint64_t bankHash() {
    BankSlot s(cstype::kEngineId, 4);
    const int chords[2][4] = {{48, 55, 60, 64}, {50, 57, 62, 65}};
    for (const auto& ch : chords) {
        for (int k = 0; k < 4; ++k) s.on(k, ch[k]);
        s.run(120);
        for (int k = 0; k < 4; ++k) s.off(k);
        s.run(60);
    }
    s.run(200);
    return hashOf(s.out, s.out);
}
}  // namespace

TEST(cstype_golden_hashes) {
    const uint64_t e = engineHash(), b = bankHash();
    std::printf("  CS-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0x0dd3bda120e6eb27ull));
    CHECK_EQ(b, uint64_t(0x4613bf0d0dad7f63ull));
}

// --- Performance filter offset (issue #23) ---------------------------------------------
// SynthVoice::cutoffOct (octaves, filled by the framework from the track's
// Performance filter) moves the CS-Type cutoff: Cutoff x 2^offset, inside the
// engine's own clamps. A patch at 1600 Hz shifted by -2 octaves sounds like
// the same patch at 400 Hz; 0 leaves the render bit-identical.
// The offset moves the LPF of both layers; the HPF stays where the patch puts
// it (it runs only when the patch raises it).

namespace {
namespace perfcut {

// One note (pitch 48, 40 blocks) from the Init patch with its cutoff set by
// `cut(patch, hz)`, the voice's cutoffOct = `oct`.
template <class Cut>
std::vector<float> render(Cut cut, float hz, float oct) {
    const int32_t engine = cstype::kEngineId;
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
for (int o : {0, int(cstype::kLayer2)}) CHECK(synthParamSet(p, o + cstype::kLpfCutoff, hz));
}

}  // namespace perfcut
}  // namespace

TEST(cstype_perf_cutoff_offset_moves_the_cutoff) {
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
