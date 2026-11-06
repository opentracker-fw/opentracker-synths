// Hand-Type chip-style engine (opentracker-synths modules/hand, engine 0x87):
// registered with a valid descriptor ("Hand-Type" on screen,
// Patches/HANDTYPE/), silent when idle; the Pulse channel's four duties, the
// sweep (direction, rate per Shift and Time, Sweep Cut ending the voice) and
// the second pulse's beating; the Wave channel's twelve tables step for step,
// the User table's expansion, the 4-level volume; the Noise channel's
// register periods (32,767 / 127 shifts), its clock and Track; the 4-bit
// envelope's step times, Up / Down, Release Cut / Fade and Length; the DAC's
// level counts; the Drums note map (groups, wraps), the hat choke, the clap's
// bursts and the kick's sweep; the CHIP tricks (arp at 60 Hz and at 1/32 of
// the tempo, stepped vibrato, slide) and glide; through a SynthBank slot at
// Poly 1 and a kit at Poly 6; no click on parameter jumps, no NaN or
// denormals, Volume never applied, the slot level, every parameter audible,
// golden hashes.
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
#include "formats/patch_pb.h"
#include "hand.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"

using namespace ot;
using namespace ot::audio;
namespace chip = ot::audio::chip;

namespace {

constexpr float kFs = ot::audio::kSampleRate;
constexpr int kN = hand::kParamCount;
constexpr int kBlocksPerSec = int(kFs / float(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979;

const PatchParamRange* range(int i) { return synthParamRange(hand::kEngineId, i); }

std::vector<float> initValues() {
    mod::ModuleHost::instance().init();
    std::vector<float> v(kN);
    for (int i = 0; i < kN; ++i) v[i] = range(i)->def;
    return v;
}

using Buf = std::vector<float>;

// The engine's voices driven directly (no framework).
struct Rig {
    ISynthEngine* e = synthEngineFor(hand::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    Buf L, R;
    float bpm = 120.0f;

    explicit Rig(const std::vector<float>& vals) {
        p.engine = hand::kEngineId;
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
// Sign changes per second (noise clocks).
double flipsPerSec(const Buf& x, size_t a, size_t b) {
    long n = 0;
    for (size_t i = a + 1; i < b; ++i) n += (x[i] > 0.0f) != (x[i - 1] > 0.0f);
    return double(n) * kFs / double(b - a);
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
size_t distinct(const Buf& x, size_t a, size_t b) {
    std::set<float> s(x.begin() + long(a), x.begin() + long(std::min(b, x.size())));
    return s.size();
}

// The phase starts at 0 with the note: frame i plays step int(32 frac((i + 1)
// dt)). Fits y = a t + b (a > 0) over the frames of three cycles away from a
// step boundary; returns the worst residual in levels (1e9 if a <= 0).
double waveFit(const Buf& x, const uint8_t* t, float pitch) {
    const double dt = hz(pitch) / kFs, per = 1.0 / dt;
    std::vector<double> tv, yv;
    for (size_t i = 0; i < size_t(3 * per); ++i) {
        const double ph = std::fmod(double(i + 1) * dt, 1.0) * 32.0;
        if (ph - std::floor(ph) < 0.05 || ph - std::floor(ph) > 0.95) continue;
        tv.push_back(t[int(ph) & 31]);
        yv.push_back(x[i]);
    }
    double st = 0, sv = 0, stt = 0, stv = 0;
    const double nn = double(tv.size());
    for (size_t k = 0; k < tv.size(); ++k) st += tv[k], sv += yv[k], stt += tv[k] * tv[k], stv += tv[k] * yv[k];
    const double a = (nn * stv - st * sv) / (nn * stt - st * st), b = (sv - a * st) / nn;
    if (!(a > 0.0)) return 1e9;
    double worst = 0;
    for (size_t k = 0; k < tv.size(); ++k) worst = std::max(worst, std::fabs(yv[k] - (a * tv[k] + b)));
    return worst / a;
}

// A held, clean channel: the envelope at 15 and staying there (Dir Up), the
// DAC clean (Bits 8), no tricks.
std::vector<float> clean(int channel = hand::kPulse) {
    std::vector<float> c = initValues();
    c[hand::kChannel] = float(channel);
    c[hand::kEnvInit] = 15.0f;
    c[hand::kEnvDir] = float(hand::kUp);
    c[hand::kBits] = 8.0f;
    c[hand::kSweepDir] = float(hand::kSweepOff);
    return c;
}
Buf play(const std::vector<float>& vals, float pitch = 57.0f, float seconds = 0.5f) {
    Rig r(vals);
    r.on(0, pitch);
    r.run(int(seconds * kFs / float(kBlock)) + 1);
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
    BankSlot(int poly, const std::vector<float>* vals = nullptr, int32_t engine = hand::kEngineId) {
        mod::ModuleHost::instance().init();
        gov.setBudget(8);
        bank.setGovernor(&gov);
        bank.setStartsPerBlock(0);
        patchInit(engine, &patch);
        if (vals)
            for (int i = 0; i < kN; ++i) synthParamSet(patch, i, (*vals)[size_t(i)]);
        bank.bindPatch(0, &patch);
        SynthSlotRecord r = SynthSlotRecord::defaults(0);
        r.model = uint32_t(engine == kSynthEngineAcd ? SynthModel::ACD : SynthModel::FAT);
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

TEST(hand_descriptor_is_valid) {
    mod::ModuleHost::instance().init();
    ISynthEngine* e = synthEngineFor(hand::kEngineId);
    CHECK(e != nullptr);
    if (!e) return;
    CHECK_STR(e->name(), "HANDTYPE");
    const mod::SynthDescriptor* d = mod::ModuleHost::instance().synthDescriptor(hand::kEngineId);
    CHECK(d != nullptr);
    if (!d) return;
    CHECK_EQ(d->engineId, int32_t(hand::kEngineId));
    CHECK_STR(d->name, "HANDTYPE");
    CHECK_STR(d->displayName, "Hand-Type");
    CHECK_EQ(int(d->paramCount), 52);
    CHECK_EQ(int(d->paramCount), kN);
    CHECK_EQ(int(d->volumeParam), int(hand::kVolume));
    CHECK_EQ(int(d->flags), 0);
    CHECK_EQ(patchVolumeParam(hand::kEngineId), int(hand::kVolume));
    CHECK_EQ(int(d->sectionCount), 15);
    CHECK_EQ(int(d->extCount), 2);   // Width, Spread
    CHECK_STR(synthEngineFolder(hand::kEngineId), "HANDTYPE");
    CHECK_STR(synthEngineDisplayName(hand::kEngineId), "Hand-Type");
    const int k = synthModelListIndex(hand::kEngineId);
    CHECK(k >= kSynthModelCount);
    CHECK_STR(synthModelListName(k), "Hand-Type");
#if defined(OT_MOD_8TY1)
    CHECK(k > synthModelListIndex(0x86));
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
    CHECK(patchInit(hand::kEngineId, &p));
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].count), 1);
    CHECK_EQ(int(p.macros[kPatchMacroVelocity].targets[0].param), int(hand::kVolume));
}

TEST(hand_patch_file_round_trip) {
    mod::ModuleHost::instance().init();
    SynthPatch p;
    CHECK(patchInit(hand::kEngineId, &p));
    const float vals[][2] = {{hand::kChannel, 3},  {hand::kDuty, 0},     {hand::kP2Detune, -33.5f}, {hand::kUser5, 3},
                             {hand::kNoiseShift, 9}, {hand::kArpMode, 11}, {hand::kVibRate, 7.25f},   {hand::kSlideStart, -12},
                             {hand::kBits, 6}};
    for (const auto& v : vals) {
        CHECK(patchParamInRange(hand::kEngineId, int(v[0]), v[1]));
        CHECK(synthParamSet(p, int(v[0]), v[1]));
    }
    std::vector<uint8_t> a(kPatchMaxFileSize), b(kPatchMaxFileSize);
    size_t na = 0, nb = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(q.engine, int32_t(hand::kEngineId));
    CHECK_EQ(int(q.paramCount), kN);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (const auto& v : vals) CHECK_EQ(q.params[int(v[0])].bits, patchFloatToBits(v[1]));
    CHECK(encodePatch(q, b.data(), b.size(), &nb) == PatchStatus::kOk);
    CHECK(na == nb && std::memcmp(a.data(), b.data(), na) == 0);
    CHECK(!patchParamInRange(hand::kEngineId, hand::kChannel, 4.0f));
    CHECK(!patchParamInRange(hand::kEngineId, hand::kBits, 3.0f));
}

TEST(hand_silent_when_idle) {
    Rig r(initValues());
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
    float l[kBlock] = {}, rr[kBlock] = {};
    r.v[3].gate = false;
    CHECK(!r.e->render(r.v[3], r.p, 1.0f, 0.0f, l, rr));
    CHECK_EQ(peak(Buf(l, l + kBlock)), 0.0f);
    // Init: the envelope falls to 0 by itself and the voice ends; exact silence after it.
    r.on(0, 60.0f);
    r.run(kBlocksPerSec / 4);
    CHECK(rms(r.L) > 0.01);
    int blocks = 0;
    while (r.run(1) && blocks < 4 * kBlocksPerSec) ++blocks;
    CHECK(blocks < 4 * kBlocksPerSec);
    r.L.clear();
    r.run(20);
    CHECK_EQ(peak(r.L), 0.0f);
}

TEST(hand_pulse_duties) {
    const double want[4] = {0.125, 0.25, 0.5, 0.75};
    for (int d = 0; d < 4; ++d) {
        std::vector<float> c = clean();
        c[hand::kDuty] = float(d);
        const Buf x = play(c, 33.0f, 0.4f);   // 55 Hz: ~800 frames per cycle
        // The time above the midpoint between the two levels.
        const size_t a = size_t(0.1 * kFs), b = size_t(0.35 * kFs);
        float lo = 1e9f, hi = -1e9f;
        for (size_t i = a; i < b; ++i) lo = std::min(lo, x[i]), hi = std::max(hi, x[i]);
        long up = 0;
        for (size_t i = a; i < b; ++i) up += x[i] > 0.5f * (lo + hi);
        const double duty = double(up) / double(b - a);
        std::printf("  duty %d: %.4f (want %.4f)\n", d, duty, want[d]);
        CHECK(std::fabs(duty - want[d]) < 0.02);
        CHECK(std::fabs(freqOf(x, a, b) - hz(33.0f)) < 0.5);
    }
}

TEST(hand_sweep_direction_rate_and_cut) {
    // Down: the pitch falls, faster with a smaller Shift and a shorter Time.
    auto drop = [](int dir, int time, int shift, float seconds) {
        std::vector<float> c = clean();
        c[hand::kSweepDir] = float(dir);
        c[hand::kSweepTime] = float(time);
        c[hand::kSweepShift] = float(shift);
        c[hand::kSweepCut] = 0.0f;
        const Buf x = play(c, 72.0f, seconds + 0.05f);
        const size_t e = size_t(seconds * kFs);
        return semis(freqOf(x, e - 600, e), hz(72.0f));
    };
    // Shift 4, Time 2: a step of 12 log2(1 + 1/16) = 1.05 st every 2/128 s.
    const double d1 = drop(hand::kSweepDown, 2, 4, 0.1f);
    const double want = -12.0 * std::log2(1.0 + 1.0 / 16.0) * std::floor(0.1 * 128.0 / 2.0);
    std::printf("  sweep down (time 2, shift 4) after 0.1 s: %.2f st (want %.2f)\n", d1, want);
    CHECK(std::fabs(d1 - want) < 1.2);
    CHECK(drop(hand::kSweepDown, 2, 2, 0.1f) < d1 - 3.0);   // smaller shift: steeper
    CHECK(drop(hand::kSweepDown, 6, 4, 0.1f) > d1 + 3.0);   // longer time: slower
    const double u = drop(hand::kSweepUp, 3, 5, 0.1f);
    std::printf("  sweep up (time 3, shift 5) after 0.1 s: %+.2f st\n", u);
    CHECK(u > 2.0);
    // Sweep Cut: a fast sweep down ends the voice (gate held, envelope at 15).
    std::vector<float> c = clean();
    c[hand::kSweepDir] = float(hand::kSweepDown);
    c[hand::kSweepTime] = 1.0f;
    c[hand::kSweepShift] = 1.0f;
    c[hand::kSweepCut] = 1.0f;
    Rig r(c);
    r.on(0, 60.0f);
    int blocks = 0;
    while (r.run(1) && blocks < kBlocksPerSec) ++blocks;
    std::printf("  sweep cut: voice ends after %d blocks\n", blocks);
    CHECK(blocks < kBlocksPerSec / 4);
    c[hand::kSweepCut] = 0.0f;
    Rig h(c);
    h.on(0, 60.0f);
    CHECK_EQ(h.run(kBlocksPerSec / 2), 1);
}

TEST(hand_pulse2_detune_beats) {
    std::vector<float> c = clean();
    c[hand::kP2Level] = 1.0f;
    c[hand::kP2Detune] = 20.0f;
    c[hand::kP2Duty] = float(hand::kDuty50);
    const Buf x = play(c, 57.0f, 1.2f);
    // The beat: the fundamental's envelope swings at 220 x (2^(20/1200) - 1) = 2.55 Hz.
    const double beat = 220.0 * (std::pow(2.0, 20.0 / 1200.0) - 1.0);
    std::vector<double> lv;
    for (int k = 0; k < 20; ++k) {
        const size_t a = size_t((0.1 + 0.05 * k) * kFs);
        lv.push_back(tone(Buf(x.begin() + long(a), x.begin() + long(a + 2048)), 220.0));
    }
    const double mx = *std::max_element(lv.begin(), lv.end()), mn = *std::min_element(lv.begin(), lv.end());
    std::printf("  pulse 2 +20 ct: fundamental swings %.4f .. %.4f (beat %.2f Hz)\n", mn, mx, beat);
    CHECK(mn < 0.5 * mx);
    c[hand::kP2Level] = 0.0f;
    const Buf y = play(c, 57.0f, 1.2f);
    CHECK(rmsDiff(x, y) > 0.01);
}

TEST(hand_wave_tables_user_and_levels) {
    // A slow wave (one step = ~86 frames): each table's 32 steps, in order.
    for (int w = 0; w < hand::kUser; ++w) {
        std::vector<float> c = clean(hand::kWaveCh);
        c[hand::kWave] = float(w);
        const float pitch = 21.0f;   // 27.5 Hz: 1,604 frames per cycle
        const Buf x = play(c, pitch, 0.3f);
        const uint8_t* t = hand::waveTable(w);
        const double err = waveFit(x, t, pitch);
        CHECK(err < 1e-3);
        if (!(err < 1e-3)) std::printf("       wave %d: worst %.5f of a level\n", w, err);
        const double per = kFs / hz(pitch);
        const size_t base = size_t(0.1 * kFs);
        // 4-bit steps: at most 16 distinct levels in a cycle.
        CHECK(distinct(x, base, base + size_t(per)) <= 16);
    }
    // User: 8 points expanded to 32 by linear interpolation, requantised.
    std::vector<float> c = clean(hand::kWaveCh);
    c[hand::kWave] = float(hand::kUser);
    const float pts[8] = {0, 15, 0, 15, 8, 8, 4, 12};
    for (int k = 0; k < 8; ++k) c[size_t(hand::kUser1 + k)] = pts[k];
    const Buf u = play(c, 21.0f, 0.3f);
    CHECK(rms(u) > 0.05);
    uint8_t want[32];
    for (int s2 = 0; s2 < 32; ++s2) {
        const float a = pts[s2 >> 2], b = pts[((s2 >> 2) + 1) & 7];
        want[s2] = uint8_t(a + (b - a) * float(s2 & 3) * 0.25f + 0.5f);
    }
    CHECK(waveFit(u, want, 21.0f) < 1e-3);
    const Buf tri = play(clean(hand::kWaveCh), 21.0f, 0.3f);
    CHECK(rmsDiff(u, tri) > 0.02);
    // Levels 4: the envelope's 16 levels collapse to 100 / 50 / 25 / 0 %.
    std::vector<float> f = clean(hand::kWaveCh);
    f[hand::kWave] = float(hand::kSquare50);
    f[hand::kEnvInit] = 15.0f;
    f[hand::kEnvDir] = float(hand::kDown);
    f[hand::kEnvStep] = 0.0f;
    const Buf a16 = play(f, 45.0f, 0.3f);
    f[hand::kLevels] = float(hand::kLevels4);
    const Buf a4 = play(f, 45.0f, 0.3f);
    std::set<int> amps16, amps4;
    for (int k = 0; k < 13; ++k) {
        const size_t a = size_t(k * 689 + 300);
        amps16.insert(int(std::lround(peak(a16, a, a + 300) * 1000)));
        amps4.insert(int(std::lround(peak(a4, a, a + 300) * 1000)));
    }
    std::printf("  wave levels: %zu amplitudes at Levels 16, %zu at Levels 4\n", amps16.size(), amps4.size());
    CHECK(amps16.size() >= 10);
    CHECK(amps4.size() <= 4);
}

TEST(hand_noise_periods_clock_and_track) {
    // Shift 3, Divider 0: one shift per frame: the 15-bit register repeats
    // every 32,767 frames, the 7-bit one every 127.
    for (int w = 0; w < 2; ++w) {
        std::vector<float> c = clean(hand::kNoise);
        c[hand::kWidth] = float(w);
        c[hand::kNoiseShift] = 3.0f;
        c[hand::kDivider] = 0.0f;
        const Buf x = play(c, 60.0f, 1.8f);
        const size_t per = w == 0 ? 32767 : 127;
        const size_t a = 1000;
        long same = 0, diff = 0;
        for (size_t i = a; i < a + 20000; ++i) {
            same += x[i] == x[i + per];
            diff += x[i] != x[i + per - 1];
        }
        std::printf("  noise %s: %ld of 20000 repeat after %zu frames, %ld differ one frame earlier\n", w ? "7-bit" : "15-bit",
                    same, per, diff);
        CHECK_EQ(same, 20000L);
        CHECK(diff > 5000);
    }
    CHECK(std::fabs(hand::noiseHz(3, 0) - kFs) < 0.5f);
    CHECK(std::fabs(hand::noiseHz(6, 2) - kFs / 32.0f) < 0.5f);
    // The clock: about half the shifts flip the output.
    std::vector<float> c = clean(hand::kNoise);
    c[hand::kNoiseShift] = 7.0f;
    const Buf x = play(c, 60.0f, 1.0f);
    const double f = flipsPerSec(x, 1000, x.size());
    std::printf("  noise shift 7: %.0f flips/s (clock %.0f Hz)\n", f, double(hand::noiseHz(7, 0)));
    CHECK(std::fabs(f - 0.5 * hand::noiseHz(7, 0)) < 0.1 * hand::noiseHz(7, 0));
    // Track: the clock follows the note.
    c[hand::kTrack] = 1.0f;
    const double lo = flipsPerSec(play(c, 40.0f, 1.0f), 1000, 40000), hi = flipsPerSec(play(c, 64.0f, 1.0f), 1000, 40000);
    std::printf("  noise track: %.0f flips/s at note 40, %.0f at note 64\n", lo, hi);
    CHECK(hi > 2.5 * lo);
}

TEST(hand_envelope_steps_release_and_length) {
    // Down from 15 at Step 0 (1/64 s a level): 15 levels in 15/64 s, the voice ends.
    std::vector<float> c = clean();
    c[hand::kEnvDir] = float(hand::kDown);
    c[hand::kEnvStep] = 0.0f;
    c[hand::kWave] = float(hand::kSquare50);
    c[hand::kChannel] = float(hand::kWaveCh);
    Rig r(c);
    r.on(0, 45.0f);
    int blocks = 0;
    while (r.run(1) && blocks < kBlocksPerSec) ++blocks;
    const double t = double(blocks + 1) * kBlock / kFs;
    std::printf("  env down 15 levels at 16 ms: ends after %.4f s (want %.4f)\n", t, 15.0 / 64.0);
    CHECK(std::fabs(t - 15.0 / 64.0) < 1.5 * kBlock / kFs);
    CHECK(std::fabs(hand::envStepSeconds(3) - 4.0f / 64.0f) < 1e-6f);
    // The level halves after 7.5 steps (a linear 4-bit ramp).
    const size_t at = size_t(7.0 / 64.0 * kFs);
    CHECK(peak(r.L, at - 200, at) < 0.62f * peak(r.L, 0, 600));
    // Up from 0: rises to full in 15 steps and stays.
    c[hand::kEnvInit] = 0.0f;
    c[hand::kEnvDir] = float(hand::kUp);
    Rig u(c);
    u.on(0, 45.0f);
    u.run(kBlocksPerSec / 2);
    CHECK(peak(u.L, 0, 400) < 0.1f * peak(u.L, u.L.size() - 1000, u.L.size()));
    const size_t full = size_t(15.5 / 64.0 * kFs);
    CHECK(std::fabs(peak(u.L, full, full + 2000) - peak(u.L, u.L.size() - 2000, u.L.size())) < 1e-4f);
    // Release Cut: silent within two blocks; Fade: one level per step.
    for (int rel = 0; rel < 2; ++rel) {
        std::vector<float> d = clean();
        d[hand::kRelease] = float(rel);
        d[hand::kEnvStep] = 1.0f;   // 2/64 s a level
        Rig q(d);
        q.on(0, 57.0f);
        q.run(20);
        q.off(0);
        int n = 0;
        while (q.run(1) && n < kBlocksPerSec * 2) ++n;
        const double tr = double(n + 1) * kBlock / kFs;
        std::printf("  release %s: ends %.4f s after the note-off\n", rel ? "fade" : "cut", tr);
        if (rel == 0) CHECK(n <= 2);
        else CHECK(std::fabs(tr - 15.0 * 2.0 / 64.0) < 1.5 * kBlock / kFs);
    }
    // Length: the voice stops by itself after 1/16 s x Length, the gate held.
    std::vector<float> l = clean();
    l[hand::kLength] = 2.0f;
    Rig g(l);
    g.on(0, 57.0f);
    int n = 0;
    while (g.run(1) && n < kBlocksPerSec) ++n;
    const double tl = double(n + 1) * kBlock / kFs;
    std::printf("  length 2: ends after %.4f s\n", tl);
    CHECK(std::fabs(tl - 0.125) < 1.5 * kBlock / kFs);
}

TEST(hand_dac_bits) {
    for (int bits = 4; bits <= 8; ++bits) {
        std::vector<float> c = clean(hand::kWaveCh);
        c[hand::kWave] = float(hand::kSine);
        c[hand::kEnvDir] = float(hand::kDown);
        c[hand::kEnvStep] = 2.0f;
        c[hand::kBits] = float(bits);
        const Buf x = play(c, 52.0f, 0.6f);
        const size_t n = distinct(x, 0, x.size());
        std::printf("  bits %d: %zu levels\n", bits, n);
        // u e on 2^Bits - 1 steps less the rounded mean: at most 2^(Bits + 1) values.
        if (bits < 8) CHECK(n <= size_t(2u << bits));
        else CHECK(n > 100);   // clean: 16 table levels at every envelope level
    }
}

TEST(hand_drum_note_map) {
    int hat = -1;
    CHECK_EQ(int(hand::drumForNote(60)), int(hand::kKick));
    CHECK_EQ(int(hand::drumForNote(61)), int(hand::kBlip));
    CHECK_EQ(int(hand::drumForNote(62)), int(hand::kSnare));
    CHECK_EQ(int(hand::drumForNote(63)), int(hand::kClap));
    CHECK_EQ(int(hand::drumForNote(71)), int(hand::kClap));
    for (int n = 64; n <= 67; ++n) {
        CHECK_EQ(int(hand::drumForNote(n, &hat)), int(hand::kHat));
        CHECK_EQ(hat, n - 64);
    }
    CHECK_EQ(int(hand::drumForNote(68)), int(hand::kBell));
    CHECK_EQ(int(hand::drumForNote(69)), int(hand::kCrash));
    CHECK_EQ(int(hand::drumForNote(70)), int(hand::kRide));
    for (int n = 48; n <= 59; ++n) CHECK_EQ(int(hand::drumForNote(n)), int(hand::kTom));
    CHECK_EQ(int(hand::drumForNote(40)), int(hand::kTom));
    CHECK_EQ(int(hand::drumForNote(72)), int(hand::kKick));
    CHECK_EQ(int(hand::drumForNote(86)), int(hand::kSnare));
    CHECK_EQ(int(hand::drumForNote(127)), int(hand::kHat));   // 127 -> 67
    // Every drum sounds and ends by itself (the gate held).
    std::vector<float> c = initValues();
    c[hand::kChannel] = float(hand::kDrums);
    for (int n : {60, 61, 62, 63, 64, 67, 68, 69, 70, 48, 59, 40, 74}) {
        Rig r(c);
        r.on(0, float(n));
        int blocks = 0;
        while (r.run(1) && blocks < 4 * kBlocksPerSec) ++blocks;
        std::printf("  drum note %d: peak %.3f, %.3f s\n", n, peak(r.L), double(blocks) * kBlock / kFs);
        CHECK(peak(r.L) > 0.05f);
        CHECK(blocks < 4 * kBlocksPerSec);
    }
    // Toms are chromatic; the low tom below 48 is transposed.
    auto tomHz = [&c](int n) {
        Rig r(c);
        r.on(0, float(n));
        r.run(40);
        return freqOf(r.L, 2500, 5000);
    };
    const double t48 = tomHz(48), t55 = tomHz(55), t40 = tomHz(40);
    std::printf("  toms: 48 %.1f Hz, 55 %.1f Hz, 40 %.1f Hz\n", t48, t55, t40);
    CHECK(std::fabs(semis(t55, t48) - 7.0) < 0.6);
    CHECK(std::fabs(semis(t48, t40) - 8.0) < 0.6);
}

TEST(hand_drum_hat_choke_clap_and_kick) {
    std::vector<float> c = initValues();
    c[hand::kChannel] = float(hand::kDrums);
    c[hand::kHatOpenDecay] = 7.0f;
    // An open hat rings on; a closed hat on another voice cuts it.
    Rig r(c);
    r.on(0, 67.0f);
    r.run(20);
    CHECK(r.alive[0]);
    r.on(1, 64.0f);
    r.run(3);
    CHECK(!r.alive[0]);
    // Another drum does not choke the hat.
    Rig s(c);
    s.on(0, 67.0f);
    s.run(5);
    s.on(1, 60.0f);
    s.run(5);
    CHECK(s.alive[0]);
    // The clap: three bursts in its first 40 ms (the level dips between them).
    Rig k(c);
    k.on(0, 63.0f);
    k.run(20);
    std::vector<float> env;
    for (size_t a = 0; a + 88 < size_t(0.045 * kFs); a += 88) env.push_back(peak(k.L, a, a + 88));
    int bursts = 0;
    bool high = false;
    const float top = *std::max_element(env.begin(), env.end());
    for (float e : env) {
        if (!high && e > 0.6f * top) {
            ++bursts;
            high = true;
        } else if (high && e < 0.25f * top) {
            high = false;
        }
    }
    std::printf("  clap: %d bursts\n", bursts);
    CHECK(bursts >= 3);
    // The kick sweeps down.
    Rig q(c);
    q.on(0, 60.0f);
    q.run(40);
    const double f0 = freqOf(q.L, 0, 900), f1 = freqOf(q.L, 2500, 4500);
    std::printf("  kick: %.1f Hz -> %.1f Hz\n", f0, f1);
    CHECK(f1 < 0.7 * f0);
    // Kick Tune moves it.
    c[hand::kKickTune] = 12.0f;
    Rig t(c);
    t.on(0, 60.0f);
    t.run(40);
    CHECK(std::fabs(semis(freqOf(t.L, 0, 900), f0) - 12.0) < 1.5);
}

TEST(hand_arp_at_60hz_and_at_the_tempo) {
    for (int rate : {int(1), int(4)}) {   // 60 Hz, 1/32
        std::vector<float> c = clean();
        c[hand::kArpMode] = 2.0f;   // Maj
        c[hand::kArpRate] = float(rate);
        Rig r(c);
        r.bpm = 120.0f;
        r.on(0, 69.0f);
        const double per = rate == 1 ? kFs / 60.0 : kFs * 2.0 / 32.0;
        r.run(int(8.5 * per / kBlock) + 2);
        const int want[3] = {0, 4, 7};
        int bad = 0;
        for (int k = 0; k < 8; ++k) {
            const size_t a = size_t(k * per + 0.2 * per), b = size_t((k + 1) * per - 0.1 * per);
            const double st = semis(freqOf(r.L, a, b), hz(69.0f));
            if (std::fabs(st - want[k % 3]) > 0.2) {
                ++bad;
                std::printf("       rate %d tick %d: %.2f st\n", rate, k, st);
            }
        }
        CHECK_EQ(bad, 0);
        // The step lands on the tick (+-1 frame): the crossing interval that holds it.
        const std::vector<double> t = crossings(r.L);
        double worst = 0.0;
        for (int k = 1; k < 7; ++k) {
            const double tk = k * per;   // tick k (a frame at or after it)
            const double pOld = kFs / (hz(69.0f) * std::pow(2.0, want[(k - 1) % 3] / 12.0));
            const double pNew = kFs / (hz(69.0f) * std::pow(2.0, want[k % 3] / 12.0));
            for (size_t j = 1; j < t.size(); ++j)
                if (t[j - 1] <= tk && t[j] > tk) {
                    // One cycle between the crossings: (x - t0) / pOld + (t1 - x) / pNew = 1.
                    const double x = (1.0 - (t[j] - t[j - 1]) / pNew) / (1.0 / pOld - 1.0 / pNew) + t[j - 1];
                    worst = std::max(worst, std::fabs(x - std::ceil(tk)));
                    break;
                }
        }
        std::printf("  arp Maj at rate %d: steps within %.2f frames of the ticks\n", rate, worst);
        CHECK(worst <= 1.5);
    }
}

TEST(hand_vibrato_and_slide) {
    std::vector<float> c = clean();
    c[hand::kVibDepth] = 1.0f;
    c[hand::kVibRate] = 4.0f;
    c[hand::kVibDelay] = 0.0f;
    c[hand::kArpRate] = 3.0f;   // 120 Hz ticks
    const Buf x = play(c, 69.0f, 1.2f);
    double lo = 0, hi = 0;
    int cyc = 0;
    double prev = 0;
    for (size_t a = size_t(0.1 * kFs); a + 420 < size_t(1.1 * kFs); a += 147) {
        const double st = semis(freqOf(x, a, a + 420), 440.0);
        lo = std::min(lo, st);
        hi = std::max(hi, st);
        if (prev < -0.3 && st >= -0.3) ++cyc;
        prev = st;
    }
    std::printf("  vibrato 1 st at 4 Hz: %.2f .. %.2f st, %d cycles in 1 s\n", lo, hi, cyc);
    CHECK(std::fabs(hi - 1.0) < 0.25);
    CHECK(std::fabs(lo + 1.0) < 0.25);
    CHECK(cyc >= 3 && cyc <= 5);
    // Slide: +12 st at the start, the note after Slide Time.
    std::vector<float> s = clean();
    s[hand::kSlideStart] = 12.0f;
    s[hand::kSlideTime] = 0.2f;
    const Buf y = play(s, 57.0f, 0.4f);
    const double f0 = semis(freqOf(y, 0, 800), hz(57.0f)), f1 = semis(freqOf(y, size_t(0.1 * kFs) - 300, size_t(0.1 * kFs) + 300), hz(57.0f)),
                 f2 = semis(freqOf(y, size_t(0.25 * kFs), size_t(0.35 * kFs)), hz(57.0f));
    std::printf("  slide +12 in 0.2 s: %.2f, %.2f, %.2f st\n", f0, f1, f2);
    CHECK(f0 > 10.5);
    CHECK(std::fabs(f1 - 6.0) < 1.0);
    CHECK(std::fabs(f2) < 0.1);
}

TEST(hand_glide) {
    std::vector<float> c = clean();
    c[hand::kGlideTime] = 0.3f;
    Rig r(c);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec / 4);
    const size_t at = r.L.size();
    r.on(0, 69.0f, true, true);
    r.run(kBlocksPerSec / 2);
    const double mid = semis(freqOf(r.L, at + 2000, at + 4000), hz(57.0f)), end = semis(freqOf(r.L, r.L.size() - 3000, r.L.size()), hz(57.0f));
    std::printf("  glide 57 -> 69: %.2f st after ~70 ms, %.2f at the end\n", mid, end);
    CHECK(mid > 1.0 && mid < 11.0);
    CHECK(std::fabs(end - 12.0) < 0.15);
    // Glide 0: the new note at once.
    c[hand::kGlideTime] = 0.0f;
    Rig q(c);
    q.on(0, 57.0f);
    q.run(20);
    const size_t at2 = q.L.size();
    q.on(0, 69.0f, true, true);
    q.run(20);
    CHECK(std::fabs(semis(freqOf(q.L, at2 + 300, at2 + 2000), 440.0)) < 0.1);
}

TEST(hand_through_the_bank_poly1_and_a_kit_at_poly6) {
    std::vector<float> c = initValues();
    c[hand::kEnvDir] = float(hand::kUp);
    BankSlot s(1, &c);
    s.on(0, 57);
    s.run(kBlocksPerSec / 4);
    s.on(1, 64);
    s.run(kBlocksPerSec / 4);
    CHECK_EQ(s.bank.voicesOf(0), 1);
    CHECK(std::fabs(freqOf(s.out, s.out.size() - 3000, s.out.size()) - hz(64.0f)) < 2.0);
    c[hand::kChannel] = float(hand::kDrums);
    BankSlot k(6, &c);
    const int kit[5] = {60, 62, 64, 69, 52};
    for (int j = 0; j < 5; ++j) k.on(j, kit[j]);
    k.run(4);
    CHECK_EQ(k.bank.voicesOf(0), 5);
    k.run(kBlocksPerSec);
    CHECK(k.pk > 0.05f);
    CHECK_EQ(k.clips, 0L);
}

TEST(hand_parameter_jumps_do_not_click) {
    struct Jump {
        int param;
        float a, b;
        int channel;
    };
    const Jump jumps[] = {{hand::kDuty, 0, 3, hand::kPulse},     {hand::kP2Level, 0, 1, hand::kPulse},
                          {hand::kP2Duty, 0, 3, hand::kPulse},   {hand::kVolume, 0.2f, 2, hand::kPulse},
                          {hand::kEnvInit, 15, 3, hand::kPulse}, {hand::kUser3, 0, 15, hand::kWaveCh}};
    for (const Jump& j : jumps) {
        std::vector<float> c = clean(j.channel);
        c[hand::kWave] = float(hand::kUser);
        c[size_t(j.param)] = j.a;
        Rig r(c);
        r.on(0, 45.0f);
        r.run(kBlocksPerSec / 4);
        auto maxStep = [&r](size_t a, size_t b) {
            double m = 0;
            for (size_t i = a + 1; i < b; ++i) m = std::max(m, double(std::fabs(r.L[i] - r.L[i - 1])));
            return m;
        };
        // Chip waves step by design: the largest step of the steady wave is the bound.
        const double steady = maxStep(r.L.size() - 40 * kBlock, r.L.size());
        c[size_t(j.param)] = j.b;
        const size_t at = r.L.size();
        r.set(c);
        r.run(3);
        const double jump = maxStep(at - 1, at + 3 * kBlock);
        if (!(jump <= 1.05 * steady + 1e-4)) std::printf("       param %d: step %.4f (steady %.4f)\n", j.param, jump, steady);
        CHECK(jump <= 1.05 * steady + 1e-4);
    }
}

TEST(hand_no_nan_or_denormals) {
    uint32_t seed = 87;
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
        if (!finite(r.L) || peak(r.L) > 1.5f) {
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
        for (int ch = 0; ch < 4; ++ch) {
            a[hand::kChannel] = float(ch);
            Rig r(a);
            r.on(0, 127.0f);
            r.on(1, 0.0f);
            r.run(kBlocksPerSec);
            r.off(0);
            r.off(1);
            r.run(kBlocksPerSec);
            CHECK(finite(r.L));
            CHECK(peak(r.L) < 1.5f);
            CHECK_EQ(subnormals(r.L), 0L);
        }
    }
}

TEST(hand_voice_level_and_volume) {
    std::vector<float> c = clean();
    c[hand::kEnvInit] = 0.0f;
    c[hand::kEnvStep] = 0.0f;
    Rig r(c);
    r.on(0, 60.0f);
    CHECK_EQ(r.v[0].level, 1.0f);
    r.run(20);
    CHECK(r.v[0].level > 0.0f && r.v[0].level < 0.5f);
    std::vector<float> a = initValues(), b = initValues();
    a[hand::kVolume] = 0.0f;
    b[hand::kVolume] = 2.0f;
    CHECK(play(a) == play(b));
}

TEST(hand_level_matches_the_synth_conventions) {
    BankSlot s(1);
    for (int k = 0; k < 20; ++k) {
        s.on(0, 45 + 3 * (k % 8));
        s.run(100);
    }
    std::vector<float> c = initValues();
    c[hand::kChannel] = float(hand::kDrums);
    BankSlot k(6, &c);
    for (int rep = 0; rep < 4; ++rep) {
        const int kit[5] = {60, 62, 64, 69, 52};
        for (int j = 0; j < 5; ++j) k.on(j, kit[j]);
        k.run(200);
    }
#if defined(OT_MOD_ACD1)
    BankSlot a(1, nullptr, kSynthEngineAcd);
    for (int j = 0; j < 20; ++j) {
        a.on(0, 45 + 3 * (j % 8));
        a.run(100);
    }
    std::printf("  Hand-Type Init at -9 dB: peak %.1f dBFS (ACD %.1f), kit of five %.1f dBFS\n", dbfs(s.pk), dbfs(a.pk), dbfs(k.pk));
    CHECK(std::fabs(dbfs(s.pk) - dbfs(a.pk)) < 3.0);
#else
    std::printf("  Hand-Type Init at -9 dB: peak %.1f dBFS, kit of five %.1f dBFS\n", dbfs(s.pk), dbfs(k.pk));
#endif
    CHECK(dbfs(s.pk) < -3.0 && dbfs(s.pk) > -20.0);
    CHECK(dbfs(k.pk) < -1.0 && dbfs(k.pk) > -20.0);
    CHECK_EQ(s.clips + k.clips, 0L);
}

namespace {
// A sequence that every parameter of a channel can act on: two notes (the
// second reused), the release.
Buf sequence(const std::vector<float>& vals) {
    Rig r(vals);
    r.on(0, 57.0f);
    r.run(kBlocksPerSec / 3);
    r.on(0, 64.0f, true, true);
    r.run(kBlocksPerSec / 3);
    r.off(0);
    r.run(kBlocksPerSec / 4);
    return r.L;
}
// The Drums: every drum in turn on one voice each.
Buf kitSequence(const std::vector<float>& vals) {
    Rig r(vals);
    const int notes[8] = {60, 61, 62, 63, 64, 68, 69, 70};
    for (int k = 0; k < 8; ++k) {
        r.on(k, float(notes[k]));
        r.run(kBlocksPerSec / 6);
    }
    r.on(0, 67.0f);
    r.run(kBlocksPerSec / 6);
    r.on(1, 52.0f);
    r.run(kBlocksPerSec / 2);
    return r.L;
}
}  // namespace

TEST(hand_every_parameter_has_an_effect) {
    for (int i = 0; i < kN; ++i) {
        if (i == hand::kVolume) continue;   // the framework's
        std::vector<float> ctx = initValues();
        ctx[hand::kEnvStep] = 6.0f;
        ctx[hand::kRelease] = float(hand::kFade);
        ctx[hand::kArpMode] = 0.0f;
        bool kit = false;
        if (i >= hand::kDuty && i <= hand::kP2Duty) {
            ctx[hand::kP2Level] = 0.5f;
            ctx[hand::kSweepDir] = float(hand::kSweepDown);
            ctx[hand::kSweepShift] = i == hand::kSweepCut ? 1.0f : 6.0f;
            ctx[hand::kSweepTime] = i == hand::kSweepCut ? 1.0f : 4.0f;
        } else if (i >= hand::kWave && i <= hand::kLevels) {
            ctx[hand::kChannel] = float(hand::kWaveCh);
            ctx[hand::kWave] = float(hand::kUser);
        } else if (i >= hand::kWidth && i <= hand::kTrack) {
            ctx[hand::kChannel] = float(hand::kNoise);
        } else if (i >= hand::kKickTune && i <= hand::kBlipDecay) {
            ctx[hand::kChannel] = float(hand::kDrums);
            kit = true;
        } else if (i >= hand::kArpMode && i <= hand::kSlideTime) {
            ctx[hand::kArpMode] = float(chip::kArpCustom);
            ctx[hand::kVibDepth] = 0.5f;
            ctx[hand::kSlideStart] = 7.0f;
        } else if (i == hand::kGlideMode) {
            ctx[hand::kGlideTime] = 0.3f;
        }
        std::vector<float> a = ctx, b = ctx;
        a[size_t(i)] = range(i)->min;
        b[size_t(i)] = range(i)->max;
        if (i == hand::kSweepDir) a[size_t(i)] = float(hand::kSweepUp);   // Off vs Down are both in the range
        if (i == hand::kGlideMode) {
            a[size_t(i)] = float(hand::kAlways);
            b[size_t(i)] = float(hand::kLegato);
        }
        const double d = kit ? rmsDiff(kitSequence(a), kitSequence(b)) : rmsDiff(sequence(a), sequence(b));
        CHECK(d > 1e-4);
        if (!(d > 1e-4)) std::printf("       param %d differs by %g only\n", i, d);
    }
}

TEST(hand_duty_jumps_at_bits_8_stay_bounded) {
    // Duty / P2 Duty steps at the clean DAC, low notes (the duty's ramp is
    // faster than the phase there): no spike above the pulse itself.
    double worst = 0.0;
    for (float note : {24.0f, 27.0f, 36.0f, 48.0f}) {
        std::vector<float> c = clean();
        c[hand::kP2Level] = 0.5f;
        c[hand::kDuty] = float(hand::kDuty75);
        c[hand::kP2Duty] = float(hand::kDuty75);
        Rig r(c);
        r.on(0, note);
        r.run(60);
        const float steady = peak(r.L, r.L.size() - 6000, r.L.size());
        for (int d : {0, 3, 1}) {
            c[hand::kDuty] = float(d);
            c[hand::kP2Duty] = float(3 - d);
            r.set(c);
            const size_t at = r.L.size();
            r.run(3);
            worst = std::max(worst, double(peak(r.L, at, r.L.size()) / steady));
        }
    }
    std::printf("  duty jumps at Bits 8: peak %.2f x the steady pulse\n", worst);
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

TEST(hand_golden_hashes) {
    const uint64_t e = hashOf(kitSequence([] { std::vector<float> c = initValues(); c[hand::kChannel] = float(hand::kDrums); return c; }())), b = bankHash();
    std::printf("  Hand-Type golden: engine %016llx, bank %016llx\n", (unsigned long long)e, (unsigned long long)b);
    CHECK_EQ(e, uint64_t(0xbf4e87fe3f454f3full));
    CHECK_EQ(b, uint64_t(0x5becc0b8eac6b1b9ull));
}
