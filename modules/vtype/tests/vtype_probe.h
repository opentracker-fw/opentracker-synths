// V-Type test helpers (engine_vtype_test.cpp): the engine's voices driven
// directly, a SynthBank slot, and the measurements: Goertzel line levels and
// phases, pitch by zero crossings, block envelopes, band energy, and the
// filter laws in double (the specification's second implementation).
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <vector>

#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "modules/synth_models.h"
#include "vtype.h"

namespace vprobe {

using namespace ot;
using namespace ot::audio;
namespace vt = ot::audio::vtype;

constexpr double kFs = ot::audio::kSampleRate;
constexpr int kN = vt::kParamCount;
constexpr int kBlocksPerSec = int(kFs / double(kBlock)) + 1;   // 345
constexpr double kPi = 3.14159265358979323846;

using Buf = std::vector<float>;
using Vals = std::vector<float>;

inline const PatchParamRange* range(int i) { return synthParamRange(vt::kEngineId, i); }

inline Vals initValues() {
    mod::ModuleHost::instance().init();
    Vals v(size_t(kN) + 2, 0.0f);
    for (int i = 0; i < kN; ++i) v[size_t(i)] = range(i)->def;
    return v;
}

// The engine's voices driven directly (no framework). Values cover the
// stored parameters and the two extension parameters (Width, Spread).
struct Rig {
    ISynthEngine* e = synthEngineFor(vt::kEngineId);
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    Buf L, R;
    float bpm = 120.0f;

    explicit Rig(const Vals& vals) {
        p.engine = vt::kEngineId;
        p.count = kN + 2;
        set(vals);
        if (e) e->slotReset(scratch);
        for (auto& x : v) {
            x = SynthVoice{};
            x.slotScratch = &scratch;
        }
    }
    void set(const Vals& vals) {
        for (int i = 0; i < kN + 2; ++i) p.start[i] = p.end[i] = vals[size_t(i)];
        ++p.serial;
    }
    // One block moving from the values now to `vals` (p.start -> p.end), then held.
    void move(const Vals& vals) {
        for (int i = 0; i < kN + 2; ++i) {
            p.start[i] = p.end[i];
            p.end[i] = vals[size_t(i)];
        }
        p.moving = true;
        ++p.serial;
        run(1);
        p.moving = false;
        for (int i = 0; i < kN + 2; ++i) p.start[i] = p.end[i];
    }
    void on(int k, float pitch, bool reused = false, bool overlap = false) {
        if (!e) return;
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
        if (!e) return;
        v[k].gate = false;
        e->noteOff(v[k], p);
    }
    int run(int blocks) {
        int live = 0;
        for (int b = 0; b < blocks; ++b) {
            float l[kBlock] = {}, r[kBlock] = {};
            live = 0;
            if (e) {
                e->setTempo(bpm);
                e->prepare(scratch, p);
                for (int k = 0; k < kSynthMaxVoices; ++k) {
                    if (!alive[k]) continue;
                    alive[k] = e->render(v[k], p, 1.0f, 0.0f, l, r);
                    live += alive[k];
                }
            }
            L.insert(L.end(), l, l + kBlock);
            R.insert(R.end(), r, r + kBlock);
        }
        return live;
    }
    void seconds(double s) { run(int(s * kFs / double(kBlock)) + 1); }
};

// The Init patch (or an edited one) through a real SynthBank slot at the
// default slot gain.
struct BankSlot {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
    float pk = 0.0f;
    long clips = 0;
    Buf out;
    BankSlot(int poly, const Vals* vals = nullptr, int32_t engine = vt::kEngineId) {
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

// --- measurements ------------------------------------------------------------------

inline bool finite(const Buf& x) {
    for (float s : x)
        if (!std::isfinite(s)) return false;
    return true;
}
inline long subnormals(const Buf& x) {
    long n = 0;
    for (float s : x) n += std::fpclassify(s) == FP_SUBNORMAL;
    return n;
}
inline float peak(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    float m = 0.0f;
    for (size_t i = a; i < std::min(b, x.size()); ++i) m = std::max(m, std::fabs(x[i]));
    return m;
}
inline double rms(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    b = std::min(b, x.size());
    double s = 0.0;
    for (size_t i = a; i < b; ++i) s += double(x[i]) * x[i];
    return b > a ? std::sqrt(s / double(b - a)) : 0.0;
}
inline double rmsDiff(const Buf& a, const Buf& b) {
    const size_t n = std::min(a.size(), b.size());
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) s += double(a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(s / double(n ? n : 1));
}
inline Buf slice(const Buf& b, double t0, double t1) {
    const size_t a = std::min(b.size(), size_t(t0 * kFs)), e = std::min(b.size(), size_t(t1 * kFs));
    return Buf(b.begin() + long(a), b.begin() + long(std::max(a, e)));
}
inline double db(double x) { return 20.0 * std::log10(std::max(x, 1e-15)); }
inline double hzOf(double pitch) { return 440.0 * std::pow(2.0, (pitch - 69.0) / 12.0); }
inline double pitchOf(double hz) { return 69.0 + 12.0 * std::log2(hz / 440.0); }
inline double cents(double f, double ref) { return 1200.0 * std::log2(f / ref); }

// Goertzel (Hann window, normalised so a sine of amplitude A reads A).
inline std::complex<double> line(const Buf& b, double f, size_t a = 0, size_t e = SIZE_MAX) {
    e = std::min(e, b.size());
    const size_t n = e > a ? e - a : 1;
    const double w = 2.0 * kPi * f / kFs, wn = 2.0 * kPi / double(n);
    std::complex<double> acc = 0.0;
    const std::complex<double> rot = std::polar(1.0, -w);
    std::complex<double> ph = 1.0;
    for (size_t i = 0; i < n; ++i) {
        const double win = 0.5 - 0.5 * std::cos(wn * double(i));
        acc += double(b[a + i]) * win * ph;
        ph *= rot;
        if ((i & 1023u) == 1023u) ph /= std::abs(ph);
    }
    return acc * (4.0 / double(n));   // Hann coherent gain 1/2, one-sided x 2
}
inline double level(const Buf& b, double f, size_t a = 0, size_t e = SIZE_MAX) { return std::abs(line(b, f, a, e)); }

// Upward zero crossings (interpolated, in frames) around the mean.
inline std::vector<double> crossings(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
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
inline double freqOf(const Buf& x, size_t a = 0, size_t b = SIZE_MAX) {
    const std::vector<double> t = crossings(x, a, b);
    if (t.size() < 3) return 0.0;
    return kFs * double(t.size() - 1) / (t.back() - t.front());
}
// Peak of |x| per block (the block-rate envelope).
inline std::vector<double> blockPeaks(const Buf& x, size_t from = 0) {
    std::vector<double> e;
    for (size_t a = from; a + kBlock <= x.size(); a += kBlock) e.push_back(peak(x, a, a + kBlock));
    return e;
}
// Mean power over the band [f0, f1] (n Goertzel lines).
inline double bandPower(const Buf& x, double f0, double f1, int n = 160) {
    double s = 0.0;
    for (int k = 0; k < n; ++k) {
        const double f = f0 + (f1 - f0) * (double(k) + 0.5) / double(n);
        const double m = level(x, f);
        s += m * m;
    }
    return s / double(n);
}
// Spectral centroid of lines h * f0 (h = 1..n).
inline double centroid(const Buf& x, double f0, int n) {
    double a = 0.0, b = 0.0;
    for (int h = 1; h <= n && h * f0 < 0.45 * kFs; ++h) {
        const double m = level(x, h * f0);
        a += double(h) * m;
        b += m;
    }
    return b > 0.0 ? a / b : 0.0;
}

// --- the laws in double -----------------------------------------------------------

// TPT state-variable stage at cutoff fc (g = tan(pi fc / fs)) and k = 2 x
// damping: low / band / high-pass responses at f (bilinear, exact).
inline std::complex<double> svfH(int out, double fc, double k, double f) {
    const double g = std::tan(kPi * std::clamp(fc, 8.0, 0.45 * kFs) / kFs);
    const std::complex<double> s(0.0, std::tan(kPi * f / kFs) / g);
    const std::complex<double> d = s * s + k * s + 1.0;
    return out == 0 ? 1.0 / d : (out == 1 ? s / d : s * s / d);
}
// A filter type's response (vtype FType): LP12, LP24 (a stage at k = 1.848
// before the resonant one), BP12, HP12, with the input gain of resoGain.
inline std::complex<double> typeH(int type, double fc, double reso, double f) {
    const double k = double(vt::resoK(float(reso))), g = double(vt::resoGain(float(k)));
    switch (type) {
        case vt::kLp12: return g * svfH(0, fc, k, f);
        case vt::kLp24: return g * svfH(0, fc, double(vt::kStage1K), f) * svfH(0, fc, k, f);
        case vt::kBp12: return g * svfH(1, fc, k, f);
        default: return g * svfH(2, fc, k, f);
    }
}
// Swarm levels (centre, each side saw) at Count index c and Mix m.
inline void swarmLevels(int c, double m, double& centre, double& side) {
    const int P = (vt::sawsOf(c) - 1) / 2;
    if (P == 0) {
        centre = 1.0;
        side = 0.0;
        return;
    }
    centre = std::sqrt(1.0 - double(vt::kMixShare) * m);
    side = std::sqrt(double(vt::kMixShare) * m / (2.0 * P));
}
// Saw k's offset in cents.
inline double sawCents(int c, double detune, int k) {
    if (k == 0) return 0.0;
    return double(vt::spreadCents(float(detune))) * double(vt::kOffsets[c][k - 1]);
}
inline double softCurve(double x) {
    x = std::clamp(x, -3.0, 3.0);
    return x * (27.0 + x * x) / (27.0 + 9.0 * x * x);
}
inline double foldCurve(double x) {   // one fold back at +-1, held past +-3
    const double h = std::clamp(x, -1.0, 1.0);
    return std::clamp(h + h - x, -1.0, 1.0);
}
inline double curveOf(int c, double x) {
    return c == vt::kSoft ? softCurve(x) : (c == vt::kHard ? std::clamp(x, -1.0, 1.0) : foldCurve(x));
}
// Fundamental amplitude of the Drive stage for a sine of amplitude a.
inline double driveGain(int curve, double d, double a) {
    const int n = 4096;
    std::complex<double> acc = 0.0;
    const double gin = std::pow(2.0, 4.0 * d), gout = std::pow(2.0, -2.4 * d);
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * double(i) / double(n);
        acc += gout * curveOf(curve, gin * a * std::sin(ph)) * std::polar(1.0, -ph);
    }
    return 2.0 * std::abs(acc) / double(n);
}

}  // namespace vprobe
