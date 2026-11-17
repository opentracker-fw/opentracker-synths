// 1-Type reference features (host only, C++17, double precision): the case
// table format of onetype_ref.inc and the measurements both sides make. The
// reference tool (tools/ref/ref1type.cpp) renders each case through the
// source engine and writes the features it measures here; the engine test
// renders the same case through the 1-Type engine and measures the same
// features with the same code, so only the engines differ.
//
// Derived from Oxo Synth2 (https://github.com/chab/oxo_synth2, commit 6bb4435):
// the measurement methods of its tools.
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
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace onetype_ref {

// --- the case table ---------------------------------------------------------------

// What a case measures. Voice kinds render one voice of a patch (key at 0,
// key2 at t2 on the same voice, the note off at tOff); kernel kinds drive
// the filter section (filter, limiter, saturation) directly.
enum Kind : uint8_t {
    kPitch,     // arg: guess Hz, t0, t1                  -> Hz
    kHarm,      // arg: guess f0, t0, t1, n               -> dB of harmonics 1..n of the measured f0
    kTones,     // arg: nwin, (t0, t1) x nwin, freqs...   -> dB per window and frequency
    kRmsCent,   // arg: t0, t1, fmax                      -> rms dB, spectral centroid Hz (40 Hz .. fmax)
    kEnv,       // (pulse at key 0) arg: t0, t1 (plateau) -> t90, decay to 1/e, plateau / peak, release to 1/e
    kEnvAt,     // (pulse at key 0) arg: times...         -> level / peak at each time
    kEdges,     // (pulse at key 0) arg: t0, t1           -> mean period of the level's falling edges (s)
    kDuty,      // arg: times...                          -> high fraction of the pulse cycle around each time
    kFreqAt,    // arg: window, times...                  -> Hz around each time (upward zero crossings)
    kGlide,     // arg: from Hz, to Hz                    -> time to 90 %, time to land (s after t2)
    kPeakSpec,  // arg: fmin, fmax, nwin, (t0, t1) x nwin -> spectral peak Hz per window
    kPanLR,     // arg: nwin, (t0, t1) x nwin             -> L, R rms dB per window
    kDc,        // arg: t0, t1                            -> the mean (DC) of L
    kFiltIr,    // kernel; arg: type, Hz, res, peak (0/1), probes... -> dB at each probe [, peak Hz]
    kSelfOsc,   // kernel; arg: type, Hz, res             -> rms dB, Hz
    kLimSat,    // kernel; arg: sat, levels...            -> output rms dB per input sine level
    kKindCount,
};

struct Set {
    int8_t i;   // 1-Type parameter index
    float v;
};

constexpr int kMaxSet = 20, kMaxArg = 16, kMaxExp = 24;

struct Case {
    const char* name;
    uint8_t kind;
    int8_t key, key2;    // -1: none
    uint8_t flags;       // kFlag*
    int16_t b2, bOff;    // key2 / note off at this block (128 frames), -1: none
    int16_t blocks;      // render length in blocks
    float tempo;
    uint8_t nset;
    Set set[kMaxSet];    // 1-Type parameters different from kBase
    uint8_t narg;
    float arg[kMaxArg];
    uint8_t nexp;
    float exp[kMaxExp];  // the source engine's features
};
enum : uint8_t {
    kFlagRef = 1,        // the level reference of the voice kinds (its first feature)
    kFlagLegato = 2,     // key2 without retriggering the envelopes
    kFlagLevel = 4,      // the features are levels (dB): compared after the reference offset
};

constexpr int kBlockFrames = 128;

// --- spectra ----------------------------------------------------------------------

constexpr double kPi = 3.14159265358979323846;

inline double db(double x) { return 20.0 * std::log10(std::max(x, 1e-12)); }

// Amplitude of the tone at hz in x[a, b) (Hann window: a sine of amplitude A reads A).
inline double toneAmp(const std::vector<float>& x, size_t a, size_t b, double hz, double fs) {
    b = std::min(b, x.size());
    if (b <= a + 2) return 0.0;
    const size_t n = b - a;
    const double w = 2.0 * kPi * hz / fs, wn = 2.0 * kPi / double(n);
    double re = 0, im = 0, ws = 0;
    for (size_t i = 0; i < n; ++i) {
        const double h = 0.5 - 0.5 * std::cos(wn * double(i));
        re += h * x[a + i] * std::cos(w * double(a + i));
        im -= h * x[a + i] * std::sin(w * double(a + i));
        ws += h;
    }
    return 2.0 * std::sqrt(re * re + im * im) / ws;
}
// The largest tone within +-0.4 % of hz (a mistuning of a few cents reads the same).
inline double toneNear(const std::vector<float>& x, size_t a, size_t b, double hz, double fs) {
    double best = 0;
    for (int k = -4; k <= 4; ++k) best = std::max(best, toneAmp(x, a, b, hz * (1.0 + 0.001 * k), fs));
    return best;
}

// Frequency near `guess` (within about +-4 Hz) in x[a, b): the phase advance of
// the tone at `guess` from one 0.1 s segment to the next, least squares.
inline double freqNear(const std::vector<float>& x, size_t a, size_t b, double guess, double fs) {
    b = std::min(b, x.size());
    const size_t seg = size_t(0.1 * fs);
    std::vector<double> ph, tm;
    double last = 0, wrap = 0, energy = 0;
    for (size_t i = a; i < b; ++i) energy += double(x[i]) * x[i];
    if (energy < 1e-12) return 0.0;   // silence
    for (size_t s = a; s + seg <= b; s += seg / 2) {
        double re = 0, im = 0;
        const double w = 2.0 * kPi * guess / fs, wn = 2.0 * kPi / double(seg);
        for (size_t i = 0; i < seg; ++i) {
            const double h = 0.5 - 0.5 * std::cos(wn * double(i));
            const double t = double(s + i);
            re += h * x[s + i] * std::cos(w * t);
            im -= h * x[s + i] * std::sin(w * t);
        }
        double p = std::atan2(im, re);
        if (!ph.empty()) {
            while (p + wrap - last > kPi) wrap -= 2.0 * kPi;
            while (p + wrap - last < -kPi) wrap += 2.0 * kPi;
        }
        last = p + wrap;
        ph.push_back(last);
        tm.push_back((double(s) + 0.5 * double(seg)) / fs);
    }
    if (ph.size() < 2) return guess;
    double mt = 0, mp = 0;
    for (size_t i = 0; i < ph.size(); ++i) mt += tm[i], mp += ph[i];
    mt /= double(ph.size());
    mp /= double(ph.size());
    double num = 0, den = 0;
    for (size_t i = 0; i < ph.size(); ++i) num += (tm[i] - mt) * (ph[i] - mp), den += (tm[i] - mt) * (tm[i] - mt);
    // The analysis rotates at -guess: a tone at guess + d advances +2 pi d per second.
    return guess + num / den / (2.0 * kPi);
}

// In-place radix-2 FFT (n a power of two).
inline void fft(std::vector<std::complex<double>>& a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * kPi / double(len);
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1);
            for (size_t k = 0; k < len / 2; ++k) {
                const auto u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}
// Power spectrum of x[a, a + n) (Hann), n a power of two.
inline std::vector<double> power(const std::vector<float>& x, size_t a, size_t n) {
    std::vector<std::complex<double>> c(n);
    for (size_t i = 0; i < n; ++i) {
        const double h = 0.5 - 0.5 * std::cos(2.0 * kPi * double(i) / double(n));
        c[i] = a + i < x.size() ? h * x[a + i] : 0.0;
    }
    fft(c);
    std::vector<double> p(n / 2);
    for (size_t k = 0; k < n / 2; ++k) p[k] = std::norm(c[k]);
    return p;
}
inline size_t pow2Below(size_t n) {
    size_t p = 1;
    while (p * 2 <= n) p *= 2;
    return p;
}

// --- level envelope of a pulse at key 0 (flat stretches of +-0.5) --------------------

// |x| on the samples away from the pulse's edges (48 samples: the band-limited
// edge's ripple), -1 elsewhere.
inline std::vector<double> flatLevel(const std::vector<float>& x) {
    std::vector<double> e(x.size(), -1.0);
    std::vector<uint8_t> bad(x.size(), 0);
    int last = 0;   // the sign of the last non-zero sample (silence before the note is no edge)
    for (size_t i = 0; i < x.size(); ++i) {
        const int sg = x[i] > 0 ? 1 : (x[i] < 0 ? -1 : 0);
        if (sg && last && sg != last)
            for (size_t k = i >= 48 ? i - 48 : 0; k < std::min(x.size(), i + 48); ++k) bad[k] = 1;
        if (sg) last = sg;
    }
    for (size_t i = 0; i < x.size(); ++i)
        if (!bad[i]) e[i] = std::fabs(double(x[i]));
    return e;
}
inline double maxOf(const std::vector<double>& e) {
    double m = 0;
    for (double v : e) m = std::max(m, v);
    return m;
}
// First valid sample at or after `from` where the level is >= (rising) or < (falling) th.
inline double crossTime(const std::vector<double>& e, size_t from, double th, bool rising, double fs) {
    for (size_t i = from; i < e.size(); ++i) {
        if (e[i] < 0) continue;
        if (rising ? e[i] >= th : e[i] < th) return double(i) / fs;
    }
    return -1.0;
}
// Mean valid level in [a, b).
inline double meanLevel(const std::vector<double>& e, size_t a, size_t b) {
    double s = 0;
    size_t n = 0;
    for (size_t i = a; i < std::min(b, e.size()); ++i)
        if (e[i] >= 0) s += e[i], ++n;
    return n ? s / double(n) : 0.0;
}

// --- zero crossings --------------------------------------------------------------

inline std::vector<double> upCrossings(const std::vector<float>& x, double fs) {
    std::vector<double> t;
    for (size_t i = 1; i < x.size(); ++i)
        if (x[i - 1] < 0 && x[i] >= 0) t.push_back((double(i - 1) + x[i - 1] / double(x[i - 1] - x[i])) / fs);
    return t;
}

// --- the features ------------------------------------------------------------------

// The features of a voice kind from its render (L, R at fs). t2: the time of
// key2 (kGlide). Returns the number of features (out has kMaxExp).
inline int voiceFeatures(const Case& c, const std::vector<float>& L, const std::vector<float>& R, double fs, float* out) {
    const float* a = c.arg;
    const auto at = [&](double t) { return size_t(std::max(0.0, t) * fs); };
    switch (c.kind) {
        case kPitch:
            out[0] = float(freqNear(L, at(a[1]), at(a[2]), a[0], fs));
            return 1;
        case kHarm: {
            const double f0 = freqNear(L, at(a[1]), at(a[2]), a[0], fs);
            const int n = int(a[3]);
            for (int k = 0; k < n; ++k) out[k] = float(db(toneAmp(L, at(a[1]), at(a[2]), f0 * (k + 1), fs)));
            return n;
        }
        case kTones: {
            const int nw = int(a[0]), nf = c.narg - 1 - 2 * nw;
            int k = 0;
            for (int w = 0; w < nw; ++w)
                for (int f = 0; f < nf; ++f)
                    out[k++] = float(db(toneNear(L, at(a[1 + 2 * w]), at(a[2 + 2 * w]), a[1 + 2 * nw + f], fs)));
            return k;
        }
        case kRmsCent: {
            const size_t i0 = at(a[0]), n = pow2Below(at(a[1]) - i0);
            double s = 0;
            for (size_t i = i0; i < i0 + n && i < L.size(); ++i) s += double(L[i]) * L[i];
            out[0] = float(db(std::sqrt(s / double(n))));
            const std::vector<double> p = power(L, i0, n);
            double num = 0, den = 0;
            for (size_t k = 1; k < p.size(); ++k) {   // 40 Hz .. fmax: DC and sub-audio left out
                const double f = double(k) * fs / double(n);
                if (f < 40.0) continue;
                if (f > a[2]) break;
                num += f * p[k], den += p[k];
            }
            out[1] = float(den > 0 ? num / den : 0.0);
            return 2;
        }
        case kEnv: {
            const std::vector<double> e = flatLevel(L);
            const double m = maxOf(e);
            const double t90 = crossTime(e, 0, 0.9 * m, true, fs);
            const double tPeak = crossTime(e, 0, 0.999 * m, true, fs);
            const double tDec = tPeak >= 0 ? crossTime(e, at(tPeak), m / std::exp(1.0), false, fs) : -1.0;
            out[0] = float(t90);
            out[1] = float(tDec >= 0 ? tDec - tPeak : -1.0);
            out[2] = float(meanLevel(e, at(a[0]), at(a[1])) / m);
            out[3] = -1.0f;
            if (c.bOff >= 0) {
                const size_t off = size_t(c.bOff) * kBlockFrames;
                const double lv = meanLevel(e, off >= 400 ? off - 400 : 0, off);
                const double t = crossTime(e, off, lv / std::exp(1.0), false, fs);
                out[3] = float(t >= 0 ? t - double(off) / fs : -1.0);
            }
            return 4;
        }
        case kEnvAt: {
            const std::vector<double> e = flatLevel(L);
            const double m = maxOf(e);
            for (int k = 0; k < c.narg; ++k) {
                const size_t i = at(a[k]);
                out[k] = float(meanLevel(e, i >= 32 ? i - 32 : 0, i + 32) / m);
            }
            return c.narg;
        }
        case kEdges: {
            const std::vector<double> e = flatLevel(L);
            const double m = maxOf(e);
            std::vector<double> fall;
            bool hi = false;
            for (size_t i = at(a[0]); i < std::min(e.size(), at(a[1])); ++i) {
                if (e[i] < 0) continue;
                const bool h = e[i] > 0.5 * m;
                if (hi && !h) fall.push_back(double(i) / fs);
                hi = h;
            }
            out[0] = fall.size() >= 2 ? float((fall.back() - fall.front()) / double(fall.size() - 1)) : -1.0f;
            return 1;
        }
        case kDuty: {
            // Rising edges (low -> high) around each time; the cycle's high fraction.
            std::vector<double> up, down;
            // The note's start counts as a rising edge when it starts high.
            for (size_t i = 0; i < L.size(); ++i)
                if (L[i] != 0.0f) {
                    if (L[i] > 0.0f) up.push_back(double(i) / fs);
                    break;
                }
            for (size_t i = 1; i < L.size(); ++i) {
                if (L[i - 1] < 0 && L[i] >= 0) up.push_back(double(i) / fs);
                if (L[i - 1] >= 0 && L[i] < 0) down.push_back(double(i) / fs);
            }
            for (int k = 0; k < c.narg; ++k) {
                out[k] = -1.0f;
                for (size_t j = 0; j + 1 < up.size(); ++j) {
                    if (up[j + 1] < a[k]) continue;
                    const auto d = std::lower_bound(down.begin(), down.end(), up[j]);
                    if (d != down.end() && *d < up[j + 1]) out[k] = float((*d - up[j]) / (up[j + 1] - up[j]));
                    break;
                }
            }
            return c.narg;
        }
        case kFreqAt: {
            const std::vector<double> z = upCrossings(L, fs);
            const double w = a[0];
            for (int k = 1; k < c.narg; ++k) {
                const auto lo = std::lower_bound(z.begin(), z.end(), a[k] - 0.5 * w);
                const auto hi = std::upper_bound(z.begin(), z.end(), a[k] + 0.5 * w);
                const long n = long(hi - lo);
                out[k - 1] = n >= 2 ? float(double(n - 1) / (*(hi - 1) - *lo)) : -1.0f;
            }
            return c.narg - 1;
        }
        case kGlide: {
            // The pitch per period (semitones) after key 2; the times it passes
            // 90 % of the interval and comes within 1 cent of the note for good,
            // interpolated between periods.
            const std::vector<double> z = upCrossings(L, fs);
            const double t2 = double(c.b2) * kBlockFrames / fs;
            const double s0 = 12.0 * std::log2(a[0] / 440.0), s1 = 12.0 * std::log2(a[1] / 440.0);
            double t90 = -1, tLand = -1, lastT = -1, lastF = 0, lastD = 0;
            for (size_t i = 0; i + 1 < z.size(); ++i) {
                const double t = 0.5 * (z[i] + z[i + 1]);
                if (t < t2) continue;
                const double s = 12.0 * std::log2(1.0 / (z[i + 1] - z[i]) / 440.0);
                const double f = (s - s0) / (s1 - s0), dist = std::fabs(s - s1);
                if (t90 < 0 && f >= 0.9) t90 = lastT < 0 ? t - t2 : lastT + (t - lastT) * (0.9 - lastF) / (f - lastF) - t2;
                if (dist < 0.01) {
                    if (tLand < 0) tLand = lastT < 0 || lastD <= dist ? t - t2 : lastT + (t - lastT) * (lastD - 0.01) / (lastD - dist) - t2;
                } else {
                    tLand = -1;
                }
                lastT = t, lastF = f, lastD = dist;
            }
            out[0] = float(t90);
            out[1] = float(tLand);
            return 2;
        }
        case kPeakSpec: {
            const int nw = int(a[2]);
            for (int w = 0; w < nw; ++w) {
                const size_t i0 = at(a[3 + 2 * w]), n = pow2Below(at(a[4 + 2 * w]) - i0);
                const std::vector<double> p = power(L, i0, n);
                // Smoothed over +-1/12 octave (noise input), the peak, parabolic.
                std::vector<double> s(p.size(), 0.0);
                for (size_t k = 1; k < p.size(); ++k) {
                    const size_t h = std::max<size_t>(2, size_t(double(k) * 0.03));
                    double v = 0;
                    for (size_t j = k > h ? k - h : 1; j <= std::min(p.size() - 1, k + h); ++j) v += p[j];
                    s[k] = v / double(2 * h + 1);
                }
                size_t best = 1;
                for (size_t k = 1; k + 1 < s.size(); ++k) {
                    const double f = double(k) * fs / double(n);
                    if (f < a[0] || f > a[1]) continue;
                    if (s[k] > s[best] || best == 1) best = k;
                }
                double off = 0;
                if (best > 1 && best + 1 < s.size()) {
                    const double y0 = std::log(s[best - 1] + 1e-300), y1 = std::log(s[best] + 1e-300), y2 = std::log(s[best + 1] + 1e-300);
                    const double d = y0 - 2.0 * y1 + y2;
                    if (d < 0) off = 0.5 * (y0 - y2) / d;
                }
                out[w] = float((double(best) + off) * fs / double(n));
            }
            return nw;
        }
        case kPanLR: {
            const int nw = int(a[0]);
            for (int w = 0; w < nw; ++w) {
                const size_t i0 = at(a[1 + 2 * w]), i1 = std::min(L.size(), at(a[2 + 2 * w]));
                double sl = 0, sr = 0;
                for (size_t i = i0; i < i1; ++i) sl += double(L[i]) * L[i], sr += double(R[i]) * R[i];
                out[2 * w] = float(db(std::sqrt(sl / double(i1 - i0))));
                out[2 * w + 1] = float(db(std::sqrt(sr / double(i1 - i0))));
            }
            return 2 * nw;
        }
        case kDc: {
            double sum = 0;
            const size_t i0 = at(a[0]), i1 = std::min(L.size(), at(a[1]));
            for (size_t i = i0; i < i1; ++i) sum += L[i];
            out[0] = float(sum / double(i1 - i0));
            return 1;
        }
        default: return 0;
    }
}

// --- kernel features -----------------------------------------------------------------

// A filter section as a sample processor: step(x) -> y.
// kFiltIr: the impulse response (0.01) over 2^16 samples, its magnitude at the
// probes (dB, interpolated between bins) and, when asked, the peak frequency.
template <class Step>
int filterIrFeatures(const Case& c, Step&& step, double fs, float* out) {
    const size_t n = 65536;
    std::vector<std::complex<double>> h(n);
    for (size_t i = 0; i < n; ++i) h[i] = step(i == 0 ? 0.01 : 0.0) / 0.01;
    fft(h);
    const auto mag = [&](double f) {
        const double x = f / fs * double(n);
        const size_t k = std::min(n / 2 - 2, size_t(x));
        const double t = x - double(k);
        return db(std::abs(h[k])) * (1.0 - t) + db(std::abs(h[k + 1])) * t;
    };
    int m = 0;
    const bool peak = c.arg[3] > 0.5f;
    for (int k = 4; k < c.narg; ++k) out[m++] = float(mag(c.arg[k]));
    if (peak) {
        size_t best = 2;
        for (size_t k = 2; k < n / 2 - 1; ++k)
            if (std::abs(h[k]) > std::abs(h[best])) best = k;
        const double y0 = db(std::abs(h[best - 1])), y1 = db(std::abs(h[best])), y2 = db(std::abs(h[best + 1]));
        const double d = y0 - 2.0 * y1 + y2;
        const double off = d < 0 ? 0.5 * (y0 - y2) / d : 0.0;
        out[m++] = float((double(best) + off) * fs / double(n));
    }
    return m;
}
// kSelfOsc: an impulse (0.01), 2 s; the last 0.5 s: rms dB and frequency.
template <class Step>
int selfOscFeatures(Step&& step, double fs, float* out) {
    const size_t n = size_t(2.0 * fs), a = size_t(1.5 * fs);
    std::vector<float> y(n);
    for (size_t i = 0; i < n; ++i) y[i] = float(step(i == 0 ? 0.01 : 0.0));
    double s = 0;
    for (size_t i = a; i < n; ++i) s += double(y[i]) * y[i];
    out[0] = float(db(std::sqrt(s / double(n - a))));
    std::vector<float> tail(y.begin() + long(a), y.end());
    const std::vector<double> z = upCrossings(tail, fs);
    out[1] = z.size() >= 2 ? float(double(z.size() - 1) / (z.back() - z.front())) : -1.0f;
    return 2;
}
// kLimSat: a 220 Hz sine at each level for 0.6 s through a fresh section
// (make()); the rms of the last 0.25 s.
template <class Make>
int limSatFeatures(const Case& c, Make&& make, double fs, float* out) {
    int m = 0;
    for (int k = 1; k < c.narg; ++k) {
        auto step = make();
        const size_t n = size_t(0.6 * fs), a = size_t(0.35 * fs);
        double s = 0;
        for (size_t i = 0; i < n; ++i) {
            const double y = step(c.arg[k] * std::sin(2.0 * kPi * 220.0 * double(i) / fs));
            if (i >= a) s += y * y;
        }
        out[m++] = float(db(std::sqrt(s / double(n - a))));
    }
    return m;
}

}  // namespace onetype_ref
