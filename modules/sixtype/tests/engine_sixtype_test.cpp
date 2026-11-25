// SPDX-License-Identifier: Apache-2.0
// 6-Type engine (modules/sixtype): SysEx parsing (single voice,
// 32-voice bank, bad checksums, junk), bank unpacking, range clamping, the
// note's pitch (ratio, fixed frequency, transpose, bend), velocity
// sensitivity, the voice end after its release, mono Retrigger / Legato,
// the engine controls (Brightness, Feedback, Attack, Release, LFO Depth,
// Detune), the algorithm 4 / 6 feedback loops, amplitude modulation, polyphony, no NaN /
// Inf and a bounded level over random voices, and every voice of the .syx
// files in $OT_SIXTYPE_SYX (a directory; skipped when unset) renders cleanly.
// Cycle counts: bench/sixtype_bench.cpp (sixtype.*, make perf-gate).
#include "test.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "audio/synth_engine.h"
#include "sixtype.h"
#include "sixtype_patch.h"
#include "sixtype_tables.h"
#include "formats/patch_pb.h"

using namespace ot;
using namespace ot::audio;
namespace fs = std::filesystem;

namespace {

constexpr int kBlocksPerSec = int(kSampleRate / float(kBlock)) + 1;   // 345

using Voice = std::vector<uint8_t>;
using Buf = std::vector<float>;

// A plain voice: algorithm 32 (six carriers), only operator 1 sounding (ratio
// 1, full level, organ envelope), the rest at output level 0.
Voice sineVoice() {
    Voice v(sixtype::kVoiceBytes, 0);
    for (int k = 0; k < 6; ++k) {
        uint8_t* o = v.data() + k * 21;
        for (int i = 0; i < 4; ++i) o[i] = 99;   // rates
        o[4] = o[5] = o[6] = 99;                 // levels 1-3 (L4 = 0)
        o[3] = 70;                               // release rate
        o[18] = 1;                               // ratio 1
        o[20] = 7;                               // no detune
    }
    v[5 * 21 + 16] = 99;   // operator 1 output level
    for (int i = 0; i < 4; ++i) {
        v[126 + i] = 99;
        v[130 + i] = 50;   // flat pitch envelope
    }
    v[134] = 31;   // algorithm 32
    v[136] = 1;    // key sync
    v[137] = 35;
    v[143] = 3;
    v[144] = 24;   // transpose 0
    std::memcpy(v.data() + 145, "SINE      ", 10);
    return v;
}

// A modulated voice: operator 2 (ratio 1) modulates operator 1.
Voice fmVoice(int alg = 0, int feedback = 0) {
    Voice v = sineVoice();
    v[4 * 21 + 16] = 80;   // operator 2 level
    v[134] = uint8_t(alg);
    v[135] = uint8_t(feedback);
    return v;
}

// 128-byte bank entry of a voice (the inverse of sixtype::unpackVoice).
std::vector<uint8_t> pack(const Voice& v) {
    std::vector<uint8_t> b(sixtype::kPackedBytes, 0);
    for (int op = 0; op < 6; ++op) {
        const uint8_t* s = v.data() + op * 21;
        uint8_t* d = b.data() + op * 17;
        std::memcpy(d, s, 11);
        d[11] = uint8_t(s[11] | (s[12] << 2));
        d[12] = uint8_t(s[13] | (s[20] << 3));
        d[13] = uint8_t(s[14] | (s[15] << 2));
        d[14] = s[16];
        d[15] = uint8_t(s[17] | (s[18] << 1));
        d[16] = s[19];
    }
    std::memcpy(b.data() + 102, v.data() + 126, 9);
    b[111] = uint8_t(v[135] | (v[136] << 3));
    std::memcpy(b.data() + 112, v.data() + 137, 4);
    b[116] = uint8_t(v[141] | (v[142] << 1) | (v[143] << 4));
    std::memcpy(b.data() + 117, v.data() + 144, 11);
    return b;
}

uint8_t checksum(const uint8_t* p, size_t n) {
    unsigned s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return uint8_t((128 - (s & 0x7f)) & 0x7f);
}

std::vector<uint8_t> singleDump(const Voice& v, bool goodSum = true) {
    std::vector<uint8_t> d = {0xF0, 0x43, 0x00, 0x00, 0x01, 0x1B};
    d.insert(d.end(), v.begin(), v.end());
    d.push_back(uint8_t((checksum(v.data(), v.size()) + (goodSum ? 0 : 1)) & 0x7f));
    d.push_back(0xF7);
    return d;
}

std::vector<uint8_t> bankDump(const std::vector<Voice>& voices) {
    std::vector<uint8_t> d = {0xF0, 0x43, 0x00, 0x09, 0x20, 0x00};
    for (int i = 0; i < sixtype::kBankVoices; ++i) {
        const auto b = pack(voices[size_t(i) % voices.size()]);
        d.insert(d.end(), b.begin(), b.end());
    }
    d.push_back(checksum(d.data() + 6, 4096));
    d.push_back(0xF7);
    return d;
}

// The engine with up to 8 voices, driven directly (no framework). Like the
// pool, a rig's voices share the engine's voice array (SynthVoice::index):
// play one rig to the end before starting notes on another.
struct Rig {
    ISynthEngine* e = sixtype::engine();
    SynthSlotScratch scratch{};
    SynthParams p;
    SynthVoice v[kSynthMaxVoices];
    bool alive[kSynthMaxVoices] = {};
    Buf L, R;

    explicit Rig(const Voice& voice) {
        p.engine = sixtype::kEngineId;
        e->slotReset(scratch);
        sixtype::setVoice(scratch, voice.data());
        for (int k = 0; k < kSynthMaxVoices; ++k) {
            v[k] = SynthVoice{};
            v[k].slotScratch = &scratch;
            v[k].index = uint8_t(k);
        }
    }
    // Engine control i (kSixtype*) = x; the others at their defaults.
    void set(int i, float x) {
        if (p.count == 0) {
            int n = 0;
            const PatchParamRange* t = patchEngineParams(sixtype::kEngineId, &n);
            p.count = n;
            for (int j = 0; j < n; ++j) p.start[j] = p.end[j] = t[j].def;
        }
        p.start[i] = p.end[i] = x;
    }
    void on(int k, float pitch, float velocity = 1.0f, bool reused = false, bool overlap = false) {
        if (!reused) v[k].level = 0.0f;   // the engine clears its own voice state
        v[k].gate = true;
        SynthNote n;
        n.pitch = pitch;
        n.note = uint8_t(std::clamp(int(pitch), 0, 127));
        n.velocity = velocity;
        n.reused = reused;
        n.overlap = overlap;
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
    return std::all_of(x.begin(), x.end(), [](float s) { return std::isfinite(s); });
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
// Frequency from upward zero crossings over [a, b).
double zeroCrossHz(const Buf& x, size_t a, size_t b) {
    int n = 0;
    size_t first = 0, last = 0;
    for (size_t i = a + 1; i < b; ++i)
        if (x[i - 1] < 0.0f && x[i] >= 0.0f) {
            if (n == 0) first = i;
            last = i;
            ++n;
        }
    return n > 1 ? double(n - 1) * kSampleRate / double(last - first) : 0.0;
}
// Share of the energy outside the fundamental (a brightness measure): 1 -
// the Goertzel power at f0 over a whole number of periods / the total.
double brightness(const Buf& x, size_t a, size_t b, double f0) {
    const double period = kSampleRate / f0;
    const size_t n = size_t(std::floor(double(b - a) / period) * period + 0.5);
    const double w = 2.0 * M_PI * f0 / kSampleRate;
    double re = 0.0, im = 0.0, e = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double v = x[a + i];
        re += v * std::cos(w * double(i));
        im -= v * std::sin(w * double(i));
        e += v * v;
    }
    return e > 0.0 ? 1.0 - 2.0 * (re * re + im * im) / (double(n) * e) : 0.0;
}

// Levels (dB) of harmonics 1..6 of f0 over [a, b) (Goertzel, whole periods).
std::vector<double> harmonicsDb(const Buf& x, size_t a, size_t b, double f0) {
    const double period = kSampleRate / f0;
    const size_t n = size_t(std::floor(double(b - a) / period) * period + 0.5);
    std::vector<double> h;
    for (int k = 1; k <= 6; ++k) {
        const double w = 2.0 * M_PI * k * f0 / kSampleRate;
        double re = 0.0, im = 0.0;
        for (size_t i = 0; i < n; ++i) {
            re += x[a + i] * std::cos(w * double(i));
            im -= x[a + i] * std::sin(w * double(i));
        }
        h.push_back(10.0 * std::log10(re * re + im * im + 1e-30));
    }
    return h;
}

double midiHz(double note) { return 440.0 * std::pow(2.0, (note - 69.0) / 12.0); }

}  // namespace

// [I] The sine table built at boot is the one that used to be stored in
// flash (generated by modules/sixtype/tools/make_sixtype_tables.py): FNV-1a of its 1025 entries.
TEST(sixtype_sine_table_built_at_boot_matches_the_stored_one) {
    sixtype::initSinTable();
    uint64_t h = 0xcbf29ce484222325ull;
    for (int32_t x : sixtype::gSixtypeSin)
        for (int k = 0; k < 4; ++k) h = (h ^ ((uint32_t(x) >> (8 * k)) & 0xff)) * 0x100000001b3ull;
    CHECK(h == 0xf63c5ec5a3b85f4full);
    CHECK(sixtype::gSixtypeSin[1] == 102943 && sixtype::gSixtypeSin[100] == 9660458 && sixtype::gSixtypeSin[700] == -15337894);
    sixtype::initSinTable();   // idempotent
    CHECK(sixtype::gSixtypeSin[256] == 1 << 24 && sixtype::gSixtypeSin[1024] == 0);
}

TEST(sixtype_parse_single_voice) {
    const Voice v = fmVoice(4, 5);
    uint8_t out[1][sixtype::kVoiceBytes];
    bool ok = false;
    const auto d = singleDump(v);
    CHECK_EQ(d.size(), size_t(163));
    CHECK_EQ(sixtype::parseSysex(d.data(), d.size(), out, 1, &ok), 1);
    CHECK(ok);
    CHECK(std::memcmp(out[0], v.data(), sixtype::kVoiceBytes) == 0);
    // A wrong checksum still loads (some hardware dumps carry one) but is reported.
    const auto bad = singleDump(v, false);
    CHECK_EQ(sixtype::parseSysex(bad.data(), bad.size(), out, 1, &ok), 1);
    CHECK(!ok);
}

TEST(sixtype_parse_bank) {
    std::vector<Voice> voices;
    for (int i = 0; i < 4; ++i) {
        Voice v = fmVoice(i * 7, i);
        v[145] = uint8_t('A' + i);
        v[5 * 21 + 20] = uint8_t(3 + i);   // detune
        v[4 * 21 + 11] = uint8_t(i & 3);   // curves
        v[4 * 21 + 12] = uint8_t(3 - (i & 3));
        v[142] = uint8_t(i);               // LFO wave
        voices.push_back(v);
    }
    const auto d = bankDump(voices);
    CHECK_EQ(d.size(), size_t(4104));
    static uint8_t out[sixtype::kBankVoices][sixtype::kVoiceBytes];
    bool ok = false;
    CHECK_EQ(sixtype::parseSysex(d.data(), d.size(), out, sixtype::kBankVoices, &ok), 32);
    CHECK(ok);
    for (int i = 0; i < sixtype::kBankVoices; ++i)
        CHECK(std::memcmp(out[i], voices[size_t(i) % 4].data(), sixtype::kVoiceBytes) == 0);
    // maxVoices limits the output.
    CHECK_EQ(sixtype::parseSysex(d.data(), d.size(), out, 5, &ok), 5);
}

TEST(sixtype_parse_rejects_other_data) {
    static uint8_t out[sixtype::kBankVoices][sixtype::kVoiceBytes];
    const uint8_t junk[] = {0xF0, 0x41, 0x10, 0x42, 0x12, 0x00, 0xF7};   // another maker's dump
    CHECK_EQ(sixtype::parseSysex(junk, sizeof junk, out, 32), 0);
    auto d = singleDump(sineVoice());
    CHECK_EQ(sixtype::parseSysex(d.data(), 100, out, 32), 0);   // truncated
    d[3] = 0x05;                                              // another format
    CHECK_EQ(sixtype::parseSysex(d.data(), d.size(), out, 32), 0);
    CHECK_EQ(sixtype::parseSysex(nullptr, 0, out, 32), 0);
}

TEST(sixtype_sanitize_clamps_ranges) {
    Voice v = sineVoice();
    v[0] = 120;     // EG rate
    v[13] = 9;      // rate scaling (max 7)
    v[134] = 40;    // algorithm (max 31)
    v[144] = 60;    // transpose (max 48)
    v[146] = 3;     // control character in the name
    CHECK_EQ(sixtype::sanitizeVoice(v.data()), 5);
    CHECK_EQ(int(v[0]), 99);
    CHECK_EQ(int(v[13]), 7);
    CHECK_EQ(int(v[134]), 31);
    CHECK_EQ(int(v[144]), 48);
    CHECK_EQ(int(v[146]), int(' '));
    CHECK_EQ(sixtype::sanitizeVoice(v.data()), 0);
    // A voice with every byte at 127 loads and plays (parseSysex sanitises).
    Voice j(sixtype::kVoiceBytes, 127);
    auto d = singleDump(j);
    uint8_t out[1][sixtype::kVoiceBytes];
    CHECK_EQ(sixtype::parseSysex(d.data(), d.size(), out, 1), 1);
    Rig rig(Voice(out[0], out[0] + sixtype::kVoiceBytes));
    rig.on(0, 60.0f);
    rig.run(kBlocksPerSec / 2);
    CHECK(finite(rig.L));
}

TEST(sixtype_pitch_ratio_fixed_transpose_bend) {
    const size_t a = size_t(kSampleRate * 0.2), b = size_t(kSampleRate * 1.2);
    for (float note : {36.0f, 60.0f, 81.0f}) {
        Rig rig(sineVoice());
        rig.on(0, note);
        rig.run(int(1.3f * kBlocksPerSec));
        CHECK_NEAR(zeroCrossHz(rig.L, a, b), midiHz(note), midiHz(note) * 0.002);
        CHECK(rig.L == rig.R);   // mono
    }
    {   // ratio 2 (coarse 2), and 0.5 (coarse 0)
        Voice v = sineVoice();
        v[5 * 21 + 18] = 2;
        Rig rig(v);
        rig.on(0, 60.0f);
        rig.run(int(1.3f * kBlocksPerSec));
        CHECK_NEAR(zeroCrossHz(rig.L, a, b), 2.0 * midiHz(60), 2.0);
        v[5 * 21 + 18] = 0;
        Rig half(v);
        half.on(0, 60.0f);
        half.run(int(1.3f * kBlocksPerSec));
        CHECK_NEAR(zeroCrossHz(half.L, a, b), 0.5 * midiHz(60), 0.5);
    }
    {   // fixed frequency: coarse 2 (100 Hz) fine 0, any note
        Voice v = sineVoice();
        v[5 * 21 + 17] = 1;
        v[5 * 21 + 18] = 2;
        for (float note : {40.0f, 70.0f}) {
            Rig rig(v);
            rig.on(0, note);
            rig.run(int(1.3f * kBlocksPerSec));
            CHECK_NEAR(zeroCrossHz(rig.L, a, b), 100.0, 0.5);
        }
    }
    {   // transpose +12 (36 = C4 + 1 octave)
        Voice v = sineVoice();
        v[144] = 36;
        Rig rig(v);
        rig.on(0, 60.0f);
        rig.run(int(1.3f * kBlocksPerSec));
        CHECK_NEAR(zeroCrossHz(rig.L, a, b), midiHz(72), 1.0);
    }
    {   // framework bend: +7 semitones
        Rig rig(sineVoice());
        rig.on(0, 60.0f);
        rig.v[0].bend = rig.v[0].bendStart = 7.0f;
        rig.run(int(1.3f * kBlocksPerSec));
        CHECK_NEAR(zeroCrossHz(rig.L, a, b), midiHz(67), 1.0);
    }
}

TEST(sixtype_level_and_velocity) {
    // One full-level carrier peaks at 0.25 (kOutScale).
    Rig rig(sineVoice());
    rig.on(0, 60.0f);
    rig.run(kBlocksPerSec);
    const float pk = peak(rig.L, size_t(kSampleRate * 0.5));
    CHECK_NEAR(pk, 0.25, 0.02);
    CHECK(rig.v[0].level > 0.4f);
    // Velocity sensitivity 7 on the carrier: velocity 0.3 is quieter; with
    // sensitivity 0 velocity changes nothing.
    Voice v = sineVoice();
    v[5 * 21 + 15] = 7;
    Rig loud(v), soft(v);
    loud.on(0, 60.0f, 1.0f);
    loud.run(kBlocksPerSec);
    soft.on(0, 60.0f, 0.3f);
    soft.run(kBlocksPerSec);
    CHECK(rms(soft.L) < 0.5 * rms(loud.L));
    Rig flat(sineVoice());
    flat.on(0, 60.0f, 0.3f);
    flat.run(kBlocksPerSec);
    CHECK_NEAR(rms(flat.L), rms(rig.L), 1e-6);
}

TEST(sixtype_voice_ends_after_release) {
    Rig rig(sineVoice());
    rig.on(0, 60.0f);
    CHECK_EQ(rig.run(kBlocksPerSec / 2), 1);
    rig.off(0);
    int live = 1, blocks = 0;
    while (live && blocks < 10 * kBlocksPerSec) {
        live = rig.run(1);
        ++blocks;
    }
    CHECK_EQ(live, 0);
    CHECK(blocks < 3 * kBlocksPerSec);   // release rate 70: well under 3 s
    // Held, the voice keeps sounding.
    Rig held(sineVoice());
    held.on(0, 60.0f);
    CHECK_EQ(held.run(3 * kBlocksPerSec), 1);
    CHECK(rms(held.L, held.L.size() - kBlock * 50) > 0.1);
}

// Largest sample-to-sample step over [a, b).
float maxStep(const Buf& x, size_t a, size_t b) {
    float m = 0.0f;
    for (size_t i = std::max<size_t>(a, 1); i < std::min(b, x.size()); ++i) m = std::max(m, std::fabs(x[i] - x[i - 1]));
    return m;
}

TEST(sixtype_mono_retrigger_restarts_without_click) {
    // Default Mono = Retrigger: a reused, still-held voice restarts its
    // envelopes from 0 like a new voice (a slow attack starts over), and the
    // gains ramp down over one control step instead of jumping.
    Voice v = sineVoice();
    v[5 * 21 + 0] = 30;   // slow attack
    Rig rig(v);
    rig.on(0, 60.0f);
    rig.run(kBlocksPerSec);
    const float before = rig.v[0].level;
    const size_t at = rig.L.size();
    const float settled = maxStep(rig.L, at - kBlock, at);
    rig.on(0, 64.0f, 1.0f, true, true);
    rig.run(2);
    CHECK(rig.v[0].level < before * 0.5f);
    CHECK(maxStep(rig.L, at - 1, at + 2 * kBlock) <= settled * 1.5f);
}

TEST(sixtype_mono_legato_overlap_moves_pitch_only) {
    // Mono = Legato: a note that overlaps the held one keeps the envelopes
    // (no drop) and only moves the pitch.
    Voice v = sineVoice();
    v[5 * 21 + 0] = 30;
    Rig rig(v);
    rig.set(kSixtypeMono, 1.0f);
    rig.on(0, 60.0f);
    rig.run(kBlocksPerSec);
    const float before = rig.v[0].level;
    const size_t at = rig.L.size();
    rig.on(0, 72.0f, 1.0f, true, true);
    rig.run(kBlocksPerSec / 2);
    CHECK(rig.v[0].level >= before * 0.9f);
    CHECK_NEAR(zeroCrossHz(rig.L, at + kBlock, rig.L.size()), midiHz(72), midiHz(72) * 0.01);
    // After a release (no overlap) Legato restarts from where the envelopes
    // are: no drop to 0 either.
    rig.off(0);
    rig.run(4);
    const float released = rig.v[0].level;
    rig.on(0, 60.0f, 1.0f, true, false);
    rig.run(1);
    CHECK(rig.v[0].level >= released * 0.5f);
}

// Brightness of one rendering of `voice` at note 48 with control i = x.
double brightAt(const Voice& voice, int i, float x) {
    Rig rig(voice);
    rig.set(i, x);
    rig.on(0, 48.0f);
    rig.run(kBlocksPerSec);
    return brightness(rig.L, size_t(kSampleRate * 0.3), size_t(kSampleRate * 0.8), midiHz(48));
}

TEST(sixtype_brightness_scales_modulators_only) {
    // Brightness raises / lowers the modulator levels: more / fewer
    // harmonics. A voice without modulators doesn't change.
    Voice fm = fmVoice(0, 0);
    fm[4 * 21 + 16] = 60;   // a light modulator (not saturated at 0)
    const double mid = brightAt(fm, kSixtypeBrightness, 0.0f);
    // (Relative: the harmonic share isn't monotonic once FM is heavy.)
    CHECK(brightAt(fm, kSixtypeBrightness, 1.0f) > mid * 2.0);
    CHECK(brightAt(fm, kSixtypeBrightness, -1.0f) < mid * 0.5);
    const Voice sine = sineVoice();
    CHECK_NEAR(brightAt(sine, kSixtypeBrightness, 1.0f), brightAt(sine, kSixtypeBrightness, 0.0f), 1e-6);
}

TEST(sixtype_feedback_control_adds_to_the_voice) {
    // Operator 6 alone on algorithm 32 (its feedback loop), feedback 0 in the
    // voice: Feedback +7 makes it brighter, -7 on a voice at 7 removes it.
    Voice v = sineVoice();
    v[5 * 21 + 16] = 0;
    v[0 * 21 + 16] = 99;
    const double none = brightAt(v, kSixtypeFeedback, 0.0f);
    CHECK(none < 0.05);
    CHECK(brightAt(v, kSixtypeFeedback, 7.0f) > none + 0.1);
    v[135] = 7;
    CHECK_NEAR(brightAt(v, kSixtypeFeedback, -7.0f), none, 1e-6);
}

// Frames until the voice level first reaches `frac` of its later peak.
size_t riseFrames(const Buf& x, float frac) {
    const float pk = peak(x);
    for (size_t i = 0; i < x.size(); ++i)
        if (std::fabs(x[i]) >= pk * frac) return i;
    return x.size();
}

TEST(sixtype_attack_and_release_offset_the_rates) {
    Voice v = sineVoice();
    v[5 * 21 + 0] = 60;   // operator 1 attack rate
    v[5 * 21 + 3] = 50;   // release rate
    auto rise = [&](float attack) {
        Rig rig(v);
        rig.set(kSixtypeAttack, attack);
        rig.on(0, 60.0f);
        rig.run(kBlocksPerSec);
        return riseFrames(rig.L, 0.5f);
    };
    CHECK(rise(1.0f) > rise(0.0f) * 2);   // + = slower
    CHECK(rise(-1.0f) < rise(0.0f));
    auto tail = [&](float release) {
        Rig rig(v);
        rig.set(kSixtypeRelease, release);
        rig.on(0, 60.0f);
        rig.run(kBlocksPerSec / 2);
        rig.off(0);
        const size_t at = rig.L.size();
        rig.run(kBlocksPerSec);
        return rms(rig.L, at + size_t(kSampleRate * 0.2), at + size_t(kSampleRate * 0.4));
    };
    CHECK(tail(1.0f) > tail(0.0f) * 2.0);   // + = longer
    CHECK(tail(-1.0f) < tail(0.0f));
}

// Spread of the zero-crossing frequency over 20 ms windows (vibrato).
double pitchWobble(const Buf& x, size_t a, size_t b) {
    const size_t w = size_t(kSampleRate * 0.02);
    double lo = 1e9, hi = 0.0;
    for (size_t i = a; i + w <= b; i += w) {
        const double f = zeroCrossHz(x, i, i + w);
        lo = std::min(lo, f);
        hi = std::max(hi, f);
    }
    return hi - lo;
}

TEST(sixtype_lfo_depth_scales_vibrato) {
    Voice v = sineVoice();
    v[137] = 40;   // LFO speed
    v[138] = 0;    // no delay
    v[139] = 40;   // PMD
    v[142] = 0;    // triangle
    v[143] = 3;    // PMS
    auto wobble = [&](float depth) {
        Rig rig(v);
        rig.set(kSixtypeLfoDepth, depth);
        rig.on(0, 72.0f);
        rig.run(kBlocksPerSec * 2);
        return pitchWobble(rig.L, size_t(kSampleRate * 0.5), rig.L.size());
    };
    const double one = wobble(1.0f);
    CHECK(one > 5.0);
    CHECK(wobble(2.0f) > one * 1.5);
    CHECK(wobble(0.0f) < 2.0);   // zero-crossing noise only
}

// Spread of the RMS over 50 ms windows (beating between detuned operators).
double beating(const Buf& x, size_t a, size_t b) {
    const size_t w = size_t(kSampleRate * 0.05);
    double lo = 1e9, hi = 0.0;
    for (size_t i = a; i + w <= b; i += w) {
        const double r = rms(x, i, i + w);
        lo = std::min(lo, r);
        hi = std::max(hi, r);
    }
    return hi > 0.0 ? (hi - lo) / hi : 0.0;
}

TEST(sixtype_detune_spreads_the_operators) {
    // Two carriers at ratio 1 (operators 1 and 2): in tune they don't beat;
    // Detune 1 pulls them apart (beating).
    Voice v = sineVoice();
    v[4 * 21 + 16] = 99;
    auto beat = [&](float detune) {
        Rig rig(v);
        rig.set(kSixtypeDetune, detune);
        rig.on(0, 60.0f);
        rig.run(kBlocksPerSec * 2);
        return beating(rig.L, size_t(kSampleRate * 0.3), rig.L.size());
    };
    CHECK(beat(0.0f) < 0.05);
    CHECK(beat(1.0f) > 0.3);
}

TEST(sixtype_init_patch_macros_aim_at_the_controls) {
    SynthPatch p;
    CHECK(patchInit(sixtype::kEngineId, &p));
    CHECK_EQ(int(p.paramCount), int(kSixtypeParamCount));
    const int target[6] = {kSixtypeBrightness, kSixtypeFeedback, kSixtypeAttack, kSixtypeRelease, kSixtypeLfoDepth, kSixtypeDetune};
    const char* name[6] = {"Bright", "Feedback", "Attack", "Release", "LFO Depth", "Detune"};
    for (int k = 0; k < 6; ++k) {
        CHECK_EQ(int(p.macros[k].count), 1);
        CHECK_EQ(int(p.macros[k].targets[0].param), target[k]);
        CHECK(std::strcmp(p.macros[k].name, name[k]) == 0);
        CHECK(p.macros[k].bipolar == (k != 5));
    }
}

TEST(sixtype_modulation_feedback_and_algorithms_4_6) {
    const size_t a = size_t(kSampleRate * 0.3), b = size_t(kSampleRate * 0.8);
    const double f0 = midiHz(48);
    Rig plain(sineVoice());
    plain.on(0, 48.0f);
    plain.run(kBlocksPerSec);
    Rig fm(fmVoice(0, 0));
    fm.on(0, 48.0f);
    fm.run(kBlocksPerSec);
    const double bPlain = brightness(plain.L, a, b, f0), bFm = brightness(fm.L, a, b, f0);
    CHECK(bPlain < 0.05);
    CHECK(bFm > bPlain + 0.1);
    // Algorithm 32: operator 6 feeds back on itself (a carrier here);
    // algorithms 4 / 6: the loop runs through operators 6 -> 5 -> 4 / 6 -> 5
    // (the last one the carrier). Feedback reshapes the wave, all bounded.
    for (int alg : {31, 3, 5}) {
        Voice v = sineVoice();
        v[5 * 21 + 16] = 0;   // operator 1 off: only the loop sounds
        if (alg == 31) {
            v[0 * 21 + 16] = 99;
        } else if (alg == 3) {
            v[0 * 21 + 16] = 60;
            v[1 * 21 + 16] = 60;
            v[2 * 21 + 16] = 99;
        } else {
            v[0 * 21 + 16] = 60;
            v[1 * 21 + 16] = 99;
        }
        v[134] = uint8_t(alg);
        Buf none;
        for (int fb : {0, 4, 7}) {
            v[135] = uint8_t(fb);
            Rig rig(v);
            rig.on(0, 48.0f);
            rig.run(kBlocksPerSec);
            CHECK(finite(rig.L));
            CHECK(peak(rig.L) > 0.05f);
            CHECK(peak(rig.L) < 1.0f);
            if (fb == 0) {
                none = rig.L;
                continue;
            }
            // The feedback reshapes the spectrum (more harmonics; in algorithm 6
            // it moves energy from the 2nd to the 3rd, as reference renders do).
            const auto h = harmonicsDb(rig.L, a, b, f0), h0 = harmonicsDb(none, a, b, f0);
            double moved = 0.0;
            for (size_t k = 1; k < h.size(); ++k) moved = std::max(moved, std::fabs(h[k] - h0[k]));
            CHECK(moved > 3.0);
            if (alg == 31 && fb == 7) CHECK(brightness(rig.L, a, b, f0) > brightness(none, a, b, f0) + 0.1);
        }
    }
}

TEST(sixtype_amplitude_modulation) {
    // AMS 3 + AMD 99, triangle LFO at speed 50, no delay: the level swings.
    Voice v = sineVoice();
    v[5 * 21 + 14] = 3;
    v[137] = 50;
    v[138] = 0;
    v[140] = 99;
    Rig am(v), flat(sineVoice());
    am.on(0, 60.0f);
    am.run(2 * kBlocksPerSec);
    flat.on(0, 60.0f);
    flat.run(2 * kBlocksPerSec);
    float lo = 1.0f, hi = 0.0f;
    for (size_t s = size_t(kSampleRate * 0.3); s + 2048 < am.L.size(); s += 1024) {
        const float p = peak(am.L, s, s + 1024);
        lo = std::min(lo, p);
        hi = std::max(hi, p);
    }
    CHECK(hi > 4.0f * lo);
    CHECK(hi <= peak(flat.L) * 1.01f);
    // AMS 0 on the operator: AMD has no effect.
    v[5 * 21 + 14] = 0;
    Rig none(v);
    none.on(0, 60.0f);
    none.run(2 * kBlocksPerSec);
    CHECK_NEAR(rms(none.L), rms(flat.L), 1e-6);
}

TEST(sixtype_polyphony_and_random_voices) {
    // Four notes sum.
    Rig one(sineVoice()), four(sineVoice());
    one.on(0, 60.0f);
    one.run(kBlocksPerSec / 2);
    for (int k = 0; k < 4; ++k) four.on(k, 60.0f + 4.0f * float(k));
    four.run(kBlocksPerSec / 2);
    CHECK(rms(four.L) > 1.6 * rms(one.L));
    // Random (sanitised) voices: finite, bounded, and they all end after the
    // release unless their release level stays up.
    uint32_t seed = 12345;
    auto rnd = [&]() { return (seed = seed * 1664525u + 1013904223u) >> 24; };
    for (int n = 0; n < 60; ++n) {
        Voice v(sixtype::kVoiceBytes);
        for (auto& x : v) x = uint8_t(rnd() & 0x7f);
        sixtype::sanitizeVoice(v.data());
        Rig rig(v);
        rig.on(0, float(30 + rnd() % 60), float(rnd() % 128) / 127.0f);
        rig.on(1, float(30 + rnd() % 60), 1.0f);
        rig.run(kBlocksPerSec / 3);
        rig.off(0);
        rig.off(1);
        rig.run(kBlocksPerSec / 3);
        CHECK(finite(rig.L));
        CHECK(peak(rig.L) < 4.0f);   // two voices, six carriers at most
    }
}

TEST(sixtype_no_voice_loaded_is_silent) {
    ISynthEngine* e = sixtype::engine();
    SynthSlotScratch sc{};
    e->slotReset(sc);
    SynthVoice v{};
    v.slotScratch = &sc;
    SynthParams p;
    SynthNote n;
    e->noteOn(v, p, n);
    float l[kBlock] = {}, r[kBlock] = {};
    e->prepare(sc, p);
    CHECK(!e->render(v, p, 1.0f, 0.0f, l, r));
    CHECK_STR(e->name(), "6TYPE");
    CHECK_EQ(e->id(), int32_t(sixtype::kEngineId));
}

// Every voice of the .syx files in $OT_SIXTYPE_SYX (e.g. a folder of
// downloaded banks): parses, plays three notes and a chord cleanly, and ends.
TEST(sixtype_syx_directory) {
    const char* dir = std::getenv("OT_SIXTYPE_SYX");
    if (!dir || !fs::is_directory(dir)) return;
    int files = 0, voices = 0;
    for (const auto& ent : fs::directory_iterator(dir)) {
        if (ent.path().extension() != ".syx") continue;
        std::ifstream in(ent.path(), std::ios::binary);
        std::vector<uint8_t> d((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        static uint8_t out[sixtype::kBankVoices][sixtype::kVoiceBytes];
        const int n = sixtype::parseSysex(d.data(), d.size(), out, sixtype::kBankVoices);
        CHECK(n > 0);
        ++files;
        for (int i = 0; i < n; ++i, ++voices) {
            Rig rig(Voice(out[i], out[i] + sixtype::kVoiceBytes));
            for (int k = 0; k < 3; ++k) rig.on(k, 48.0f + 12.0f * float(k), 0.8f);
            rig.run(kBlocksPerSec);
            for (int k = 0; k < 3; ++k) rig.off(k);
            rig.run(2 * kBlocksPerSec);
            CHECK(finite(rig.L));
            CHECK(peak(rig.L) > 0.01f);
            CHECK(peak(rig.L) < 3.0f);
        }
    }
    std::printf("    sixtype_syx_directory: %d files, %d voices\n", files, voices);
    CHECK(files > 0);
}
