// ref1type: the 1-Type reference (host only, C++20). Renders the cases of the
// 1-Type engine test through the source engine's voice (Oxo Synth2's
// EngineCore, in double at 44.1 kHz; effects, arp and unison off: the voice,
// its filter and the two LFOs only), measures their features with
// tests/onetype_features.h and writes tests/onetype_ref.inc. Each case is
// defined in the source engine's own knob values; the tool converts the
// patch to 1-Type's engineering units (the knob laws below) and writes the
// parameters that differ from the neutral patch.
//
//   make OXO=<oxo_synth2 checkout at 6bb4435>          -> ../../tests/onetype_ref.inc
//   make OXO=... tables                                 -> the engine's tables (stdout)
//
// Derived from Oxo Synth2 (https://github.com/chab/oxo_synth2, commit 6bb4435):
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
#include "EngineCore/Lfo.hpp"
#include "EngineCore/Voice.hpp"

#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "../../src/onetype_params.h"
#include "../../tests/onetype_features.h"
#include "../sy1/sy1_map.h"

namespace {

namespace s2 = oxo::s2;
namespace tb = oxo::s2::tables;
namespace ty = ot::audio::onetype;
namespace rf = onetype_ref;

constexpr double kFs = 44100.0;

// --- knob laws: the source's patch values -> 1-Type's units ----------------------------

double svfHz(int knob) { return 16.7 * std::exp2(knob / 12.624); }
double cutoffHz(int type, int knob) { return type == 1 ? double(tb::kLadderCutoffHz[knob]) : svfHz(knob); }
// Saturation: a = aMax (2^(10 s) - 1) / 1023 (as tools/sy1/sy1_map.h).
double satUnits(int v) { return onetype_sy1::satUnits(v); }
double lfoHz(int v) { return s2::Lfo::SpeedHz(v); }

// The source patch in 1-Type units: the converter's mapping (tools/sy1/sy1_map.h).
std::vector<float> ours(const s2::PatchValues& v) {
    int32_t raw[s2::kParamCount];
    for (int i = 0; i < s2::kParamCount; ++i) raw[i] = v[i];
    std::vector<float> o(ty::kParamCount, 0.0f);
    onetype_sy1::toOnetype(raw, o.data());
    return o;
}

// The neutral patch: osc 1 saw alone, LP12 open, the envelopes flat, the
// LFOs off, oscillators reset at the note (phase 1: osc 1 and 2 at 0).
s2::PatchValues neutral() {
    using namespace oxo::s2;
    PatchValues n = DefaultValues();
    const std::pair<int, int> v[] = {
        {kOsc1Shape, 1},      {kOsc2Shape, 1},     {kOsc2Pitch, 64},      {kOsc2Fine, 64},     {kOsc2KbdTrack, 1},
        {kOscMix, 0},         {kOsc2Sync, 0},      {kOsc2Ring, 0},        {kOscPulseWidth, 127}, {kOscKeyShift, 0},
        {kModEnvOn, 0},       {kModEnvAmount, 64}, {kModEnvAttack, 0},    {kModEnvDecay, 0},   {kFilterType, 0},
        {kFilterAttack, 0},   {kFilterDecay, 0},   {kFilterSustain, 127}, {kFilterRelease, 0}, {kFilterFreq, 127},
        {kFilterRes, 0},      {kFilterAmount, 64}, {kFilterKbdTrack, 0},  {kFilterSat, 0},     {kFilterVelSwitch, 0},
        {kAmpAttack, 0},      {kAmpDecay, 64},     {kAmpSustain, 127},    {kAmpRelease, 0},    {kAmpGain, 64},
        {kAmpVelSens, 0},     {kArpOn, 0},         {kDelayOn, 0},         {kChorusOn, 0},      {kEffectOn, 0},
        {kPlayMode, 0},       {kPortaTime, 0},     {kPortaAuto, 0},       {kLfo1On, 0},        {kLfo2On, 0},
        {kLfo1Depth, 0},      {kLfo2Depth, 0},     {kLfo1Dest, 3},        {kLfo2Dest, 3},      {kLfo1TempoSync, 0},
        {kLfo2TempoSync, 0},  {kLfo1KeySync, 0},   {kLfo2KeySync, 0},     {kOsc1FM, 0},        {kModEnvDest, 0},
        {kOscFineTune, 64},   {kUnisonOn, 0},      {kOsc1Detune, 0},      {kPan, 64},          {kOscPhase, 1},
        {kSubGain, 0},        {kSubShape, 0},      {kSubOctave, 1},       {kEqTone, 64},       {kEqLevel, 64},
    };
    for (const auto& [id, x] : v) n[id] = x;
    return n;
}

int pitchKnob(int semis) {
    for (int i = 0; i < 128; ++i)
        if (tb::kOsc2PitchSemis[i] == semis) return i;
    return 64;
}
int fineKnob(int cents) {
    for (int i = 0; i < 128; ++i)
        if (tb::kFineCents[i] == cents) return i;
    return 64;
}
double keyHz(double note) { return 440.0 * std::exp2((note - 69.0) / 12.0); }

// --- rendering ----------------------------------------------------------------------

struct Def {
    std::string name;
    uint8_t kind = rf::kPitch;
    int key = 60, key2 = -1;
    uint8_t flags = 0;
    double t2 = -1, tOff = -1, dur = 1.0;
    float tempo = 120.0f;
    std::vector<std::pair<int, int>> nat;   // the source's patch values
    std::vector<float> arg;
    // kernel kinds: filter type, cutoff knob, resonance knob, saturation knob
    int ftype = 0, fknob = 127, fres = 0, fsat = 0;
};

int blocksOf(double t) { return t < 0 ? -1 : int(std::lround(t * kFs / rf::kBlockFrames)); }

// One voice of patch p as the source engine plays it (Synth::Render without
// the effects): the LFOs rendered per 64-frame chunk, key 2 on the same voice
// (mono: retrigger, legato: glide), the note off, the LFO pan after the voice.
void renderVoice(const Def& d, const s2::PatchValues& p, std::vector<float>& L, std::vector<float>& R) {
    using namespace oxo::s2;
    const int total = blocksOf(d.dur) * rf::kBlockFrames;
    L.assign(size_t(total), 0.0f);
    R.assign(size_t(total), 0.0f);
    Voice v;
    v.Prepare(kFs);
    v.InitSlot(0);
    Lfo lfo[2];
    if (p[kLfo1KeySync]) lfo[0].Restart();
    if (p[kLfo2KeySync]) lfo[1].Restart();
    const bool porta = p[kPortaTime] > 0;
    v.Start(d.key, 127, porta && !p[kPortaAuto], p, 0);
    const int at2 = d.t2 < 0 ? -1 : blocksOf(d.t2) * rf::kBlockFrames, atOff = d.tOff < 0 ? -1 : blocksOf(d.tOff) * rf::kBlockFrames;
    double buf[2][64];
    int32_t steps[2][64];
    for (int pos = 0; pos < total; pos += 64) {
        if (pos == at2) {
            const bool mono = p[kPlayMode] == 1;
            v.ChangeNote(d.key2, 127, mono, porta);
            if (mono) v.StartPhases(p);
        }
        if (pos == atOff) v.Release();
        const int n = std::min(64, total - pos);
        const double* lp[2] = {nullptr, nullptr};
        const int32_t* sp[2] = {nullptr, nullptr};
        for (int k = 0; k < 2; ++k) {
            const int speed = p[k ? kLfo2Speed : kLfo1Speed];
            const double hz = p[k ? kLfo2TempoSync : kLfo1TempoSync] ? d.tempo / 60.0 / Lfo::SyncBeats(speed) : Lfo::SpeedHz(speed);
            const int dest = p[k ? kLfo2Dest : kLfo1Dest];
            const bool on = p[k ? kLfo2On : kLfo1On], pitch = on && dest <= 2;
            lfo[k].Render(buf[k], n, p[k ? kLfo2Type : kLfo1Type], hz, kFs, pitch ? steps[k] : nullptr,
                          Lfo::PitchSteps(p[k ? kLfo2Depth : kLfo1Depth]), dest == 4);
            if (on) lp[k] = buf[k];
            sp[k] = pitch ? steps[k] : nullptr;
        }
        v.Render(L.data() + pos, R.data() + pos, n, pos, INT64_MAX, p, 0.0, lp, sp, p[kFilterFreq], 0.0);
        // The LFOs on pan (Synth::LfoPanGains): after the voice.
        double sc[2];
        const double* pv[2];
        int routes = 0;
        for (int k = 0; k < 2; ++k)
            if (Voice::LfoDest(p, lp, k) == 7) pv[routes] = lp[k], sc[routes++] = p[k ? kLfo2Depth : kLfo1Depth] / 127.0;
        for (int i = 0; i < n && routes; ++i) {
            double s = 0;
            for (int k = 0; k < routes; ++k) s += sc[k] * pv[k][i];
            const double u = 1.0 - std::fabs(s), cube = u * u * u, q = s >= 0 ? 1.0 - cube : cube - 1.0;
            L[size_t(pos + i)] = float(L[size_t(pos + i)] * (q >= 0 ? 1.0 + 0.5 * q : 1.0 + q));
            R[size_t(pos + i)] = float(R[size_t(pos + i)] * (q >= 0 ? 1.0 - q : 1.0 - 0.5 * q));
        }
    }
}

// The filter section of the source voice (filter, its output limiter, the saturation).
struct Section {
    s2::VoiceFilter f;
    int type, knob, res;
    double a;
    Section(int t, int k, int r, int sat) : type(t), knob(k), res(r), a(tb::kSaturationA[sat]) {
        f.Prepare(kFs);
        f.Reset();
    }
    double operator()(double x) {
        f.SetParams(type, knob, res);
        return s2::Saturate(f.Process(x), a);
    }
};

std::vector<Def> defs() {
    using namespace oxo::s2;
    std::vector<Def> v;
    const auto add = [&](Def d) { v.push_back(std::move(d)); };
    // --- M1: pitch -------------------------------------------------------------------
    for (int key : {24, 48, 72, 96}) add({"pitch_osc1_k" + std::to_string(key), rf::kPitch, key, -1, 0, -1, -1, 1.05, 120, {}, {float(keyHz(key)), 0.2f, 1.0f}});
    const auto osc2 = [&](const char* name, int key, int semis, int cents, int track, int shift, int tune) {
        const double base = track ? key + shift + tune / 100.0 : 57.0 + tune / 100.0;
        add({name, rf::kPitch, key, -1, 0, -1, -1, 1.05, 120,
             {{kOscMix, 127}, {kOsc2Pitch, pitchKnob(semis)}, {kOsc2Fine, fineKnob(cents)}, {kOsc2KbdTrack, track},
              {kOscKeyShift, shift}, {kOscFineTune, fineKnob(tune)}},
             {float(keyHz(base + semis + cents / 100.0)), 0.2f, 1.0f}});
    };
    osc2("pitch_osc2_m12_k48", 48, -12, 0, 1, 0, 0);
    osc2("pitch_osc2_p7_k48", 48, 7, 0, 1, 0, 0);
    osc2("pitch_osc2_p19_k60", 60, 19, 0, 1, 0, 0);
    osc2("pitch_osc2_fine_m30", 60, 0, -30, 1, 0, 0);
    osc2("pitch_osc2_fine_p45", 60, 0, 45, 1, 0, 0);
    osc2("pitch_osc2_notrack_k24", 24, 0, 0, 0, 0, 0);
    osc2("pitch_osc2_notrack_k72", 72, 0, 0, 0, 0, 0);
    osc2("pitch_osc2_notrack_tune", 72, 0, 0, 0, 0, 40);
    osc2("pitch_osc2_shift_p7", 60, 5, 0, 1, 7, 0);
    const auto osc1 = [&](const char* name, int key, int shift, int tune) {
        add({name, rf::kPitch, key, -1, 0, -1, -1, 1.05, 120, {{kOscKeyShift, shift}, {kOscFineTune, fineKnob(tune)}},
             {float(keyHz(key + shift + tune / 100.0)), 0.2f, 1.0f}});
    };
    osc1("pitch_osc1_shift_m12", 60, -12, 0);
    osc1("pitch_osc1_shift_p7", 60, 7, 0);
    osc1("pitch_osc1_tune_m40", 60, 0, -40);
    osc1("pitch_osc1_tune_p40", 60, 0, 40);
    // --- M1: harmonics ---------------------------------------------------------------
    const auto harm = [&](std::string name, int key, std::vector<std::pair<int, int>> nat, double f0, uint8_t flags = 0) {
        add({name, rf::kHarm, key, -1, uint8_t(rf::kFlagLevel | flags), -1, -1, 0.75, 120, nat, {float(f0), 0.2f, 0.7f, 16.0f}});
    };
    harm("harm_osc1_sine_k60", 60, {{kOsc1Shape, 0}}, keyHz(60), rf::kFlagRef);
    for (int key : {36, 60, 84}) {
        const std::string k = "_k" + std::to_string(key);
        if (key != 60) harm("harm_osc1_sine" + k, key, {{kOsc1Shape, 0}}, keyHz(key));
        harm("harm_osc1_saw" + k, key, {{kOsc1Shape, 1}}, keyHz(key));
        harm("harm_osc1_pw10" + k, key, {{kOsc1Shape, 2}, {kOscPulseWidth, 13}}, keyHz(key));
        harm("harm_osc1_pw50" + k, key, {{kOsc1Shape, 2}, {kOscPulseWidth, 64}}, keyHz(key));
        harm("harm_osc1_pw90" + k, key, {{kOsc1Shape, 2}, {kOscPulseWidth, 114}}, keyHz(key));
        harm("harm_osc1_tri" + k, key, {{kOsc1Shape, 3}}, keyHz(key));
    }
    harm("harm_osc2_saw_k60", 60, {{kOscMix, 127}, {kOsc2Shape, 1}}, keyHz(60));
    harm("harm_osc2_pulse_k60", 60, {{kOscMix, 127}, {kOsc2Shape, 2}, {kOscPulseWidth, 64}}, keyHz(60));
    harm("harm_osc2_tri_k60", 60, {{kOscMix, 127}, {kOsc2Shape, 3}}, keyHz(60));
    static const char* const kSub[4] = {"sine", "tri", "saw", "square"};
    for (int s = 0; s < 4; ++s)
        for (int oct : {0, 1})
            harm(std::string("harm_sub_") + kSub[s] + (oct ? "_down" : "_same"), 60,
                 {{kOsc1Shape, 0}, {kSubGain, 127}, {kSubShape, s}, {kSubOctave, oct}}, keyHz(oct ? 48 : 60));
    // --- M1: the mix law (osc 1 saw, osc 2 saw a fourth up, the square sub an octave down)
    for (int sub : {0, 32, 127})
        for (int mix : {0, 32, 64, 96, 127})
            add({"mix_m" + std::to_string(mix) + "_s" + std::to_string(sub), rf::kTones, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
                 {{kOscMix, mix}, {kOsc2Pitch, pitchKnob(5)}, {kSubGain, sub}, {kSubShape, 3}, {kSubOctave, 1}},
                 {1, 0.2f, 0.7f, float(keyHz(48)), float(keyHz(53)), float(keyHz(36))}});
    // --- M1: noise ------------------------------------------------------------------------
    add({"noise_rms", rf::kRmsCent, 60, -1, rf::kFlagLevel, -1, -1, 0.75, 120, {{kOscMix, 127}, {kOsc2Shape, 4}}, {0.2f, 0.7f, 15000}});
    // --- M1: the amp envelope (a square at key 0: flat stretches show the level) ----------
    const std::vector<std::pair<int, int>> sq = {{kOsc1Shape, 2}, {kOscPulseWidth, 127}};
    const auto env = [&](std::string name, std::vector<std::pair<int, int>> nat, double dur, double tOff, float p0, float p1) {
        std::vector<std::pair<int, int>> all = sq;
        all.insert(all.end(), nat.begin(), nat.end());
        add({name, rf::kEnv, 0, -1, 0, -1, tOff, dur, 120, all, {p0, p1}});
    };
    for (int a : {20, 40, 60, 80, 95}) env("env_attack_" + std::to_string(a), {{kAmpAttack, a}}, tb::kAmpAttackSeconds[a] + 0.3, -1, 0, 0);
    for (int d : {40, 55, 70, 85, 100})
        env("env_decay_" + std::to_string(d), {{kAmpDecay, d}, {kAmpSustain, 0}}, 1.5 * tb::kEnvTauSeconds[d] + 0.1, -1, 0, 0);
    for (int s : {20, 50, 80, 107, 127}) env("env_sustain_" + std::to_string(s), {{kAmpDecay, 30}, {kAmpSustain, s}}, 0.55, -1, 0.3f, 0.5f);
    for (int r : {40, 55, 70, 85, 100})
        env("env_release_" + std::to_string(r), {{kAmpRelease, r}}, 0.3 + 1.5 * tb::kEnvTauSeconds[r] + 0.1, 0.3, 0, 0);
    // --- M2: filter env depth and key track (noise into a resonant BP12) --------------------
    const std::vector<std::pair<int, int>> bp = {{kOscMix, 127}, {kOsc2Shape, 4}, {kFilterType, 3}, {kFilterRes, 100}};
    const auto peak = [&](std::string name, int key, std::vector<std::pair<int, int>> nat) {
        std::vector<std::pair<int, int>> all = bp;
        all.insert(all.end(), nat.begin(), nat.end());
        add({name, rf::kPeakSpec, key, -1, 0, -1, -1, 1.0, 120, all, {20, 18000, 1, 0.1f, 0.95f}});
    };
    for (int amt : {44, 54, 64, 74, 84}) peak("fenv_amount_" + std::to_string(amt), 60, {{kFilterFreq, 50}, {kFilterAmount, amt}});
    for (int kt : {0, 64, 127})
        for (int key : {24, 48, 72, 96})
            peak("keytrack_" + std::to_string(kt) + "_k" + std::to_string(key), key, {{kFilterFreq, 60}, {kFilterKbdTrack, kt}});
    // The same on LP24 (its own knob axis): a resonant LP24's peak on noise.
    const auto peak24 = [&](std::string name, int key, std::vector<std::pair<int, int>> nat) {
        std::vector<std::pair<int, int>> all = {{kOscMix, 127}, {kOsc2Shape, 4}, {kFilterType, 1}, {kFilterRes, 115}};
        all.insert(all.end(), nat.begin(), nat.end());
        add({name, rf::kPeakSpec, key, -1, 0, -1, -1, 1.0, 120, all, {20, 18000, 1, 0.1f, 0.95f}});
    };
    for (int amt : {44, 64, 84, 104})
        peak24("fenv24_amount_" + std::to_string(amt), 60, {{kFilterFreq, 50}, {kFilterAmount, amt}});
    for (int kt : {64, 127})
        for (int key : {24, 72, 96})
            peak24("keytrack24_" + std::to_string(kt) + "_k" + std::to_string(key), key, {{kFilterFreq, 60}, {kFilterKbdTrack, kt}});
    // --- M2: filter responses (kernel) ------------------------------------------------------
    static const char* const kType[5] = {"lp12", "lp24", "hp12", "bp12", "lpdl"};
    for (int type = 0; type < 4; ++type)
        for (int knob = 10; knob <= 120; knob += 10)
            for (int res : {0, 40, 80, 120}) {
                Def d{std::string("filter_") + kType[type] + "_c" + std::to_string(knob) + "_r" + std::to_string(res), rf::kFiltIr};
                d.ftype = type, d.fknob = knob, d.fres = res;
                add(d);
            }
    for (int knob : {20, 40, 60, 80, 100, 120})
        for (int res : {0, 64, 120}) {
            Def d{"filter_lpdl_c" + std::to_string(knob) + "_r" + std::to_string(res), rf::kFiltIr};
            d.ftype = 4, d.fknob = knob, d.fres = res;
            add(d);
        }
    for (int knob : {64, 80, 96}) {
        Def d{"selfosc_lp24_c" + std::to_string(knob), rf::kSelfOsc};
        d.ftype = 1, d.fknob = knob, d.fres = 127;
        add(d);
    }
    for (int sat : {0, 32, 64, 100}) {
        Def d{"limsat_s" + std::to_string(sat), rf::kLimSat};
        d.fsat = sat;
        add(d);
    }
    // --- M3: FM, ring, sync, detune --------------------------------------------------------
    const double f1 = keyHz(60), f2 = keyHz(67);
    for (int fm : {16, 32, 48, 64})
        add({"fm_" + std::to_string(fm), rf::kTones, 60, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
             {{kOsc1Shape, 0}, {kOsc2Shape, 3}, {kOsc2Pitch, pitchKnob(7)}, {kOsc1FM, fm}},
             {1, 0.2f, 0.7f, float(f1), float(f1 + f2), float(f2 - f1), float(f1 + 2 * f2), float(2 * f2 - f1), float(f1 + 3 * f2),
              float(3 * f2 - f1)}});
    for (int semis : {5, 12, 19}) {
        add({"ring_p" + std::to_string(semis), rf::kRmsCent, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
             {{kOscMix, 127}, {kOsc2Ring, 1}, {kOsc2Pitch, pitchKnob(semis)}}, {0.2f, 0.7f, 4000}});
        add({"sync_p" + std::to_string(semis), rf::kRmsCent, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
             {{kOscMix, 127}, {kOsc2Sync, 1}, {kOsc2Pitch, pitchKnob(semis)}}, {0.2f, 0.7f, 10000}});
    }
    // Sync and FM at once (osc 1 steps by osc 2, osc 2 resets on osc 1's drops).
    for (int fm : {12, 24})
        add({"syncfm_" + std::to_string(fm), rf::kRmsCent, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
             {{kOscMix, 127}, {kOsc2Sync, 1}, {kOsc2Pitch, pitchKnob(7)}, {kOsc1Shape, 0}, {kOsc1FM, fm}}, {0.2f, 0.7f, 4000}});
    // Sync and FM where osc 1 steps backward (osc 2 resets on every backward
    // step too: the voice locks): the mean of the output.
    add({"syncback_tri_fm60", rf::kDc, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
         {{kOscMix, 127}, {kOsc2Sync, 1}, {kOsc2Shape, 3}, {kOsc1Shape, 0}, {kOsc1FM, 60}}, {0.2f, 0.7f}});
    add({"syncback_noise_fm70", rf::kDc, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
         {{kOscMix, 127}, {kOsc2Sync, 1}, {kOsc2Shape, 4}, {kOsc1FM, 70}}, {0.2f, 0.7f}});
    add({"ring_fm_pulse", rf::kRmsCent, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
         {{kOscMix, 127}, {kOsc2Ring, 1}, {kOsc2Shape, 2}, {kOscPulseWidth, 64}, {kOsc2Pitch, pitchKnob(7)}, {kOsc1FM, 20}},
         {0.2f, 0.7f, 10000}});
    // Detune: free-running phases (the copies sum incoherently); the level only
    // (the source's 8 copies are 2 here, its copies read truncated tables).
    for (int det : {32, 96})
        add({"detune_" + std::to_string(det), rf::kRmsCent, 48, -1, rf::kFlagLevel, -1, -1, 0.75, 120,
             {{kOsc1Detune, det}, {kOscPhase, 0}}, {0.2f, 0.7f, 10000}});
    // --- M3: the mod envelope --------------------------------------------------------------
    for (int amt : {96, 80, 32})
        add({"modenv_pitch_" + std::to_string(amt), rf::kFreqAt, 60, -1, 0, -1, -1, 0.6, 120,
             {{kOscMix, 127}, {kOsc2Shape, 3}, {kModEnvOn, 1}, {kModEnvAmount, amt}, {kModEnvDecay, 60}, {kModEnvDest, 0}},
             {0.012f, 0.01f, 0.04f, 0.1f, 0.25f, 0.5f}});
    for (const auto& [amt, base] : std::initializer_list<std::pair<int, int>>{{127, 0}, {96, 0}, {32, 64}})
        add({"modenv_fm_" + std::to_string(amt), rf::kTones, 60, -1, rf::kFlagLevel, -1, -1, 0.55, 120,
             {{kOsc1Shape, 0}, {kOsc2Shape, 3}, {kOsc2Pitch, pitchKnob(7)}, {kOsc1FM, base}, {kModEnvOn, 1}, {kModEnvAmount, amt},
              {kModEnvDecay, 70}, {kModEnvDest, 1}},
             {3, 0.02f, 0.07f, 0.15f, 0.2f, 0.45f, 0.5f, float(f1), float(f1 + f2)}});
    // The FM offset's tail: the source's envelope aims 1/2048 under 0, so its
    // dB law falls to 0 with it (decay 50: at 0.25 s).
    add({"modenv_fmtail_127", rf::kTones, 60, -1, rf::kFlagLevel, -1, -1, 0.4, 120,
         {{kOsc1Shape, 0}, {kOsc2Shape, 3}, {kOsc2Pitch, pitchKnob(7)}, {kModEnvOn, 1}, {kModEnvAmount, 127}, {kModEnvDecay, 50},
          {kModEnvDest, 1}},
         {2, 0.15f, 0.21f, 0.222f, 0.252f, float(f1), float(f1 + f2)}});
    for (int amt : {96, 32, 127})
        add({"modenv_pw_" + std::to_string(amt), rf::kDuty, 12, -1, 0, -1, -1, 0.75, 120,
             {{kOsc1Shape, 2}, {kOscPulseWidth, 64}, {kModEnvOn, 1}, {kModEnvAmount, amt}, {kModEnvDecay, 75}, {kModEnvDest, 2}},
             {0.02f, 0.1f, 0.3f, 0.6f}});
    // --- M3: the LFOs ------------------------------------------------------------------------
    static const char* const kWave[6] = {"saw", "tri", "square", "sh", "smooth", "sine"};
    {
        const double hz = lfoHz(55), T = 1.0 / hz;
        std::vector<float> at;
        for (int i = 0; i < 16; ++i) at.push_back(float((i + 0.5) * T / 8.0));   // two cycles, 8 points each
        for (int w = 0; w < 6; ++w) {
            std::vector<std::pair<int, int>> nat = sq;
            for (const auto& x : std::vector<std::pair<int, int>>{
                     {kLfo1On, 1}, {kLfo1Dest, 4}, {kLfo1Type, w}, {kLfo1Speed, 55}, {kLfo1Depth, 127}, {kLfo1KeySync, 1}})
                nat.push_back(x);
            add({std::string("lfo_wave_") + kWave[w], rf::kEnvAt, 0, -1, 0, -1, -1, 2.2 * T, 120, nat, at});
        }
    }
    const auto lfo1 = [](int dest, int depth, int speed = 43) {
        return std::vector<std::pair<int, int>>{{kLfo1On, 1},       {kLfo1Dest, dest},  {kLfo1Type, 2},
                                                {kLfo1Speed, speed}, {kLfo1Depth, depth}, {kLfo1KeySync, 1}};
    };
    const auto cat = [](std::vector<std::pair<int, int>> a, const std::vector<std::pair<int, int>>& b) {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    };
    const std::vector<std::pair<int, int>> sine1 = {{kOsc1Shape, 0}};
    const std::vector<std::pair<int, int>> tri2 = {{kOscMix, 127}, {kOsc2Shape, 3}};
    const std::vector<std::pair<int, int>> pulse12 = {{kOsc1Shape, 2}, {kOscPulseWidth, 64}};
    const std::vector<std::pair<int, int>> fmPair = {{kOsc1Shape, 0}, {kOsc2Shape, 3}, {kOsc2Pitch, pitchKnob(7)}, {kOsc1FM, 30}};
    for (int depth : {32, 64}) {
        const std::string d = "_d" + std::to_string(depth);
        add({"lfo_osc2" + d, rf::kFreqAt, 60, -1, 0, -1, -1, 1.05, 120, cat(tri2, lfo1(1, depth)), {0.1f, 0.25f, 0.75f}});
        add({"lfo_osc12" + d, rf::kFreqAt, 60, -1, 0, -1, -1, 1.05, 120, cat(sine1, lfo1(2, depth)), {0.1f, 0.25f, 0.75f}});
        add({"lfo_filter" + d, rf::kPeakSpec, 60, -1, 0, -1, -1, 1.05, 120, cat(cat(bp, {{kFilterFreq, 50}}), lfo1(3, depth)),
             {20, 18000, 2, 0.05f, 0.5f, 0.56f, 1.01f}});
        add({"lfo_amp" + d, rf::kEnvAt, 0, -1, 0, -1, -1, 1.05, 120, cat(sq, lfo1(4, depth)), {0.25f, 0.75f}});
        add({"lfo_pw" + d, rf::kDuty, 12, -1, 0, -1, -1, 1.05, 120, cat(pulse12, lfo1(5, depth)), {0.2f, 0.7f}});
        add({"lfo_fm" + d, rf::kTones, 60, -1, rf::kFlagLevel, -1, -1, 1.05, 120, cat(fmPair, lfo1(6, depth)),
             {2, 0.05f, 0.48f, 0.56f, 0.99f, float(f1), float(f1 + f2)}});
        add({"lfo_pan" + d, rf::kPanLR, 48, -1, rf::kFlagLevel, -1, -1, 1.05, 120, lfo1(7, depth), {2, 0.05f, 0.48f, 0.56f, 0.99f}});
    }
    // Both LFOs on one destination: LFO 1 a square at ~1 Hz, LFO 2 one at ~2 Hz.
    const auto both = [&](int dest, int d1, int d2) {
        std::vector<std::pair<int, int>> l = lfo1(dest, d1);
        for (const auto& x : std::vector<std::pair<int, int>>{
                 {kLfo2On, 1}, {kLfo2Dest, dest}, {kLfo2Type, 2}, {kLfo2Speed, 55}, {kLfo2Depth, d2}, {kLfo2KeySync, 1}})
            l.push_back(x);
        return l;
    };
    add({"lfo_both_osc12", rf::kFreqAt, 60, -1, 0, -1, -1, 1.05, 120, cat(sine1, both(2, 32, 16)), {0.08f, 0.13f, 0.385f, 0.64f, 0.9f}});
    add({"lfo_both_filter", rf::kPeakSpec, 60, -1, 0, -1, -1, 1.05, 120, cat(cat(bp, {{kFilterFreq, 50}}), both(3, 40, 30)),
         {20, 18000, 4, 0.03f, 0.25f, 0.28f, 0.5f, 0.53f, 0.75f, 0.79f, 1.01f}});
    add({"lfo_both_amp", rf::kEnvAt, 0, -1, 0, -1, -1, 1.05, 120, cat(sq, both(4, 64, 64)), {0.13f, 0.385f, 0.64f, 0.9f}});
    add({"lfo_both_pw", rf::kDuty, 12, -1, 0, -1, -1, 1.05, 120, cat(pulse12, both(5, 40, 40)), {0.1f, 0.33f, 0.58f, 0.82f}});
    add({"lfo_both_pan", rf::kPanLR, 48, -1, rf::kFlagLevel, -1, -1, 1.05, 120, both(7, 64, 32),
         {4, 0.03f, 0.25f, 0.28f, 0.5f, 0.53f, 0.75f, 0.79f, 1.01f}});
    // Rates: free (knobs 50, 70, 90) and tempo synced at 120 BPM (1/4, 1/8, 1/16T).
    for (int s : {50, 70, 90})
        add({"lfo_rate_" + std::to_string(s), rf::kEdges, 0, -1, 0, -1, -1, 2.0, 120, cat(sq, lfo1(4, 127, s)), {0.05f, 1.95f}});
    for (const auto& [s, nm] : std::initializer_list<std::pair<int, const char*>>{{77, "q"}, {94, "8th"}, {111, "16t"}}) {
        std::vector<std::pair<int, int>> nat = cat(sq, lfo1(4, 127, s));
        nat.push_back({kLfo1TempoSync, 1});
        add({std::string("lfo_sync_") + nm, rf::kEdges, 0, -1, 0, -1, -1, 1.6, 120, nat, {0.05f, 1.55f}});
    }
    // --- M3: glide (legato; the first note lands before key 2) --------------------------------
    for (int knob : {30, 60, 90, 120})
        for (int to : {72, 53}) {
            const double land = std::log(1.0135 / 0.0135) * tb::kPortaTau[knob], t2 = land + 0.2;
            Def d{"glide_" + std::to_string(knob) + (to > 60 ? "_up" : "_down"), rf::kGlide, 60, to, rf::kFlagLegato, t2, -1,
                  t2 + land + 0.25, 120, cat(sine1, {{kPlayMode, 2}, {kPortaTime, knob}}), {float(keyHz(60)), float(keyHz(to))}};
            add(d);
        }
    return v;
}

// A float literal: %.6g with a decimal point, then f.
void emitFloat(double x) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6g", x);
    std::printf("%s%sf", b, std::strpbrk(b, ".e") ? "" : ".0");
}

}  // namespace

int main(int argc, char** argv) {
    using namespace oxo::s2;
    if (argc > 1 && std::strcmp(argv[1], "tables") == 0) {
        // The output limiter's gain, log2, on log2 of its envelope -1.5 .. 2.5 in 1/16 octaves.
        std::printf("// limiter: log2 gain at log2 e = -1.5 + i / 16\n");
        for (int i = 0; i <= 64; ++i) {
            const double e = std::exp2(-1.5 + i / 16.0);
            const double g = std::pow(1.0 + std::pow(e / 1.038, 11.54), -0.856 / 11.54);
            std::printf("%.7gf,%s", std::log2(g), i % 8 == 7 ? "\n" : " ");
        }
        // LP24 feedback compensation on the cutoff axis 16.7 Hz x 2^(k / 12.624), k = 2 i.
        std::printf("\n// lp24 comp at k = 2 i\n");
        for (int i = 0; i <= 65; ++i) {
            const double hz = svfHz(2 * i);
            double c = 127;
            for (int j = 0; j < 127; ++j)
                if (hz <= tb::kLadderCutoffHz[j + 1]) {
                    const double a = std::log(std::max(1e-9, double(tb::kLadderCutoffHz[j]))), b = std::log(double(tb::kLadderCutoffHz[j + 1]));
                    c = j + std::clamp((std::log(hz) - a) / (b - a), 0.0, 1.0);
                    break;
                }
            const int j = std::min(int(c), 126);
            const double t = c - j;
            std::printf("%.7gf,%s", tb::kLadderFeedbackComp[j] * (1 - t) + tb::kLadderFeedbackComp[j + 1] * t, i % 8 == 7 ? "\n" : " ");
        }
        // LP24 on its own knob axis (the source's ladder table): log2 Hz and the
        // feedback compensation at knob 2 i (i = 64: knob 127's).
        std::printf("\n// lp24 log2 Hz at knob 2 i\n");
        for (int i = 0; i <= 64; ++i)
            std::printf("%.7gf,%s", std::log2(double(tb::kLadderCutoffHz[std::min(2 * i, 127)])), i % 8 == 7 ? "\n" : " ");
        std::printf("\n// lp24 comp at knob 2 i\n");
        for (int i = 0; i <= 64; ++i)
            std::printf("%.7gf,%s", double(tb::kLadderFeedbackComp[std::min(2 * i, 127)]), i % 8 == 7 ? "\n" : " ");
        // LFO pitch depth: cents at full swing for depth u = i / 64.
        std::printf("\n// lfo pitch cents at u = i / 64\n");
        std::vector<double> cents(65);
        for (int i = 0; i <= 64; ++i) {
            const double k = 127.0 * i / 64.0;
            const int j = std::min(int(k), 126);
            cents[size_t(i)] = i == 0 ? 0.0 : tb::kLfoPitchCents[j] + (tb::kLfoPitchCents[j + 1] - tb::kLfoPitchCents[j]) * (k - j);
            std::printf("%.7gf,%s", cents[size_t(i)], i % 8 == 7 ? "\n" : " ");
        }
        double worst = 0;
        for (int k = 8; k < 128; ++k) {
            const double x = k / 127.0 * 64.0;
            const int i = std::min(int(x), 63);
            const double c = cents[size_t(i)] + (cents[size_t(i) + 1] - cents[size_t(i)]) * (x - i);
            worst = std::max(worst, std::fabs(c / tb::kLfoPitchCents[k] - 1.0));
        }
        std::printf("\n// lfo pitch table: worst relative error (depth 8..127) %.4f\n", worst);
        return 0;
    }
    const PatchValues base = neutral();
    const std::vector<float> ob = ours(base);
    if (argc > 2 && std::strcmp(argv[1], "dump") == 0) {   // ref1type dump <case>: the render, one sample per line
        for (const Def& d : defs()) {
            if (d.name != argv[2]) continue;
            PatchValues p = base;
            for (const auto& [id, x] : d.nat) p[id] = x;
            std::vector<float> L, R;
            renderVoice(d, p, L, R);
            for (size_t i = 0; i < L.size(); ++i) std::printf("%.9g %.9g\n", L[i], R[i]);
        }
        return 0;
    }
    std::printf("// GENERATED by modules/onetype/tools/ref/ref1type.cpp from Oxo Synth2 at 6bb4435\n"
                "// (MIT, Copyright (c) 2026 Francois Chabloz and contributors; see the tool for\n"
                "// the full notice). Features of the source engine for engine_onetype_test.cpp.\n"
                "// Do not edit by hand.\n\n");
    std::printf("namespace onetype_ref {\n\nconstexpr double kRefFs = %.1f;\n\n", kFs);
    std::printf("// The neutral patch in 1-Type units.\nconstexpr float kBase[%d] = {", int(ob.size()));
    for (size_t i = 0; i < ob.size(); ++i) {
        if (i) std::printf(", ");
        emitFloat(ob[i]);
    }
    std::printf("};\n\nconstexpr Case kCases[] = {\n");
    int count = 0;
    for (const Def& d : defs()) {
        PatchValues p = base;
        for (const auto& [id, x] : d.nat) p[id] = x;
        const std::vector<float> o = ours(p);
        std::vector<std::pair<int, float>> sets;
        for (int i = 0; i < int(o.size()); ++i)
            if (o[size_t(i)] != ob[size_t(i)]) sets.push_back({i, o[size_t(i)]});
        std::vector<float> args = d.arg;
        float feat[rf::kMaxExp] = {};
        int nf = 0;
        rf::Case c{};
        c.kind = d.kind;
        c.key = int8_t(d.key);
        c.key2 = int8_t(d.key2);
        c.b2 = int16_t(blocksOf(d.t2));
        c.bOff = int16_t(blocksOf(d.tOff));
        c.blocks = int16_t(blocksOf(d.dur));
        if (d.kind == rf::kFiltIr || d.kind == rf::kSelfOsc || d.kind == rf::kLimSat) {
            // Kernel: the arguments in 1-Type units.
            sets.clear();
            c.blocks = 0;
            const double hz = d.kind == rf::kLimSat ? svfHz(127) : cutoffHz(d.ftype, d.fknob);
            if (d.kind == rf::kLimSat) {
                args = {float(satUnits(d.fsat)), 0.1f, 0.3f, 0.5f, 0.8f, 1.2f, 2.0f, 3.0f, 5.0f};
                c.narg = uint8_t(args.size());
                std::copy(args.begin(), args.end(), c.arg);
                nf = rf::limSatFeatures(c, [&] { return Section(0, 127, 0, d.fsat); }, kFs, feat);
            } else {
                args = {float(d.ftype), float(hz), float(d.fres / 127.0)};
                if (d.kind == rf::kFiltIr) {
                    const bool pk = d.fres >= 80;
                    args.push_back(pk ? 1.0f : 0.0f);
                    for (double o2 : {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0}) args.push_back(float(std::clamp(hz * std::exp2(o2), 20.0, 18000.0)));
                    c.narg = uint8_t(args.size());
                    std::copy(args.begin(), args.end(), c.arg);
                    Section s(d.ftype, d.fknob, d.fres, 0);
                    nf = rf::filterIrFeatures(c, s, kFs, feat);
                } else {
                    c.narg = uint8_t(args.size());
                    std::copy(args.begin(), args.end(), c.arg);
                    Section s(d.ftype, d.fknob, d.fres, 0);
                    nf = rf::selfOscFeatures(s, kFs, feat);
                }
            }
        } else {
            c.narg = uint8_t(args.size());
            std::copy(args.begin(), args.end(), c.arg);
            c.flags = d.flags;
            std::vector<float> L, R;
            renderVoice(d, p, L, R);
            nf = rf::voiceFeatures(c, L, R, kFs, feat);
        }
        if (sets.size() > size_t(rf::kMaxSet) || args.size() > size_t(rf::kMaxArg) || nf > rf::kMaxExp) {
            std::fprintf(stderr, "case %s: too many values\n", d.name.c_str());
            return 1;
        }
        std::printf("    {\"%s\", %d, %d, %d, %d, %d, %d, %d, ", d.name.c_str(), int(c.kind), int(c.key), int(c.key2), int(d.flags),
                    int(c.b2), int(c.bOff), int(c.blocks));
        emitFloat(d.tempo);
        std::printf(",\n     %d, {", int(sets.size()));
        for (size_t i = 0; i < sets.size(); ++i) {
            std::printf("%s{%d, ", i ? ", " : "", sets[i].first);
            emitFloat(sets[i].second);
            std::printf("}");
        }
        std::printf("},\n     %d, {", int(args.size()));
        for (size_t i = 0; i < args.size(); ++i) {
            if (i) std::printf(", ");
            emitFloat(args[i]);
        }
        std::printf("},\n     %d, {", nf);
        for (int i = 0; i < nf; ++i) {
            if (i) std::printf(", ");
            emitFloat(feat[i]);
        }
        std::printf("}},\n");
        ++count;
    }
    std::printf("};\n\nconstexpr int kCaseCount = %d;\n\n}  // namespace onetype_ref\n", count);
    return 0;
}
