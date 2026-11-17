// The .sy1 mapping (tools/sy1/sy1_map.h): the file format and the original's
// conversion of old versions (re-expressed from Oxo Synth2's ParamsTests:
// current format, no version, the legacy fixture, spot values, the tempo
// sync steps, the defaults in range), every mapped parameter at its minimum,
// default and maximum inside 1-Type's ranges, the built-in init patch
// converting to 1-Type's Init patch, and a converted patch that validates and
// survives the patch file round trip with its macros.
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
#include "test.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../tools/sy1/sy1_map.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "modules/module.h"
#include "onetype.h"

using namespace ot;
namespace sy = onetype_sy1;

namespace {
std::string here() {
    std::string f = __FILE__;
    return f.substr(0, f.find_last_of('/') + 1);
}
const PatchParamRange* range(int i) { return audio::synthParamRange(audio::onetype::kEngineId, i); }
}  // namespace

TEST(sy1_parses_current_format) {
    const sy::Patch p = sy::parse("Lead A\r\ncolor=red\r\nver=112\r\n0,3\r\n9,-12\r\n19,127\r\n94,32\r\n21,128\r\n");
    const std::string name = p.name;
    CHECK_STR(name.c_str(), "Lead A");
    CHECK_EQ(p.version, 112);
    CHECK(!p.legacy());
    CHECK_EQ(p.v[sy::kOsc1Shape], 3);
    CHECK_EQ(p.v[sy::kOscKeyShift], -12);
    CHECK_EQ(p.v[sy::kFilterFreq], 127);
    CHECK_EQ(p.v[sy::kPolyphony], 32);
    CHECK_EQ(p.v[sy::kFilterAmount], 127);                       // out of range: clamped
    CHECK_EQ(p.v[sy::kOsc2Pitch], sy::kParams[sy::kOsc2Pitch].def);   // a missing line keeps the init value
}

TEST(sy1_legacy_has_no_version) {
    const sy::Patch p = sy::parse("old melody\n0,1\n45,0\n33,64\n");
    CHECK(p.legacy());
    CHECK_EQ(p.v[sy::kArpBeat], 18);
}

// The fixture: old patches with the values the original keeps after loading them.
TEST(sy1_legacy_conversion_matches_the_original) {
    std::FILE* f = std::fopen((here() + "sy1_legacy_fixture.txt").c_str(), "r");
    CHECK(f != nullptr);
    if (!f) return;
    int count = 0, ver = 0, bad = 0;
    while (std::fscanf(f, "%d", &ver) == 1) {
        std::string text = "fixture\n" + (ver ? "ver=" + std::to_string(ver) + "\n" : std::string());
        for (int i = 0, x = 0; i < sy::kCount && std::fscanf(f, "%d", &x) == 1; ++i) text += std::to_string(i) + "," + std::to_string(x) + "\n";
        CHECK(std::fscanf(f, " |") == 0);
        int expected[sy::kCount];
        for (int& x : expected) CHECK(std::fscanf(f, "%d", &x) == 1);
        const sy::Patch p = sy::parse(text);
        for (int i = 0; i < sy::kCount; ++i) bad += p.v[i] != expected[i];
        ++count;
    }
    std::fclose(f);
    CHECK_EQ(bad, 0);
    CHECK(count >= 200);
}

TEST(sy1_legacy_spot_values) {
    sy::Patch p = sy::parse("old\nver=105\n21,0\n26,20\n38,0\n44,19\n50,127\n87,10\n93,8\n");
    CHECK_EQ(p.v[sy::kFilterAmount], 63);
    CHECK_EQ(p.v[sy::kAmpDecay], 8);
    CHECK_EQ(p.v[sy::kUnisonVoices], 2);
    CHECK_EQ(p.v[sy::kUnisonDetune], 48);
    CHECK_EQ(p.v[sy::kWheelSpeedSens], 117);
    CHECK_EQ(p.v[sy::kCtrlAssign1], 44);
    p = sy::parse("old\nver=106\n38,1\n");
    CHECK_EQ(p.v[sy::kUnisonVoices], 4);
    CHECK_EQ(p.v[sy::kUnisonDetune], 70);
    p = sy::parse("new\nver=109\n21,0\n26,20\n93,8\n");   // current versions are untouched
    CHECK_EQ(p.v[sy::kFilterAmount], 0);
    CHECK_EQ(p.v[sy::kAmpDecay], 20);
    CHECK_EQ(p.v[sy::kUnisonVoices], 8);
    p = sy::parse("odd\nver=113\n46,0\n58,1\n21,128\n31,0\n57,2\n85,85\n");   // the nearest in-range value
    CHECK_EQ(p.v[sy::kLfo2Dest], 1);
    CHECK_EQ(p.v[sy::kLfo2On], 1);
    CHECK_EQ(p.v[sy::kFilterAmount], 127);
    CHECK_EQ(p.v[sy::kArpType], 1);
    CHECK_EQ(p.v[sy::kLfo1On], 1);
    CHECK_EQ(p.v[sy::kUnisonPitch], 48);
}

TEST(sy1_lfo_tempo_sync_steps) {
    // 30 steps over the knob: Sync 1 + step
    CHECK_EQ(sy::syncStep(0), 0);
    CHECK_EQ(sy::syncStep(4), 0);
    CHECK_EQ(sy::syncStep(5), 1);
    CHECK_EQ(sy::syncStep(63), 14);
    CHECK_EQ(sy::syncStep(64), 15);
    CHECK_EQ(sy::syncStep(127), 29);
    sy::Patch p = sy::parse("sync\nver=112\n67,1\n43,64\n");
    float o[audio::onetype::kParamCount];
    sy::toOnetype(p.v, o);
    CHECK_EQ(o[audio::onetype::kLfo1Sync], 16.0f);   // 1.75 beats (7/16)
}

TEST(sy1_defaults_in_range) {
    for (int i = 0; i < sy::kCount; ++i) {
        CHECK(sy::kParams[i].min <= sy::kParams[i].def && sy::kParams[i].def <= sy::kParams[i].max);
    }
}

// Every source parameter at its minimum, default and maximum (the rest at
// the defaults) maps inside 1-Type's ranges.
TEST(sy1_every_mapped_parameter_stays_in_range) {
    mod::ModuleHost::instance().init();
    long out = 0;
    for (int i = 0; i < sy::kCount; ++i)
        for (int which = 0; which < 3; ++which) {
            sy::Patch p = sy::defaults();
            p.v[i] = which == 0 ? sy::kParams[i].min : (which == 1 ? sy::kParams[i].def : sy::kParams[i].max);
            float o[audio::onetype::kParamCount], ext[2];
            sy::toOnetype(p.v, o, ext);
            for (int k = 0; k < audio::onetype::kParamCount; ++k)
                if (!patchParamInRange(audio::onetype::kEngineId, k, o[k])) {
                    ++out;
                    std::printf("       source %d = %d -> param %d = %g out of range\n", i, p.v[i], k, double(o[k]));
                }
            CHECK(ext[0] >= 0.0f && ext[0] <= 1.0f && ext[1] >= 0.0f && ext[1] <= 1.0f);
        }
    CHECK_EQ(out, 0L);
}

// The original's built-in init patch is 1-Type's Init patch.
TEST(sy1_init_converts_to_the_init_patch) {
    mod::ModuleHost::instance().init();
    const sy::Patch p = sy::parse("init\nver=112\n");
    float o[audio::onetype::kParamCount];
    sy::toOnetype(p.v, o);
    for (int k = 0; k < audio::onetype::kParamCount; ++k) {
        const float d = range(k)->def;
        const bool ok = std::fabs(o[k] - d) <= 1e-4f * std::max(1.0f, std::fabs(d));
        if (!ok) std::printf("       param %d: converted %g, Init %g\n", k, double(o[k]), double(d));
        CHECK(ok);
    }
}

// A converted patch: valid, its macros set, through the patch file and back.
TEST(sy1_a_converted_patch_validates_and_round_trips) {
    mod::ModuleHost::instance().init();
    const sy::Patch src = sy::parse("Big Lead\nver=112\n0,1\n1,2\n76,40\n73,1\n84,100\n14,1\n19,70\n20,90\n30,64\n24,1\n57,1\n44,50\n");
    float o[audio::onetype::kParamCount], ext[2];
    sy::toOnetype(src.v, o, ext);
    SynthPatch p;
    CHECK(patchInit(audio::onetype::kEngineId, &p));
    for (int k = 0; k < audio::onetype::kParamCount; ++k) CHECK(audio::synthParamSet(p, k, o[k]));
    const std::vector<sy::Macro> ms = sy::macros(o);
    CHECK_EQ(int(ms.size()), 6);
    for (size_t m = 0; m < ms.size(); ++m) {
        PatchMacro& pm = p.macros[m];
        pm.bipolar = ms[m].bipolar;
        pm.count = int32_t(ms[m].targets.size());
        for (size_t t = 0; t < ms[m].targets.size(); ++t) {
            pm.targets[t].param = ms[m].targets[t].param;
            pm.targets[t].amountBits = patchFloatToBits(ms[m].targets[t].amount);
        }
    }
    CHECK(sy::velocityVolume(src.v) > 0.8f && sy::velocityVolume(src.v) < 0.84f);
    CHECK_EQ(int(sy::velocityExtra(src.v).size()), 1);
    CHECK(ext[1] > 0.5f);   // the unison pan spread -> Spread
    CHECK(o[audio::onetype::kOsc1Detune] >= 40.0f / 127.0f - 1e-6f);
    CHECK_EQ(validatePatch(p, nullptr, 0), 0);
    std::vector<uint8_t> a(kPatchMaxFileSize);
    size_t na = 0;
    CHECK(encodePatch(p, a.data(), a.size(), &na) == PatchStatus::kOk);
    SynthPatch q;
    CHECK(decodePatch(a.data(), na, &q) == PatchStatus::kOk);
    CHECK_EQ(validatePatch(q, nullptr, 0), 0);
    for (int k = 0; k < audio::onetype::kParamCount; ++k) CHECK_EQ(q.params[k].bits, patchFloatToBits(o[k]));
    const std::vector<std::string> r = sy::report(src.v);
    bool unison = false;
    for (const std::string& s : r) unison = unison || s.find("unison") != std::string::npos;
    CHECK(unison);
    const std::string name = sy::patchName("Lead/Mk*2  ");
    CHECK_STR(name.c_str(), "Lead_Mk_2");
}
