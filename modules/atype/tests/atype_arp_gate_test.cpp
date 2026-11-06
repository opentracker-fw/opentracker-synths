// The step FX Arp on A-Type through the real Sequencer and SynthBank: at the
// default Arp Gate 100 a held patch keeps its level across the notes (the
// release gets no time) and nothing changes from the behaviour before the
// gate; below 100 the level falls before each note and the attack is heard
// again, every note, at Poly 1 and above; the voice count stays within Poly;
// the level comes up on the note, in tune with the arp's order; Legato modes
// retrigger too (the note-off comes first); nothing stays on after Stop.
#include "test.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "atype.h"
#include "audio/synth_bank.h"
#include "audio/synth_engine.h"
#include "audio/synth_params.h"
#include "formats/patch_pb.h"
#include "model/pattern.h"
#include "model/project.h"
#include "modules/module.h"
#include "modules/synth_models.h"
#include "sequencer/sequencer.h"

using namespace ot;
using namespace ot::audio;

namespace {

struct Host : ISequencerHost {
    uint8_t gate = 100;
    uint8_t synthArpGate(uint8_t) override { return gate; }
};

struct Sink : ISequencerSink {
    SynthBank* bank = nullptr;
    int block = 0, maxVoices = 0;
    std::vector<int> onBlock, offBlock;
    std::vector<int> notes;
    void voiceNoteOn(uint8_t, const VoiceNoteOn&) override {}
    void voiceNoteOff(uint8_t, NoteOffMode) override {}
    void voiceApplyFx(uint8_t, uint8_t, uint8_t, uint8_t) override {}
    void resetAllVoiceFx() override {}
    void engineBlockUpdate(float) override {}
    void metronomeClick(bool, uint8_t) override {}
    void midiSend(uint8_t, const uint8_t*, uint8_t) override {}
    void synthNoteOn(uint8_t t, const SynthNoteOn& n) override {
        onBlock.push_back(block);
        notes.push_back(n.note);
        bank->noteOn(t, n);
    }
    void synthNoteOff(uint8_t t, NoteOffMode m) override {
        offBlock.push_back(block);
        bank->noteOff(t, m);
    }
};

struct Patch {
    float attack = 0.001f, decay = 0.1f, sustain = 1.0f, release = 0.01f;
    int glideMode = atype::kAlways;
    float glideTime = 0.0f;
};

struct Song {
    SynthPatch patch;
    Governor gov;
    SynthBank bank;
    SynthMixInputs in;
    Project project;
    Pattern a, b;
    Host host;
    Sink sink;
    Sequencer seq;
    std::vector<float> level;   // the block RMS

    Song(const Patch& q, int poly, int gate) {
        mod::ModuleHost::instance().init();
        gov.setBudget(8);
        bank.setGovernor(&gov);
        bank.setStartsPerBlock(0);
        patchInit(atype::kEngineId, &patch);
        synthParamSet(patch, atype::kVco2Level, 0.0f);
        synthParamSet(patch, atype::kCutoff, 20000.0f);
        synthParamSet(patch, atype::kEnvAmt, 0.0f);
        synthParamSet(patch, atype::kAttack, q.attack);
        synthParamSet(patch, atype::kDecay, q.decay);
        synthParamSet(patch, atype::kSustain, q.sustain);
        synthParamSet(patch, atype::kRelease, q.release);
        synthParamSet(patch, atype::kGlideMode, float(q.glideMode));
        synthParamSet(patch, atype::kGlideTime, q.glideTime);
        bank.bindPatch(0, &patch);
        SynthSlotRecord r = SynthSlotRecord::defaults(0);
        r.model = uint32_t(SynthModel::ACD);
        r.polyphony = uint32_t(poly);
        r.delaySend = r.reverbSend = 0.0f;
        bank.setSlotRecord(0, r);
        bank.setSlotEngine(0, atype::kEngineId);
        host.gate = uint8_t(gate);
        sink.bank = &bank;
        a.setDefault();
        b.setDefault();
        project.setTempo(120.0f);
        seq.attach(&sink, &host, &project, &a, &b);
        Step& st = seq.currentPattern()->tracks[0].steps[0];
        st.note = 60;
        st.instrument = 64;
        st.fx[1] = StepFx{21, 6};   // Arp up, a note per step (125 ms at 120 BPM)
        st.fx[0] = StepFx{14, 5};   // Chord 0 4 7
    }
    void run(int blocks) {
        float L[kBlock], R[kBlock], D[kBlock], V[kBlock];
        for (int k = 0; k < blocks; ++k, ++sink.block) {
            seq.processBlock(kBlock);
            for (uint32_t i = 0; i < kBlock; ++i) L[i] = R[i] = D[i] = V[i] = 0.0f;
            bank.render(in, L, R, D, V, nullptr);
            float e = 0.0f;
            for (uint32_t i = 0; i < kBlock; ++i) e += L[i] * L[i];
            level.push_back(std::sqrt(e / float(kBlock)));
            sink.maxVoices = std::max(sink.maxVoices, bank.voicesOf(0));
        }
    }
    // Inside the arp notes after the first: the quietest block over the loudest (dB).
    double swingDb() const {
        double sum = 0.0;
        int n = 0;
        for (size_t k = 1; k + 1 < sink.onBlock.size(); ++k) {
            float lo = 9.0f, hi = 0.0f;
            for (int b = sink.onBlock[k]; b < sink.onBlock[k + 1] && size_t(b) < level.size(); ++b) {
                lo = std::min(lo, level[size_t(b)]);
                hi = std::max(hi, level[size_t(b)]);
            }
            sum += 20.0 * std::log10(double(hi) / std::max(double(lo), 1e-6));
            ++n;
        }
        return n ? sum / n : 0.0;
    }
};

constexpr int kBlocks = 600;   // ~1.7 s: 13 arp notes

double swing(const Patch& q, int poly, int gate) {
    Song s(q, poly, gate);
    s.seq.play(0);
    s.run(kBlocks);
    return s.swingDb();
}

}  // namespace

TEST(atype_arp_gate_100_keeps_a_held_patch_flat) {
    const double d = swing(Patch{}, 1, 100);
    std::printf("  held patch, Poly 1, gate 100: %.1f dB\n", d);
    CHECK(d < 3.0);
}

TEST(atype_arp_gate_retriggers_a_held_patch) {
    for (int poly : {1, 4}) {
        const double d = swing(Patch{}, poly, 50);
        std::printf("  held patch, Poly %d, gate 50: %.1f dB\n", poly, d);
        CHECK(d > 40.0);
    }
}

TEST(atype_arp_gate_brings_the_attack_back) {
    // A 30 ms attack: each note starts low and climbs, every note alike.
    Patch q;
    q.attack = 0.03f;
    Song s(q, 1, 50);
    s.seq.play(0);
    s.run(kBlocks);
    CHECK(s.sink.onBlock.size() >= 10u);
    for (size_t k = 1; k + 1 < s.sink.onBlock.size(); ++k) {
        const int b = s.sink.onBlock[k];
        // Over a few blocks: one block (2.9 ms) is shorter than a cycle of the note.
        auto mean = [&](int from, int n) {
            float e = 0.0f;
            for (int i = from; i < from + n; ++i) e += s.level[size_t(i)];
            return e / float(n);
        };
        const float start = mean(b, 3), up = mean(b + 12, 6);   // ~0..9 ms, ~35..52 ms
        CHECK(start < 0.5f * up);
    }
}

TEST(atype_arp_gate_releases_on_time) {
    // Gate 50 of a 125 ms note: the off 62.5 ms on (21.5 blocks), to the block;
    // silent by the next note with a 10 ms release.
    Song s(Patch{}, 1, 50);
    s.seq.play(0);
    s.run(kBlocks);
    CHECK(s.sink.offBlock.size() >= 10u);
    for (size_t k = 0; k + 1 < s.sink.onBlock.size(); ++k) {
        const int d = s.sink.offBlock[k] - s.sink.onBlock[k];
        CHECK(d >= 21 && d <= 22);
        const int p = s.sink.onBlock[k + 1] - s.sink.onBlock[k];
        CHECK(p >= 43 && p <= 44);
        if (k > 0) CHECK(s.level[size_t(s.sink.onBlock[k + 1]) - 1] < 0.002f);
    }
}

TEST(atype_arp_gate_plays_the_arp_order) {
    Song s(Patch{}, 1, 30);
    s.seq.play(0);
    s.run(kBlocks);
    const int expect[3] = {60, 64, 67};
    for (size_t k = 0; k < s.sink.notes.size(); ++k) CHECK_EQ(s.sink.notes[k], expect[k % 3]);
}

TEST(atype_arp_gate_voices_stay_within_poly) {
    for (int poly : {1, 2, 4}) {
        Patch q;
        q.release = 0.5f;   // long tails against a short gate
        Song s(q, poly, 25);
        s.seq.play(0);
        s.run(kBlocks);
        std::printf("  Poly %d: at most %d voices\n", poly, s.sink.maxVoices);
        CHECK(s.sink.maxVoices <= poly);
    }
    // A short release frees the voice before the next note, whatever the Poly.
    Song s(Patch{}, 4, 50);
    s.seq.play(0);
    s.run(kBlocks);
    CHECK_EQ(s.sink.maxVoices, 1);
}

TEST(atype_arp_gate_retriggers_in_legato_mode) {
    Patch q;
    q.glideMode = atype::kLegato;
    q.glideTime = 0.02f;
    CHECK(swing(q, 1, 50) > 40.0);
}

TEST(atype_arp_gate_nothing_sounds_after_stop) {
    Patch q;
    q.release = 0.05f;
    for (int gate : {100, 50}) {
        Song s(q, 1, gate);
        s.seq.play(0);
        s.run(200);
        s.seq.stop();
        s.run(345);
        CHECK(s.level.back() == 0.0f);
        CHECK_EQ(s.bank.voicesOf(0), 0);
    }
}
