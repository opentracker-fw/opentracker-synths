// 1-Type engine (see onetype.h).
#include "onetype.h"

#include <cstring>

namespace ot {
namespace audio {
namespace onetype {

namespace {

struct VoiceState {
    uint8_t started;
};
static_assert(sizeof(VoiceState) <= kSynthVoiceStateBytes, "1-Type voice state");

class OnetypeEngine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "1TYPE"; }
    void slotReset(SynthSlotScratch& sc) override { std::memset(sc.bytes, 0, sizeof(sc.bytes)); }
    void noteOn(SynthVoice& v, const SynthParams&, const SynthNote&) override { v.as<VoiceState>().started = 1; }
    void noteOff(SynthVoice&, const SynthParams&) override {}
    bool render(SynthVoice& v, const SynthParams&, float, float, float*, float*) override {
        v.as<VoiceState>().started = 0;
        v.level = 0.0f;
        return false;
    }
};

OnetypeEngine gOnetype;

}  // namespace

ISynthEngine* engine() { return &gOnetype; }

}  // namespace onetype
}  // namespace audio
}  // namespace ot
