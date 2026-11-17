// V-Type engine (see vtype.h).
#include "vtype.h"

namespace ot {
namespace audio {
namespace vtype {

namespace {

class VtypeEngine final : public ISynthEngine {
public:
    int32_t id() const override { return kEngineId; }
    const char* name() const override { return "VTYPE"; }
    void noteOn(SynthVoice& v, const SynthParams& p, const SynthNote& n) override {
        (void)v;
        (void)p;
        (void)n;
    }
    void noteOff(SynthVoice& v, const SynthParams& p) override {
        (void)v;
        (void)p;
    }
    bool render(SynthVoice& v, const SynthParams& p, float g0, float gInc, float* L, float* R) override {
        (void)p;
        (void)g0;
        (void)gInc;
        (void)L;
        (void)R;
        v.level = 0.0f;
        return v.gate;   // silent: the voice is held until its note off
    }
};

VtypeEngine gVtype;

}  // namespace

ISynthEngine* engine() { return &gVtype; }

}  // namespace vtype
}  // namespace audio
}  // namespace ot
