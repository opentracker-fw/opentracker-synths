// 1-Type synth module (1TY1): a module engine with its own id (0x8A), in the
// Synth page model list after the Tracker Mini models and the module engines
// before it in the profile (modules/module.h). No static buffers.
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "onetype.h"

namespace ot {
namespace mod {

extern const SynthDescriptor kOnetypeDescriptor;   // onetype_desc.cpp

namespace {

const ModuleInfo kInfo = {"1TY1", kModuleApi, 100, "1-Type synth", kCapSynthEngine | kCapSynthDescriptor, 0, 0, 0, 45000, 6};

class Module final : public IModule {
public:
    const ModuleInfo& info() const override { return kInfo; }
    bool attach(ModuleHost&) override { return true; }
    audio::ISynthEngine* synthEngine(int32_t engineId) override {
        return engineId == audio::onetype::kEngineId ? audio::onetype::engine() : nullptr;
    }
    const SynthDescriptor* synthDescriptor(int i) const override { return i == 0 ? &kOnetypeDescriptor : nullptr; }
};

Module gModule;

}  // namespace

IModule& onetypeModule() { return gModule; }

}  // namespace mod
}  // namespace ot
