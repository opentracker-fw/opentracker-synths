// V-Type synth module (VTY1): a module engine with its own id (0x8B), in the
// Synth page model list after the Tracker Mini models and the module engines
// before it in the profile (modules/module.h). No static buffers.
#include "modules/module.h"
#include "modules/synth_descriptor.h"
#include "vtype.h"

namespace ot {
namespace mod {

extern const SynthDescriptor kVtypeDescriptor;   // vtype_desc.cpp

namespace {

const ModuleInfo kInfo = {"VTY1", kModuleApi, 100, "V-Type synth", kCapSynthEngine | kCapSynthDescriptor, 0, 0, 0, 40000, 6};

class Module final : public IModule {
public:
    const ModuleInfo& info() const override { return kInfo; }
    bool attach(ModuleHost&) override { return true; }
    audio::ISynthEngine* synthEngine(int32_t engineId) override {
        return engineId == audio::vtype::kEngineId ? audio::vtype::engine() : nullptr;
    }
    const SynthDescriptor* synthDescriptor(int i) const override { return i == 0 ? &kVtypeDescriptor : nullptr; }
};

Module gModule;

}  // namespace

IModule& vtypeModule() { return gModule; }

}  // namespace mod
}  // namespace ot
