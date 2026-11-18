// DX7 synth module (DX7S) [OT]: the DX7 engine (dx7.h), engine id 0x8C, in
// the Synth page model list after the Tracker Mini models (modules/module.h).
#include "dx7.h"
#include "dx7_tables.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

extern const SynthDescriptor kDx7Descriptor;   // dx7_desc.cpp

namespace {

// sram_audio_bytes: the sine table (dx7_tables.h gDx7Sin).
const ModuleInfo kInfo = {"DX7S", kModuleApi, 100, "DX7 synth", kCapSynthEngine | kCapSynthDescriptor,
                          4100, 0, 0, 22000, 8};

class Module final : public IModule {
public:
    const ModuleInfo& info() const override { return kInfo; }
    bool attach(ModuleHost&) override {
        audio::dx7::initSinTable();
        return true;
    }
    audio::ISynthEngine* synthEngine(int32_t engineId) override {
        return engineId == audio::dx7::kEngineId ? audio::dx7::engine() : nullptr;
    }
    const SynthDescriptor* synthDescriptor(int i) const override { return i == 0 ? &kDx7Descriptor : nullptr; }
};

Module gModule;

}  // namespace

IModule& dx7Module() { return gModule; }

}  // namespace mod
}  // namespace ot
