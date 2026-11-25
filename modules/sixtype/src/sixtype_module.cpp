// 6-Type synth module (6TY1) [OT]: the 6-Type engine (sixtype.h), engine id 0x8C, in
// the Synth page model list after the Tracker Mini models (modules/module.h).
#include "sixtype.h"
#include "sixtype_tables.h"
#include "modules/module.h"
#include "modules/synth_descriptor.h"

namespace ot {
namespace mod {

extern const SynthDescriptor kSixtypeDescriptor;   // sixtype_desc.cpp

namespace {

// sram_audio_bytes: the sine table (sixtype_tables.h gSixtypeSin).
const ModuleInfo kInfo = {"6TY1", kModuleApi, 100, "6-Type synth", kCapSynthEngine | kCapSynthDescriptor,
                          4100, 0, 0, 22000, 8};

class Module final : public IModule {
public:
    const ModuleInfo& info() const override { return kInfo; }
    bool attach(ModuleHost&) override {
        audio::sixtype::initSinTable();
        return true;
    }
    audio::ISynthEngine* synthEngine(int32_t engineId) override {
        return engineId == audio::sixtype::kEngineId ? audio::sixtype::engine() : nullptr;
    }
    const SynthDescriptor* synthDescriptor(int i) const override { return i == 0 ? &kSixtypeDescriptor : nullptr; }
};

Module gModule;

}  // namespace

IModule& sixtypeModule() { return gModule; }

}  // namespace mod
}  // namespace ot
