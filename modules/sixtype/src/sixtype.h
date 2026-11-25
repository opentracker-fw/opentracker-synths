// 6-Type: a six-operator FM engine, 32 algorithms, that plays .syx voice
// dumps (32-voice banks of 4104 B, single voices of 163 B) [OT].
//
// Not a Tracker Mini model. The sound core follows Google's Music
// Synthesizer for Android (msfa, Copyright 2012-2013 Google Inc., Apache
// License 2.0, modules/sixtype/msfa.txt): its envelope, pitch envelope,
// LFO, frequency / level laws, algorithm table and operator kernels, ported
// to this engine interface with these changes (Apache 2.0 section 4b):
//  - compact per-voice state (the module's own array, one entry per pool
//    record: SynthVoice::index) and per-slot state (the voice data and the
//    LFO in SynthSlotScratch); tables precomputed in flash
//    (sixtype_tables.cpp);
//  - float note pitch with the framework's bend, the voice transpose, and
//    oscillator key sync;
//  - our own additions where msfa has none: amplitude modulation (LFO AMD x
//    operator AMS), the two- and three-operator feedback loops of algorithms
//    4 and 6, and the end of a voice.
//
// Voice data: the 155-byte single-voice parameter block (the body of a
// 163-byte "F0 43 0n 00 01 1B" dump), see unpackVoice() for the packed
// 128-byte bank entry (32 per 4104-byte bank).
//
// Engine contract (audio/synth_engine.h): Volume is never applied here;
// output is mono (L = R).
//
// [I] A module engine (id kEngineId): its patches keep the voice in patch
// field 100, and Load Patch lists .syx files (sixtype_desc.cpp PatchImport).
#pragma once

#include <cstddef>
#include <cstdint>

#include "audio/synth_engine.h"

namespace ot {
namespace audio {
namespace sixtype {

constexpr int32_t kEngineId = 0x8C;   // a module engine (0x80..), not a Tracker Mini model
constexpr int kVoiceBytes = 155;      // single voice
constexpr int kPackedBytes = 128;     // packed bank entry
constexpr int kBankVoices = 32;
constexpr int kNameLen = 10;

// Bank entry (128 bytes, packed) -> single voice (155 bytes).
void unpackVoice(const uint8_t packed[kPackedBytes], uint8_t out[kVoiceBytes]);

// A SysEx file's voices. Accepts a single-voice dump (163 bytes) or a 32-voice
// bank (4104 bytes); with or without valid checksums (checksumOk reports).
// Returns the number of voices written to out (0: not a voice dump).
int parseSysex(const uint8_t* data, size_t len, uint8_t out[][kVoiceBytes], int maxVoices,
               bool* checksumOk = nullptr);

// The same files, one voice at a time (the Load Patch browser): the voices in
// the file (1, 32, or 0: not a voice dump), and voice k (unpacked, sanitized)
// into out (false past the end).
int sysexVoiceCount(const uint8_t* data, size_t len);
bool sysexVoice(const uint8_t* data, size_t len, int k, uint8_t out[kVoiceBytes]);

// Out-of-range values clamped to the parameter ranges (in place). Returns how many
// were changed.
int sanitizeVoice(uint8_t v[kVoiceBytes]);

// Load voice v into a slot (audio context, between blocks). Voices already
// sounding keep their note setup; new notes use the new voice.
void setVoice(SynthSlotScratch& slot, const uint8_t v[kVoiceBytes]);

ISynthEngine* engine();

}  // namespace sixtype
}  // namespace audio
}  // namespace ot
