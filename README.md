# opentracker-synths

Synth engine modules for [OpenTracker](https://github.com/sandlbn/polyend-tracker-re) 3.x, the open-source firmware
for the original Polyend Tracker. The engines are inspired by the Tracker Mini's synths but are
not identical to them.

| Module | Id | Engine |
|---|---|---|
| `modules/acd` | ACD1 | ACD (acid mono / poly synth) |
| `modules/fat` | FAT1 | FAT (supersaw-style pad and lead synth) |
| `modules/vap` | VAP1 | VAP (virtual analog poly synth) |
| `modules/perc` | PRC1 | PERC (drum machine, one drum per note) |
| `modules/wtfm` | WTF1 | WTFM (2-operator wavetable FM, three feedback paths) |
| `modules/fm4` | FM4S | FM4: four-operator FM, 8 algorithms, feedback, 5-stage envelopes per operator (module engine 0x80, Patches/FM4/) |
| `modules/ju6` | JU6S | JU6: one-DCO poly voice: one DCO (polyBLEP saw, PWM pulse, sub, noise), 4-position HPF, resonant 24 dB VCF, ADSR, delayed LFO, the built-in chorus Off / I / II once per slot (module engine 0x81, Patches/JU6/) |
| `modules/dtype` | DTY1 | D-Type ("D-Type" on screen): three-oscillator mono synth: 3 VCOs (6 waves, Lo..2', VCO 3 off the keyboard as an LFO), white / pink noise, mixer feedback, 4-pole ladder to self-oscillation, two ADS contours with a Decay switch, glide, Mod Mix (VCO 3 .. noise) into the VCOs and the filter by the Wheel (module engine 0x82, Patches/DTYPE/) |
| `modules/cstype` | CST1 | CS-Type ("CS-Type" on screen): two-layer polysynth: per note two layers, each saw + pulse (own PWM LFO), sine, noise, resonant 12 dB HPF and LPF (IL / AL contour), own ADSR VCA; Brilliance, Resonance, II Detune, ring modulator with sweep, sub-osc LFO, Touch (macro), Ribbon (macro), glide (module engine 0x83, Patches/CSTYPE/) |
| `modules/atype` | ATY1 | A-Type ("A-Type" on screen): duophonic two-oscillator synth: 2 VCOs (saw / pulse with PWM, hard sync), noise or ring modulator, sample & hold, HPF, 4-pole ladder or 2-pole SVF, ADSR and AR, LFO, glide; Keyboard Duo and Priority Last / Low / High at Poly 1 through the core's note priority (module engine 0x84, Patches/ATYPE/) |
| `modules/ttype` | TTY1 | T-Type ("T-Type" on screen): a drum synth in the classic analog drum machine style, one patch = one kit: kick, snare, clap, three toms, rim, cowbell, hats (closed .. open), crash / ride; a Character per drum, per-drum sends, Accent, Drive, Width; a cost governor (module engine 0x85, Patches/TTYPE/) |
| `modules/eight` | 8TY1 | 8-Type ("8-Type" on screen): a 3-voice 8-bit home computer style voice: three oscillators (tri, saw, pulse with PWM, 23-bit noise, the combined waves), ring and sync from the oscillator before, Filt / Clean routing, one 12 dB multimode filter per voice (Curve, 16 resonance steps, Dirt, Key Track, envelope), Grit, a 16-step ADSR, the CHIP tricks (module engine 0x86, Patches/8TYPE/) |
| `modules/hand` | HND1 | Hand-Type ("Hand-Type" on screen): a 4-channel handheld console style voice: one patch = one channel (Pulse with duty, sweep and a second pulse; Wave, 32 steps of 4 bits; Noise from a 15 / 7-bit register) or a drum kit by note; a 4-bit stepped envelope, Length, a small DAC, the CHIP tricks (module engine 0x87, Patches/HANDTYPE/) |
| `modules/btype` | BTY1 | B-Type ("B-Type" on screen): a 4-channel 8-bit home computer style voice with polynomial counters: divider-clocked channels (Tuning Free / Coarse / Fine), the buzz distortions, a second channel in divider steps, the high-pass pair trick, a 4-bit ADSR in 1/60 s frames, a small DAC, the CHIP tricks (module engine 0x88, Patches/BTYPE/) |
| `modules/arc` | ARC1 | Arc-Type ("Arc-Type" on screen): a home-console 4-operator FM style voice: eight algorithms, feedback, per operator MUL / DT / Level / Rate Scaling / a 5-rate envelope with loop modes / AMS, an LFO (PMS, AMS), a 9-bit DAC with a ladder character, an Aux square / noise voice, the CHIP arp and slide (module engine 0x89, Patches/ARCTYPE/) |

## How modules work

OpenTracker has no runtime plugin loading: the MK66 runs everything from flash. A **build
profile** in the core (`reimpl/profiles/<name>.toml`) lists the modules to compile in, and the
core's `tools/profile.py` reads each `module.toml`, checks it and generates the module registry.
Each module provides `ot::mod::IModule& <entry>()` (see the core's
`src/core/modules/module.h`, API major 1).

A module folder holds:

- `module.toml`: id, version, API major, entry function, sources, tests and declared budgets
  (flash, SRAM, SDRAM, fastrun bytes, CPU instructions per unit);
- `src/`: the engine and its `*_module.cpp` wrapper;
- `tests/`: host tests, built and run by the core's `make host-test`.

## Building

The core looks for this repo next to it (`MODULES_DIR`, default `../../opentracker-synths`
seen from `reimpl/`):

```
make -C ../ReversedPolytracker-3x/reimpl PROFILE=full MODULES_DIR=$PWD            # image
make -C ../ReversedPolytracker-3x/reimpl PROFILE=full MODULES_DIR=$PWD host-test  # tests
```

or `make image` / `make test` here (`CORE=<path to reimpl>`).

## Rules for module code

- No static constructors (namespace-scope objects must be constant-initialised), no heap.
- Voice state lives in the host's `SynthVoice::state`; per-voice render must stay within the
  declared `cpu_insn_per_unit` (checked by the core's `make perf-gate`).
- Only core headers: `modules/module.h`, `audio/synth_engine.h`, `audio/dsp.h`, `audio/va_dsp.h`,
  `audio/chip_dsp.h`, `audio/fm_dsp.h`, `fastrun.h` (the full list is in the core's `modules/module.h`).

License: MIT (LICENSE). Contains no Polyend code.
