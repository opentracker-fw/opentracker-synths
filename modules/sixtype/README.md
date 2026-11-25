<!-- SPDX-License-Identifier: Apache-2.0 -->
# 6-Type

A six-operator FM voice for the OpenTracker 3.x Full profile (card module `6TY1`, engine 0x8C,
patches in `Patches/6TYPE/`, "6-Type" on the Synth page): six-operator FM, 32 algorithms; reads
32-voice (4104 B) and single-voice (163 B) `.syx` voice dumps. Load Patch lists the voices of the
`.syx` files in `Patches/6TYPE/`; a patch keeps its voice (155 bytes) in patch field 100.

## The voice

- **Operators:** six, each with a 4-rate / 4-level envelope, output level, key level scaling
  (break point, two depths and curves), rate scaling, velocity sensitivity, amplitude
  modulation sensitivity, ratio or fixed frequency and detune.
- **Algorithms:** the 32 operator layouts with operator 6's feedback (0..7); algorithms 4 and 6
  feed back through a chain of three / two operators.
- **Voice:** pitch envelope, LFO (six waves, delay, key sync; pitch and amplitude depths),
  transpose, oscillator key sync.
- **Controls on top of the voice:** Mono (Retrigger / Legato at polyphony 1), Brightness
  (modulator levels), Feedback, Attack, Release, LFO Depth and Detune; the Init patch puts
  M1-M6 on them.

## Budgets

| | |
|---|---|
| CPU | at most 23,000 M4 instructions per voice and block (`sixtype.unit` 21,554: the costliest algorithm of the `sixtype.scan` search, algorithm 4, six operators at level 99, feedback 7, AM and PM on; the cost does not depend on the pitch), 8 voices |
| Card pool | `sixtype.otm` about 19 KB, plus 6 KB of bss: about 25 KB in the pool |
| State | 6,116 B of bss (the sine table, 4,100 B, and 8 voices of 252 B), about 260 B of slot scratch |

`bench/sixtype_bench.cpp` measures the lines; build it with `-DOT_SIXTYPE_SCAN` to run the search.

## Licence

The engine and its tables are derived from msfa, Google's Music Synthesizer for Android, and are
under the Apache License 2.0, not this repository's MIT licence. `msfa.txt` holds the licence
text and lists the msfa sources used; the changes are listed at the top of `src/sixtype.h` and
`src/sixtype.cpp`. Ship this notice (and `msfa.txt`) with `sixtype.otm`:

```
6-Type (sixtype.otm) contains code derived from msfa (Music Synthesizer for Android,
https://github.com/google/music-synthesizer-for-android), Copyright 2012-2013 Google Inc.,
licensed under the Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0).
```
