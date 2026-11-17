# V-Type

A swarm-oscillator virtual analog voice in the style of the late-90s desktop VA synths, for the
OpenTracker 3.x Full profile (card module `VTY1`, engine 0x8B, patches in `Patches/VTYPE/`,
"V-Type" on the Synth page). Our own design and code (MIT, as the rest of this repository); no
third-party code, firmware, wavetables or patches.

## The voice

- **Swarm:** 1, 3, 5 or 7 band-limited saws. Detune D spreads them by 60 D (0.2 + 0.8 D) cents
  at uneven offsets; Swarm Mix moves power from the centre saw to the side saws at a constant
  total. Phase Free starts every note at new phases, Reset at fixed ones. A sub one octave down
  (square or sine) is locked to the centre saw.
- **Osc 2:** saw, pulse (Pulse Width), triangle or sine with Semi and Fine; Sync restarts it on
  the centre saw's cycle; Ring adds the centre saw times osc 2.
- **Mixer:** Balance (Swarm .. osc 2 at equal power), white noise with a Colour tilt (low pass
  below 0, high pass above), Key Shift, Fine Tune.
- **Two filters:** LP12, LP24, BP12 or HP12 each, with cutoff, resonance (no self-oscillation),
  Env 1 amount and key track. **Drive** sits after filter 1 (Soft, Hard or Fold).
- **Routing:** Single (filter 1), Serial (filter 1 into filter 2, Flt Balance picks the tap),
  Parallel (both filters on the same input, Flt Balance mixes them) or Split (Swarm and sub
  through filter 1 on the left, osc 2 and noise through filter 2 on the right, Split Width).
- **Modulation:** Amp envelope, Env 1 (filters), Env 2 (matrix only), two LFOs per slot (rate or
  a tempo division, Key Sync), a 4-slot matrix over 13 destinations, glide.

There is no voice stacking: the Swarm already is seven oscillators per note; Spread and Split
Width widen the sound.

## Budgets

| | |
|---|---|
| CPU | at most 40,000 M4 instructions per voice and block (`vtype.unit`, the costliest path of the `vtype.scan` search), 6 voices |
| Card pool | `vtype.otm` about 22 KB; it fits beside 1-Type and any one card effect |
| State | 128 B per voice, 384 B per slot, no static buffers |

`bench/vtype_bench.cpp` measures the lines; build it with `-DOT_VTYPE_SCAN` to run the search.
