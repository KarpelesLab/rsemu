# SNES S-DSP: `SDspChip`

Consumed by: `src/dev/snes/dsp.rs` (feature `dev-snes-dsp`), behind the seam in
`src/dev/snes/dsp_seam.rs`. The chip has no class and no place in the device
graph. The S-SMP owns it: it forwards `$F2`/`$F3` to `SDsp::read`/`write`,
calls `SDsp::clock` once per SMP clock with audio RAM, and gets a stereo
sample back on every 32nd call (1 024 000 / 32 = 32 000 Hz exactly).

## Sources

| Document | Answers |
| --- | --- |
| nocash **fullsnes**, *SNES APU DSP BRR Samples* | the directory entry, the BRR header, `(nibble SHL shift) SAR 1`, shifts 13-15, the four filters' exact formulas, the clamp to 16 bits and the wrap to 15, the loop/end codes |
| fullsnes, *BRR Pitch* | the pitch counter, PMON's `(OUTX SAR 4) + $400` factor, the **Gaussian table** (512 entries), and the interpolator's wrap/saturate order |
| fullsnes, *ADSR/Gain Envelope* | the ADSR and GAIN modes and their steps, the **rate table**, the phase bookkeeping that continues in GAIN mode, the 11-bit "hidden" value bent-increase compares against |
| fullsnes, *Volume*, *Control*, *Echo Registers* | the mixer's saturation, FLG's bits and reset value `$E0`, ENDX's write-to-clear, the noise LFSR, KON/KOFF poll semantics, the echo buffer's layout, ESA's one-sample lag, EDL's reload-at-index-zero, the FIR's wrap-then-saturate |
| fullsnes, *SNES APU Low Level Timings* | the 32-slot chart of which register and RAM byte each slot touches; every stage of this model sits on that chart |
| SNESdev wiki, *DSP envelopes* | the period table again and the **per-column offsets** (536 / 0 / 1040), and the rate counter's `$7800`-sample period (after anomie's S-DSP document) |
| SNESdev wiki, *S-DSP registers*, *BRR samples* | the sustain-level compare on envelope bits 10-8; "playback begins centred on the second sample"; ESA/EDL errata |

The Gaussian table was extracted mechanically from fullsnes's text, not retyped.
It was then checked against fullsnes's own claim that every set
`g[i] + g[$FF-i] + g[$100+i] + g[$1FF-i]` sums to `$7FF`-`$801`. A unit test
keeps that check. No emulator source was consulted (`ROADMAP.md` §1). The
SNESdev wiki cites emulator source in two places (ENDX after key-on, the
references list). Nothing here depends on those citations.

## The model

One `clock` call runs the jobs of one slot. Each voice has nine stages, which
`dsp.rs`'s module documentation tabulates, and the echo unit, output and poll
have global stages at T23-T31. A register therefore takes effect at the slot
that reads it. KON and KOFF are seen only at the T31 poll, every other sample.
VOLL is read at the voice's BRR2 slot and VOLR one slot later. DIR, NON and EON
are latched at T29 and apply from the next sample.

Each voice keeps three groups of four decoded samples. A crossing of `$4000` in
the pitch counter decodes the next group from the header and data bytes read
this sample. The pitch step is capped at `$3FFF`, so a crossing can never
happen twice in one sample. Key-on runs as follows:

1. The poll zeroes the envelope and sets Attack.
2. The voice's next DIR slot reads the start address and starts a five-sample
   delay.
3. The last three delay samples decode the first three groups.
4. The envelope does not run during the delay, and the output stays at zero.

So the first audible sample is the sixth, centred on BRR sample 1, and ENDX
stays clear until the fourth group, the end of the first block, has been
decoded.

## Interpretations

Where the documents leave a choice, `dsp.rs` lists the choice it made under
"Interpretations". In short:

- the decode buffer and the delay schedule above;
- the `$3FFF` cap on the pitch step;
- decay ending on bits 10-8 equal to SL, rather than fullsnes's
  `<= (SL+1)*$100`;
- volume products wrapping at 23 bits, from fullsnes's
  "-8000h*-80h=-400000h";
- the rate counter ticking at T30;
- the output taken before the post-amplifier's XOR inversion;
- reset keeping the register array except FLG.

fullsnes has one internal inconsistency here. A BRR result above `+$7FFF` is
clamped to `$7FFF`, and the 15-bit rule fullsnes gives beside it maps that
value to `-1`. The parenthetical "(clipped to +3FFFh below)" says otherwise.
The model follows the rule, and a unit test pins it.

## Unverified

Nothing here has been compared against a real S-DSP or against SPC recordings,
and no permissively licensed SPC test corpus was found to build a golden test
on. The slot of each stage comes from the chart. Within a slot, the order of
the work (for example decode after interpolation) is this model's choice.
