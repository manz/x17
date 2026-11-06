# Probes

Programs whose silicon result would confirm or kill a rule the emulator (and sosumi's
DSP-shaped layer, `ref/dsp/ops.zig`) already relies on.

Status 2026-11-06: all nine below MEASURED, every rule held, no kill value seen (2 or 3
repeats each; the x02 bundle was already the both-signed setting, w1 = 0x0000A000). They are
gates in `src/silicon_gates.zig` under the same names. A 57-syllable variant with one more
op-23 bundle wedged after its stores landed; cause OPEN, unrelated to the values.

Each probe is a two-line program in x17 syntax, run from a clean engine with the `pre` registers set; the `post` register is read
back. A measured result becomes a gate in `src/silicon_gates.zig`.

## x19-shift: 0x19 ALU op 23

Rule today (isa.toml, fact 147): b >= 0 gives sat24(a << b); b < 0 gives the rounded
a >> -b. Only a = 7, b = 8 is gated (own-x19alu). sosumi uses the negative count as its only
right shift.

    x19.alu s7, s3, s6, #23
    x17.halt

| Probe | s3 | s6 | Rule predicts s7 | Kills the rule |
|---|---|---|---|---|
| own-x19shr-round | 0x000708 | 0xFFFFFC (-4) | 0x000071 | 0x000070 (floor) |
| own-x19shr-neg | 0xFFF8F8 (-0x708) | 0xFFFFFC | 0xFFFF90 (half up) | 0xFFFF8F (floor or half away) |
| own-x19shr-23 | 0x7FFFFF | 0xFFFFE9 (-23) | 0x000001 | 0x000000 |
| own-x19shl-neg | 0xFFFFFF (-1) | 0x000004 | 0xFFFFF0 | 0x7FFFFF (unsigned clamp) |
| own-x19shl-sat | 0x400000 | 0x000002 | 0x7FFFFF | 0x000000 (wrap) |
| own-x19shl-satneg | 0xC00000 | 0x000002 | 0x800000 | 0x000000 or 0x7FFFFF |

## x02-round: the rounded multiply on signed operands

Rule today: x02.mpy gives round(a * b / 2^23), half up, both operands signed in the
signedness setting sosumi's `mulRound23` assumes. Only 0x4000 * 0x6000 is gated (own-x02mpy).
`mulWide` (exact 16 x 24 products) depends on the half-up rounding of negative products.

    x02.mpy s2, s1, s2
    x17.halt

| Probe | s1 | s2 | Rule predicts s2 | Kills the rule |
|---|---|---|---|---|
| own-x02mpy-neg | 0xFFC000 (-0x4000) | 0x006000 | 0xFFFFD0 | anything else (signedness) |
| own-x02mpy-halfneg | 0xFFFFFF (-1) | 0x400000 | 0x000000 (-0.5 half up) | 0xFFFFFF (floor / away), 0x800000 (a unsigned) |
| own-x02mpy-halfpos | 0x000001 | 0x400000 | 0x000001 (0.5 half up) | 0x000000 |

If `x02.mpy`'s encoding is not the both-signed setting (w1[15] = 1, w1[16] = 0), run these in
that setting as well; the both-signed result is the one sosumi needs.

## Not needed

- The low half (2ab)[15:0] (own-x02plo) is the same for signed and unsigned operands.
- A zero test: sosumi derives it from the measured negative test (x == 0 iff neither x nor
  -x is negative).
