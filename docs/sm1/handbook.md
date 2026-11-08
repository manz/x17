# SM1 programmer's handbook

The sm1 is the audio DSP in the S5L8702 (iPod Classic 6G). This handbook describes it as a
programmer sees it: memories, registers, bundles, timing, arithmetic, addressing, program control
and the host contract. The per-form encodings live in the instruction reference
(`reference.md`, generated from `src/isa.toml` by `x17 doc`).

The layout follows the period DSP family manuals (Motorola DSP56000, ADSP-2100). That is a layout
choice only: nothing here assumes those parts behave like this one.

## How to read this handbook

Every fact carries one of three levels:

- **Measured**: observed on silicon with a probe program of ours. The cite is `fact §N`, the
  dirty-side lab notebook, or an x17 silicon gate (`src/silicon_gates.zig`, `own-*`).
- **Inferred**: follows from measured facts but was not observed directly. It is always labelled.
- **Undefined**: either never measured, or measured to wedge or to be inconsistent. Programs must
  not rely on it. Appendix A lists these.

This document carries interface facts only: what the machine does. It says nothing about the
firmware's own sm1 programs, and it does not reproduce `isa.toml`'s `[tables.*]` sections.

## 1. Overview

- One sequencer issuing **bundles** of 1 to 7 32-bit **syllables**. A bundle's slots all read
  their operands before any slot writes (§47, §491).
- Two data memories: **X** (24-bit words) and **Y** (16-bit halfwords), plus a 4-lane program
  memory.
- A 24-bit data path, with a 40-bit multiply-accumulator (§203).
- Three address banks: **e** (Y addresses), **A** (X addresses) and **R** (modifiers). Several
  address units combine them: AGU10, AGU10B, AGU3, AGUA and agu_xr.
- No interrupts, no traps, no flags register (none found). Hazards are the programmer's job.

**Identity.** The block's PrimeCell ID registers (+0xFD0..+0xFFC) read designer 0x3B (ARM), part
0x902, class 0xD (the spec notes §2). OpenOCD's part table names 0x902 "OptimoDE DESS" (§7), ARM's
configurable VLIW data engine. The patent lineage is AudioDE (US 7,920,584 B2, §11) and PICO/VLIW
(§36). These corroborate the identity; no encoding in this handbook comes from them.

## 2. Memory map

| Space | Unit | Host view | Notes |
|---|---|---|---|
| X | 24-bit word | `0x22030000 + 4*a`, one word per 32-bit cell | stores keep 24 bits (§51, §371); some read paths see only [15:0], placed at [23:8] (§161, §236) |
| Y | 16-bit halfword | `0x22038000 + 2*a` | 16-bit addresses; only some windows exist (below) |
| P | 32-bit syllable | lane n at `0x22020000 + n*0x4000 + 4*row` | 4 lanes x 0x1000 rows = 0x4000 syllables (§2) |

- **Y windows.** A 0x120-byte table the host uploads at +0x938 sets which Y windows exist; some are
  32 bits wide (§2). That the engine translates through this table is *inferred*.
  Y:0xFD80..0xFDFF sits at 0x2203FF00, and the status mailbox is Y:0xFD94 = 0x2203FF28 (§18, §81).
- **Program order.** The sequencer's q order runs column-major over row pairs:
  `q = 8*(row/2) + 2*lane + (row%2)` (§45). The pc counts syllables in q order.
- **Mode 0** programs use X:0..0x1BFF (plan §0).
- **Mirrors and wrap** above the top of any space: undefined. Runaway pcs reached 0xEDBB (§18).

## 3. Registers

| Bank | Count | Width | Notes |
|---|---|---|---|
| s | 16 | 24 | data; s0 reads 0 and drops writes (§438), but x17's `mv` reads the real s0 (§70) |
| e | 16 | 16 | Y addresses, wrap at 16 bits (§59, §88) |
| R | 16 | 16 | modifiers; R0 is hardwired 0 (§88) |
| A | 32 | 16+ | X addresses; A0 reads 0 (§122); tst sees A as 16-bit signed (§375); wider bits unknown |
| p | 16 | 1 | predicates; p0 is hardwired true (§86) |

**Latches**, readable through slot muxes:

- **yd**: a 16-bit Y-read latch. Measured to set it: yld10, x03, x02's R-slot Y side, x16alu's
  mux Y read, the x17 class-101 fresh read, x19aagu's byte load (§565, §571) and x00's s-slot
  sub-mux 11 fresh read (after §595), and x18 class 111's muxes 3-5 fresh read (after §597). Y stores don't
  (§550). Other forms are unmeasured, and so is the timing within one bundle: don't read yd in
  the bundle that refreshes it.
- **xd**: the last X read (§161).
- **xa**: the X-store address, used before its own bundle updates it (§90).
- **ax**: the dual-load X address (§58).
- **xi**: the X index (§116).
- **T**: a phase toggle (§180).
- **MAC**: the 40-bit accumulator, read out as [39:24] and [15:0] (§203, §213).

**Index-0 conventions.** e index 0 in a store tail means "the previous word's [20:5]" (§466); some
slots read Y:0 for e index 0 (§505).

**Reset.** s, e, R, A, p and the latches persist across runs and across the driver's reset (§85,
§88, §90). Power-on values are unknown: initialise everything a program reads.

**Hidden state.** Some internal loop state survives a run and is not visible over MMIO (§26,
*inferred*).

## 4. Bundles

- **Length** comes from the head syllable's low 5 bits; only bits [4:1] are decoded (§39, §44):

  | low5 | syllables |
  |---|---|
  | 0x00-0x01 | 7 |
  | 0x02-0x04 | 6 |
  | 0x05-0x0D | 1 |
  | 0x0E-0x15 | 2 |
  | 0x16-0x17 | 3 |
  | 0x18-0x19 | 4 |
  | 0x1A-0x1F | **undefined**: jumps back to the previous head |

- **No alignment.** A bundle may start at any q and cross rows. Inserting a syllable shifts every
  later bundle (§46).
- **Class bits.** Each non-head syllable's [31:29] is its class. Many forms dispatch on the
  (head low5, syllable classes) tuple. Some class bits are aliases (§551), and some are fields of
  a register index (§555, §556). The reference lists each form's accepted classes.

## 5. Pipeline and hazards

Reads precede writes within a bundle. Across bundles, results land late, and nothing interlocks:
a violated distance gives a lost or wrong value, never a trap.

| Hazard | Minimum distance |
|---|---|
| `lda`, then a bundle using that A | 4 nops (§157) |
| x16ld's A write, then a load through it | 2 instructions (§122) |
| x02, then x02 | 3 nops (§188) |
| shifter bundle, then register writes | writes 2-4 instructions later are lost (§91) |
| store, then two `ldi` | stalls (§46) |

**Delay slots.** Every jump and branch executes the next 2 bundles (§83, §94, §302).

## 6. Timing

- About **37.4 ns per bundle** of any length (≈26.7 M bundles/s, §379).
- A taken jump costs 3 bundle times (§379).
- The clock rate (~27 MHz) is *inferred*: the clock registers were not read, and the clock gate is
  undefined (§12).
- Memory stalls are not characterised.

`zig build bench` turns emulator profiles into bundle counts per stage, using this rate.

## 7. Arithmetic

**Wrap, not saturate.** The ALU wraps at 24 bits (§497, §518). The exceptions are listed below.

- **Multiply (Q23):** rounds half up. Only -1 x -1 saturates, to 0x7FFFFF (§386, §402).
- **MAC:** 40-bit accumulator; the widest measured (§203). The patent's 48 bits are corroboration
  only.
- **Shifter (op 23):** left shifts saturate; right shifts round half up. Counts of 24 or more
  saturate or shift everything out (§366, §515). `asrrs` rounds (§66).
- **Adder code 4:** a right shift with rounding (§211, §549).
- **|b|** exists (§238). **norm(b)** returns the redundant sign bits at 24-bit width (§502).
- **x16alu post-op (§563, §564).** On the 3-syllable x16alu form with S2 class 111,
  head[30:25] selects a post-operation and S2[7:4] a shift count n. The operation is a shift
  (left, arithmetic right, or rounded right) of the native ALU result, of a logic result, of a or
  of b, or else one of a few fixed functions. Operands are 16-bit signed, and the shift sees the
  exact sum: (0x5505 + 0x4404) asr 4 = 0x0990, not a wrapped value. The full table is in the
  reference. Cells outside the six measured (op, S1[15]) rows are undefined.

Other rounding modes: unknown.

## 8. Address generation

**Units.** AGU10, AGU10B, AGU3 and AGUA are 16-entry tables. They hold only add, subtract, ±1,
negate and immediate, plus one AGUA entry, (x-1)//2+i (§109, §120, §144, §149). agu_xr is the
one-A, one-R adder that x02, x03, x16alu and x19agu share (§338, §403). Post-modify slots exist
(§100, §157).

**Wrap-once modulo (§566-§568).** agu_xr's result can wrap once by a modulus register:
`M = R<k>`, with k taken from a 4-bit field in the bundle (k = 0 means no wrap).

    sum <  0 (16-bit signed)  ->  sum + M
    sum >= M                  ->  sum - M
    otherwise                 ->  sum

Measured on x02's A-slot sum and its R slot, and on x16alu's R slot. This is circular-buffer
addressing for buffers of length M, provided each step moves by less than M. Whether x02's second A
write (sum + A) wraps before or after its add is undefined. No bit-reversed mode has been measured.

**Widths.** Y addresses are 16 bits; e and R wrap at 16 (§5, §88). A's width beyond 16 bits is
unknown.

## 9. Program control

- **Halt:** the 3-syllable `[x17, mv, stop]`; stop has [31:29] = 101 and bit 1 set (§31, §47,
  §97). At least an `ldi` plus a store after the stop still execute (§72).
- **Halt deadline.** Certain bundles **arm** the engine: a continuing x17, x02 with w2[30], x18
  with w2[18], jmp modes 12/15, and x00 control 0x0F (§307-§314, §477-§483). After one, the halt
  must come within 7 nops (8 after a bare continuing x17) (§98, §484). Missing the deadline parks
  the engine (+0xA98 = 0x04C00002), and the next run wedges until a reset. Whether the deadline is
  counted in words or cycles is undefined.
- **Jumps:** relative on a predicate (§83, §94), or absolute (mode 4, §310). Two delay slots.
  The wide-bundle jump's offset is sx15({w2[0], w1[31:18]}) from q (§575); its condition
  field w4[31:28] is defined only at 0, always taken; the other codes read hidden state that
  earlier runs change, not p1..p15 (§578).
- **Loops:** "repeat this bundle while p" (§263, §452). DO-style hardware loops were disproved
  (§71, §449).
- **Calls and returns:** undefined (§230, §310).
- **Wedges:** runaway words (§16, §44), low5 0x1A-0x1F after the stop (§72), a missed halt
  deadline, and the per-form wedge bits in the reference (for example xld10 S1[18] = 0, §559).

## 10. Host interface

Offsets are from the sm1 block's MMIO base.

- **Ownership lock +0x100C.** While the engine holds it, SRAM reads return a per-boot sentinel and
  host writes don't land. Writing 0 releases it (§13). After a run, uploads don't land until the
  driver's release sequence (the spec notes "reconfigure").
- **Load.** The ARM copies the images in with plain CPU stores, in two passes. Pass 1 is data plus
  a 10-14-row stub at row 0; pass 2 is the main program at row 0x40 (§1, §2). How pass 2 is
  entered is undefined. Test programs enter at q 0 (§45).
- **Arm order:** +0x988, +0x98C, +0x118, +0xA8C (the spec notes "arm answers").
- **Input:** a DMA descriptor chain at +0x100 feeds a stream port that is not visible in SRAM
  (§82).
- **Status, +0xA98:**
  - bit 2: completion, set by the program (§13). It raises VIC line 18; acknowledge with +0xC48 <- 4.
  - bit 26: running (§16).
  - bit 1: a continuing x17 ran (§98).
  - bit 12: feed wait, answered with +0x804 = 1 and +0x858 = 1 (§82).
- **+0xC10:** bit 16 = running, [15:0] = the q pc. Bits 14-15 seen during stalls are undefined
  (§18, §35, §73).
- **Do not touch +0x1004.** It is a driver scratch flag; poking it dropped the unit off USB (§331).

The codec-side contract (parameter block, packet staging, output ring) is in `src/host.zig` and
sosumi's `specs/questions.md`.

## 11. Instruction reference

`reference.md` is generated by `x17 doc` from `src/isa.toml`, `src/subforms.txt`,
`src/operands.txt` and the emulator handlers' notes. Each form lists its encoding, operand fields,
evidence and notes. CI fails when the file is stale (`x17 doc --check docs/sm1/reference.md`).
The notes are the lab's own and still carry some superseded wording; fixing a note fixes the
reference.

## Appendix A. Undefined behaviour

Programs must avoid all of these. The emulator refuses each one (`UnknownForm`) rather than
guessing.

- Head low5 0x1A-0x1F; ops 0-3 of the x16 ALU (they wedge).
- Calls and returns; how pass 2 is entered; mirrors and wrap at the top of each space.
- Whether the halt deadline counts words or cycles.
- Modulo or bit-reversed modes outside agu_xr's wrap-once; the order of x02's second A write
  against the wrap.
- x16alu post-op cells outside the six measured (op, S1[15]) rows.
- Every bit a form's reference entry marks OPEN, and every bit it marks as a wedge gate.
- Power-on register values.

## Appendix B. Evidence

- `fact §N`: the dirty-side notebook (not public); each entry holds the hypothesis, the
  probe, and the verdict.
- `own-*`: an x17 silicon gate. The program and its expected end state are in
  `src/silicon_gates.zig`, and `zig build test` replays them against the emulator.
- x99: the dirty side's harness that runs one bundle under canary presets.
