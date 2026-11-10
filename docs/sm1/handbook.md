# SM1 programmer's handbook

The sm1 is the audio DSP in the S5L8702 (iPod Classic 6G). This handbook describes it as a
programmer sees it: memories, registers, bundles, timing, arithmetic, addressing, program control
and the host contract. The per-form encodings live in the instruction reference
(`reference.md`, generated from `src/isa.toml` by `x17 doc`).

The layout follows the period DSP family manuals (Motorola DSP56000, ADSP-2100). That is a layout
choice only: nothing here assumes those parts behave like this one.

## How to read this handbook

Every fact carries one of three levels:

- **Measured**: observed on silicon with a probe program of ours. The cite is `§N` or `fact N`, a
  numbered measurement, or an x17 silicon gate (`src/silicon_gates.zig`, `own-*`).
- **Inferred**: follows from measured facts but was not observed directly. It is always labelled.
- **Undefined**: either never measured, or measured to wedge or to be inconsistent. Programs must
  not rely on it. Appendix A lists these.

This document carries interface facts only: what the machine does. It says nothing about the
firmware's own sm1 programs.

## 1. Overview

- One sequencer issuing **bundles** of 1 to 7 32-bit **syllables**. A bundle's slots all read
  their operands before any slot writes (§47, §491).
- Two data memories: **X** (24-bit words) and **Y** (16-bit halfwords), plus a 4-lane program
  memory.
- A 24-bit data path, with a 48-bit multiply-accumulator (§701).
- Three address banks: **e** (Y addresses), **A** (X addresses) and **R** (modifiers). Several
  address units combine them: AGU10, AGU10B, AGU3, AGUA and agu_xr.
- No interrupts, no traps, no flags register (none found). Hazards are the programmer's job.

**Identity.** The block's PrimeCell ID registers (+0xFD0..+0xFFC) read designer 0x3B (ARM), part
0x902, class 0xD. OpenOCD's part table names 0x902 "OptimoDE DESS", ARM's
configurable VLIW data engine. The patent lineage is AudioDE (US 7,920,584 B2) and PICO/VLIW. These corroborate the identity; no encoding in this handbook comes from them.

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
- **Mode 0** programs use X:0..0x1BFF. X is 0x1C00 words with no alias: above it, stores vanish
  and reads give 0 (§649, §653). Y:0..0x2E7F is 16-bit and host-visible in place (§653).
  Y:0x2E80..0x2EFF is one 32-bit host word per address: a store keeps s[23:0] in bytes 0..2 and
  leaves byte 3 untouched, so the host masks or sign-extends it (§656). 128 24-bit samples per
  fill.
- **Stores through A0 and A31.** `stx` via A0 lands at the previous bundle's immediate or result;
  via A31 at X:0 (§649).
- **Mirrors and wrap** above the top of any space: undefined. Runaway pcs reached 0xEDBB (§18).

## 3. Registers

| Bank | Count | Width | Notes |
|---|---|---|---|
| s | 16 | 24 | data; s0 reads 0 and drops writes (§438), but x17's `mv` reads the real s0 (§70) |
| e | 16 | 16 | Y addresses, wrap at 16 bits (§59, §88); `rop` passes e and R to s sign-extended (§810) |
| R | 16 | 16 | modifiers; R0 is hardwired 0 (§88) |
| A | 32 | 16+ | X addresses; A0 reads 0 (§122); tst sees A as 16-bit signed (§375); wider bits unknown |
| p | 16 | 1 | predicates; p0 is hardwired true (§86) and writes to it are undefined (after §608) |

**Latches**, readable through slot muxes:

- **yd**: a 16-bit Y-read latch. Measured to set it: yld10, x03, x02's R-slot Y side, x16alu's
  mux Y read, the x17 class-101 fresh read, x19aagu's byte load (§565, §571) and x00's s-slot
  sub-mux 11 fresh read (after §595), x18 class 111's muxes 3-5 fresh read (after §597), and x03's override-11 fresh read (after §602). Y stores don't
  (§550). Other forms are unmeasured, and so is the timing within one bundle: don't read yd in
  the bundle that refreshes it.
- **xd**: the last X read (§161).
- **xa**: the X-store address, used before its own bundle updates it (§90).
- **ax**: the dual-load X address (§58).
- **xi**: the X index (§116).
- **T**: a phase toggle (§180).
- **MAC**: the 48-bit accumulator, read out as hi = [47:24] and lo = [23:0] (§701). Older
  readings of [39:24] and [15:0] were 16-bit dumps (§203, §213).

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
| `ldi`, `lde`, `lda`, then a bundle reading that register | none: the next bundle sees it (§659; supersedes §157's 4 nops). Measured for ldy via a new e, an x19a43 read via a new A, and x17sty of a new s |
| alu, then ptst on its result, then a conditional jump | none; the branch goes the right way both ways (§662) |
| x02's A step (A<k> + 1), then a bundle using A<k> | none; steps chain back to back, and the x02's own X read uses the old A (§660) |
| x16ld's A write, then a load through it | 2 instructions (§122) |
| x02, then an x02 reading its result; a shift, then an alu reading it; a store, then two `ldi` | none (§666). The old §46, §91 and §188 windows were a sticky wedge left by earlier runs, not hazards |

**Delay slots.** Every jump and branch executes the next 2 bundles (§83, §94, §302), of any class
and length: ldi, lde/ldy, x17sty, a 6-word x02, a 4-word x18 and a 4-word x19a43 read all run
there, and the jump still lands right (§661). The target counts words from the jump, so long slot
bundles add to the offset; labels handle it. Fill slots with work, not nops.

**Side writes (§345, §349, §360, §429, §654, §658).** Several templates carry a second
destination field that writes even when the program never asked for it. Zero it unless the
write is meant:

| Form | Side write |
|---|---|
| x19a43 | A<h[29:25]> <- AGU3[w3[24:20]](x = A<w3[19:15]>, i) |
| x18 class 011 | A<2*w1[3:0] + h[31]> <- the result |
| x03 (A-copy mode) | e<2*w4[2:0] + w3[31]> <- e<w3[26:23]> - 1 |
| x17sty, x17ld, x17agu2 | e<2*w1[2:0] + h[31]> <- AGU10[w1[10:7]](x = e<w1[18:15]>) |
| x02 multiply templates naming A1 | A1 <- A1 + 1 |
| x02.asrc An, x02.esrc | A1 <- An + 1, or A1 + 1 (§157, §795): read A1 before them |
| x01 (R slot) | R<{w4[0], w3[31:29]}> <- R5 - 2 in the norm word (§669) |
| x01 | e<w4[12:9]> <- value unknown (§669): the emulator refuses a nonzero field |
| x18 class 111 MAC templates | A14 <- A14 + 1 (§668; field unknown, not modelled) |
| x03 A copy | A<w2[21:17]> <- A<w3[4:0]> (fn w2[26:22] = 5) or + 1 (fn 0) (§670, §672) |

The assembler's templates clear these fields. Used on purpose, they are free post-increments and
decrements: a pointer step rides on a compute or store bundle instead of costing an x18.eagu.

## 6. Timing

- About **37.4 ns per bundle** (≈26.7 M bundles/s, §379), flat from 36.8 to 37.4 ns across a
  1-word alu, the 6-word x02 multiply, a Y load, an X store and alu/ldy interleaved. Lengths 2-5
  and 7, and a bundle reading X and storing together, are not timed.
- A taken jump costs 3 bundle times: the jump and its two delay slots, no extra stall (§379).
  The emulator's bundle count already includes them. A not-taken jump still runs its slots
  (§680); its cost is untimed, so count 3.
- One bundle stream: no second issue path or engine-side coprocessor is known. The packet feed
  (§82) is filled from the host side and the engine waits on it: an I/O wait, not parallel work.
- The four program "lanes" are interleaved banks that let one fetch cover a whole bundle, not
  parallel execution units.
- The clock rate (~27 MHz) is *inferred*: the clock registers were not read, and the clock gate is
  undefined (§12).
- Memory stalls are not characterised.

`x17 run --profile N` gives the bundle counts to set against this rate.

## 7. Arithmetic

**Wrap, not saturate.** The ALU wraps at 24 bits (§497, §518). The exceptions are listed below.

- **Multiply (Q23):** rounds half up. Only -1 x -1 saturates, to 0x7FFFFF (§386, §402). x02's
  >>23 form is floor(x + 1/2) on both signs; its >>24 form (the MAC high part) is a true floor
  (§657).
- **Low products.** Every low-product slot measured (x02's second destination, x03's, x18's)
  holds 2P mod 2^24 (w1[17] = 0 on x02/x03) or P mod 2^24: signed 24-bit, not masked to 16
  (§655, §657, §663). Mask with x18.and when 16 bits are
  wanted. An X read into s0 is dropped (§652).
- **MAC:** 48-bit accumulator, saturating on overflow (§701).
- **Shifter (op 23):** left shifts saturate; right shifts round half up. Counts of 24 or more
  saturate or shift everything out (§366, §515). `asrrs` rounds (§66).
- **Adder code 4:** a right shift with rounding (§211, §549).
- **|b|** exists (§238). **norm(b)** returns the redundant sign bits at 24-bit width (§502).
- **Other multiplies, one bundle each.** Operands are 24-bit signed unless noted.
  - `x16.mul`: s <- (a b) >> 23, truncated (§118). `x16.mul2`: (a b [+ 2^22]) >> 23 [>> 1]
    by its round and half bits, and writes 0 when head[28] = 0 (§121); S2[3:2] = 11 adds an X
    store in the same bundle (§209). `x16.agumul` runs the multiply beside an AGU slot (§212).
  - `mul13`: s <- round(a b / 2^23) (§128, §697), plus a head ALU slot and, with
    S[16:15] = 11, the `x13.st` X store, all reading before any write (§535): a Q23 product, an
    ALU op and a store for the cost of one bundle.
  - `x19.mac`: a multiply whose s slot takes the product's high part and whose low part can go
    to an e or R (§145, §386), with no saturation through |xy| = 2^41, the measured range. A
    head ALU op, an A step and an X store share the bundle (§312, §384, §385). By w1[17]:
    - 0: low = (2xy)[15:0], the doubled product; high at 24 bits by w1[15]: 1 (the pmul form)
      rounds half up, round(xy / 2^23) (1.5 -> 2, -1.5 -> -1); 0 truncates, floor(xy / 2^23)
      (0x7FFF^2 -> 0x7F, -1 x 0x7FFF -> -1) (§312, §386);
    - 1: high = floor(xy / 2^24), low = low16(xy), undoubled (§145, §312);
    - undefined (§312): w1[16] = 1 gives d + x instead, and w1[12:11] add a high-byte term that
      depends on state. The emulator refuses both.
    The head ALU runs for w1 classes 010, 011 and 101; w1[31:29] is also the X store source's
    low bits, s<{w2[0], w1[31:29]}> (§384, §807).
- **Third-slot ops on x02 (§627, §628).** Besides the adder codes, the third slot runs code 6
  (asr b by a, no rounding, no saturation), code 7 (asrrs: rounds right, saturates left), codes
  22/31 (a logic table: xor, xnor, or, and, not a, a; code 22 then shifts left by w5[5:2]) and
  codes 32/34 (clear). w5[5] negates the shift count.
- **Product accumulate (§603-§605, §600, §601).** x03's high product adds 2ab to the 48-bit
  accumulator built from two s registers and takes [47:24], rounding half up on request. x18's
  class-110 multiply gives (2ab)>>24 doubled or integer, signed or unsigned. x18's MAC can add
  2^23 for round-to-nearest.
- **x16alu post-op (§563, §564).** On the 3-syllable x16alu form with S2 class 111,
  head[30:25] selects a post-operation and S2[7:4] a shift count n. The operation is a shift
  (left, arithmetic right, or rounded right) of the native ALU result, of a logic result, of a or
  of b, or else one of a few fixed functions. Operands are 16-bit signed, and the shift sees the
  exact sum: (0x5505 + 0x4404) asr 4 = 0x0990, not a wrapped value. The full table is in the
  reference. Cells outside the six measured (op, S1[15]) rows are undefined.

Other rounding modes: unknown.

## 8. Address generation

**Units.** AGU10, AGU10B, AGU10Q, AGU3 and AGUA are 16-entry tables. AGU10Q adds a third input
q = e<k>: q, q+1, y-q, q-y, y-q±1 (§636); AGU10B is AGU10Q with q = 0. They hold only add, subtract, ±1,
negate and immediate, plus one AGUA entry, (x-1)//2+i (§109, §120, §144, §149). agu_xr is the
one-A, one-R adder that x02, x03, x16alu and x19agu share (§338, §403). Post-modify slots exist
(§100, §157).

**Wrap-once modulo (§566-§568).** agu_xr's result can wrap once by a modulus register:
`M = R<k>`, with k taken from a 4-bit field in the bundle (k = 0 means no wrap).

    sum <  0 (16-bit signed)  ->  sum + M
    sum >= M                  ->  sum - M
    otherwise                 ->  sum

Measured on x02's A-slot sum and its R slot, and on x16alu's R slot. This is circular-buffer
addressing for buffers of length M, provided each step moves by less than M. x02's second A
write wraps the sum, then adds A unwrapped (§626). No bit-reversed mode has been measured.

**Byte loads.** x19agu's and x16agu's s mux 3 load Y[7:0] << 8 (§637, §638); x19agu only with
its w2[26] gate. x16 b16's Y load places the word by S1[13:12]: 00 gives 0, 01 sign-extends,
10 zero-extends (§800), 11 shifts it left 8.

**Immediates into A.** ldi2 and ytst heads also load their immediate into an A register (§634,
§635); A0 is never written there (*inferred*).

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
- **Unconditional relative jump:** jr2 runs a head ALU word and jumps to q + 2·sx14(w1[18:5])
  (§617).
- **Bit tests:** ptst op 0x1E sets p from (a & b) != 0, measured against s10 and s12 only
  (§632, §633).
- **Loops:** x03's conditional jump to itself, counted by its e counter through p3 (§675,
  §679, §680; §263's "repeat" was this jump). DO-style hardware loops were disproved (§71, §449).
- **Conditional stop:** an x17sty with S1[28:27] = 00 and S1[5] = 0 is a stop form keyed by
  S1[2:0]: 1 stops after the store when p1 is set, 3 runs away, others always stop (§648, §667). The store templates set S1[27] so no plain store is an exit.
  Under R mux 2 (S2[3:1] = 2) its e side write takes the R slot's AGU result, the value R
  gets; when that e is the store's own address, a stop stores a second time there (§801).
- **Calls and returns:** undefined (§230, §310).
- **Wedges:** runaway words (§16, §44), low5 0x1A-0x1F after the stop (§72), a missed halt
  deadline, and the per-form wedge bits in the reference (for example xld10 S1[18] = 0, §559).
  `x19.ld` never completes with 0, 1 or 2 bundles between it and an `st.wr` (§804); 7 between
  is measured clean, 3 to 6 are not, so keep 7 or more. `x17.sty` before it is harmless.

## 10. Writing fast code

Measured idioms, cheapest first:

- **No padding.** Loads, A steps, alu results feeding branches and back-to-back multiplies all
  need no gap (§659-§662, §666). Every nop in a program should be a delay slot nobody could
  fill.
- **Fill both delay slots** (§661). Work from the fall-through path goes there when the taken
  path does not read what it writes; a loop's first bundles go there when the back edge jumps
  past them.
- **Counted loop.** x03 with w2[28] = 1 is a jr by sext(w1[28:18]) on p<w5[17:14]>, any k:
  w2[27] = 1 jumps if set, 0 if clear; w2[31:29] is ignored (§394, §679). p0 reads 1, so k = 0
  with w2[27] = 1 always jumps (§682). w2[28:27] = 01 jumps unconditionally (§329, §679). Both delay slots run on every pass (§680). With
  w2[28] = 1 the e counter e<2*w4[2:0] + w3[31]> <- e<w3[26:23]> - 1 also writes
  p3 <- (e > 0, signed 16) after the jump has read p (§675). So an offset-0 jump on p3 is a
  counted loop of three bundles (the x03 and its two slots): preset e = N - 1 and p3 = 1 for N
  passes (N >= 2), leaving p3 = 0 and e = -1. The x03's X store steps its A every pass, ending
  at A + N. Measured on mode-0 store rows. On the x03 tap row the jump works with a nonzero
  offset, all three forms, but its counter's p3 write is not seen, so a jump to itself there never ends (§682):
  a counted tap loop must be a jump over a longer body, or unrolled. A halt in
  a taken jump's delay slot still lets the target bundle run once before the stop (§681).
- **Loops.** The counted loop above is the only zero-overhead loop, three bundles long. For
  longer bodies, a pcond on A against R plus a jmp whose two slots do work (§662) costs about
  one bundle. Unroll when the body is short. A MAC row with w2[31:29] = 011 runs once and then
  hangs (§673): the emulator refuses it.
- **Free pointer steps.** Stores (x17.styi, §658) and x17ld loads (§429) post-increment an e; x03
  counts an e down (§349); x02 steps an A (§660). Each replaces an x18.eagu or aset bundle.
- **Free wraps.** agu_xr's wrap-once modulo (§8, §566-§568) wraps a ring or window pointer
  inside the step it already takes, instead of an x17.mve or x18.eagu bundle per wrap. Each step
  must move by less than M. Measured on x02's A slot and R slot and x16alu's R slot.
- **Packed bundles that are measured** (§155, §183, §185, §347-§350, §513, §174, §203):
  - x02: X read, multiply, second product, third-slot ALU, A step, and an optional X store;
  - x03: X read, multiply, accumulate, A copy, e counter, and a Y store;
  - x18 class 111: Y load, negate, e step and MAC;
  - x03 one-bundle tap (§671, §672): s <- X:[A], A <- A + 1 and {hi:lo} += 2 s b. The read
    lands after the multiply has taken its operands, so each bundle multiplies the value the
    previous bundle loaded: prime the register first, and the last bundle's load is a dummy.
    Measured with one stepping pointer and a fixed second operand.
  Other combinations are unmeasured.
- **Count leading bits in one bundle.** The x01 slot's xori code with m = w5[29:27] = 5 gives
  s <- norm(b), the redundant sign bits at 24 bits: norm(0x100) = 14, norm(-0x100) = 15,
  norm(0x7FFF) = 8, norm(0) = 24, norm(-1) = 23, norm(-2) = 22 (§502, §666, §669). A run of
  k >= 1 leading ones gives norm = k - 1; test the sign first for k = 0. The result is ready for
  the next bundle. Use the clean word with both side writes cleared (§669):
  `28001381 00000580 280000e0 000280a0 00040002 28200900 00300090` = s9 <- norm(s8), written
  `x01.norm s9, s8`. Its m = 7 sibling is `x01.selneg sD, sB, #i` = b < 0 ? b : i.
- **Running MAC.** Mode 5 computes acc = hi * 2^24 + lo + 2ab with lo taken unsigned 24 bits,
  and returns lo' = acc mod 2^24. With rounding off, hi' = floor(acc / 2^24), so feeding
  (hi', lo') back is an exact running sum: one bundle per tap, round once at the end. Two forms
  are measured: x18 class 111 as `x18.macf` (w3[8] = 0; `x18.mac` adds 2^23 to hi only, §665,
  §668) and x03 mode 5 with w1[15] = 0 (§670). Both take full 24-bit signed a and b into a
  48-bit acc (§701): hi and lo are 24 bits each, exact within [-2^47, 2^47 - 1] and saturating
  at both ends, to {0x7FFFFF, 0xFFFFFF} and {0x800000, 0} (§702).
  x18 chains of 4 or more wedged after the program's stores, values intact; x03 did not, so
  prefer x03 for long sums. The one-bundle tap is written `x03.tap sL, #n, sB, sHi, sLo`:
  sL <- X:[An] after the multiply reads sL, An += 1, {sHi:sLo} += 2 sL sB. All five are free
  fields; the word repeats sL, An, sHi and sLo in further fields and the assembler fills them
  (operand rows named `=x`). Prime sL before the first tap. `x03.tap0`, the same word in mode 4,
  writes {sHi:sLo} <- 2 sL sB ignoring their old values (§702): use it as the first tap.
  The Y-side twin is `x18.ytap sL, eN, sB, sHi, sLo`: sL <- Y:[eN] after the multiply, eN += 1
  (§703); its Y load zero-extends (§705). No bundle reads both an X and a Y operand (x03 has
  one read mux, §703), so a tap with two stepping pointers takes two bundles (§705):
  `x19.ld sB, #k, eN, #2` (sB <- Y:[eN] sign-extended, eN += 1) then `x03.tap`, or an x02 X
  load with an A step then `x18.ytap`. A separate load feeds the very next MAC; the tap's own
  load lands after its MAC, so run one tap before the loop to prime sL (its MAC is 0 * 0).
  Keep x19.ld's w1[20] and w1[5] clear: set, they write the tap's A pointer and grow the e
  step on each later load, so the pair reads past its data (§790, §791, §793). Pure x03.tap
  chains, one or two X pointers, are exact on silicon (§791, §792).
  The sum is the named {sHi:sLo} pair, nothing hidden: taps that rotate through several pairs
  keep each one an exact, independent running sum, and a pair no tap names is untouched
  (§798). So one loaded operand in sB can feed several sums, one tap each: four sums sharing
  x[k] cost 1.25 bundles a product with an x02 load into sB every four taps (§798).
  Both pairs are exact over 32 taps (§705; the x03 pair with the clear word, §794); prefer the x03 pair (signed Y, no x18 chain). The rounded forms (w1[15] = 1, `x18.mac`) are still measured at
  16-bit operands only.
- **No computed jumps.** Every measured jump takes an immediate target: x00/x01 control
  relative and absolute (§230, §575), 0x04 (§304), §301/§302, and jmp/jmp-if (§661). None takes
  its target from a register or memory. Share code through absolute-jump dispatch, counted
  repeats and predicated jumps.
- **24-bit logic, min/max and abs (§708).** `x18.or`, `x18.max` and `x18.min` (class 011,
  signed 24-bit compares) and `x18.una sD, sA, #7` = |a| over 24 bits. abs wraps at -2^23:
  abs(0x800000) = 0x800000, so a max|v| scan must treat 0x800000 as the largest magnitude.
  x03's X store writes all 24 bits of s, so a counted loop copies 24-bit words.
- **s0 is not a zero source for stores (§709).** `stx An, s0` stores the xd latch << 8 (the
  last X word read, shifted), not 0. To zero memory, store a register cleared by `ldi #0` or an
  ALU result. As a destination, s0 drops writes: `x03.tap0 sL, #n, s0, s0, s0` is a pure X load
  and step.
- **Selects without branches.** x19alu's write can be conditional on p1 (§646); the x01 slot's
  m = 7 gives b < 0 ? b : i (§502). `psel` is a 24-bit select, s <- p ? a op b : b, with op from
  the x16 ALU table (add measured) and p's sense set by S[21]; its head write,
  s<head[24:21]> <- #head[20:5] or 0, runs in the same bundle (§497, §531, §607, §608).
- **Free R load.** x18 class 011 with w3[3] = 1 writes #head[20:5] to R<w3[10:7]> beside its
  ALU slot (§688); an immediate of 0 clears it.
- **Latches at entry.** One X read latch (xd) serves x03 and x18; the latest read sets it,
  stores do not, and it carries from the previous run (§690). yd starts each run at a value
  the program did not set. Read before relying on either.
- **Constants.** ldi is 16 bits, sign-extended: 0xFFFF costs ldi plus two adds, or a mask.
  Compute constants once per loop, not per pass.
- **Masks.** Every low product is 24 bits (§655, §657, §663): x18.and when 16 are wanted.

`x17 run --profile N` lists the hottest bundles with their source lines, macro call sites
included.

## 11. Host interface

Offsets are from the sm1 block's MMIO base.

- **Ownership lock +0x100C.** While the engine holds it, SRAM reads return a per-boot sentinel and
  host writes don't land. Writing 0 releases it (§13). After a run, uploads don't land until the
  driver's release sequence.
- **Load.** The ARM copies the images in with plain CPU stores, in two passes. Pass 1 is data plus
  a 10-14-row stub at row 0; pass 2 is the main program at row 0x40 (§1, §2). How pass 2 is
  entered is undefined. Test programs enter at q 0 (§45).
- **Arm order:** +0x988, +0x98C, +0x118, +0xA8C.
- **Input:** a DMA descriptor chain at +0x100 feeds a stream port that is not visible in SRAM
  (§82). The program reads it as a bit stream:
  - `ld0e.sen #addr, #n` with s12 = 1 and s14 = 4 enables it (any s14 with bit 2 set and bits 1:0
    clear). Until then reads give 0 and consume nothing (§783).
  - `x16.getb #k, #n` loads A<2k+1> with the next n+1 bits (1 to 13), MSB first, right-aligned.
    Each read continues where the last stopped, also across packets (§783, §784).
  - A read with too few bits left stops the program at that read: +0xA98 = 0x04C00206, +0xC10 =
    0x10000 | the read's q. Arming the next packet resumes it, with no feed answer (§784).
- **Output:** a two-slot ring in Y, filled alternately per packet (§785):
  - slot k's samples at Y:0x80 + 0x800k, up to 2048 16-bit units;
  - slot k's entry: Y:2k+5 = 1 (owned), Y:2k+6 = length in units;
  - then Y:0xFD94 bit 0 = 1, and the stop with bit 2.
  - A slot is reused only once its owned word is 0; the host releases it by writing 0 there.
    With both slots owned, the program writes Y:0xFD95 = 0x21 and stops in the bit-12 feed
    wait (+0xA98 = 0x04801202), the descriptor still queued (+0x114 = 4) (§797). Whether
    releasing a slot and arming then resumes that program is unmeasured.
  - A slot's length is that frame's samples times its own channel count: a mono frame stays
    mono, and an MPEG-2 frame is half an MPEG-1 one (§797).
- **Status, +0xA98:**
  - bit 2: completion, set by the program (§13). It raises VIC line 18; acknowledge with +0xC48 <- 4.
  - bit 26: running (§16).
  - bit 1: a continuing x17 ran (§98).
  - bit 12: feed wait, answered with +0x804 = 1 and +0x858 = 1 (§82).
- **+0xC10:** bit 16 = running, [15:0] = the q pc. Bits 14-15 seen during stalls are undefined
  (§18, §35, §73).
- **Do not touch +0x1004.** It is a driver scratch flag; poking it dropped the unit off USB (§331).

The codec-side contract (parameter block, input stream, output ring) is in `src/host.zig` and
sosumi's `specs/questions.md`.

## 12. Instruction reference

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
- Modulo or bit-reversed modes outside agu_xr's wrap-once.
- x16alu post-op cells outside the six measured (op, S1[15]) rows.
- Every bit a form's reference entry marks OPEN, and every bit it marks as a wedge gate.
- Power-on register values.

## Appendix B. Evidence

- `§N` / `fact N`: a numbered silicon measurement, kept in a private lab notebook; each entry
  holds the hypothesis, the probe, and the verdict.
- `own-*`: an x17 silicon gate. The program and its expected end state are in
  `src/silicon_gates.zig`, and `zig build test` replays them against the emulator.
- x99: the dirty side's harness that runs one bundle under canary presets.
