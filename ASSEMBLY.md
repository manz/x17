# x17 assembly surface

How we write sm1 programs.  Design locked to what `isa.toml` actually says
about the ISA, not borrowed wholesale from 65c816 / any other target.

## What we know about the ISA (summary)

- **Syllables**: 32-bit words.  Four per row (`row_syllables = 4`).  Lane
  `l` of row `r` lives at `0x22020000 + l*0x4000 + r*4`.
- **Bundles**: 1-7 syllables, length picked by the HEAD syllable's low5
  (`fmt` field = `[4:0]`).  Example: low5 `0x0D` = 1-syllable, `0x17` = 3,
  `0x01` = 7.
- **Forms**: 26 known today, named by their role (`st.y`, `ldi`, `wr`,
  `alu`, `rop`, `yagu`, `rli`, `bcc`, `x17`, `loop`, ...).  Each is picked
  by a `(mask, value)` pair over the bundle's bits; operand fields live at
  form-specific bit positions.
- **x17 specifically**: 3-syllable bundle `[head, S1, S2]` with a
  sub-form grid: `(S1 class = S1[31:29], S2 class = S2[31:29])` picks
  what the bundle does (ALU op, AGU op, immediate load, element update,
  halt).
- **Register files**: `s<N>` scalar regs, `e<N>` element regs, `R<N>`
  16-entry modifier file (R0 = 0, persistent), `p<N>` predicates.
- **Memory**: Y:addr (data), X:[A<idx>] (X data indexed via A regs).
- **PC**: `PC q = 8*(row/2) + 2*lane + row%2` - column-major over row
  pairs.  The main program lands at `main_row = 0x40`.

## Source file shape (one `.s` per program)

```
# A line is either a directive, a label, or an instruction.
# Each instruction encodes to ONE bundle (1-7 syllables of 32 bits each).

# -------------------- directives --------------------

.const SAMPLE_RATE  = 44100
.const WINDOW_LEN   = 0x400

.pass decoder                   # this program loads at row 0x40 (q256)
.ydata 0x20, 0x1234, -1         # host-written Y for this pass
.org 0x44                       # move forward to row 0x44 (zero fill)

# -------------------- instructions ------------------

main:
  # One form per line.  Operands in the order isa.toml's form ref
  # names them: dest first, then sources, then immediates.
  rli   R3, s4, #(SAMPLE_RATE >> 4)   # R3 <- imm16, s4 <- imm16
  ldi   s0, #0x1234                   # scalar immediate
  alu   s7, s2, s9, add               # s7 <- s2 + s9 (op name, not #n)

  # x17 is a bundle NAMESPACE: sub-mnemonic picks the (S1, S2) class pair.
  x17.alu  s4, s9, s3, #5             # s4 <- X17OP[5](s9, s3, n=5)
  x17.imm  s4, #0x1234                # s4 <- 16-bit imm from head[26:11]
  x17.agu  R6, e2, R1, #0xA           # R6 <- AGU10[0xA](x=e2, y=R1)
  x17.halt                            # bundle halts the program

  # Memory stores name the address space explicitly: Y:<expr> or X:A<n>.
  st.y    Y:1, s4                     # Y:1 <- s4  (the stub's canonical store)
  xst     X:A<3>, s2                  # X:[A<3 + head[31:30]>] <- s2
  ystm    Y:[e0], s4                  # Y:[e<head[8:5]>] <- s4 with e update

  # Control flow: labels + bcc, respecting the two delay slots.
back_edge:
  bcc     #0, back_edge               # p0 always; offset (target - q) / 4

# -------------------- comptime asserts --------------

.assert sizeof(main) <= 16 rows       # layout budget
.assert cycles(main) <= 980           # per-sample deadline (once cycle
                                      # column lands in isa.toml)
```

## Operand grammar (v1)

| Kind              | Syntax                           | Notes                                                     |
|-------------------|----------------------------------|-----------------------------------------------------------|
| Scalar reg        | `s0` .. `s15`                    | 4-bit field in most forms                                 |
| Element reg       | `e0` .. `e15`                    | 4-bit field                                               |
| R reg             | `R0` .. `R15`                    | R0 = 0, persistent                                        |
| Predicate         | `p0` .. `p15`                    | p0 is "always true" in `bcc`                              |
| Y address         | `Y:1`, `Y:[e0]`, `Y:0xFDFF`      | literal or `[e<reg>]` indexed                             |
| X address         | `X:A<3>`, `X:A<2+head[31:30]>`   | A regs 0-7, modifier from head bits named by the form     |
| Immediate         | `#0x1234`, `#(expr)`             | width form-specific; const expressions fold at assemble   |
| Label ref         | `back_edge`, `(end - start)`     | resolves at final symbol pass                             |
| Named op          | `add`, `sub`, `shl`, ...         | symbolic op selector when the form has an op subfield     |

## Why mnemonic-per-form, not raw bundle encoding

Two paths were weighed:

1. Raw `x17 head=0x..., S1=0x..., S2=0x...` with hand-set bits.
2. One mnemonic per form; the assembler encodes the bitfields.

Path 2 wins because isa.toml already carries the mask/value and operand
bit positions for every form - the information to emit bundles is there,
and if the author has to re-type bit positions in source, they'll drift
from isa.toml the first time an opcode form is refined.  Mnemonic-per-form
means isa.toml stays the single source of truth; a form change
re-encodes every program that used it on the next build.

## Why sub-mnemonics for x17 (and nothing else)

x17 is the ONE form that is really a grid of forms selected by two class
fields.  Every other form in the 26 is picked by a single (mask, value)
pair and has a flat operand layout.  Rather than extend the class-pair
test to every form, x17 gets a namespace (`x17.alu`, `x17.agu`,
`x17.imm`, `x17.e`, `x17.halt`) and the rest stay flat.  Matches how
isa.toml ALREADY describes them: "x17 S1 class 011 + S2 class 111: imm
load" reads as `x17.imm` without further ceremony.

`.stop` suffix: any x17 sub-form whose base has S1[22:20] = 000 and no operand on those bits
takes `.stop`, the same operands with S1[22:20] = 010 (`x17.ms.stop`, `x17.sty.stop`).  The
slots run, two more bundles run, then the engine stops without a visible halt (fact 457,
465, 478).  disasm prints the suffix back; a form like `x17.ld`, whose operands cover those
bits, has none.

## Placement and layout

An image build (`x17 image -o out.x17i [-g out.json] file.sm`) takes a source
of passes; the flat `x17 asm` takes one program at q0 and refuses the pass and
data directives.

```
.mode 3                         # optional: the configure mode, before any pass (fact 960)
.pass init                      # an init pass: loads at row 0 (q0), arms in turn
.xdata 0x100, 0x123456, -1      # host-written X for this pass (24-bit words)
.ydata 0x40, 0xBEEF             # host-written Y for this pass (16-bit words)
    ...code that halts...
.pass init                      # the next init pass, again at q0
    ...
.pass decoder                   # the decoder: loads at row 0x40 = q256 (fact 770)
.param 0xFD83, 4                # the parameter block, Y:0xFD80..0xFD93
    ...code...
```

- Bundles pack in q order, `q = 8*(row/2) + 2*lane + row%2` (fact 45); a
  row holds 4 syllables, one per lane, and rows go in pairs.
- `.mode N` (optional, once, before the first `.pass`): the configure mode the
  image is built for, 0, 1, 3 or 5.  Each mode maps its own DSP Y (fact 960);
  with a mode, `.ydata` must sit in that mode's 16-bit range (mode 0
  0..0x2E7F, 1 0..0x177F, 3 0..0x227F, 5 0..0x15FF), and `x17 play` runs
  with that map (a different `--mode` is refused).
- `.pass init` / `.pass decoder`: init passes come first, in load order;
  at most one decoder, last.  A source with no `.pass` is one init pass.
  Every pass needs at least one bundle.  Labels are q values, so a decoder
  label reads 256 and up.
- `.org row`: move forward to an even row inside the current pass,
  zero-filling.  **A zero syllable is not a no-op** (low5 0x00 heads a
  7-syllable bundle): code must jump over the gap or halt before it, never
  fall into it.  Backwards and odd rows are refused.
- `.xdata addr, w...` (X below 0x1C00, fact 653; 24-bit words),
  `.ydata addr, w...` (Y below 0x2E80, fact 760; 16-bit words, fact 758),
  `.param addr, w...` (decoder pass only, Y:0xFD80..0xFD93; Y:0xFD94 is the
  engine's status header).  Words are constant expressions; negatives store
  as two's complement.  Blocks of one space must not overlap inside a pass.
- Each pass is followed by two zero rows (fact 72), and those rows count:
  the program plus its zero rows must stay within row 0xFFF (fact 771), and
  an init pass's must stay below the decoder's row 0x40.

### The image file (version 1, little-endian)

```
0   "X17I"
4   u16 version = 1
6   u16 section count
8   sections, back to back:
      u8  kind     1 P, 2 X, 3 Y, 4 param, 5 mode
      u8  pass     init passes 0..N-1 in load order, 0xFF the decoder
      u16 0
      u32 addr     P: first row; X, Y, param: first word address
      u32 count    P: rows; X, Y, param: words
      payload      P: count*4 u32, row by row, lanes 0..3 per row
                   X: count u32 (24-bit); Y, param: count u32 (16-bit)
```

A mode section (from `.mode N`) comes first: pass 0xFF, addr = the mode,
count 0, no payload.  Images without `.mode` have none.

Sections come pass by pass: each pass's P, then its data blocks in source
order.  A P section runs from the pass's base row through its two zero
rows; lane l of row r goes to the host's P lane l, row r (handbook §2).

### The debug sidecar (`-g`, JSON)

`{"version":1,"passes":[{"pass","kind","base_row","end_row","bundles":[...],"labels":{name: q}}]}`.
Each bundle: `q`, `len`, `slots` (`[row, lane]` per syllable), `file`,
`line`, `text`, and `macro` (`name`, `file`, `line` of the call) when it
came from a macro body.

## What v1 ships, what waits

**v1**:
- Lex + parse; forms and sub-forms from isa.toml, operands.txt and
  subforms.txt.  Shipped.
- Two passes, a816-style: pass 1 sizes bundles and binds labels (`name:`)
  and `.const NAME = expr`; pass 2 encodes with every symbol known.
  Shipped.
- Constant expressions in immediates: `+ - * / << >> & | ~`, comparisons
  (`== != < <= > >=`, giving 0 or 1), parentheses, symbols; negative
  values store as two's complement.  Shipped.
- Branch targets: `jmp`, `br` and `bcc` take a bare label; the assembler
  emits the form's offset (syllables from the bundle's own q; `bcc` in
  units of 4 and refuses a target that is not a multiple of 4 away; `br`
  only reaches forward).  `#expr` stays the raw field.  Shipped.
- `.assert expr`, checked in pass 2 (`.assert end - start <= 64`).  Shipped.
- `.include "file"` (relative to the including file, cycles refused) and
  a816-style macros: `.macro name(a, b) {` ... `}`, called as `name(x, y)`.
  Parameters substitute as whole identifiers; labels a body defines are
  renamed per expansion; an error inside a macro also names the call.
  `examples/macros.sm` assembles to the same bytes as `labels.sm`.  Shipped.
- `x17 fmt [--check] <file.sm>...`: one canonical layout (labels at 0,
  code at 8, trailing comments at 32, one space after commas, `.const`
  runs aligned).  Only whitespace moves; the file is assembled before and
  after and left alone if its bytes would change.  CI checks `examples/`.
  Shipped.
- Passes, `.org`, `.xdata` / `.ydata` / `.param`, the image file and its
  debug sidecar (`x17 image`; see Placement and layout).  Shipped.
- Still to come: round-trip gate on a whole program; a silicon-safe gate
  (measured forms only) over an image.

`.align` is deliberately absent: a zero syllable is not a no-op (low5 0x00
heads a 7-syllable bundle), so padding needs a measured no-op form first.

**v2 (after the three decoders ask for them)**:
- `.struct` for the sm1 descriptor / PCM ring layout (shared with
  the host firmware).
- `.assert cycles` once isa.toml grows a cycle-cost column.

**Explicitly deferred**:
- Linker, `.import` / `.extern`, modules.  See ARCHITECTURE.md's note on
  leaving room without building it.
- Scheduler, register allocator.  DSL-era.

