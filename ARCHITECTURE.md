# x17 architecture

Design sketch for the Samsung sm1 DSP dev rig.  Written to get the shape
right before cutting code.  Inspired by a816's layered idiom; adapted to a
DSP ISA with bundle instructions, cycle budgets, and no banks.

## Goals (and non-goals)

**Goals**:
- Assemble x17 source (`.s`) into a `.bin` the sm1 loader eats.
- Run that `.bin` through a bit-exact emulator for PCM output; diff it
  against a libav/ffmpeg reference decode.
- Catch cycle-budget violations at ASSEMBLE TIME, not as silence on silicon.
- Share `isa.toml` as the single source of truth across assembler + emulator
  + the Python ISA model.

**Non-goals (v1)**:
- No higher-level DSP language yet (that is the future epic
  `design-a-higher-level-sm1-dsp-language`; this rig is its backend when it
  arrives).
- No multi-file linker, no modules, no `.import` as a SHIPPING feature.
  One `.s` per decoder is enough until a second decoder wants a shared
  table.  But the pipeline keeps the SHAPE that admits one (deferred
  symbol resolution, late placement) so adding it later is a change, not
  a rewrite.  This is a direct lesson from a816, which did not need
  linking at the start and paid for the retrofit later.
- No silicon runner here.  Silicon validation lives in the host firmware tree.

## The data flow (end to end)

    .s source          [one file per program]
        |
        v
    scanner  ----------+
        |              |
        v              |  (diagnostics route through
    parser             |   one error-catalog: line, col,
        |              |   code, hint - a816 shape)
        v              |
    AST                |
        |              |
        v              |
    scope resolve  ----+
        |
        v
    expression fold    [consts, offsets, immediate fields]
        |
        v
    bundle emit        [head + S1 + S2 syllables]
        |
        v
    cycle check        [.assert budgets against isa.toml timing]
        |
        v
    .bin               [word stream, row/lane layout per isa.toml]
        |
        +----> loader on silicon
        |
        +----> Zig emulator ----> PCM out
                                      |
                                      v
                               diff vs libav/ffmpeg reference
                                      |
                                      v
                                 zero = green

## Module layout (src/)

```
src/
  main.zig          CLI entry, subcommand dispatch (asm, emu, dump-isa)
  isa.zig           isa.toml -> comptime-typed Layout + opcode-form tables
                    (LANDED: layout half.  TODO: bundle encoding tables.)

  lex/              tokens, source positions, error spans
    token.zig
    scanner.zig

  parse/            AST types + state machine, a816-flavoured
    ast.zig
    parser.zig

  scope/            name resolution, constant env, .include expansion (v2)
    scope.zig
    const_fold.zig

  emit/             AST -> bundle stream
    bundle.zig      head + S1 + S2 encoding, drives off isa.zig tables
    layout.zig      place bundles into rows/lanes per isa.toml [layout]
    cycle.zig       cycle budget accounting, .assert evaluation

  emu/              CPU-side bit-exact oracle (port of model.py)
    state.zig       scalar regs, element regs, AGUs, SRAM, descriptors
    step.zig        one bundle at a time; dispatch from the comptime
                    opcode table in isa.zig
    io.zig          descriptor chain input, PCM output buffer

  cli/              subcommand verbs
    asm.zig         x17 asm <file.s> -> <file.bin>
    emu.zig         x17 emu <file.bin> --input <bitstream> --pcm-out <...>
    dump.zig        x17 dump-isa (what the current comptime table holds)

  errors/           one error catalog, a816 style: code + message + hint
    catalog.zig
```

## Terminology (pinned, so reports and code agree)

| Term       | Meaning                                                                 |
|------------|-------------------------------------------------------------------------|
| syllable   | one 32-bit word; the smallest unit of the instruction stream            |
| bundle     | an instruction made of N syllables (N from `isa.toml` length table)     |
| head       | the first syllable of a bundle; its low5 selects the bundle length     |
| S1 / S2    | the second and third syllables of an x17 bundle                         |
| row / lane | the 2D layout sm1 loads into: lane l of row r at `0x22020000 + l*0x4000 + r*4` |
| program    | one `.s` source; produces one `.bin`                                    |
| PC q       | sm1's instruction counter; `8*(row/2) + 2*lane + row%2` (column-major)  |

## Patterns borrowed from a816 (and the ones we skip)

**Take**:
- **Scope + expression language** at the AST level.  Constants, offsets,
  folded expressions in immediate fields.  a816 does this; we need the
  same.
- **One error catalog** (`errors/catalog.zig`) with stable codes.
  Diagnostics carry file + line + col + code + one-sentence hint.  a816's
  `error_codes.py` pattern.
- **`.assert`** at assemble time.  a816 has it for layout invariants; we
  use it for cycle-budget invariants (`.assert cycles(main_loop) <= 980`).
- **Build cache** when the assembler gets slow enough to notice.  Not v1;
  earn it.
- **Linter / formatter** idiom (`fluff` in a816).  A later tool; x17 code
  will have its own cliches (AGU setup, bundle alignment) worth catching.
- **LSP** idiom.  Also later; wait until there is a second author.

**Skip (until earned)**:
- **Banks, HiROM/LoROM maps**.  x17 has a flat program memory addressed
  as row/lane; no bank concept.
- **Register-width tracking (m/x flags)**.  Not applicable.
- **Multi-file linker + `.import` / `.extern`** as a SHIPPING feature.
  One file per program is enough for ALAC and MP3.  BUT a816 started the
  same way and grew it in - so leave room for it in the AST / scope /
  emit layers even in v1: symbols resolve at the END of the pipeline from
  a flat env (not during emit), expressions can refer to not-yet-defined
  labels, and every AST node carries a source span so cross-file
  diagnostics drop in without a rewrite.  The LINKER itself waits; the
  room for it does not.
- **Boards + `[map.N]`**.  sm1 has ONE memory map, defined in `isa.toml`;
  no per-board variance.
- **BPS diff**.  Unrelated; a816-specific SNES patch tooling.

## isa.toml as the source of truth

Both halves (assembler + emulator) `@embedFile` it and parse at comptime.
Any malformed row fails the build.  The Python RE emulator reads the same
file at runtime; a CI check cross-decodes a known program through both
and diffs - the first divergence is the ISA table disagreeing with itself.

When the ISA is extended (new opcode form discovered during RE), the
change lands in `isa.toml` once and both paths rebuild / re-read.  No
hand-written encoding tables duplicated across tools.

## Future seats

- **x17-DSL compiler** (future epic): slots in above `emit/` as a frontend
  layer that lowers signal-flow / schedule-annotated source to AST.  Reuses
  `scope/`, `emit/`, `cycle/` verbatim.  This is why assembler is Zig: so
  the compiler can share its backend without a Python-to-Zig port.
- **Silicon test runner**: subprocess target from the test harness that
  pushes the `.bin` to a connected iPod and compares.  Lives in the
  host firmware tree; x17 just emits the `.bin`.

## Open questions (resolved when the spike grows into them)

- **Scheduler**: x17 bundles have internal parallelism (head + S1 + S2 run
  together, dependencies matter).  v1 assembler trusts the author.  A real
  scheduler is DSL-era work.
- **Register allocation**: same answer - author-managed in v1, DSL-era
  for automated AGU/element/scalar allocation.
- **Pipelining**: unknown if x17 has delayed slots or pipeline hazards
  the assembler must respect.  Measurement item, blocks real assembler
  work; park for now.
- **Cycle model**: isa.toml today has lengths but no per-opcode cycle
  cost.  `cycle.zig` + `.assert` needs that column; shape it with the
  next isa.toml extension.
