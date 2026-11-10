# x17

Dev rig for **Samsung sm1**'s DSP, the hardware audio decoder in the iPod
Classic 6G (S5L8702) and friends.  Three Zig tools sharing `data/isa.toml`
as the one source of truth:

- **Assembler**: reads x17 source (`.s`), emits a `.bin` the sm1 loader
  takes.  isa.toml is embedded at compile time; a malformed table fails the
  build, not some codec's first run.
- **Emulator**: CPU-side bit-exact oracle.  Ported from a Python ISA model
  to Zig for CI-scale throughput.
- **Bit-exact test harness**: pytest-shaped; feeds an input bitstream + our
  `.s` through assembler + emulator and diffs the PCM against
  libav/ffmpeg's reference decode.  Zero diff is the gate.

End users (codec authors) write `.s` files, commit them,
CI runs the harness.  Nobody but codec authors touches this repo.

## Using in CI

Downstream codec repos put `x17` on PATH with one composite action step:

```yaml
runs-on: ubuntu-26.04-arm
steps:
  - uses: manz/x17@v0.1.0
```

On an ARM64 Linux runner pinned to a release tag, the action downloads that
release's static binary. On any other runner, or at a branch ref, it builds
x17 from source with Zig (`zig-version`, default 0.17.0) and caches the
binary per OS, arch, ref and Zig version.

Pushing a `v*` tag runs `.github/workflows/release.yml`, which builds the
stripped aarch64-linux binary, smoke-tests it and attaches it to the release.

## Layout

    build.zig / build.zig.zon
    src/
      main.zig              CLI stub
      isa.zig               isa.toml -> comptime-typed layout table
    data/
      isa.toml              single source of truth

## Status (2026-11-06)

- `x17 asm`: two-pass assembler, labels, `.const`, constant expressions,
  branch targets, `.assert`, `.include`, macros (ASSEMBLY.md).
  `examples/labels.sm`, `examples/macros.sm`.
- `x17 image`: init passes, the decoder at row 0x40, per-pass X/Y data and
  the parameter block, written as one image file plus a JSON debug sidecar
  (ASSEMBLY.md, Placement and layout).
- `x17 run`: the emulator over every silicon-measured form; 298 own probe
  programs are gated byte-exact in `src/silicon_gates.zig`.
- `x17 play IMAGE.x17i -p PACKETS -o OUT.pcm`: runs an image as the host does,
  init passes at q0 then each packet through `src/host.zig` from q256, and
  writes the drained s16 PCM. Packets are a u32 LE length plus bytes, repeated.
  An unmodelled form stops the run with its packet, pc and syllables;
  `--trap-list` skips them as nops instead and ranks them by hits.
  `--trace N` prints the first N bundles run, each with the registers and
  predicates it changed and its X / Y reads and writes.
- `x17 disasm`: decodes syllables back to source.
- `x17 fmt`: canonical layout for .sm sources; `--check` in CI.
- `src/host.zig`: the host around a codec program, one packet per run:
  parameter block, packet staging, the output ring drain and its owned-buffer
  rule.  Interface facts from sosumi's specs/questions.md; packet reads and
  clobbered regions are still named stubs.
- `x17 run --profile N`: the hottest bundles with their source lines, from the
  emulator's per-syllable profile (`State.profile`). Rate: one bundle of any form takes about 37.4 ns on silicon (26.7 M/s;
  the sm1 clock itself is unread, so "one bundle per cycle" is inferred).

## Mnemonics

Every opcode and sub-form name in this repo is **reverse-engineered**,
not vendored.  Samsung ships no datasheet for sm1; we coined the names
while bit-sweeping silicon with probe programs of our own.

Two patterns:

- **Behavior-derived**: `ldi` (load immediate), `rli` (register-load
  imm: writes both R and s), `st.y` (store to Y memory), `alu` (ALU op),
  `rop` (R-file op), `yagu` (Y address-gen unit), `plog`/`ptst`
  (predicate log / test), and for sub-forms: `.imm`, `.ld`, `.alu05`,
  `.mve`, `.rsub`, `.mul`, `.aagu`, `.xpm`, `.xld_a10`, `.halt`,
  `.cont`.
- **Opcode-derived**: `x17` and `x19` from their low5 values (0x17,
  0x19), because each one is a family of VLIWs rather than one op.

Every form row in `data/isa.toml` cites the fact number (`fact N`) of the
silicon measurement it was recovered from.  If a Samsung datasheet ever surfaces, mnemonics are
fair game to rename.

## Origin

The ISA is measured on silicon with probe programs of our own; the numbered
facts behind each rule are kept in a private lab notebook.  The emulator
started as a port of a Python model of the same facts.
