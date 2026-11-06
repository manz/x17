# x17

Dev rig for **Samsung sm1**'s DSP, the hardware audio decoder in the iPod
Classic 6G (S5L8702) and friends.  Three Zig tools sharing `data/isa.toml`
as the one source of truth:

- **Assembler**: reads x17 source (`.s`), emits a `.bin` the sm1 loader
  takes.  isa.toml is embedded at compile time; a malformed table fails the
  build, not some codec's first run.
- **Emulator**: CPU-side bit-exact oracle.  Ported from the Python RE
  scratchpad (`RE tree/upstream/tools/sm1isa/model.py`) to Zig for CI-scale
  throughput.
- **Bit-exact test harness**: pytest-shaped; feeds an input bitstream + our
  `.s` through assembler + emulator and diffs the PCM against
  libav/ffmpeg's reference decode.  Zero diff is the gate.

End users (codec authors on the RE tree side) write `.s` files, commit them,
CI runs the harness.  Nobody but codec authors touches this repo.

## Using in CI

Downstream codec repos put `x17` on PATH with one composite action step:

```yaml
- uses: manz/x17@master
  # Pin to a tag once releases are cut: manz/x17@v0.3.0
```

The action builds x17 from source with the matching Zig toolchain and
caches the binary across runs (key: OS + action ref + Zig version).
Override Zig with `zig-version: '0.17.0'` if your consumer needs it.

## Layout

    build.zig / build.zig.zon
    src/
      main.zig              CLI stub
      isa.zig               isa.toml -> comptime-typed layout table
    data/
      isa.toml              single source of truth (synced from RE tree)

## Status (2026-11-06)

- `x17 asm`: two-pass assembler, labels, `.const`, constant expressions,
  branch targets, `.assert`, `.include`, macros (ASSEMBLY.md).
  `examples/labels.sm`, `examples/macros.sm`.
- `x17 run`: the emulator over every silicon-measured form; 283 own probe
  programs are gated byte-exact in `src/silicon_gates.zig`.
- `x17 disasm`: decodes syllables back to source.
- `x17 fmt`: canonical layout for .sm sources; `--check` in CI.
- `src/host.zig`: the host around a codec program, one packet per run:
  parameter block, packet staging, the output ring drain and its owned-buffer
  rule.  Interface facts from sosumi's specs/questions.md; packet reads and
  clobbered regions are still named stubs.
- To come: cycle costs, the bit-exact harness.

## Mnemonics

Every opcode and sub-form name in this repo is **reverse-engineered**,
not vendored.  Samsung ships no datasheet for sm1; we coined the names
while bit-sweeping silicon with the harness at
`RE tree/upstream/tools/sm1isa/`.

Two patterns:

- **Behavior-derived**: `ldi` (load immediate), `rli` (register-load
  imm: writes both R and s), `st.y` (store to Y memory), `alu` (ALU op),
  `rop` (R-file op), `yagu` (Y address-gen unit), `plog`/`ptst`
  (predicate log / test), and for sub-forms: `.imm`, `.ld`, `.alu05`,
  `.mve`, `.rsub`, `.mul`, `.aagu`, `.xpm`, `.xld_a10`, `.halt`,
  `.cont`.
- **Opcode-derived**: `x17` and `x19` from their low5 values (0x17,
  0x19), because each one is a family of VLIWs rather than one op.

Every form row in `data/isa.toml` cites the `fact` pass it was
recovered in.  If a Samsung datasheet ever surfaces, mnemonics are
fair game to rename.

## Origin

sm1 is reverse-engineered in the RE tree project; see
`RE tree/the spec notes` and `upstream/tools/sm1isa/` for the ISA work
and the Python emulator this rig ports from.

Roadmap items live under RE tree/audio in the shared teach workspace:
`build-the-x17-codec-dev-rig` (epic) and its three children.
