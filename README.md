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

## Spike status (2026-11-05)

Comptime parse of `[layout]` section lands the `row_syllables`, `main_row`
and length lookup table.  `zig build run` prints them.  Opcode form tables
(x17 bundles, head + S1 + S2 classes), the assembler's parser, the emulator
and the test harness are all to come.

## Origin

sm1 is reverse-engineered in the RE tree project; see
`RE tree/the spec notes` and `upstream/tools/sm1isa/` for the ISA work
and the Python emulator this rig ports from.

Roadmap items live under RE tree/audio in the shared teach workspace:
`build-the-x17-codec-dev-rig` (epic) and its three children.
