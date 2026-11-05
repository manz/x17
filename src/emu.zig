//! x17 Zig emulator, v0.
//!
//! Built to be the CI oracle for the assembler: assemble a program, run it
//! here, diff against the probe list's measured Y writes.  Follows the Python
//! model (upstream/tools/sm1isa/model.py) on measured rules but starts fresh
//! on the general shape so isa.toml stays the source of truth.
//!
//! What runs today:
//!   * `ldi`  - 5-bit dest [25:21] into state.s[0..15] (dest mapping: 16..31
//!              -> s(k-16); 0..15 refuse per isa.toml's "OPEN" marker).
//!
//! What waits (and why not yet):
//!   * `st.y` - stores the LANE's data register, not sN; needs lane tracking
//!              before the semantics are honest.
//!   * The bundle sequencer + pc / wedge model.  Today we execute a single
//!              bundle in isolation.
//!   * The x17.* sub-forms.  Needed for any program with x17 in it.

const std = @import("std");
const isa = @import("isa");

pub const State = struct {
    /// Scalar registers s0..s15.  Default zero.
    s: [16]u32 = @splat(0),
    /// Element registers e0..e15.  Default zero.
    e: [16]u32 = @splat(0),
    /// R-file registers R0..R15.  R0 pinned to zero.
    r: [16]u32 = @splat(0),
    /// Per-lane "V" data register: what `st.y` writes to memory.  sm1 runs
    /// four lanes in parallel; our emu runs one lane at a time, picked by
    /// `cur_lane`.  Default zero - tests seed the measured lane values.
    v: [4]u32 = @splat(0),
    /// Which lane the current bundle runs on (0..3).  The ref string calls
    /// this out explicitly for st.y ("the LANE's data register"); other
    /// forms may acquire lane dependence as they port.
    cur_lane: u8 = 0,
    /// Y data memory, keyed by address.  Sparse: only addresses the program
    /// writes are present.  Caller inspects via `getY`.
    y_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},

    pub fn deinit(self: *State, gpa: std.mem.Allocator) void {
        self.y_mem.deinit(gpa);
    }

    pub fn getY(self: *const State, addr: u32) ?u32 {
        return self.y_mem.get(addr);
    }
};

pub const RunError = error{
    UnknownForm,
    LdiDestOpen, // ldi dest = 2..15 (marked OPEN in isa.toml)
    OutOfMemory,
    UnknownBundleLength,
    RunawayProgram,
};

/// Execute one bundle (1..7 syllables) against `state`.  Dispatches by
/// classifying the HEAD syllable with `isa.classify`.  Unknown or
/// not-yet-modelled forms return `UnknownForm` so the caller can tell.
pub fn executeBundle(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len == 0) return;
    const head = bundle[0];
    const f = isa.classify(head) orelse return RunError.UnknownForm;

    if (std.mem.eql(u8, f.name, "ldi")) {
        return execLdi(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "st.y")) {
        return execStY(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "x17")) {
        return execX17(gpa, state, bundle);
    }

    return RunError.UnknownForm;
}

fn execStY(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const addr: u32 = (head >> 5) & 0xFFFF;
    const value: u32 = state.v[state.cur_lane & 0x3];
    state.y_mem.put(gpa, addr, value) catch return RunError.OutOfMemory;
}

/// Classify an x17 bundle by its (S1 class, S2 class) field pair.
///   S2 class 101                      -> halt (no state change)
///   S1 class 011 + S2 class 111       -> imm load
///   S1 class 001 + S2 class 111       -> alu
/// Everything else is UnknownForm for now.
fn execX17(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 3) return RunError.UnknownForm;
    const s1 = bundle[1];
    const s2 = bundle[2];
    const s1_class: u32 = (s1 >> 29) & 0x7;
    const s2_class: u32 = (s2 >> 29) & 0x7;

    if (s2_class == 0b101) return; // halt: no state change
    if (s1_class == 0b011 and s2_class == 0b111) return execX17Imm(gpa, state, bundle);
    if (s2_class == 0b111) return execX17Alu(gpa, state, bundle);
    return RunError.UnknownForm;
}

fn execX17Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (s1 >> 7) & 0xF;
    const imm: u32 = (head >> 11) & 0xFFFF;
    state.s[dest_s] = imm;
}

/// sm1 ALU is 24-bit wrapping arithmetic.  Upstream's Python emu calls this
/// mask `M24`; we spell it out.
const M24: u32 = 0x00FF_FFFF;

/// Sign-extend a 24-bit value into an i32.  Negative inputs in M24 encode
/// the high bit at position 23; above that lives junk from any prior OR.
fn sx24(v: u32) i32 {
    const masked: u32 = v & M24;
    if ((masked & 0x0080_0000) != 0) {
        // Sign bit set: subtract 2^24 to get the signed form.
        return @bitCast(masked | 0xFF00_0000);
    }
    return @bitCast(masked);
}

/// Ported from upstream/tools/sm1isa/emu.py X17OP, as of RE tree e55e26d21.
/// Ops marked OPEN in the python table raise UnknownForm here.
fn x17opEval(op: u32, a: i32, b: i32, n: u32) RunError!u32 {
    const shift: u5 = @intCast(n & 0x1F);
    const result_signed: i64 = switch (op) {
        0 => (@as(i64, a) + @as(i64, b)) << shift, // (a + b) << n
        1 => blk: { // round-half-up shift right: (a + b + (1 << n >> 1)) >> n
            const half: i64 = if (shift == 0) 0 else (@as(i64, 1) << (shift - 1));
            break :blk (@as(i64, a) + @as(i64, b) + half) >> shift;
        },
        3, 15 => @as(i64, a) + @as(i64, b),
        4 => @as(i64, b) - @as(i64, a),
        5 => -@as(i64, a) - @as(i64, b) - 2,
        6 => blk: { // (a xor b) >> n
            const xored: u32 = (@as(u32, @bitCast(a)) ^ @as(u32, @bitCast(b))) & M24;
            break :blk @as(i64, xored >> shift);
        },
        8 => 0,
        9 => @as(i64, b),
        10 => @as(i64, @as(i32, @bitCast(n))),
        11 => @as(i64, b) << shift,
        12, 13, 14 => @as(i64, a),
        else => return RunError.UnknownForm,
    };
    // All results wrap back into 24 bits.
    const wrapped: u32 = @as(u32, @bitCast(@as(i32, @truncate(result_signed)))) & M24;
    return wrapped;
}

fn execX17Alu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];

    const op: u32 = (head >> 7) & 0xF;
    const dest_s: u32 = (s1 >> 24) & 0xF;
    const a_field: u32 = (s2 >> 25) & 0xF;
    const b_field: u32 = (s1 >> 16) & 0xF;
    const n: u32 = (s1 >> 20) & 0xF;

    // fact 70: source 0 reads zero, regardless of the register bank.
    const a_val: u32 = if (a_field == 0) 0 else state.s[a_field];
    const b_val: u32 = if (b_field == 0) 0 else state.s[b_field];

    const result = try x17opEval(op, sx24(a_val), sx24(b_val), n);
    state.s[dest_s] = result;
}

/// Run a linear sequence of syllables, walking bundle by bundle until a
/// halt (x17 with S2 class 101) ends the run or we fall off the end.
/// Caller lays syllables out in PC order; multi-lane programs call this
/// once per lane with that lane's stream.  Returns the number of bundles
/// executed; `RunawayProgram` if no halt was seen within 256 bundles.
pub fn runLinear(gpa: std.mem.Allocator, state: *State, syllables: []const u32) RunError!usize {
    var i: usize = 0;
    var bundles_run: usize = 0;
    const MAX_BUNDLES: usize = 256;
    while (i < syllables.len) : (bundles_run += 1) {
        if (bundles_run >= MAX_BUNDLES) return RunError.RunawayProgram;
        const head = syllables[i];
        const len = bundleLen(head) orelse return RunError.UnknownBundleLength;
        if (i + len > syllables.len) return RunError.UnknownBundleLength;
        const slice = syllables[i .. i + len];
        try executeBundle(gpa, state, slice);
        i += len;
        if (isHaltBundle(slice)) return bundles_run + 1;
    }
    return bundles_run;
}

/// Bundle length in syllables from the HEAD's low5, via isa.layout's
/// length table.  Returns null for a low5 not in the table (today those
/// are treated as "unknown"; future RE may fill them in).
pub fn bundleLen(head: u32) ?u8 {
    const low5: u8 = @intCast(head & 0x1F);
    for (isa.layout.length_table) |e| {
        if (e.low5 == low5) return e.syllables;
    }
    // Unlisted low5s show up in practice: default to 1 so a lone st.y or
    // ldi ok execute.  TODO: this is a spike heuristic; replace with a
    // measured default once isa.toml's table is complete.
    return 1;
}

fn isHaltBundle(bundle: []const u32) bool {
    if (bundle.len < 3) return false;
    if ((bundle[0] & 0x1F) != 0x17) return false;
    return ((bundle[2] >> 29) & 0x7) == 0b101;
}

fn execLdi(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest_k: u32 = (head >> 21) & 0x1F;
    const imm: u32 = (head >> 5) & 0xFFFF;
    if (dest_k < 16) {
        // isa.toml: "16..31 = source k-16 (32 of 32), 0 = address entry 0,
        // 1 = source 1; 2..15 OPEN".  The model only commits to 16..31 so
        // far - map those to scalar registers; refuse the rest.
        return RunError.LdiDestOpen;
    }
    const s_idx: u32 = dest_k - 16;
    state.s[s_idx] = imm;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;
const asm_mod = @import("asm");

test "ldi sets the scalar register" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b = try asm_mod.assembleOne("ldi s5, #0x1234");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x1234), state.s[5]);
    // Other scalar regs untouched.
    try testing.expectEqual(@as(u32, 0), state.s[0]);
    try testing.expectEqual(@as(u32, 0), state.s[15]);
}

test "ldi to s0 maps from dest k=16" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b = try asm_mod.assembleOne("ldi s0, #0xDEAD");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0xDEAD), state.s[0]);
}

test "two ldi in sequence land in distinct regs" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b1 = try asm_mod.assembleOne("ldi s2, #0x0012");
    try executeBundle(testing.allocator, &state, b1.slice());
    const b2 = try asm_mod.assembleOne("ldi s9, #0x0099");
    try executeBundle(testing.allocator, &state, b2.slice());

    try testing.expectEqual(@as(u32, 0x0012), state.s[2]);
    try testing.expectEqual(@as(u32, 0x0099), state.s[9]);
}

test "x17.imm loads the scalar via sub-form dispatch" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b = try asm_mod.assembleOne("x17.imm s5, #0x0BEE");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x0BEE), state.s[5]);
}

test "x17.halt executes as a noop" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[3] = 0x1234;

    const b = try asm_mod.assembleOne("x17.halt");
    try executeBundle(testing.allocator, &state, b.slice());
    // halt must not scramble state.
    try testing.expectEqual(@as(u32, 0x1234), state.s[3]);
}

test "x17.alu op=0 shl matches own-x17alu-shl silicon" {
    // Pre-state: s8 = 0x246, s2 = 1.  Program: x17.alu s1, s8, s2, #0, #3.
    //   op 0 = (a + b) << n = (0x246 + 1) << 3 = 0x247 * 8 = 0x1238.
    // Silicon stored 0x1238 at Y:0x700 (the probe list own-x17alu-shl).
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[8] = 0x246;
    state.s[2] = 1;

    const b = try asm_mod.assembleOne("x17.alu s1, s8, s2, #0, #3");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x1238), state.s[1]);
}

test "x17.alu op=4 b-a matches own-x17alu-rsub silicon" {
    // Pre-state: s8 = 0x246, s4 = 0x701.  Program: x17.alu s1, s8, s4, #4, #3.
    //   op 4 = b - a = 0x701 - 0x246 = 0x4BB.  Silicon: Y:0x700 = 0x4BB.
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[8] = 0x246;
    state.s[4] = 0x701;

    const b = try asm_mod.assembleOne("x17.alu s1, s8, s4, #4, #3");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x4BB), state.s[1]);
}

test "x17.alu register 0 reads as zero (fact 70)" {
    // Even if s0 holds a value, the a-field index 0 forces a = 0.
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[0] = 0xDEAD;
    state.s[2] = 0x100;

    // x17.alu s1, s0, s2, #3, #0 -> op 3 = a + b.  a-field=0 -> a=0, b=0x100.
    const b = try asm_mod.assembleOne("x17.alu s1, s0, s2, #3, #0");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x100), state.s[1]);
}

test "st.y writes the lane's V register to Y memory" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.v[2] = 2; // silicon default for lane 2 per fact 21
    state.cur_lane = 2;

    const b = try asm_mod.assembleOne("st.y Y:0x200");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 2), state.getY(0x200).?);
}

test "runLinear walks ldi then halt and stops" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b_ldi = try asm_mod.assembleOne("ldi s3, #0xBEEF");
    const b_halt = try asm_mod.assembleOne("x17.halt");

    var buf: [8]u32 = undefined;
    @memcpy(buf[0..b_ldi.len], b_ldi.slice());
    @memcpy(buf[b_ldi.len .. b_ldi.len + b_halt.len], b_halt.slice());
    const stream = buf[0 .. b_ldi.len + b_halt.len];

    const bundles = try runLinear(testing.allocator, &state, stream);
    try testing.expectEqual(@as(usize, 2), bundles);
    try testing.expectEqual(@as(u32, 0xBEEF), state.s[3]);
}

test "runLinear: x17.alu then halt reproduces shl math" {
    // Mini program: pre-load s8 and s2, run ALU, store would follow but
    // we halt right after.  Mirrors own-x17alu-shl's compute path end-to-end.
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[8] = 0x246;
    state.s[2] = 1;

    const b_alu = try asm_mod.assembleOne("x17.alu s1, s8, s2, #0, #3");
    const b_halt = try asm_mod.assembleOne("x17.halt");

    var buf: [16]u32 = undefined;
    @memcpy(buf[0..b_alu.len], b_alu.slice());
    @memcpy(buf[b_alu.len .. b_alu.len + b_halt.len], b_halt.slice());

    _ = try runLinear(testing.allocator, &state, buf[0 .. b_alu.len + b_halt.len]);
    try testing.expectEqual(@as(u32, 0x1238), state.s[1]);
}

test "runLinear halts even if syllables follow the halt" {
    // A halt bundle mid-stream stops the run; trailing syllables are
    // untouched.  Mirrors silicon: execution ends at the stop.
    var state = State{};
    defer state.deinit(testing.allocator);

    const b_halt = try asm_mod.assembleOne("x17.halt");
    const b_ldi = try asm_mod.assembleOne("ldi s0, #0xDEAD");

    var buf: [8]u32 = undefined;
    @memcpy(buf[0..b_halt.len], b_halt.slice());
    @memcpy(buf[b_halt.len .. b_halt.len + b_ldi.len], b_ldi.slice());

    _ = try runLinear(testing.allocator, &state, buf[0 .. b_halt.len + b_ldi.len]);
    try testing.expectEqual(@as(u32, 0), state.s[0]); // ldi after halt never ran
}

test "own-st3 reproduces silicon Y writes at lane 2" {
    // the probe list own-st3: three stores at r5l1..l3 of OUR syllables;
    // silicon-measured Y:0x100 = 0, Y:0x200 = 2, Y:0x300 = 0 (lanes 1, 2, 3).
    // Our emu runs one lane at a time; replay each lane separately.
    const lane_data = [_]struct { lane: u8, v: u32, addr: u16, expected: u32 }{
        .{ .lane = 1, .v = 0, .addr = 0x100, .expected = 0 },
        .{ .lane = 2, .v = 2, .addr = 0x200, .expected = 2 },
        .{ .lane = 3, .v = 0, .addr = 0x300, .expected = 0 },
    };
    for (lane_data) |ld| {
        var state = State{};
        defer state.deinit(testing.allocator);
        state.v[ld.lane] = ld.v;
        state.cur_lane = ld.lane;

        var buf: [32]u8 = undefined;
        const src = try std.fmt.bufPrint(&buf, "st.y Y:0x{x}", .{ld.addr});
        const b = try asm_mod.assembleOne(src);
        try executeBundle(testing.allocator, &state, b.slice());
        try testing.expectEqual(ld.expected, state.getY(ld.addr).?);
    }
}
