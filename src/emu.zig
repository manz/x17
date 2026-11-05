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

    return RunError.UnknownForm;
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

test "unknown form returns UnknownForm" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b = try asm_mod.assembleOne("st.y Y:1");
    try testing.expectError(RunError.UnknownForm, executeBundle(testing.allocator, &state, b.slice()));
}
