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
    /// Address registers A0..A15: feed X/Y load+store addresses.  Default
    /// zero; seeded via tests or set-up forms that write them.
    a: [16]u32 = @splat(0),
    /// Predicate bits p0..p15 (1-bit each; stored as u8 for ergonomics).
    /// Written by ptst / plog; consumed by psel / jmp / bcc.  Survives
    /// across bundles per fact.
    p: [16]u8 = @splat(0),
    /// Y data memory, keyed by address.  Sparse: only addresses the program
    /// writes are present.  Caller inspects via `getY`.
    y_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},
    /// X data memory, keyed by address.  Separate bank from Y per facts.
    x_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},

    pub fn deinit(self: *State, gpa: std.mem.Allocator) void {
        self.y_mem.deinit(gpa);
        self.x_mem.deinit(gpa);
    }

    pub fn getY(self: *const State, addr: u32) ?u32 {
        return self.y_mem.get(addr);
    }

    pub fn getX(self: *const State, addr: u32) ?u32 {
        return self.x_mem.get(addr);
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

    // Low5 0x10 bundles share value/mask across bcc/ystm/agu10r; isa.classify
    // returns the first match.  Discriminate by the S class before falling
    // through to the generic classifier.
    const low5: u32 = head & 0x1F;
    if (low5 == 0x10 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b101) return execAgu10r(gpa, state, bundle);
        if (s_class == 0b001) return execLdi2(gpa, state, bundle);
        if (s_class == 0b011) {
            const s7: u32 = (bundle[1] >> 7) & 1;
            if (s7 == 1) return execXld10(gpa, state, bundle);
            return execPair(gpa, state, bundle);
        }
        if (s_class == 0b111) return execYld10(gpa, state, bundle);
    }
    if (low5 == 0x14 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b001) return execXst(gpa, state, bundle);
        if (s_class == 0b000) return execYtst(gpa, state, bundle);
        if (s_class == 0b101) return execYrm(gpa, state, bundle);
        if (s_class == 0b010) return execP010(gpa, state, bundle);
    }
    if (low5 == 0x18 and bundle.len >= 4) {
        const w3_class: u32 = (bundle[3] >> 29) & 0x7;
        if (w3_class == 0b110) return execX18Imm(gpa, state, bundle);
    }
    if (low5 == 0x02 and bundle.len >= 6) {
        const w4_mode: u32 = (bundle[4] >> 11) & 0x7;
        if (w4_mode == 1) return execX02Imm(gpa, state, bundle);
        if (w4_mode == 5) return execX02Xrd(gpa, state, bundle);
    }
    if (low5 == 0x0E and bundle.len >= 2) {
        const hb: u32 = (head >> 30) & 0x3;
        if (hb == 0) return execPtest(gpa, state, bundle);
        if (hb == 1) return execPsel(gpa, state, bundle);
        return execLd0e(gpa, state, bundle);
    }

    const f = isa.classify(head) orelse return RunError.UnknownForm;

    if (std.mem.eql(u8, f.name, "ldi")) {
        return execLdi(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "rli")) {
        return execRli(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "rop")) {
        return execRop(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "lde")) {
        return execLde(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "ld0e")) {
        return execLd0e(gpa, state, bundle);
    }
    if (std.mem.eql(u8, f.name, "stx")) {
        return execStx(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "ldy")) {
        return execLdy(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "ptst")) {
        return execPtst(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "plog")) {
        return execPlog(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "st.y")) {
        return execStY(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "alu")) {
        return execAlu(gpa, state, head);
    }
    if (std.mem.eql(u8, f.name, "x17")) {
        return execX17(gpa, state, bundle);
    }
    if (std.mem.eql(u8, f.name, "x19")) {
        return execX19(gpa, state, bundle);
    }
    // Low5 0x18 isn't a classify-registered top-level form; handle by low5.
    if (std.mem.eql(u8, f.name, "alu11")) {
        return execAlu11(gpa, state, bundle);
    }
    if (std.mem.eql(u8, f.name, "agu10r")) {
        return execAgu10r(gpa, state, bundle);
    }

    return RunError.UnknownForm;
}

fn execP010(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const s1 = bundle[1];
    const p_dest: u32 = (s1 >> 4) & 0xF;
    const cond: u32 = (s1 >> 8) & 0xF;
    const v: u8 = switch (cond) {
        9 => 1, // measured as constant-true on own-p010-true
        else => return RunError.UnknownForm,
    };
    if (p_dest != 0) state.p[p_dest] = v;
}

fn execYrm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    const dest_r: u32 = s1 & 0xF;
    const src_e: u32 = (s1 >> 21) & 0xF;
    if (dest_s != 0) state.s[dest_s] = imm;
    if (dest_r != 0) state.r[dest_r] = state.e[src_e];
}

fn execX02Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w4 = bundle[4];
    const dest_s: u32 = (w4 >> 3) & 0xF;
    const src_a: u32 = (w4 >> 17) & 0xF;
    const addr: u32 = state.a[src_a];
    const v: u32 = state.getX(addr) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX02Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const head = bundle[0];
    const w4 = bundle[4];
    const dest_s: u32 = (w4 >> 3) & 0xF;
    const imm: u32 = (head >> 11) & 0xFFFF;
    if (dest_s != 0) state.s[dest_s] = imm;
}

fn execX18Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 4) return RunError.UnknownForm;
    const head = bundle[0];
    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    if (dest_s != 0) state.s[dest_s] = imm;
}

fn execYtst(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const s1 = bundle[1];
    const p_dest: u32 = (s1 >> 11) & 0xF;
    const src_e: u32 = (s1 >> 21) & 0xF;
    const addr: u32 = state.e[src_e];
    const yv: u32 = state.getY(addr) orelse 0;
    const v: u8 = if ((yv & 1) == 0) 1 else 0;
    if (p_dest != 0) state.p[p_dest] = v;
}

fn execXst(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const a_half: u32 = (head >> 30) & 0x3;
    const a_low: u32 = s1 & 0x7;
    const src_s: u32 = (s1 >> 4) & 0xF;
    const a_idx: u32 = (4 * a_low + a_half) & 0xF;
    const addr: u32 = state.a[a_idx];
    const value: u32 = state.s[src_s] & M24;
    try state.x_mem.put(gpa, addr, value);
}

fn execXld10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const a_half: u32 = (head >> 30) & 0x3;
    const dest_s2: u32 = (head >> 21) & 0xF;
    const imm16: u32 = (head >> 5) & 0xFFFF;
    const dest_s_x: u32 = (s1 >> 7) & 0xF;
    const a_low: u32 = s1 & 0x7;
    const a_idx: u32 = (4 * a_low + a_half) & 0xF;
    const addr: u32 = state.a[a_idx];
    const v: u32 = state.getX(addr) orelse 0;
    if (dest_s_x != 0) state.s[dest_s_x] = v;
    if (dest_s2 != 0) state.s[dest_s2] = imm16;
}

fn execYld10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const src_e: u32 = (s1 >> 25) & 0xF;
    const addr: u32 = state.e[src_e];
    const v = state.getY(addr) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execPair(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const imm: u32 = (head >> 5) & 0xFFFF;
    const e_idx: u32 = (s1 >> 3) & 0x3;
    state.e[e_idx] = imm;
}

fn execLdi2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const dest_s: u32 = (head >> 21) & 0x1F;
    const imm: u32 = (head >> 5) & 0xFFFF;
    if (dest_s == 0 or dest_s >= 16) return;
    state.s[dest_s] = imm;
}

fn execAgu10r(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];

    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    const y_base: u32 = (head >> 27) & 0x3;
    const dest_r: u32 = (s1 >> 21) & 0xF;
    const x_idx: u32 = (s1 >> 25) & 0xF;
    const fn_op: u32 = (s1 >> 1) & 0xF;

    if (dest_s != 0) state.s[dest_s] = imm;

    const x_val: u32 = if (x_idx == 0) 0 else state.e[x_idx];
    const y_idx: u32 = (4 * y_base) & 0xF;
    const y_val: u32 = if (y_idx == 0) 0 else state.r[y_idx];
    const v = try agu10Eval(fn_op, x_val, y_val);
    if (dest_r != 0) state.r[dest_r] = v;
}

fn execAlu11(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];

    // ALU half.  Zero dest_s means "no write" per the measured programs.
    const alu_dest: u32 = (head >> 15) & 0xF;
    if (alu_dest != 0) {
        const a_idx: u32 = (head >> 11) & 0xF;
        const b_idx: u32 = (head >> 26) & 0xF;
        const code: u32 = (head >> 5) & 0x3F;
        const a_val: u32 = if (a_idx == 0) 0 else state.s[a_idx];
        const b_val: u32 = if (b_idx == 0) 0 else state.s[b_idx];
        const op = ALU05[code];
        state.s[alu_dest] = alu05Eval(op, sx24(a_val), sx24(b_val));
    }

    // AGU half.  dest_R == 0 AND dest_e == 0 means noop.
    const agu_dest_r: u32 = (s1 >> 25) & 0xF;
    const agu_dest_e: u32 = (s1 >> 9) & 0xF;
    if (agu_dest_r != 0 or agu_dest_e != 0) {
        const fn_op: u32 = (s1 >> 17) & 0xF;
        const x_idx: u32 = (s1 >> 21) & 0xF;
        const y_idx: u32 = (s1 >> 13) & 0xF;
        const x_val: u32 = if (x_idx == 0) 0 else state.e[x_idx];
        const y_val: u32 = if (y_idx == 0) 0 else state.r[y_idx];
        const v = try agu10Eval(fn_op, x_val, y_val);
        if (agu_dest_r != 0) state.r[agu_dest_r] = v;
        if (agu_dest_e != 0) state.e[agu_dest_e] = v;
    }
}

/// 4-syllable VLIW.  Today only the (000, 000, 111) triple is modelled
/// (x19.ld); more sub-forms land as more silicon gates arrive.
fn execX19(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 4) return RunError.UnknownForm;
    const w1_class: u32 = (bundle[1] >> 29) & 0x7;
    const w2_class: u32 = (bundle[2] >> 29) & 0x7;
    const w3_class: u32 = (bundle[3] >> 29) & 0x7;
    if (w1_class == 0 and w2_class == 0 and w3_class == 7) {
        return execX19Ld(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 2) {
        return execX19Aagu(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 1 and w3_class == 6) {
        return execX19Mul(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 1) {
        return execX19Imm(gpa, state, bundle);
    }
    return RunError.UnknownForm;
}

fn execX19Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];

    const head_31_29: u32 = (head >> 29) & 0x7;
    const w1_0: u32 = w1 & 0x1;
    const dest_e: u32 = ((w1_0 << 3) | head_31_29) & 0xF;
    const x_src: u32 = (w3 >> 25) & 0xF;
    const y_src: u32 = (head >> 25) & 0xF;
    const fn_op: u32 = (w1 >> 1) & 0xF;
    const imm: u32 = (w2 >> 15) & 0xFFFF;

    // Only fn_op = 10 (y + i) modelled today.
    if (fn_op != 10) return RunError.UnknownForm;

    const y_val: u32 = if (y_src == 0) 0 else state.r[y_src];
    _ = x_src; // unused for fn_op 10
    state.e[dest_e] = (y_val +% imm) & 0xFFFF;
}

fn execX19Mul(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];

    const dest_e: u32 = (w1 >> 3) & 0xF;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const op: u32 = (w1 >> 11) & 0x7;
    const a_src: u32 = (w2 >> 5) & 0xF;
    const b_src: u32 = (w2 >> 1) & 0xF;

    // Only op = 4 modelled today: low16(a * b).
    if (op != 4) return RunError.UnknownForm;

    const a_val: u32 = if (a_src == 0) 0 else state.s[a_src];
    const b_val: u32 = if (b_src == 0) 0 else state.s[b_src];
    const prod: u32 = (a_val *% b_val) & 0xFFFF;

    state.e[dest_e] = prod;
    if (dest_r != 0) state.r[dest_r] = prod;
}

fn execX19Aagu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w1 = bundle[1];
    const w3 = bundle[3];

    const dest_a: u32 = (w1 >> 3) & 0x1F;
    const src_a: u32 = (w3 >> 15) & 0x1F;
    const fn_op: u32 = (w3 >> 20) & 0x1F;
    const imm: u32 = w3 & 0x7FFF;

    // AGUA table today models the one measured op: fn_op = 2 -> x + 2i.
    const x_val: u32 = state.a[src_a & 0xF];
    const new_val: u32 = switch (fn_op) {
        0 => x_val,
        2 => (x_val +% (2 * imm)) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    state.a[dest_a & 0xF] = new_val;
}

fn execX19Ld(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];

    const dest_s: u32 = (w2 >> 10) & 0xF;
    const dest_e_half: u32 = w1 & 0x7;
    const head31: u32 = (head >> 31) & 0x1;
    const dest_e: u32 = (2 * dest_e_half + head31) & 0xF;
    const y_src_e: u32 = (w3 >> 25) & 0xF;
    const fn_op: u32 = (w1 >> 7) & 0xF;

    // 1) Load from Y memory using the pre-state element register.
    const src_e_val: u32 = if (y_src_e == 0) 0 else state.e[y_src_e];
    const loaded: u32 = state.getY(src_e_val) orelse 0;
    state.s[dest_s] = loaded;

    // 2) AGU update: feed the same source register as x and y, then write back.
    const new_e = try agu10Eval(fn_op, src_e_val, src_e_val);
    state.e[dest_e] = new_e;

    // 3) Store this lane's V to Y:[new_e].
    const v = state.v[state.cur_lane & 0x3];
    state.y_mem.put(gpa, new_e, v) catch return RunError.OutOfMemory;
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
    if (s1_class == 0b001 and s2_class == 0b010 and (s1 >> 28) & 1 == 1) return execX17Agu(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b100) return execX17E(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b011) return execX17Ld(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b110) return execX17Alu05(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b010) return execX17Mve(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b001) return execX17Rsub(gpa, state, bundle);
    if (s1_class == 0b100 and s2_class == 0b110) return execX17Xpm(gpa, state, bundle);
    if (s1_class == 0b010 and s2_class == 0b110 and ((bundle[0] >> 27) & 1) == 0) return execX17XldA10(gpa, state, bundle);
    return RunError.UnknownForm;
}

fn execX17XldA10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s2 = bundle[2];

    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a_half: u32 = s2 & 0x3;
    // A-offset pinned at 2 (S1 class = 010): a_idx = 8*a_half + 2.
    const a_idx: u32 = (8 * a_half + 2) & 0xF;

    const addr: u32 = state.a[a_idx];
    const loaded: u32 = state.getX(addr) orelse 0;
    state.s[dest_s] = loaded;
}

fn execX17Xpm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s2 = bundle[2];

    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a_half: u32 = s2 & 0x3;
    const a_idx: u32 = (8 * a_half + 4) & 0xF;

    const addr: u32 = state.a[a_idx];
    const loaded: u32 = state.getX(addr) orelse 0;
    state.s[dest_s] = loaded;
}

fn execX17Rsub(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s1 = bundle[1];
    const s2 = bundle[2];

    const dest_s: u32 = (s1 >> 19) & 0xF;
    const imm_b: u32 = (s1 >> 8) & 0xF;
    const op: u32 = (s1 >> 12) & 0x7;
    const a_src: u32 = (s2 >> 25) & 0xF;

    // Only op = 5 is modelled today (b - a).  Other ops await a measurement.
    if (op != 5) return RunError.UnknownForm;

    const a_val: u32 = if (a_src == 0) 0 else state.s[a_src];
    const b_signed: i64 = @intCast(imm_b);
    const a_signed: i64 = sx24(a_val);
    const diff: i32 = @truncate(b_signed - a_signed);
    state.s[dest_s] = @as(u32, @bitCast(diff)) & M24;
}

fn execX17Mve(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s1 = bundle[1];
    const s2 = bundle[2];

    const dest_e: u32 = (s1 >> 16) & 0xF;
    const src_s: u32 = (s2 >> 25) & 0xF;

    const src_val: u32 = if (src_s == 0) 0 else state.s[src_s];
    state.e[dest_e] = (src_val +% 1) & M24;
}

fn execX17Alu05(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s2 = bundle[2];

    const dest: u32 = (head >> 15) & 0xF;
    const a_idx: u32 = (head >> 11) & 0xF;
    const b_idx: u32 = (s2 >> 25) & 0xF;
    const code: u32 = (head >> 5) & 0x3F;

    const a_val: u32 = if (a_idx == 0) 0 else state.s[a_idx];
    const b_val: u32 = if (b_idx == 0) 0 else state.s[b_idx];

    const op = ALU05[code];
    state.s[dest] = alu05Eval(op, sx24(a_val), sx24(b_val));
}

fn execX17Ld(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s1 = bundle[1];
    const s2 = bundle[2];

    const dest_s: u32 = (s1 >> 19) & 0xF;
    const src_s: u32 = (s2 >> 25) & 0xF;
    const shift: u32 = (s1 >> 23) & 0xF;
    const src_e_mem: u32 = (s1 >> 15) & 0xF;
    const dest_e_half: u32 = s1 & 0x7;

    // Scalar asr first (reads pre-state s<src_s>, writes s<dest_s>).
    const a_val: u32 = if (src_s == 0) 0 else state.s[src_s];
    const shifted: u32 = @as(u32, @bitCast(sx24(a_val) >> @as(u5, @intCast(shift & 0x1F)))) & M24;
    state.s[dest_s] = shifted;

    // Y-memory load: fetch Y:[e<src_e_mem>] and land it in e<2*dest_e_half>.
    const y_addr: u32 = if (src_e_mem == 0) 0 else state.e[src_e_mem];
    const loaded: u32 = state.getY(y_addr) orelse 0;
    state.e[2 * dest_e_half] = loaded;
}

fn execX17E(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];

    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    const dest_e_a: u32 = (s2 >> 8) & 0xF;
    const dest_e_b: u32 = (s1 >> 13) & 0xF;
    const src_e: u32 = (s2 >> 16) & 0xF;
    const en1: bool = ((s1 >> 18) & 1) == 1;
    const en2: bool = ((s1 >> 17) & 1) == 1;

    // Reads must happen BEFORE writes: "old e<S2[19:16]>" is the pre-state.
    const src_val: u32 = if (src_e == 0) 0 else state.e[src_e];

    if (en1) {
        state.e[dest_e_a] = imm;
        state.s[dest_s] = imm;
    }
    if (en2) {
        state.e[dest_e_b] = (src_val -% 1) & 0xFF_FFFF;
    }
}

fn execX17Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (s1 >> 7) & 0xF;
    const imm: u32 = (head >> 11) & 0xFFFF;
    state.s[dest_s] = imm;
}

// ---------------------------------------------------------------------------
// ALU05: the 64-entry class-05 ALU table shared across many sm1 forms.
// Ported from upstream/tools/sm1isa/emu.py (RE tree e55e26d21, fact 66+).
// ---------------------------------------------------------------------------

/// Op name for each of the 64 class-05 codes, as decoded in emu.py's ALU05.
pub const AluOp = enum {
    add,
    addi,
    sub,
    rsub,
    nadd,
    xor_op,
    xori,
    clr,
    mova,
    movb,
    shli,
    asr,
    asrrs,
    lsr,
    sel,
};

const ALU05: [64]AluOp = blk: {
    var t: [64]AluOp = @splat(AluOp.add); // default; overwritten below
    // "add" codes
    for ([_]u8{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x3c, 0x3d, 0x3f }) |c| t[c] = .add;
    t[0x09] = .addi;
    t[0x0a] = .addi;
    t[0x10] = .sub;
    t[0x11] = .sub;
    t[0x12] = .rsub;
    t[0x13] = .rsub;
    t[0x14] = .nadd;
    t[0x15] = .nadd;
    // xor family
    for ([_]u8{ 0x16, 0x17, 0x18, 0x19, 0x1d, 0x1e, 0x1f, 0x25 }) |c| t[c] = .xor_op;
    t[0x1c] = .xori;
    // clr
    for ([_]u8{ 0x20, 0x21, 0x22, 0x28, 0x29, 0x2e, 0x2f, 0x39, 0x3b }) |c| t[c] = .clr;
    // mova
    for ([_]u8{ 0x23, 0x24, 0x27, 0x2b }) |c| t[c] = .mova;
    // movb
    for ([_]u8{ 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 }) |c| t[c] = .movb;
    t[0x3a] = .shli;
    t[0x06] = .asr;
    t[0x1a] = .asr;
    t[0x07] = .asrrs;
    t[0x08] = .lsr;
    t[0x1b] = .lsr;
    t[0x3e] = .lsr;
    // sel
    for ([_]u8{ 0x26, 0x2a, 0x2c, 0x2d }) |c| t[c] = .sel;
    break :blk t;
};

/// Evaluate an ALU05 op with no m/s modifiers (adder-m == 0, shift-s == 0).
/// Mirrors upstream emu.py's `alu05()` with the shift ops dispatched through
/// `aluShift`.  Returns the 24-bit wrapped result.
pub fn alu05Eval(op: AluOp, a_s: i32, b_s: i32) u32 {
    const a24: u32 = @as(u32, @bitCast(a_s)) & M24;
    const b24: u32 = @as(u32, @bitCast(b_s)) & M24;
    return switch (op) {
        .add => @as(u32, @bitCast(@as(i32, @truncate(@as(i64, a_s) + @as(i64, b_s))))) & M24,
        .addi => b24, // immediate forms handled by callers that know `s`; raw base is b
        .sub => @as(u32, @bitCast(@as(i32, @truncate(@as(i64, a_s) - @as(i64, b_s))))) & M24,
        .rsub => @as(u32, @bitCast(@as(i32, @truncate(@as(i64, b_s) - @as(i64, a_s))))) & M24,
        .nadd => @as(u32, @bitCast(@as(i32, @truncate(-@as(i64, a_s) - @as(i64, b_s) - 2)))) & M24,
        .xor_op => (a24 ^ b24) & M24,
        .xori, .shli => b24, // immediate: result is the imm field; wraps through callers w/ `s`
        .mova => a24,
        .movb => b24,
        .clr => 0,
        .sel => if (@as(i64, a_s) + @as(i64, b_s) >= 0) a24 else b24,
        .asr, .asrrs, .lsr => aluShift(op, a_s, b_s),
    };
}

fn aluShift(op: AluOp, a_s: i32, b_s: i32) u32 {
    // upstream shift(): n = sx24(a); v = (b & M24) if lsr else sx24(b).
    const n: i32 = a_s;
    const v: i64 = if (op == .lsr) @as(i64, @as(u32, @bitCast(b_s)) & M24) else @as(i64, b_s);
    if (n < 0) {
        const amount: u6 = @intCast(@min(@as(i32, -n), 48));
        const shifted: i64 = v << amount;
        if (op == .asrrs) {
            const sat: i64 = @max(@as(i64, -0x800000), @min(@as(i64, 0x7FFFFF), shifted));
            return @as(u32, @bitCast(@as(i32, @truncate(sat)))) & M24;
        }
        return @as(u32, @bitCast(@as(i32, @truncate(shifted)))) & M24;
    }
    var vv: i64 = v;
    if (op == .asrrs and n != 0) vv += @as(i64, 1) << @as(u6, @intCast(n - 1));
    const amount: u6 = @intCast(@min(n, 48));
    const result: i64 = vv >> amount;
    return @as(u32, @bitCast(@as(i32, @truncate(result)))) & M24;
}

// ADDER_M table rewrites the (x, y, c) tuple fed into x + y + c, per upstream
// emu.py.  7 entries; [25:23] of the ALU word picks one.  Returned fields
// are 24-bit signed ints (as i64 because ~x can grow by a bit).
const AdderTuple = struct { x: i64, y: i64, c: i64 };

fn adderM(m: u8, x: i64, y: i64, c: i64) AdderTuple {
    return switch (m & 7) {
        0 => .{ .x = x, .y = y, .c = c },
        1 => .{ .x = ~x, .y = y, .c = 1 },
        2 => .{ .x = 0, .y = y, .c = c },
        3 => .{ .x = 0, .y = y, .c = 1 },
        4 => .{ .x = -1, .y = y, .c = c },
        5 => .{ .x = x, .y = ~y, .c = 1 },
        6 => .{ .x = 0, .y = ~y, .c = 1 },
        7 => .{ .x = 0, .y = y, .c = c },
        else => unreachable,
    };
}

/// XOR_M: for xor at code 0x16 with s = 0, [25:23] = m picks the combiner.
/// Entry 5 is OPEN in upstream.  Returns null for OPEN.
fn xorM(m: u8, a: u32, b: u32) ?u32 {
    return switch (m & 7) {
        0 => (a ^ b) & M24,
        1 => (~(a ^ b)) & M24,
        2 => (a | b) & M24,
        3 => (a & b) & M24,
        4 => (~a) & M24,
        5 => null,
        6 => a & M24,
        7 => a & M24,
        else => unreachable,
    };
}

/// Upstream emu.py's `alu_word`: one class-05 word with its [25:19]
/// modifiers resolved.  Returns null for cases upstream marks OPEN (clr with
/// modifiers), and `RunError.UnknownForm` for a (code, m, s) triple no rule
/// in upstream covers either.
pub fn aluWord(h: u32, a_s: i32, b_s: i32) RunError!?u32 {
    const code: u8 = @intCast((h >> 5) & 0x3F);
    const m: u8 = @intCast((h >> 23) & 0x7);
    const s_mod: u8 = @intCast((h >> 19) & 0xF);
    const op = ALU05[code];
    const a24: u32 = @as(u32, @bitCast(a_s)) & M24;
    const b24: u32 = @as(u32, @bitCast(b_s)) & M24;

    // m == 0 and s == 0: fall through to the base ALU05 eval.
    if (m == 0 and s_mod == 0) {
        return alu05Eval(op, a_s, b_s);
    }
    // clr with modifiers: upstream marks the source OPEN.
    if (op == .clr) return null;
    // mova/asr/asrrs/lsr with s == 0: modifier m is ignored.
    if ((op == .mova or op == .asr or op == .asrrs or op == .lsr) and s_mod == 0) {
        return alu05Eval(op, a_s, b_s);
    }
    // adder family: add / addi / sub-with-s=0.
    if (op == .add or op == .addi or (op == .sub and s_mod == 0)) {
        var x0: i64 = undefined;
        var y0: i64 = undefined;
        var c0: i64 = undefined;
        switch (op) {
            .add => {
                x0 = a_s;
                y0 = b_s;
                c0 = 0;
            },
            .addi => {
                x0 = @as(i32, s_mod);
                y0 = b_s;
                c0 = 0;
            },
            .sub => {
                x0 = a_s;
                // Python: sx24(y & M24) if y >= 0 else y. Here we feed ~b.
                y0 = ~@as(i64, b_s);
                c0 = 1;
            },
            else => unreachable,
        }
        const t = adderM(m, x0, y0, c0);
        var r: i64 = t.x + t.y + t.c;
        if (op == .add and code <= 2) {
            r = r << @as(u6, @intCast(s_mod & 0x3F));
        } else if (op == .add and code <= 5) {
            const lo: u32 = @as(u32, @bitCast(@as(i32, @truncate(r)))) & M24;
            var lo_s: i64 = sx24FromU32(lo);
            if (code == 4 and s_mod != 0) lo_s += @as(i64, 1) << @as(u6, @intCast(s_mod - 1));
            r = lo_s >> @as(u6, @intCast(s_mod & 0x3F));
        }
        return @as(u32, @bitCast(@as(i32, @truncate(r)))) & M24;
    }
    if (m == 0 and op == .xori) return (b24 ^ s_mod) & M24;
    if (m == 0 and op == .shli) return (b24 << @as(u5, @intCast(s_mod & 0x1F))) & M24;
    if (m == 0 and code == 0x30) return b24;
    // xor at code 0x16 or 0x1F with the right m/s.
    if ((s_mod == 0 and code == 0x16) or (code == 0x1F and m == 2 and s_mod == 0)) {
        return xorM(m, a24, b24);
    }
    return RunError.UnknownForm;
}

fn sx24FromU32(v: u32) i64 {
    if ((v & 0x0080_0000) != 0) return @as(i64, v) - 0x100_0000;
    return @as(i64, v);
}

fn execAlu(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    // dest s[18:15], a s[14:11], b s[29:26], op [10:5]; [25:19] = 0 reserved.
    const dest: u32 = (head >> 15) & 0xF;
    const a_f: u32 = (head >> 11) & 0xF;
    const b_f: u32 = (head >> 26) & 0xF;
    const code: u32 = (head >> 5) & 0x3F;

    // fact 70: source 0 reads zero.
    const a_val: u32 = if (a_f == 0) 0 else state.s[a_f];
    const b_val: u32 = if (b_f == 0) 0 else state.s[b_f];

    const op = ALU05[code];
    const result = alu05Eval(op, sx24(a_val), sx24(b_val));
    state.s[dest] = result;
}

// ---------------------------------------------------------------------------
// AGU10: address-generation ops indexed by a 4-bit FN field.  Ported from
// upstream emu.py (fact 109).  Values compute over (x, y) and wrap at
// 16 bits (the AGU is a 16-bit address unit, not a 24-bit ALU).
// ---------------------------------------------------------------------------

fn agu10Eval(fn_op: u32, x: u32, y: u32) RunError!u32 {
    const xw: u32 = x & 0xFFFF;
    const yw: u32 = y & 0xFFFF;
    const r: u32 = switch (fn_op & 0xF) {
        0 => xw,
        1 => yw,
        2 => (xw +% 1) & 0xFFFF,
        3 => (yw +% 1) & 0xFFFF,
        4 => (xw -% 1) & 0xFFFF,
        5 => (yw -% 1) & 0xFFFF,
        6 => (xw +% yw) & 0xFFFF,
        7 => (xw -% yw) & 0xFFFF,
        // 8 = "i" (uses an immediate not in this bundle): OPEN here.
        9 => 0,
        // 10 = "y+i", 11 = "y-i": also need the imm form; OPEN here.
        12, 14, 15 => yw,
        13 => (xw +% yw) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    return r;
}

fn execX17Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s1 = bundle[1];
    const s2 = bundle[2];
    const fn_op: u32 = (s1 >> 24) & 0xF;
    const x_idx: u32 = (s1 >> 5) & 0xF;
    const y_idx: u32 = (s2 >> 1) & 0xF;
    const dest: u32 = (s2 >> 5) & 0xF;

    // fact 70: index 0 reads zero.
    const x_val: u32 = if (x_idx == 0) 0 else state.e[x_idx];
    const y_val: u32 = if (y_idx == 0) 0 else state.r[y_idx];
    const v = try agu10Eval(fn_op, x_val, y_val);
    if (dest != 0) state.r[dest] = v;
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
    // S2 class 101 is a control bundle; S1[22:20] decides.
    //   011 -> halt       (the tail)
    //   000 -> continue   (no state change, PC falls through)
    if (((bundle[2] >> 29) & 0x7) != 0b101) return false;
    const ctl: u32 = (bundle[1] >> 20) & 0x7;
    return ctl == 0b011;
}

fn execPtst(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const p_dest: u32 = (head >> 26) & 0xF;
    const cond: u32 = (head >> 21) & 0x1F;
    const a_s: u32 = (head >> 11) & 0xF;
    const a_val: u32 = state.s[a_s] & M24;
    // 24-bit signed: sign bit is bit 23.
    const signed: i32 = if (a_val & 0x800000 != 0)
        @as(i32, @bitCast(a_val | 0xFF000000))
    else
        @as(i32, @bitCast(a_val));
    const v: u8 = switch (cond) {
        0, 2 => if (signed <= 0) 1 else 0, // c0 / c2: <= 0
        1 => if (signed < 0) 1 else 0, // c1: < 0
        4, 6 => if (signed >= 0) 1 else 0, // c4 / c6: >= 0
        7 => 1, // c7: always true
        else => return RunError.UnknownForm,
    };
    if (p_dest != 0) state.p[p_dest] = v;
}

fn execPlog(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest: u32 = (head >> 11) & 0xF;
    const a: u32 = (head >> 21) & 0xF;
    const b: u32 = (head >> 26) & 0x7;
    const fn_op: u32 = (head >> 15) & 0xF;
    const av: u8 = state.p[a];
    const bv: u8 = state.p[b];
    const v: u8 = switch (fn_op) {
        0 => bv,
        1 => if (av == bv) 1 else 0,
        2 => av ^ bv,
        3 => av & bv,
        4 => av | bv,
        5 => (~(av & bv)) & 1,
        6 => (~(av | bv)) & 1,
        7 => (~bv) & 1,
        8 => av,
        9 => 0,
        else => 1,
    };
    if (dest != 0) state.p[dest] = v;
}

fn execLdy(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest_k: u32 = (head >> 26) & 0x1F;
    const src_e: u32 = (head >> 5) & 0xF;
    if (dest_k < 16) return;
    const dest_s: u32 = dest_k - 16;
    const addr: u32 = state.e[src_e];
    const v: u32 = state.getY(addr) orelse 0;
    state.s[dest_s] = v;
}

fn execStx(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const a_idx: u32 = (head >> 11) & 0xF;
    const src_s: u32 = (head >> 16) & 0xF;
    const addr: u32 = state.a[a_idx];
    const v: u32 = state.s[src_s] & M24;
    try state.x_mem.put(gpa, addr, v);
}

fn execPtest(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const p_src: u32 = (s1 >> 24) & 0xF;
    const fn_op: u32 = (s1 >> 21) & 0x7;
    const pv: u8 = state.p[p_src];
    const v: u32 = switch (fn_op) {
        0 => if (pv == 0) 1 else 0, // !p
        1 => @as(u32, pv), // p
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execPsel(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    _ = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (s1 >> 8) & 0xF;
    const a_s: u32 = (s1 >> 28) & 0xF;
    const b_s: u32 = (s1 >> 4) & 0xF;
    const op: u32 = (s1 >> 16) & 0x7;
    const p_src: u32 = (s1 >> 24) & 0xF;
    const pol: u32 = (s1 >> 21) & 1;
    const raw: u8 = state.p[p_src];
    const pv: u8 = if (pol == 1) raw else @as(u8, 1) -% raw;
    const a_val: u32 = state.s[a_s] & M24;
    const b_val: u32 = state.s[b_s] & M24;
    const combined: u32 = switch (op) {
        4 => (a_val +% b_val) & M24,
        else => return RunError.UnknownForm,
    };
    const v: u32 = if (pv != 0) combined else b_val;
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execLd0e(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const addr: u32 = (head >> 5) & 0xFFFF;
    const n: u32 = (s1 >> 1) & 0x7;
    if (dest_s != 0) state.s[dest_s] = 0;
    const a_idx: u32 = (4 * n + 1) & 0xF;
    state.a[a_idx] = addr;
}

fn execLde(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const k: u32 = (head >> 26) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    // k = 0 maps to an A-register store (still OPEN).  Don't guess.
    if (k == 0) return;
    state.e[k] = imm;
}

fn execRop(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest_s: u32 = (head >> 26) & 0x7;
    const dest_r: u32 = (head >> 22) & 0xF;
    const x_e: u32 = (head >> 5) & 0xF;
    const y_r: u32 = (head >> 13) & 0xF;
    const fn_op: u32 = (head >> 17) & 0xF;

    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];

    // ROP07: minimal table seeded with the two measured ops.
    const v: u32 = switch (fn_op) {
        0 => x_val & M24,
        6 => (x_val +% y_val) & M24,
        else => return RunError.UnknownForm,
    };

    if (dest_s != 0) state.s[dest_s] = v;
    if (dest_r != 0) state.r[dest_r] = v;
}

fn execRli(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest_r: u32 = (head >> 27) & 0xF;
    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    if (dest_r != 0) state.r[dest_r] = imm;
    if (dest_s != 0) state.s[dest_s] = imm;
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

test "alu add: s7 = s2 + s9" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0x100;
    state.s[9] = 0x200;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x00");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x300), state.s[7]);
}

test "alu sub: 0x300 - 0x100 = 0x200 (code 0x10)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0x300;
    state.s[9] = 0x100;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x10");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x200), state.s[7]);
}

test "alu rsub: s9 - s2 = 0x100 (code 0x12)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0x200;
    state.s[9] = 0x300;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x12");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x100), state.s[7]);
}

test "alu xor: 0xAAAA ^ 0x5555 = 0xFFFF (code 0x16)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0xAAAA;
    state.s[9] = 0x5555;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x16");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0xFFFF), state.s[7]);
}

test "alu clr: s7 = 0 (code 0x20)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0xDEAD;
    state.s[9] = 0xBEEF;
    state.s[7] = 0x1234;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x20");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0), state.s[7]);
}

test "alu movb: s7 = s9 (code 0x30)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0xDEAD;
    state.s[9] = 0x6789;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x30");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x6789), state.s[7]);
}

test "alu nadd: -a - b - 2 (code 0x14) wraps to 24 bits" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[2] = 0x100;
    state.s[9] = 0x200;

    const b = try asm_mod.assembleOne("alu s7, s2, s9, #0x14");
    try executeBundle(testing.allocator, &state, b.slice());
    // -0x100 - 0x200 - 2 = -0x302; wrapped to 24 bits = 0xFFFCFE.
    try testing.expectEqual(@as(u32, 0xFFFCFE), state.s[7]);
}

test "aluWord parity: m=0 s=0 matches alu05Eval" {
    // add with no modifiers: word h encodes code=0 at [10:5], m/s both 0.
    // a=0x100, b=0x200 -> 0x300.
    const h: u32 = 0 << 5;
    const r = (try aluWord(h, 0x100, 0x200)).?;
    try testing.expectEqual(@as(u32, 0x300), r);
}

test "aluWord: add with m=2 (x=0) yields y + c = b" {
    // code=0 (add), m=2, s=0: adderM(2) -> (0, b, 0); result = b.
    // h fields: m at [25:23]. m=2 -> bits [25:23]=010, byte = (2 << 23).
    const h: u32 = (2 << 23); // code=0, s=0
    const r = (try aluWord(h, 0x100, 0x200)).?;
    try testing.expectEqual(@as(u32, 0x200), r);
}

test "aluWord: add with s=3 code<=2 shifts left" {
    // code=0 (add), m=0, s=3: result = (a + b) << 3.
    // s at [22:19], s=3 -> bits [22:19]=0011, byte = (3 << 19).
    const h: u32 = (3 << 19); // code=0, m=0
    const r = (try aluWord(h, 0x100, 0x200)).?;
    try testing.expectEqual(@as(u32, 0x1800), r);
}

test "aluWord: clr with modifiers is OPEN (null)" {
    // code 0x20 is clr; m=1 -> OPEN per upstream.
    const h: u32 = (0x20 << 5) | (1 << 23);
    const r = try aluWord(h, 0x100, 0x200);
    try testing.expect(r == null);
}

test "aluWord: xori at m=0, s=5 xors immediate" {
    // code 0x1C = xori; m=0, s=5 -> result = b ^ 5.
    const h: u32 = (0x1C << 5) | (5 << 19);
    const r = (try aluWord(h, 0, 0x00F)).?;
    try testing.expectEqual(@as(u32, 0x0A), r);
}

test "aluWord: shli at m=0, s=4 shifts immediate" {
    // code 0x3A = shli; result = b << s.
    const h: u32 = (0x3A << 5) | (4 << 19);
    const r = (try aluWord(h, 0, 0x3)).?;
    try testing.expectEqual(@as(u32, 0x30), r);
}

test "alu register 0 reads as zero (fact 70)" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[0] = 0xDEAD; // ignored for both a and b slots
    state.s[9] = 0x100;

    // add: a (s0 -> 0) + b (s9) = 0x100.
    const b = try asm_mod.assembleOne("alu s7, s0, s9, #0x00");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0x100), state.s[7]);
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
