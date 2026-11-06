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
    /// When set, profile[i] counts the bundles run that start at syllable i.
    profile: ?[]u64 = null,
    /// Scalar registers s0..s15.  Default zero.
    s: [16]u32 = @splat(0),
    /// Element registers e0..e15.  Default zero.
    e: [16]u32 = @splat(0),
    /// R-file registers R0..R15.  R0 pinned to zero.
    r: [16]u32 = @splat(0),
    /// Per-lane "V" data register: what `st.y` writes to memory.  A program is
    /// one sequential stream laid across four lane images (seq word i in lane
    /// (i // 2) % 4); no parallel execution has been seen, so lanes are storage
    /// as far as anything measured shows.  The lane of the current syllable is
    /// `cur_lane`.  Default zero - tests seed the measured lane values.
    v: [4]u32 = @splat(0),
    /// Which lane the current bundle runs on (0..3).  The ref string calls
    /// this out explicitly for st.y ("the LANE's data register"); other
    /// forms may acquire lane dependence as they port.
    cur_lane: u8 = 0,
    /// 0x16 X index latch (fact 116, 122); null until a bundle sets it.
    x16_xi: ?u32 = null,
    /// Address registers A0..A15: feed X/Y load+store addresses.  Default
    /// zero; seeded via tests or set-up forms that write them.
    a: [32]u32 = @splat(0),
    /// Predicate bits p0..p15 (1-bit each; stored as u8 for ergonomics).
    /// Written by ptst / plog; consumed by psel / jmp / bcc.  Survives
    /// across bundles per fact.
    p: [16]u8 = @splat(0),
    /// Y data memory, keyed by address.  Sparse: only addresses the program
    /// writes are present.  Caller inspects via `getY`.
    y_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},
    /// X data memory, keyed by address.  Separate bank from Y per facts.
    x_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},
    /// Pending branch state.  `jump_delay` counts down bundles until the
    /// jump fires; 0 means no pending jump.  `jump_target` is the syllable
    /// index to resume at when the delay elapses.  sm1 takes TWO delay-slot
    /// bundles (fact 94): the jmp bundle sets delay = 2, the two
    /// following bundles run, then i jumps to `jump_target`.
    jump_delay: u8 = 0,
    jump_target: usize = 0,
    /// Set by a jmp/br/bcc instruction to request a jump.  Measured in
    /// syllables relative to the jmp bundle's own syllable position; null
    /// when no jump requested this bundle.  runLinear consumes it.
    pending_jmp_offset: ?i32 = null,
    /// The running bundle's syllable index, for absolute jumps.
    pc: usize = 0,
    /// The value the last X read left (fact 241).
    xd: u32 = 0,
    /// xd as it stood before the running bundle: the multiply's X (fact 161).
    xd_pre: u32 = 0,
    /// The Y read latch yagu loads (fact 100, 105, 110).
    yd: u32 = 0,
    /// Set when runLinear met a stop; a run without one is a wedge on silicon.
    halted: bool = false,
    /// The 0x16 X store address latch, one bundle behind (fact 90).
    xa: ?u32 = null,
    /// The 0x18 class-000 phase T, flipped by w2[17] = 0 (fact 180).
    x18_t: bool = false,

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

    /// An executor's X read: also latches xd, the last X word read
    /// (fact 161, 241).
    pub fn readX(self: *State, addr: u32) ?u32 {
        const v = self.x_mem.get(addr);
        self.xd = v orelse 0;
        return v;
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
    state.xd_pre = state.xd;

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
        if (s_class == 0b110) return execEalu(gpa, state, bundle);
        if (s_class == 0b000) return execBcc(gpa, state, bundle);
        if (s_class == 0b100) return execX10Alu(state, bundle);
        if (s_class == 0b010) return execYstm(gpa, state, bundle);
    }
    if (low5 == 0x14 and bundle.len >= 2 and head & 0xA0000000 == 0 and (bundle[1] >> 25) & 1 == 1) {
        if (bundle[1] >> 29 == 0b100) return execYagu(state, bundle);
        if (bundle[1] >> 30 == 0b11) return execYpair(state, bundle);
    }
    if (low5 == 0x12 and bundle.len == 2 and bundle[1] & 0xE1F00000 == 0x20000000 and (bundle[1] >> 19) & 1 == 1) {
        return execTst(state, bundle);
    }
    if (low5 == 0x0A and (head >> 30) & 1 == 1 and (head >> 20) & 3 == 3) return execPar(gpa, state, head);
    if (low5 == 0x14 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b001) return execXst(gpa, state, bundle);
        if (s_class == 0b000) return execYtst(gpa, state, bundle);
        if (s_class == 0b101) return execYrm(gpa, state, bundle);
        if (s_class == 0b010) return execP010(gpa, state, bundle);
    }
    if (low5 == 0x12 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b001) return execX12Agu(gpa, state, bundle);
        if (s_class == 0b101) return execX12Aset(gpa, state, bundle);
    }
    if (low5 == 0x0B) {
        const high3: u32 = (head >> 29) & 0x7;
        if (high3 == 0b010 or high3 == 0b011) return execJmp(gpa, state, head);
        if (high3 == 0b001 and (head >> 19) & 1 == 1) return execPlog(gpa, state, head);
        if (high3 == 0b100) return execAset0b(state, head);
    }
    // The class-05 ALU word with its [25:19] modifiers (fact 65-67).
    if (low5 == 0x05 and (head >> 30) == 1) return execAlu(gpa, state, head);
    // ptst at class 011: head[29] is the dest's bit 3 (fact 219).
    if (low5 == 0x0D and (head >> 29) == 0b011) return execPtst(gpa, state, head);
    // lde with [31] = 1 (fact 280): same fields as class 01x.
    if (low5 == 0x06 and (head >> 30) == 0b11) return execLde(gpa, state, head);
    // rop at classes 011 and 100 (fact 281, 285): s<[29:26]>.
    if (low5 == 0x07 and ((head >> 29) == 0b011 or (head >> 29) == 0b100)) return execRop(gpa, state, head);
    if (low5 == 0x0C and (head >> 31) == 1) return execRagu0c(state, head);
    if (low5 == 0x0F and bundle.len == 2) {
        const w1 = bundle[1];
        if (w1 >> 29 == 0b001 and (head >> 29) != 0b001) return execSte(state, bundle);
        if (w1 >> 29 == 0b101 and w1 & 0x00500000 == 0x00500000) return execStx0f(gpa, state, bundle);
        if (w1 >> 29 == 0b101) return execRimm(state, bundle);
    }
    if (low5 == 0x08 and (head >> 29) == 0b010 and (head >> 5) & 0x30 == 0x30) {
        return execStk(gpa, state, head);
    }
    if (low5 == 0x08) {
        return execBr(gpa, state, head);
    }
    if (low5 == 0x15 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b101) return execX15Alu(gpa, state, bundle);
        if (s_class == 0b001) return execX15Pt(gpa, state, bundle);
    }
    if (low5 == 0x16 and bundle.len >= 3) {
        const s2_class: u32 = (bundle[2] >> 29) & 0x7;
        const s2_mode: u32 = (bundle[2] >> 16) & 0x1F;
        const s1_class16: u32 = (bundle[1] >> 29) & 0x7;
        if (s2_class == 0b110 and s1_class16 == 0 and s2_mode <= 7) return execX16Mul(gpa, state, bundle);
        if (s2_class == 0b110 and s1_class16 == 1) return execX16Mac(state, bundle);
        if (s2_class == 0b100) return execX16Agu100(state, bundle);
        if (s1_class16 == 1 and s2_class == 0b111) return execX16B16(gpa, state, bundle);
        if (s1_class16 == 0 and s2_class == 0 and (bundle[2] >> 12) & 1 == 1) return execX16Sh(state, bundle);
        if (s1_class16 == 0 and s2_class == 0b010) return execX16Xs(state, bundle);
        if (s1_class16 == 3 and s2_class == 0b000) return execX16Xld(state, bundle);
        if (s1_class16 == 3 and s2_class == 0b111) return execX16St2(gpa, state, bundle);
        if (s2_class == 0b101) {
            if (s1_class16 == 0) return execX16Agu2(state, bundle);
            return execX16AguMul(state, bundle);
        }
        if (s1_class16 == 0 and s2_class == 0b111) return execX16AluX(state, bundle);
        if (s1_class16 == 0 and s2_class == 0b000) return execX16Ld(state, bundle);
        if (s2_class == 0b011) return execX16Agu(gpa, state, bundle);
        if (s2_class == 0b001) {
            // head[25:11] carrying an ALU05 payload → x16.hd (head-ALU).
            const alu_code: u32 = (head >> 5) & 0x3F;
            const alu_dest: u32 = (head >> 15) & 0xF;
            if (alu_dest != 0 and alu_code != 0) return execX16Hd(gpa, state, bundle);
            return execX16Mul2(gpa, state, bundle);
        }
    }
    if (low5 == 0x13 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        if (s_class == 0b001) return execX13Agu(gpa, state, bundle);
        if (s_class == 0b101) {
            const h13 = bundle[0];
            const code13: u32 = (h13 >> 5) & 0x3F;
            if ((bundle[1] >> 19) & 1 == 1) return execMul13(state, bundle);
            if (code13 != 0) return execX13St(gpa, state, bundle);
            // mul13 dest field location still open; not yet modelled.
            return RunError.UnknownForm;
        }
    }
    if (low5 == 0x18 and bundle.len >= 4) {
        const w3_class: u32 = (bundle[3] >> 29) & 0x7;
        const w1_mode: u32 = (bundle[1] >> 12) & 0x7;
        if (w3_class == 0b110 and ((bundle[3] >> 8) & 1) == 1) return execX18Br(gpa, state, bundle);
        if (w3_class == 0b011 and bundle[3] & 0x101 == 0x100) return execX18Br(gpa, state, bundle);
        if (w3_class == 0b110 and w1_mode == 5) return execX18Xrd(gpa, state, bundle);
        if (w3_class == 0b110 and bundle[2] != 0) return execX18Mpy(gpa, state, bundle);
        if (w3_class == 0b110) return execX18Imm(gpa, state, bundle);
        if (w3_class == 0b000) return execX18C0(gpa, state, bundle);
        if (w3_class == 0b100) return execX18C4(gpa, state, bundle);
        if (w3_class == 0b011) return execX18C3(gpa, state, bundle);
        if (w3_class == 0b111) return execX18C7(gpa, state, bundle);
        if (w3_class == 0b101) return execX18C5(gpa, state, bundle);
        if (w3_class == 0b010) return execX18C2(gpa, state, bundle);
        if (w3_class == 0b001) return execX18C1(gpa, state, bundle);
    }
    if (low5 == 0x02 and bundle.len >= 6) {
        const w4_mode: u32 = (bundle[4] >> 11) & 0x7;
        if (w4_mode == 1) return execX02Imm(gpa, state, bundle);
        if (w4_mode == 5) {
            // Slots are VLIW; all slot inputs latch before any write
            // commits (fact 182/184 / own-x02x3, own-x02coll).
            // Snapshot s-regs so x3 reads pre-slot values.
            const s_pre = state.s;
            try execX02Xrd(gpa, state, bundle);
            try execX02Mpy(gpa, state, bundle);
            try execX02Ainc(gpa, state, bundle);
            try execX02X3Pre(gpa, state, bundle, s_pre);
            return;
        }
        if (w4_mode == 1) return execX02Imm(gpa, state, bundle);
        if (w4_mode == 0) return execX02Agu(gpa, state, bundle);
        if (w4_mode == 2) return execX02Xrd(gpa, state, bundle); // esrc path
    }
    if (low5 == 0x03 and bundle.len >= 6) {
        const w4_mode: u32 = (bundle[4] >> 11) & 0x7;
        if (w4_mode == 5) {
            try execX03Xrd(gpa, state, bundle);
            return execX03Agu(gpa, state, bundle);
        }
        if (w4_mode == 0) {
            try execX03Mode0(gpa, state, bundle);
            // w3[31] re-issues the bundle while p<w5[17:14]> holds, the X
            // address post-incrementing (fact 262-263).  p3 measured one
            // re-issue; p5 ran away (7168 stores).  What ends p3's is OPEN.
            if (bundle[3] >> 31 == 1) {
                const pi: u32 = (bundle[5] >> 14) & 0xF;
                if (state.p[pi] != 0) {
                    if (pi != 3) return RunError.RunawayProgram;
                    const ai = x03AIndex(bundle);
                    state.a[ai] +%= 1;
                    try execX03Mode0(gpa, state, bundle);
                }
            }
            return;
        }
        // Y path (w5[21] = 1): the read enters the 24-bit lane as D << 8;
        // mux 011 takes lane[15:0], mux 100 lane[16:1] (fact 236).
        if ((w4_mode == 3 or w4_mode == 4) and ((bundle[5] >> 21) & 1) == 1) {
            const dest_s: u32 = (bundle[4] >> 3) & 0xF;
            const src_e: u32 = (bundle[3] >> 27) & 0xF;
            const lane: u32 = ((state.getY(state.e[src_e]) orelse 0) << 8) & M24;
            const v: u32 = if (w4_mode == 3) lane & 0xFFFF else (lane >> 1) & 0xFFFF;
            if (dest_s != 0) state.s[dest_s] = v;
            return;
        }
        // w4 mux 001 with w5[29:28] = 10: s<w4[6:3]> <- e<w3[30:27]>
        // (fact 190 / own-x03esrc).
        if (w4_mode == 1 and ((bundle[5] >> 28) & 0x3) == 0b10) {
            const dest_s: u32 = (bundle[4] >> 3) & 0xF;
            const src_e: u32 = (bundle[3] >> 27) & 0xF;
            if (dest_s != 0) state.s[dest_s] = state.e[src_e];
            return;
        }
    }
    if ((low5 == 0x00 or (low5 == 0x01 and (head >> 30) == 0)) and bundle.len == 7) {
        return execX00(gpa, state, bundle);
    }
    if (low5 == 0x04 and bundle.len >= 6) {
        return execX04Agu(gpa, state, bundle);
    }
    if (low5 == 0x0E and bundle.len >= 2) {
        if (head >> 31 == 1) return execLd0e(gpa, state, bundle);
        if ((head >> 25) & 0x3F == 0x23) return execPsel(gpa, state, bundle);
        return execX0E(state, bundle);
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
        return execStY(gpa, state, bundle);
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
    // Bit c of the row is cond c; the row is picked by head[28:25] and what
    // it reads is OPEN (fact 107).
    const g: u32 = (bundle[0] >> 25) & 0xF;
    const row: u32 = if (g <= 10) 0xFFFF else if (g <= 13) 0xF8E3 else 0xFD1B;
    const v: u8 = @intCast((row >> @intCast(cond)) & 1);
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

fn execX02Ainc(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const w3 = bundle[3];
    // isa.toml (fact 157): A<w2[21:17]> and A<w3[14:10]> <- A<w3[4:0]> + 1.
    const dest_a: u32 = (w2 >> 17) & 0x1F;
    const dest_b: u32 = (w3 >> 10) & 0x1F;
    const src_a: u32 = w3 & 0x1F;
    // 5-bit A index; A0 reads zero.
    const src_val: u32 = if (src_a == 0) 0 else state.a[src_a & 0xF];
    const v: u32 = (src_val +% 1) & 0xFFFFFF;
    if (dest_a != 0) state.a[dest_a & 0xF] = v;
    if (dest_b != 0) state.a[dest_b & 0xF] = v;
}

fn execX02Mpy(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w1 = bundle[1];
    const w2 = bundle[2];
    // mode w1[13:11] = 4 (P).  Operand signedness (fact 185):
    // a is signed when w1[15] != w1[16]; b is signed when w1[15] = 1.
    // w1[17] = 1: floor(P / 2^24); else round(P / 2^23).
    const a_s: u32 = (w2 >> 1) & 0xF;
    const b_s: u32 = (w2 >> 5) & 0xF;
    const dest_s: u32 = (w2 >> 9) & 0xF;
    // w5[19] = 1: the w2 slot takes the w4 slot's X read instead of P
    // (fact 158, 161 / own-x02c6).
    if ((bundle[5] >> 19) & 1 == 1) {
        const a_idx: u32 = (bundle[4] >> 17) & 0xF;
        if (dest_s != 0) state.s[dest_s] = state.readX(state.a[a_idx]) orelse 0;
        return;
    }
    const b15: u32 = (w1 >> 15) & 1;
    const b16: u32 = (w1 >> 16) & 1;
    const av: i64 = mpyOperand(state.s[a_s], b15 != b16);
    const bv: i64 = mpyOperand(state.s[b_s], b15 == 1);
    const prod: i64 = av * bv;
    const scaled: i64 = if ((w1 >> 17) & 1 == 1)
        prod >> 24
    else
        (prod + (1 << 22)) >> 23;
    // Modes (fact 161): X = xd before the bundle, at the [15:0] slice;
    // 1 = a + X, 5 = P + X, 7 = X - P, otherwise P.
    const xv: i64 = @intCast((state.xd_pre << 8) & 0xFFFF);
    const r: i64 = switch ((w1 >> 11) & 7) {
        1 => @as(i64, sx24(state.s[a_s])) + xv,
        5 => scaled + xv,
        7 => xv - scaled,
        else => scaled,
    };
    const v: u32 = @truncate(@as(u64, @bitCast(r)) & M24);
    if (dest_s != 0) state.s[dest_s] = v;
    // x02.plo: w4[10:7] carries a second dest that gets (2P)[15:0].
    const w4 = bundle[4];
    const plo_dest: u32 = (w4 >> 7) & 0xF;
    if (plo_dest != 0) state.s[plo_dest] = @truncate(@as(u64, @bitCast(prod *% 2)) & 0xFFFF);
}

fn mpyOperand(raw: u32, signed: bool) i64 {
    const v: u32 = raw & M24;
    if (signed and (v & 0x800000) != 0) return @as(i64, v) - 0x1000000;
    return @as(i64, v);
}

/// Class 001 (fact 199): R slot R<w3[10:7]> <- head[30] = 0:
/// AGU10B[w1[31:28]](x = e<w3[28:25]>, y = R<w3[14:11]>); head[30] = 1:
/// AGU3[w3[24:20]](x = A<w3[19:15]>, y = R<w3[14:11]>).  Y store (w1[19]
/// and w2[11]) of s<head[25:22]> at Y:[e<w3[28:25]>].  Then the multiply.
fn execX18C1(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const xe_val: u32 = state.e[(w3 >> 25) & 0xF];
    const y: u32 = state.r[(w3 >> 11) & 0xF];
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const r_val: u32 = if ((h >> 30) & 1 == 1)
        agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0xF], y, 0)
    else
        agu10bEval((w1 >> 28) & 0xF, xe_val, y);
    const st_val: u32 = state.s[(h >> 22) & 0xF] & 0xFFFF;
    if ((w1 >> 19) & 1 == 1 and (w2 >> 11) & 1 == 1) {
        state.y_mem.put(gpa, xe_val, st_val) catch return RunError.OutOfMemory;
    }
    try execX18Q23c(gpa, state, bundle);
    if (dest_r != 0) state.r[dest_r] = r_val;
}

fn execX18Q23c(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 4) return RunError.UnknownForm;
    const w2 = bundle[2];
    const w3 = bundle[3];
    const dest_s: u32 = (w2 >> 12) & 0xF;
    const a_s: u32 = (w2 >> 16) & 0xF;
    const b_s: u32 = (w2 >> 20) & 0xF;
    const mode: u32 = (w2 >> 28) & 0x7;
    if (dest_s == 0) return;
    const av: u32 = if (a_s == 0) 0 else state.s[a_s] & 0xFFFF;
    const bv: u32 = if (b_s == 0) 0 else state.s[b_s] & 0xFFFF;
    const prod: u32 = av *% bv;
    const shift: u5 = if ((w3 >> 2) & 1 == 1) 24 else 23;
    const v: u32 = switch (mode) {
        4 => (prod >> shift) & M24,
        else => return,
    };
    state.s[dest_s] = v;
}

fn execX03Mpy(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w4 = bundle[4];
    const dest_s: u32 = (w2 >> 9) & 0xF;
    const a_s: u32 = (w2 >> 1) & 0xF;
    const b_s: u32 = (w2 >> 5) & 0xF;
    // w5[6] = 1: the w2 slot reads X:[A<{w1[2:0], head[31:30]}>] instead
    // of multiplying (fact 192 / own-x03xw2).
    if ((bundle[5] >> 6) & 1 == 1) {
        if (dest_s != 0) state.s[dest_s] = state.readX(state.a[x03AIndex(bundle)]) orelse 0;
        return;
    }
    const mode: u32 = (w1 >> 11) & 0x7;
    // Same signedness and scaling bits as 0x02 (fact 185, 191).
    const b15: u32 = (w1 >> 15) & 1;
    const b16: u32 = (w1 >> 16) & 1;
    const av: i64 = mpyOperand(state.s[a_s], b15 != b16);
    const bv: i64 = mpyOperand(state.s[b_s], b15 == 1);
    const prod: i64 = av * bv;
    const p: i64 = if ((w1 >> 17) & 1 == 1)
        prod >> 24
    else
        (prod + (1 << 22)) >> 23;
    if (dest_s != 0) {
        const v: i64 = switch (mode) {
            4 => p,
            // mode 5: P + X; w1[14] = 1 takes X from s<w1[31:29]>
            // (fact 254 / own-x03xsn).
            5 => blk: {
                if ((w1 >> 14) & 1 == 0) return;
                const x_s: u32 = (w1 >> 29) & 0x7;
                break :blk p + @as(i64, state.s[x_s] & M24);
            },
            else => return,
        };
        state.s[dest_s] = @truncate(@as(u64, @bitCast(v)) & M24);
    }
    // x03.plo: w4[10:7] carries a second dest that gets (2P)[15:0].
    const plo_dest: u32 = (w4 >> 7) & 0xF;
    if (plo_dest != 0) state.s[plo_dest] = @truncate(@as(u64, @bitCast(prod *% 2)) & 0xFFFF);
}

fn execX03Mode0(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    try execX03XStore(gpa, state, bundle);
    try execX03Alu(gpa, state, bundle);
    try execX03Mpy(gpa, state, bundle);
    return execX03RSlot(gpa, state, bundle);
}

fn x03AIndex(bundle: []const u32) u32 {
    return (((bundle[1] & 0x7) << 2) | ((bundle[0] >> 30) & 0x3)) & 0xF;
}

/// w5[19:18] = 11: X:[A<{w1[2:0], head[31:30]}>] <- sx24 s<{w2[0], w1[31:29]}>
/// (fact 255 / own-x03xst).  w5[18] alone stores nothing.
fn execX03XStore(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (((bundle[5] >> 18) & 0x3) != 0b11) return;
    const src_s: u32 = ((bundle[2] & 1) << 3) | ((bundle[1] >> 29) & 0x7);
    const raw: u32 = state.s[src_s] & 0xFFFF;
    const v: u32 = if (raw & 0x8000 != 0) raw | 0xFF0000 else raw;
    try state.x_mem.put(gpa, state.a[x03AIndex(bundle)], v);
}

/// 0x03 carries 0x02's R slot (fact 250).  w5[20] = 1 switches it
/// from AGU10B to AGU10 with i = 0 (fact 252 / own-x03ragu10).
fn execX03RSlot(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if ((bundle[5] >> 20) & 1 == 0) return execX02Agu(gpa, state, bundle);
    const fn_op: u32 = (bundle[1] >> 7) & 0xF;
    const dest_r: u32 = (bundle[3] >> 19) & 0xF;
    const y_r: u32 = (bundle[5] >> 22) & 0xF;
    if (dest_r == 0) return;
    const y_val: u32 = state.r[y_r];
    state.r[dest_r] = switch (fn_op) {
        3 => (y_val +% 1) & 0xFFFF,
        else => return,
    };
}

fn execX03Alu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const dest_s: u32 = (head >> 15) & 0xF;
    const a_s: u32 = (head >> 11) & 0xF;
    const b_s: u32 = (head >> 26) & 0xF;
    const code: u32 = (head >> 5) & 0x3F;
    if (dest_s == 0) return;
    const av: u32 = if (a_s == 0) 0 else state.s[a_s];
    const bv: u32 = if (b_s == 0) 0 else state.s[b_s];
    const op = ALU05[code];
    state.s[dest_s] = alu05Eval(op, sx24(av), sx24(bv));
}

/// 0x04 slots, inputs latched (fact 276, 290, 292, 293):
/// R<w3[22:19]> <- w5[27:26] = 01: AGU10[w1[10:7]](y = R<head[30:27]>,
/// i = head[26:11]), 10 = y+i, 11 = y-i; = 00: the Y read.  The Y read is
/// Y:[e<w3[30:27]>] and also goes to s<w4[6:3]> when w5[27:26] = 00.  X store
/// (w5[23:22] = 11) of sx24 s<{w2[0], w1[31:29]}> at X:[A<w4[21:17]>].
fn execX04Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const rsel: u32 = (w5 >> 26) & 3;
    const yd: u32 = (state.getY(state.e[(w3 >> 27) & 0xF]) orelse 0) & 0xFFFF;
    const dest_r: u32 = (w3 >> 19) & 0xF;
    const r_val: u32 = if (dest_r == 0) 0 else switch (rsel) {
        0 => yd,
        1 => blk: {
            const y: u32 = state.r[(head >> 27) & 0xF];
            const i: u32 = (head >> 11) & 0xFFFF;
            break :blk switch ((w1 >> 7) & 0xF) {
                10 => (y +% i) & 0xFFFF,
                11 => (y -% i) & 0xFFFF,
                else => return RunError.UnknownForm,
            };
        },
        else => return RunError.UnknownForm,
    };
    const dest_s: u32 = (w4 >> 3) & 0xF;
    const xst = (w5 >> 22) & 3 == 3;
    const x_addr: u32 = state.a[(w4 >> 17) & 0xF];
    const x_val: u32 = state.s[((w2 & 1) << 3) | (w1 >> 29)] & M24;
    if (xst) state.x_mem.put(gpa, x_addr, x_val) catch return RunError.OutOfMemory;
    if (dest_r != 0) state.r[dest_r] = r_val;
    if (dest_s != 0 and rsel == 0) state.s[dest_s] = yd;
}

/// 0x10 with a class-100 second (fact 269, 283): the head is a 0x05
/// ALU word (a = s<head[14:11]>, b = s<head[29:26]>) written to R<w1[24:21]>.
const X00_OPS = [16][]const u8{ "a", "0", "b", "-a", "0", "b+1", "a", "a", "0", "0", "0", "0", "0", "0", "1", "b-1" };
const X01_OPS = [16][]const u8{ "a", "y", "b", "-a", "-y", "b+1", "a+y", "a-y", "0", "0", "y", "y", "y-b", "b-y", "y-b+1", "y-b-1" };

/// w1[10:7] on the s slot's e path (fact 234): a = e<w4[8:5]>,
/// b = e<w4[4:1]>.  0x01's y source is OPEN.
fn x00Op(x01: bool, op: u32, a: u32, b: u32) RunError!u32 {
    const t = (if (x01) X01_OPS else X00_OPS)[op & 0xF];
    const r: u32 = if (std.mem.eql(u8, t, "a")) a else if (std.mem.eql(u8, t, "b")) b else if (std.mem.eql(u8, t, "-a")) 0 -% a else if (std.mem.eql(u8, t, "b+1")) b +% 1 else if (std.mem.eql(u8, t, "b-1")) b -% 1 else if (std.mem.eql(u8, t, "0")) 0 else if (std.mem.eql(u8, t, "1")) 1 else return RunError.UnknownForm;
    return r & 0xFFFF;
}

/// The 7-word 0x00 bundle and 0x01 (head[29] inert), slots latched then
/// written (fact 227-230, 234, 241-244, 286):
/// s slot s<w6[15:12]> <- mux w6[3:1] (0 / sub / sub / D / D / D / #head[26:11] / 0),
///   sub-mux w5[24:23] (#imm / 1 / the w1[10:7] op / D); D is not modelled;
/// Y store Y:[e<w4[8:5]>] <- s<w4[16:13]>, X store X:[A<w2[22:18]>] <- sx24 s<w2[13:10]>,
///   enabled by w5[5:4] / w5[2:1] = 11 on 0x00 and 8 bits higher on 0x01;
/// ALU slot s<w6[7:4]> <- ALU05[head[10:5]](0, s<w2[5:2]>) with shift w2[9:6];
/// MAC slot s<w6[11:8]> <- w4[23] = 1: xd (w4[17] = 1); else acc[15:0],
///   acc = mode w1[13:11]({s<w2[13:10]> : sx24 s<w4[15:13]>}, 2 s<w6[27:24]> s<w6[31:28]>);
/// multiply s<w6[19:16]> <- mode w1[13:11](X = s<w2[13:10]>, P = s*s >> 23 (24 if w1[17])
///   rounded when w1[15] = 1);
/// control w3[4:0]: jr +w1[25:18], ja w1[25:18], two delay slots.
fn execX00(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const w6 = bundle[6];
    const x01 = (h & 0x1F) == 0x01;
    const imm: u32 = (h >> 11) & 0xFFFF;

    const s_dest: u32 = (w6 >> 12) & 0xF;
    const s_val: u32 = if (s_dest == 0) 0 else blk: {
        const src: u32 = (w6 >> 1) & 7;
        const sub: u32 = (w5 >> 23) & 3;
        const op: u32 = (w1 >> 7) & 0xF;
        if (x01 and op != 2 and !(sub == 2 and (src == 1 or src == 2))) return RunError.UnknownForm;
        const sub_val: u32 = switch (sub) {
            0 => imm,
            1 => 1,
            2 => try x00Op(x01, op, state.e[(w4 >> 5) & 0xF], state.e[(w4 >> 1) & 0xF]),
            else => return RunError.UnknownForm,
        };
        break :blk switch (src) {
            0, 7 => 0,
            1, 2 => sub_val,
            6 => imm,
            else => return RunError.UnknownForm,
        };
    };

    const ye: u5 = if (x01) 12 else 4;
    const xe: u5 = if (x01) 9 else 1;
    const y_on = (w5 >> ye) & 3 == 3;
    const x_on = (w5 >> xe) & 3 == 3;
    const y_addr: u32 = state.e[(w4 >> 5) & 0xF];
    const y_val: u32 = state.s[(w4 >> 13) & 0xF] & 0xFFFF;
    const x_addr: u32 = state.a[(w2 >> 18) & 0xF];
    const x_src: u32 = (w2 >> 10) & 0xF;
    const x_val: u32 = if (x01 and x_src == 0) (state.xd << 8) & M24 else state.s[x_src] & M24;

    const alu_dest: u32 = (w6 >> 4) & 0xF;
    const alu_val: u32 = if (alu_dest == 0) 0 else blk: {
        if ((w4 >> 19) & 1 == 1) return RunError.UnknownForm;
        const word: u32 = (((h >> 5) & 0x3F) << 5) | (((w2 >> 6) & 0xF) << 19);
        break :blk (try aluWord(word, 0, sx24(state.s[(w2 >> 2) & 0xF]))) orelse return RunError.UnknownForm;
    };

    const mode: u32 = (w1 >> 11) & 7;
    const xs: i64 = sx24(state.s[(w2 >> 10) & 0xF]);
    const pa: i64 = sx16(state.s[(w6 >> 24) & 0xF]);
    const pb: i64 = sx16(state.s[(w6 >> 28) & 0xF]);
    const mac_dest: u32 = (w6 >> 8) & 0xF;
    const mac_val: u32 = if (mac_dest == 0) 0 else blk: {
        if ((w4 >> 23) & 1 == 1) {
            if ((w4 >> 17) & 1 == 0) return RunError.UnknownForm;
            break :blk state.xd & 0xFFFF;
        }
        if (mode == 0) break :blk 0;
        const xl: u32 = (w4 >> 13) & 7;
        if (xl == 0) return RunError.UnknownForm;
        const x40: i64 = (xs << 24) + sx24(state.s[xl]);
        const p2: i64 = 2 * pa * pb;
        const acc: i64 = switch (mode) {
            3 => x40,
            5 => x40 + p2,
            6 => p2 - x40,
            7 => x40 - p2,
            else => return RunError.UnknownForm,
        };
        break :blk @truncate(@as(u64, @bitCast(acc)) & 0xFFFF);
    };
    const mpy_dest: u32 = (w6 >> 16) & 0xF;
    const mpy_val: u32 = if (mpy_dest == 0) 0 else blk: {
        if ((w1 >> 14) & 1 == 0) return RunError.UnknownForm;
        const sh: u6 = if ((w1 >> 17) & 1 == 1) 24 else 23;
        const bias: i64 = if ((w1 >> 15) & 1 == 1) @as(i64, 1) << (sh - 1) else 0;
        const p: i64 = (pa * pb + bias) >> sh;
        const r: i64 = switch (mode) {
            3 => xs,
            5 => xs + p,
            6 => p - xs,
            7 => xs - p,
            else => return RunError.UnknownForm,
        };
        break :blk @truncate(@as(u64, @bitCast(r)) & M24);
    };

    const ctl: u32 = w3 & 0x1F;
    const target: u32 = (w1 >> 18) & 0xFF;
    switch (ctl) {
        1, 3, 0xE, 0x11, 0x13, 0x16, 0x17 => state.pending_jmp_offset = @intCast(target),
        4 => state.pending_jmp_offset = @as(i32, @intCast(target)) - @as(i32, @intCast(state.pc)),
        0, 2, 5, 0xC, 0x18, 0x10, 0x12, 0x14, 0x15, 0x19, 0x1A, 0x1C => {},
        else => return RunError.UnknownForm,
    }

    if (y_on) state.y_mem.put(gpa, y_addr, y_val) catch return RunError.OutOfMemory;
    if (x_on) state.x_mem.put(gpa, x_addr, x_val) catch return RunError.OutOfMemory;
    if (s_dest != 0) state.s[s_dest] = s_val;
    if (alu_dest != 0) state.s[alu_dest] = alu_val;
    if (mac_dest != 0) state.s[mac_dest] = mac_val;
    if (mpy_dest != 0) state.s[mpy_dest] = mpy_val;
}

const AGU14_F = [16]?u8{ 0, 1, 2, null, 4, 5, 6, 7, 9, 9, 1, 1, 1, 6, 1, 1 };

/// yagu, low5 0x14 with a class-100 second and S[25] (fact 100, 103):
/// v = AGU14[S[20:17]](x = e<S[24:21]>, y = R<S[16:13]>); e<S[24:21]> and
/// R<S[3:0]> <- v; s<S[7:4]> <- head[30] = 1: Y:[x] (and yd), else yd;
/// head target s<head[24:21]> <- S[28:27] = 0: 0, 1-2: v.
fn execYagu(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const sw = bundle[1];
    const k: u32 = (sw >> 21) & 0xF;
    const x: u32 = state.e[k];
    const fn_ag = AGU14_F[(sw >> 17) & 0xF] orelse return RunError.UnknownForm;
    const v = try agu10Eval(fn_ag, x, state.r[(sw >> 13) & 0xF]);
    const s_dest: u32 = (sw >> 4) & 0xF;
    if (s_dest != 0) {
        if ((h >> 30) & 1 == 1) state.yd = (state.getY(x) orelse 0) & 0xFFFF;
        state.s[s_dest] = state.yd;
    }
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) {
        const m: u32 = (sw >> 27) & 3;
        if (m == 3) return RunError.UnknownForm;
        state.s[hd] = if (m == 0) 0 else v;
    }
    state.e[k] = v;
    if (sw & 0xF != 0) state.r[sw & 0xF] = v;
}

/// ypair, low5 0x14 with a class-11x second and S[25] (fact 105):
/// s<2*S[2:0]> <- S[28:27] from class 110: 0 / e<head[8:5]> / e / 0xB600,
/// class 111: 0x5B00 / yd / #head[20:5] / 0.
fn execYpair(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const sw = bundle[1];
    const dest: u32 = 2 * (sw & 7);
    if (dest == 0) return;
    const m: u32 = (sw >> 27) & 3;
    state.s[dest] = if (sw >> 29 == 0b110)
        (switch (m) {
            0 => 0,
            3 => 0xB600,
            else => state.e[(h >> 5) & 0xF],
        })
    else
        (switch (m) {
            0 => 0x5B00,
            1 => state.yd,
            2 => (h >> 5) & 0xFFFF,
            else => 0,
        });
}

/// ystm, low5 0x10 with a class-010 second (fact 109, 110): Y:[e<head[8:5]>]
/// <- s<S[15:12]> when S[17:16] = 11; v = head[26] = 1: AGU10[head[20:17]](x =
/// e<head[8:5]>, y = R<head[16:13]>, i = {S[11:0], head[31:28]}), else yd;
/// e<head[12:9]> and R<S[28:25]> <- v.  S[18] = 0 wedges.
fn execYstm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const sw = bundle[1];
    if ((sw >> 18) & 1 == 0) return RunError.UnknownForm;
    const x: u32 = state.e[(h >> 5) & 0xF];
    const y: u32 = state.r[(h >> 13) & 0xF];
    if ((sw >> 16) & 3 == 3) {
        state.y_mem.put(gpa, x, state.s[(sw >> 12) & 0xF] & 0xFFFF) catch return RunError.OutOfMemory;
    }
    const v: u32 = if ((h >> 26) & 1 == 0) state.yd else blk: {
        const i: u32 = ((sw & 0xFFF) << 4) | (h >> 28);
        break :blk switch ((h >> 17) & 0xF) {
            8 => i & 0xFFFF,
            10 => (y +% i) & 0xFFFF,
            11 => (y -% i) & 0xFFFF,
            else => |f| try agu10Eval(f, x, y),
        };
    };
    if ((h >> 9) & 0xF != 0) state.e[(h >> 9) & 0xF] = v;
    if ((sw >> 25) & 0xF != 0) state.r[(sw >> 25) & 0xF] = v;
}

/// tst (fact 84, 85): p<S[28:25]> <- f(head[29:26], head[25:21]); f is
/// measured on hk 8 and on sel 0 / 1 only (what it compares is OPEN).
fn execTst(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const hk: u32 = (h >> 26) & 0xF;
    const sel: u32 = (h >> 21) & 0x1F;
    const v: u8 = if (hk == 8)
        (if (sel == 0 or sel == 6 or sel == 12 or sel == 16 or sel == 18 or sel == 24 or sel == 26 or sel == 29 or sel == 31) 1 else 0)
    else if (sel == 0)
        @intFromBool("TTTTnnnTTTTTTTTT"[hk] == 'T')
    else if (sel == 1)
        @intFromBool("nnnnnnnTnnTTTTTn"[hk] == 'T')
    else
        return RunError.UnknownForm;
    state.p[(bundle[1] >> 25) & 0xF] = v;
}

/// class-0A parallel move (fact 68, 69, 76): X:[A<[15:11]>] <- s<[19:16]>
/// with [21:20] = 11, and s<[30:26]> <- Y:[e<[8:5]>] (bit 9 set; without it
/// the address lags, OPEN).  The store reads the old s.
fn execPar(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const k: u32 = (head >> 26) & 0x1F;
    // Without bit 9 the load's address lags (OPEN): only an s0 target is let through.
    if ((head >> 9) & 1 == 0 and k != 16) return RunError.UnknownForm;
    const xv: u32 = state.s[(head >> 16) & 0xF] & M24;
    state.x_mem.put(gpa, state.a[(head >> 11) & 0x1F], xv) catch return RunError.OutOfMemory;
    if ((head >> 9) & 1 == 0) return;
    if (k < 16) return RunError.UnknownForm;
    state.s[k - 16] = (state.getY(state.e[(head >> 5) & 0xF]) orelse 0) & 0xFFFF;
}

/// mul13, low5 0x13, S class 101 with S[19] (fact 128):
/// s<S[14:11]> <- round(s<S[6:3]> * s<2*S[2:0] + head[31]> / 2^23).
fn execMul13(state: *State, bundle: []const u32) RunError!void {
    const sw = bundle[1];
    // S[18:17] (+-X), S[23:22], the side writes S[10:7] and head clears are unmodelled.
    if ((sw >> 17) & 3 != 0 or (sw >> 22) & 3 != 0 or (sw >> 7) & 0xF != 0 or (bundle[0] >> 15) & 0xF != 0) return RunError.UnknownForm;
    const b_idx: u32 = 2 * (sw & 7) + (bundle[0] >> 31);
    const prod: i64 = sx16(state.s[(sw >> 3) & 0xF]) * sx16(state.s[b_idx]);
    const dest: u32 = (sw >> 11) & 0xF;
    if (dest != 0) state.s[dest] = @truncate(@as(u64, @bitCast((prod + (1 << 22)) >> 23)) & M24);
}

fn execX10Alu(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const dest: u32 = (bundle[1] >> 21) & 0xF;
    if ((h >> 5) & 0x3F == 0x3F and (h >> 23) & 7 == 2) {
        if (dest != 0) state.r[dest] = shiftM2((h >> 19) & 0xF, state.s[(h >> 11) & 0xF] & 0xFFFF, state.s[(h >> 26) & 0xF] & 0xFFFF);
        return;
    }
    const v = (try aluWord(h, sx24(state.s[(h >> 11) & 0xF]), sx24(state.s[(h >> 26) & 0xF]))) orelse return RunError.UnknownForm;
    if (dest != 0) state.r[dest] = v & 0xFFFF;
}

/// Code 0x3F under m2 (fact 269): #[22:21] = 00 is b, 01 is b << a, 1x
/// is b >> a; a left shift past the width gives 0, or 0xFFFF when #[20] = 1.
fn shiftM2(n: u32, a: u32, b: u32) u32 {
    return switch ((n >> 2) & 3) {
        0 => b,
        1 => if (a >= 16) (if ((n >> 1) & 1 == 1) 0xFFFF else 0) else (b << @intCast(a)) & 0xFFFF,
        else => if (a >= 16) 0 else b >> @intCast(a),
    };
}

fn execX03Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const fn_op: u32 = (w2 >> 22) & 0x1F;
    const dest_a: u32 = (w3 >> 10) & 0x1F;
    const x_val: u32 = state.a[w3 & 0xF];
    const i_val: u32 = ((w5 & 1) << 15) | ((w4 >> 17) & 0x7FFF);
    // Second A dest (fact 193): A<w2[21:17]> <- A<w3[4:0]>, or with
    // w5[5] = 1 the Y read Y:[e<w3[30:27]>].
    const acp_dest: u32 = (w2 >> 17) & 0x1F;
    const acp_val: u32 = if ((w5 >> 5) & 1 == 1) (state.getY(state.e[(w3 >> 27) & 0xF]) orelse 0) & 0xFFFF else x_val;
    // AGU table "5" = x + i (fact 164).
    if (dest_a != 0) {
        if (fn_op != 5) return RunError.UnknownForm;
        state.a[dest_a & 0xF] = (x_val +% i_val) & M24;
    }
    if (acp_dest != 0 and acp_dest != dest_a) state.a[acp_dest & 0xF] = acp_val;
}

fn execX03Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const head = bundle[0];
    const w1 = bundle[1];
    const w4 = bundle[4];
    const dest_s: u32 = (w4 >> 3) & 0xF;
    const a_hi: u32 = w1 & 0x7;
    const a_lo: u32 = (head >> 30) & 0x3;
    const a_idx: u32 = ((a_hi << 2) | a_lo) & 0xF;
    // w5[21] = 1 turns the w4 slot into a Y read at e<w3[30:27]>
    // (fact 190 / own-x03yrd).
    const v: u32 = if ((bundle[5] >> 21) & 1 == 1) blk: {
        const src_e: u32 = (bundle[3] >> 27) & 0xF;
        break :blk state.getY(state.e[src_e]) orelse 0;
    } else state.readX(state.a[a_idx]) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX02X3Pre(gpa: std.mem.Allocator, state: *State, bundle: []const u32, s_pre: [16]u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return;
    const w4 = bundle[4];
    const w5 = bundle[5];
    const dest_lo: u32 = w5 & 0x3;
    const dest_hi: u32 = (w4 >> 30) & 0x3;
    const dest_s: u32 = (dest_lo << 2) | dest_hi;
    if (dest_s == 0) return;
    const op: u32 = (w5 >> 6) & 0x7;
    const a_s: u32 = (w4 >> 22) & 0xF;
    const b_s: u32 = (w4 >> 26) & 0xF;
    const shift: u5 = @intCast((w5 >> 2) & 0xF);
    const av: u32 = if (a_s == 0) 0 else s_pre[a_s] & M24;
    const bv: u32 = if (b_s == 0) 0 else s_pre[b_s] & M24;
    const base: u32 = switch (op) {
        0 => (av +% bv) & M24,
        5 => (av -% bv) & M24,
        else => return,
    };
    state.s[dest_s] = (base << shift) & M24;
}

fn execX02Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w1 = bundle[1];
    const w3 = bundle[3];
    const w5 = bundle[5];
    const fn_op: u32 = (w1 >> 7) & 0xF;
    const dest_r: u32 = (w3 >> 19) & 0xF;
    const y_r: u32 = (w5 >> 22) & 0xF;
    if (dest_r == 0) return;
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    const cin: u32 = (w5 >> 27) & 1;
    const v: u32 = switch (fn_op) {
        10 => (y_val +% cin) & 0xFFFF,
        14 => (y_val +% 1 +% cin) & 0xFFFF,
        else => return,
    };
    state.r[dest_r] = v;
}

fn execX02Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const dest_s: u32 = (w4 >> 3) & 0xF;
    // Y-slot bit: w5[12] = 1 selects Y:[e<w3[30:27]>] instead of X:[A<w4[21:17]>].
    const is_y: bool = ((w5 >> 12) & 1) == 1;
    // Mux bits at w4[13:11]: 101 = X-read (default), 010 = scalar e (esrc).
    const mux: u32 = (w4 >> 11) & 0x7;
    const v: u32 = if (is_y) blk: {
        const src_e: u32 = (w3 >> 27) & 0xF;
        const addr: u32 = state.e[src_e];
        break :blk state.getY(addr) orelse 0;
    } else if (mux == 0b010) blk: {
        // w5[29:28] selects the mux-010 subsource.
        const sub: u32 = (w5 >> 28) & 0x3;
        if (sub == 0b01) {
            // asrc: s <- A<w3[4:0]> + 1
            const src_a: u32 = w3 & 0x1F;
            break :blk ((if (src_a == 0) 0 else state.a[src_a]) +% 1) & M24;
        }
        const src_e: u32 = (w3 >> 27) & 0xF;
        break :blk if (src_e == 0) 0 else state.e[src_e];
    } else blk: {
        const src_a: u32 = (w4 >> 17) & 0xF;
        const addr: u32 = state.a[src_a];
        break :blk state.readX(addr) orelse 0;
    };
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

fn execX12Aset(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s = bundle[1];
    const dest_a: u32 = 2 * (s & 0x7);
    const src_r: u32 = (head >> 13) & 0xF;
    if (dest_a == 0) return;
    state.a[dest_a & 0xF] = if (src_r == 0) 0 else state.r[src_r];
}

fn execX12Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_e: u32 = (head >> 9) & 0xF;
    const x_e: u32 = (head >> 5) & 0xF;
    const y_r: u32 = (head >> 13) & 0xF;
    const fn_op: u32 = (head >> 17) & 0xF;
    const s_hi: u32 = (head >> 31) & 1;
    const s_low: u32 = s1 & 0x7;
    const dest_r: u32 = (s1 >> 21) & 0xF;
    const s_dest: u32 = (2 * s_low + s_hi) & 0xF;
    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    const v = try agu10Eval(fn_op, x_val, y_val);
    if (dest_e != 0) state.e[dest_e] = v;
    if (s_dest != 0) state.s[s_dest] = v;
    if (dest_r != 0) state.r[dest_r] = v;
}

fn execBcc(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const s1 = bundle[1];
    // S[8:0] signed 9-bit, scaled by 4; pred S[18:15]; S[14] = 1 wedges.
    const off9: u32 = s1 & 0x1FF;
    const off_signed: i32 = if (off9 & 0x100 != 0)
        @as(i32, @bitCast(off9 | 0xFFFFFE00))
    else
        @as(i32, @intCast(off9));
    if (((s1 >> 14) & 1) != 0) return RunError.UnknownForm;
    const pred_idx: u32 = (s1 >> 15) & 0xF;
    // p0 is always true (fact 111 / own-bcc-fwd).
    if (pred_idx == 0 or state.p[pred_idx] != 0) state.pending_jmp_offset = off_signed * 4;
}

fn execBr(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    // offset [25:11]: unsigned 15-bit in syllables; pred [29:26]: 4-bit.
    const off: i32 = @intCast((head >> 11) & 0x7FFF);
    const pred_idx: u32 = (head >> 26) & 0xF;
    if (state.p[pred_idx] != 0) state.pending_jmp_offset = off;
}

fn execJmp(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    // offset [20:5]: signed 16-bit in syllables.
    const off_bits: u32 = (head >> 5) & 0xFFFF;
    const off_signed: i32 = if (off_bits & 0x8000 != 0)
        @as(i32, @bitCast(off_bits | 0xFFFF0000))
    else
        @as(i32, @intCast(off_bits));
    const mode: u32 = (head >> 21) & 0x1F;
    // p<{[29], [28:26]}> (fact 218).
    const pred_idx: u32 = (head >> 26) & 0xF;
    const p_val: u32 = state.p[pred_idx];
    const taken: bool = switch (mode) {
        1 => true,
        2 => p_val == 0,
        3 => p_val != 0,
        else => return RunError.UnknownForm,
    };
    if (taken) state.pending_jmp_offset = off_signed;
}

fn execX15Pt(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const lut: u32 = (s >> 8) & 0x7;
    const p_idx: u32 = (s >> 3) & 0xF;
    if (dest_s == 0) return;
    const p_val: u8 = if (p_idx == 0) 0 else state.p[p_idx];
    const v: u32 = switch (lut) {
        1 => p_val, // identity
        0 => @as(u32, 1) ^ p_val, // not
        else => return,
    };
    state.s[dest_s] = v;
}

fn execX15Alu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const code: u32 = (head >> 5) & 0x3F;
    const a_s: u32 = (head >> 11) & 0xF;
    const dest_s: u32 = (head >> 15) & 0xF;
    const b_s: u32 = (head >> 26) & 0xF;
    const mux: u32 = (s1 >> 22) & 0x7;
    const dest_s2: u32 = (s1 >> 7) & 0xF;
    const a_val: u32 = state.s[a_s] & M24;
    const b_val: u32 = state.s[b_s] & M24;
    const v = try alu05Base(code, a_val, b_val);
    if (dest_s != 0) state.s[dest_s] = v;
    if (mux == 0b111 and dest_s2 != 0) state.s[dest_s2] = v;
}

/// ALU05 operation table, m = 0, s = 0 slice (fact 33 /
/// upstream/tools/sm1isa/emu.py).  Codes outside the measured set yield
/// UnknownForm so the gate harness flags them visibly.
fn alu05Base(code: u32, a: u32, b: u32) RunError!u32 {
    const am: u32 = a & M24;
    const bm: u32 = b & M24;
    return switch (code & 0x3F) {
        // add family
        0, 1, 2, 3, 4, 5, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x3C, 0x3D, 0x3F => (am +% bm) & M24,
        0x10, 0x11 => (am -% bm) & M24, // sub
        0x12, 0x13 => (bm -% am) & M24, // rsub
        0x14, 0x15 => (0 -% am -% bm -% 2) & M24, // nadd
        0x16, 0x17, 0x18, 0x19, 0x1D, 0x1E, 0x1F, 0x25 => (am ^ bm) & M24,
        0x1C => bm & M24, // xori (b is the immediate; m=0 passes through)
        0x20, 0x21, 0x22, 0x28, 0x29, 0x2E, 0x2F, 0x39, 0x3B => 0, // clr
        0x23, 0x24, 0x27, 0x2B => am, // mova
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 => bm, // movb
        // Shift ops: a is the shift count (signed 24-bit), b is the value.
        // asr/asrrs treat b as signed; lsr unsigned.  Negative count shifts
        // left.  fact 33, upstream/tools/sm1isa/emu.py `shift`.
        0x26, 0x2A, 0x2C, 0x2D => blk: {
            // sel: sign of (sx24(a) + sx24(b)) picks a (>=0) or b.
            const as: i32 = if (am & 0x800000 != 0)
                @bitCast(am | 0xFF000000)
            else
                @intCast(am);
            const bs: i32 = if (bm & 0x800000 != 0)
                @bitCast(bm | 0xFF000000)
            else
                @intCast(bm);
            break :blk if (as + bs >= 0) am else bm;
        },
        0x06, 0x1A => alu05Shift(am, bm, .asr),
        0x07 => alu05Shift(am, bm, .asrrs),
        0x08, 0x1B, 0x3E => alu05Shift(am, bm, .lsr),
        else => return RunError.UnknownForm,
    };
}

const ShiftKind = enum { asr, asrrs, lsr };

fn alu05Shift(a: u32, b: u32, kind: ShiftKind) u32 {
    const n: i32 = if (a & 0x800000 != 0)
        @as(i32, @bitCast(a | 0xFF000000))
    else
        @as(i32, @intCast(a));
    const v_signed: i32 = if (kind == .lsr)
        @as(i32, @bitCast(b & 0xFFFFFF))
    else if (b & 0x800000 != 0)
        @as(i32, @bitCast(b | 0xFF000000))
    else
        @as(i32, @intCast(b));
    if (n < 0) {
        const shift: u5 = @intCast(@min(-n, 24));
        const shifted: i64 = @as(i64, v_signed) << shift;
        if (kind == .asrrs) {
            const sat: i64 = @max(-0x800000, @min(0x7FFFFF, shifted));
            return @as(u32, @bitCast(@as(i32, @intCast(sat)))) & M24;
        }
        return @as(u32, @bitCast(@as(i32, @truncate(shifted)))) & M24;
    }
    var vv: i32 = v_signed;
    if (kind == .asrrs and n > 0) vv += @as(i32, 1) << @as(u5, @intCast(n - 1));
    const shift: u5 = @intCast(@min(n, 24));
    const shifted: i32 = vv >> shift;
    return @as(u32, @bitCast(shifted)) & M24;
}

fn execX16Hd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 3) return RunError.UnknownForm;
    const head = bundle[0];
    const s2 = bundle[2];
    const dest_s: u32 = (head >> 15) & 0xF;
    const a_s: u32 = (head >> 11) & 0xF;
    const b_s: u32 = (s2 >> 25) & 0xF;
    if (dest_s == 0) return;
    const av: u32 = if (a_s == 0) 0 else state.s[a_s];
    const bv: u32 = if (b_s == 0) 0 else state.s[b_s];
    // Full class-05 word with its [25:19] m/s modifiers (fact 210).
    const v = (try aluWord(head, sx24(av), sx24(bv))) orelse return RunError.UnknownForm;
    state.s[dest_s] = v;
}

fn execX16Mul2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 3) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const dest_s: u32 = (((s2) & 0x3) << 2) | ((s1 >> 30) & 0x3);
    const a_s: u32 = (s1 >> 1) & 0xF;
    const b_s: u32 = (s1 >> 26) & 0xF;
    const half: bool = (s1 & 1) == 1;
    // S2[3:2] = 11: X:[A<S2[8:4]>] <- s<S1[25:22]> (fact 209).
    if ((s2 >> 2) & 3 == 3) {
        try state.x_mem.put(gpa, state.a[(s2 >> 4) & 0xF], state.s[(s1 >> 22) & 0xF] & M24);
    }
    if (dest_s == 0) return;
    // head[28] = 0: the multiply slot writes 0 (fact 121).
    if ((head >> 28) & 1 == 0) {
        state.s[dest_s] = 0;
        return;
    }
    const av: u32 = if (a_s == 0) 0 else state.s[a_s] & 0xFFFF;
    const bv: u32 = if (b_s == 0) 0 else state.s[b_s] & 0xFFFF;
    const prod: u32 = av *% bv;
    const round_bias: u32 = if ((head >> 30) & 1 == 1) @as(u32, 1) << 22 else 0;
    const shifted: u32 = (prod +% round_bias) >> 23;
    const v: u32 = if (half) shifted >> 1 else shifted;
    state.s[dest_s] = v & M24;
}

/// S1 class != 000 with S2 class 101: head[28] runs the multiply,
/// s<4*S2[1:0] + S1[31:30]> <- s<S1[4:1]> * s<S2[5:2]> >> 23 (fact 212).
fn execX16AguMul(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const dest: u32 = ((s2 & 3) << 2) | ((s1 >> 30) & 3);
    if (dest == 0) return;
    if ((h >> 28) & 1 == 0) {
        state.s[dest] = 0;
        return;
    }
    const a: i64 = sx16(state.s[(s1 >> 1) & 0xF]);
    const b: i64 = sx16(state.s[(s2 >> 2) & 0xF]);
    state.s[dest] = @truncate(@as(u64, @bitCast((a * b) >> 23)) & 0xFFFF);
}

const AGU10B_TAB = [16]u8{ 0, 1, 2, 3, 4, 5, 6, 7, 2, 2, 1, 1, 1, 8, 9, 10 };

/// AGU10B (fact 120, S2[8] = 0): x, y, 0, -x, -y, 1, x+y, x-y, 0, 0,
/// y, y, y, -y, y+1, y-1.
fn agu10bEval(fn_op: u32, x: u32, y: u32) u32 {
    const r: u32 = switch (AGU10B_TAB[fn_op & 0xF]) {
        0 => x,
        1 => y,
        2 => 0,
        3 => 0 -% x,
        4 => 0 -% y,
        5 => 1,
        6 => x +% y,
        7 => x -% y,
        8 => 0 -% y,
        9 => y +% 1,
        else => y -% 1,
    };
    return r & 0xFFFF;
}

/// S1 class 000, S2 class 101 (fact 120): mux S1[17:15] = 001 runs
/// (S2[8] ? AGU10 : AGU10B)[S1[29:26]](e<S2[24:21]>, R<S1[21:18]>) into
/// s<S1[12:9]>; mux 000 writes 0.
fn execX16Agu2(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    const mux: u32 = (s1 >> 15) & 7;
    if ((s1 >> 13) & 1 == 0 or (s1 >> 30) & 1 == 1 or s2 & 3 != 0) return RunError.UnknownForm;
    const fn_op: u32 = (s1 >> 26) & 0xF;
    const x: u32 = state.e[(s2 >> 21) & 0xF];
    const y: u32 = state.r[(s1 >> 18) & 0xF];
    const v: u32 = switch (mux) {
        0 => 0,
        1 => if ((s2 >> 8) & 1 == 1) try agu10Eval(fn_op, x, y) else agu10bEval(fn_op, x, y),
        else => return RunError.UnknownForm,
    };
    const dest: u32 = (s1 >> 9) & 0xF;
    if (dest != 0) state.s[dest] = v;
}

/// S1 class 011, S2 class 000: S2[3] latches xi <- S2[8:4], then
/// s<S1[31:28]> <- X:[A<xi>][15:0] (fact 216).
fn execX16Xld(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 3) & 1 == 1) state.x16_xi = (s2 >> 4) & 0x1F;
    const xi = state.x16_xi orelse return RunError.UnknownForm;
    const dest: u32 = (s1 >> 28) & 0xF;
    const addr: u32 = if (xi == 0) 0 else state.a[xi & 0xF];
    if (dest != 0) state.s[dest] = (state.readX(addr) orelse 0) & 0xFFFF;
}

/// S1 class 011, S2 class 111: X:[A<S2[12:8]>] <- s<S1[31:28]> when
/// S2[14:13] = 11; Y:[e<S2[24:21]>] <- s<S2[3:0]> when S2[16] and S2[18]
/// (fact 214).
fn execX16St2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    const xv: u32 = state.s[(s1 >> 28) & 0xF] & M24;
    const yv: u32 = state.s[s2 & 0xF] & 0xFFFF;
    if ((s2 >> 13) & 3 == 3) try state.x_mem.put(gpa, state.a[(s2 >> 8) & 0xF], xv);
    if ((s2 >> 16) & 1 == 1 and (s2 >> 18) & 1 == 1) {
        state.y_mem.put(gpa, state.e[(s2 >> 21) & 0xF], yv) catch return RunError.OutOfMemory;
    }
}

/// S1 class 000, S2 class 010 (fact 119, 208): the X16 ALU, plus x18's
/// X slot into s<head[24:21]>: mux S1[14:12] = 101 with S2[12:11] = 10
/// reads Y:[e<S2[24:21]>]; 001 takes #head[20:5].
fn execX16Xs(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    try x16Alu(state, s1, s2);
    const dest: u32 = (h >> 21) & 0xF;
    if (dest == 0) return;
    const v: u32 = switch ((s1 >> 12) & 7) {
        5 => if ((s2 >> 11) & 3 == 2)
            (state.getY(state.e[(s2 >> 21) & 0xF]) orelse 0) & 0xFFFF
        else
            return RunError.UnknownForm,
        1 => (h >> 5) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    state.s[dest] = v;
}

fn execX16Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const fn_op: u32 = (s1 >> 7) & 0xF;
    const dest_s: u32 = (s2 >> 3) & 0xF;
    // x = e<{S2[2:0], S1[31]}> (fact 207 / own-x16agu-x1).
    const x_e: u32 = ((2 * (s2 & 0x7)) | (s1 >> 31)) & 0xF;
    const y_r: u32 = (head >> 27) & 0xF;
    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    // S1[30:28] is a source mux (fact 207): 001 = AGU10, 101 = Y:[x],
    // 011 = Y:[x][7:0] << 8, 100 = (Y:[x] << 7)[15:0], 110 = head[26:11] << 6.
    const mux: u32 = (s1 >> 28) & 7;
    const yd: u32 = (state.getY(x_val) orelse 0) & 0xFFFF;
    const v: u32 = switch (mux) {
        1 => switch (fn_op) {
            10 => (y_val +% ((head >> 11) & 0xFFFF)) & 0xFFFF,
            8, 11 => return RunError.UnknownForm,
            else => try agu10Eval(fn_op, x_val, y_val),
        },
        5 => yd,
        3 => (yd & 0xFF) << 8,
        4 => (yd << 7) & 0xFFFF,
        6 => (((head >> 11) & 0xFFFF) << 6) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

/// op = S2[20:16]: op & 1 multiplies 16-bit signed, op & 4 rounds; even
/// ops give 0 (fact 118).
fn execX16Mul(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 3) return RunError.UnknownForm;
    const s2 = bundle[2];
    const op: u32 = (s2 >> 16) & 0x1F;
    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a: i64 = sx16(state.s[(s2 >> 1) & 0xF]);
    const b: i64 = sx16(state.s[(s2 >> 5) & 0xF]);
    const bias: i64 = if (op & 4 != 0) 1 << 22 else 0;
    const v: u32 = if (op & 1 != 0)
        @truncate(@as(u64, @bitCast((a * b + bias) >> 23)) & 0xFFFF)
    else
        0;
    if (dest_s != 0) state.s[dest_s] = v;
}

fn sx16(v: u32) i64 {
    const w: u32 = v & 0xFFFF;
    return if (w & 0x8000 != 0) @as(i64, w) - 0x10000 else @as(i64, w);
}

/// s<S1[11:8]> <- X16_ALU[S1[15]][S1[18:16]](s<S2[28:25]>, s<S1[7:4]>)
/// (fact 116, 119).  Ops 0-3 wedge silicon.
fn x16Alu(state: *State, s1: u32, s2: u32) RunError!void {
    const op: u32 = (s1 >> 16) & 7;
    if (op < 4) return RunError.UnknownForm;
    const a: u32 = state.s[(s2 >> 25) & 0xF];
    const b: u32 = state.s[(s1 >> 4) & 0xF];
    const v: u32 = if ((s1 >> 15) & 1 == 0) switch (op) {
        4 => a +% b,
        5 => a,
        6 => a -% 1,
        else => 0 -% a,
    } else switch (op) {
        4 => a -% b,
        5 => a +% 1,
        else => a,
    };
    const dest: u32 = (s1 >> 8) & 0xF;
    if (dest != 0) state.s[dest] = v & 0xFFFF;
}

/// S2 class 111: S2[13] latches xi <- S2[12:8]; source S1[14:12] = 101 with
/// S2[19] reads X:[A<xi>] into s<head[24:21]> (fact 116).
fn execX16AluX(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 13) & 1 == 1) state.x16_xi = (s2 >> 8) & 0x1F;
    try x16Alu(state, s1, s2);
    const src: u32 = (s1 >> 12) & 7;
    if (src != 5 or (s2 >> 19) & 1 == 0) return RunError.UnknownForm;
    const xi = state.x16_xi orelse return RunError.UnknownForm;
    if (xi == 0) return RunError.UnknownForm;
    const dest: u32 = (bundle[0] >> 21) & 0xF;
    if (dest != 0) state.s[dest] = (state.readX(state.a[xi & 0xF]) orelse 0) & 0xFFFF;
}

/// S1/S2 class 000 (fact 115, 122, 217): S2[3] latches xi <- S2[8:4];
/// source S1[14:12]: 101 = X:[A<xi>], 111 = 0, 110 with S1[26:25] = 11 =
/// #head[20:5].  head[31] with S1[3] = 0: A<2*S1[2:0]+1> <- head[20:5].
fn execX16Ld(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const imm_mux: bool = (s1 >> 25) & 3 == 3;
    if (s2 & 4 != 0 or (h >> 27) & 0xF != 0) return RunError.UnknownForm;
    if ((s2 >> 3) & 1 == 1) state.x16_xi = (s2 >> 4) & 0x1F;
    if (!imm_mux) try x16Alu(state, s1, s2);
    const dest: u32 = (h >> 21) & 0xF;
    if (h >> 31 == 1 and (s1 >> 3) & 1 == 0) state.a[(2 * (s1 & 7) + 1) & 0xF] = (h >> 5) & 0xFFFF;
    if (dest == 0) return;
    const src: u32 = (s1 >> 12) & 7;
    const v: u32 = switch (src) {
        5 => blk: {
            const xi = state.x16_xi orelse return RunError.UnknownForm;
            const addr: u32 = if (xi == 0) 0 else state.a[xi & 0xF];
            break :blk (state.readX(addr) orelse 0) & 0xFFFF;
        },
        7 => 0,
        6 => if (imm_mux) (h >> 5) & 0xFFFF else return RunError.UnknownForm,
        // S1[26:25] = 11 with mux 011: X:[A<xi>][7:0] << 8 (fact 217).
        3 => if (imm_mux) blk: {
            const xi = state.x16_xi orelse return RunError.UnknownForm;
            const addr: u32 = if (xi == 0) 0 else state.a[xi & 0xF];
            break :blk ((state.readX(addr) orelse 0) & 0xFF) << 8;
        } else return RunError.UnknownForm,
        else => return RunError.UnknownForm,
    };
    state.s[dest] = v;
}

fn execX13St(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const dest_s: u32 = (head >> 15) & 0xF;
    const a_s: u32 = (head >> 19) & 0xF;
    // Only the sub (code 0x10) with m0 is modelled: s_dest <- 0 - s<a_s>.
    const code: u32 = (head >> 5) & 0x3F;
    if (code != 0x10) return RunError.UnknownForm;
    const a_val: u32 = state.s[a_s] & M24;
    const v: u32 = (0 -% a_val) & M24;
    if (dest_s != 0) state.s[dest_s] = v;
    // The X store side of this form is not modelled (needs address units).
}

fn execX13Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_e: u32 = (s1 >> 9) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    const dest_r: u32 = (s1 >> 25) & 0xF;
    const x_e: u32 = (s1 >> 21) & 0xF;
    const y_r: u32 = (s1 >> 13) & 0xF;
    const fn_op: u32 = (s1 >> 17) & 0xF;
    if (dest_e != 0) state.e[dest_e] = imm;
    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    const v = try agu10Eval(fn_op, x_val, y_val);
    if (dest_r != 0) state.r[dest_r] = v;
}

fn execX18Mpy(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const a_s: u32 = (w2 >> 1) & 0xF;
    const b_s: u32 = (w2 >> 5) & 0xF;
    const lo_dest: u32 = (w2 >> 24) & 0xF;
    const hi_dest: u32 = (w2 >> 9) & 0xF;
    const av: u64 = state.s[a_s] & M24;
    const bv: u64 = state.s[b_s] & M24;
    const prod: u64 = av * bv;
    // Integer mode (w3[24] = 1): low half is P & 0xFFFF per fact 167;
    // the measured 0x4001 * 0x6003 = 0x18012003 shows low = 0x2003.  High
    // half is P >> 24.
    const lo: u32 = @truncate(prod & 0xFFFF);
    const hi: u32 = @truncate((prod >> 24) & M24);
    if (lo_dest != 0) state.s[lo_dest] = lo;
    if (hi_dest != 0) state.s[hi_dest] = hi;
}

fn execX18Q23(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const mode: u32 = (w2 >> 28) & 0x7;
    const dest_s: u32 = (w2 >> 12) & 0xF;
    const a_s: u32 = (w2 >> 16) & 0xF;
    const b_s: u32 = (w2 >> 20) & 0xF;
    const av: u64 = state.s[a_s] & M24;
    const bv: u64 = state.s[b_s] & M24;
    const prod: u64 = av * bv;
    const v: u32 = switch (mode) {
        4 => @truncate(((prod + (1 << 22)) >> 23) & M24), // P: rounded
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX18Rimm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const e_hi: u32 = w2 & 1;
    const e_lo: u32 = (w1 >> 29) & 0x7;
    const dest_e: u32 = ((e_hi << 3) | e_lo) & 0xF;
    const op: u32 = (w2 >> 25) & 0x7;
    const imm: u32 = (w2 >> 8) & 0xFFFF;
    const r_src: u32 = (w3 >> 11) & 0xF;
    const r_val: u32 = if (r_src == 0) 0 else state.r[r_src];
    const v: u32 = switch (op) {
        0 => imm,
        2 => (r_val +% imm) & 0xFFFF, // R + i
        else => return RunError.UnknownForm,
    };
    if (dest_e != 0) state.e[dest_e] = v;
}

fn execX18Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w2 = bundle[2];
    const dest_s: u32 = (head >> 21) & 0xF;
    const src_a: u32 = (w2 >> 17) & 0x1F;
    const a_val: u32 = if (src_a == 0) 0 else state.a[src_a & 0xF];
    const v: u32 = state.readX(a_val) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// The x18 class 011/111 ALU slot (fact 168, 206): op w1[17:15] over
/// a = s<w2[5:2]>, b = s<w1[7:4]>.  Op 0 is only measured as pass (179).
fn x18SlotAlu(op: u32, a: u32, b: u32) RunError!u32 {
    return switch (op) {
        0, 2 => a & M24,
        1 => (a -% b) & M24,
        3 => (a +% 1) & M24,
        5 => (b -% a) & M24,
        7 => @intCast(@abs(sx16(a)) & M24),
        else => RunError.UnknownForm,
    };
}

/// Class 011 (fact 168, 179, 200, 206): the slot result goes to
/// s<w1[11:8]> and R<w3[10:7]>, and p<w2[17:14]> <- cond head[28:25] of it.
/// Only cond 11 (negative) is measured; other conds leave p alone.
/// w3[0] = 0 is the branch form (178), not modelled.
fn execX18C3(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    if (w3 & 1 == 0) return RunError.UnknownForm;
    const a: u32 = state.s[(w2 >> 2) & 0xF];
    const b: u32 = state.s[(w1 >> 4) & 0xF];
    const v = try x18SlotAlu((w1 >> 15) & 0x7, a, b);
    const dest_s: u32 = (w1 >> 8) & 0xF;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    if (dest_s != 0) state.s[dest_s] = v;
    if (dest_r != 0) state.r[dest_r] = v & 0xFFFF;
    if (((h >> 25) & 0xF) == 11) state.p[(w2 >> 14) & 0xF] = if (sx16(v) < 0) 1 else 0;
}

/// Class 111 (fact 174, 202, 203, 206).  40-bit MAC: P =
/// s<w2[27:24]> * s<w2[31:28]> << 1 (16-bit signed), X = s<w1[31:28]> in
/// bits 39:24 plus sx16 s<w2[3:0]> as bits 23:0; mode w3[6:4] combines them,
/// hi acc[39:24] -> s<w3[3:0]>, lo acc[15:0] -> s<w2[15:12]>.  ALU slot:
/// s<w1[11:8]> <- op w1[17:15] of a = s<w2[19:16]>, b = s<w1[7:4]>, then
/// shifted by w3[14:11] (head[27] = 1: right, rounding; 0: left).  Y store
/// (w2[22:21] = 11) of s<w2[3:0]> at Y:[e<w3[28:25]>].  Inputs latch first.
fn execX18C7(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const hi_dest: u32 = w3 & 0xF;
    const lo_dest: u32 = (w2 >> 12) & 0xF;
    const alu_dest: u32 = (w1 >> 8) & 0xF;
    const x_lo_raw: u32 = state.s[w2 & 0xF] & 0xFFFF;
    const st_val: u32 = x_lo_raw;
    var acc: i64 = 0;
    if (hi_dest != 0 or lo_dest != 0) {
        const p: i64 = sx16(state.s[(w2 >> 24) & 0xF]) * sx16(state.s[(w2 >> 28) & 0xF]) * 2;
        const x_lo: i64 = @as(i64, if (x_lo_raw & 0x8000 != 0) x_lo_raw | 0xFF0000 else x_lo_raw);
        const x: i64 = (@as(i64, state.s[(w1 >> 28) & 0xF] & 0xFFFF) << 24) + x_lo;
        acc = switch ((w3 >> 4) & 7) {
            3 => x,
            4 => p,
            5 => x + p,
            6 => p - x,
            7 => x - p,
            else => return RunError.UnknownForm,
        };
    }
    var alu_v: u32 = 0;
    if (alu_dest != 0) {
        const r = try x18SlotAlu((w1 >> 15) & 7, state.s[(w2 >> 16) & 0xF], state.s[(w1 >> 4) & 0xF]);
        const n: u5 = @intCast((w3 >> 11) & 0xF);
        alu_v = if ((h >> 27) & 1 == 1) blk: {
            const sr: i64 = sx24(r);
            const bias: i64 = if (n == 0) 0 else @as(i64, 1) << (n - 1);
            break :blk @truncate(@as(u64, @bitCast((sr + bias) >> n)) & 0xFFFF);
        } else (r << n) & 0xFFFF;
    }
    if ((w2 >> 21) & 3 == 3) {
        state.y_mem.put(gpa, state.e[(w3 >> 25) & 0xF], st_val) catch return RunError.OutOfMemory;
    }
    const acc_u: u64 = @bitCast(acc);
    if (hi_dest != 0) state.s[hi_dest] = @truncate((acc_u >> 24) & 0xFFFF);
    if (lo_dest != 0) state.s[lo_dest] = @truncate(acc_u & 0xFFFF);
    if (alu_dest != 0) state.s[alu_dest] = alu_v;
}

/// The class 010/101 R slot (fact 194, 195):
/// R<w3[10:7]> <- AGU3[w3[24:20]](x = A<w3[19:15]>, y = R<w3[14:11]>).
fn x18RSlot(state: *const State, w3: u32) u32 {
    return agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0xF], state.r[(w3 >> 11) & 0xF], 0);
}

/// Class 010 (fact 176, 194, 267): X slot s<head[24:21]> by mux
/// w1[14:12] (001 = e<w3[28:25]>, 110 = #head[20:5]); e AGU
/// e<w1[23:20]> <- AGU10[w1[27:24]](e<w3[28:25]>) when w1[29] = 1; R slot;
/// and the multiply when w2[30:28] = 4.  Inputs latch before writes.
fn execX18C2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const x_e_val: u32 = state.e[(w3 >> 25) & 0xF];
    const dest_s: u32 = (h >> 21) & 0xF;
    const dest_e: u32 = (w1 >> 20) & 0xF;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const xs_val: u32 = if (dest_s == 0) 0 else switch ((w1 >> 12) & 7) {
        1 => x_e_val,
        6 => (h >> 5) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    const e_val: u32 = if (dest_e == 0) 0 else blk: {
        if ((w1 >> 29) & 1 == 0) return RunError.UnknownForm;
        break :blk try agu10Eval((w1 >> 24) & 0xF, x_e_val, 0);
    };
    const r_val: u32 = x18RSlot(state, w3);
    if (((w2 >> 28) & 7) == 4) try execX18Q23(gpa, state, bundle);
    if (dest_s != 0) state.s[dest_s] = xs_val;
    if (dest_e != 0) state.e[dest_e] = e_val;
    if (dest_r != 0) state.r[dest_r] = r_val;
}

/// Class 101 (fact 195): the rimm e slot, class 010's R slot, and an
/// X slot s<head[24:21]> whose mux 001 is the R slot's value.
///
/// Also the ALU word slot (fact 237): s<w1[11:8]> <- ALU05 code
/// head[30:25], m w1[17:15], # w2[7:4] over a = s<w1[7:4]>, b = s<w3[6:3]>;
/// and the branch (201): w2[30:29] = 00 jumps sext(w1[28:18]) words when
/// p<{w3[2:0], w2[31]}> is set.  Other w2[30:29] values are measured only
/// as not branching.
fn execX18C5(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const r_val: u32 = x18RSlot(state, w3);
    const dest_s: u32 = (h >> 21) & 0xF;
    if (dest_s != 0 and ((w1 >> 12) & 7) != 1) return RunError.UnknownForm;
    const alu_dest: u32 = (w1 >> 8) & 0xF;
    const alu_val: ?u32 = if (alu_dest == 0) null else blk: {
        const word: u32 = (((h >> 25) & 0x3F) << 5) | (((w1 >> 15) & 7) << 23) | (((w2 >> 4) & 0xF) << 19);
        break :blk try aluWord(word, sx24(state.s[(w1 >> 4) & 0xF]), sx24(state.s[(w3 >> 3) & 0xF]));
    };
    try execX18Rimm(gpa, state, bundle);
    const dest_r: u32 = (w3 >> 7) & 0xF;
    if (dest_r != 0) state.r[dest_r] = r_val;
    if (dest_s != 0) state.s[dest_s] = r_val;
    if (alu_dest != 0) state.s[alu_dest] = alu_val orelse return RunError.UnknownForm;
    if ((w2 >> 29) & 3 == 0 and state.p[(((w3 & 7) << 1) | (w2 >> 31)) & 0xF] == 1) {
        const off11: u32 = (w1 >> 18) & 0x7FF;
        state.pending_jmp_offset = if (off11 & 0x400 != 0)
            @as(i32, @bitCast(off11 | 0xFFFFF800))
        else
            @as(i32, @intCast(off11));
    }
}

/// Class 000 (fact 169, 177, 180, 205, 266), inputs latched before writes:
/// one e AGU, fn w1[29:26], x = e<w3[28:25]>, y = e<{w2[1:0], w1[31:30]}>, w2[15]
/// picking the table; e<w1[25:22]> <- it; s<head[24:21]> and s<w1[21:18]> <- X mux
/// w1[17:15] (0 / - / the AGU / - / - / X:[A<w2[12:8]>] when w2[14:13] = 01, else
/// xd / - / 0); Y store (w2[7], w2[16]) of s<{w3[2:0], w2[31]}> at Y:[x]; second e
/// slot e<w2[5:2]> <- y - 1, where w2[17] = 0 reads y as 0 in phase T = 0 and
/// flips T (T survives runs, reset to 0).
fn execX18C0(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const x: u32 = state.e[(w3 >> 25) & 0xF];
    const y: u32 = state.e[((w2 & 3) << 2) | ((w1 >> 30) & 3)];
    const f: u32 = (w1 >> 26) & 0xF;
    const eop: u32 = (if ((w2 >> 15) & 1 == 1) switch (f) {
        0, 6, 7, 13 => x,
        2 => x +% 1,
        3 => 1,
        4 => x -% 1,
        5 => 0xFFFF,
        else => 0,
    } else switch (f) {
        0, 6, 7 => x,
        2 => y,
        3 => 0 -% x,
        5 => y +% 1,
        14 => 1,
        15 => y -% 1,
        else => 0,
    }) & 0xFFFF;
    const dest_e: u32 = (w1 >> 22) & 0xF;
    const dest_s: u32 = (h >> 21) & 0xF;
    const dest_s2: u32 = (w1 >> 18) & 0xF;
    const src: u32 = if (dest_s == 0 and dest_s2 == 0) 0 else switch ((w1 >> 15) & 7) {
        0, 7 => 0,
        2 => eop,
        5 => if ((w2 >> 13) & 3 == 1) state.readX(state.a[(w2 >> 8) & 0x1F]) orelse 0 else state.xd_pre,
        else => return RunError.UnknownForm,
    };
    const st_val: u32 = state.s[((w3 & 7) << 1) | (w2 >> 31)] & 0xFFFF;
    const dest_e2: u32 = (w2 >> 2) & 0xF;
    const y2: u32 = if ((w2 >> 17) & 1 == 0 and !state.x18_t) 0 else y;
    if ((w2 >> 17) & 1 == 0) state.x18_t = !state.x18_t;
    if ((w2 >> 7) & 1 == 1 and (w2 >> 16) & 1 == 1) {
        state.y_mem.put(gpa, x, st_val) catch return RunError.OutOfMemory;
    }
    if (dest_e != 0) state.e[dest_e] = eop;
    if (dest_s != 0) state.s[dest_s] = src;
    if (dest_s2 != 0) state.s[dest_s2] = src;
    if (dest_e2 != 0) state.e[dest_e2] = (y2 -% 1) & 0xFFFF;
}

/// Class 100 (fact 172, 196-198, 204), slots latched then written:
/// e slot e<w2[11:8]> <- e<w2[7:4]> -1 (w2[20] = 1) or +1; Y store
/// (w2[22:21] = 11) of s<w2[3:0]> at Y:[e<w3[28:25]>]; R slot
/// R<{w3[2:0], w2[31]}> <- AGU10[w1[10:7]](e<w3[28:25]>, R<w2[26:23]>) with
/// w2[20] = 1 (the other table is unmeasured); multiply s<w3[16:13]> <- mode
/// w1[13:11] = 4, round(s<w3[20:17]> * s<w3[24:21]> / 2^23); shift
/// s<w1[27:24]> <- s<w2[19:16]> by w1[23:20] (head[27] = 1: right,
/// rounding); X slot s<head[24:21]> <- Y:[e<w3[28:25]>][7:0] << 8 when
/// w2[22:21] = 01 and w3[12:10] = 011.
fn execX18C4(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const xe_val: u32 = state.e[(w3 >> 25) & 0xF];
    const ysel: u32 = (w2 >> 21) & 3;
    const st_val: u32 = state.s[w2 & 0xF] & 0xFFFF;
    const dest_e: u32 = (w2 >> 8) & 0xF;
    const src_e_val: u32 = state.e[(w2 >> 4) & 0xF];
    const e_val: u32 = if ((w2 >> 20) & 1 == 1) (src_e_val -% 1) & 0xFFFF else (src_e_val +% 1) & 0xFFFF;
    const dest_r: u32 = ((w3 & 7) << 1) | (w2 >> 31);
    const r_val: u32 = if (dest_r == 0) 0 else blk: {
        if ((w2 >> 20) & 1 == 0) return RunError.UnknownForm;
        break :blk try agu10Eval((w1 >> 7) & 0xF, xe_val, state.r[(w2 >> 23) & 0xF]);
    };
    const dest_m: u32 = (w3 >> 13) & 0xF;
    const m_val: u32 = if (dest_m == 0) 0 else blk: {
        if (((w1 >> 11) & 7) != 4) return RunError.UnknownForm;
        const prod: i64 = sx16(state.s[(w3 >> 17) & 0xF]) * sx16(state.s[(w3 >> 21) & 0xF]);
        break :blk @truncate(@as(u64, @bitCast((prod + (1 << 22)) >> 23)) & 0xFFFF);
    };
    const dest_sh: u32 = (w1 >> 24) & 0xF;
    const sh_val: u32 = if (dest_sh == 0) 0 else blk: {
        const src: u32 = state.s[(w2 >> 16) & 0xF];
        const n: u5 = @intCast((w1 >> 20) & 0xF);
        if ((h >> 27) & 1 == 1) {
            const bias: i64 = if (n == 0) 0 else @as(i64, 1) << (n - 1);
            break :blk @truncate(@as(u64, @bitCast((@as(i64, sx24(src)) + bias) >> n)) & 0xFFFF);
        }
        break :blk (src << n) & 0xFFFF;
    };
    const dest_x: u32 = (h >> 21) & 0xF;
    const x_val: u32 = if (dest_x == 0) 0 else blk: {
        if (ysel != 1 or ((w3 >> 10) & 7) != 3) return RunError.UnknownForm;
        break :blk ((state.getY(xe_val) orelse 0) & 0xFF) << 8;
    };
    if (ysel == 3) state.y_mem.put(gpa, xe_val, st_val) catch return RunError.OutOfMemory;
    if (dest_e != 0) state.e[dest_e] = e_val;
    if (dest_r != 0) state.r[dest_r] = r_val;
    if (dest_m != 0) state.s[dest_m] = m_val;
    if (dest_sh != 0) state.s[dest_sh] = sh_val;
    if (dest_x != 0) state.s[dest_x] = x_val;
}

fn execX18Br(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 4) return RunError.UnknownForm;
    const w1 = bundle[1];
    const w2 = bundle[2];
    const pred_idx: u32 = (w2 >> 28) & 0xF;
    // sext({w2[0], w1[31:18]}) in words (syllables) from the branch bundle
    // (fact 178: 00200000 = +8, FFE00000 with w2[0] = 1 = -8).
    const off15: u32 = ((w2 & 1) << 14) | ((w1 >> 18) & 0x3FFF);
    const sx: i32 = if (off15 & 0x4000 != 0)
        @as(i32, @bitCast(off15 | 0xFFFF8000))
    else
        @as(i32, @intCast(off15));
    if (pred_idx != 0 and state.p[pred_idx] == 1) {
        state.pending_jmp_offset = sx;
    }
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
    const v: u32 = state.readX(addr) orelse 0;
    if (dest_s_x != 0) state.s[dest_s_x] = v;
    if (dest_s2 != 0) state.s[dest_s2] = imm16;
}

fn execEalu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const s1 = bundle[1];
    const dest_e: u32 = (s1 >> 3) & 0xF;
    const src_s: u32 = (s1 >> 18) & 0xF;
    const op_bit: u32 = (s1 >> 11) & 1;
    const op: u32 = (s1 >> 12) & 0x7;
    const a_val: u32 = state.s[src_s] & M24;
    const v: u32 = switch (op_bit) {
        1 => switch (op) {
            5 => (a_val +% 1) & M24,
            else => return RunError.UnknownForm,
        },
        else => return RunError.UnknownForm,
    };
    if (dest_e != 0) state.e[dest_e] = v;
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
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const dest_s: u32 = (head >> 21) & 0x1F;
    const imm: u32 = (head >> 5) & 0xFFFF;
    // S[20:19] = 11 stores Y:[e<S[28:25]>] <- s<S[24:21]> (fact 89).
    const sw = bundle[1];
    if ((sw >> 19) & 3 == 3) {
        const v: u32 = state.s[(sw >> 21) & 0xF] & 0xFFFF;
        state.y_mem.put(gpa, state.e[(sw >> 25) & 0xF], v) catch return RunError.OutOfMemory;
    }
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
    if (w1_class == 0 and w2_class == 4 and w3_class == 2) {
        return execX19Aagu(gpa, state, bundle);
    }
    // w1 class 101 is inert on the A AGU (fact 271 / own-x19aagu5).
    if (w1_class == 5 and w2_class == 0 and w3_class == 2) {
        return execX19Aagu(gpa, state, bundle);
    }
    // w2 class 100 is inert on the imm form (fact 272 / own-x19imm4).
    if (w1_class == 0 and w2_class == 4 and w3_class == 1) {
        return execX19Imm(gpa, state, bundle);
    }
    // w2[31] = 1 makes R<w3[10:7]> <- AGUA(A<w3[19:15]>) (fact 278).
    if (w1_class == 0 and w2_class == 4 and w3_class == 0) {
        return execX19RAgua(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 1 and w3_class == 6) {
        return execX19Mul(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 1) {
        return execX19Imm(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 0) {
        return execX19Agu(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 4) {
        return execX19Alu(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 4 and w3_class == 4) {
        return execX19Alu(gpa, state, bundle);
    }
    if (w1_class == 2 and w2_class == 0 and w3_class == 0) {
        return execX19E(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 4 and w3_class == 3) {
        const src_sel: u32 = (bundle[1] >> 15) & 0x7;
        if (src_sel == 5) return execX19Xrd(gpa, state, bundle);
        if (src_sel == 1) return execX19A43(gpa, state, bundle);
    }
    // w2[9] = w2[1] = 1 with w3 class 3: Y store, any w1 class
    // (fact 257 / own-x19a3st).
    if (w2_class == 0 and w3_class == 3 and (bundle[2] & 0x202) == 0x202) {
        return execX19A3St(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 3) {
        return execX19A3(gpa, state, bundle);
    }
    if (w1_class == 3 and w2_class == 0 and w3_class == 0) {
        return execX19K00(gpa, state, bundle);
    }
    return RunError.UnknownForm;
}

fn execX19K00(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const w1 = bundle[1];
    const w3 = bundle[3];
    const dest_s: u32 = (head >> 21) & 0xF;
    const mux: u32 = (w1 >> 12) & 0x7;
    const w1_mod: u32 = (w1 >> 15) & 0x7;
    // The store source is read before the bundle's writes (fact 239).
    const store_val: u32 = state.s[(w1 >> 28) & 0xF] & M24;
    const v: u32 = switch (mux) {
        1, 2 => blk: {
            if (w1_mod == 5 or w1_mod == 7) {
                // own-x19kalu (m=5, a-b) and own-x19kabs (m=7, |b|).
                const a_s: u32 = (w1 >> 4) & 0xF;
                const b_s: u32 = (w3 >> 3) & 0xF;
                const a_val: u32 = state.s[a_s] & M24;
                const b_val: u32 = state.s[b_s] & M24;
                const dest_kalu: u32 = (w1 >> 8) & 0xF;
                const alu_v: u32 = if (w1_mod == 7) blk2: {
                    const low16: u32 = b_val & 0xFFFF;
                    const signed: i32 = if (low16 & 0x8000 != 0)
                        @bitCast(low16 | 0xFFFF0000)
                    else
                        @intCast(low16);
                    const abs_val: u32 = if (signed < 0) @intCast(-signed) else @intCast(signed);
                    break :blk2 abs_val & M24;
                } else (a_val -% b_val) & M24;
                if (dest_kalu != 0) state.s[dest_kalu] = alu_v;
                break :blk alu_v;
            }
            const fn_op: u32 = (w1 >> 24) & 0xF;
            const x_e: u32 = (w3 >> 25) & 0xF;
            const y_r: u32 = (w3 >> 11) & 0xF;
            const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
            const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
            break :blk try agu10Eval(fn_op, x_val, y_val);
        },
        6 => blk: {
            // mux 110: head[20:5] imm (own-x19kimm).
            break :blk (head >> 5) & 0xFFFF;
        },
        else => return RunError.UnknownForm,
    };
    if ((mux == 1 or mux == 2) and (w1_mod == 5 or w1_mod == 7)) return;
    if (dest_s != 0) state.s[dest_s] = v;
    // own-x19kst: the X store side.  Enabled by w2[23:22] = 11 (fact 239).
    // X:[A<w2[21:17]>] <- sx24 s<w1[31:28]>.
    const w2 = bundle[2];
    const store_en: u32 = (w2 >> 22) & 0x3;
    if (store_en == 0b11) {
        const a_idx: u32 = (w2 >> 17) & 0x1F;
        const src_s: u32 = (w1 >> 28) & 0xF;
        if (src_s != 0) try state.x_mem.put(gpa, state.a[a_idx & 0xF], store_val);
    }
}

/// Y:[e<w3[28:25]>] <- s<{w2[0], w1[31:29]}>.
/// Also runs the s slot: s<head[24:21]> <- AGU3[w3[24:20]](x = A<w3[19:15]>).
fn execX19A3St(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const src_s: u32 = ((bundle[2] & 1) << 3) | ((bundle[1] >> 29) & 0x7);
    const addr_e: u32 = (bundle[3] >> 25) & 0xF;
    const v: u32 = state.s[src_s] & 0xFFFF;
    const w3 = bundle[3];
    const dest_s: u32 = (bundle[0] >> 21) & 0xF;
    const s_val = agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0xF], 0, 2 * (w3 & 0x7FFF));
    state.y_mem.put(gpa, state.e[addr_e], v) catch return RunError.OutOfMemory;
    if (dest_s != 0) state.s[dest_s] = s_val;
}

fn execX19A3(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w1 = bundle[1];
    const w3 = bundle[3];
    // s<head[24:21]> <- AGU3[w3[24:20]](i = 2*w3[14:0]).  Only fn 25
    // measured, which returns i.
    const dest_s: u32 = (head >> 21) & 0xF;
    const agu3_fn: u32 = (w3 >> 20) & 0x1F;
    const i_raw: u32 = w3 & 0x7FFF;
    const s_val: u32 = switch (agu3_fn) {
        25 => (2 * i_raw) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = s_val;
    // R<w1[14:11]> <- AGU10[w1[10:7]](x = e<w3[28:25]>, y = 0)
    const dest_r: u32 = (w1 >> 11) & 0xF;
    const agu10_fn: u32 = (w1 >> 7) & 0xF;
    const x_e: u32 = (w3 >> 25) & 0xF;
    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const r_val = try agu10Eval(agu10_fn, x_val, 0);
    if (dest_r != 0) state.r[dest_r] = r_val;
}

fn execX19A43(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const dest_s: u32 = (head >> 21) & 0xF;
    const agu3_fn: u32 = (w3 >> 20) & 0x1F;
    const i_base: u32 = w3 & 0x7FFF;
    const w2_hi: u32 = (w2 >> 31) & 1;
    const v: u32 = switch (agu3_fn) {
        25 => ((i_base << 1) | w2_hi) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX19Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w3 = bundle[3];
    const dest_s: u32 = (head >> 21) & 0xF;
    const src_a: u32 = (w3 >> 15) & 0x1F;
    const a_val: u32 = if (src_a == 0) 0 else state.a[src_a & 0xF];
    const v: u32 = state.readX(a_val) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// 0x19 ALU op 23 (fact 147): b >= 0 gives sat24(a << b); b < 0 gives the rounded
/// a >> -b, (a + 2^(-b-1)) >> -b, half up on negatives.  MEASURED: own-x19alu and the
/// own-x19shr-* / own-x19shl-* gates (PROBES.md).
pub fn shift23(a: i64, b: i64) i64 {
    if (b >= 0) {
        const s: u6 = @intCast(@min(b, 40));
        return std.math.clamp(a << s, -0x800000, 0x7FFFFF);
    }
    const n: u6 = @intCast(@min(-b, 40));
    return (a + (@as(i64, 1) << (n - 1))) >> n;
}

fn execX19Alu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    // head[9:5] = 23 (shift op), a = s<w2[21:18]>, b = s<w1[19:16]>
    // dest = s<w1[27:24]>.  Only the sat24-shift op is measured; others OPEN.
    const code: u32 = (head >> 5) & 0x1F;
    const dest_s: u32 = (w1 >> 24) & 0xF;
    const a_s: u32 = (w2 >> 18) & 0xF;
    const b_s: u32 = (w1 >> 16) & 0xF;
    const a_val: i64 = sx24(state.s[a_s]);
    const b_val: i64 = sx24(state.s[b_s]);
    const v: u32 = switch (code) {
        23 => @truncate(@as(u64, @bitCast(shift23(a_val, b_val))) & M24),
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX19E(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    // e<w1[23:20]> <- head[20:5]; s<head[24:21]> <- source picked by w1[14:12].
    const e_imm: u32 = (head >> 5) & 0xFFFF;
    const dest_e: u32 = (w1 >> 20) & 0xF;
    const dest_s: u32 = (head >> 21) & 0xF;
    const mux: u32 = (w1 >> 12) & 0x7;
    if (dest_e != 0) state.e[dest_e] = e_imm;
    const v: u32 = switch (mux) {
        2 => blk: {
            const e_src: u32 = (w2 >> 4) & 0xF;
            break :blk state.e[e_src];
        },
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = v;
}

fn execX19Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w1 = bundle[1];
    const w3 = bundle[3];
    const fn_op: u32 = (w1 >> 24) & 0xF;
    const dest_e: u32 = (w1 >> 20) & 0xF;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const x_e: u32 = (w3 >> 25) & 0xF;
    const y_r: u32 = (w3 >> 11) & 0xF;
    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    const v = try agu10Eval(fn_op, x_val, y_val);
    // w2[24] = 1: the e destination takes #head[20:5] (fact 270).
    const e_val: u32 = if ((bundle[2] >> 24) & 1 == 1) (bundle[0] >> 5) & 0xFFFF else v;
    if (dest_e != 0) state.e[dest_e] = e_val;
    if (dest_r != 0) state.r[dest_r] = v;
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

    // fn 0 (x, fact 272) and fn 10 (y + i) are the measured cases.
    const v: u32 = switch (fn_op) {
        0 => state.e[x_src] & 0xFFFF,
        10 => ((if (y_src == 0) 0 else state.r[y_src]) +% imm) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    if (dest_e != 0) state.e[dest_e] = v;
}

fn execX19RAgua(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w3 = bundle[3];
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const x_val: u32 = state.a[(w3 >> 15) & 0xF];
    // Only fn 0 with w3[14:0] = 0 is measured: x + 1 (w2[31] is the class
    // bit here, not an i bit, per own-x19ragua's 0x346).
    if (((w3 >> 20) & 0x1F) != 0 or (w3 & 0x7F) != 0) return RunError.UnknownForm;
    if (dest_r != 0) state.r[dest_r] = (x_val +% 1) & 0xFFFF;
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

    const w2 = bundle[2];
    const dest_a: u32 = (w1 >> 3) & 0x1F;
    const src_a: u32 = (w3 >> 15) & 0x1F;
    const fn_op: u32 = (w3 >> 20) & 0x1F;
    // When w2 class is 100 (bit [31] set), i gains an extra high bit:
    //   i = (w3[14:0] << 1) | w2[31]
    const w2_bit31: u32 = (w2 >> 31) & 1;
    const imm_base: u32 = w3 & 0x7FFF;
    const imm_scaled: u32 = (imm_base << 1) | w2_bit31;

    const x_val: u32 = state.a[src_a & 0xF];
    const new_val: u32 = aguaEval(fn_op, x_val, imm_scaled) orelse return RunError.UnknownForm;
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

/// st (fact 21, 47, 65, 75): Y:[head[20:5]] <- the lane's V; a second
/// syllable with [16:15] = 11 is a wr, Y:[e<[20:17]>] <- reg[25:21] (16 reads 0),
/// its address read before st's own e1 <- [20:5].
fn execStY(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const addr: u32 = (head >> 5) & 0xFFFF;
    // sel [22:21] (fact 21-22): before the store, 1 sets V2 and 2 sets
    // V1 and V3 to the previous store's address; 3 does nothing.
    switch ((head >> 21) & 3) {
        1 => state.v[2] = state.e[1],
        2 => {
            state.v[1] = state.e[1];
            state.v[3] = state.e[1];
        },
        else => {},
    }
    const value: u32 = state.v[state.cur_lane & 0x3];
    state.y_mem.put(gpa, addr, value) catch return RunError.OutOfMemory;
    if (bundle.len >= 2 and bundle[1] != 0) {
        const w = bundle[1];
        if (w & 0xE0018000 != 0x20018000) {
            if (!(w >> 29 == 1 and (w >> 15) & 3 == 0)) return RunError.UnknownForm;
        } else {
            const src: u32 = (w >> 21) & 0x1F;
            if (src < 16) return RunError.UnknownForm;
            const v: u32 = if (src == 16) 0 else state.s[src - 16] & 0xFFFF;
            state.y_mem.put(gpa, state.e[(w >> 17) & 0xF], v) catch return RunError.OutOfMemory;
        }
    }
    state.e[1] = addr;
    state.e[0] = addr;
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

    // S2 class 101: S2[1] stores Y:[e<S1[16:15]>] <- reg<S1[28:24]> (fact
    // 22, 32, 47; [16:15] = 10 hits the drifting pointer a2).
    if (s2_class == 0b101) {
        if (s2 & 2 != 0) {
            const k: u32 = (s1 >> 24) & 0x1F;
            if (k < 16) return RunError.UnknownForm;
            const v: u32 = if (k == 16) 0 else state.s[k - 16] & 0xFFFF;
            state.y_mem.put(gpa, state.e[(s1 >> 15) & 3], v) catch return RunError.OutOfMemory;
        }
        return;
    }
    if (s1_class == 0b011 and s2_class == 0b111) return execX17Imm(gpa, state, bundle);
    if (s2_class == 0b111) return execX17Alu(gpa, state, bundle);
    if (s1_class == 0b001 and s2_class == 0b010 and (s1 >> 28) & 1 == 1) return execX17Agu(gpa, state, bundle);
    if (s1_class == 0b001 and s2_class == 0b000) return execX17Agu2(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b100) return execX17E(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b011) return execX17Ld(gpa, state, bundle);
    if (s2_class == 0b011) return execX17Sty(gpa, state, bundle);
    if (s1_class == 0b100 and s2_class == 0b100) return execX17Eagu(state, bundle);
    if (s1_class == 0b000 and s2_class == 0b000) return execX17Yld(state, bundle);
    if (s1_class == 0b000 and s2_class == 0b110) return execX17Alu05(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b010) return execX17Mve(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b001) return execX17Rsub(gpa, state, bundle);
    if (s1_class == 0b100 and s2_class == 0b110) return execX17Xpm(gpa, state, bundle);
    if (s1_class == 0b010 and s2_class == 0b110 and ((bundle[0] >> 27) & 1) == 0) return execX17XldA10(gpa, state, bundle);
    return RunError.UnknownForm;
}

/// (S1 class != 0, S2 class 011) (fact 275): the x17ld bundle becomes
/// a Y store, Y:[e<S1[18:15]>] <- s<{S2[0], S1[31:29]}>, when S2[12:11] = 11.
fn execX17Sty(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 11) & 3 != 3) return RunError.UnknownForm;
    const v: u32 = state.s[((s2 & 1) << 3) | (s1 >> 29)] & 0xFFFF;
    state.y_mem.put(gpa, state.e[(s1 >> 15) & 0xF], v) catch return RunError.OutOfMemory;
}

/// (S1 class 100, S2 class 100) with S2[3:0] = 10 (fact 273):
/// s<head[24:21]> <- AGU10[S2[28:25]](x = e<S2[7:4]>, y = R<8 + S1[31:30]>).
fn execX17Eagu(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if (s2 & 0xF != 10) return RunError.UnknownForm;
    const v = try agu10Eval((s2 >> 25) & 0xF, state.e[(s2 >> 4) & 0xF], state.r[8 + ((s1 >> 30) & 3)]);
    const dest: u32 = (bundle[0] >> 21) & 0xF;
    if (dest != 0) state.s[dest] = v;
}

/// (S1 class 000, S2 class 000) (fact 274): s<S2[13:10]> <- Y:[e<S2[24:21]>].
fn execX17Yld(state: *State, bundle: []const u32) RunError!void {
    const s2 = bundle[2];
    const dest: u32 = (s2 >> 10) & 0xF;
    if (dest != 0) state.s[dest] = (state.getY(state.e[(s2 >> 21) & 0xF]) orelse 0) & 0xFFFF;
}

/// x16 S1 class 001 / S2 class 110, 40-bit MAC (fact 203, 213):
/// acc = (s<{S2[0], S1[31:29]}> << 24) + s<S1[27:24]> + P, P =
/// s<S2[4:1]> * s<S2[8:5]> << 1 under mode S2[15:14] = 01 (only one measured);
/// acc[39:24] -> s<S2[13:10]>, acc[15:0] -> s<S1[23:20]>.
fn execX16Mac(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 14) & 3 != 1) return RunError.UnknownForm;
    const x_hi: i64 = sx16(state.s[((s2 & 1) << 3) | (s1 >> 29)]);
    const x_lo: i64 = sx16(state.s[(s1 >> 24) & 0xF]);
    const p: i64 = sx16(state.s[(s2 >> 1) & 0xF]) * sx16(state.s[(s2 >> 5) & 0xF]) * 2;
    const acc: u64 = @bitCast((x_hi << 24) + x_lo + p);
    const hi_dest: u32 = (s2 >> 10) & 0xF;
    const lo_dest: u32 = (s1 >> 20) & 0xF;
    if (lo_dest != 0) state.s[lo_dest] = @truncate(acc & 0xFFFF);
    if (hi_dest != 0) state.s[hi_dest] = @truncate((acc >> 24) & 0xFFFF);
}

const X16_FN100 = [16]?i8{ 0, null, 1, null, -1, null, 0, 0, null, null, null, null, null, 0, null, null };

/// x16 S2 class 100 (fact 92): s<head[24:21]> <- FN[S1[10:7]](x = e<S1[18:15]>),
/// FN = x, 0, x+1, 1, x-1, -1, x, x, 0 x5, x, 0, 0; R<S1[14:11]> <- #head[20:5].
fn execX16Agu100(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    if ((s1 >> 25) & 0xF != 0) return RunError.UnknownForm;
    const f: u32 = (s1 >> 7) & 0xF;
    const x: u32 = state.e[(s1 >> 15) & 0xF];
    const v: u32 = switch (f) {
        3 => 1,
        5 => 0xFFFF,
        else => if (X16_FN100[f]) |d| (x +% @as(u32, @bitCast(@as(i32, d)))) & 0xFFFF else 0,
    };
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) state.s[hd] = v;
    if ((s1 >> 11) & 0xF != 0) state.r[(s1 >> 11) & 0xF] = (h >> 5) & 0xFFFF;
}

/// x16 S1 class 001 / S2 class 111 (fact 90): Y load s<head[24:21]> <-
/// Y:[e<S2[24:21]>] placed by S1[13:12] (00: 0, 01: plain, 11: << 8); X store
/// X:[xa] <- sx24 s<S1[31:28]> with S1[18], S2[14:13] = 11, S2[11] = 0, using xa
/// as it was before the bundle; then xa <- head[20:5] when head[31] and S1[3].
fn execX16B16(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s1 >> 8) & 0xF != 0) return RunError.UnknownForm;
    const yv: u32 = (state.getY(state.e[(s2 >> 21) & 0xF]) orelse 0) & 0xFFFF;
    const hv: u32 = switch ((s1 >> 12) & 3) {
        0 => 0,
        1 => yv,
        3 => (yv << 8) & M24,
        else => return RunError.UnknownForm,
    };
    if ((s1 >> 18) & 1 == 1 and (s2 >> 13) & 3 == 3 and (s2 >> 11) & 1 == 0) {
        if (state.xa) |addr| {
            state.x_mem.put(gpa, addr, sx16To24(state.s[(s1 >> 28) & 0xF])) catch return RunError.OutOfMemory;
        }
    }
    if (h >> 31 == 1 and (s1 >> 3) & 1 == 1) state.xa = (h >> 5) & 0xFFFF;
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) state.s[hd] = hv;
}

fn sx16To24(v: u32) u32 {
    const w: u32 = v & 0xFFFF;
    return if (w & 0x8000 != 0) w | 0xFF0000 else w;
}

/// x16 shifter, S1 / S2 class 000 with S2[12] (fact 91): s<head[24:21]> <-
/// S1[14:12] = 110: #head[20:5], 111: 0, 010: yd; s<S1[11:8]> <- s<S2[28:25]> asr
/// S1[23:20] with head[26:25] = 11 (amount 0 writes nothing).  The result
/// arrives late (a write-back window); that is not modelled.
fn execX16Sh(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if (s1 >> 29 != 0 or (s1 >> 17) & 1 == 1 or (h >> 25) & 3 != 3) return RunError.UnknownForm;
    const hv: u32 = switch ((s1 >> 12) & 7) {
        6 => (h >> 5) & 0xFFFF,
        7 => 0,
        2 => state.yd,
        else => return RunError.UnknownForm,
    };
    const n: u32 = (s1 >> 20) & 0xF;
    const dest: u32 = (s1 >> 8) & 0xF;
    const sh: u32 = @as(u32, @bitCast(sx24(state.s[(s2 >> 25) & 0xF]) >> @intCast(n))) & M24;
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) state.s[hd] = hv;
    if (dest != 0 and n != 0) state.s[dest] = sh;
}

fn execX17XldA10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s2 = bundle[2];

    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a_half: u32 = s2 & 0x3;
    // A-offset pinned at 2 (S1 class = 010): a_idx = 8*a_half + 2.
    const a_idx: u32 = (8 * a_half + 2) & 0xF;

    const addr: u32 = state.a[a_idx];
    const loaded: u32 = state.readX(addr) orelse 0;
    state.s[dest_s] = loaded;
}

fn execX17Xpm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s2 = bundle[2];

    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a_half: u32 = s2 & 0x3;
    const a_idx: u32 = (8 * a_half + 4) & 0xF;

    const addr: u32 = state.a[a_idx];
    const loaded: u32 = state.readX(addr) orelse 0;
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

fn execX17Agu2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const fn_op: u32 = (s1 >> 7) & 0xF;
    const y_r: u32 = (head >> 27) & 0xF;
    const imm: u32 = (head >> 11) & 0xFFFF;
    const dest_s: u32 = (s2 >> 10) & 0xF;
    const dest_e: u32 = (2 * (s1 & 0x7)) & 0xF;
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    // AGU10[10] = "y+i" (the imm lives in head[26:11], not an s/e reg).
    const result: u32 = switch (fn_op) {
        10 => (y_val +% imm) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    if (dest_s != 0) state.s[dest_s] = result;
    if (dest_e != 0) state.e[dest_e] = result;
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
    // dest s[18:15], a s[14:11], b s[29:26], op [10:5], m [25:23], # [22:19].
    const dest: u32 = (head >> 15) & 0xF;
    const a_f: u32 = (head >> 11) & 0xF;
    const b_f: u32 = (head >> 26) & 0xF;

    // fact 70: source 0 reads zero.
    const a_val: u32 = if (a_f == 0) 0 else state.s[a_f];
    const b_val: u32 = if (b_f == 0) 0 else state.s[b_f];

    const result = (try aluWord(head, sx24(a_val), sx24(b_val))) orelse return RunError.UnknownForm;
    state.s[dest] = result;
}

// ---------------------------------------------------------------------------
// AGU10: address-generation ops indexed by a 4-bit FN field.  Ported from
// upstream emu.py (fact 109).  Values compute over (x, y) and wrap at
// 16 bits (the AGU is a 16-bit address unit, not a 24-bit ALU).
// ---------------------------------------------------------------------------

/// 0x19 s/A slot AGU (fact 149; fn 4 from 164).  fn = w3[24:20].
fn agu3Eval(fn_op: u32, x: u32, y: u32, i: u32) u32 {
    const r: u32 = switch (fn_op & 0x1F) {
        0, 6, 12 => x +% 1,
        1, 7, 13 => x -% 1,
        2, 8, 14 => x +% y,
        3, 9, 15, 23, 24 => x -% y,
        4 => i -% x,
        10 => 0 -% x,
        5, 11, 16, 17 => x,
        18 => y +% 1,
        19 => y -% 1,
        20 => 0 -% y,
        22 => y,
        25 => i,
        21, 26, 27, 28, 29, 30, 31 => x -% i,
        else => unreachable,
    };
    return r & 0xFFFF;
}

/// 0x19 A-register AGU (fact 144; fn 4 from 164).  null = OPEN.
fn aguaEval(fn_op: u32, x: u32, i: u32) ?u32 {
    const r: u32 = switch (fn_op & 0x1F) {
        0, 6, 12 => x +% i +% 1,
        1, 7, 13 => x +% i -% 1,
        2, 3, 5, 8, 9, 11, 14, 15, 17, 23, 24 => x +% i,
        16 => ((x -% 1) >> 1) +% i,
        18 => i +% 1,
        19 => i -% 1,
        20, 22, 25 => i,
        21, 26, 27, 28, 29, 30, 31 => x -% i,
        4 => i -% x,
        else => return null,
    };
    return r & 0xFFFF;
}

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
    return runFrom(gpa, state, syllables, 0, 256);
}

/// runLinear from syllable `start`, with a bundle budget of `max_bundles`
/// (a codec program runs far past runLinear's 256).
pub fn runFrom(gpa: std.mem.Allocator, state: *State, syllables: []const u32, start: usize, max_bundles: usize) RunError!usize {
    var i: usize = start;
    var bundles_run: usize = 0;
    var shadow: ?u8 = null;
    const lane0: usize = state.cur_lane;
    state.halted = false;
    while (i < syllables.len) : (bundles_run += 1) {
        if (bundles_run >= max_bundles) return RunError.RunawayProgram;
        const head = syllables[i];
        // Each syllable sits in lane (position mod 4) (fact 21).
        state.cur_lane = @intCast((lane0 + i) & 3);
        var len = bundleLen(head) orelse return RunError.UnknownBundleLength;
        // st pairs only with a wr second ([31:29] = 001, [16:15] = 11).
        if (head & 0x1F == 0x0F and head >> 29 == 1 and len == 2 and
            (i + 1 >= syllables.len or syllables[i + 1] & 0xE0018000 != 0x20018000)) len = 1;
        if (i + len > syllables.len) return RunError.UnknownBundleLength;
        const slice = syllables[i .. i + len];
        const pc_at_bundle = i;
        state.pc = i;
        if (state.profile) |p| p[i] += 1;
        try executeBundle(gpa, state, slice);
        i += len;
        // The stop has a shadow (fact 72): an ldi and a st + wr after it
        // still run and the next pair does not; two bundles, exact count OPEN.
        if (shadow) |*n| {
            if (n.* == 0) return bundles_run + 1;
            n.* -= 1;
            continue;
        }
        if (isHaltBundle(slice)) {
            state.halted = true;
            shadow = 1;
            continue;
        }
        // Resolve a pending jmp/br/bcc issued by this bundle.  Target is
        // syllable offset relative to the jump bundle's own start.  TWO
        // delay-slot bundles run before the jump fires (fact 94).
        if (state.pending_jmp_offset) |off| {
            const target_i64: i64 = @as(i64, @intCast(pc_at_bundle)) + off;
            if (target_i64 < 0 or target_i64 > @as(i64, @intCast(syllables.len))) {
                return RunError.UnknownBundleLength;
            }
            state.jump_target = @intCast(target_i64);
            state.jump_delay = 2;
            state.pending_jmp_offset = null;
        } else if (state.jump_delay > 0) {
            state.jump_delay -= 1;
            if (state.jump_delay == 0) {
                i = state.jump_target;
            }
        }
    }
    return bundles_run;
}

pub fn bundleLen(head: u32) ?u8 {
    return isa.bundleLen(head);
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
    // p<{[29], [28:26]}> (fact 220).
    const b: u32 = (head >> 26) & 0xF;
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

/// low5 0x0E, head[31] = 0, code != 0x23 (fact 221): s<S[11:8]> <-
/// ALU05[head[30:25]](a = s<S[7:4]>, b = s<S[31:28]>), m = S[17:15]; and
/// s<head[24:21]> <- mux S[14:12]: c / # / # / D[7:0]<<8 / (D<<7)[15:0] / D / # /
/// the ALU result, # = head[20:5], D = X:[A<xi>], c = execPtest's value.
fn execX0E(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const sw = bundle[1];
    const mux: u32 = (sw >> 12) & 7;
    const alu_dest: u32 = (sw >> 8) & 0xF;
    const alu_val: u32 = if (alu_dest == 0 and mux != 7) 0 else blk: {
        const word: u32 = (((h >> 25) & 0x3F) << 5) | (((sw >> 15) & 7) << 23);
        break :blk (try aluWord(word, sx24(state.s[(sw >> 4) & 0xF]), sx24(state.s[(sw >> 28) & 0xF]))) orelse return RunError.UnknownForm;
    };
    const imm: u32 = (h >> 5) & 0xFFFF;
    const hd: u32 = (h >> 21) & 0xF;
    const d: u32 = if (mux >= 3 and mux <= 5) blk: {
        const xi = state.x16_xi orelse return RunError.UnknownForm;
        break :blk state.readX(state.a[xi & 0x1F]) orelse 0;
    } else 0;
    const hv: u32 = switch (mux) {
        0 => blk: {
            const pv: u8 = state.p[(sw >> 24) & 0xF];
            break :blk switch ((sw >> 21) & 7) {
                0 => if (pv == 0) 1 else 0,
                1 => pv,
                else => return RunError.UnknownForm,
            };
        },
        1, 2, 6 => imm,
        3 => (d & 0xFF) << 8,
        4 => (d << 7) & 0xFFFF,
        5 => d,
        else => alu_val,
    };
    if (alu_dest != 0) state.s[alu_dest] = alu_val;
    if (hd != 0) state.s[hd] = hv;
}

/// low5 0x0F, S class 101 without S[22] & S[20] (fact 127): R<S[14:11]> <- #head[20:5].
fn execRimm(state: *State, bundle: []const u32) RunError!void {
    const dest: u32 = (bundle[1] >> 11) & 0xF;
    if (dest != 0) state.r[dest] = (bundle[0] >> 5) & 0xFFFF;
}

/// low5 0x0F, S class 101 with S[22] and S[20] (fact 127):
/// X:[A<4*S[2:0] + head[31:30]>] <- sx24 s<S[28:25]>.
fn execStx0f(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const sw = bundle[1];
    const a_idx: u32 = 4 * (sw & 7) + (bundle[0] >> 30);
    const v: u32 = state.s[(sw >> 25) & 0xF] & M24;
    state.x_mem.put(gpa, state.a[a_idx], v) catch return RunError.OutOfMemory;
}

/// low5 0x08, [31:29] = 010, op[10:5] = 0x30 + k (fact 61, 101):
/// Y:[e<k>] <- s<[25:22]>.
fn execStk(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const v: u32 = state.s[(head >> 22) & 0xF] & 0xFFFF;
    state.y_mem.put(gpa, state.e[(head >> 5) & 0xF], v) catch return RunError.OutOfMemory;
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
    // op[0] = s1[0] adds 2 (fact 287): A<4n + 1 + 2*op[0]>.
    const a_idx: u32 = (4 * n + 1 + 2 * (s1 & 1)) & 0xF;
    state.a[a_idx] = addr;
}

/// 0x0B class 100 (fact 259): s<[28:26]> <- Y:[e<[8:5]>].  The A<[18:15]>
/// side named in the note is not modelled.
fn execAset0b(state: *State, head: u32) RunError!void {
    const dest: u32 = (head >> 26) & 7;
    if (dest != 0) state.s[dest] = (state.getY(state.e[(head >> 5) & 0xF]) orelse 0) & 0xFFFF;
}

/// 0x0C with [31] = 1 (fact 282): R<[30:27]> <- AGUA[[11:7]] over
/// x = A<[16:12]>, i = [26:17].  Only i = 0 is measured.
fn execRagu0c(state: *State, head: u32) RunError!void {
    if ((head >> 17) & 0x3FF != 0) return RunError.UnknownForm;
    const v = aguaEval((head >> 7) & 0x1F, state.a[(head >> 12) & 0xF], 0) orelse return RunError.UnknownForm;
    const dest: u32 = (head >> 27) & 0xF;
    if (dest != 0) state.r[dest] = v;
}

/// 0x0F head class 000 with a class-001 second (fact 279):
/// e<{w1[0], head[31:29]}> <- #head[20:5].  The bundle's Y store is not
/// modelled.
fn execSte(state: *State, bundle: []const u32) RunError!void {
    const dest: u32 = ((bundle[1] & 1) << 3) | (bundle[0] >> 29);
    if (dest != 0) state.e[dest] = (bundle[0] >> 5) & 0xFFFF;
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
    // s<[29:26]>: class 010 keeps bit 29 clear, class 011 sets it (281).
    const dest_s: u32 = (head >> 26) & 0xF;
    const dest_r: u32 = (head >> 22) & 0xF;
    const x_e: u32 = (head >> 5) & 0xF;
    const y_r: u32 = (head >> 13) & 0xF;
    const fn_op: u32 = (head >> 17) & 0xF;

    const x_val: u32 = if (x_e == 0) 0 else state.e[x_e];
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];

    // ROP07 table from upstream/tools/sm1isa/emu.py:
    //   x, y, 0, -x, -y, 1, x+y, x-y, 0, 0, y, y, y, -y, y+1, y-1
    const v: u32 = switch (fn_op & 0xF) {
        0 => x_val & M24,
        1, 10, 11, 12 => y_val & M24,
        2, 8, 9 => 0,
        3 => (0 -% x_val) & M24,
        4, 13 => (0 -% y_val) & M24,
        5 => 1,
        6 => (x_val +% y_val) & M24,
        7 => (x_val -% y_val) & M24,
        14 => (y_val +% 1) & M24,
        15 => (y_val -% 1) & M24,
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
    // The immediate is sign-extended to 24 bits (own-x02u24's operands, built with ldi:
    // 0xC321 lands as 0xFFC321).
    state.s[s_idx] = sx16To24(imm);
    // e0 = the last instruction's [20:5] (fact 65).
    state.e[0] = imm;
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
    try testing.expectEqual(@as(u32, 0xFFDEAD), state.s[0]); // sign-extended
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
    try testing.expectEqual(@as(u32, 0xFFBEEF), state.s[3]); // sign-extended
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

test "runLinear runs the stop's two-bundle shadow, then halts" {
    // fact 72: an ldi and a st + wr after the stop land, the next
    // pair does not.
    var state = State{};
    defer state.deinit(testing.allocator);

    const b_halt = try asm_mod.assembleOne("x17.halt");
    const b1 = try asm_mod.assembleOne("ldi s1, #0x111");
    const b2 = try asm_mod.assembleOne("ldi s2, #0x222");
    const b3 = try asm_mod.assembleOne("ldi s3, #0x333");

    var buf: [16]u32 = undefined;
    var n: usize = 0;
    for ([_]@TypeOf(b_halt){ b_halt, b1, b2, b3 }) |b| {
        @memcpy(buf[n .. n + b.len], b.slice());
        n += b.len;
    }
    _ = try runLinear(testing.allocator, &state, buf[0..n]);
    try testing.expectEqual(@as(u32, 0x111), state.s[1]);
    try testing.expectEqual(@as(u32, 0x222), state.s[2]);
    try testing.expectEqual(@as(u32, 0), state.s[3]);
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

test "a file with labels and constants runs: bcc over four adds gives s1 = 4" {
    const gpa = testing.allocator;
    const r = try asm_mod.assembleFile(gpa,
        \\.const ADD = 14
        \\        ldi s2, #1
        \\        bcc #0, skip
        \\        alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\skip:   alu s1, s1, s2, #ADD
        \\        alu s1, s1, s2, #ADD
        \\        x17.halt
    );
    const words = switch (r) {
        .ok => |w| w,
        .err => return error.TestUnexpectedResult,
    };
    defer gpa.free(words);
    var st: State = .{};
    defer st.deinit(gpa);
    _ = try runLinear(gpa, &st, words);
    try testing.expect(st.halted);
    try testing.expectEqual(@as(u32, 4), st.s[1]);
}

test "0x19 op 23: a negative count is a rounded right shift; left shifts saturate both ways" {
    try testing.expectEqual(@as(i64, 0x700), shift23(7, 8)); // own-x19alu
    try testing.expectEqual(@as(i64, 0x71), shift23(0x708, -4));
    try testing.expectEqual(@as(i64, -0x70), shift23(-0x708, -4));
    try testing.expectEqual(@as(i64, 0x7FFFFF), shift23(0x400000, 2));
    try testing.expectEqual(@as(i64, -0x800000), shift23(-0x400000, 2));
    try testing.expectEqual(@as(i64, -16), shift23(-1, 4));
}

test "x19.alu #23 with a negative count register shifts right, rounded" {
    var state = State{};
    defer state.deinit(testing.allocator);
    state.s[3] = 0x708;
    state.s[6] = 0xFFFFFC; // -4
    const b_alu = try asm_mod.assembleOne("x19.alu s7, s3, s6, #23");
    const b_halt = try asm_mod.assembleOne("x17.halt");
    var buf: [16]u32 = undefined;
    @memcpy(buf[0..b_alu.len], b_alu.slice());
    @memcpy(buf[b_alu.len .. b_alu.len + b_halt.len], b_halt.slice());
    _ = try runLinear(testing.allocator, &state, buf[0 .. b_alu.len + b_halt.len]);
    try testing.expectEqual(@as(u32, 0x71), state.s[7]);
}

test "ldi sign-extends its 16-bit immediate" {
    var state = State{};
    defer state.deinit(testing.allocator);
    const neg = try asm_mod.assembleOne("ldi s5, #-9");
    try executeBundle(testing.allocator, &state, neg.slice());
    try testing.expectEqual(@as(u32, 0xFFFFF7), state.s[5]);
    const pos = try asm_mod.assembleOne("ldi s5, #0x7FFF");
    try executeBundle(testing.allocator, &state, pos.slice());
    try testing.expectEqual(@as(u32, 0x7FFF), state.s[5]);
}

test "profile counts the bundles run at each syllable, jump delay slots included" {
    const gpa = std.testing.allocator;
    const prog = switch (try asm_mod.assembleFile(gpa,
        \\        ldi s1, #2
        \\        ldi s2, #1
        \\loop:   alu s1, s1, s2, #0x10
        \\        ptst p1, s1, #1, #0x0B
        \\        jmp #2, #1, loop
        \\        alu s0, s0, s0, #0
        \\        alu s0, s0, s0, #0
        \\        x17.halt
        \\
    )) {
        .ok => |p| p,
        .err => return error.Assemble,
    };
    defer gpa.free(prog);
    const profile = try gpa.alloc(u64, prog.len);
    defer gpa.free(profile);
    @memset(profile, 0);
    var state = State{ .profile = profile };
    defer state.deinit(gpa);
    const n = try runFrom(gpa, &state, prog, 0, 100);
    var sum: u64 = 0;
    for (profile) |c| sum += c;
    try std.testing.expectEqual(@as(u64, n), sum);
    try std.testing.expectEqual(@as(u64, 3), profile[2]); // the loop body, three passes
}
