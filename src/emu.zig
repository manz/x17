//! x17 Zig emulator, v0.
//!
//! Built to be the CI oracle for the assembler: assemble a program, run it
//! here, diff against the probes' measured Y writes.  Follows the Python
//! model (the Python ISA model) on measured rules but starts fresh
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
    /// across bundles (measured).
    /// p0 reads 1 (608: ld0e's cond; branch decode treats p<0> as unconditional); a write to
    /// it is unmeasured and refuses.
    p: [16]u8 = .{1} ++ @as([15]u8, @splat(0)),
    /// Y data memory, keyed by address.  Sparse: only addresses the program
    /// writes are present.  Caller inspects via `getY`.
    y_mem: std.AutoHashMapUnmanaged(u32, u32) = .{},
    /// X data memory, keyed by address.  Separate bank from Y (measured).
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
    /// Set by a bundle that stops the program one bundle later (x17e S2[3:0] = 4, fact 448).
    stop_pending: bool = false,
    /// Set by a bundle after which one more bundle runs and the program stops without a halt
    /// (x17 S2 class 101 with S1[22:20] = 010, fact 457).
    wedge_pending: bool = false,
    /// Set by a 0x00 END (control 6 / 7, fact 759): two more bundles run, then the program halts.
    end_pending: bool = false,
    /// Set by 0x00 control 0x08 / 0x0D (315, 768, 769): the next arm is abnormal (OPEN).
    next_arm_parks: bool = false,
    /// Set by an x17sty stop form (648, 667): the run ends after this bundle, no bundle after
    /// it runs and no completion rises.
    stop_now: bool = false,
    /// Set when a run ended at such a stop: the host sees a wedge, not a halt.
    silent_stop: bool = false,
    /// When set, a bundle refused as UnknownForm is reported here and skipped as a nop instead
    /// of ending the run (`x17 play --trap-list`).  The state after a skip is a guess.
    on_unknown: ?UnknownHook = null,
    /// Whether e0, the address entry, is known (fact 466, 468): see updateE0.
    e0_known: bool = false,
    /// The 0x16 X store address latch, one bundle behind (fact 90).
    xa: ?u32 = null,
    /// The 0x18 class-000 phase T, flipped by w2[17] = 0 (fact 180).
    x18_t: bool = false,

    /// When set, counts X / Y reads of addresses nothing has written (compat runs).
    unset_reads: ?*UnsetReads = null,

    /// The input stream (fact 783): packet bytes the host queued, read MSB first, one bit
    /// stream across packets (784).
    stream: std.ArrayList(u8) = .empty,
    /// Bits of `stream` already read.
    stream_bit: usize = 0,
    /// Set by the ld0e enable word (783); reads give 0 before it.
    stream_on: bool = false,
    /// Set when a read found too few bits queued: the run stops at that read, which runs
    /// again once the host queues more (784).
    stream_starved: bool = false,

    pub fn deinit(self: *State, gpa: std.mem.Allocator) void {
        self.y_mem.deinit(gpa);
        self.x_mem.deinit(gpa);
        self.stream.deinit(gpa);
    }

    /// The host arms a packet: its bytes join the stream after any still unread (784).
    pub fn queueStream(self: *State, gpa: std.mem.Allocator, bytes: []const u8) error{OutOfMemory}!void {
        try self.stream.appendSlice(gpa, bytes);
    }

    /// Y words are 16 bits (700): a store keeps s[15:0], so only those bits read back.
    pub fn getY(self: *const State, addr: u32) ?u32 {
        const v = self.y_mem.get(addr);
        if (v == null) if (self.unset_reads) |u| {
            u.y += 1;
            if (u.first_y == null) u.first_y = addr;
        };
        return if (v) |w| w & 0xFFFF else null;
    }

    pub fn getX(self: *const State, addr: u32) ?u32 {
        const v = self.x_mem.get(addr);
        if (v == null) if (self.unset_reads) |u| {
            u.x += 1;
            if (u.first_x == null) u.first_x = addr;
        };
        return v;
    }

    /// An executor's X read: also latches xd, the last X word read
    /// (fact 161, 241).
    pub fn readX(self: *State, addr: u32) ?u32 {
        const v = self.getX(addr);
        self.xd = v orelse 0;
        return v;
    }
};

/// X / Y reads of never-written addresses (and OPEN p skips), counted while `State.unset_reads` is set.
pub const UnsetReads = struct {
    x: u64 = 0,
    y: u64 = 0,
    first_x: ?u32 = null,
    first_y: ?u32 = null,
    /// x18 class-011 p writes left alone because their compare is OPEN (776).
    open_p: u64 = 0,
};

pub const RunError = error{
    UnknownForm,
    LdiDestOpen, // ldi dest = 2..15 (marked OPEN in isa.toml)
    OutOfMemory,
    UnknownBundleLength,
    RunawayProgram,
    /// A halt too far after the latest armer never completes (fact 98, 477, 484).
    HaltDeadline,
    /// The arm after a 0x08 / 0x0D: abnormal, what runs OPEN (315, 768, 769).
    Parked,
};

/// Execute one bundle (1..7 syllables) against `state`.  Dispatches by
/// classifying the HEAD syllable with `isa.classify`.  Unknown or
/// not-yet-modelled forms return `UnknownForm` so the caller can tell.
/// isa.toml forms executeBundle serves by bit pattern before classify, or refuses (unmodelled).
const pattern_forms = [_][]const u8{ "bcc", "br", "jmp", "loop", "mv", "param", "pc14", "st1", "win32", "wr", "xst", "yagu", "ypair", "yrm", "ystm", "ytst", "zero" };

const Handler = *const fn (std.mem.Allocator, *State, []const u32) RunError!void;

/// A handler that reads only the head syllable.
fn onHead(comptime f: fn (std.mem.Allocator, *State, u32) RunError!void) Handler {
    return struct {
        fn run(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
            return f(gpa, state, bundle[0]);
        }
    }.run;
}

/// A head-only handler with no memory writes.
fn onHeadNoAlloc(comptime f: fn (*State, u32) RunError!void) Handler {
    return struct {
        fn run(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
            _ = gpa;
            return f(state, bundle[0]);
        }
    }.run;
}

/// The forms isa.classify names, after the bit-pattern rows in executeBundle.
const named_forms = std.StaticStringMap(Handler).initComptime(.{
    .{ "ldi", onHead(execLdi) },
    .{ "rli", onHead(execRli) },
    .{ "rop", onHead(execRop) },
    .{ "lde", onHead(execLde) },
    .{ "ld0e", @as(Handler, execLd0e) },
    .{ "stx", onHead(execStx) },
    .{ "dld", onHeadNoAlloc(execDld) },
    .{ "ldy", onHead(execLdy) },
    .{ "ldy.r", onHead(execLdy) },
    .{ "ptst", onHead(execPtst) },
    .{ "plog", onHead(execPlog) },
    .{ "st.y", @as(Handler, execStY) },
    .{ "alu", onHead(execAlu) },
    .{ "x17", @as(Handler, execX17) },
    .{ "x19", @as(Handler, execX19) },
    .{ "alu11", @as(Handler, execAlu11) },
    .{ "agu10r", @as(Handler, execAgu10r) },
});

pub fn executeBundle(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    // s0 always reads 0 and a write to it is discarded (fact 438).
    state.s[0] = 0;
    defer state.s[0] = 0;
    if (bundle.len == 0) return;
    // A0 reads 0 and writes to it have no visible effect, like R0 (silicon probe
    // 2026-11-06: lda A0 then a readback gives 0; an x19mac A0 source reads 0).
    state.a[0] = 0;
    defer state.a[0] = 0;
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
    if (low5 == 0x14 and bundle.len >= 2 and (bundle[1] >> 25) & 1 == 1) {
        // ypair class 110 admits head[31:29] 0-3 (616); head[31] = 1 leaves the s write undone
        // and the rest is unmeasured: refused.  Class 111 and yagu keep head[31:29] 0 / 2.
        if (bundle[1] >> 29 == 0b110) {
            if (head >> 31 == 1) return RunError.UnknownForm;
            return execYpair(state, bundle);
        }
        if (head & 0xA0000000 == 0) {
            if (bundle[1] >> 29 == 0b100) return execYagu(state, bundle);
            if (bundle[1] >> 30 == 0b11) return execYpair(state, bundle);
        }
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
        if (s_class == 0b010) return execPc14(gpa, state, bundle);
        // 486: a class-011 second with S[28:27] = 10 writes the immediate unconditionally.
        if (s_class == 0b011 and (bundle[1] >> 27) & 3 == 2 and pc14C3PMeasured(bundle)) return execPc14(gpa, state, bundle);
    }
    if (low5 == 0x12 and bundle.len >= 2) {
        const s_class: u32 = (bundle[1] >> 29) & 0x7;
        // S class 011 runs as 001 (618).
        if (s_class == 0b001 or s_class == 0b011) return execX12Agu(gpa, state, bundle);
        // S class 111 runs as 101 (fact 551).
        if (s_class == 0b101 or s_class == 0b111) return execX12Aset(gpa, state, bundle);
    }
    if (low5 == 0x0B) {
        const high3: u32 = (head >> 29) & 0x7;
        if (high3 == 0b010 or high3 == 0b011) return execJmp(gpa, state, head);
        if (high3 == 0b001 and (head >> 19) & 1 == 1) return execPlog(gpa, state, head);
        if (high3 == 0b100 or high3 == 0b101) return execAset0b(state, head);
    }
    // jr2 (302, 303, 617): 0x11 + a class-100 second with w1[23:21] = 001.
    if (low5 == 0x11 and bundle.len == 2 and bundle[1] >> 29 == 0b100 and (bundle[1] >> 21) & 7 == 1) return execJr2(gpa, state, bundle);
    // The class-05 ALU word with its [25:19] modifiers (fact 65-67).
    if (low5 == 0x05 and (head >> 30) == 1) return execAlu(gpa, state, head);
    // ptst at class 011: head[29] is the dest's bit 3 (fact 219).
    if (low5 == 0x0D and (head >> 29) == 0b011) return execPtst(gpa, state, head);
    // lde with [31] = 1 (fact 280): same fields as class 01x.
    if (low5 == 0x06 and (head >> 30) == 0b11) return execLde(gpa, state, head);
    // lde class 101 (fact 479): e<h[29:26]> <- #h[20:5]; h[29] = 1 makes it e8..e15.
    // The halt-deadline arm is the runner's.
    if (low5 == 0x06 and (head >> 29) == 0b101) return execLde(gpa, state, head);
    // rop at classes 011 and 100 (fact 281, 285): s<[29:26]>.
    if (low5 == 0x07 and ((head >> 29) == 0b011 or (head >> 29) == 0b100)) return execRop(gpa, state, head);
    if (low5 == 0x0C and (head >> 31) == 1) return execRagu0c(state, head);
    if (low5 == 0x0F and bundle.len == 2) {
        const w1 = bundle[1];
        if (w1 >> 29 == 0b001 and (head >> 29) != 0b001) return execSte(state, bundle);
        if (w1 >> 29 == 0b101 and w1 & 0x00500000 == 0x00500000) return execStx0f(gpa, state, bundle);
        if (w1 >> 29 == 0b101) return execRimm(state, bundle);
        // 757: st.y with a class-011 second; the loop head (0x6044800x) keeps its own row.
        if (w1 >> 29 == 0b011 and head & 0xFFFFFFF0 != 0x60448000) return execSte011(state, bundle);
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
        // 783: head[31] with head[30:25] 0x20-0x22, S1 class 000 with S1[18]: the stream read.
        const code16: u32 = (head >> 25) & 0x3F;
        if (s2_class == 0b111 and s1_class16 == 0 and head >> 31 == 1 and code16 >= 0x20 and code16 <= 0x22 and (bundle[1] >> 18) & 1 == 1) return execStreamRead(state, bundle);
        // 778: S1[18] with S2 & 0xE0050000 = 0xE0040000 is b16 under S1 classes other than
        // 000; class 000 keeps x16alu (563, 571 measured bundles match the same pattern).
        if (s2_class == 0b111 and s1_class16 != 0 and ((bundle[1] >> 18) & 1 == 1 and bundle[2] & 0xE0050000 == 0xE0040000)) return execX16B16(gpa, state, bundle);
        if (s1_class16 == 1 and s2_class == 0b111) return execX16B16(gpa, state, bundle);
        if (s1_class16 == 0 and s2_class == 0 and (bundle[2] >> 12) & 1 == 1) return execX16Sh(state, bundle);
        if (s2_class == 0b010 and (s1_class16 == 0 or (bundle[1] >> 24) & 1 == 1)) return execX16Xs(state, bundle);
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
        // S class 011 runs as 001 (621).
        if (s_class == 0b001 or s_class == 0b011) return execX13Agu(gpa, state, bundle);
        if (s_class == 0b101) {
            const h13 = bundle[0];
            const code13: u32 = (h13 >> 5) & 0x3F;
            if ((bundle[1] >> 19) & 1 == 1) return execMul13(gpa, state, bundle);
            if (code13 != 0) return execX13St(gpa, state, bundle);
            // mul13 dest field location still open; not yet modelled.
            return RunError.UnknownForm;
        }
    }
    if (low5 == 0x18 and bundle.len >= 4) {
        const w3_class: u32 = (bundle[3] >> 29) & 0x7;
        const w1_mode: u32 = (bundle[1] >> 12) & 0x7;
        // Class 110's X store (599) is measured beside the p / multiply slots only.
        const st110 = w3_class == 0b110 and (bundle[2] >> 22) & 3 == 3;
        if (st110 and (((bundle[3] >> 8) & 1) == 1 or w1_mode == 5)) return RunError.UnknownForm;
        if (w3_class == 0b110 and ((bundle[3] >> 8) & 1) == 1) {
            // The class-110 p write and multiply slots run beside the branch (fact 470:
            // own-x18p1 carries w3[8] with branch predicate p0).
            try x18PWrite(state, bundle);
            try execX18Mpy(gpa, state, bundle);
            return execX18Br(gpa, state, bundle);
        }
        if (w3_class == 0b011 and bundle[3] & 0x101 == 0x100) return execX18Br(gpa, state, bundle);
        if (w3_class == 0b110) {
            // The ALU2 slot reads before the bundle writes, and writes last (527).
            const alu2 = try x18Alu110(state, bundle);
            const st_s: u32 = ((bundle[2] & 1) << 3) | (bundle[1] >> 29);
            const st_old: u32 = state.s[st_s];
            if (w1_mode == 5) {
                // The p write runs beside the X read too (706's word has w1 mode 5).
                try x18PWrite(state, bundle);
                // The multiply beside the X read reads xd before the read sets it (689).
                if ((bundle[2] >> 9) & 0xF != 0 or (bundle[2] >> 24) & 0xF != 0) try execX18Mpy(gpa, state, bundle);
                try execX18Xrd(gpa, state, bundle);
            } else if (bundle[2] != 0) {
                try x18PWrite(state, bundle);
                try execX18Mpy(gpa, state, bundle);
            } else try execX18Imm(gpa, state, bundle);
            if (alu2) |w| state.s[w.dest] = w.value;
            if (st110) {
                // X:[A<w2[21:17]>] <- sx24 s<{w2[0], w1[31:29]}> (599); the old s, and only
                // measured where nothing else in the bundle writes that s.
                if (state.s[st_s] != st_old) return RunError.UnknownForm;
                state.x_mem.put(gpa, state.a[(bundle[2] >> 17) & 0x1F], st_old & M24) catch return RunError.OutOfMemory;
            }
            return;
        }
        // 485, 487: with w2[17], w2[18] and w3[7] all clear a class-000 bundle may stop two
        // slots later or run on (own-x18e2nd-w17 of 180 runs, the template wedges with w3[8]
        // set or clear); no field decides yet.
        if (w3_class == 0b000 and bundle[2] & 0x60000 == 0 and bundle[3] & 0x80 == 0) return RunError.UnknownForm;
        if (w3_class == 0b000) return execX18C0(gpa, state, bundle);
        if (w3_class == 0b100) return execX18C4(gpa, state, bundle);
        if (w3_class == 0b011) return execX18C3(gpa, state, bundle);
        if (w3_class == 0b111) return execX18C7(gpa, state, bundle);
        if (w3_class == 0b101) return execX18C5(gpa, state, bundle);
        if (w3_class == 0b010) return execX18C2(gpa, state, bundle);
        if (w3_class == 0b001) return execX18C1(gpa, state, bundle);
    }
    if (low5 == 0x02 and bundle.len >= 6) {
        // The X store and the immediate/yd R source (fact 513) latch before the mode's
        // slots write and land after them.
        const common = try x02Common(state, bundle);
        const handled = try execX02Mode(gpa, state, bundle);
        try common.apply(gpa, state);
        if (handled) return;
    }
    if (low5 == 0x03 and bundle.len >= 6) {
        try x03CondJump(state, bundle);
        try x03RYRead(state, bundle);
        const w4_mode: u32 = (bundle[4] >> 11) & 0x7;
        if (w4_mode == 5) {
            // The repeat class w2[31:29] = 011 runs the MAC row once and hangs later (673).
            if (bundle[2] >> 29 == 3) return RunError.UnknownForm;
            // The product low half runs beside the Xrd path (603), reading before it writes.
            if ((bundle[4] >> 7) & 0xF != 0 and (bundle[5] >> 6) & 1 == 1) return RunError.UnknownForm;
            // The w2[12:9] high dest is live here too (604, 605).
            const phi = try x03ProductHigh(state, bundle);
            const plo = try x03ProductLow(state, bundle);
            const e_pre = state.e;
            try execX03Xrd(gpa, state, bundle);
            try execX03Agu(gpa, state, bundle, e_pre);
            if (phi) |w| state.s[w.dest] = w.value;
            if (plo) |w| state.s[w.dest] = w.value;
            return;
        }
        if (w4_mode == 0) {
            try execX03Mode0(gpa, state, bundle);
            // w3[31] is the counter's index bit, not a repeat (679): one pass.  Loops are
            // x03CondJump's jumps.  With w2[28] = 1 the X store post-increments its A on every
            // pass (675, 679); the field is not separated from the jump.  The step is agu_xr
            // fn w2[26:22] with r = R<w5[25:22]>: 0 x + 1, 1 x - 1, 2 x + r, 3 x - r (741).
            if ((bundle[2] >> 28) & 1 == 1 and ((bundle[5] >> 18) & 3) == 3) {
                const fn_op: u32 = (bundle[2] >> 22) & 0x1F;
                if (fn_op > 3) return RunError.UnknownForm;
                const ai = x03AIndex(bundle);
                state.a[ai] = aguXrEval(fn_op, state.a[ai], state.r[(bundle[5] >> 22) & 0xF]);
            }
            return;
        }
        // Y path (w5[21] = 1): the read enters the 24-bit lane as D << 8;
        // mux 011 takes lane[15:0], mux 100 lane[16:1] (fact 236).
        // With w5[1] = 0 and w5[21] = 0 the lane takes yd instead of a fresh read (565).
        const yd_lane = (bundle[5] >> 1) & 1 == 0 and (bundle[5] >> 21) & 1 == 0;
        if ((w4_mode == 3 or w4_mode == 4) and ((bundle[5] >> 21) & 1 == 1 or yd_lane)) {
            const dest_s: u32 = (bundle[4] >> 3) & 0xF;
            const src_e: u32 = (bundle[3] >> 27) & 0xF;
            const d: u32 = (if (yd_lane) state.yd else state.getY(state.e[src_e]) orelse 0) & 0xFFFF;
            state.yd = d;
            const lane: u32 = (d << 8) & M24;
            const v: u32 = if (w4_mode == 3) lane & 0xFFFF else (lane >> 1) & 0xFFFF;
            if (dest_s != 0) state.s[dest_s] = v;
            return;
        }
        // w4 mux 001 / 010 with w5[29:28] = 11: s<w4[6:3]> <- Y:[e<w3[30:27]>] with w5[21],
        // else yd (602, 565's rule).  The fresh read sets yd (after 602).
        if ((w4_mode == 1 or w4_mode == 2) and ((bundle[5] >> 28) & 0x3) == 0b11) {
            const dest_s: u32 = (bundle[4] >> 3) & 0xF;
            const fresh = (bundle[5] >> 21) & 1 == 1;
            const v: u32 = if (fresh) (state.getY(state.e[(bundle[3] >> 27) & 0xF]) orelse 0) & 0xFFFF else state.yd;
            if (fresh) state.yd = v;
            if (dest_s != 0) state.s[dest_s] = v;
            return;
        }
        // w4 mux 001 / 010 with w5[29:28] = 10: s<w4[6:3]> <- e<w3[30:27]>, or e<w3[26:23]>
        // with w1[8] (fact 190 / own-x03esrc; 519, 2 of 3 rows).
        if ((w4_mode == 1 or w4_mode == 2) and ((bundle[5] >> 28) & 0x3) == 0b10) {
            const dest_s: u32 = (bundle[4] >> 3) & 0xF;
            const alt = (bundle[1] >> 8) & 1 == 1;
            const src_e: u32 = if (alt) (bundle[3] >> 23) & 0xF else (bundle[3] >> 27) & 0xF;
            // 519: w1[8] with index 0 read 0x3422, not e0 (OPEN)
            if (alt and src_e == 0) return RunError.UnknownForm;
            if (dest_s != 0) state.s[dest_s] = state.e[src_e];
            return;
        }
    }
    // 0x01's head[31] is inert and head[30:27] is its R slot's y index (fact 320).
    if ((low5 == 0x00 or low5 == 0x01) and bundle.len == 7) {
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

    const run = named_forms.get(f.name) orelse return RunError.UnknownForm;
    return run(gpa, state, bundle);
}

/// pc14, low5 0x14 with a class-010 second and S[25] (fact 107, 380, 381, 464), or a
/// class-011 second with mux 10 (486) whose p write is measured (487):
/// p<S[7:4]> <- cond(group h[28:25], code S[11:8]) of v = mode S[14:12] applied to
/// a = s<S[22:19]> (read before the s write); s<h[24:21]> <- by S[28:27]: 00 the bundle's own
/// condition negated (478, not 107's 0), 01/10 the immediate h[20:5], 11 refused.  Unmeasured
/// (group, code) cells are refused.
fn execPc14(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const h = bundle[0];
    const sw = bundle[1];
    const p_dest: u32 = (sw >> 4) & 0xF;
    const c: u32 = (sw >> 8) & 0xF;
    const g: u32 = (h >> 25) & 0xF;
    const v = pc14Operand((sw >> 12) & 7, state.s[(sw >> 19) & 0xF]);
    const bit = try pc14Cond(g, c, v);
    const s_dest: u32 = (h >> 21) & 0xF;
    if (s_dest != 0) {
        const src: u32 = (sw >> 27) & 3;
        if (src == 3) return RunError.UnknownForm;
        state.s[s_dest] = if (src == 0) @intFromBool(bit == 0) else (h >> 5) & 0xFFFF;
    }
    if (p_dest != 0) state.p[p_dest] = bit;
}

/// A class-011 pc14's p write is measured only for group 0 code 0, which sets it (487: the
/// measured pair sets p6 from 0 and from 1).
fn pc14C3PMeasured(bundle: []const u32) bool {
    if ((bundle[1] >> 4) & 0xF == 0) return true;
    return (bundle[0] >> 25) & 0xF == 0 and (bundle[1] >> 8) & 0xF == 0;
}

/// pc14's operand transform (fact 380): 0-2 -a, 3 ~a, 4 1-a, 5-6 a, 7 -|a|.
fn pc14Operand(m: u32, raw: u32) i32 {
    const a: i32 = @as(i32, @bitCast(raw << 8)) >> 8;
    return switch (m) {
        0, 1, 2 => -%a,
        3 => ~a,
        4 => 1 -% a,
        5, 6 => a,
        else => -%@as(i32, @intCast(@abs(a))),
    };
}

/// The measured pc14 conditions; every other (group, code) is refused.
fn pc14Cond(g: u32, c: u32, v: i32) RunError!u8 {
    switch (g) {
        0 => if (c < 8 or c == 9) return 1,
        11 => if (c == 12) return @intFromBool(v & 1 == 0),
        12 => {
            if (c == 15) return 1;
            if (c < 12) return @intFromBool(switch (c % 6) {
                0 => v >= 0,
                1 => v <= 0,
                2 => v > 0,
                3 => v < 0,
                4 => v != 0,
                else => v == 0,
            });
        },
        14 => return @intCast((@as(u32, 0xFD1B) >> @intCast(c)) & 1),
        else => {},
    }
    return RunError.UnknownForm;
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

/// The w4[13:11] modes' slots; false when no mode claims the bundle.
fn execX02Mode(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!bool {
    const w4_mode: u32 = (bundle[4] >> 11) & 0x7;
    if (w4_mode == 1) {
        try execX02Xrd(gpa, state, bundle);
        return true;
    }
    if (w4_mode == 5) {
        // Slots are VLIW; all slot inputs latch before any write
        // commits (fact 182/184 / own-x02x3, own-x02coll).
        // Snapshot s-regs so x3 reads pre-slot values.
        const s_pre = state.s;
        // The X read writes before the multiply reads here.  Whether silicon's
        // multiply sees the old or the new s is OPEN (own-x02yrd overlaps them
        // but observes only the read); code should not lean on either.  A read
        // into s0 is never seen (652).
        try execX02Xrd(gpa, state, bundle);
        try execX02Mpy(gpa, state, bundle);
        try execX02Ainc(gpa, state, bundle);
        try execX02X3Pre(gpa, state, bundle, s_pre);
        return true;
    }
    if (w4_mode == 0) {
        // The third slot runs here too (fact 549, own-x02t3).
        const s_pre = state.s;
        try execX02Ainc(gpa, state, bundle);
        try execX02X3Pre(gpa, state, bundle, s_pre);
        return true;
    }
    if (w4_mode == 2) {
        try execX02Xrd(gpa, state, bundle); // esrc path
        // The A slot runs here too: asrc and esrc name A1 as its destination, so asrc An
        // sets A1 <- An + 1 and esrc steps A1 (157, 795).
        try execX02Ainc(gpa, state, bundle);
        return true;
    }
    return false;
}

const X02Common = struct {
    x_on: bool = false,
    x_addr: u32 = 0,
    x_val: u32 = 0,
    r_dest: u32 = 0,
    r_val: u32 = 0,
    /// The R slot's Y side sets yd (571): a fresh read with w5[12], else yd itself.
    yd: ?u32 = null,

    fn apply(c: X02Common, gpa: std.mem.Allocator, state: *State) RunError!void {
        if (c.x_on) state.x_mem.put(gpa, c.x_addr, c.x_val) catch return RunError.OutOfMemory;
        if (c.r_dest != 0) state.r[c.r_dest] = c.r_val;
        if (c.yd) |v| state.yd = v;
    }
};

/// 0x02 slots outside the w4 modes (fact 513, 547): w5[10:9] = 11 stores sx24
/// s<{w2[0], w1[31:29]}> at X:[A<w4[21:17]>]; R<w3[22:19]> <- by w5[27:26]: 00 w5[31:30] = 11
/// #head[26:11], 00 the Y side, 10 the third slot's value (594), 01 refused; 01 (w5[11] ? AGU10 : AGU10B)[w1[10:7]](x =
/// e<w3[30:27]>, y = R<w5[25:22]>); w5[27] = 1 OPEN (265's "R <- 1" holds only in fresh state).
/// Both read before the bundle writes.
fn x02Common(state: *const State, bundle: []const u32) RunError!X02Common {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    var c: X02Common = .{};
    if ((w5 >> 9) & 3 == 3) {
        c.x_on = true;
        c.x_addr = state.a[(w4 >> 17) & 0x1F];
        c.x_val = state.s[((w2 & 1) << 3) | (w1 >> 29)] & M24;
    }
    const r_dest: u32 = (w3 >> 19) & 0xF;
    if (r_dest != 0) {
        c.r_dest = r_dest;
        c.r_val = switch ((w5 >> 26) & 3) {
            0 => switch (w5 >> 30) {
                3 => (h >> 11) & 0xFFFF,
                2 => (try x02RFromX3(state, bundle)) & 0xFFFF, // 594
                0 => blk: {
                    const v = x02YSide(state, bundle) & 0xFFFF;
                    c.yd = v;
                    break :blk v;
                },
                else => return RunError.UnknownForm,
            },
            1 => blk: {
                const fn_op: u32 = (w1 >> 7) & 0xF;
                const x: u32 = state.e[(w3 >> 27) & 0xF];
                const y: u32 = state.r[(w5 >> 22) & 0xF];
                break :blk if ((w5 >> 11) & 1 == 1) try agu10Eval(fn_op, x, y) else agu10bEval(fn_op, x, y);
            },
            // The A slot's sum, wrapped by R<w3[18:15]> (566, 567).
            2 => wrapOnce(state, aguXrEval((w2 >> 22) & 0x1F, state.a[w3 & 0x1F], state.r[(w5 >> 22) & 0xF]), (w3 >> 15) & 0xF),
            else => return RunError.UnknownForm,
        };
    }
    return c;
}

/// x02's Y-side source (fact 186, 545): Y:[e<w3[30:27]>] when w5[12], else yd, the last
/// Y read before the bundle.
fn x02YSide(state: *const State, bundle: []const u32) u32 {
    if ((bundle[5] >> 12) & 1 == 0) return state.yd;
    return state.getY(state.e[(bundle[3] >> 27) & 0xF]) orelse 0;
}

fn execX02Ainc(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w5 = bundle[5];
    // The A slot (fact 403, own-x02aa): sum = agu_xr(w2[26:22], A<w3[4:0]>, R<w5[25:22]>);
    // A<w2[21:17]> <- sum, or the Y latch yd if w5[18]; A<w3[14:10]> <- sum + A<w3[9:5]>, or sum
    // alone if w5[21].  The second add is measured for fn < 16 only.  w1 is inert.
    const fn_op: u32 = (w2 >> 22) & 0x1F;
    const dest_a: u32 = (w2 >> 17) & 0x1F;
    const dest_b: u32 = (w3 >> 10) & 0x1F;
    // Wrapped once by R<w3[18:15]> (567); the second write's add is not wrapped (626).
    const mk: u32 = (w3 >> 15) & 0xF;
    const sum = wrapOnce(state, aguXrEval(fn_op, state.a[w3 & 0x1F], state.r[(w5 >> 22) & 0xF]), mk);
    if (dest_a != 0) state.a[dest_a] = if ((w5 >> 18) & 1 == 1) state.yd else sum;
    if (dest_b != 0) {
        if ((w5 >> 21) & 1 == 1) {
            state.a[dest_b] = sum;
        } else {
            if (fn_op >= 16) return RunError.UnknownForm;
            state.a[dest_b] = (sum +% state.a[(w3 >> 5) & 0x1F]) & 0xFFFF;
        }
    }
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
        const a_idx: u32 = (bundle[4] >> 17) & 0x1F;
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
        @min(roundShr(prod, 23), 0x7FFFFF); // -1 * -1 saturates (fact 402)
    // Modes (fact 161, 733, 735): X = xd << 8 at 24 bits, xd the X word read before the bundle.
    // 1 = a + X, 2 = a - X; 5 = P + X, 6 = P - X, 7 = X - P; otherwise P.
    // 756: w1[17] = 1 shifts modes 1 / 2's X right 24, X = sx16(xd[15:0]) << 8, so only the
    // sign survives; measured with w1[16:15] = 01 only.
    const x24: i64 = @intCast((state.xd_pre << 8) & M24);
    const mode: u32 = (w1 >> 11) & 7;
    const w17 = (w1 >> 17) & 1 == 1;
    if ((mode == 1 or mode == 2) and w17 and (w1 >> 15) & 3 != 1) return RunError.UnknownForm;
    const x_sh: i64 = @as(i64, sx16(state.xd_pre & 0xFFFF)) << 8;
    const r: i64 = switch (mode) {
        1 => @as(i64, sx24(state.s[a_s])) + if (w17) x_sh >> 24 else x24,
        2 => @as(i64, sx24(state.s[a_s])) + if (w17) (-x_sh) >> 24 else -x24,
        5 => scaled + x24,
        6 => scaled - x24,
        7 => x24 - scaled,
        else => scaled,
    };
    const v: u32 = @truncate(@as(u64, @bitCast(r)) & M24);
    if (dest_s != 0) state.s[dest_s] = v;
    // x02.plo: w4[10:7] carries a second dest: (2P) mod 2^24 (fact 657), or with w1[17] = 1
    // P mod 2^24 (fact 655). Neither is masked to 16 bits; 399's product sat under 2^15.
    const w4 = bundle[4];
    const plo_dest: u32 = (w4 >> 7) & 0xF;
    if (plo_dest != 0) state.s[plo_dest] = if ((w1 >> 17) & 1 == 1)
        @truncate(@as(u64, @bitCast(prod)) & M24)
    else
        @truncate(@as(u64, @bitCast(prod *% 2)) & M24);
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
    // 745: w1[26] = 0 wedges the probe (6/6).
    if ((w1 >> 26) & 1 == 0) return RunError.UnknownForm;
    const xe_val: u32 = state.e[(w3 >> 25) & 0xF];
    const y: u32 = state.r[(w3 >> 11) & 0xF];
    const dest_r: u32 = (w3 >> 7) & 0xF;
    const r_val: u32 = if ((h >> 30) & 1 == 1)
        agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0x1F], y, 0)
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
    // 745, 746: signed 24-bit operands; w3[0] = 1 rounds >> 23, 0 truncates (as class 010, 684);
    // w3[2:0] = 101 is >> 24.  747: 100 is unsigned 24 x unsigned 24 >> 24, truncated (class 100's).
    const rnd = w3 & 1 == 1;
    const unsigned = (w3 >> 2) & 1 == 1 and !rnd;
    const av: i64 = if (a_s == 0) 0 else if (unsigned) state.s[a_s] & M24 else sx24(state.s[a_s]);
    const bv: i64 = if (b_s == 0) 0 else if (unsigned) state.s[b_s] & M24 else sx24(state.s[b_s]);
    const prod: i64 = av * bv;
    const r: i64 = if ((w3 >> 2) & 1 == 1) prod >> 24 else if (rnd) roundShr(prod, 23) else prod >> 23;
    const v: u32 = switch (mode) {
        4 => @as(u32, @truncate(@as(u64, @bitCast(r)))) & M24,
        else => return,
    };
    state.s[dest_s] = v;
}

/// x03 jumps (394, 679, 680, 682): w2[28] = 1 is jr q + sext(w1[28:18]) on p<w5[17:14]>, if set
/// with w2[27] = 1, if clear with w2[27] = 0; w2[31:29] is ignored; p0 reads 1, so k = 0 with
/// w2[27] = 1 always jumps.  w2[28:27] = 01 is unconditional (329).  Two delay slots, both run on every pass of a loop (680).
/// With w2[28] = 1 the e counter e<2*w4[2:0] + w3[31]> <- e<w3[26:23]> - 1 (349) also writes
/// p3 <- (e_new > 0, signed 16) after the jump has read p, so a jump on p3 sees it one pass late
/// (675): an offset-0 jump on p3 with e = N - 1 and p3 = 1 runs the bundle and its slots N times.
/// Measured on mode-0 rows only.
fn x03CondJump(state: *State, bundle: []const u32) RunError!void {
    const w2 = bundle[2];
    const cond = (w2 >> 28) & 1 == 1;
    const sense = (w2 >> 27) & 1 == 1;
    if (!cond and !sense) return;
    const mode: u32 = (bundle[4] >> 11) & 7;
    const off11: u32 = (bundle[1] >> 18) & 0x7FF;
    const off: i32 = @as(i32, @bitCast(off11 << 21)) >> 21;
    if (!cond) {
        // w2[28:27] = 01: unconditional, mode 0 and the tap row (329, 679, 682); a jump to
        // itself runs away.
        if (off == 0 or (mode != 0 and mode != 5)) return RunError.UnknownForm;
        state.pending_jmp_offset = off;
        return;
    }
    const k: u32 = (bundle[5] >> 14) & 0xF;
    // p0 reads 1 here whatever it holds (682).
    const p_set = k == 0 or state.p[k] != 0;
    const taken = p_set == sense;
    if (mode == 5) {
        // The tap row (672) jumps like mode 0, both senses; its counter's p3 write is not
        // seen, so a jump to itself never ends (682).
        if (off == 0) return RunError.UnknownForm;
        if (taken) state.pending_jmp_offset = off;
        return;
    }
    // the counter on modes 1-4, 6, 7 is unmeasured
    if (mode != 0) return RunError.UnknownForm;
    const e_new: u32 = (state.e[(bundle[3] >> 23) & 0xF] -% 1) & 0xFFFF;
    const ec_dest: u32 = ((bundle[4] & 7) << 1) | (bundle[3] >> 31);
    if (ec_dest != 0) state.e[ec_dest] = e_new;
    state.p[3] = @intFromBool(sx16(e_new) > 0);
    // a jump to itself on a p the counter does not write never ends
    if (off == 0 and taken and k != 3) return RunError.RunawayProgram;
    if (taken) state.pending_jmp_offset = off;
}

fn execX03Mpy(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 6) return RunError.UnknownForm;
    const w1 = bundle[1];
    const w2 = bundle[2];
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
        roundShr(prod, 23);
    if (dest_s != 0) {
        const v: i64 = switch (mode) {
            4 => p,
            // mode 5: P + X; w1[14] = 1 takes X from s<{w2[0], w1[31:29]}>
            // (fact 254 / own-x03xsn, 605); X index 0 is OPEN.
            5 => blk: {
                if ((w1 >> 14) & 1 == 0) return;
                const x_s: u32 = x03AccXIndex(bundle);
                if (x_s == 0) return RunError.UnknownForm;
                break :blk p + @as(i64, state.s[x_s] & M24);
            },
            else => return,
        };
        state.s[dest_s] = @truncate(@as(u64, @bitCast(v)) & M24);
    }
    if (try x03ProductLow(state, bundle)) |w| state.s[w.dest] = w.value;
}

/// x03's accumulate X_hi index {w2[0], w1[31:29]} (605, the store's index).
fn x03AccXIndex(bundle: []const u32) u32 {
    return ((bundle[2] & 1) << 3) | (bundle[1] >> 29);
}

/// x03's high dest beside w4 mode 5 (604, 605): s<w2[12:9]> <- the 40-bit
/// {X_hi : X_lo} op 2ab, [39:24]; X_hi = s<{w2[0], w1[31:29]}>, X_lo = s<w5[12:9]>; op by
/// mode 4 P, 5 X + P, 6 P - X, 7 X - P; w1[15] = 1 adds 2^23 first, 0 floors.  Mode 5 takes
/// X_lo unsigned (670).  Measured with
/// w1[14] = 1 and w1[17:16] = 0 only, the operands signed (0x7000 * -0x123 floors to -1).
/// The running MAC's accumulator (701): 48 bits, hi = acc[47:24] and lo = acc[23:0], both
/// 24-bit, fed back as X = sx24(hi) << 24 + lo (unsigned); 24-bit signed operands, P = 2ab.
/// The sum saturates at both ends of [-2^47, 2^47 - 1] (701, 702).
/// Modes 6 / 7 run the same acc as 2ab - X / X - 2ab (737).
fn mac48(mode: u32, x_hi: u32, x_lo: u32, a: u32, b: u32) i64 {
    const x: i64 = (@as(i64, sx24(x_hi)) << 24) + (x_lo & M24);
    const p: i64 = 2 * @as(i64, sx24(a)) * sx24(b);
    const acc: i64 = switch (mode) {
        4 => p,
        6 => p - x,
        7 => x - p,
        else => x + p,
    };
    return std.math.clamp(acc, -(@as(i64, 1) << 47), (@as(i64, 1) << 47) - 1);
}

/// x03 modes 4-7 with w1[15] = 0 and w1[17:16] = 0 (670's row): the 701 running MAC;
/// mode 4 starts it, acc = 2ab with the X registers ignored (702); 6 / 7 subtract (737).
fn x03Mac48(state: *const State, bundle: []const u32) RunError!?i64 {
    const w1 = bundle[1];
    const w2 = bundle[2];
    const mode = (w1 >> 11) & 7;
    if (mode < 4 or (w1 >> 14) & 1 == 0 or (w1 >> 15) & 7 != 0) return null;
    const a = state.s[(w2 >> 1) & 0xF];
    const b = state.s[(w2 >> 5) & 0xF];
    if (mode == 4) return mac48(4, 0, 0, a, b);
    const x_hi_s = x03AccXIndex(bundle);
    if (x_hi_s == 0) return RunError.UnknownForm;
    return mac48(mode, state.s[x_hi_s], state.s[(bundle[5] >> 9) & 0xF], a, b);
}

fn x03ProductHigh(state: *const State, bundle: []const u32) RunError!?SWrite {
    const w1 = bundle[1];
    const w2 = bundle[2];
    const dest: u32 = (w2 >> 9) & 0xF;
    if (dest == 0) return null;
    const mode: u32 = (w1 >> 11) & 0x7;
    if ((w1 >> 14) & 1 == 0 or (w1 >> 16) & 3 != 0 or mode < 4) return RunError.UnknownForm;
    if (try x03Mac48(state, bundle)) |acc| return .{ .dest = dest, .value = @truncate(@as(u64, @bitCast(acc >> 24)) & M24) };
    const x_hi_s = x03AccXIndex(bundle);
    if (mode != 4 and x_hi_s == 0) return RunError.UnknownForm;
    const a_m: i64 = sx24(state.s[(w2 >> 1) & 0xF]);
    const b_m: i64 = sx24(state.s[(w2 >> 5) & 0xF]);
    // The rounded running sum (w1[15] = 1) is measured with 16-bit magnitudes only.
    if (mode == 5 and (a_m < -0x8000 or a_m > 0x7FFF or b_m < -0x8000 or b_m > 0x7FFF)) return RunError.UnknownForm;
    const prod: i64 = a_m * b_m;
    // Mode 5's X_lo is unsigned 24 (670: X_lo = 0x900000, P = 0 gives hi 0), so (hi, lo) fed
    // back chain exactly with w1[15] = 0; modes 6 and 7 keep 605's signed reading.
    const x_lo_v: u32 = state.s[(bundle[5] >> 9) & 0xF];
    const x_lo: i64 = if (mode == 5) @as(i64, x_lo_v & M24) else sx24(x_lo_v);
    const x: i64 = if (mode == 4) 0 else (@as(i64, state.s[x_hi_s] & 0xFFFF) << 24) + x_lo;
    const acc: i64 = switch (mode) {
        4, 5 => x + prod * 2,
        6 => prod * 2 - x,
        else => x - prod * 2,
    } + if ((w1 >> 15) & 1 == 1) @as(i64, 1 << 23) else 0;
    return .{ .dest = dest, .value = @truncate(@as(u64, @bitCast(acc >> 24)) & 0xFFFF) };
}

/// x03's product low half s<w4[10:7]> (191, 603): (X op P) mod 2^24, unmasked (663), P = a * b over
/// s<w2[4:1]>, s<w2[8:5]> (<< 1 unless w1[17]), X = s<w5[12:9]>, op by mode w1[13:11]:
/// 4 P, 5 X + P, 6 P - X, 7 X - P.  w1[14] = 0 is measured for mode 4 only (own-x03plo).
fn x03ProductLow(state: *const State, bundle: []const u32) RunError!?SWrite {
    const w1 = bundle[1];
    const w2 = bundle[2];
    const dest: u32 = (bundle[4] >> 7) & 0xF;
    if (dest == 0) return null;
    const mode: u32 = (w1 >> 11) & 0x7;
    if ((w1 >> 14) & 1 == 0 and mode != 4) return RunError.UnknownForm;
    if (try x03Mac48(state, bundle)) |acc| return .{ .dest = dest, .value = @truncate(@as(u64, @bitCast(acc)) & M24) };
    const b15: u32 = (w1 >> 15) & 1;
    const b16: u32 = (w1 >> 16) & 1;
    const prod: i64 = mpyOperand(state.s[(w2 >> 1) & 0xF], b15 != b16) * mpyOperand(state.s[(w2 >> 5) & 0xF], b15 == 1);
    const pl: i64 = if ((w1 >> 17) & 1 == 1) prod else prod *% 2;
    const x: i64 = sx24(state.s[(bundle[5] >> 9) & 0xF]);
    const lv: i64 = switch (mode) {
        4 => pl,
        5 => x + pl,
        6 => pl - x,
        7 => x - pl,
        else => return RunError.UnknownForm,
    };
    return .{ .dest = dest, .value = @truncate(@as(u64, @bitCast(lv)) & M24) };
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

/// w5[19:18] = 11: X:[A<{w1[2:0], head[31:30]}>] <- s<{w2[0], w1[31:29]}>, all 24 bits
/// (fact 255 / own-x03xst, 708).  w5[18] alone stores nothing.
fn execX03XStore(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (((bundle[5] >> 18) & 0x3) != 0b11) return;
    const src_s: u32 = ((bundle[2] & 1) << 3) | ((bundle[1] >> 29) & 0x7);
    try state.x_mem.put(gpa, state.a[x03AIndex(bundle)], state.s[src_s] & M24);
}

/// x03's R slot Y read (fact 519, own-x03ry): w5[21] = 1 with w5[27:26] = 00 gives
/// R<w3[22:19]> <- Y:[e<w3[30:27]>], fresh (w5[21] = 0 keeps the stale latch).  Refused where
/// it is OPEN: beside w5[20] (a row storing Y in the same bundle read 0x1101), at e index 0 (no
/// row measured), and beside the AGU fns 10 / 14 that write the same R.  Nothing
/// else in the bundle reads R, so it lands first.
fn x03RYRead(state: *State, bundle: []const u32) RunError!void {
    const w5 = bundle[5];
    const dest_r: u32 = (bundle[3] >> 19) & 0xF;
    if (dest_r == 0 or (w5 >> 21) & 1 == 0 or (w5 >> 26) & 3 != 0) return;
    const idx: u32 = (bundle[3] >> 27) & 0xF;
    const fn_op: u32 = (bundle[1] >> 7) & 0xF;
    if ((w5 >> 20) & 1 == 1 or idx == 0 or fn_op == 10 or fn_op == 14) return RunError.UnknownForm;
    state.r[dest_r] = (state.getY(state.e[idx]) orelse 0) & 0xFFFF;
}

/// x03's R slot without w5[20] (fact 250): fn 10 y, fn 14 y + 1 (AGU10B's); other fns
/// write nothing.  w5[27]'s "+1" (265) holds only in fresh state (547): refused.
fn x03RAgu10b(state: *State, bundle: []const u32) RunError!void {
    const dest_r: u32 = (bundle[3] >> 19) & 0xF;
    if (dest_r == 0) return;
    if ((bundle[5] >> 27) & 1 == 1) return RunError.UnknownForm;
    const y_val: u32 = state.r[(bundle[5] >> 22) & 0xF];
    state.r[dest_r] = switch ((bundle[1] >> 7) & 0xF) {
        10 => y_val & 0xFFFF,
        14 => (y_val +% 1) & 0xFFFF,
        else => return,
    };
}

/// 0x03 carries 0x02's R slot (fact 250).  w5[20] = 1 switches it
/// from AGU10B to AGU10 with i = 0 (fact 252 / own-x03ragu10).
fn execX03RSlot(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if ((bundle[5] >> 20) & 1 == 0) return x03RAgu10b(state, bundle);
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
    // w5[16:14] can force an add (569, 570, 606), gated by head[22:19] with no rule: OPEN.
    if ((bundle[5] >> 14) & 7 != 0) return RunError.UnknownForm;
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
/// The multiply's low half as an R slot reads it (fact 516, 522): (a * b)[15:0], the
/// product doubled first unless w1[17].
fn productLow(a: u32, b: u32, w1: u32) u32 {
    const prod: u64 = @bitCast(@as(i64, sx24(a)) * sx24(b));
    const sh: u6 = if ((w1 >> 17) & 1 == 1) 0 else 1;
    return @truncate((prod << sh) & 0xFFFF);
}

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
        // fact 522: w5[30] = 1 in mode w1[13:11] = 4 routes the low half of
        // s<w2[8:5]> * s<w2[4:1]> (doubled unless w1[17]); otherwise the Y read.
        0 => blk: {
            if ((w5 >> 30) & 1 == 0 or (w1 >> 11) & 7 != 4) break :blk yd;
            if ((w5 >> 25) & 1 == 1) return RunError.UnknownForm;
            break :blk productLow(state.s[(w2 >> 5) & 0xF], state.s[(w2 >> 1) & 0xF], w1);
        },
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
    const x_addr: u32 = state.a[(w4 >> 17) & 0x1F];
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

/// 0x00's D (595, 550's rule): w5[4] = 1 a fresh Y:[e<w4[8:5]>] (setting yd), 0 yd.
fn x00D(state: *const State, w4: u32, w5: u32, yd_new: *?u32) u32 {
    if ((w5 >> 4) & 1 == 0) return state.yd & 0xFFFF;
    const ri: u32 = (w4 >> 5) & 0xF;
    const v = (state.getY(if (ri == 0) 0 else state.e[ri]) orelse 0) & 0xFFFF;
    yd_new.* = v;
    return v;
}

/// The 7-word 0x00 bundle and 0x01 (head[29] inert), slots latched then
/// written (fact 227-230, 234, 241-244, 286):
/// s slot s<w6[15:12]> <- mux w6[3:1] (0 / sub / sub / D << 8 / D << 7 / D / #head[26:11] / 0),
///   sub-mux w5[24:23] (#imm / 1 / the w1[10:7] op / D), D = x00D (595, 724);
/// Y store Y:[e<w4[8:5]>] <- s<w4[16:13]>, X store X:[A<w2[22:18]>] <- sx24 s<w2[13:10]>,
///   enabled by w5[5:4] / w5[2:1] = 11 on 0x00 and 8 bits higher on 0x01;
/// ALU slot s<w6[7:4]> <- ALU05[head[10:5]](s<w6[23:20]>, s<w2[5:2]>), shift w2[9:6], m w5[29:27] (725);
/// MAC slot s<w6[11:8]> <- w4[23] = 1: xd (w4[17] = 1); else acc[15:0],
///   acc = mode w1[13:11]({s<w2[13:10]> : X_lo}, 2 s<w6[27:24]> s<w6[31:28]>),
///   X_lo = sx24 s<w4[15:13]>, or xd << 8 when w4[15:13] = 0 (755);
/// multiply s<w6[19:16]> <- mode w1[13:11](X = s<w2[13:10]>, P = s*s >> 23 (24 if w1[17])
///   rounded when w1[15] = 1);
/// control w3[4:0]: jr +w1[25:18], ja w1[25:18], two delay slots.
/// The 0x01 ALU slot s<w6[7:4]> (fact 498-500): w5[3:2] = 0x is the class-05 adder,
/// alu_word with a = s<w6[23:20]>, b = s<w2[5:2]>, m = w5[29:27], shift w2[9:6]; w4[19] inert.
/// w5[3:2] = 10 (source OPEN) and 11 (yd, not modelled) are refused, as are non-add codes with
/// a or m set.
fn x01Alu(state: *const State, h: u32, w2: u32, w5: u32, w6: u32) RunError!u32 {
    if ((w5 >> 3) & 1 == 1) return RunError.UnknownForm;
    const code: u32 = (h >> 5) & 0x3F;
    const a_idx: u32 = (w6 >> 20) & 0xF;
    const m: u32 = (w5 >> 27) & 7;
    const b: i64 = sx24(state.s[(w2 >> 2) & 0xF]);
    const i: i64 = (w2 >> 6) & 0xF;
    if (code == 0x1C) return x01Xori(m, b, i);
    if (code == 0x06) return x01Asr(m, b, i, sx24(state.s[a_idx]));
    return adderSlot(state, code, w2, a_idx, m);
}

/// The 0x00 / 0x01 class-05 slot over a = s<a_idx>, b = s<w2[5:2]>, shift w2[9:6], m (499, 725):
/// adder codes take a and m; other codes with a or m set are OPEN, refused.
fn adderSlot(state: *const State, code: u32, w2: u32, a_idx: u32, m: u32) RunError!u32 {
    const adder = switch (ALU05[code]) {
        .add, .addi, .sub => true,
        else => false,
    };
    if (!adder and (a_idx != 0 or m != 0)) return RunError.UnknownForm;
    const word: u32 = (code << 5) | (((w2 >> 6) & 0xF) << 19) | (m << 23);
    return (try aluWord(word, sx24(state.s[a_idx]), sx24(state.s[(w2 >> 2) & 0xF]))) orelse RunError.UnknownForm;
}

/// 0x01 slot xori (fact 502): m over b and i = w2[9:6], a ignored.  m5 counts the
/// redundant sign bits at 24 bits (24 for 0, 23 for -1).
fn x01Xori(m: u32, b: i64, i: i64) RunError!u32 {
    const r: i64 = switch (m) {
        0 => b ^ i,
        1 => ~(b ^ i),
        2 => b | i,
        3 => b & i,
        4 => ~i,
        5 => blk: {
            // norm(-1) = 23, norm(-2) = 22 (667 follow-up)
            if (b == 0) break :blk 24;
            const v: u32 = @intCast((if (b < 0) ~b else b) & M24);
            break :blk @as(i64, @clz(v)) - 9;
        },
        6 => i,
        7 => if (b < 0) b else i,
        else => unreachable,
    };
    return @as(u32, @truncate(@as(u64, @bitCast(r)))) & M24;
}

/// 0x01 slot asr (fact 503-504, 506): ADDER_M[m](x = i, y = b) >> s<a>, arithmetic,
/// a < 0 shifting left; m7 is |b| (i ignored).  Counts >= 24 shift everything out; a left
/// shift past 24 bits wraps (measured on a positive value; negative overflow and counts below
/// -23 are refused, as are sums past 24 bits and |-2^23|).
fn x01Asr(m: u32, b: i64, i: i64, a: i32) RunError!u32 {
    if (a < -23) return RunError.UnknownForm;
    const v: i64 = if (m == 7) blk: {
        if (b == -0x800000) return RunError.UnknownForm;
        break :blk if (b < 0) -b else b;
    } else blk: {
        const t = adderM(@intCast(m), i, b, 0);
        break :blk t.x + t.y + t.c;
    };
    if (v < -0x800000 or v > 0x7FFFFF) return RunError.UnknownForm;
    if (a >= 0) return @as(u32, @truncate(@as(u64, @bitCast(v >> @intCast(@min(a, 24)))))) & M24;
    const r: i64 = v << @intCast(-a);
    if (v < 0 and r < -0x800000) return RunError.UnknownForm;
    return @as(u32, @truncate(@as(u64, @bitCast(r)))) & M24;
}

/// 0x00's asr (727, 731, 732): ADDER_M[m](x = w2[8:6], y = b) >> s<a>, 696's class-05 asr
/// (left shifts truncated, right arithmetic); w2[9] negates the count, as w5[5] does on x02.
/// Every m measured (732, 754).
fn x00Asr(count: u32, b: i64, m: u32, n: u32) RunError!u32 {
    const t = adderM(@intCast(m), n & 7, b, 0);
    const v: i64 = t.x + t.y + t.c;
    if (v < -0x800000 or v > 0x7FFFFF) return RunError.UnknownForm;
    const c: u32 = if (n & 8 != 0) (0 -% count) & M24 else count & M24;
    return alu05Shift(c, @as(u32, @truncate(@as(u64, @bitCast(v)))) & M24, .asr);
}

/// Fact 646's condition code n over one predicate p: 0 / 1 / 3 when !p, 2 / 5 / 7 when p,
/// 6 / 9 always, the rest never.  0x00's codes 0x14 / 0x23 read it on p4 (749, 751), x18 class
/// 011's S codes on p3 (752).
fn cond646(n: u32, p: bool) bool {
    return switch (n) {
        0, 1, 3 => !p,
        2, 5, 7 => p,
        6, 9 => true,
        else => false,
    };
}

/// The 0x00 class-05 slot (725, 727-729, 732): 0x01's a = s<w6[23:20]> and m = w5[29:27], n = w2[9:6].
/// 0x14 adds; 0x23 adds unshifted for n in {0, 1, 3, 9} and is a for n in {2, 4, 8, 15};
/// 0x14 splits on n too (732); 0x06 is x00Asr; 0x26 selects b when ADDER_M[m](a, b) < 0, else a.
/// Unmeasured corners refused.
fn x00Alu(state: *const State, h: u32, w2: u32, w5: u32, w6: u32) RunError!u32 {
    const code: u32 = (h >> 5) & 0x3F;
    const a_idx: u32 = (w6 >> 20) & 0xF;
    const m: u32 = (w5 >> 27) & 7;
    const n: u32 = (w2 >> 6) & 0xF;
    const a: i64 = sx24(state.s[a_idx]);
    const b: i64 = sx24(state.s[(w2 >> 2) & 0xF]);
    return switch (code) {
        0x06 => x00Asr(state.s[a_idx], b, m, n),
        // 727, 732, 749, 750: never shifts; (x, y, c) = ADDER_M[m](a, b): n in {0, 1, 3, 6, 9}
        // (646's condition table read false) x + y + c, other n ~x + ~y + c.
        0x14 => blk: {
            const t = adderM(@intCast(m), a, b, 0);
            const r: i64 = if (cond646(n, state.p[4] != 0)) t.x + t.y + t.c else ~t.x + ~t.y + t.c;
            break :blk @as(u32, @truncate(@as(u64, @bitCast(r)))) & M24;
        },
        // 729, 749: never shifts; the same n split, the other result a.
        0x23 => if (cond646(n, state.p[4] != 0)) adderSlot(state, 0, w2 & ~@as(u32, 0xF << 6), a_idx, m) else @as(u32, @truncate(@as(u64, @bitCast(a)))) & M24,
        // 728, 753: select on the adder's sign, every m; n and p4 ignored.
        0x26 => blk: {
            const t = adderM(@intCast(m), a, b, 0);
            const sum: i64 = t.x + t.y + t.c;
            if (sum < -0x800000 or sum > 0x7FFFFF) break :blk RunError.UnknownForm;
            break :blk @as(u32, @truncate(@as(u64, @bitCast(if (sum < 0) b else a)))) & M24;
        },
        else => adderSlot(state, code, w2, a_idx, m),
    };
}

fn x01Table(state: *const State, h: u32, w4: u32, w5: u32, op: u32, imm: u32) RunError!u32 {
    const y: u32 = state.r[(h >> 27) & 0xF];
    if (op == 10) return (y +% imm) & 0xFFFF;
    if (op == 11) return (y -% imm) & 0xFFFF;
    if ((w5 >> 11) & 1 == 1) return if (op == 8) imm else try agu10Eval(op, state.e[(w4 >> 5) & 0xF], y);
    return x00Op(true, op, state.e[(w4 >> 5) & 0xF], state.e[(w4 >> 1) & 0xF]);
}

fn execX00(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const w6 = bundle[6];
    const x01 = (h & 0x1F) == 0x01;
    // 0x01 also writes e<w4[12:9]>, value OPEN (fact 669): refused when nonzero.
    if (x01 and (w4 >> 9) & 0xF != 0) return RunError.UnknownForm;
    const imm: u32 = (h >> 11) & 0xFFFF;

    const s_dest: u32 = (w6 >> 12) & 0xF;
    // A fresh Y read through sub-mux 11 sets yd (after 595), committed with the writes.
    var yd_new: ?u32 = null;
    const s_val: u32 = if (s_dest == 0) 0 else blk: {
        const src: u32 = (w6 >> 1) & 7;
        const sub: u32 = (w5 >> 23) & 3;
        const op: u32 = (w1 >> 7) & 0xF;
        if (x01 and op != 2 and !(sub == 2 and (src == 1 or src == 2))) return RunError.UnknownForm;
        const sub_val: u32 = switch (sub) {
            0 => imm,
            1 => 1,
            // 0x01 (508, 509): rows 10/11 are y + i, y - i (y = R<head[30:27]>, i = head[26:11]).
            // The rest is w5[11] = 1: AGU10 (x = e<w4[8:5]>), 0: 234's table (512).
            2 => if (x01) try x01Table(state, h, w4, w5, op, imm) else try x00Op(x01, op, state.e[(w4 >> 5) & 0xF], state.e[(w4 >> 1) & 0xF]),
            3 => if (x01) return RunError.UnknownForm else x00D(state, w4, w5, &yd_new),
            else => unreachable,
        };
        break :blk switch (src) {
            0, 7 => 0,
            // 726, 729: sub-mux D sign-extends on mux 1, zero-extends on mux 2.
            1 => if (sub == 3) sx16To24(sub_val) else sub_val,
            2 => sub_val,
            // 724, 726: muxes 3/4/5 ignore w5[24:23]; 3 = D << 8, 4 = zx(D) << 7, 5 = zx(D).
            3, 4, 5 => blk2: {
                if (x01) return RunError.UnknownForm;
                const d = x00D(state, w4, w5, &yd_new);
                break :blk2 switch (src) {
                    3 => (d << 8) & M24,
                    4 => d << 7,
                    else => d,
                };
            },
            6 => imm,
            else => unreachable,
        };
    };

    const ye: u5 = if (x01) 12 else 4;
    const xe: u5 = if (x01) 9 else 1;
    const y_on = (w5 >> ye) & 3 == 3;
    const x_on = (w5 >> xe) & 3 == 3;
    const y_addr: u32 = state.e[(w4 >> 5) & 0xF];
    const y_val: u32 = state.s[(w4 >> 13) & 0xF] & 0xFFFF;
    const x_addr: u32 = state.a[(w2 >> 18) & 0x1F];
    const x_src: u32 = (w2 >> 10) & 0xF;
    const x_val: u32 = if (x01 and x_src == 0) (state.xd << 8) & M24 else state.s[x_src] & M24;

    const alu_dest: u32 = (w6 >> 4) & 0xF;
    const alu_val: u32 = if (alu_dest == 0) 0 else if (x01) try x01Alu(state, h, w2, w5, w6) else blk: {
        if ((w4 >> 19) & 1 == 1) return RunError.UnknownForm;
        break :blk try x00Alu(state, h, w2, w5, w6);
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
        // 755: w4[15:13] = 0 reads X_lo = xd << 8, not s0.
        const xl: u32 = (w4 >> 13) & 7;
        const x_lo: u32 = if (xl == 0) (state.xd << 8) & M24 else state.s[xl];
        const x40: i64 = (xs << 24) + sx24(x_lo);
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
        // 0x00 mode 0 with w4[22] (316's swap; 550): a memory word instead of the product,
        // X:[A<w2[22:18]>] (408's address, read by the bundle itself) when w4[18] = 0, else
        // Y:[e<w4[8:5]>] with w5[4]; yd (w5[4] = 0) is unmeasured.
        if (!x01 and mode == 0 and (w4 >> 22) & 1 == 1) {
            if ((w4 >> 18) & 1 == 0) break :blk state.readX(state.a[(w2 >> 18) & 0x1F]) orelse 0;
            if ((w5 >> 4) & 1 == 0) return RunError.UnknownForm;
            break :blk (state.getY(state.e[(w4 >> 5) & 0xF]) orelse 0) & 0xFFFF;
        }
        // 762: 0x00 mode 0 without w4[22] writes 0; product and X unused.
        if (!x01 and mode == 0) break :blk 0;
        // 0x01 mode 4 (496, 507): rnd(a * b >> 23), X ignored, w4[22] inert; measured with
        // w1[17:14] = 0010 and operands inside 16 bits only.
        if (x01 and mode == 4) {
            if ((w1 >> 14) & 0xF != 0b0010) return RunError.UnknownForm;
            if (sx24(state.s[(w6 >> 24) & 0xF]) != pa or sx24(state.s[(w6 >> 28) & 0xF]) != pb)
                return RunError.UnknownForm;
            break :blk @truncate(@as(u64, @bitCast(roundShr(pa * pb, 23))) & M24);
        }
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
        // q + sx15({w2[0], w1[31:18]}), two delay slots, no link (575, retracting 572-574).
        // 772 (retracting 578's hidden state): condition c = w4[31:28] tests p<c> (p0 reads 1),
        // measured under control 3 only; other controls keep c = 0.
        1, 3, 0xE, 0x11, 0x13, 0x16, 0x17 => {
            const c: u32 = bundle[4] >> 28;
            if (c != 0 and ctl != 3) return RunError.UnknownForm;
            const f15: u32 = ((w2 & 1) << 14) | ((w1 >> 18) & 0x3FFF);
            if (state.p[c] != 0) state.pending_jmp_offset = @as(i32, @bitCast(f15 << 17)) >> 17;
        },
        4 => state.pending_jmp_offset = @as(i32, @intCast(target)) - @as(i32, @intCast(state.pc)),
        // 759: 0x00 END: two delay slots run, then the normal halt; target fields inert, no link.
        6, 7 => {
            if (x01) return RunError.UnknownForm;
            state.end_pending = true;
        },
        // 315 (768, retracting 761): 0x08 / 0x0D fall through and park the next arm.  A bare 0x08
        // in an otherwise empty bundle wedged (230): refused.
        8, 0xD => {
            if (x01) return RunError.UnknownForm;
            if (w1 | w2 | w4 | w5 | w6 | (h & ~@as(u32, 0x1F)) | (w3 & ~@as(u32, 0x1F)) == 0) return RunError.UnknownForm;
            state.next_arm_parks = true;
        },
        0, 2, 5, 0xC, 0x18, 0x10, 0x12, 0x14, 0x15, 0x19, 0x1A, 0x1C => {},
        else => return RunError.UnknownForm,
    }

    // The 0x01 R slot (fact 320, 321, 425, 505): R<{w4[0], w3[31:29]}> <- w5[21] = 1:
    // AGU10[w1[10:7]](y = R<head[30:27]>, i = head[26:11]), fns 8 (i), 10 (y+i), 11 (y-i) and
    // 14 (y) measured; w5[21] = 0: Y:[e<w4[8:5]>], Y:[0] at index 0.  Reads before this
    // bundle's writes.  0x00 has the same slot (516) and three more sources: w5[26:25] = 11 the
    // immediate, 10 zero, and w5[25] = 1 in mode 4 the low 16 bits of s<w6[27:24]> *
    // s<w6[31:28]>, doubled unless w1[17].  On 0x01 the product route holds too (522); the
    // immediate and zero bits read Y there (505; OPEN).
    const r_dest: u32 = ((w4 & 1) << 3) | (w3 >> 29);
    const r_val: u32 = if (r_dest == 0) 0 else blk: {
        if ((w5 >> 21) & 1 == 0) {
            const sel: u32 = (w5 >> 25) & 3;
            if (sel == 3 and !x01) break :blk imm;
            if (sel == 2 and !x01) break :blk 0;
            if (sel == 1 and mode == 4) {
                break :blk productLow(state.s[(w6 >> 24) & 0xF], state.s[(w6 >> 28) & 0xF], w1);
            }
        }
        if ((w5 >> 21) & 1 == 1) {
            const y: u32 = state.r[(h >> 27) & 0xF];
            break :blk switch ((w1 >> 7) & 0xF) {
                8 => imm,
                10 => (y +% imm) & 0xFFFF,
                11 => (y -% imm) & 0xFFFF,
                14 => y & 0xFFFF,
                else => return RunError.UnknownForm,
            };
        }
        // 0x00 with w5[4] = 0 reads yd, the last Y read, not a fresh word (550).
        if (!x01 and (w5 >> 4) & 1 == 0) break :blk state.yd & 0xFFFF;
        const ri: u32 = (w4 >> 5) & 0xF;
        break :blk (state.getY(if (ri == 0) 0 else state.e[ri]) orelse 0) & 0xFFFF;
    };

    if (y_on) state.y_mem.put(gpa, y_addr, y_val) catch return RunError.OutOfMemory;
    if (x_on) state.x_mem.put(gpa, x_addr, x_val) catch return RunError.OutOfMemory;
    if (r_dest != 0) state.r[r_dest] = r_val;
    if (yd_new) |v| state.yd = v;
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
    // h[26] = 0 (514): s[17:16] = 10 reads Y:[e<h[8:5]>] fresh (and latches it); s[17] = 0
    // gives the stale latch.  h[26] = 1 with the store on: s[20] = 0 is AGU10B, 1 AGU10.
    const store_on = (sw >> 16) & 3 == 3;
    const v: u32 = if ((h >> 26) & 1 == 0) blk0: {
        if ((sw >> 16) & 3 == 2) {
            state.yd = (state.getY(x) orelse 0) & 0xFFFF;
        }
        break :blk0 state.yd;
    } else if (store_on and (sw >> 20) & 1 == 0) agu10bEval((h >> 17) & 0xF, x, y) else blk: {
        const i: u32 = ((sw & 0xFFF) << 4) | (h >> 28);
        break :blk switch ((h >> 17) & 0xF) {
            8 => i & 0xFFFF,
            10 => (y +% i) & 0xFFFF,
            11 => (y -% i) & 0xFFFF,
            else => |f| try agu10Eval(f, x, y),
        };
    };
    // the fresh read only runs with the store off, so the order cannot show
    if (store_on) {
        state.y_mem.put(gpa, x, state.s[(sw >> 12) & 0xF] & 0xFFFF) catch return RunError.OutOfMemory;
    }
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
    if ((bundle[1] >> 25) & 0xF == 0) return RunError.UnknownForm; // p0 write: unmeasured
    state.p[(bundle[1] >> 25) & 0xF] = v;
}

/// class-0A parallel move (fact 68, 69, 76): X:[A<[15:11]>] <- s<[19:16]>
/// with [21:20] = 11, and s<[30:26]> <- Y:[e<[8:5]>] (bit 9 set; without it
/// the address lags, OPEN).  The store reads the old s.
fn execPar(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const k: u32 = (head >> 26) & 0x1F;
    // Without bit 9 the load's address lags (OPEN): only an s0 target is let through.
    // R<[25:22]> also gets the load (fact 523, 7B368AAA writes R12); without bit 9 both
    // take the stale latch, unmodelled.
    const dest_r: u32 = (head >> 22) & 0xF;
    if ((head >> 9) & 1 == 0 and (k != 16 or dest_r != 0)) return RunError.UnknownForm;
    const xv: u32 = try stxValue(state, (head >> 16) & 0xF);
    // The X store through A0 lands at the previous bundle's immediate or result, through A31
    // at X:0 (fact 649): refused.
    const xa: u32 = (head >> 11) & 0x1F;
    if (xa == 0 or xa == 31) return RunError.UnknownForm;
    state.x_mem.put(gpa, state.a[xa], xv) catch return RunError.OutOfMemory;
    if ((head >> 9) & 1 == 0) return;
    if (k < 16) return RunError.UnknownForm;
    const yv: u32 = (state.getY(try yAddrE(state, (head >> 5) & 0xF)) orelse 0) & 0xFFFF;
    state.s[k - 16] = yv;
    if (dest_r != 0) state.r[dest_r] = yv;
}

/// mul13, low5 0x13, S class 101 with S[19] (fact 128):
/// s<S[14:11]> <- round(s<S[6:3]> * s<2*S[2:0] + head[31]> / 2^23).
/// mul13, low5 0x13 with S class 101 and S[19] (fact 128, 535): s<S[14:11]> <-
/// round(s<S[6:3]> * s<2*S[2:0] + head[31]> / 2^23) over 24-bit operands (697); the head ALU slot,
/// s<h[18:15]> <- alu_word(h, a = s<h[14:11]>, b = s<h[22:19]>); and with S[16:15] = 11
/// X:[A<h[30:26]>] <- s<S[28:25]>.  Everything reads before anything writes.  Refused:
/// S[18:17], S[23:22], S[10:7] (unmodelled), S[16:15] = 01/10 and one s for both writes.
fn execMul13(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const sw = bundle[1];
    if ((sw >> 17) & 3 != 0 or (sw >> 22) & 3 != 0 or (sw >> 7) & 0xF != 0) return RunError.UnknownForm;
    const store: bool = switch ((sw >> 15) & 3) {
        0 => false,
        3 => true,
        else => return RunError.UnknownForm,
    };
    const dest: u32 = (sw >> 11) & 0xF;
    const alu_dest: u32 = (h >> 15) & 0xF;
    if (dest != 0 and dest == alu_dest) return RunError.UnknownForm;
    // 24x24 operands (697: 0x400000 * 0x400000 gives 0x200000).
    const prod: i64 = @as(i64, sx24(state.s[(sw >> 3) & 0xF])) * sx24(state.s[2 * (sw & 7) + (h >> 31)]);
    const alu_v: ?u32 = if (alu_dest == 0) null else try aluWord(h, sx24(state.s[(h >> 11) & 0xF]), sx24(state.s[(h >> 19) & 0xF])) orelse
        return RunError.UnknownForm;
    if (store) state.x_mem.put(gpa, state.a[(h >> 26) & 0x1F], state.s[(sw >> 25) & 0xF] & M24) catch return RunError.OutOfMemory;
    state.s[dest] = @truncate(@as(u64, @bitCast(roundShr(prod, 23))) & M24);
    if (alu_v) |v| state.s[alu_dest] = v;
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

fn execX03Agu(gpa: std.mem.Allocator, state: *State, bundle: []const u32, e_pre: [16]u32) RunError!void {
    _ = gpa;
    const w2 = bundle[2];
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const fn_op: u32 = (w2 >> 22) & 0x1F;
    const dest_a: u32 = (w3 >> 10) & 0x1F;
    const x_val: u32 = state.a[w3 & 0x1F];
    const i_val: u32 = ((w5 & 1) << 15) | ((w4 >> 17) & 0x7FFF);
    // Second A dest (fact 193): A<w2[21:17]> <- fn(A<w3[4:0]>) by w2[26:22], fn 5 = x (670),
    // fn 0 = x + 1 (672, the one-bundle tap's pointer step), fns 1-3 agu_xr's x - 1, x + r,
    // x - r with r = R<w5[25:22]> (740, 741); other fns unmeasured. With w5[5] = 1 it is the
    // Y read Y:[e<w3[30:27]>].
    const acp_dest: u32 = (w2 >> 17) & 0x1F;
    const acp_val: u32 = if ((w5 >> 5) & 1 == 1) (state.getY(state.e[(w3 >> 27) & 0xF]) orelse 0) & 0xFFFF else switch (fn_op) {
        5 => x_val,
        0 => (x_val +% 1) & M24,
        1, 2, 3 => aguXrEval(fn_op, x_val, state.r[(w5 >> 22) & 0xF]),
        else => if (acp_dest != 0) return RunError.UnknownForm else 0,
    };
    // AGU table "5" = x + i (fact 164).
    if (dest_a != 0) {
        if (fn_op != 5) return RunError.UnknownForm;
        state.a[dest_a] = (x_val +% i_val) & M24;
    }
    if (acp_dest != 0 and acp_dest != dest_a) state.a[acp_dest] = acp_val;
    // The e counter (fact 349): e<2 * w4[2:0] + w3[31]> <- e<w3[26:23]> - 1, the source read
    // before the bundle; e0 as the destination is off.  Measured on the mode-5 A copy.
    const ec_dest: u32 = ((w4 & 7) << 1) | (w3 >> 31);
    if (ec_dest != 0) state.e[ec_dest] = (e_pre[(w3 >> 23) & 0xF] -% 1) & 0xFFFF;
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
    const a_idx: u32 = ((a_hi << 2) | a_lo) & 0x1F;
    // w5[1] = 1: X:[A<4*w1[2:0] + h[31:30]>] whatever w5[21] says (565).  w5[1] = 0: w5[21] = 1
    // reads Y:[e<w3[30:27]>] (190 / own-x03yrd), 0 delivers yd (565).
    const w5 = bundle[5];
    const fresh = (w5 >> 1) & 1 == 0 and (w5 >> 21) & 1 == 1;
    const v: u32 = if ((w5 >> 1) & 1 == 1)
        state.readX(state.a[a_idx]) orelse 0
    else if (fresh)
        (state.getY(state.e[(bundle[3] >> 27) & 0xF]) orelse 0) & 0xFFFF
    else
        state.yd;
    if (fresh) state.yd = v;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// x02's third slot (fact 182, 549): s<{w5[1:0], w4[31:30]}> <- alu_word over the head
/// ALU code h[10:5] with m = w5[8:6] and #n = w5[5:2], a = s<w4[25:22]>, b = s<w4[29:26]>,
/// read before the bundle's slots write (`s_pre`).  Only adder codes are measured: codes 0-2
/// shift left, 3/5 right, 4 right rounded, the rest unshifted; non-adder codes are OPEN.
fn execX02X3Pre(gpa: std.mem.Allocator, state: *State, bundle: []const u32, s_pre: [16]u32) RunError!void {
    _ = gpa;
    const w4 = bundle[4];
    const w5 = bundle[5];
    const dest_s: u32 = ((w5 & 3) << 2) | (w4 >> 30);
    if (dest_s == 0) return;
    const code: u32 = (bundle[0] >> 5) & 0x3F;
    if (ALU05[code] == .add) {
        state.s[dest_s] = try x02X3Value(bundle, s_pre);
        return;
    }
    state.s[dest_s] = try x02X3Other(code, w5, s_pre[(w4 >> 22) & 0xF] & M24, s_pre[(w4 >> 26) & 0xF] & M24);
}

/// x02 third slot XOR_M[w5[8:6]] (628); f5 OPEN.
const X02_XOR_M = [8]?enum { xor, xnor, or_, and_, not_a, a }{ .xor, .xnor, .or_, .and_, .not_a, null, .a, .a };

/// The third slot's non-adder head codes (627, 628), w5[16] = 0 only: 6 asr and 7 asrrs of
/// b by a (w5[5] negates a), 22 / 31 XOR_M (22 then << w5[5:2]), 32 / 34 clear.
fn x02X3Other(code: u32, w5: u32, a: u32, b: u32) RunError!u32 {
    if ((w5 >> 16) & 1 == 1) return RunError.UnknownForm;
    const n: u32 = if ((w5 >> 5) & 1 == 1) (0 -% a) & M24 else a;
    return switch (code) {
        6 => if ((w5 >> 2) & 7 != 0) RunError.UnknownForm else alu05Shift(n, b, .asr),
        7 => alu05Shift(n, b, .asrrs),
        22, 31 => blk: {
            const v: u32 = switch (X02_XOR_M[(w5 >> 6) & 7] orelse return RunError.UnknownForm) {
                .xor => a ^ b,
                .xnor => ~(a ^ b),
                .or_ => a | b,
                .and_ => a & b,
                .not_a => ~a,
                .a => a,
            };
            const sh: u5 = if (code == 22) @intCast((w5 >> 2) & 0xF) else 0;
            break :blk (v << sh) & M24;
        },
        32, 34 => 0,
        else => RunError.UnknownForm,
    };
}

/// The third slot's alu_word value (182, 549), also the R slot's source with w5[31:30] = 10.
fn x02X3Value(bundle: []const u32, s: [16]u32) RunError!u32 {
    const w4 = bundle[4];
    const w5 = bundle[5];
    const code: u32 = (bundle[0] >> 5) & 0x3F;
    const word: u32 = (code << 5) | (((w5 >> 6) & 7) << 23) | (((w5 >> 2) & 0xF) << 19);
    return try aluWord(word, sx24(s[(w4 >> 22) & 0xF]), sx24(s[(w4 >> 26) & 0xF])) orelse RunError.UnknownForm;
}

/// x02's R slot with w5[31:30] = 10 (594): the third slot's value, measured for head codes 0,
/// 3 and 0xE (a shift only with code 0), every m = w5[8:6].
fn x02RFromX3(state: *const State, bundle: []const u32) RunError!u32 {
    const w5 = bundle[5];
    const code: u32 = (bundle[0] >> 5) & 0x3F;
    const sh: u32 = (w5 >> 2) & 0xF;
    const code_ok = code == 0 or ((code == 3 or code == 0xE) and sh == 0);
    if (!code_ok) return RunError.UnknownForm;
    return try x02X3Value(bundle, state.s);
}

/// x02's w4 slot, s<w4[6:3]> <- by mode w4[13:11] (fact 186, 187, 191, 545, 548): 101
/// memory, X:[A<w4[21:17]>] when w5[14], else the Y side; 001 / 010 by the w5[29:28] override:
/// 00 the mode's own source (001 #head[26:11], 010 e<w3[30:27]>), 10 x01's T[w1[10:7]] (a =
/// e<w3[30:27]>, y = R<w5[25:22]>, b = e<w3[26:23]>; fns 0-2 seen), 01 A<w3[4:0]> + 1 (010 only;
/// 001 OPEN), 11 the Y side.
fn execX02Xrd(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const w3 = bundle[3];
    const w4 = bundle[4];
    const w5 = bundle[5];
    const mode: u32 = (w4 >> 11) & 7;
    const e_src: u32 = state.e[(w3 >> 27) & 0xF];
    const v: u32 = if (mode == 5) blk: {
        if ((w5 >> 14) & 1 == 1) {
            // 739: w5[10:9] gates the read: 01 reads X memory; 00 / 10 / 11 (the store, 730) give xd.
            if ((w5 >> 9) & 3 != 1) break :blk state.xd;
            break :blk state.readX(state.a[(w4 >> 17) & 0x1F]) orelse 0;
        }
        break :blk x02YSide(state, bundle);
    } else switch ((w5 >> 28) & 3) {
        0 => if (mode == 1) (bundle[0] >> 11) & 0xFFFF else e_src,
        2 => switch ((bundle[1] >> 7) & 0xF) {
            0 => e_src,
            1 => state.r[(w5 >> 22) & 0xF],
            2 => state.e[(w3 >> 23) & 0xF],
            else => return RunError.UnknownForm,
        },
        1 => if (mode == 2) (state.a[w3 & 0x1F] +% 1) & M24 else return RunError.UnknownForm,
        else => x02YSide(state, bundle),
    };
    // Read into s0, the word reaches no other slot of the bundle: s0 stays 0 (fact 652).
    const dest_s: u32 = (w4 >> 3) & 0xF;
    if (dest_s != 0) state.s[dest_s] = v;
}
fn execX12Aset(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s = bundle[1];
    const dest_a: u32 = 2 * (s & 0x7);
    const src_r: u32 = (head >> 13) & 0xF;
    // fact 522: S[10:8] = 101 also stores X:[A<S[19:15]>] <- sx24 s<S[7:4]>, the address
    // read before this bundle's A write.  Source index 0 is OPEN (neither s0 nor a constant).
    if ((s >> 8) & 7 == 5) {
        const src_s: u32 = (s >> 4) & 0xF;
        if (src_s == 0) return RunError.UnknownForm;
        const addr: u32 = state.a[(s >> 15) & 0x1F];
        state.x_mem.put(gpa, addr, state.s[src_s] & M24) catch return RunError.OutOfMemory;
    }
    if (dest_a == 0) return;
    state.a[dest_a] = if (src_r == 0) 0 else state.r[src_r];
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
/// the Python ISA model).  Codes outside the measured set yield
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
        // left.  fact 33, the Python ISA model `shift`.
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
    // 774: n >= 24 gives the sign (asr) or 0 (lsr; asrrs, whose half-up bias reaches 2^23).
    if (kind == .asrrs and n >= 24) return 0;
    var vv: i64 = v_signed;
    if (kind == .asrrs and n > 0) vv += @as(i64, 1) << @as(u6, @intCast(n - 1));
    const shift: u6 = @intCast(@min(n, 24));
    const shifted: i64 = vv >> shift;
    return @as(u32, @truncate(@as(u64, @bitCast(shifted)))) & M24;
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
        try state.x_mem.put(gpa, state.a[(s2 >> 4) & 0x1F], state.s[(s1 >> 22) & 0xF] & M24);
    }
    if (dest_s == 0) return;
    // head[28] = 0: the multiply slot writes 0 (fact 121).
    if ((head >> 28) & 1 == 0) {
        state.s[dest_s] = 0;
        return;
    }
    // 24-bit signed operands, 24-bit result (698).
    const av: i64 = if (a_s == 0) 0 else sx24(state.s[a_s]);
    const bv: i64 = if (b_s == 0) 0 else sx24(state.s[b_s]);
    const prod: i64 = av * bv;
    const round_bias: i64 = if ((head >> 30) & 1 == 1) @as(i64, 1) << 22 else 0;
    const shifted: i64 = (prod + round_bias) >> 23;
    const v: i64 = if (half) shifted >> 1 else shifted;
    state.s[dest_s] = @as(u32, @truncate(@as(u64, @bitCast(v)))) & M24;
}

/// S1 class != 000 with S2 class 101: head[28] runs the multiply,
/// s<4*S2[1:0] + S1[31:30]> <- s<S1[4:1]> * s<S2[5:2]> >> 23 (fact 212).
fn execX16AguMul(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const dest: u32 = ((s2 & 3) << 2) | ((s1 >> 30) & 3);
    // S1[30:28] = 000 also runs the 120 form's s slot, both reading the old s (638).
    const slot = if ((s1 >> 28) & 7 == 0) try x16Agu2Slot(state, bundle) else null;
    const a: i64 = sx16(state.s[(s1 >> 1) & 0xF]);
    const b: i64 = sx16(state.s[(s2 >> 2) & 0xF]);
    const prod: u32 = if ((h >> 28) & 1 == 0) 0 else @truncate(@as(u64, @bitCast((a * b) >> 23)) & 0xFFFF);
    if (slot) |w| {
        if (w.dest != 0 and w.dest == dest) return RunError.UnknownForm;
        if (w.dest != 0) state.s[w.dest] = w.val;
    }
    if (dest != 0) state.s[dest] = prod;
}

const AGU10B_TAB = [16]u8{ 0, 1, 2, 3, 4, 5, 6, 7, 2, 2, 1, 1, 1, 8, 9, 10 };

/// AGU10B (fact 120, S2[8] = 0): x, y, 0, -x, -y, 1, x+y, x-y, 0, 0,
/// y, y, y, -y, y+1, y-1.
const Agu10QOp = enum { x, y, q, neg_x, neg_y, q_inc, add, x_sub_y, zero, y_sub_q, q_sub_y, y_sub_q_inc, y_sub_q_dec };

/// AGU10Q (fact 636), 16 of 16 measured; AGU10B (120) is this table with q = 0.
const AGU10Q = [16]Agu10QOp{ .x, .y, .q, .neg_x, .neg_y, .q_inc, .add, .x_sub_y, .zero, .zero, .y, .y, .y_sub_q, .q_sub_y, .y_sub_q_inc, .y_sub_q_dec };

fn agu10qEval(fn_op: u32, x: u32, y: u32, q: u32) u32 {
    const r: u32 = switch (AGU10Q[fn_op & 0xF]) {
        .x => x,
        .y => y,
        .q => q,
        .neg_x => 0 -% x,
        .neg_y => 0 -% y,
        .q_inc => q +% 1,
        .add => x +% y,
        .x_sub_y => x -% y,
        .zero => 0,
        .y_sub_q => y -% q,
        .q_sub_y => q -% y,
        .y_sub_q_inc => y -% q +% 1,
        .y_sub_q_dec => y -% q -% 1,
    };
    return r & 0xFFFF;
}

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
    if (bundle[2] & 3 != 0) return RunError.UnknownForm;
    const w = try x16Agu2Slot(state, bundle);
    if (w.dest != 0) state.s[w.dest] = w.val;
}

/// The 120 form's s slot, s<S1[12:9]> <- mux S1[17:15]; also runs beside agumul (638).
fn x16Agu2Slot(state: *State, bundle: []const u32) RunError!struct { dest: u32, val: u32 } {
    const s1 = bundle[1];
    const s2 = bundle[2];
    const mux: u32 = (s1 >> 15) & 7;
    if ((s1 >> 13) & 1 == 0 or (s1 >> 30) & 1 == 1) return RunError.UnknownForm;
    const fn_op: u32 = (s1 >> 26) & 0xF;
    const x: u32 = state.e[(s2 >> 21) & 0xF];
    const y: u32 = state.r[(s1 >> 18) & 0xF];
    const dest: u32 = (s1 >> 9) & 0xF;
    const v: u32 = switch (mux) {
        // 610: mux 0's 0 reads the head code h[10:5] somehow (code 12 gave 1): only code 14's 0
        // is measured.
        0 => if (dest != 0 and (bundle[0] >> 5) & 0x3F != 14) return RunError.UnknownForm else 0,
        1 => if ((s2 >> 8) & 1 == 1) try agu10Eval(fn_op, x, y) else agu10bEval(fn_op, x, y),
        // 638: Y:[x][7:0] << 8; no gate seen (not swept).
        3 => ((state.getY(x) orelse 0) & 0xFF) << 8,
        else => return RunError.UnknownForm,
    };
    return .{ .dest = dest, .val = v };
}

/// S1 class 011, S2 class 000: S2[3] latches xi <- S2[8:4], then
/// s<S1[31:28]> <- X:[A<xi>][15:0] (fact 216).
fn execX16Xld(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 3) & 1 == 1) state.x16_xi = (s2 >> 4) & 0x1F;
    const xi = state.x16_xi orelse return RunError.UnknownForm;
    const dest: u32 = (s1 >> 28) & 0xF;
    const addr: u32 = if (xi == 0) 0 else state.a[xi & 0x1F];
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
    if ((s2 >> 13) & 3 == 3) try state.x_mem.put(gpa, state.a[(s2 >> 8) & 0x1F], xv);
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
    // S1[24] picks agu_xr for R<S2[16:13]>, wrapped once by R<S2[5:2]> (568).
    const rd: u32 = if ((s1 >> 24) & 1 == 1) (s2 >> 13) & 0xF else 0;
    const rv: u32 = if (rd == 0) 0 else wrapOnce(state, aguXrEval((s2 >> 6) & 0x1F, state.a[(s1 >> 25) & 0x1F], state.r[((s2 & 3) << 2) | (s1 >> 30)]), (s2 >> 2) & 0xF);
    try x16Alu(state, s1, s2);
    if (rd != 0) state.r[rd] = rv;
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
    // 011 = Y:[x][7:0] << 8 (refused, 488), 100 = (Y:[x] << 7)[15:0], 110 = head[26:11] << 6.
    const mux: u32 = (s1 >> 28) & 7;
    const yd: u32 = (state.getY(x_val) orelse 0) & 0xFFFF;
    const v: u32 = switch (mux) {
        1 => switch (fn_op) {
            10 => (y_val +% ((head >> 11) & 0xFFFF)) & 0xFFFF,
            8, 11 => return RunError.UnknownForm,
            else => try agu10Eval(fn_op, x_val, y_val),
        },
        5 => yd,
        // 488: mux 3 varies from run to run on silicon: refused.
        4 => shl7(yd),
        6 => (((head >> 11) & 0xFFFF) << 6) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    const alu = try x16AguAlu(state, head, s1, s2);
    if (alu) |w| if (dest_s != 0 and dest_s == w.dest) return RunError.UnknownForm;
    if (dest_s != 0) state.s[dest_s] = v;
    if (alu) |w| state.s[w.dest] = w.val;
}

/// The x16agu's 0x05 ALU word (fact 592): s<S1[27:24]> <- alu_word(code h[10:5],
/// a = s<S1[19:16]>, b = s<S2[28:25]>, m = S1[13:11], #s = S1[23:20]); 0 means no write.
/// Measured codes only: add 0 / 1 at m 0/2/3/6/7 (with #s only at m 2); at m 2 with #s = 0 sub
/// 16, asr 6, mova 35, sel 42, movb 51, clr 32 / 34.  Read 16-bit, so the top byte is the
/// alu_word's.
fn x16AguAlu(state: *const State, h: u32, s1: u32, s2: u32) RunError!?struct { dest: u32, val: u32 } {
    const d: u32 = (s1 >> 24) & 0xF;
    if (d == 0) return null;
    const code: u32 = (h >> 5) & 0x3F;
    const m: u32 = (s1 >> 11) & 7;
    const sh: u32 = (s1 >> 20) & 0xF;
    const ok = switch (code) {
        0, 1 => (m == 0 or m == 2 or m == 3 or m == 6 or m == 7) and (sh == 0 or m == 2),
        16, 6, 35, 42, 51, 32, 34 => m == 2 and sh == 0,
        else => false,
    };
    if (!ok) return RunError.UnknownForm;
    const a = sx24(state.s[(s1 >> 16) & 0xF]);
    const b = sx24(state.s[(s2 >> 25) & 0xF]);
    // m 2 left the non-adder ops as they are at m 0 (592): evaluate them unmodified.
    if (ALU05[code] != .add and ALU05[code] != .sub) return .{ .dest = d, .val = alu05Eval(ALU05[code], a, b) };
    const word: u32 = (code << 5) | (sh << 19) | (m << 23);
    const v = (try aluWord(word, a, b)) orelse return RunError.UnknownForm;
    return .{ .dest = d, .val = v };
}

/// op = S2[20:16]: op & 1 multiplies 16-bit signed, op & 4 rounds; even
/// ops give 0 (fact 118).
/// x16mul, S1 class 000 + S2 class 110 (fact 118, 213, 538): s<S2[13:10]> <- the product
/// of s<S2[4:1]> and s<S2[8:5]> (16-bit) >> 23, rounded with op S2[20:16] bit 2, 0 unless bit
/// 0; s<h[24:21]> <- #h[20:5] when S1[14:13] is 00, 01 (639) or 11; with S1[28] and S2[9],
/// Y:[e<S2[24:21]>] <- s<S1[27:24]>, read before the immediate lands.  Refused: S1[14:13] = 10, an s0 store
/// source (0xA400, OPEN), S1[11:8] (a gated s write, source OPEN) and one s for both writes.
fn execX16Mul(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 3) return RunError.UnknownForm;
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s1 >> 8) & 0xF != 0) return RunError.UnknownForm;
    if ((s1 >> 13) & 3 == 2) return RunError.UnknownForm;
    const store: bool = (s1 >> 28) & 1 == 1 and (s2 >> 9) & 1 == 1;
    const src: u32 = (s1 >> 24) & 0xF;
    if (store and src == 0) return RunError.UnknownForm;
    const mul_dest: u32 = (s2 >> 10) & 0xF;
    const imm_dest: u32 = (h >> 21) & 0xF;
    if (mul_dest != 0 and mul_dest == imm_dest) return RunError.UnknownForm;
    const op: u32 = (s2 >> 16) & 0x1F;
    // 24-bit signed operands, 24-bit result (698).
    const prod: i64 = @as(i64, sx24(state.s[(s2 >> 1) & 0xF])) * sx24(state.s[(s2 >> 5) & 0xF]);
    const v: u32 = if (op & 1 == 0) 0 else @truncate(@as(u64, @bitCast(if (op & 4 != 0) roundShr(prod, 23) else prod >> 23)) & M24);
    if (store) state.y_mem.put(gpa, try yAddrE(state, (s2 >> 21) & 0xF), state.s[src] & 0xFFFF) catch return RunError.OutOfMemory;
    state.s[mul_dest] = v;
    state.s[imm_dest] = (h >> 5) & 0xFFFF;
}

fn sx16(v: u32) i64 {
    const w: u32 = v & 0xFFFF;
    return if (w & 0x8000 != 0) @as(i64, w) - 0x10000 else @as(i64, w);
}

/// s<S1[11:8]> <- X16_ALU[S1[15]][S1[18:16]](s<S2[28:25]>, s<S1[7:4]>)
/// (fact 116, 119).  Ops 0-3 wedge silicon.
fn x16Alu(state: *State, s1: u32, s2: u32) RunError!void {
    const v = try x16AluOp(s1, state.s[(s2 >> 25) & 0xF], state.s[(s1 >> 4) & 0xF]);
    const dest: u32 = (s1 >> 8) & 0xF;
    // 24 bits wide (697: 0x007FFF - 0xFF8000 = 0x00FFFF, no 16-bit wrap).
    if (dest != 0) state.s[dest] = v & M24;
}

/// X16_ALU[S[15]][S[18:16]](a, b), unmasked (fact 116, 119; psel too, 126).  Ops 0-3
/// wedge silicon.
fn x16AluOp(sw: u32, a: u32, b: u32) RunError!u32 {
    const op: u32 = (sw >> 16) & 7;
    if (op < 4) return RunError.UnknownForm;
    return if ((sw >> 15) & 1 == 0) switch (op) {
        4 => a +% b,
        5 => a,
        6 => a -% 1,
        else => 0 -% a,
    } else switch (op) {
        4 => a -% b,
        5 => a +% 1,
        else => a,
    };
}

/// S2 class 111: S2[13] latches xi <- S2[12:8]; source S1[14:12] = 101 with
/// S2[19] reads X:[A<xi>] into s<head[24:21]> (fact 116).
fn execX16AluX(state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 13) & 1 == 1) state.x16_xi = (s2 >> 8) & 0x1F;
    try x16AluPost(state, (bundle[0] >> 25) & 0x3F, s1, s2);
    const src: u32 = (s1 >> 12) & 7;
    // mux 1 / 5 with S2[18] (and not S2[19]): Y:[e<S2[24:21]>] fresh, not yd (521, 15 of 15)
    if ((src == 1 or src == 5) and (s2 >> 18) & 3 == 1) {
        const dest_y: u32 = (bundle[0] >> 21) & 0xF;
        const v: u32 = (state.getY(state.e[(s2 >> 21) & 0xF]) orelse 0) & 0xFFFF;
        if (dest_y != 0) state.s[dest_y] = v;
        state.yd = v; // 571, own-ydx16
        return;
    }
    if (src != 5 or (s2 >> 19) & 1 == 0) return RunError.UnknownForm;
    const xi = state.x16_xi orelse return RunError.UnknownForm;
    if (xi == 0) return RunError.UnknownForm;
    const dest: u32 = (bundle[0] >> 21) & 0xF;
    if (dest != 0) state.s[dest] = (state.readX(state.a[xi & 0x1F]) orelse 0) & 0xFFFF;
}

/// x16.getb, the stream read (fact 783): A<2*S1[3:0] + 1> <- the next S2[7:4] + 1 stream bits,
/// MSB first, right-aligned.  head[30:25] = 0x21 or 0x22 writes 0 and reads nothing.  Before the
/// ld0e enable a read gives 0 and does not advance; with too few bits queued the program stops
/// at the read until the host queues more (784).  Admitted: the measured shape, head[24:5] = 0,
/// S1 only [18] and [3:0] (index 7 and up wrote none of the odd A, refused), S2[28:13] = 0
/// (S2[12:8] and [3:0] inert), 1 to 13 bits.  head[24:21]'s s write beside the read is refused.
fn execStreamRead(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((h >> 5) & 0xFFFFF != 0) return RunError.UnknownForm;
    if (s1 & ~@as(u32, 0x0004000F) != 0) return RunError.UnknownForm;
    if (s2 & 0x1FFFE000 != 0) return RunError.UnknownForm;
    const k: u32 = s1 & 0xF;
    if (k >= 8) return RunError.UnknownForm;
    const count: usize = ((s2 >> 4) & 0xF) + 1;
    if (count > 13) return RunError.UnknownForm;
    const dest: u32 = 2 * k + 1;
    if ((h >> 25) & 0x3F != 0x20 or !state.stream_on) {
        state.a[dest] = 0;
        return;
    }
    if (state.stream_bit + count > state.stream.items.len * 8) {
        state.stream_starved = true;
        return;
    }
    var v: u32 = 0;
    for (0..count) |_| {
        const b = state.stream_bit;
        const byte = state.stream.items[b >> 3];
        v = (v << 1) | ((byte >> @intCast(7 - (b & 7))) & 1);
        state.stream_bit += 1;
    }
    state.a[dest] = v;
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
    if (h >> 31 == 1 and (s1 >> 3) & 1 == 0) state.a[2 * (s1 & 7) + 1] = (h >> 5) & 0xFFFF;
    if (dest == 0) return;
    const src: u32 = (s1 >> 12) & 7;
    const v: u32 = switch (src) {
        5 => blk: {
            const xi = state.x16_xi orelse return RunError.UnknownForm;
            const addr: u32 = if (xi == 0) 0 else state.a[xi & 0x1F];
            break :blk (state.readX(addr) orelse 0) & 0xFFFF;
        },
        7 => 0,
        6 => if (imm_mux) (h >> 5) & 0xFFFF else return RunError.UnknownForm,
        // S1[26:25] = 11 with mux 011: X:[A<xi>][7:0] << 8 (fact 217).
        3 => if (imm_mux) blk: {
            const xi = state.x16_xi orelse return RunError.UnknownForm;
            const addr: u32 = if (xi == 0) 0 else state.a[xi & 0x1F];
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

/// The x18 class-110 p write (fact 469, 470, 471, 476): p<w2[16:13]> <- fn of
/// p<w3[17:14]>, picked by code h[28:25], op w1[17:15], f w3[28:25] and w3[9:8].  Measured:
/// w3[9:8] != 0 under op 0/2, and codes 0-10 under op 0, give 1; code 14 under op 0/2 maps f
/// 0, 4, 10-15 to 1, 1, 3, 8 to p, 2, 5 to !p, 6, 7, 9 to 0; code 11 under op 0/2 maps f 0, 1,
/// 5, 6, 12 to 1 and 2, 3, 9, 10 to 0; code 11 under op 1 a signed relation of two s (706).
/// Every other cell is refused.
fn x18PWrite(state: *State, bundle: []const u32) RunError!void {
    const dest: u32 = (bundle[2] >> 13) & 0xF;
    if (dest == 0) return;
    const w3 = bundle[3];
    const code: u32 = (bundle[0] >> 25) & 0xF;
    const op: u32 = (bundle[1] >> 15) & 7;
    const f: u32 = (w3 >> 25) & 0xF;
    const p: u8 = state.p[(w3 >> 14) & 0xF];
    const op02 = op == 0 or op == 2;
    const v: u8 = blk: {
        if (op02 and (w3 >> 8) & 3 != 0) break :blk 1;
        if (op == 0 and code <= 10) break :blk 1;
        if (op02 and code == 14) break :blk switch (f) {
            0, 4, 10...15 => 1,
            1, 3, 8 => p,
            2, 5 => @intFromBool(p == 0),
            else => 0,
        };
        // 706: op 1 (the subtract compare) under code 11 relates a = s<w3[6:3]> and
        // b = s<w1[7:4]>, signed, by w3[26:25]: 00 !=, 01 ==, 10 <=, 11 >=.  Measured with
        // w3[28:27] = 01 and w3[9:8] = 0 only.
        if (op == 1 and code == 11) {
            if ((w3 >> 27) & 3 != 1 or (w3 >> 8) & 3 != 0) return RunError.UnknownForm;
            const a = sx24(state.s[(w3 >> 3) & 0xF]);
            const b = sx24(state.s[(bundle[1] >> 4) & 0xF]);
            break :blk @intFromBool(switch ((w3 >> 25) & 3) {
                0 => a != b,
                1 => a == b,
                2 => a <= b,
                else => a >= b,
            });
        }
        if (op02 and code == 11) break :blk switch (f) {
            0, 1, 5, 6, 12 => 1,
            2, 3, 9, 10 => 0,
            else => return RunError.UnknownForm,
        };
        return RunError.UnknownForm;
    };
    state.p[dest] = v;
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
    // w3[24] = 0 doubles the product first (fact 469: 0x123 * 0x123 gave lo 0x9592,
    // hi 0); only a hi of 0 is measured there.
    const integer = (bundle[3] >> 24) & 1 == 1;
    const pp: u64 = if (integer) prod else prod << 1;
    // 24 bits, unmasked (fact 663, doubled mode: 2P = 0x2AC00 reads 0x02AC00); 167 and 469
    // read [15:0] only. Integer mode taken as P mod 2^24 like x02's (655).
    const lo: u32 = @truncate(pp & M24);
    // Doubled mode (600): hi = P[39:24] of the signed product, w3[22] = 1 rounding half up,
    // 0 flooring.  Integer mode (after 600): no rounding; w3[22] = 1 a signed product,
    // 0 unsigned 24-bit (b measured negative, a only positive: a negative a refuses).
    const hi: u32 = if (integer) blk: {
        if ((bundle[3] >> 22) & 1 == 0) {
            if (hi_dest != 0 and sx24(state.s[a_s]) < 0) return RunError.UnknownForm;
            break :blk @truncate((pp >> 24) & M24);
        }
        const sp: i64 = @as(i64, sx24(state.s[a_s])) * sx24(state.s[b_s]);
        break :blk @as(u32, @truncate(@as(u64, @bitCast(sp >> 24)))) & M24;
    } else blk: {
        const sp: i64 = (@as(i64, sx24(state.s[a_s])) * sx24(state.s[b_s])) << 1;
        const half: i64 = if ((bundle[3] >> 22) & 1 == 1) 1 << 23 else 0;
        break :blk @as(u32, @truncate(@as(u64, @bitCast((sp + half) >> 24)))) & M24;
    };
    // The hi output follows the mode w3[20:18] (687, 689) over acc[47:24]: 0 zero, 1 a + X,
    // 2 a - X, 3 ~X (never rounded), 4 P, 5 P + X, 6 P - X, 7 X - P; 16 bits wide.  X is
    // picked by w3[21] and w3[13:10] below (711-714).
    const mode: u32 = (bundle[3] >> 18) & 7;
    const w21 = (bundle[3] >> 21) & 1 == 1;
    const w23 = (bundle[3] >> 23) & 1 == 1;
    // The X terms (711-718).  Index X: s<{w2[0], w1[31:29]}> (599's store index, 715); field
    // X: s<w3[13:10]>.  Either one at 0 is s0, the incoming xd latch << 8 (709, 716, 717).
    // hi takes the index X under w3[21] = 1 and the field X under w3[21] = 0 (711), except
    // mode 3, which takes the index X either way (718; 712's field-blind 0x0001).  lo takes
    // the field X.  hi terms sit at acc[47:24] (a, X) beside P = 2ab; lo terms are 24-bit (b,
    // X) beside P mod 2^24.
    const x_reg: u32 = (bundle[3] >> 10) & 0xF;
    const xs_idx: u32 = ((w2 & 1) << 3) | (bundle[1] >> 29);
    const p_only = mode == 4 and !w23;
    const uses_x = (hi_dest != 0 or lo_dest != 0) and !p_only;
    if (uses_x and (xs_idx == 0 or x_reg == 0) and state.xd > 0xFFFF) return RunError.UnknownForm;
    const xs_val: u32 = if (xs_idx == 0) (state.xd << 8) & M24 else state.s[xs_idx];
    const xf_val: u32 = if (x_reg == 0) (state.xd << 8) & M24 else state.s[x_reg];
    // w3[23] = 1 is a second table (718, 719, 721), measured in every form and mode.
    const lo_m: u32 = if (lo_dest == 0 or p_only) lo else blk: {
        const b_lo: u32 = state.s[b_s] & M24;
        const v: u32 = if (integer) (if (w23) switch (mode) {
            // 719, 721: integer; w3[21] only moves mode 0, which is 2 * (P + Xf) under
            // w3[21] = 1 (721, 722).
            0 => if (!w21) lo +% xf_val else (lo +% xf_val) << 1,
            1, 2 => b_lo,
            3 => b_lo +% xf_val,
            4 => lo,
            else => lo +% xf_val,
        } else switch (mode) {
            // 713, 717: P + X / P - X whatever w3[21]; other integer modes unmeasured.
            5 => lo +% xf_val,
            6 => lo -% xf_val,
            else => return RunError.UnknownForm,
        }) else if (!w21) (if (w23) switch (mode) {
            // 719: fractional, w3[21] = 0; modes 3, 6 and 7 add no X to lo.
            0 => lo -% xf_val,
            1, 2, 3 => b_lo,
            4 => b_lo +% xf_val,
            5 => b_lo -% xf_val,
            else => lo,
        } else switch (mode) {
            // 713: the field went to hi and lo is P (modes 5, 6); mode 3 is +X (719).
            3 => xf_val,
            5, 6 => lo,
            else => return RunError.UnknownForm,
        }) else if (w23) switch (mode) {
            0 => xf_val -% lo,
            1, 2 => b_lo,
            3, 4 => b_lo +% xf_val,
            5 => b_lo -% xf_val,
            else => lo +% xf_val,
        } else switch (mode) {
            0 => 0,
            1 => b_lo +% xf_val,
            2 => b_lo -% xf_val,
            3 => xf_val,
            5 => lo +% xf_val,
            6 => lo -% xf_val,
            else => xf_val -% lo,
        };
        break :blk v & M24;
    };
    const hi_m: u32 = if (p_only or hi_dest == 0) hi else blk: {
        if (integer) {
            if (w23) break :blk (if (w21) switch (mode) {
                // 719, 721: integer, w3[21] = 1; mode 0 is 2P (Xi does not enter).
                0 => hi << 1,
                1, 2 => state.s[a_s],
                3 => state.s[a_s] +% xs_val,
                4 => hi,
                else => hi +% xs_val,
            } else switch (mode) {
                // 721: integer, w3[21] = 0; hi drops Xi except in mode 5.
                1, 2, 3 => state.s[a_s],
                5 => hi +% xs_val,
                else => hi,
            }) & 0xFFFF;
            // w3[21] = 0 with a field: no X term (712, 713); w3[21] = 1: hi +- the index X (714).
            if (mode != 5 and mode != 6) return RunError.UnknownForm;
            if (!w21) break :blk if (x_reg != 0) hi else return RunError.UnknownForm;
            break :blk (if (mode == 5) hi +% xs_val else hi -% xs_val) & 0xFFFF;
        }
        const xi: i64 = @as(i64, sx24(xs_val)) << 24;
        const xf: i64 = @as(i64, sx24(xf_val)) << 24;
        const a_hi: i64 = @as(i64, state.s[a_s] & M24) << 24;
        const sp: i64 = (@as(i64, sx24(state.s[a_s])) * sx24(state.s[b_s])) << 1;
        // Mode 3 under w3[23] = 0 is +index X with any index, 0 included (719, 720; 689's ~
        // cell is withdrawn).
        const acc: i64 = if (w23 and w21) switch (mode) {
            0 => xi - sp,
            1, 2 => a_hi,
            3, 4 => a_hi + xi,
            5 => a_hi - xi,
            else => sp + xi,
        } else if (w23) switch (mode) {
            // 719: fractional, w3[21] = 0.  Mode 0 is P + ~Xi, with no rounding carry.
            0 => sp + ~xi,
            1, 2 => a_hi,
            3 => a_hi + xf,
            4 => a_hi + xi,
            5 => a_hi - xi,
            else => sp + xf,
        } else blk2: {
            const x = if (w21 or mode == 3) xi else xf;
            break :blk2 switch (mode) {
                0 => 0,
                1 => a_hi + x,
                2 => a_hi - x,
                3 => x,
                5 => sp + x,
                6 => sp - x,
                else => x - sp,
            };
        };
        const no_round = mode == 3 or (w23 and !w21 and mode == 0);
        const half: i64 = if ((bundle[3] >> 22) & 1 == 1 and !no_round) 1 << 23 else 0;
        break :blk @as(u32, @truncate(@as(u64, @bitCast((acc + half) >> 24)))) & 0xFFFF;
    };
    if (lo_dest != 0) state.s[lo_dest] = lo_m;
    if (hi_dest != 0) state.s[hi_dest] = hi_m;
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
    // w3[0] = 1 rounds to nearest, 0 truncates (684).
    const rnd = bundle[3] & 1 == 1;
    const v: u32 = switch (mode) {
        4 => @truncate((if (rnd) @as(u64, @intCast(roundShr(@intCast(prod), 23))) else prod >> 23) & M24),
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
    const a_val: u32 = if (src_a == 0) 0 else state.a[src_a];
    // The X read returns all 24 bits (693, revising 692's 16-bit reading).
    const v: u32 = state.readX(a_val) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// The x18 class 011/111 ALU slot (fact 168, 206): op w1[17:15] over
/// a = s<w2[5:2]>, b = s<w1[7:4]>.  Op 0 is only measured as pass (179).
const SWrite = struct { dest: u32, value: u32 };

/// The class-110 ALU2 slot (fact 527): s<w1[11:8]> <- op w1[17:15] of a = s<w3[6:3]>,
/// b = s<w1[7:4]>.  Op 0 under head[30:25] = 0x04 is the rounded shift (a + b + 2^(n-1)) >> n,
/// n = w3[28:25] (n = 0 a plain add); 0x09 and 0x0F the plain add.  Other codes, other ops
/// and a sum outside 24 bits are refused.
fn x18Alu110(state: *const State, bundle: []const u32) RunError!?SWrite {
    const w1 = bundle[1];
    const w3 = bundle[3];
    const dest: u32 = (w1 >> 8) & 0xF;
    if (dest == 0) return null;
    if ((w1 >> 15) & 7 != 0) return RunError.UnknownForm;
    const n: u6 = switch ((bundle[0] >> 25) & 0x3F) {
        0x04 => @intCast((w3 >> 25) & 0xF),
        0x09, 0x0F => 0,
        else => return RunError.UnknownForm,
    };
    const sum: i64 = @as(i64, sx24(state.s[(w3 >> 3) & 0xF])) + sx24(state.s[(w1 >> 4) & 0xF]);
    if (sum >= 1 << 23 or sum < -(1 << 23)) return RunError.UnknownForm;
    return .{ .dest = dest, .value = @as(u32, @truncate(@as(u64, @bitCast(roundShr(sum, n))))) & M24 };
}

fn x18SlotAlu(op: u32, a: u32, b: u32) RunError!u32 {
    return switch (op) {
        // 491: op 0 adds b (168's "pass" had b = s0).
        0 => (a +% b) & M24,
        2 => a & M24,
        1 => (a -% b) & M24,
        3 => (a +% 1) & M24,
        5 => (b -% a) & M24,
        7 => @intCast(@abs(sx16(a)) & M24),
        else => RunError.UnknownForm,
    };
}

/// x18 class 011 table B (641): head code 0x10 with n = w2[9:6] outside X18C3_ALU_N.
fn x18C3TableB(op: u32, a: u32, b: u32) RunError!u32 {
    return switch (op) {
        0 => (b -% a) & M24,
        1 => ~(a +% b) & M24,
        2, 3 => (0 -% a) & M24,
        4 => ~a & M24,
        5 => (a +% b +% 1) & M24,
        6 => (a +% 1) & M24,
        // a < 0 at 16 or 24 bits is not separated: refuse where the two disagree.
        7 => if ((sx16(a) < 0) != (sx24(a) < 0)) RunError.UnknownForm else if (sx24(a) < 0) (a +% 1) & M24 else (0 -% a) & M24,
        else => unreachable,
    };
}

/// The slot ALU as class 011 runs it: ops 4 (a - 1) and 6 (-a, from a's low 16 bits, like op 7's
/// abs) are measured there (643 follow-up, fact 641 rows); class 111 still refuses them.
fn x18C3SlotAlu(op: u32, a: u32, b: u32) RunError!u32 {
    return switch (op) {
        4 => (a -% 1) & M24,
        6 => @as(u32, @truncate(@as(u64, @bitCast(-sx16(a))))) & M24,
        // |a| over 24 bits, wrapping at -2^23: abs(0x800000) = 0x800000 (708).
        7 => @intCast(@abs(sx24(a)) & M24),
        else => x18SlotAlu(op, a, b),
    };
}

/// shr / rshr / lsr by a register count (644): b >= 0 shifts right (arithmetic, rounded, or
/// logical over 24 bits), b < 0 shifts left by -b.  Counts are measured over |b| <= 12 (644,
/// 645); any other b is refused, as is a left shift past 24 bits.
const X18C3ShiftKind = enum { shr, rshr, lsr };
fn x18C3ShiftBy(kind: X18C3ShiftKind, v: u32, b: u32) RunError!u32 {
    const k = sx24(b);
    if (k < -12 or k > 12) return RunError.UnknownForm;
    if (k < 0) return x18C3Shl(v, @intCast(-k));
    return switch (kind) {
        .shr => x18C3Shr(v, @intCast(k)),
        .rshr => x18C3Rnd(v, @intCast(k)),
        .lsr => (v & M24) >> @intCast(k),
    };
}

/// 0x36 / 0x37 / 0x38's register-count shift kinds (645).
const X18C3_SHIFT_36 = [3]X18C3ShiftKind{ .shr, .rshr, .lsr };

/// rot24(v, b): rotate the 24-bit value left by b, right for b < 0 (645, |b| <= 12).
fn x18C3Rot(v: u32, b: u32) RunError!u32 {
    const k = sx24(b);
    if (k < -12 or k > 12) return RunError.UnknownForm;
    const l: u5 = @intCast(@mod(k, 24));
    const x = v & M24;
    if (l == 0) return x;
    return ((x << l) | (x >> @intCast(24 - @as(u6, l)))) & M24;
}

/// Ops 2/3/4 min, op 6 max, signed 24-bit (644).
fn x18C3MinMax(op: u32, x: u32, y: u32) u32 {
    const lo = sx24(x) < sx24(y);
    return (if ((op == 6) != lo) x else y) & M24;
}

/// x18 class 011 unary ops 2/3/4/6 under the 45 codes 641-643 left (644).  Null for codes this
/// table does not cover.  #n is the immediate n; S = {2, 5, 6, 7, 9}.  The fit covers n = 0..2,
/// so n > 2 is refused except where a code selects on S.  Op 7 runs only under 0x09-0x0F.
fn x18C3Unary644(code: u32, op: u32, n: u32, a: u32, b: u32, p3: bool) RunError!?u32 {
    const covered = switch (code) {
        0x06...0x0F, 0x12...0x15, 0x1A...0x22, 0x24...0x2F, 0x36...0x39, 0x3B...0x3F => true,
        else => false,
    };
    if (!covered) return null;
    if (op == 7 and !(code >= 0x09 and code <= 0x0F)) return RunError.UnknownForm;
    const uses_s = switch (code) {
        0x12...0x15, 0x24, 0x25 => true,
        else => false,
    };
    if (n > 2 and !uses_s) return RunError.UnknownForm;
    const in_s = cond646(n, p3);
    const imm = n;
    const logic_imm: u32 = (switch (op) {
        2 => a | imm,
        3 => a & imm,
        4 => ~imm,
        else => imm,
    }) & M24;
    const logic_b: u32 = (switch (op) {
        2 => a | b,
        3 => a & b,
        4 => ~b,
        else => b,
    }) & M24;
    const pick_imm = if (op == 6) a & M24 else imm; // 0x29 / 0x2F: #n for ops 2/3/4, a for 6
    return switch (code) {
        0x06 => try x18C3ShiftBy(.shr, try x18C3SlotAlu(op, a, b), b),
        0x07 => try x18C3ShiftBy(.rshr, try x18C3SlotAlu(op, a, b), b),
        0x08 => if (op == 6) RunError.UnknownForm else try x18C3ShiftBy(.lsr, try x18C3SlotAlu(op, a, b), b),
        0x09...0x0F => try x18C3SlotAlu(op, a, b),
        0x12, 0x13 => if (in_s) try x18C3SlotAlu(op, a, b) else (switch (op) {
            2 => a,
            3 => a -% 1,
            4 => a +% 1,
            else => ~a,
        }) & M24,
        0x14, 0x15 => if (in_s) try x18C3SlotAlu(op, a, b) else (switch (op) {
            2 => ~(a +% 1),
            3, 4 => ~a,
            else => a,
        }) & M24,
        0x1A => try x18C3ShiftBy(.shr, logic_imm, b),
        0x1B => try x18C3ShiftBy(.lsr, logic_imm, b),
        0x1C => logic_imm,
        0x1D...0x1F => logic_b,
        0x20, 0x22 => 0,
        0x21 => if (op == 4) RunError.UnknownForm else 0,
        0x24 => if (in_s) try x18C3SlotAlu(op, a, b) else b & M24,
        0x25 => if (in_s) a & M24 else logic_b,
        0x26, 0x2A => x18C3MinMax(op, a, b),
        0x27, 0x2B => if (op == 6) a & M24 else b & M24,
        0x28 => x18C3MinMax(op, a, imm),
        0x29 => pick_imm,
        0x2C => try x18C3Shl(x18C3MinMax(op, a, b), n),
        0x2D => if (n == 0) x18C3MinMax(op, a, b) else try x18C3Rnd(x18C3MinMax(op, a, b), n),
        0x2E => try x18C3ShiftBy(.shr, x18C3MinMax(op, a, imm), b),
        0x2F => try x18C3ShiftBy(.lsr, pick_imm, b),
        0x3B => if (op == 4) b & M24 else 0,
        0x3C => if (n <= 1) try x18C3SlotAlu(op, a, b) else if (n == 2) b & M24 else RunError.UnknownForm,
        0x3D, 0x3F => if (n <= 1) try x18C3SlotAlu(op, a, b) else RunError.UnknownForm,
        0x3E => if (op == 4) try x18C3Shr(a, n) else RunError.UnknownForm,
        0x36...0x38 => switch (op) {
            2 => try x18C3ShiftBy(X18C3_SHIFT_36[code - 0x36], a, b),
            // 645: op 3 at n = 2 shifts left by b (0x37 rounding the reverse shift); n < 2 gives a.
            3 => if (n < 2) a & M24 else try x18C3ShiftBy(if (code == 0x37) .rshr else .shr, a, (0 -% b) & M24),
            4 => if (n < 2) a & M24 else try x18C3ShiftBy(X18C3_SHIFT_36[code - 0x36], a, b),
            // 645: op 6 rotates the 24-bit value by b; 0x36 / 0x37 rotate a | (a < 0).
            else => try x18C3Rot(if (code != 0x38 and sx24(a) < 0) a | 1 else a, b),
        },
        // 645: 0x39, n < 2 shlz for every op; n = 2 asrz for op 3, lsrz for the others.
        else => blk: {
            const k = sx24(b);
            if (k < -12 or k > 12) return RunError.UnknownForm;
            if (n < 2) break :blk if (k < 0) 0 else try x18C3Shl(a, @intCast(k));
            if (op == 3) break :blk if (k < 0) (if (sx24(a) < 0) M24 else 0) else try x18C3Shr(a, @intCast(k));
            break :blk if (k < 0) 0 else (a & M24) >> @intCast(k);
        },
    };
}

/// v << n at 24 bits; a result past 24 bits is unmeasured and refused.
fn x18C3Shl(v: u32, n: u32) RunError!u32 {
    const r = @as(i64, sx24(v)) << @intCast(n);
    if (r >= 1 << 23 or r < -(1 << 23)) return RunError.UnknownForm;
    return @as(u32, @truncate(@as(u64, @bitCast(r)))) & M24;
}

/// v >> n, arithmetic (floor), at 24 bits.
fn x18C3Shr(v: u32, n: u32) RunError!u32 {
    return @as(u32, @truncate(@as(u64, @bitCast(@as(i64, sx24(v)) >> @intCast(n))))) & M24;
}

/// (a + 2^(n-1)) >> n, n >= 1 (643, code 0x34); a sum past 24 bits is unmeasured and refused.
fn x18C3Rnd(a: u32, n: u32) RunError!u32 {
    const sum = @as(i64, sx24(a)) + (@as(i64, 1) << @intCast(n - 1));
    if (sum >= 1 << 23) return RunError.UnknownForm;
    return @as(u32, @truncate(@as(u64, @bitCast(sum >> @intCast(n))))) & M24;
}

/// The n values (w2[9:6]) under which head codes 0x10 and 0x23 run the slot ALU (641).
/// The class-011 slot value (641, correcting 361; 752): head code head[30:25] 0x10 and 0x23 run the
/// slot ALU when n = w2[9:6]'s 646 condition holds on p3 (641's {2, 5, 6, 7, 9} was p3 = 1);
/// otherwise table B (0x10) or b (0x23).  Unary ops (w1[17:15] 2, 3, 4, 6, 7) under codes 0x00-0x02 shift the result left
/// by n, under 0x03-0x05 right, arithmetic (642, correcting 361's pre-shift of a).  Under
/// 0x16-0x19, 0x30-0x35 and 0x3A (643) unary ops give LOGIC {2: a|b, 3: a&b, 4: ~b, 6/7: b}
/// << n (0x16/0x17) or >> n (0x18/0x19); a << n / a (0x30-0x32), a >> n / a (0x33, 0x35),
/// rounded a >> n / a (0x34) for ops 2/6/7 / 3/4; 0x3A a << n for ops 2/3, a >> n for 4/6/7.
/// Any other n != 0 is refused (370 measures it only as a shift on binary ops).
fn x18C3Value(h: u32, w1: u32, w2: u32, a: u32, b: u32, p3: bool) RunError!u32 {
    const op = (w1 >> 15) & 7;
    const n = (w2 >> 6) & 0xF;
    return switch ((h >> 25) & 0x3F) {
        // 777: 0x0E op 0 is a + b at every n; n only picks 776's p write.
        0x0E => if (op == 0) x18C3SlotAlu(op, a, b) else if (n != 0) RunError.UnknownForm else x18C3SlotAlu(op, a, b),
        // 644: 0x11 runs as 0x10.
        0x10, 0x11 => if (cond646(n, p3)) x18C3SlotAlu(op, a, b) else x18C3TableB(op, a, b),
        0x23 => if (cond646(n, p3)) x18C3SlotAlu(op, a, b) else b & M24,
        // 710: 0 for every op (measured at n = 0 and 2).
        0x22 => if (n <= 2) 0 else RunError.UnknownForm,
        else => |code| {
            const unary = op == 2 or op == 3 or op == 4 or op == 6 or op == 7;
            if (unary) {
                const logic: u32 = switch (op) {
                    2 => a | b,
                    3 => a & b,
                    4 => ~b,
                    else => b,
                } & M24;
                const shifting = op == 2 or op == 6 or op == 7; // 0x30-0x35: ops 3 / 4 give a
                switch (code) {
                    // 642: codes 0x00-0x02 shift the result left by n, 0x03-0x05 right (floor).
                    0x00...0x02 => return x18C3Shl(try x18C3SlotAlu(op, a, b), n),
                    0x03...0x05 => return x18C3Shr(try x18C3SlotAlu(op, a, b), n),
                    // 643: the logic ops, shifted.
                    0x16, 0x17 => return x18C3Shl(logic, n),
                    0x18, 0x19 => return x18C3Shr(logic, n),
                    0x30...0x32 => return if (shifting) x18C3Shl(a, n) else a & M24,
                    0x33, 0x35 => return if (shifting) x18C3Shr(a, n) else a & M24,
                    0x34 => return if (!shifting or n == 0) a & M24 else x18C3Rnd(a, n),
                    0x3A => return if (op == 2 or op == 3) x18C3Shl(a, n) else x18C3Shr(a, n),
                    else => if (try x18C3Unary644(code, op, n, a, b, p3)) |v| return v,
                }
            }
            return if (n != 0) RunError.UnknownForm else x18C3SlotAlu(op, a, b);
        },
    };
}

/// Class 011 (fact 168, 179, 200, 206, 683): the slot result goes to
/// s<w1[11:8]> and R<w3[10:7]> (#head[20:5] when w3[3] = 1, 688), and p<w2[17:14]> <- cond head[28:25] of it.
/// Conds 0 (true), 11 (v <= 0) and 13 (v == 0) are measured (743); other conds leave p alone.
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
    const v = try x18C3Value(h, w1, w2, a, b, state.p[3] != 0);
    const dest_s: u32 = (w1 >> 8) & 0xF;
    // The head's load slot (own-x18b and siblings, 645 note): s<head[24:21]> <- Y:[e<w3[28:25]>].
    // Only words below 0x8000 were loaded (sign unmeasured), and a dest shared with the slot's
    // dest or operands has no measured order: refuse those.
    // The source mux w1[14:12] (683, 369): 000 delivers 0, 001/010/101 the Y read, 110 the
    // immediate head[20:5]; 011, 100, 111 are OPEN, and so is e1 under 010/101.
    const ld_dest: u32 = (h >> 21) & 0xF;
    var ld_val: u32 = 0;
    if (ld_dest != 0) {
        switch ((w1 >> 12) & 7) {
            0 => ld_val = 0,
            1, 2, 5 => |m| {
                if (m != 1 and (w3 >> 25) & 0xF == 1) return RunError.UnknownForm;
                ld_val = state.getY(state.e[(w3 >> 25) & 0xF]) orelse 0;
                if (ld_val & 0x8000 != 0) return RunError.UnknownForm;
            },
            6 => ld_val = (h >> 5) & 0xFFFF,
            else => return RunError.UnknownForm,
        }
        if (ld_dest == dest_s or ld_dest == (w2 >> 2) & 0xF or ld_dest == (w1 >> 4) & 0xF) return RunError.UnknownForm;
    }
    // w3[3] = 1 writes #head[20:5] to the R dest instead of the result (688, revising 683).
    const r_val: u32 = if ((w3 >> 3) & 1 == 1) (h >> 5) & 0xFFFF else v & 0xFFFF;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    // The result also lands in A<2 * w1[3:0] + head[31]> (fact 345): measured with the and
    // (A2 <- s12 & s11). A31 is not a plain register (649): refused.
    const dest_a: u32 = ((w1 & 0xF) << 1) | (h >> 31);
    if (dest_a == 31) return RunError.UnknownForm;
    if (dest_s != 0) state.s[dest_s] = v;
    if (ld_dest != 0) state.s[ld_dest] = ld_val;
    if (dest_r != 0) state.r[dest_r] = r_val;
    if (dest_a != 0) state.a[dest_a] = v;
    if (try x18C3Cond(h, w1, w2, v, if (state.unset_reads) |u| &u.open_p else null)) |bit| {
        const pd: u32 = (w2 >> 14) & 0xF;
        // p0 reads 1 (682): cond 0's true into p0 changes nothing; other p0 writes are unmeasured.
        if (pd != 0) state.p[pd] = @intFromBool(bit) else if ((h >> 25) & 0xF != 0) return RunError.UnknownForm;
    }
}

/// The class-011 p<w2[17:14]> write, cond = head[28:25] (the code's low bits).  Codes below
/// 0x10 always write (776): 0x0E is p <- n not in {2, 5, 6, 7, 9}, operand-free; the others,
/// at n = 0 with b = s0, 2 / 5 / 8 v >= 0, 11 / 12 v <= 0, 13 v == 0, the rest 1.  With n != 0
/// or b != s0 b enters the compare (OPEN): p is left alone, as 743 left the unmeasured conds,
/// so the measured value rows (641-644) keep running.  Codes 0x10 up keep 743: cond 0 true,
/// 11 v <= 0, 13 v == 0, others leave p alone.  null: no write.
fn x18C3Cond(h: u32, w1: u32, w2: u32, v: u32, open_p: ?*u64) RunError!?bool {
    const code: u32 = (h >> 25) & 0x3F;
    const cond: u32 = code & 0xF;
    const n: u32 = (w2 >> 6) & 0xF;
    const sv = sx16(v);
    if (code == 0x0E) return switch (n) {
        2, 5, 6, 7, 9 => false,
        else => true,
    };
    if (code < 0x10) {
        if (n != 0 or (w1 >> 4) & 0xF != 0) {
            if (open_p) |c| c.* += 1;
            return null;
        }
        return switch (cond) {
            2, 5, 8 => sv >= 0,
            11, 12 => sv <= 0,
            13 => sv == 0,
            else => true,
        };
    }
    return switch (cond) {
        0 => true,
        11 => sv <= 0,
        13 => sv == 0,
        else => null,
    };
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
    // X slot s<head[24:21]>: muxes 1, 2 and 6 the immediate head[20:5] (175); 3, 4 and 5 lanes
    // of Y:[e<w3[28:25]>] (486: Y[7:0] << 8, (Y << 7)[15:0], Y) when w2[22:21] = 01.  Other
    // w2[22:21]: mux 3 takes yd[7:0] << 8 (597), muxes 4 / 5 are OPEN.
    const xs_dest: u32 = (h >> 21) & 0xF;
    // The muxes 3-5 fresh read sets yd to the raw word (after 597); only yd[7:0] is measured,
    // the rest follows x00.
    var yd_new: ?u32 = null;
    const xs_val: u32 = if (xs_dest == 0) 0 else blk: {
        const fresh = (w2 >> 21) & 3 == 1;
        const lane = (w1 >> 12) & 7;
        if (fresh and lane >= 3 and lane <= 5) yd_new = (state.getY(state.e[(w3 >> 25) & 0xF]) orelse 0) & 0xFFFF;
        const y: u32 = if (fresh) (state.getY(state.e[(w3 >> 25) & 0xF]) orelse 0) & 0xFFFF else state.yd & 0xFFFF;
        break :blk switch ((w1 >> 12) & 7) {
            1, 2, 6 => (h >> 5) & 0xFFFF,
            3 => (y & 0xFF) << 8,
            4 => if (fresh) shl7(y) else return RunError.UnknownForm,
            5 => if (fresh) y else return RunError.UnknownForm,
            else => return RunError.UnknownForm,
        };
    };
    var acc: i64 = 0;
    const mode: u32 = (w3 >> 4) & 7;
    // Mode 5 floor (w3[8] = 0): the 701 running MAC, 24-bit operands and a 48-bit acc.
    const mac48_on = mode == 5 and (w3 >> 8) & 1 == 0;
    if ((hi_dest != 0 or lo_dest != 0) and mac48_on) {
        acc = mac48(5, state.s[(w1 >> 28) & 0xF], state.s[w2 & 0xF], state.s[(w2 >> 24) & 0xF], state.s[(w2 >> 28) & 0xF]);
    } else if (hi_dest != 0 or lo_dest != 0) {
        // a and b are measured at 16-bit magnitudes only (665): wider values refused.
        const a_v: i64 = sx24(state.s[(w2 >> 24) & 0xF]);
        const b_v: i64 = sx24(state.s[(w2 >> 28) & 0xF]);
        if (a_v < -0x8000 or a_v > 0x7FFF or b_v < -0x8000 or b_v > 0x7FFF) return RunError.UnknownForm;
        const p: i64 = a_v * b_v * 2;
        // Mode 5's X_lo is the 24-bit register unsigned, and its low output acc mod 2^24, so
        // (hi, lo) fed back chain exactly with w3[8] = 0 (665, 668). The other modes keep 203's
        // 16-bit readings: X_lo sign-extended from 16, taken unsigned at 24.
        const x_lo: i64 = if (mode == 5) state.s[w2 & 0xF] & M24 else sx16To24(x_lo_raw);
        const x: i64 = (@as(i64, state.s[(w1 >> 28) & 0xF] & 0xFFFF) << 24) + x_lo;
        acc = switch (mode) {
            3 => x,
            4 => p,
            5 => x + p,
            6 => p - x,
            7 => x - p,
            else => return RunError.UnknownForm,
        };
    }
    // w3[8] rounds the high write half toward +inf on the final sum, every mode (601 and
    // after); the low write is the unrounded acc (665).
    const acc_hi: i64 = if ((w3 >> 8) & 1 == 1) acc + (1 << 23) else acc;
    var alu_v: u32 = 0;
    if (alu_dest != 0) {
        const r = try x18SlotAlu((w1 >> 15) & 7, state.s[(w2 >> 16) & 0xF], state.s[(w1 >> 4) & 0xF]);
        const n: u5 = @intCast((w3 >> 11) & 0xF);
        alu_v = if ((h >> 27) & 1 == 1) blk: {
            break :blk @truncate(@as(u64, @bitCast(roundShr(sx24(r), n))) & 0xFFFF);
        } else (r << n) & 0xFFFF;
    }
    if ((w2 >> 21) & 3 == 3) {
        state.y_mem.put(gpa, state.e[(w3 >> 25) & 0xF], st_val) catch return RunError.OutOfMemory;
    }
    const acc_u: u64 = @bitCast(acc);
    const hi_u: u64 = @bitCast(acc_hi);
    // The e slot (703): e<w1[23:20]> <- AGU10[w1[27:24]](e<w3[28:25]>); fn 2 (+1) measured.
    const dest_e: u32 = (w1 >> 20) & 0xF;
    if (dest_e != 0 and (w1 >> 24) & 0xF != 2) return RunError.UnknownForm;
    if (xs_dest != 0) state.s[xs_dest] = xs_val;
    if (yd_new) |v| state.yd = v;
    if (dest_e != 0) state.e[dest_e] = (state.e[(w3 >> 25) & 0xF] +% 1) & 0xFFFF;
    if (hi_dest != 0) state.s[hi_dest] = @truncate((hi_u >> 24) & (if (mac48_on) M24 else 0xFFFF));
    if (lo_dest != 0) state.s[lo_dest] = @truncate(acc_u & (if (mode == 5) M24 else 0xFFFF));
    if (alu_dest != 0) state.s[alu_dest] = alu_v;
}

/// The class 010/101 R slot (fact 194, 195):
/// R<w3[10:7]> <- AGU3[w3[24:20]](x = A<w3[19:15]>, y = R<w3[14:11]>).
fn x18RSlot(state: *const State, w3: u32) u32 {
    return agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0x1F], state.r[(w3 >> 11) & 0xF], 0);
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
    // Mux 011 (685, 686): w1[28] = 1 reads Y:[e<w3[28:25]>] and sets yd to it; w1[28] = 0 takes
    // yd.  Either way the slot gets [7:0] << 8, class 111's mux-3 rule (597).
    const fresh3 = (w1 >> 12) & 7 == 3 and (w1 >> 28) & 1 == 1;
    const y3: u32 = if (fresh3) (state.getY(x_e_val) orelse 0) & 0xFFFF else state.yd & 0xFFFF;
    const xs_val: u32 = if (dest_s == 0) 0 else switch ((w1 >> 12) & 7) {
        1 => x_e_val,
        3 => (y3 & 0xFF) << 8,
        6 => (h >> 5) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    const e_val: u32 = if (dest_e == 0) 0 else blk: {
        if ((w1 >> 29) & 1 == 0) return RunError.UnknownForm;
        break :blk try agu10Eval((w1 >> 24) & 0xF, x_e_val, 0);
    };
    // w2[27:24] != 0 can offset the R write (598: 1, 6 and F measured, no rule yet): OPEN.
    if (dest_r != 0 and (w2 >> 24) & 0xF != 0) return RunError.UnknownForm;
    const r_val: u32 = x18RSlot(state, w3);
    if (((w2 >> 28) & 7) == 4) try execX18Q23(gpa, state, bundle);
    if (dest_s != 0) state.s[dest_s] = xs_val;
    if (fresh3 and dest_s != 0) state.yd = y3;
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
        break :blk @truncate(@as(u64, @bitCast(roundShr(prod, 23))) & 0xFFFF);
    };
    const dest_sh: u32 = (w1 >> 24) & 0xF;
    const sh_val: u32 = if (dest_sh == 0) 0 else blk: {
        const src: u32 = state.s[(w2 >> 16) & 0xF];
        const n: u5 = @intCast((w1 >> 20) & 0xF);
        if ((h >> 27) & 1 == 1) {
            break :blk @truncate(@as(u64, @bitCast(roundShr(sx24(src), n))) & 0xFFFF);
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

/// ytst (fact 99, 625): p<S1[14:11]> <- !Y:[e<S1[24:21]>][0]; head classes 0/2/4/6;
/// with S1[8] and S1[3] both set, X:[A<{S1[2:0], head[31:30]}>] <- sx24 s<S1[7:4]>.  The head
/// also writes s<h[24:21]> <- e<S1[24:21]> and A<h[28:25]> <- h[20:5] (635).
fn execYtst(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 2) return RunError.UnknownForm;
    const s1 = bundle[1];
    const h = bundle[0];
    if ((h >> 29) & 1 == 1) return RunError.UnknownForm;
    const dest_a: u32 = (h >> 25) & 0xF;
    const stores = (s1 >> 8) & 1 == 1 and (s1 >> 3) & 1 == 1;
    const ai: u32 = ((s1 & 7) << 2) | (h >> 30);
    // The store's A against the head's A write on one register: unmeasured.
    if (stores and dest_a != 0 and ai == dest_a) return RunError.UnknownForm;
    if (stores) state.x_mem.put(gpa, state.a[ai], state.s[(s1 >> 4) & 0xF] & M24) catch return RunError.OutOfMemory;
    const dest_s: u32 = (h >> 21) & 0xF;
    if (dest_s != 0) state.s[dest_s] = state.e[(s1 >> 21) & 0xF];
    // A0 skipped as in ldi2 (INFERRED).
    if (dest_a != 0) state.a[dest_a] = (h >> 5) & 0xFFFF;
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
    const a_idx: u32 = (4 * a_low + a_half) & 0x1F;
    const addr: u32 = state.a[a_idx];
    const value: u32 = state.s[src_s] & M24;
    try state.x_mem.put(gpa, addr, value);
}

fn execXld10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    // Every measured row sets S1[18]; clearing it wedges (559).
    if ((bundle[1] >> 18) & 1 == 0) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const a_half: u32 = (head >> 30) & 0x3;
    const dest_s2: u32 = (head >> 21) & 0xF;
    const imm16: u32 = (head >> 5) & 0xFFFF;
    const dest_s_x: u32 = (s1 >> 7) & 0xF;
    const a_low: u32 = s1 & 0x7;
    const a_idx: u32 = (4 * a_low + a_half) & 0x1F;
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
    const neg_a: u32 = (0 -% a_val) & M24;
    // 766: w[11] = 0, op 4 is a + b with b = 0 (w[28:22], w[2:0] inert).  132 (table 119's
    // w[11] = 0 row): op 5 = a, op 7 = -a; head[9] = 1 turns op 5 into -a.  Table-only rows
    // (op 6, w[11] = 1 ops 4/6/7, head[9] elsewhere) stay refused until measured in ealu.
    // head[9] is inert at op 4 (767), -a at op 5 (132), OPEN at the other ops.
    const h9 = (bundle[0] >> 9) & 1 == 1;
    const sel: u32 = op_bit << 3 | op;
    if (h9 and sel != 4 and sel != 5) return RunError.UnknownForm;
    const v: u32 = if (h9 and sel == 5) neg_a else switch (sel) {
        4, 5 => a_val,
        7 => neg_a,
        8 | 5 => (a_val +% 1) & M24,
        else => return RunError.UnknownForm,
    };
    if (dest_e != 0) state.e[dest_e] = v;
    // 767: w[10:7] one-hot picks s1 / s2 / s4 / s8 <- #sx16(head[26:11]); two or more bits write
    // nothing (487's 1001).
    const side: u32 = (s1 >> 7) & 0xF;
    if (@popCount(side) == 1) state.s[side] = sx16To24((bundle[0] >> 11) & 0xFFFF);
}

fn execYld10(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    // S[6:3] names an s write whose source is OPEN (fact 542, revising 123's clears).
    if ((s1 >> 3) & 0xF != 0) return RunError.UnknownForm;
    const dest_s: u32 = (head >> 21) & 0xF;
    const src_e: u32 = (s1 >> 25) & 0xF;
    const addr: u32 = state.e[src_e];
    const v = (state.getY(addr) orelse 0) & 0xFFFF;
    if (dest_s != 0) state.s[dest_s] = v;
    state.yd = v; // 571
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

/// IMM10_SECONDS of class 001 (fact 74 pass 100); the class-011 ones decode elsewhere.
const LDI2_IMM10_SECONDS = [_]u32{ 0x20040000, 0x20400000, 0x20440000, 0x22463000 };

fn execLdi2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const sw = bundle[1];
    // 634: h[31:30] = 00 and a second with S[20:19] = 11, S[18:0] = 0, or one of the class-001
    // IMM10_SECONDS (74 pass 100: no extra write).  Other seconds wedge or are OPEN.  Whether
    // the A load runs with an IMM10 second is unmeasured: A<h[29:25]> != 0 refused there.
    if (head >> 30 != 0) return RunError.UnknownForm;
    const store = sw & 0x1FFFFF == 0x180000;
    if (!store) {
        if (std.mem.indexOfScalar(u32, &LDI2_IMM10_SECONDS, sw) == null) return RunError.UnknownForm;
        if ((head >> 25) & 0x1F != 0) return RunError.UnknownForm;
    }
    const imm: u32 = (head >> 5) & 0xFFFF;
    // Y:[e<S[28:25]>] <- s<S[24:21]>, s read before the head's write (fact 89, 634).
    if (store) {
        const v: u32 = state.s[(sw >> 21) & 0xF] & 0xFFFF;
        state.y_mem.put(gpa, state.e[(sw >> 25) & 0xF], v) catch return RunError.OutOfMemory;
    }
    // s<h[24:21]> <- sx16 #, and A<h[29:25]> <- # (634).  A0 is skipped: INFERRED, A0 reads 0
    // elsewhere and a write there is unmeasured.
    const dest_s: u32 = (head >> 21) & 0xF;
    const dest_a: u32 = (head >> 25) & 0x1F;
    if (dest_s != 0) state.s[dest_s] = sx16To24(imm);
    if (dest_a != 0) state.a[dest_a] = imm;
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
    // w2 classes 001 and 101 too (556): w2[29] is the s<w2[30:27]> dest's bit.
    if (w1_class == 0 and (w2_class == 0 or w2_class == 1 or w2_class == 4 or w2_class == 5) and w3_class == 2) {
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
    // x19mac: w1 class 000/001 plain, 010/101 with the head ALU (fact 312, 384-386).
    // w2 class 100 runs it as 001 (521, 7 of 7); with w3 class 101 it is OPEN and falls
    // through to the refusal below.  The two rows that wedged silicon (w2 = 0x80C80000) are
    // refused already: one by the open X store (w2[23:22] = 11), one by its w1 class 111.
    // 551: w2 classes 000 and 101 run it too.
    if ((w2_class == 0 or w2_class == 1 or w2_class == 4 or w2_class == 5) and w3_class == 6 and (w1_class == 0 or w1_class == 1 or w1_class == 2 or w1_class == 5)) {
        return execX19Mac(gpa, state, bundle);
    }
    if (w1_class == 0 and w2_class == 0 and w3_class == 1) {
        return execX19Imm(gpa, state, bundle);
    }
    // w2[30] is inert (640): w2 class 010 is the same form.
    if (w1_class == 0 and (w2_class == 0 or w2_class == 2) and w3_class == 0) {
        return execX19Agu(gpa, state, bundle);
    }
    // Any w2 class (555): w2[30:29] are the s<w2[30:27]> dest's high bits.
    if (w1_class == 0 and w3_class == 4) {
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
    // x19k: w1[31:29] is the X-store source's high bits, not a selector (fact 235, 534).
    // w2[30:29] are inert there too (551): w2 classes 000-011.
    if (w2_class < 4 and w3_class == 0 and (w1_class == 1 or w1_class == 3 or w1_class == 4 or w1_class == 5 or w1_class == 7)) {
        return execX19K00(gpa, state, bundle);
    }
    return RunError.UnknownForm;
}

/// x19k (fact 235-240, 534): every slot reads before any writes, and all fire together.
///   s<head[24:21]> <- mux w1[14:12]: 0 and 7 zero, 6 #head[20:5], 1/2 AGU10[w1[27:24]](x, y);
///     with w1[17:15] = 5 / 7 instead s<w1[11:8]> <- a - b / |b| (16-bit), a = s<w1[7:4]>,
///     b = s<w3[6:3]>, and no head write.
///   R<w3[10:7]> <- (w2[25] ? AGU10 : AGU10B)[w1[27:24]](x, y).
///   X:[A<w2[21:17]>] <- s<w1[31:28]> when w2[23:22] = 11 (239); w1 class 000 is x19agu, so
///     the source is never s0.
///   Y:[e<w3[28:25]>] <- s<w2[3:0]> when w2[26] = w2[16] = 1 (677); with w1[18] = 0 the engine
///     wedges after the bundle, so that form is refused.
/// x = e<w3[28:25]>, y = R<w3[14:11]>.  Muxes 3-5 are OPEN.
fn execX19K00(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const fn_op: u32 = (w1 >> 24) & 0xF;
    const x: u32 = state.e[(w3 >> 25) & 0xF];
    const y: u32 = state.r[(w3 >> 11) & 0xF];
    const store_val: u32 = state.s[(w1 >> 28) & 0xF] & M24;
    const y_store = (w2 >> 26) & 1 == 1 and (w2 >> 16) & 1 == 1;
    if (y_store and (w1 >> 18) & 1 == 0) return RunError.UnknownForm;
    const y_val: u32 = state.s[w2 & 0xF] & 0xFFFF;
    const kalu: bool = switch ((w1 >> 15) & 7) {
        5, 7 => true,
        else => false,
    };
    const mux: u32 = (w1 >> 12) & 7;
    const s_val: u32 = switch (mux) {
        0, 7 => 0,
        6 => (head >> 5) & 0xFFFF,
        1, 2 => if (kalu) x19KAlu(state, w1, w3) else try agu10Eval(fn_op, x, y),
        else => return RunError.UnknownForm,
    };
    const r_val: u32 = if ((w2 >> 25) & 1 == 1) try agu10Eval(fn_op, x, y) else agu10bEval(fn_op, x, y);
    if (kalu and (mux == 1 or mux == 2)) state.s[(w1 >> 8) & 0xF] = s_val else state.s[(head >> 21) & 0xF] = s_val;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    if (dest_r != 0) state.r[dest_r] = r_val;
    if ((w2 >> 22) & 3 == 3) state.x_mem.put(gpa, state.a[(w2 >> 17) & 0x1F], store_val) catch return RunError.OutOfMemory;
    if (y_store) state.y_mem.put(gpa, x, y_val) catch return RunError.OutOfMemory;
}

/// x19k's w1[17:15] ALU (own-x19kalu m = 5: a - b; own-x19kabs m = 7: |b| over 16 bits).
fn x19KAlu(state: *const State, w1: u32, w3: u32) u32 {
    const b: u32 = state.s[(w3 >> 3) & 0xF] & M24;
    if ((w1 >> 15) & 7 == 7) return @intCast(@abs(sx16(b)) & M24);
    return ((state.s[(w1 >> 4) & 0xF] & M24) -% b) & M24;
}

/// Y:[e<w3[28:25]>] <- s<{w2[0], w1[31:29]}>.
/// Also runs the s slot: s<head[24:21]> <- by 529's mux w1[17:15]: 0 zero, 2 AGU3[w3[24:20]](x =
/// A<w3[19:15]>) (612, measured with w1[31:29] = 0 and 7; 1-6 INFERRED); other muxes refused.
/// The store reads s before that write.  w1[31:29] = 7: e<w1[6:3]> <- #head[20:5] (148), and
/// A<w1[26:22]> (x99 only) refuses with a dest; other values: R<w1[14:11]> <- AGU10[w1[10:7]](
/// e<w3[28:25]>, R<w1[21:18]>) (144, 149).
fn execX19A3St(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const src_s: u32 = ((bundle[2] & 1) << 3) | ((bundle[1] >> 29) & 0x7);
    const addr_e: u32 = (bundle[3] >> 25) & 0xF;
    const v: u32 = state.s[src_s] & 0xFFFF;
    const w3 = bundle[3];
    const dest_s: u32 = (bundle[0] >> 21) & 0xF;
    const w1 = bundle[1];
    const c7 = w1 >> 29 == 7;
    if (c7 and (w1 >> 22) & 0x1F != 0) return RunError.UnknownForm;
    const e_dest: u32 = if (c7) (w1 >> 3) & 0xF else 0;
    const r_dest: u32 = if (c7) 0 else (w1 >> 11) & 0xF;
    const r_val: u32 = if (r_dest == 0) 0 else try agu10Eval((w1 >> 7) & 0xF, state.e[addr_e], state.r[(w1 >> 18) & 0xF]);
    const s_val: u32 = if (dest_s == 0) 0 else switch ((w1 >> 15) & 7) {
        0 => 0,
        2 => agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0x1F], 0, 2 * (w3 & 0x7FFF)),
        else => return RunError.UnknownForm,
    };
    state.y_mem.put(gpa, state.e[addr_e], v) catch return RunError.OutOfMemory;
    if (dest_s != 0) state.s[dest_s] = s_val;
    if (e_dest != 0) state.e[e_dest] = (bundle[0] >> 5) & 0xFFFF;
    if (r_dest != 0) state.r[r_dest] = r_val;
}

/// x19a3, classes (0,0,3) (fact 149, 529): with x = A<w3[19:15]>, y = R<w1[21:18]>, i =
/// 2*w3[14:0], A<head[29:25]> <- AGU3[w3[24:20]] and s<head[24:21]> <- mux w1[17:15]: 1/2 the
/// AGU3 value, 0 and 7 zero, 5 D = X:[A<{w1[2:0], head[31:30]}>][15:0] read before the A write,
/// 4 (D << 7)[15:0] (611; 4 and 7 from the x99 sweep), 3 and 6 OPEN.  R<w1[14:11]> <-
/// AGU10[w1[10:7]](e<w3[28:25]>, y).  All read before any writes.
fn execX19A3(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const head = bundle[0];
    const w1 = bundle[1];
    const w3 = bundle[3];
    // Every measured row sets w1[27]; clearing it wedges (559).
    if ((w1 >> 27) & 1 == 0) return RunError.UnknownForm;
    const y: u32 = state.r[(w1 >> 18) & 0xF];
    const agu = agu3Eval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0x1F], y, 2 * (w3 & 0x7FFF));
    const d: u32 = (state.readX(state.a[((w1 & 7) << 2) | (head >> 30)]) orelse 0) & 0xFFFF;
    const s_val: u32 = switch ((w1 >> 15) & 7) {
        1, 2 => agu,
        0, 7 => 0,
        5 => d,
        4 => shl7(d),
        else => return RunError.UnknownForm,
    };
    const r_val = try agu10Eval((w1 >> 7) & 0xF, state.e[(w3 >> 25) & 0xF], y);
    // w2[13:9] = 01101 also stores X:[A<{w1[2:0], head[31:30]}>] <- sx24 s<w2[8:5]>, the A
    // read before the A write (613); other w2[13:9] values store nothing seen.
    const w2 = bundle[2];
    const x_addr: u32 = state.a[((w1 & 7) << 2) | (head >> 30)];
    const x_st: ?u32 = if ((w2 >> 9) & 0x1F == 0b01101) state.s[(w2 >> 5) & 0xF] & M24 else null;
    const dest_a: u32 = (head >> 25) & 0x1F;
    if (dest_a != 0) state.a[dest_a] = agu;
    state.s[(head >> 21) & 0xF] = s_val;
    if (x_st) |v| state.x_mem.put(gpa, x_addr, v) catch return RunError.OutOfMemory;
    const dest_r: u32 = (w1 >> 11) & 0xF;
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
    const w1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    // The read index is A<{w1[2:0], head[31:30]}> (fact 651, which revises 154); w3[19:15]
    // feeds the A write below, not the read. A0 and A31 are not plain address registers (649): refused.
    const src_a: u32 = ((w1 & 7) << 2) | (head >> 30);
    if (src_a == 0 or src_a == 31) return RunError.UnknownForm;
    // The bundle also writes A<head[29:25]> <- AGU3[w3[24:20]](x = A<w3[19:15]>, i) (fact
    // 654), unmodelled here: only the no-write form, head[29:25] = 0, runs.
    if ((head >> 25) & 0x1F != 0) return RunError.UnknownForm;
    const v: u32 = state.readX(state.a[src_a]) orelse 0;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// (w << 7)[15:0], the load muxes' scaled source (fact 116, 221, 532, 533).
fn shl7(w: u32) u32 {
    return (w << 7) & 0xFFFF;
}

/// x >> n rounded half up (half up on negatives too), the multiplier's and the shifters'
/// rounding (fact 147, 165); n = 0 is x.
pub fn roundShr(x: i64, n: u6) i64 {
    if (n == 0) return x;
    return (x + (@as(i64, 1) << (n - 1))) >> n;
}

/// 0x19 ALU op 23 (fact 147): b >= 0 gives sat24(a << b); b < 0 gives the rounded
/// a >> -b, (a + 2^(-b-1)) >> -b, half up on negatives.  MEASURED: own-x19alu and the
/// own-x19shr-* / own-x19shl-* gates (PROBES.md); counts past +-23 shift fully out (515).
pub fn shift23(a: i64, b: i64) i64 {
    if (b >= 0) {
        const s: u6 = @intCast(@min(b, 40));
        return std.math.clamp(a << s, -0x800000, 0x7FFFFF);
    }
    return roundShr(a, @intCast(@min(-b, 40)));
}

fn execX19Alu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const head = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    // head[9:5] = 23 (shift op), a = s<w2[21:18]>, b = s<w1[19:16]>
    // dest = s<w1[27:24]>.  Mode 0xB: the 32-op table (147, 699).
    const code: u32 = (head >> 5) & 0x1F;
    const dest_s: u32 = (w1 >> 24) & 0xF;
    const a_s: u32 = (w2 >> 18) & 0xF;
    const b_s: u32 = (w1 >> 16) & 0xF;
    const a_val: i64 = sx24(state.s[a_s]);
    const b_val: i64 = sx24(state.s[b_s]);
    // w1[14:11] is the mode (fact 454): 0xB the table below (147), 8 ops on a + b, 0xA
    // ops on a, 9 writes nothing; w1[2:0] != 0 beside 8/9/0xA is OPEN.
    const mode: u32 = (w1 >> 11) & 0xF;
    const xw = try x19AluX(state, w2, dest_s);
    // Mode 0xB's 32 ops, 24 bits wide (147, 699).
    const r: ?i64 = if (mode == 0xB) switch (code) {
        23 => shift23(a_val, b_val),
        0, 1, 2, 8, 9, 14, 15, 22, 24, 25, 27, 30 => 0,
        3, 4, 28, 29 => a_val + 1,
        5, 16, 17, 18, 19, 20, 21, 26 => a_val,
        6, 7, 10, 11, 12, 13 => b_val,
        31 => a_val + 2,
        else => return RunError.UnknownForm,
    } else blk: {
        // w1[2:0] != 0 beside 8/9/0xA is OPEN for the ALU result; with no ALU destination the
        // X writes still land (533, own-x19alx's second bundle).
        if (w1 & 7 != 0 and dest_s != 0) return RunError.UnknownForm;
        break :blk switch (mode) {
            8 => x19ModeOp(code, a_val, b_val, false),
            0xA => x19ModeOp(code, a_val, 0, true),
            9 => null,
            else => return RunError.UnknownForm,
        };
    };
    state.s[(w2 >> 4) & 0xF] = xw.x;
    state.s[(w2 >> 27) & 0xF] = xw.muxed;
    // 646: the ALU write is conditional on p1 through w1[23:20]: 0, 1, 3 write while p1 = 0;
    // 2, 5, 7 while p1 = 1; 6 and 9 always.  4, 8 and 10-15 did not write in the one context
    // measured, so they are refused until a program needs them.
    if (r != null and dest_s != 0) {
        const p1 = state.p[1] == 1;
        const writes = switch ((w1 >> 20) & 0xF) {
            0, 1, 3 => !p1,
            2, 5, 7 => p1,
            6, 9 => true,
            else => return RunError.UnknownForm,
        };
        if (!writes) return;
    }
    if (r) |x| state.s[dest_s] = @truncate(@as(u64, @bitCast(x)) & M24);
    // 647: with w3[23:20] = 0xE the bundle sets p<w2[17:14]> <- 1, whatever the result; 15
    // touches no predicate.  Other w3[23:20] values leave p alone here (OPEN).
    const pk = (w2 >> 14) & 0xF;
    if ((bundle[3] >> 20) & 0xF == 0xE and pk != 15) state.p[pk] = 1;
}

/// x19alu's two X writes (fact 533), both from X = X:[A<w2[26:22]>]: s<w2[7:4]> <- X and
/// s<w2[30:27]> <- mux w2[3:1]: 5 X, 4 (X << 7)[15:0], 1 the stale yd, 0 zero (others OPEN).
/// Two writes to one s are refused.
fn x19AluX(state: *State, w2: u32, alu_dest: u32) RunError!struct { x: u32, muxed: u32 } {
    const d1: u32 = (w2 >> 4) & 0xF;
    const d2: u32 = (w2 >> 27) & 0xF;
    if ((d1 != 0 and (d1 == d2 or d1 == alu_dest)) or (d2 != 0 and d2 == alu_dest)) return RunError.UnknownForm;
    const x: u32 = if (d1 == 0 and d2 == 0) 0 else state.readX(state.a[(w2 >> 22) & 0x1F]) orelse 0;
    const muxed: u32 = switch ((w2 >> 1) & 7) {
        0 => 0,
        1 => state.yd,
        4 => shl7(x),
        5 => x,
        else => return RunError.UnknownForm,
    };
    return .{ .x = x, .muxed = muxed };
}

/// The x19alu ops in modes 8 and 0xA (fact 454, one operand pair): mode 0xA is mode 8
/// with b = 0, except that the ops mode 8 leaves unwritten give a. Null is no write. Rows that
/// pass a through may be shifts by an amount that was 0 in the measured pair (OPEN).
fn x19ModeOp(code: u32, a: i64, b: i64, on_a: bool) ?i64 {
    return switch (code) {
        0...5, 11...15 => a + b,
        9, 10, 28 => a,
        6, 7, 8, 26, 27 => 0,
        16, 17 => b - a,
        20, 21 => -(a + b) - 2,
        else => if (on_a) a else null, // 18, 19, 22-25, 29-31
    };
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
    // The AGU10 value feeds e, R and s mux 1; mux 2 (636) has its own function table, so a
    // bundle using only mux 2 does not need AGU10[fn] to be known.
    const needs_v = dest_e != 0 or dest_r != 0 or (bundle[1] >> 12) & 7 == 1;
    const v = if (needs_v) try agu10Eval(fn_op, x_val, y_val) else 0;
    // w2[24] = 1: the e destination takes #head[20:5] (fact 270).
    const e_val: u32 = if ((bundle[2] >> 24) & 1 == 1) (bundle[0] >> 5) & 0xFFFF else v;
    const s_val = try x19AguS(state, bundle, v);
    if (dest_e != 0) state.e[dest_e] = e_val;
    if (dest_r != 0) state.r[dest_r] = v;
    state.s[(bundle[0] >> 21) & 0xF] = s_val;
}

/// x19agu's s<head[24:21]> source, mux w1[14:12] (fact 532): 0 zero, 1 the AGU value, 4
/// (X << 7)[15:0], 5 X, with X = X:[A<w2[21:17]>], or the stale yd latch when w3[1] = 0.  The
/// other muxes are OPEN.
fn x19AguS(state: *State, bundle: []const u32, agu: u32) RunError!u32 {
    const mux: u32 = (bundle[1] >> 12) & 7;
    if (mux == 0 or mux == 6) return 0; // 6: 640
    if (mux == 1) return agu;
    if (mux == 2) {
        // 636: (w2[25] ? AGU10 : AGU10Q)[w1[27:24]](x = e<w3[28:25]>, y = R<w3[14:11]>,
        // q = e<w2[7:4]>); index 0 reads 0.  w2[24] inert here.
        const fn_op: u32 = (bundle[1] >> 24) & 0xF;
        const xi: u32 = (bundle[3] >> 25) & 0xF;
        const yi: u32 = (bundle[3] >> 11) & 0xF;
        const qi: u32 = (bundle[2] >> 4) & 0xF;
        const x: u32 = if (xi == 0) 0 else state.e[xi];
        const y: u32 = if (yi == 0) 0 else state.r[yi];
        if ((bundle[2] >> 25) & 1 == 1) return agu10Eval(fn_op, x, y);
        return agu10qEval(fn_op, x, y, if (qi == 0) 0 else state.e[qi]);
    }
    if (mux == 3) {
        // 637: Y:[e<w3[28:25]>][7:0] << 8, only with w2[26] = 1, w2[16] = 0, w3[1] = 0.
        if ((bundle[2] >> 26) & 1 != 1 or (bundle[2] >> 16) & 1 != 0 or (bundle[3] >> 1) & 1 != 0) return RunError.UnknownForm;
        return ((state.getY(state.e[(bundle[3] >> 25) & 0xF]) orelse 0) & 0xFF) << 8;
    }
    if (mux != 4 and mux != 5) return RunError.UnknownForm;
    const x: u32 = if ((bundle[3] >> 1) & 1 == 0) state.yd else state.readX(state.a[(bundle[2] >> 17) & 0x1F]) orelse 0;
    return if (mux == 4) shl7(x) else x;
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
    const x_val: u32 = state.a[(w3 >> 15) & 0x1F];
    // Only fn 0 with w3[14:0] = 0 is measured: x + 1 (w2[31] is the class
    // bit here, not an i bit, per own-x19ragua's 0x346).
    if (((w3 >> 20) & 0x1F) != 0 or (w3 & 0x7F) != 0) return RunError.UnknownForm;
    if (dest_r != 0) state.r[dest_r] = (x_val +% 1) & 0xFFFF;
}

/// x19mac (fact 145, 312, 384-386), one bundle, every read before any write
/// (the multiply sees the old s when the head ALU writes it: measured 2026-11-06).
///   head ALU, w1 class 010/101: s<h[18:15]> <- ALU05[h[10:5]](s<h[14:11]>, s<h[29:26]>)
///   multiply, op w1[13:11] (0 -> 0, 4 -> P; others OPEN), x = s<w2[8:5]>, y = s<w2[4:1]>
///   (24-bit signed): P = low16(xy), doubled when w1[17] = 0; e<w1[6:3]> <- P (0 unless
///   w2[24]), R<w3[10:7]> <- P (0 if w2[28]); s<w2[12:9]> <- floor(xy / 2^24) (w1[17] = 1)
///   or floor(xy / 2^23 + 1/2) (w1[17] = 0, w1[15] = 1)
///   A slot: A<w2[20:17]> <- agu_xr(w3[24:20], A<w3[19:15]>, R<w3[14:11]>); dest 0 is no
///   write and A0 reads 0.  X store (w2[23:22] = 3, 312 / 384): X:[A<{w1[2:0], h[31:30]}>] <-
///   sx24 s<{w2[0], w1[31:29]}>, read before the writes.
fn execX19Mac(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    const op: u32 = (w1 >> 11) & 0x7;
    if (op != 0 and op != 4) return RunError.UnknownForm;
    // w1[16] = 1 gives d + x instead (312, OPEN): refused.
    if ((w1 >> 16) & 1 == 1) return RunError.UnknownForm;
    // X store (312, 384): w2[23:22] = 11 stores sx24 s<{w2[0], w1[31:29]}> at X:[A<{w1[2:0],
    // h[31:30]}>] as-is, both read before the bundle's writes; the store's own A step is
    // unmeasured (none seen).
    const x_on = (w2 >> 22) & 3 == 3;
    // 521: w2 = 0x80C80000 (store on, w2[31] = 1) wedged twice; w2[31] is necessary to it.
    if (x_on and w2 >> 31 == 1) return RunError.UnknownForm;
    const x_addr: u32 = state.a[((w1 & 7) << 2) | (bundle[0] >> 30)];
    const x_val: u32 = state.s[((w2 & 1) << 3) | (w1 >> 29)] & M24;
    const xs: u32 = (w2 >> 5) & 0xF;
    const ys: u32 = (w2 >> 1) & 0xF;
    const x: i64 = if (xs == 0) 0 else sx24(state.s[xs]);
    const y: i64 = if (ys == 0) 0 else sx24(state.s[ys]);
    const prod: i64 = x * y;
    const wide = (w1 >> 17) & 1 == 1;
    const p: u32 = if (op == 4) @as(u32, @truncate(@as(u64, @bitCast(if (wide) prod else 2 * prod)))) & 0xFFFF else 0;

    const hi_s: u32 = (w2 >> 9) & 0xF;
    var hi: u32 = 0;
    if (hi_s != 0) {
        if (op != 4) return RunError.UnknownForm;
        // w1[17] = 0: w1[15] = 1 rounds half up, 0 truncates (312); sat: 402.
        const h: i64 = if (wide) prod >> 24 else if ((w1 >> 15) & 1 == 1) @min(roundShr(prod, 23), 0x7FFFFF) else @min(prod >> 23, 0x7FFFFF);
        hi = @as(u32, @truncate(@as(u64, @bitCast(h)))) & M24;
    }
    const a_dest: u32 = (w2 >> 17) & 0xF;
    const a_val: u32 = aguXrEval((w3 >> 20) & 0x1F, state.a[(w3 >> 15) & 0x1F], state.r[(w3 >> 11) & 0xF]);

    const cls: u32 = w1 >> 29;
    if (cls == 2 or cls == 5) try execAlu(gpa, state, bundle[0]);

    state.e[(w1 >> 3) & 0xF] = if ((w2 >> 24) & 1 == 1) p else 0;
    const dest_r: u32 = (w3 >> 7) & 0xF;
    if (dest_r != 0) state.r[dest_r] = if ((w2 >> 28) & 1 == 1) 0 else p;
    if (hi_s != 0) state.s[hi_s] = hi;
    if (a_dest != 0) state.a[a_dest] = a_val;
    if (x_on) state.x_mem.put(gpa, x_addr, x_val) catch return RunError.OutOfMemory;
}

/// x19aagu (fact 144, 152, 335, 341, 342), every slot reading before any writes:
///   T = agu_xr[w3[24:20]](x = A<w3[19:15]>, r = R<h[30:27]>), i = 2*w3[14:0] | w2[31];
///   A<w1[7:3]> <- T + i when w1[25] (fn 21: only A0, discarded);
///   A<w1[20:16]> <- T (fn 21: x - i), or s<w2[21:18]> with w1[15];
///   e<{w1[2:0], h[31]}> <- s<w2[21:18]>;
///   X:[A<w2[26:22]>] <- sx24 s<w1[24:21]> when w1[8] and w1[10];
///   R<w1[14:11]> <- (w2[13] ? AGU10 : table B)[w1[31:28]](x = e<w3[28:25]>, y = R<h[30:27]>,
///     i = h[26:11]);
///   e<w2[3:0]> <- w2[13] = 0: table B, + i on fn 12-15; w2[13] = 1: e<w2[7:4]> - 1 on fn < 12,
///     else E1[fn - 12] (measured with w2[7:4] = 0 only);
///   s<w2[30:27]> <- Y:[e<w3[28:25]>][7:0] << 8 when w2[12:11] = 2 (the other muxes OPEN).
/// w1[26] = 0 wedges.
fn execX19Aagu(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    const w2 = bundle[2];
    const w3 = bundle[3];
    if ((w1 >> 26) & 1 == 0) return RunError.UnknownForm;
    const fn_a: u32 = (w3 >> 20) & 0x1F;
    const x: u32 = state.a[(w3 >> 15) & 0x1F];
    const ry: u32 = state.r[(h >> 27) & 0xF];
    const i: u32 = ((w3 & 0x7FFF) << 1) | (w2 >> 31);
    const t: u32 = aguXrEval(fn_a, x, ry);
    const s_src: u32 = state.s[(w2 >> 18) & 0xF];

    const main_dest: u32 = if ((w1 >> 25) & 1 == 1) (w1 >> 3) & 0x1F else 0;
    if (fn_a == 21 and main_dest != 0) return RunError.UnknownForm;
    const main_val: u32 = (t +% i) & 0xFFFF;
    const copy_dest: u32 = (w1 >> 16) & 0x1F;
    const copy_val: u32 = if ((w1 >> 15) & 1 == 1) s_src else if (fn_a == 21) (x -% i) & 0xFFFF else t;
    const e2_dest: u32 = ((w1 & 7) << 1) | (h >> 31);

    const st_on = (w1 >> 8) & 1 == 1 and (w1 >> 10) & 1 == 1;
    if (!st_on and ((w1 >> 8) & 1 == 1 or (w1 >> 10) & 1 == 1)) return RunError.UnknownForm;
    const st_addr: u32 = state.a[(w2 >> 22) & 0x1F];
    const st_val: u32 = sx16To24(state.s[(w1 >> 21) & 0xF]);

    const fn_r: u32 = w1 >> 28;
    const ex: u32 = state.e[(w3 >> 25) & 0xF];
    const ri: u32 = (h >> 11) & 0xFFFF;
    const use10 = (w2 >> 13) & 1 == 1;
    const r_dest: u32 = (w1 >> 11) & 0xF;
    const r_val: u32 = if (r_dest == 0) 0 else if (use10) try agu10IEval(fn_r, ex, ry, ri) else x19aaguB(fn_r, ex, ry, ri);
    const e1_dest: u32 = w2 & 0xF;
    const e1_val: u32 = if (e1_dest == 0) 0 else if (use10) blk: {
        if (fn_r < 12) break :blk (state.e[(w2 >> 4) & 0xF] -% 1) & 0xFFFF;
        if ((w2 >> 4) & 0xF != 0) return RunError.UnknownForm;
        break :blk switch (fn_r) {
            12 => (ry +% ri) & 0xFFFF,
            13 => (ex +% ry +% ri) & 0xFFFF,
            14 => ((ry >> 1) +% ri) & 0xFFFF,
            else => ry >> 1,
        };
    } else (x19aaguB(fn_r, ex, ry, ri) +% (if (fn_r >= 12) ri else 0)) & 0xFFFF;
    if (e1_dest != 0 and e1_dest == e2_dest) return RunError.UnknownForm;

    const sd: u32 = (w2 >> 27) & 0xF;
    const yv: u32 = if (sd == 0) 0 else (state.getY(try yAddrE(state, (w3 >> 25) & 0xF)) orelse 0) & 0xFFFF;
    const sv: ?u32 = if (sd == 0) null else switch ((w2 >> 11) & 3) {
        2 => (yv & 0xFF) << 8,
        else => return RunError.UnknownForm,
    };

    if (st_on) state.x_mem.put(gpa, st_addr, st_val) catch return RunError.OutOfMemory;
    if (main_dest != 0) state.a[main_dest] = main_val;
    if (copy_dest != 0) state.a[copy_dest] = copy_val;
    if (r_dest != 0) state.r[r_dest] = r_val;
    if (e1_dest != 0) state.e[e1_dest] = e1_val;
    if (e2_dest != 0) state.e[e2_dest] = s_src & 0xFFFF;
    if (sv) |v| {
        state.s[sd] = v;
        state.yd = yv; // 571
    }
}

/// x19aagu's table B (fact 342, 343): AGU10B with i at 8, 10 and 11.
fn x19aaguB(fn_op: u32, x: u32, y: u32, i: u32) u32 {
    const v: u32 = switch (fn_op & 0xF) {
        0 => x,
        1 => y,
        2, 9 => 0,
        3 => 0 -% x,
        4, 13 => 0 -% y,
        5 => 1,
        6 => x +% y,
        7 => x -% y,
        8 => i,
        10 => y +% i,
        11 => y -% i,
        12 => y,
        14 => y +% 1,
        else => y -% 1,
    };
    return v & 0xFFFF;
}

/// AGU10 with its immediate entries (fact 109, 341): 8 i, 10 y + i, 11 y - i.
fn agu10IEval(fn_op: u32, x: u32, y: u32, i: u32) RunError!u32 {
    return switch (fn_op & 0xF) {
        8 => i & 0xFFFF,
        10 => (y +% i) & 0xFFFF,
        11 => (y -% i) & 0xFFFF,
        else => agu10Eval(fn_op, x, y),
    };
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

    // w1 outside its operand fields: w1[3] set, the rest clear.  w1[20] and w1[5] write an A
    // (from the last X access's index) and grow the e step on later loads (793): refused.
    if (w1 & ~@as(u32, 0x0787) != 0x8) return RunError.UnknownForm;

    // 1) Load from Y memory using the pre-state element register, sign-extended (705).
    const src_e_val: u32 = if (y_src_e == 0) 0 else state.e[y_src_e];
    const loaded: u32 = sx16To24((state.getY(src_e_val) orelse 0) & 0xFFFF);
    state.s[dest_s] = loaded;

    // 2) AGU update: feed the same source register as x and y, then write back.
    const new_e = try agu10Eval(fn_op, src_e_val, src_e_val);
    state.e[dest_e] = new_e;

    // No store (707: 143's Y write came from its program's own trailing st.y).
    _ = gpa;
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
    const pair = bundle.len >= 2 and bundle[1] & 0xE0018000 == 0x20018000;
    if (pair) {
        // 765: with a class-001 store second the one store is Y:[e<w[20:17]>] <- s<w[24:21]>,
        // e as it stood before the bundle; w[25] inert, no lane or position dependence.
        const w = bundle[1];
        const v: u32 = state.s[(w >> 21) & 0xF] & 0xFFFF;
        state.y_mem.put(gpa, try yAddrE(state, (w >> 17) & 0xF), v) catch return RunError.OutOfMemory;
    } else {
        const value: u32 = state.v[state.cur_lane & 0x3];
        state.y_mem.put(gpa, addr, value) catch return RunError.OutOfMemory;
        if (bundle.len >= 2 and bundle[1] != 0) {
            const w = bundle[1];
            if (!(w >> 29 == 1 and (w >> 15) & 3 == 0)) return RunError.UnknownForm;
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
    if (s2_class == 0b101) return execX17Ms(gpa, state, bundle);
    // S1[22:20] = 010 on any other continue-shape form: its fields, then the stop
    // (fact 478: all 20 silicon rows stopped; the two-bundle shadow is the synthetic's).
    if (s1 & (7 << 20) == isa.STOP_ROW) {
        const b3 = [3]u32{ bundle[0], s1 & ~isa.STOP_ROW, s2 };
        // Forms with an operand on those bits (x17.ld, ...) read 010 as a field: not a stop.
        if (isa.matchSubForm(&b3)) |sf| if (isa.stopCapable(sf)) {
            try execX17(gpa, state, &b3);
            state.wedge_pending = true;
            return;
        };
    }
    if (s1_class == 0b011 and s2_class == 0b111) return execX17Imm(gpa, state, bundle);
    if (s2_class == 0b111) return execX17Alu(gpa, state, bundle);
    // Any S1 class but 000 (x17mve's) with S1[28]: 001 and 010 (609's row) measured, the rest
    // INFERRED from 113's "any S1[31:29]".
    if (s1_class != 0b000 and s2_class == 0b010 and (s1 >> 28) & 1 == 1) return execX17Agu(gpa, state, bundle);
    // S1 classes 011, 101 (fact 551) and 111 (619) run as 001.
    if (s1_class & 1 == 1 and s2_class == 0b000) return execX17Agu2(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b100) return execX17E(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b011) return execX17Ld(gpa, state, bundle);
    if (s2_class == 0b011) return execX17Sty(gpa, state, bundle);
    if (s1_class == 0b100 and s2_class == 0b100) return execX17Eagu(state, bundle);
    if (s1_class == 0b000 and s2_class == 0b000) return execX17Yld(state, bundle);
    if (s1_class == 0b000 and s2_class == 0b110) return execX17Alu05(gpa, state, bundle);
    if (s1_class == 0b000 and s2_class == 0b010) return execX17Mve(gpa, state, bundle);
    // x17sub ignores S1[31:29] (619, 620).
    if (s2_class == 0b001) return execX17Rsub(gpa, state, bundle);
    if (s1_class == 0b100 and s2_class == 0b110) return execX17Xpm(gpa, state, bundle);
    if ((s1_class == 0b001 or s1_class == 0b010) and s2_class == 0b110 and ((bundle[0] >> 27) & 1) == 0) return execX17Xld(state, bundle);
    return RunError.UnknownForm;
}

/// S2 class 101.  S1[22:20] = 011 is the stop, with its S2[1] store.  Otherwise four slots, reads before writes (fact
/// 410-413, own-x17st/-r/-e/-y).  Y:[e<S1[18:15]>] <- s<S1[27:24]>; R<S2[8:5]> and
/// e<{S2[0], S1[31:29]}> <- #h[20:5]; with S1[28], s<h[24:21]> <- mux S2[3:1] of the Y word at
/// e<S1[18:15]>: 0 gives 0, 2 the word, 6 its low byte << 8.  S1[22:20] = 010 runs the slots,
/// two more bundles and stops unhalted (457, 465); S1[19], S1[22] and other S1[21] rows wedge;
/// mux 2 sign-extends (414), mux 4 is the same word (436); S1[28] = 0 with mux 4 loads 0 in a plain program (490); muxes
/// 1/3/5 are OPEN.
fn execX17Ms(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s1 >> 20) & 7 == 0b011) {
        // The stop (halt row): S2[1] stores Y:[e<S1[16:15]>] <- reg<S1[28:24]> (fact
        // 22, 32, 47); the stop itself is the runner's.
        if (s2 & 2 != 0) {
            const k: u32 = (s1 >> 24) & 0x1F;
            if (k < 16) return RunError.UnknownForm;
            const v: u32 = if (k == 16) 0 else state.s[k - 16] & 0xFFFF;
            state.y_mem.put(gpa, state.e[(s1 >> 15) & 3], v) catch return RunError.OutOfMemory;
        }
        return;
    }
    // S1[22:20] = 010 runs the slots, then one more bundle, then stops unhalted (457).
    const wedge_row = (s1 >> 20) & 7 == 0b010;
    if ((s1 >> 19) & 1 == 1 or (s1 >> 22) & 1 == 1 or ((s1 >> 21) & 1 == 1 and !wedge_row)) return RunError.UnknownForm;
    if (wedge_row) {
        // With S1[27:24] = 0 silicon stored s1, not nothing (465): refused.
        if ((s1 >> 24) & 0xF == 0) return RunError.UnknownForm;
        state.wedge_pending = true;
    }
    const y_addr: u32 = state.e[(s1 >> 15) & 0xF];
    // S1[28] = 0 reads yd, the last Y read, instead of Y:[e] (550; 490's "0 in a plain program,
    // 0xA050 under x99" was yd).
    const yv: u32 = (if ((s1 >> 28) & 1 == 1) state.getY(y_addr) orelse 0 else state.yd) & 0xFFFF;
    const imm: u32 = (h >> 5) & 0xFFFF;
    const ld_dest: u32 = (h >> 21) & 0xF;
    var ld_val: u32 = 0;
    if (ld_dest != 0) {
        ld_val = switch ((s2 >> 1) & 7) {
            0 => 0,
            // mux 2 sign-extends, like ldy (414); mux 4 is the same word (436), its top byte
            // unmeasured and taken as 2's
            2, 4 => sx16To24(yv),
            6 => (yv & 0xFF) << 8,
            else => return RunError.UnknownForm,
        };
    }
    const st_src: u32 = (s1 >> 24) & 0xF;
    const st_val: u32 = state.s[st_src] & 0xFFFF;
    if (st_src != 0) state.y_mem.put(gpa, y_addr, st_val) catch return RunError.OutOfMemory;
    const r_dest: u32 = (s2 >> 5) & 0xF;
    if (r_dest != 0) state.r[r_dest] = imm;
    const e_dest: u32 = ((s2 & 1) << 3) | (s1 >> 29);
    if (e_dest != 0) state.e[e_dest] = imm;
    if (ld_dest != 0) state.s[ld_dest] = ld_val;
    if ((s1 >> 28) & 1 == 1) state.yd = yv; // the fresh read sets yd (571)
}

/// (S1 class != 0, S2 class 011) (fact 275): the x17ld bundle becomes
/// a Y store, Y:[e<S1[18:15]>] <- s<{S2[0], S1[31:29]}>, when S2[12:11] = 11.
/// With S2[3:1] = 1 or 5 (439): R<S1[14:11]> <- X17LD_OP[h[10:5]](s<S2[28:25]>, S1[26:23]),
/// S1[27] = 1 (0 wedges). Only row 0, a << n, is measured, and only with S2[12:11] = 10 (no Y
/// store seen); the other rows and the store beside it are refused.
fn execX17Sty(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const s1 = bundle[1];
    const s2 = bundle[2];
    const r_mode: u32 = (s2 >> 1) & 7;
    if (r_mode == 1 or r_mode == 5) {
        if ((s1 >> 27) & 1 == 0 or (bundle[0] >> 5) & 0x3F != 0 or (s2 >> 11) & 3 != 2) return RunError.UnknownForm;
        const a: u32 = state.s[(s2 >> 25) & 0xF];
        const r_dest: u32 = (s1 >> 11) & 0xF;
        if (r_dest != 0) state.r[r_dest] = (a << @intCast((s1 >> 23) & 0xF)) & 0xFFFF;
        return;
    }
    if ((s2 >> 11) & 3 != 3) return RunError.UnknownForm;
    // Mux 7 and 2 (781, own-x17stym2): R<S1[14:11]> <- AGU10[S1[10:7]](x = e<S1[18:15]>, the
    // store's own address register, y = R<h[30:27]>, i = #h[26:11]) (fact 543, own-x17styr);
    // mux 0: yd[15:0] (550).
    const h = bundle[0];
    const r_val: ?u32 = switch (r_mode) {
        2, 7 => try agu10ImmEval((s1 >> 7) & 0xF, state.e[(s1 >> 15) & 0xF], state.r[(h >> 27) & 0xF], (h >> 11) & 0xFFFF),
        0 => state.yd & 0xFFFF,
        else => null,
    };
    // S1[28:27] = 00 with S1[5] = 0 is a stop form keyed by S1[2:0] (648, 667): k = 1 stops
    // after the store iff p1, k = 3 runs away (refused), any other k stops.  A stop lands the
    // store once and ends the run with no completion (bit 2 never rises): `stop_now`.
    const stop_form = (s1 >> 27) & 3 == 0 and (s1 >> 5) & 1 == 0;
    if (stop_form and s1 & 7 == 3) return RunError.UnknownForm;
    const stops = stop_form and (s1 & 7 != 1 or state.p[1] != 0);
    // e<2*S1[2:0] + h[31]> <- AGU10[S1[10:7]](x = e<k>), written after the store (fact 658,
    // 429), measured with R mux 0 (fns x and x+1).  Under mux 2 it takes the R slot's AGU
    // result instead (801, measured on the stop form); elsewhere under a nonzero mux: refused.
    const ec_dest: u32 = ((s1 & 7) << 1) | (h >> 31);
    if (ec_dest != 0 and r_mode != 0 and !stops and !(r_mode == 2 and stop_form)) return RunError.UnknownForm;
    const k: u32 = (s1 >> 15) & 0xF;
    const e_post: u32 = if (ec_dest == 0) 0 else try agu10ImmEval((s1 >> 7) & 0xF, state.e[k], state.r[(h >> 27) & 0xF], (h >> 11) & 0xFFFF);
    const v: u32 = state.s[((s2 & 1) << 3) | (s1 >> 29)] & 0xFFFF;
    state.y_mem.put(gpa, state.e[k], v) catch return RunError.OutOfMemory;
    if (r_val) |r| {
        const dest: u32 = (s1 >> 11) & 0xF;
        if (dest != 0) state.r[dest] = r;
    }
    // Under mux 7 the e write lands elsewhere; after a stop nothing reads it.
    if (ec_dest != 0 and r_mode == 0) state.e[ec_dest] = e_post;
    if (ec_dest != 0 and r_mode == 2 and stop_form) {
        state.e[ec_dest] = r_val.?;
        // A stop whose side write moves its own store's e stores again at the new address
        // (801, measured with k = 1 under p1).
        if (stops and ec_dest == k) state.y_mem.put(gpa, state.e[k], v) catch return RunError.OutOfMemory;
    }
    if (stops) state.stop_now = true;
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
    const alu_dest: u32 = (s1 >> 25) & 0xF;
    const rd: u32 = (s1 >> 11) & 0xF;
    const r_from_cell = rd != 0 and (s1 >> 24) & 1 == 0;
    // 589, 591: the cell runs without an s destination too, feeding R.
    const alu_val: u32 = if (alu_dest == 0 and !r_from_cell) 0 else try x16Agu100Alu(state, h, s1, bundle[2]);
    const f: u32 = (s1 >> 7) & 0xF;
    const x: u32 = state.e[(s1 >> 15) & 0xF];
    const v: u32 = switch (f) {
        3 => 1,
        5 => 0xFFFF,
        else => if (X16_FN100[f]) |d| (x +% @as(u32, @bitCast(@as(i32, d)))) & 0xFFFF else 0,
    };
    // R<S1[14:11]> muxes on S1[24] (558): 1 the head's #imm, 0 the cell's value (589).
    const rv: u32 = if ((s1 >> 24) & 1 == 1) (h >> 5) & 0xFFFF else alu_val & 0xFFFF;
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) state.s[hd] = v;
    if (rd != 0) state.r[rd] = rv;
    if (alu_dest != 0) state.s[alu_dest] = alu_val;
}

/// The class-100 row's s ALU (fact 517, 518), gated on S2 & 0x01FFFFF8 = 0 alone:
/// s<S1[28:25]> <- cell (head[29:27], S2[2:0]) of (a = s<S2[28:25]>, n = S1[31:29]), 24-bit
/// wrapping.  Mux 0 even codes but 6 give a << n, odd but 7 a << (n + 8) (589); mux 1 even codes
/// a >> n (arithmetic, rounded half up, 589); mux 2 a + n, a - n, a, a + 1 by code pairs.  The rest is OPEN.  Outside
/// the gate X16_EXTRA (583) applies.
const X16Op = enum { b, add, sub, add1, a, inc, dec, asr8, xor, xnor, or_, and_, xor8, xnor8, or8, and8, rsh_rnd, lsh, inc_lsh, rsh, nrsh, nlsh, add_rsh, sub_rsh, inc_rsh, rsh_rnd_pos, add_asr8, sub_asr8, inc_asr8, rsub, neg, neg_rsh_rnd, not_b, zero };

/// The class-100 s ALU outside its gate by (head[30], head[29:27], S2[2:0]): fact 583
/// (head[30] = 0) and 584 (head[30] = 1).
const X16_EXTRA = [2][8][8]X16Op{ .{
    .{ .add, .add_asr8, .sub, .sub_asr8, .a, .asr8, .inc, .inc_asr8 },
    .{ .rsh_rnd, .lsh, .rsh_rnd, .lsh, .rsh_rnd, .lsh, .rsh_rnd, .inc_lsh },
    .{ .add, .add, .sub, .sub, .a, .a, .inc, .inc },
    .{ .add, .add, .sub, .sub, .a, .a, .inc, .inc },
    .{ .sub, .sub, .add1, .add1, .a, .a, .dec, .dec },
    .{ .xor, .xor8, .xnor, .xnor8, .or_, .or8, .and_, .and8 },
    .{ .rsh, .lsh, .nrsh, .nlsh, .rsh, .lsh, .zero, .zero },
    .{ .xor, .xor, .xnor, .xnor, .or_, .or_, .and_, .and_ },
}, .{
    .{ .b, .b, .b, .b, .b, .b, .b, .b },
    .{ .b, .b, .b, .b, .b, .b, .b, .b },
    .{ .b, .b, .b, .b, .b, .b, .b, .b },
    .{ .zero, .zero, .zero, .zero, .zero, .zero, .zero, .zero },
    .{ .a, .a, .a, .a, .a, .asr8, .a, .a },
    .{ .a, .a, .lsh, .lsh, .rsh_rnd_pos, .rsh_rnd_pos, .a, .a },
    .{ .zero, .zero, .zero, .zero, .zero, .zero, .zero, .zero },
    .{ .add, .add_rsh, .sub, .sub_rsh, .a, .rsh, .inc, .inc_rsh },
} };

/// fact 588: cells 8-15 (S2[3] set) by (head[30], head[29:27], S2[2:0]); null is OPEN.
const X16_EXTRA_HI = blk: {
    var t: [2][8][8]?X16Op = @splat(@splat(@splat(null)));
    for ([_]u32{ 0, 2 }) |mx| {
        t[0][mx][0] = .dec;
        t[0][mx][2] = .rsub;
    }
    t[0][0][4] = .neg;
    t[0][3][4] = .neg;
    t[0][0][6] = .a;
    t[0][1][4] = .neg_rsh_rnd;
    t[0][5][0] = .not_b;
    t[0][7][4] = .b;
    t[0][7][6] = .b;
    t[1][0][4] = .b;
    break :blk t;
};

/// fact 588: S2[8] measured inert, alone and with n = 0, at these (head[30], mx, cell).
fn x16S2b8Inert(h30: u32, mx: u32, c: u32) bool {
    const k = h30 * 64 + mx * 8 + c;
    return switch (k) {
        0, 1 * 8 + 4, 3 * 8 + 2, 7 * 8 + 4, 64 + 4 => true,
        else => false,
    };
}

/// One X16_EXTRA cell on 24-bit operands, 24-bit result (586, 587).  Rounded >> is arithmetic,
/// half up; >> 8 is arithmetic, truncating; every truncating >> b is logical in 24 bits; a
/// signed amount b below 0 on head[30] = 0's rounded >> shifts left, saturating at 0x7FFFFF
/// (582).  OPEN, so refused: other negative amounts, amounts of 0x20 or more, amounts of 15 or
/// more on a negative value, and a signed result outside 24 bits.
fn x16Extra(op: X16Op, a_raw: u32, b_raw: u32) RunError!u32 {
    if (op == .b) return b_raw; // 586: all 24 bits, raw
    const a: i64 = sx24(a_raw);
    const b: i64 = sx24(b_raw);
    const big_neg = a < 0 and b >= 15;
    const r: i64 = switch (op) {
        .b => unreachable,
        .add => a + b,
        .sub => a - b,
        .add1 => a + b + 1,
        .a => a,
        .inc => a + 1,
        .dec => a - 1,
        .asr8 => a >> 8,
        .add_asr8 => (a + b) >> 8,
        .sub_asr8 => (a - b) >> 8,
        .inc_asr8 => (a + 1) >> 8,
        .xor => a ^ b,
        .xnor => ~(a ^ b),
        .or_ => a | b,
        .and_ => a & b,
        .xor8 => (a ^ b) << 8,
        .xnor8 => ~(a ^ b) << 8,
        .or8 => (a | b) << 8,
        .and8 => (a & b) << 8,
        .rsh_rnd => if (b < 0) try satShl(a, -b) else try rshRound(a, b, big_neg),
        .rsh_rnd_pos => if (b < 0) return RunError.UnknownForm else try rshRound(a, b, big_neg),
        .lsh => if (b < 0 or b >= 0x20) return RunError.UnknownForm else try satShl(a, b),
        .inc_lsh => if (b < 0 or b >= 0x20) return RunError.UnknownForm else try satShl(a + 1, b),
        .rsh => try lsr24(a, b),
        .nrsh => try lsr24(~a, b),
        .nlsh => if (b < 0 or b >= 0x20) return RunError.UnknownForm else try satShl(~a, b),
        .add_rsh => try lsr24(a + b, b),
        .sub_rsh => try lsr24(a - b, b),
        .inc_rsh => try lsr24(a + 1, b),
        .rsub => b - a,
        .neg => -a,
        .neg_rsh_rnd => if (b < 0) return RunError.UnknownForm else try rshRound(-a, b, -a < 0 and b >= 15),
        .not_b => ~b,
        .zero => 0,
    };
    return in24(r);
}

/// Logical >> of v's 24 bits (587), as a signed value so in24 keeps the pattern.  OPEN, so
/// refused: a negative amount, 15 or more on a negative v (586), v past 24 bits.
fn lsr24(v: i64, b: i64) RunError!i64 {
    if (b < 0 or b >= 0x20 or (v < 0 and b >= 15)) return RunError.UnknownForm;
    const pat: u32 = try in24(v);
    return sx24(pat >> @intCast(b));
}

fn rshRound(a: i64, b: i64, big_neg: bool) RunError!i64 {
    if (big_neg or b >= 0x20) return RunError.UnknownForm;
    if (b == 0) return a;
    return (a + (@as(i64, 1) << @intCast(@min(b - 1, 62)))) >> @intCast(@min(b, 63));
}

/// A table result as 24 bits; one outside the signed 24-bit range is OPEN (586 saw no wrap at
/// 16 bits, never the 24-bit edge).
fn in24(r: i64) RunError!u32 {
    if (r < -0x800000 or r > 0x7FFFFF) return RunError.UnknownForm;
    return @truncate(@as(u64, @bitCast(r)) & M24);
}

/// fact 585: the X16_EXTRA cells (head[30] = 0) measured with n = S1[31:29] != 0.
fn x16ExtraN(cell: u32, a_raw: u32, b_raw: u32, n: u32) RunError!u32 {
    const a: i64 = sx24(a_raw);
    const b: i64 = sx24(b_raw);
    const r: i64 = switch (cell) {
        0 => (a + b) >> @intCast(n), // mx 0 c 0, arithmetic (586)
        8 * 5 => try satShl(a ^ b, n), // mx 5 c 0
        8 * 1 + 1 => if (b < 0) return RunError.UnknownForm else try satShl(a + n, b), // mx 1 c 1
        8 * 2 + 2 => a - b, // mx 2 c 2: n ignored
        else => return RunError.UnknownForm,
    };
    return in24(r);
}

fn satShl(v: i64, n: i64) RunError!i64 {
    const r: i64 = v << @intCast(@min(n, 40));
    if (r > 0x7FFFFF) return 0x7FFFFF;
    if (r < -0x800000) return RunError.UnknownForm;
    return r;
}

fn x16Agu100Alu(state: *State, h: u32, s1: u32, s2: u32) RunError!u32 {
    // S1[4:3] = 01 (623): head[30] = 1's mx 0 cell 4 move reads a instead of b; every other cell
    // is unmeasured there and refuses.  S1[2] is inert.
    if ((s1 >> 3) & 3 == 1) {
        const move = (h >> 30) & 1 == 1 and (h >> 25) & 0x1F == 3 and s2 & 0x01FFFF0F == 4 and s1 >> 29 == 0;
        if (!move) return RunError.UnknownForm;
        return state.s[(s2 >> 25) & 0xF] & M24;
    }
    // With S2 bits in 0x01FFFFF8 (580-588): b = s<S2[7:4]>, cell S2[3:0].  Cells 0-7 are
    // X16_EXTRA[head[30]][head[29:27]][c], 8-15 X16_EXTRA_HI (refused where null); n =
    // S1[31:29] != 0 only in 585's and 588's cells; S2[8] only alone at X16_S2B8_INERT.
    if (s2 & 0x01FFFFF8 != 0) {
        // The row is head[29:25] (590): X16_EXTRA was measured at head[26:25] = 3, a few cells at
        // 2 (591).  The rest OPEN.
        if (s2 & 0x01FFFE00 != 0) return RunError.UnknownForm;
        const src: u32 = (s2 >> 4) & 0xF;
        const b: u32 = if (src == 0) 0 else state.s[src] & M24;
        const h30: u32 = (h >> 30) & 1;
        const mx: u32 = (h >> 27) & 7;
        const c: u32 = s2 & 0xF;
        const a: u32 = state.s[(s2 >> 25) & 0xF] & M24;
        const n: u32 = s1 >> 29;
        if ((h >> 25) & 3 == 2) {
            // head[26:25] = 2 (581, 591), n = 0: the move (head[30] = 1, mx 0, c 0 / 4),
            // a << b (mx 1 c 5), a (mx 3 c 4, S2[8] inert there), 0 (head[30] = 1, mx 0 c 12).
            if (n != 0) return RunError.UnknownForm;
            const b8 = (s2 >> 8) & 1 == 1;
            const op: X16Op = switch (h30 * 128 + mx * 16 + c) {
                128, 128 + 4 => .b,
                16 + 5 => .lsh,
                3 * 16 + 4 => .a,
                128 + 12 => .zero,
                else => return RunError.UnknownForm,
            };
            if (b8 and op != .a) return RunError.UnknownForm;
            return x16Extra(op, a, b);
        }
        if ((h >> 25) & 3 != 3) return RunError.UnknownForm;
        if ((s2 >> 8) & 1 == 1) {
            if (s2 & 0x01FFFFF8 != 0x100 or n != 0) return RunError.UnknownForm;
            if (!x16S2b8Inert(h30, mx, c)) return RunError.UnknownForm;
        }
        if (c >= 8) {
            if (n != 0) return RunError.UnknownForm;
            return x16Extra(X16_EXTRA_HI[h30][mx][c - 8] orelse return RunError.UnknownForm, a, b);
        }
        if (n == 0) return x16Extra(X16_EXTRA[h30][mx][c], a, b);
        if (h30 == 1) {
            // 588: mx 0 c 2/4/6 with n in {2, 5, 6, 7} runs mx 2's cell, else the move.
            if (mx != 0 or c & 1 == 1 or c == 0) return RunError.UnknownForm;
            const alu = n == 2 or n >= 5;
            return x16Extra(if (alu) X16_EXTRA[0][2][c] else .b, a, b);
        }
        return x16ExtraN(mx * 8 + c, a, b, n);
    }
    // The row is head[29:25] (590); the gate grid holds at head[26:25] = 0 and head[30] = 0
    // only (with head[30] = 1 its R feed read 0, 589).  591 measured two more cells giving a:
    // head[26:25] = 2 mx 3 c 4 n 0, and head[26:25] = 0 mx 4 c 4 n 7.
    if ((h >> 30) & 1 == 0 and s2 & 7 == 4) {
        const row: u32 = (h >> 25) & 0x1F;
        if ((row == 3 * 4 + 2 and s1 >> 29 == 0) or (row == 4 * 4 and s1 >> 29 == 7))
            return state.s[(s2 >> 25) & 0xF] & M24;
    }
    if ((h >> 25) & 3 != 0 or (h >> 30) & 1 == 1) return RunError.UnknownForm;
    const a: u32 = state.s[(s2 >> 25) & 0xF] & M24;
    const n: u5 = @intCast(s1 >> 29);
    const code: u32 = s2 & 7;
    const r: u32 = switch ((h >> 27) & 7) {
        // 589: odd cells are a << (n + 8); mx 1's >> rounds half up.
        0 => if (code == 6 or code == 7) return RunError.UnknownForm else if (code & 1 == 0) a << n else a << (n + 8),
        1 => if (code & 1 == 0) @truncate(@as(u64, @bitCast(try rshRound(sx24(a), n, false)))) else return RunError.UnknownForm,
        2 => switch (code >> 1) {
            0 => a +% n,
            1 => a -% n,
            2 => a,
            else => a +% 1,
        },
        else => return RunError.UnknownForm,
    };
    return r & M24;
}

/// x16 b16 (fact 90, 778): S1 class 001 / S2 class 111, or any S1 class with S1[18] = 1 and
/// S2 & 0xE0050000 = 0xE0040000.  Y load s<head[24:21]> <- Y:[e<S2[24:21]>] placed by
/// S1[13:12] (00: 0, 01: sign-extended, 10: zero-extended (800), 11: << 8); s<S1[11:8]> <- s<S2[28:25]> (a move,
/// 778); X store X:[xa] <- sx24 s<S1[31:28]> with S1[18], S2[14:13] = 11, S2[11] = 0, using
/// xa as it was before the bundle; then xa <- head[20:5] when head[31] and S1[3].  The move
/// and the store read before the bundle's writes.
fn execX16B16(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    const yv: u32 = (state.getY(state.e[(s2 >> 21) & 0xF]) orelse 0) & 0xFFFF;
    const hv: u32 = switch ((s1 >> 12) & 3) {
        0 => 0,
        1 => sx16To24(yv),
        2 => yv, // zero-extended (800)
        3 => (yv << 8) & M24,
        else => unreachable,
    };
    const mv_dest: u32 = (s1 >> 8) & 0xF;
    const mv_val: u32 = state.s[(s2 >> 25) & 0xF] & M24;
    if ((s1 >> 18) & 1 == 1 and (s2 >> 13) & 3 == 3 and (s2 >> 11) & 1 == 0) {
        if (state.xa) |addr| {
            state.x_mem.put(gpa, addr, state.s[(s1 >> 28) & 0xF] & M24) catch return RunError.OutOfMemory;
        }
    }
    if (h >> 31 == 1 and (s1 >> 3) & 1 == 1) state.xa = (h >> 5) & 0xFFFF;
    if (mv_dest != 0) state.s[mv_dest] = mv_val;
    const hd: u32 = (h >> 21) & 0xF;
    if (hd != 0) state.s[hd] = hv;
}

/// A 16-bit word sign-extended to 24 bits, as ldy loads one (fact 409).
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

/// x16alu with its post-op code (fact 563, 564): codes 11, 15, 63, and 3 with n = 0, keep
/// the plain ALU; any other code runs X16_POST's cell for (op, S1[15]), refused where unmeasured.
fn x16AluPost(state: *State, code: u32, s1: u32, s2: u32) RunError!void {
    const n: u32 = (s2 >> 4) & 0xF;
    const plain = code == 11 or code == 15 or code == 63 or (code == 3 and n == 0);
    if (plain) return x16Alu(state, s1, s2);
    const a: i64 = sx16(state.s[(s2 >> 25) & 0xF]);
    const b: i64 = sx16(state.s[(s1 >> 4) & 0xF]);
    const v = x16Post((s1 >> 16) & 7, (s1 >> 15) & 1, code, a, b, @intCast(n)) orelse return RunError.UnknownForm;
    const dest: u32 = (s1 >> 8) & 0xF;
    if (dest != 0) state.s[dest] = @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF;
}

const PostShift = enum { none, shl, asr, asr_rnd, imm_n, b_is_n };
const PostFn = enum { a, b, zero, neg1, add, sub, rsub, a_or_b, a_xor_b, a_and_b, xnor, neg_a, neg_a2, neg_ab2, a1, a_m1, not_a, not_ab, ab1 };
const PostCell = struct { sh: PostShift, f: PostFn };
const PostRow = [64]?PostCell;
const PostTable = struct { o50: PostRow, o40: PostRow, o51: PostRow, o41: PostRow, o70: PostRow, o61: PostRow };

fn postFn(f: PostFn, a: i64, b: i64) i64 {
    return switch (f) {
        .a => a,
        .b => b,
        .zero => 0,
        .neg1 => -1,
        .add => a + b,
        .sub => a - b,
        .rsub => b - a,
        .a_or_b => a | b,
        .a_xor_b => a ^ b,
        .a_and_b => a & b,
        .xnor => ~(a ^ b),
        .neg_a => -a,
        .neg_a2 => -a - 2,
        .neg_ab2 => -a - b - 2,
        .a1 => a + 1,
        .a_m1 => a - 1,
        .not_a => ~a,
        .not_ab => ~(a + b),
        .ab1 => a + b + 1,
    };
}

/// The post-op on 16-bit signed operands; the shift sees the exact value (0x5505 + 0x4404 asr 4
/// = 0x0990).  null: unmeasured.
fn x16Post(op: u32, s15: u32, code: u32, a: i64, b: i64, n: u6) ?i64 {
    const row: *const PostRow = switch (op << 1 | s15) {
        5 << 1 => &X16_POST.o50,
        4 << 1 => &X16_POST.o40,
        5 << 1 | 1 => &X16_POST.o51,
        4 << 1 | 1 => &X16_POST.o41,
        7 << 1 => &X16_POST.o70,
        6 << 1 | 1 => &X16_POST.o61,
        else => return null,
    };
    const c = row[code] orelse return null;
    return switch (c.sh) {
        .none => postFn(c.f, a, b),
        .shl => postFn(c.f, a, b) << n,
        .asr => postFn(c.f, a, b) >> n,
        .asr_rnd => asrRound(postFn(c.f, a, b), n),
        .imm_n => n,
        .b_is_n => postFn(c.f, a, n),
    };
}

pub const x16PostForTest = x16Post;

/// fact 563/564, one row per (op, S1[15]) the sweeps covered; missing codes are OPEN.
const X16_POST: PostTable = blk: {
    @setEvalBranchQuota(20000);
    const G = struct { codes: []const u6, sh: PostShift, f: PostFn };
    const fill = struct {
        fn row(groups: []const G) PostRow {
            var r: PostRow = @splat(null);
            for (groups) |g| for (g.codes) |c| {
                r[c] = .{ .sh = g.sh, .f = g.f };
            };
            return r;
        }
        // The groups ops 4 and 5 share at S1[15] = 0, over (native, logic).
        fn base0(nat: PostFn, lg: PostFn) [13]G {
            return .{
                .{ .codes = &.{ 0, 1, 2 }, .sh = .shl, .f = nat },
                .{ .codes = &.{58}, .sh = .shl, .f = .a },
                .{ .codes = &.{ 3, 5 }, .sh = .asr, .f = nat },
                .{ .codes = &.{4}, .sh = .asr_rnd, .f = nat },
                .{ .codes = &.{ 11, 12, 13, 14, 15, 60, 61, 63 }, .sh = .none, .f = nat },
                .{ .codes = &.{ 22, 23 }, .sh = .shl, .f = lg },
                .{ .codes = &.{ 24, 25 }, .sh = .asr, .f = lg },
                .{ .codes = &.{ 29, 30, 31, 37 }, .sh = .none, .f = lg },
                .{ .codes = &.{ 32, 33, 34, 59 }, .sh = .none, .f = .zero },
                .{ .codes = &.{ 35, 36, 38, 39, 42, 43 }, .sh = .none, .f = .b },
                .{ .codes = &.{ 40, 41 }, .sh = .imm_n, .f = .zero },
                .{ .codes = &.{44}, .sh = .shl, .f = .b },
                .{ .codes = &.{45}, .sh = .asr_rnd, .f = .b },
            };
        }
        // The groups ops 4 and 5 share at S1[15] = 1.
        fn base1(nat: PostFn, lg: PostFn) [12]G {
            return .{
                .{ .codes = &.{ 0, 1, 2 }, .sh = .shl, .f = nat },
                .{ .codes = &.{ 3, 5 }, .sh = .asr, .f = nat },
                .{ .codes = &.{4}, .sh = .asr_rnd, .f = nat },
                .{ .codes = &.{ 11, 12, 13, 14, 15, 60, 61, 63 }, .sh = .none, .f = nat },
                .{ .codes = &.{ 22, 23 }, .sh = .shl, .f = lg },
                .{ .codes = &.{ 24, 25 }, .sh = .asr, .f = lg },
                .{ .codes = &.{ 29, 30, 31, 37 }, .sh = .none, .f = lg },
                .{ .codes = &.{ 6, 7, 8, 27, 32, 33, 34, 46, 47, 57, 59, 62 }, .sh = .none, .f = .zero },
                .{ .codes = &.{ 35, 36 }, .sh = .none, .f = .b },
                .{ .codes = &.{ 40, 41 }, .sh = .imm_n, .f = .zero },
                .{ .codes = &.{ 48, 49, 50, 51, 52, 53 }, .sh = .none, .f = .a },
                .{ .codes = &.{58}, .sh = .shl, .f = .a },
            };
        }
        // The groups ops 7 (S1[15] = 0) and 6 (S1[15] = 1) share.
        fn base67(nat: PostFn) [9]G {
            return .{
                .{ .codes = &.{ 0, 1, 2 }, .sh = .shl, .f = nat },
                .{ .codes = &.{ 3, 5 }, .sh = .asr, .f = nat },
                .{ .codes = &.{4}, .sh = .asr_rnd, .f = nat },
                .{ .codes = &.{ 11, 12, 13, 14, 15, 60, 61, 63 }, .sh = .none, .f = nat },
                .{ .codes = &.{6}, .sh = .none, .f = .neg1 },
                .{ .codes = &.{ 7, 8, 26, 27, 32, 33, 34, 46, 47, 57 }, .sh = .none, .f = .zero },
                .{ .codes = &.{ 35, 36 }, .sh = .none, .f = .b },
                .{ .codes = &.{ 48, 49, 50 }, .sh = .shl, .f = .a },
                .{ .codes = &.{ 51, 52, 53, 58, 62 }, .sh = .asr, .f = .a },
            };
        }
    };
    const t: PostTable = .{
        .o50 = fill.row(&(fill.base0(.a, .a_or_b) ++ [_]G{
            .{ .codes = &.{ 48, 49, 50 }, .sh = .shl, .f = .a },
            .{ .codes = &.{ 51, 53 }, .sh = .asr, .f = .a },
            .{ .codes = &.{52}, .sh = .asr_rnd, .f = .a },
            .{ .codes = &.{ 9, 10, 18, 19 }, .sh = .none, .f = .a },
            .{ .codes = &.{28}, .sh = .b_is_n, .f = .a_or_b },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .neg_a },
            .{ .codes = &.{ 20, 21 }, .sh = .none, .f = .neg_a2 },
        })),
        .o40 = fill.row(&(fill.base0(.add, .a_xor_b) ++ [_]G{
            .{ .codes = &.{ 48, 49, 50, 51, 52, 53, 54, 55, 56 }, .sh = .none, .f = .a },
            .{ .codes = &.{ 6, 7, 8, 26, 27, 46, 47, 57, 62 }, .sh = .none, .f = .zero },
            .{ .codes = &.{ 9, 10 }, .sh = .b_is_n, .f = .add },
            .{ .codes = &.{28}, .sh = .b_is_n, .f = .a_xor_b },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .rsub },
            .{ .codes = &.{ 18, 19 }, .sh = .none, .f = .sub },
            .{ .codes = &.{ 20, 21 }, .sh = .none, .f = .neg_ab2 },
        })),
        .o51 = fill.row(&(fill.base1(.a1, .a_and_b) ++ [_]G{
            .{ .codes = &.{ 9, 10 }, .sh = .none, .f = .a1 },
            .{ .codes = &.{ 54, 55, 56 }, .sh = .none, .f = .a },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .neg_a },
            .{ .codes = &.{ 18, 19 }, .sh = .none, .f = .a_m1 },
            .{ .codes = &.{ 20, 21 }, .sh = .none, .f = .not_a },
            .{ .codes = &.{26}, .sh = .none, .f = .zero },
            .{ .codes = &.{28}, .sh = .b_is_n, .f = .a_and_b },
            .{ .codes = &.{ 38, 39, 42, 43 }, .sh = .none, .f = .b },
            .{ .codes = &.{44}, .sh = .shl, .f = .b },
            .{ .codes = &.{45}, .sh = .asr_rnd, .f = .b },
        })),
        // 38, 39, 42, 43 and 44 OPEN.
        .o41 = fill.row(&(fill.base1(.sub, .xnor) ++ [_]G{
            .{ .codes = &.{ 9, 10 }, .sh = .b_is_n, .f = .sub },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .not_ab },
            .{ .codes = &.{ 18, 19 }, .sh = .none, .f = .ab1 },
            .{ .codes = &.{ 20, 21 }, .sh = .none, .f = .rsub },
            .{ .codes = &.{ 26, 55 }, .sh = .none, .f = .neg1 },
            .{ .codes = &.{28}, .sh = .b_is_n, .f = .xnor },
            .{ .codes = &.{45}, .sh = .asr_rnd, .f = .a },
            .{ .codes = &.{ 54, 56 }, .sh = .none, .f = .zero },
        })),
        .o70 = fill.row(&(fill.base67(.neg_a) ++ [_]G{
            .{ .codes = &.{ 9, 10 }, .sh = .none, .f = .neg_a },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .a1 },
            .{ .codes = &.{ 18, 19 }, .sh = .none, .f = .not_a },
            .{ .codes = &.{ 20, 21, 38, 39, 40, 41, 42, 43, 54, 55, 56 }, .sh = .none, .f = .a },
            .{ .codes = &.{ 22, 23 }, .sh = .shl, .f = .b },
            .{ .codes = &.{ 24, 25 }, .sh = .asr, .f = .b },
            .{ .codes = &.{ 29, 30, 31, 37 }, .sh = .none, .f = .b },
            .{ .codes = &.{28}, .sh = .imm_n, .f = .zero },
            .{ .codes = &.{44}, .sh = .shl, .f = .a },
            .{ .codes = &.{45}, .sh = .asr_rnd, .f = .a },
            .{ .codes = &.{59}, .sh = .none, .f = .zero },
        })),
        // The logic codes (22-25, 28-31, 37) are OPEN.
        .o61 = fill.row(&(fill.base67(.rsub) ++ [_]G{
            .{ .codes = &.{ 9, 10 }, .sh = .b_is_n, .f = .rsub },
            .{ .codes = &.{ 16, 17 }, .sh = .none, .f = .ab1 },
            .{ .codes = &.{ 18, 19 }, .sh = .none, .f = .not_ab },
            .{ .codes = &.{ 20, 21 }, .sh = .none, .f = .sub },
            .{ .codes = &.{ 38, 39, 42, 43, 59 }, .sh = .none, .f = .b },
            .{ .codes = &.{ 40, 41 }, .sh = .none, .f = .a },
            .{ .codes = &.{44}, .sh = .shl, .f = .b },
            .{ .codes = &.{45}, .sh = .asr_rnd, .f = .b },
            .{ .codes = &.{ 54, 56 }, .sh = .none, .f = .zero },
            .{ .codes = &.{55}, .sh = .none, .f = .neg1 },
        })),
    };
    break :blk t;
};

/// x asr n rounded half up.
fn asrRound(x: i64, n: u6) i64 {
    return if (n == 0) x else (x + (@as(i64, 1) << (n - 1))) >> n;
}

/// An alu_word (op, m, #s) triple a slot has measured OPEN.
const AluTriple = struct { op: AluOp, m: u32, s: u32 };

/// Whether alu_word `word` (code [10:5], m [25:23], #s [22:19]) is on a slot's OPEN list.
fn aluOpen(open: []const AluTriple, word: u32) bool {
    for (open) |t| {
        if (ALU05[(word >> 5) & 0x3F] == t.op and (word >> 23) & 7 == t.m and (word >> 19) & 0xF == t.s) return true;
    }
    return false;
}

/// x17xld's head ALU slot (528): add (code 0x01) with m = 5, #12.
const xld_alu_open = [_]AluTriple{.{ .op = .add, .m = 5, .s = 12 }};

/// x17xld, S1 class 001 / 010 + S2 class 110, head[27] clear: s<S2[13:10]> <- X:[A<8*S2[1:0] +
/// S1[31:29]>] (fact 141); and x17a05's head ALU slot, s<h[18:15]> <- alu_word(h,
/// s<h[14:11]>, s<S2[28:25]>) (528, own-x17xla).  Both read before either writes; the pairs
/// alu_word leaves OPEN, and one s for both writes, are refused.
fn execX17Xld(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const s2 = bundle[2];
    const ld_dest: u32 = (s2 >> 10) & 0xF;
    const alu_dest: u32 = (h >> 15) & 0xF;
    if (alu_dest != 0 and alu_dest == ld_dest) return RunError.UnknownForm;
    if (alu_dest != 0 and aluOpen(&xld_alu_open, h)) return RunError.UnknownForm;
    const alu_v: ?u32 = if (alu_dest == 0) null else try aluWord(h, sx24(state.s[(h >> 11) & 0xF]), sx24(state.s[(s2 >> 25) & 0xF])) orelse
        return RunError.UnknownForm;
    const a_idx: u32 = (8 * (s2 & 3) + (bundle[1] >> 29)) & 0x1F;
    state.s[ld_dest] = state.readX(state.a[a_idx]) orelse 0;
    if (alu_v) |v| state.s[alu_dest] = v;
}

fn execX17Xpm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const s2 = bundle[2];

    const dest_s: u32 = (s2 >> 10) & 0xF;
    const a_half: u32 = s2 & 0x3;
    const a_idx: u32 = (8 * a_half + 4) & 0x1F;

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

/// x17agu2's bare alu05 op (fact 629); index 0 reads 0 (s0 is 0 here).
fn x17Agu2Alu(head: u32, a_raw: u32, b_raw: u32) RunError!u32 {
    const a = a_raw & M24;
    const b = b_raw & M24;
    return switch (ALU05[(head >> 5) & 0x3F]) {
        .add => (a +% b) & M24,
        .sub => (a -% b) & M24,
        .rsub => (b -% a) & M24,
        .xor_op => a ^ b,
        .mova => a,
        .movb, .addi => b,
        .clr => 0,
        .asr => alu05Shift(a, b, .asr),
        else => RunError.UnknownForm,
    };
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
    // 780: e<2*S1[2:0] + h[31]>.
    const dest_e: u32 = (2 * (s1 & 0x7) + (head >> 31)) & 0xF;
    const y_val: u32 = if (y_r == 0) 0 else state.r[y_r];
    // v = AGU10[S1[10:7]](x = e<S2[24:21]>, y = R<h[30:27]>, i = h[26:11]) (631); fn 11 is
    // y - i here (780, own-a2f11; still refused in x17ld, 443).
    const x_e: u32 = (s2 >> 21) & 0xF;
    const result = if (fn_op == 11) (y_val -% imm) & 0xFFFF else try agu10ImmEval(fn_op, if (x_e == 0) 0 else state.e[x_e], y_val, imm);
    // S2[5:2] = k != 0 (630): e<k> <- e<{S2[1:0], S1[31:30]}> - 1, read before the writes.
    const dec_k: u32 = (s2 >> 2) & 0xF;
    const dec_v: u32 = (state.e[((s2 & 3) << 2) | (s1 >> 30)] -% 1) & 0xFFFF;
    if (dec_k != 0 and dec_k == dest_e) return RunError.UnknownForm;
    // s<S1[22:19]> <- the bare alu05 op h[10:5] (629): no h[25:19] modifiers (they are the
    // immediate).  Each class was measured on one code; the class's other codes are INFERRED.
    const alu_d: u32 = (s1 >> 19) & 0xF;
    const alu_v: u32 = if (alu_d == 0) 0 else try x17Agu2Alu(head, state.s[(s1 >> 15) & 0xF], state.s[(s2 >> 25) & 0xF]);
    if (alu_d != 0 and alu_d == dest_s) return RunError.UnknownForm;
    if (alu_d != 0) state.s[alu_d] = alu_v;
    if (dest_s != 0) state.s[dest_s] = result;
    if (dest_e != 0) state.e[dest_e] = result;
    if (dec_k != 0) state.e[dec_k] = dec_v;
    // R<S1[14:11]> takes it too, x17ld's R field (fact 540, own-x17a2r).
    const dest_r: u32 = (s1 >> 11) & 0xF;
    if (dest_r != 0) state.r[dest_r] = result;
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

    // S1[27] = 0 (fact 443, 460): with S2[2], R<S1[14:11]> <- AGU10[S1[10:7]](y =
    // R<h[30:27]>, i = #h[26:11]); without it no R write. Only fn 10 (y + i) is measured (fn 11
    // wedges), and only with S1[22:15] and S1[2:0] zero, so whether the asr and Y slots still run
    // beside it is unmeasured: anything else is refused.
    if ((s1 >> 27) & 1 == 0) {
        if ((s1 >> 15) & 0xFF != 0 or s1 & 7 != 0) return RunError.UnknownForm;
        if ((s2 >> 2) & 1 == 0) return;
        if ((s1 >> 7) & 0xF != 10) return RunError.UnknownForm;
        const h = bundle[0];
        const r_dest: u32 = (s1 >> 11) & 0xF;
        if (r_dest != 0) state.r[r_dest] = try agu10ImmEval(10, 0, state.r[(h >> 27) & 0xF], (h >> 11) & 0xFFFF);
        return;
    }

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

    // S1[23:20] != 0: of 17 silicon rows 9 run, 4 arm the halt deadline and 4 stop, and no
    // field predicts which (fact 481): refused.
    if ((s1 >> 20) & 0xF != 0) return RunError.UnknownForm;
    const dest_s: u32 = (head >> 21) & 0xF;
    const imm: u32 = (head >> 5) & 0xFFFF;
    // S2[3:0] = 4 stops the program one bundle later and loads s<h[24:21]> from a counter that
    // rises 9 a run (fact 448); its value is unmeasured, so a dest is refused.
    if (s2 & 0xF == 4) {
        if (dest_s != 0 or (s1 >> 17) & 3 != 0) return RunError.UnknownForm;
        state.stop_pending = true;
        return;
    }
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

/// x17imm, S1 class 011 + S2 class 111 (fact 134, 537): s<S1[10:7]> <- #h[26:11]; and the
/// ALU slot T = alu_word(code h[10:5], m = S1[13:11], #S1[23:20], a = s<S1[19:16]>, b =
/// s<S2[28:25]>) into s<S1[27:24]> and R<S2[16:13]>, from pre-bundle operands.  Refused: the
/// OPEN triples, the Y slot (S2[24:21] != 0), and two writes to one s.
fn execX17Imm(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    const h = bundle[0];
    const s1 = bundle[1];
    const s2 = bundle[2];
    if ((s2 >> 21) & 0xF != 0) return RunError.UnknownForm;
    const imm_dest: u32 = (s1 >> 7) & 0xF;
    const t_dest: u32 = (s1 >> 24) & 0xF;
    const r_dest: u32 = (s2 >> 13) & 0xF;
    if (t_dest != 0 and t_dest == imm_dest) return RunError.UnknownForm;
    const word: u32 = (h & (0x3F << 5)) | (((s1 >> 11) & 7) << 23) | (((s1 >> 20) & 0xF) << 19);
    const t: ?u32 = if (t_dest == 0 and r_dest == 0) null else blk: {
        // 537's OPEN triples (xori m3 #15, sel m1 #4) are outside alu_word's domain already.
        break :blk try aluWord(word, sx24(state.s[(s1 >> 16) & 0xF]), sx24(state.s[(s2 >> 25) & 0xF])) orelse return RunError.UnknownForm;
    };
    state.s[imm_dest] = (h >> 11) & 0xFFFF;
    if (t) |v| {
        state.s[t_dest] = v;
        if (r_dest != 0) state.r[r_dest] = v & 0xFFFF;
    }
}

// ---------------------------------------------------------------------------
// ALU05: the 64-entry class-05 ALU table shared across many sm1 forms.
// Ported from the Python ISA model (fact 66+).
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
/// Mirrors the Python ISA model's `alu05()` with the shift ops dispatched through
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
    // the Python model\'s shift(): n = sx24(a); v = (b & M24) if lsr else sx24(b).
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

// ADDER_M table rewrites the (x, y, c) tuple fed into x + y + c, per the Python model
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
        // 238, measured with a negative b after 594: |y| for add (c = 0); sub is refused upstream.
        7 => .{ .x = 0, .y = if (y < 0) -y else y, .c = 0 },
        else => unreachable,
    };
}

/// XOR_M: for xor at code 0x16 with s = 0, [25:23] = m picks the combiner.
/// Entry 5 is OPEN in the Python model.  Returns null for OPEN.
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

/// the Python ISA model's `alu_word`: one class-05 word with its [25:19]
/// modifiers resolved.  Returns null for cases the Python model marks OPEN (clr with
/// modifiers), and `RunError.UnknownForm` for a (code, m, s) triple no rule
/// in the Python model covers either.
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
    // 779: code 0x28 is a move, not a clear (0x29 INFERRED the same); a is inert, # is
    // [22:19] zero-extended.  m 5/6: d <- s<b> (# when b's index is 0); m 1/4: d <- #
    // (0 when b's index is 0); m 0/2/3/7: d <- #.
    if (code == 0x28 or code == 0x29) {
        const b_zero = (h >> 26) & 0xF == 0;
        return switch (m) {
            5, 6 => if (b_zero) s_mod else b24,
            1, 4 => if (b_zero) 0 else s_mod,
            else => s_mod,
        };
    }
    // clr with modifiers: 0x22 writes 0 for every m and # (775); the other clr codes are OPEN.
    if (op == .clr) return if (code == 0x22) 0 else null;
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
        // sub with m 7 fits neither |~b| nor the carry-kept form (measured after 594): OPEN.
        if (op == .sub and m & 7 == 7) return RunError.UnknownForm;
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

/// jr2 (fact 302, 303, 617): the head is a 0x05 ALU word (head[31:30] inert) and the
/// bundle jumps unconditionally to q + 2 * sx14(w1[18:5]) after two delay slots; w1[20:19]
/// inert.
fn execJr2(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    try execAlu(gpa, state, bundle[0]);
    const off: i32 = @as(i32, @bitCast(((bundle[1] >> 5) & 0x3FFF) << 18)) >> 18;
    state.pending_jmp_offset = 2 * off;
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
// the Python ISA model (fact 109).  Values compute over (x, y) and wrap at
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

/// AGU10 in the forms that carry an immediate i = #h[26:11] (fact 443, 540, 543): fn 10 is
/// y + i; the rest as agu10Eval, which refuses fn 8 (i, unmeasured) and fn 11 (y - i, wedges).
fn agu10ImmEval(fn_op: u32, x: u32, y: u32, i: u32) RunError!u32 {
    if (fn_op & 0xF == 10) return (y +% i) & 0xFFFF;
    return agu10Eval(fn_op, x, y);
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
    // s<S1[23:20]> takes the same value (609), measured for fn 0 (x) only.
    const dest_s: u32 = (s1 >> 20) & 0xF;
    if (dest_s != 0 and fn_op != 0) return RunError.UnknownForm;
    if (dest != 0) state.r[dest] = v;
    if (dest_s != 0) state.s[dest_s] = v;
}

/// sm1 ALU is 24-bit wrapping arithmetic.  the Python ISA model calls this
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

/// Ported from the Python ISA model X17OP.
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
    // S1 class 000 (554): S2[4:3] = 11 also stores Y:[e<S2[24:21]>] <- s<2*S2[2:0]>, the value
    // before this bundle's write; S2[4:3] = 01 and a zero source index are OPEN.
    if (s1 >> 29 == 0) switch ((s2 >> 3) & 3) {
        3 => {
            const src: u32 = 2 * (s2 & 7);
            if (src == 0) return RunError.UnknownForm;
            const addr = try yAddrE(state, (s2 >> 21) & 0xF);
            state.y_mem.put(gpa, addr, state.s[src] & 0xFFFF) catch return RunError.OutOfMemory;
        },
        1 => return RunError.UnknownForm,
        else => {},
    };
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

/// The host's two arms (770): pass-1 (+0x984) enters q0; decode (+0x98C) enters at the engine-
/// order syllable the host wrote to +0x988.
/// Called with the pc and syllables of a refused bundle that is then skipped.
pub const UnknownHook = struct {
    ctx: *anyopaque,
    f: *const fn (ctx: *anyopaque, pc: usize, bundle: []const u32) void,
};

pub const Arm = union(enum) { pass1, decode: usize };

/// Run a host arm.  After a 0x08 / 0x0D the next arm is abnormal (block 1 never stores; what runs
/// moves with layout, OPEN): reported as Parked, and the arm after it runs (315, 768, 769).
pub fn arm(gpa: std.mem.Allocator, state: *State, syllables: []const u32, kind: Arm, max_bundles: usize) RunError!usize {
    if (state.next_arm_parks) {
        state.next_arm_parks = false;
        return RunError.Parked;
    }
    const entry: usize = switch (kind) {
        .pass1 => 0,
        .decode => |q| q,
    };
    if (entry >= syllables.len) return RunError.UnknownBundleLength;
    return runFrom(gpa, state, syllables, entry, max_bundles);
}

/// runLinear from syllable `start`, with a bundle budget of `max_bundles`
/// (a codec program runs far past runLinear's 256).
pub fn runFrom(gpa: std.mem.Allocator, state: *State, syllables: []const u32, start: usize, max_bundles: usize) RunError!usize {
    var i: usize = start;
    var bundles_run: usize = 0;
    var shadow: ?u8 = null;
    // Inside x17.ms 010's shadow a Y store is lost or misaddressed (fact 465): refused.
    var wedge_shadow = false;
    // The gap since the latest halt-deadline armer (fact 98, 477, 484), null when none ran.
    var since_armer: ?DeadlineGap = null;
    const lane0: usize = state.cur_lane;
    state.halted = false;
    state.silent_stop = false;
    state.stop_now = false;
    state.stream_starved = false;
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
        if (wedge_shadow) {
            var before = state.y_mem.clone(gpa) catch return RunError.OutOfMemory;
            defer before.deinit(gpa);
            try executeBundle(gpa, state, slice);
            if (!sameY(&before, &state.y_mem)) return RunError.UnknownForm;
        } else executeBundle(gpa, state, slice) catch |e| {
            const hook = state.on_unknown orelse return e;
            if (e != RunError.UnknownForm) return e;
            hook.f(hook.ctx, i, slice);
        };
        if (state.stream_starved) {
            // 784: the read waits for the next packet and nothing in its bundle ran; the
            // caller queues bytes and resumes with runFrom at state.pc.  Inside a jump's delay
            // or a stop's shadow unmeasured.
            if (state.jump_delay > 0 or state.pending_jmp_offset != null or shadow != null) return RunError.UnknownForm;
            return bundles_run;
        }
        if (state.stop_now) {
            // 667: the store lands and the engine never reaches the next bundle; inside a
            // jump's delay or a stop's shadow unmeasured.
            state.stop_now = false;
            if (state.jump_delay > 0 or state.pending_jmp_offset != null or shadow != null) return RunError.UnknownForm;
            state.silent_stop = true;
            return bundles_run + 1;
        }
        updateE0(state, slice);
        if (shadow == null) {
            if (isHaltBundle(slice)) {
                if (since_armer) |g| if (!g.completes()) return RunError.HaltDeadline;
            } else if (deadlineLimit(slice)) |limit| {
                since_armer = .{ .limit = limit };
            } else if (since_armer) |*g| g.words += slice.len;
        }
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
            if (state.jump_delay > 0) {
                // A halt in a taken jump's delay slot (681): the target bundle runs once, then
                // the engine stops.  Measured with the halt in slot 1 (nothing after it) and
                // with nop + halt; a working bundle in slot 2 after a slot-1 halt is unmeasured.
                if (state.jump_delay == 2 and i < syllables.len and syllables[i] != 0x40000005) return RunError.UnknownForm;
                i = state.jump_target;
                state.jump_delay = 0;
                shadow = 0;
                continue;
            }
            shadow = 1;
            continue;
        }
        if (state.end_pending) {
            // 759: like a halt, two delay slots then the stop; inside a jump's delay unmeasured.
            state.end_pending = false;
            if (state.jump_delay > 0) return RunError.UnknownForm;
            state.halted = true;
            shadow = 1;
            continue;
        }
        if (state.wedge_pending) {
            // S1[22:20] = 010: two more bundles, then a stop no poll sees (fact 457, 465).
            state.wedge_pending = false;
            wedge_shadow = true;
            shadow = 1;
            continue;
        }
        if (state.stop_pending) {
            // x17e's stop: one more bundle runs, the next does not (fact 448).
            state.stop_pending = false;
            state.halted = true;
            shadow = 0;
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

/// e0 is no register (fact 65, 466, 468, 470): after ldi, lde e0, a 0x05 alu/movb (the
/// nop included), a pc14 pair loading an immediate and the st pair it is that word's [20:5];
/// an x19alu and an lde e<k != 0> leave it. No general rule is known: after anything else e0
/// is unknown, and a Y address read through it is refused.
fn updateE0(state: *State, b: []const u32) void {
    const h = b[0];
    const low5 = h & 0x1F;
    const name: []const u8 = if (isa.classify(h)) |f| f.name else "";
    const lde = b.len == 1 and ((low5 == 0x06 and h >> 30 == 3) or std.mem.eql(u8, name, "lde"));
    // lde e<k != 0> passes the latch through (470, measured with k = 7), as does lde.arm,
    // lde class 101 (480).
    if (lde and (h >> 26) & 0xF != 0) return;
    if (b.len == 1 and low5 == 0x06 and h >> 29 == 0b101) return;
    // pc14 (fact 107, 380): measured only when the pair loads an immediate, s<h[24:21]> != s0.
    const pc14 = low5 == 0x14 and b.len == 2 and (b[1] >> 29 == 2 or b[1] >> 29 == 3) and
        (b[1] >> 25) & 1 == 1 and (h >> 21) & 0xF != 0;
    const setter = pc14 or lde or (b.len == 1 and ((low5 == 0x05 and h >> 30 == 1) or
        std.mem.eql(u8, name, "ldi")));
    if (setter) {
        state.e[0] = (h >> 5) & 0xFFFF;
        state.e0_known = true;
        return;
    }
    if (low5 == 0x0F and b.len == 2 and h >> 29 == 1 and b[1] & 0xE0018000 == 0x20018000) {
        state.e0_known = true; // execStY wrote it (65)
        return;
    }
    if (low5 == 0x19 and b.len == 4 and b[1] >> 29 == 0 and b[2] >> 29 == 4 and b[3] >> 29 == 4) return;
    state.e0_known = false;
}

/// e<k> as a Y address: e0 only while it is known.
fn yAddrE(state: *const State, k: u32) RunError!u32 {
    if (k == 0 and !state.e0_known) return RunError.UnknownForm;
    return state.e[k];
}

fn sameY(a: *const std.AutoHashMapUnmanaged(u32, u32), b: *const std.AutoHashMapUnmanaged(u32, u32)) bool {
    if (a.count() != b.count()) return false;
    var it = a.iterator();
    while (it.next()) |kv| {
        if ((b.get(kv.key_ptr.*) orelse return false) != kv.value_ptr.*) return false;
    }
    return true;
}

/// Words between an armer and the halt: the halt completes while they stay within the armer's
/// limit (fact 484, 485: after an x18 armer 7 nops complete, and so do 5 nops and a
/// two-word st pair, while 6 and the pair park). The unit may be cycles (OPEN).
const DeadlineGap = struct {
    limit: usize,
    words: usize = 0,

    fn completes(g: DeadlineGap) bool {
        return g.words <= g.limit;
    }
};

/// Bundles that arm the halt deadline, with their limit: an x17 with S1[15] = 1 that does not
/// halt, S2 class 000, 101 or 111 (fact 98), limit 8 (the bare continuing x17 of 484;
/// 98's store-carrying x17 allowed 7, a form x17 does not separate yet); an x02 with w2[30] = 1 (477) and an
/// x18 class 000 with w2[18:17] = 10 (483), limit 7 (484); an x03 with w2[31:29] = 010, an lde
/// with h[30] = 0 and an rop with h[31] = 1 (479), limit 7 by analogy.
fn deadlineLimit(bundle: []const u32) ?usize {
    if (bundle[0] & 0x1F == 0x17) return if (isDeadlineArmer(bundle)) 8 else null;
    return if (isDeadlineArmer(bundle)) 7 else null;
}

fn isDeadlineArmer(bundle: []const u32) bool {
    const low5 = bundle[0] & 0x1F;
    // S2 classes 000, 101 and 111 are the measured ones; a class-011 store's S1[15] is its e
    // field's low bit and does not arm (479).
    if (low5 == 0x17 and bundle.len >= 3) {
        const cls = bundle[2] >> 29;
        return (cls == 0 or cls == 5 or cls == 7) and (bundle[1] >> 15) & 1 == 1;
    }
    if (low5 == 0x02 and bundle.len >= 6) return (bundle[2] >> 30) & 1 == 1;
    // 483: x18 class 000 with w2[18] = 1 arms, and w2[17] = 1 cancels it.
    if (low5 == 0x18 and bundle.len >= 4 and bundle[3] >> 29 == 0) return bundle[2] & 0x60000 == 0x40000;
    if (low5 == 0x03 and bundle.len >= 6) return bundle[2] >> 29 == 0b010; // 479
    // 479: lde with h[30] = 0 (lde.arm, A0000026) and rop with h[31] = 1 (882000E7, which x17
    // does not execute yet).
    if (bundle.len == 1 and low5 == 0x06 and (bundle[0] >> 30) & 1 == 0) return true;
    if (bundle.len == 1 and low5 == 0x07 and bundle[0] >> 31 == 1) return true;
    return false;
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
    const op: u32 = (head >> 5) & 0x3F;
    if (op == 0x1E) {
        // 632, 633: p <- (a & b) != 0, measured only with h[18:15] = 10 and c 14 (b = s10) or
        // 15 (b = s12); the b index rule is OPEN.
        const b_s: u32 = switch (cond) {
            14 => 10,
            15 => 12,
            else => return RunError.UnknownForm,
        };
        if ((head >> 15) & 0xF != 10) return RunError.UnknownForm;
        if (p_dest != 0) state.p[p_dest] = @intFromBool(a_val & state.s[b_s] & M24 != 0);
        return;
    }
    // 24-bit signed: sign bit is bit 23.
    const signed: i32 = if (a_val & 0x800000 != 0)
        @as(i32, @bitCast(a_val | 0xFF000000))
    else
        @as(i32, @bitCast(a_val));
    // fact 376/377: row c[4:2], b = s<4 * c[1:0]> (s0, s4, s8, s12).  Silicon
    // probe 2026-11-06: c1 is a <= -s4 (7 of 7 rows), not a < 0.  own-ptst-eq is c7 = row 001
    // (a >= b, b = s12); row 111 is b == 0 per the 377 sweep fit.
    const b_s: u32 = (cond & 3) * 4;
    const b_raw: u32 = state.s[b_s] & M24;
    const b: i32 = if (b_raw & 0x800000 != 0) @as(i32, @bitCast(b_raw | 0xFF000000)) else @as(i32, @bitCast(b_raw));
    const t: bool = switch (@as(u3, @intCast(cond >> 2))) {
        0 => signed <= -b,
        1 => signed >= b,
        2 => b <= 0,
        3 => b < 0,
        4 => b <= 1,
        5 => signed <= b,
        6 => b >= 0,
        7 => b == 0,
    };
    const v: u8 = @intFromBool(t);
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

/// dld, low5 0x09 with [31] = 0 (fact 524): s<[25:22]> <- X:[A<[19:15]>] when [20] = 1;
/// s<[29:26]> <- sx16 Y:[e<[8:5]>] when [13] = 1 (as ldy, 54).  Refused: [30] = 0 (wedges, 54,
/// 116), [20] = 0 beside an X destination (another word, OPEN), [13] = 0 beside a Y one (the
/// stale latch) and one s for both halves.
fn execDld(state: *State, head: u32) RunError!void {
    if ((head >> 30) & 1 == 0) return RunError.UnknownForm;
    const xd: u32 = (head >> 22) & 0xF;
    const yd: u32 = (head >> 26) & 0xF;
    if (xd != 0 and yd == xd) return RunError.UnknownForm;
    var xv: u32 = 0;
    if (xd != 0) {
        if ((head >> 20) & 1 == 0) return RunError.UnknownForm;
        xv = (state.readX(state.a[(head >> 15) & 0x1F]) orelse 0) & M24;
    }
    var yv: u32 = 0;
    if (yd != 0) {
        if ((head >> 13) & 1 == 0) return RunError.UnknownForm;
        yv = sx16To24((state.getY(try yAddrE(state, (head >> 5) & 0xF)) orelse 0) & 0xFFFF);
    }
    state.s[xd] = xv;
    state.s[yd] = yv;
}

fn execLdy(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    _ = gpa;
    const dest_k: u32 = (head >> 26) & 0x1F;
    const src_e: u32 = (head >> 5) & 0xF;
    // R<[25:22]> gets the same word (fact 523), measured with [30] = 1 only.
    const dest_r: u32 = (head >> 22) & 0xF;
    if (dest_k < 16) {
        if (dest_r != 0) return RunError.UnknownForm;
        return;
    }
    const dest_s: u32 = dest_k - 16;
    const addr: u32 = try yAddrE(state, src_e);
    const raw: u32 = (state.getY(addr) orelse 0) & 0xFFFF;
    // Sign-extends the 16-bit word (fact 409, own-ldysx).
    state.s[dest_s] = sx16To24(raw);
    if (dest_r != 0) state.r[dest_r] = raw;
}

fn execStx(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const a_idx: u32 = (head >> 11) & 0x1F;
    const src_s: u32 = (head >> 16) & 0xF;
    // Through A0 the store lands at the previous bundle's immediate or result, through A31
    // at X:0 (fact 649): neither is an address register here, so both are refused.
    if (a_idx == 0 or a_idx == 31) return RunError.UnknownForm;
    const addr: u32 = state.a[a_idx];
    try state.x_mem.put(gpa, addr, try stxValue(state, src_s));
}

/// The value an stx store writes from s<src>: s0 stores the xd latch << 8, not 0 (709:
/// 0x011100 after reading 0x111).  Measured with a small xd; a shift past 24 bits is refused.
fn stxValue(state: *const State, src: u32) RunError!u32 {
    if (src != 0) return state.s[src] & M24;
    if (state.xd > 0xFFFF) return RunError.UnknownForm;
    return (state.xd << 8) & M24;
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
        4 => shl7(d),
        5 => d,
        else => alu_val,
    };
    if (alu_dest != 0) state.s[alu_dest] = alu_val;
    if (hd != 0) state.s[hd] = hv;
}

/// low5 0x0F, S class 101 without S[22] & S[20] (fact 127): R<S[14:11]> <- #head[20:5];
/// a bundle that also writes s<head[24:21]> is refused (488).
fn execRimm(state: *State, bundle: []const u32) RunError!void {
    try checkRimm(bundle);
    rimmWrite(state, bundle);
}

/// 488: rimm's s<head[24:21]> write reads 0x0700 before any st.y and 0x5000 after (source
/// OPEN): refused, on rimm and on st.x alike (525).
fn checkRimm(bundle: []const u32) RunError!void {
    if ((bundle[0] >> 21) & 0xF != 0) return RunError.UnknownForm;
}

/// R<S[14:11]> <- #head[20:5] (127; st.x too, 525).
fn rimmWrite(state: *State, bundle: []const u32) void {
    const dest: u32 = (bundle[1] >> 11) & 0xF;
    if (dest != 0) state.r[dest] = (bundle[0] >> 5) & 0xFFFF;
}

/// low5 0x0F, S class 101 with S[22] and S[20] (fact 127):
/// X:[A<4*S[2:0] + head[31:30]>] <- sx24 s<S[28:25]>; and e<S[6:3]> <- #head[20:5] under any
/// head[31:30] (437, measured with S[22:20] = 111 only); and rimm's writes (525).
fn execStx0f(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    try checkRimm(bundle);
    const sw = bundle[1];
    const a_idx: u32 = 4 * (sw & 7) + (bundle[0] >> 30);
    // Through A0 the store lands at the previous word's h[20:5] (694; a nop before it gives 0):
    // not modelled, refused.
    if (a_idx == 0) return RunError.UnknownForm;
    const v: u32 = state.s[(sw >> 25) & 0xF] & M24;
    state.x_mem.put(gpa, state.a[a_idx], v) catch return RunError.OutOfMemory;
    const e_dest: u32 = (sw >> 3) & 0xF;
    if (e_dest != 0) state.e[e_dest] = (bundle[0] >> 5) & 0xFFFF;
    rimmWrite(state, bundle);
}

/// low5 0x08, [31:29] = 010, op[10:5] = 0x30 + k (fact 61, 101):
/// Y:[e<k>] <- s<[25:22]>.
fn execStk(gpa: std.mem.Allocator, state: *State, head: u32) RunError!void {
    const v: u32 = state.s[(head >> 22) & 0xF] & 0xFFFF;
    state.y_mem.put(gpa, try yAddrE(state, (head >> 5) & 0xF), v) catch return RunError.OutOfMemory;
}

/// psel, low5 0x0E with head[30:25] = 0x23 (fact 126, 152): s<S[11:8]> <- c ? r : b with
/// c = p<S[27:24]> (S[21] = 0 inverts) and r = X16_ALU(a = s<S[31:28]>, b = s<S[7:4]>); and
/// shape A's head write (531), s<head[24:21]> <- mux S[14:12]: 0 zero (152), 1/2/6
/// #head[20:5]; the other muxes are OPEN and refused.
fn execPsel(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const h = bundle[0];
    const s1 = bundle[1];
    const hv: u32 = switch ((s1 >> 12) & 7) {
        0 => 0,
        1, 2, 6 => (h >> 5) & 0xFFFF,
        else => return RunError.UnknownForm,
    };
    const raw: u8 = state.p[(s1 >> 24) & 0xF];
    const pv: u8 = if ((s1 >> 21) & 1 == 1) raw else @as(u8, 1) -% raw;
    const b_val: u32 = state.s[(s1 >> 4) & 0xF] & M24;
    const r = try x16AluOp(s1, state.s[(s1 >> 28) & 0xF] & M24, b_val);
    state.s[(s1 >> 8) & 0xF] = if (pv != 0) r & M24 else b_val;
    state.s[(h >> 21) & 0xF] = hv;
}

fn execLd0e(gpa: std.mem.Allocator, state: *State, bundle: []const u32) RunError!void {
    _ = gpa;
    if (bundle.len < 2) return RunError.UnknownForm;
    const head = bundle[0];
    const s1 = bundle[1];
    const dest_s: u32 = (head >> 21) & 0xF;
    const addr: u32 = (head >> 5) & 0xFFFF;
    const n: u32 = (s1 >> 1) & 0x7;
    // psel shape A's layout (607, replacing 333's s <- 0).  ALU word s<S[11:8]> <- a = s<S[7:4]>
    // op b = s<S[31:28]>, code head[30:25], m S[17:15]: measured codes 14 (m 0 / 2), 22, 51, 6
    // at m 0; 16-21 / 35 / 36 read a + b (569's override set) and the rest are OPEN.
    const alu_dest: u32 = (s1 >> 8) & 0xF;
    // 783: code 0x22 at m 2 over a = s12, b = s14 enables the stream when s12 = 1 and s14 has
    // bit 2 set and bits 1:0 clear; other values leave it off.  s14 with bits 2:1 = 11 starts
    // elsewhere, other operand registers and the ALU result are unmeasured: refused.
    if ((head >> 25) & 0x3F == 0x22 and (s1 >> 15) & 7 == 2) {
        if ((s1 >> 4) & 0xF != 12 or s1 >> 28 != 14 or alu_dest != 0) return RunError.UnknownForm;
        const a12: u32 = state.s[12] & M24;
        const b14: u32 = state.s[14] & M24;
        if (a12 == 1 and b14 & 4 != 0) switch (b14 & 3) {
            0 => state.stream_on = true,
            2 => return RunError.UnknownForm,
            else => {},
        };
    }
    const alu_val: u32 = if (alu_dest == 0) 0 else blk: {
        const code: u32 = (head >> 25) & 0x3F;
        const m: u32 = (s1 >> 15) & 7;
        const ok = (code == 14 and (m == 0 or m == 2)) or ((code == 22 or code == 51 or code == 6) and m == 0);
        if (!ok) return RunError.UnknownForm;
        const word: u32 = (code << 5) | (m << 23);
        break :blk (try aluWord(word, sx24(state.s[(s1 >> 4) & 0xF]), sx24(state.s[(s1 >> 28) & 0xF]))) orelse return RunError.UnknownForm;
    };
    // Head write s<head[24:21]> by mux S[14:12]: 1 / 6 #head[20:5]; 0 the cond on p<S[27:24]>
    // (p0 reads 1) by modifier S[23:21]: 0 !p, 1 p, 2 0, 3 1 (608; 4-7 OPEN); the rest OPEN.
    const hv: u32 = if (dest_s == 0) 0 else switch ((s1 >> 12) & 7) {
        1, 6 => addr,
        0 => blk: {
            const pi: u32 = (s1 >> 24) & 0xF;
            const pv: u32 = @intFromBool(state.p[pi] != 0);
            break :blk switch ((s1 >> 21) & 7) {
                0 => pv ^ 1,
                1 => pv,
                2 => 0,
                3 => 1,
                else => return RunError.UnknownForm,
            };
        },
        else => return RunError.UnknownForm,
    };
    if (alu_dest != 0) state.s[alu_dest] = alu_val;
    if (dest_s != 0) state.s[dest_s] = hv;
    // op[0] = s1[0] adds 2 (fact 287): A<4n + 1 + 2*op[0]>.
    const a_idx: u32 = (4 * n + 1 + 2 * (s1 & 1)) & 0x1F;
    state.a[a_idx] = addr;
}

/// 0x0B with head[31:30] = 10 (fact 258, 259, 324): v = Y:[e<[8:5]>]; s<[29:26]> ([29] = bit 3,
/// so classes 100 and 101 are one op), R<[25:22]> <- v, A<[18:15]> <- v when [14:13] = 01, else
/// yd.  [20] = 0 wedges (258); [19], [10:9] inert.  How the 16-bit value widens into a 24-bit A
/// is unmeasured: refused when its bit 15 is set.
fn execAset0b(state: *State, head: u32) RunError!void {
    if ((head >> 20) & 1 == 0) return RunError.UnknownForm;
    const v: u32 = (state.getY(try yAddrE(state, (head >> 5) & 0xF)) orelse 0) & 0xFFFF;
    const dest_s: u32 = (head >> 26) & 0xF;
    const dest_r: u32 = (head >> 22) & 0xF;
    const dest_a: u32 = (head >> 15) & 0xF;
    const a_val: u32 = if ((head >> 13) & 3 == 1) v else state.yd & 0xFFFF;
    if (dest_a != 0 and a_val & 0x8000 != 0) return RunError.UnknownForm;
    if (dest_s != 0) state.s[dest_s] = v;
    if (dest_r != 0) state.r[dest_r] = v;
    if (dest_a != 0) state.a[dest_a] = a_val;
}

/// 0x0C with [31] = 1 (fact 282, 378; measured 2026-11-06): one form.
/// R<[30:27]> <- agu_xr(fn = [11:7], x = A<[16:12]>, r = R<[20:17]>), and with
/// R dest 0 the pcond rows: p<[24:21]> <- c22-27 R >0 >=0 <=0 <0 !=0 ==0,
/// c28-31 R<A R>=A R!=A R==A (signed).  [5] = 1 required (without it, wedge
/// per 306).  OPEN: p with R dest != 0, p0 with slot 0, pcond c0-21.
fn execRagu0c(state: *State, head: u32) RunError!void {
    if ((head >> 5) & 1 == 0) return RunError.UnknownForm;
    const dest: u32 = (head >> 27) & 0xF;
    const slot: u32 = (head >> 21) & 0xF;
    const fn_op: u32 = (head >> 7) & 0x1F;
    const x: u32 = state.a[(head >> 12) & 0x1F];
    const r: u32 = state.r[(head >> 17) & 0xF];
    if (dest != 0) {
        if (slot != 0) return RunError.UnknownForm;
        state.r[dest] = aguXrEval(fn_op, x, r);
        return;
    }
    if (slot == 0) return;
    const rs: i64 = sx16(r);
    const as: i64 = sx16(x);
    const t: bool = switch (fn_op) {
        22 => rs > 0,
        23 => rs >= 0,
        24 => rs <= 0,
        25 => rs < 0,
        26 => rs != 0,
        27 => rs == 0,
        28 => rs < as,
        29 => rs >= as,
        30 => rs != as,
        31 => rs == as,
        else => return RunError.UnknownForm,
    };
    state.p[slot] = @intFromBool(t);
}

/// agu_xr (fact 338, 385): the x19mac A slot and the 0x0C R form.
/// agu_xr's once-only wrap (fact 566-568): with M = R<k>, a sum >= M becomes sum - M and
/// a negative one (16-bit signed) sum + M.  k = 0 (no wrap) needs no case: R0 reads 0.
fn wrapOnce(state: *const State, sum: u32, k: u32) u32 {
    const m: u32 = state.r[k];
    if (sum & 0x8000 != 0) return (sum +% m) & 0xFFFF;
    return if (sum >= m) (sum -% m) & 0xFFFF else sum;
}

fn aguXrEval(fn_op: u32, x: u32, r: u32) u32 {
    const v: u32 = switch (fn_op & 0x1F) {
        16, 17, 21 => x,
        18 => r +% 1,
        19 => r -% 1,
        20 => 0 -% r,
        22, 23, 24, 25, 26, 27 => r,
        28, 29, 30, 31 => x -% r,
        else => switch ((fn_op & 0xF) % 6) {
            0 => x +% 1,
            1 => x -% 1,
            2 => x +% r,
            3 => x -% r,
            4 => 0 -% x,
            else => x,
        },
    };
    return v & 0xFFFF;
}

/// 0x0F with a class-001 second (fact 279, 552): e<{w1[0], head[31:29]}> <- #head[20:5];
/// with that index 0, s<head[24:21]> <- e<w1[20:17]>, + 1 when w1[2].  The bundle's Y store is
/// not modelled.
fn execSte(state: *State, bundle: []const u32) RunError!void {
    const w1 = bundle[1];
    const dest: u32 = ((w1 & 1) << 3) | (bundle[0] >> 29);
    if (dest != 0) {
        state.e[dest] = (bundle[0] >> 5) & 0xFFFF;
        return;
    }
    state.s[(bundle[0] >> 21) & 0xF] = (state.e[(w1 >> 17) & 0xF] +% ((w1 >> 2) & 1)) & 0xFFFF;
}

/// 0x0F with a class-011 second (fact 757): e<{w1[0], head[31:29]}> <- #head[20:5]; when
/// head[24:21] != 0, s<head[24:21]> <- 326's mux on w1[2:1] (0, R<head[28:25]>, e<w1[8:5]>, 0).
/// Measured with w1[16:15] = 00 only (no store); the store forms and index 0 are refused.
fn execSte011(state: *State, bundle: []const u32) RunError!void {
    const h = bundle[0];
    const w1 = bundle[1];
    if ((w1 >> 15) & 3 != 0) return RunError.UnknownForm;
    const dest: u32 = ((w1 & 1) << 3) | (h >> 29);
    if (dest == 0) return RunError.UnknownForm;
    const s_dest: u32 = (h >> 21) & 0xF;
    const s_val: u32 = switch ((w1 >> 1) & 3) {
        1 => state.r[(h >> 25) & 0xF],
        2 => state.e[(w1 >> 5) & 0xF],
        else => 0,
    };
    state.e[dest] = (h >> 5) & 0xFFFF;
    if (s_dest != 0) state.s[s_dest] = s_val;
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

    // head[21] = 1 computes through AGU10 (fact 521, own-ropagu, 162 silicon rows), 0
    // through ROP07 = AGU10B:  x, y, 0, -x, -y, 1, x+y, x-y, 0, 0, y, y, y, -y, y+1, y-1
    const v: u32 = if ((head >> 21) & 1 == 1) try agu10Eval(fn_op, x_val, y_val) else switch (fn_op & 0xF) {
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
    // head[29:27] is a source mux (fact 501): 0 and 7 load 0, 1/2/6 the immediate,
    // 3/4/5 lanes of D (source OPEN, refused).
    const k: u32 = (head >> 27) & 7;
    if (k >= 3 and k <= 5) return RunError.UnknownForm;
    // The immediate is sign-extended to 24 bits (own-x02u24's operands, built with ldi:
    // 0xC321 lands as 0xFFC321).
    state.s[s_idx] = if (k == 0 or k == 7) 0 else sx16To24(imm);
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

test "ldi to s0 (dest k=16) is discarded: s0 always reads 0 (fact 438)" {
    var state = State{};
    defer state.deinit(testing.allocator);

    const b = try asm_mod.assembleOne("ldi s0, #0xDEAD");
    try executeBundle(testing.allocator, &state, b.slice());
    try testing.expectEqual(@as(u32, 0), state.s[0]);
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
    // Silicon stored 0x1238 at Y:0x700 (own-x17alu-shl).
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
    // code 0x20 is clr; m=1 -> OPEN per the Python model.
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
    // own-st3: three stores at r5l1..l3 of OUR syllables;
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
    // 515 (own-x19shrfar and its sweep): counts past 23 shift fully out, no wrap mod 32 or 64
    for ([_]i64{ 0x7FFFFF, 0x400000, -0x400000, -1, -0x800000 }) |a| {
        for ([_]i64{ -24, -25, -31, -32, -40, -64 }) |b| try testing.expectEqual(@as(i64, 0), shift23(a, b));
        for ([_]i64{ 24, 40 }) |b| try testing.expectEqual(@as(i64, if (a > 0) 0x7FFFFF else -0x800000), shift23(a, b));
    }
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
        \\        ldi s4, #2
        \\        ldi s2, #1
        \\loop:   alu s4, s4, s2, #0x10
        \\        ptst p1, s0, #13, #0x0B
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

test "every isa.toml form has a handler or a bit-pattern row" {
    for (isa.forms) |f| {
        const listed = for (pattern_forms) |n| {
            if (std.mem.eql(u8, n, f.name)) break true;
        } else false;
        if (named_forms.get(f.name) == null and !listed) {
            std.debug.print("form {s} has no handler\n", .{f.name});
            return error.TestUnexpectedResult;
        }
    }
}

test "roundShr rounds half up, negatives included" {
    try testing.expectEqual(@as(i64, 5), roundShr(5, 0));
    try testing.expectEqual(@as(i64, 3), roundShr(5, 1));
    try testing.expectEqual(@as(i64, -1), roundShr(-3, 1));
    try testing.expectEqual(@as(i64, 0), roundShr(-1, 1));
    try testing.expectEqual(@as(i64, -2), roundShr(-5, 1));
    try testing.expectEqual(@as(i64, 0x30), roundShr(0x4000 * 0x6000, 23));
}

test "aluOpen matches op, m and #s together" {
    const add_m5_12: u32 = (5 << 23) | (12 << 19) | (0x01 << 5);
    try testing.expect(aluOpen(&xld_alu_open, add_m5_12));
    try testing.expect(aluOpen(&xld_alu_open, (add_m5_12 & ~@as(u32, 0x3F << 5)) | (0x0e << 5))); // another add code
    try testing.expect(!aluOpen(&xld_alu_open, add_m5_12 ^ (1 << 19)));
    try testing.expect(!aluOpen(&xld_alu_open, add_m5_12 ^ (1 << 23)));
    try testing.expect(!aluOpen(&xld_alu_open, (add_m5_12 & ~@as(u32, 0x3F << 5)) | (0x10 << 5))); // sub
}

test "stream read (783): 0 before the enable, then MSB-first bits across bytes" {
    const gpa = std.testing.allocator;
    var state = State{};
    defer state.deinit(gpa);
    try state.queueStream(gpa, &.{ 0xA5, 0x5A });
    const read3_a15 = [_]u32{ 0xC0000016, 0x00040007, 0xE0000020 };
    const read8_a11 = [_]u32{ 0xC0000016, 0x00040005, 0xE0000070 };
    state.a[15] = 0x77;
    try executeBundle(gpa, &state, &read3_a15);
    try testing.expectEqual(@as(u32, 0), state.a[15]);
    try testing.expectEqual(@as(usize, 0), state.stream_bit);
    // ld0e.sen: s12 = 1, s14 = 4; its A13 load runs as any ld0e's.
    state.s[12] = 1;
    state.s[14] = 4;
    try executeBundle(gpa, &state, &.{ 0xC400000E | (0x123 << 5), 0xE02500C0 | (3 << 1) });
    try testing.expect(state.stream_on);
    try testing.expectEqual(@as(u32, 0x123), state.a[13]);
    try executeBundle(gpa, &state, &read3_a15);
    try testing.expectEqual(@as(u32, 0b101), state.a[15]); // 0xA5 = 101 00101
    try executeBundle(gpa, &state, &read8_a11);
    try testing.expectEqual(@as(u32, 0b00101_010), state.a[11]); // 00101 then 0x5A's 010
    // head[30:25] = 0x21 writes 0 and reads nothing.
    try executeBundle(gpa, &state, &.{ 0xC2000016, 0x00040007, 0xE0000020 });
    try testing.expectEqual(@as(u32, 0), state.a[15]);
    try testing.expectEqual(@as(usize, 11), state.stream_bit);
}

test "stream enable (783): other values leave it off; the unmeasured ones refuse" {
    const gpa = std.testing.allocator;
    const sen = [_]u32{ 0xC400000E, 0xE02500C0 };
    for ([_][2]u32{ .{ 1, 5 }, .{ 1, 8 }, .{ 2, 4 }, .{ 0, 12 } }) |v| {
        var state = State{};
        state.s[12] = v[0];
        state.s[14] = v[1];
        try executeBundle(gpa, &state, &sen);
        try testing.expect(!state.stream_on);
    }
    var state = State{};
    state.s[12] = 1;
    state.s[14] = 6; // starts elsewhere
    try testing.expectError(RunError.UnknownForm, executeBundle(gpa, &state, &sen));
    state.s[14] = 4;
    try testing.expectError(RunError.UnknownForm, executeBundle(gpa, &state, &.{ sen[0], sen[1] | (1 << 8) })); // an ALU dest
    try testing.expectError(RunError.UnknownForm, executeBundle(gpa, &state, &.{ 0xC0000016, 0x00040008, 0xE0000020 })); // A17
    try testing.expectError(RunError.UnknownForm, executeBundle(gpa, &state, &.{ 0xC0200016, 0x00040007, 0xE0000020 })); // an s write
}

test "stream underrun (784): the run stops at the read and resumes when bytes arrive" {
    const gpa = std.testing.allocator;
    const prog = switch (try asm_mod.assembleFile(gpa,
        \\        ldi s12, #1
        \\        ldi s14, #4
        \\        ld0e.sen #0, #0
        \\        x16.getb #7, #12
        \\        x16.getb #6, #2
        \\        x17.halt
        \\
    )) {
        .ok => |p| p,
        .err => return error.Assemble,
    };
    defer gpa.free(prog);
    var state = State{};
    defer state.deinit(gpa);
    try state.queueStream(gpa, &.{0xFF});
    _ = try runFrom(gpa, &state, prog, 0, 100);
    try testing.expect(state.stream_starved);
    try testing.expect(!state.halted);
    const at = state.pc;
    try state.queueStream(gpa, &.{ 0x80, 0x00 }); // the next packet: one stream with the first
    _ = try runFrom(gpa, &state, prog, at, 100);
    try testing.expect(!state.stream_starved);
    try testing.expect(state.halted);
    try testing.expectEqual(@as(u32, 0x1FF0), state.a[15]); // 11111111 10000
    try testing.expectEqual(@as(u32, 0), state.a[13]);
    try testing.expectEqual(@as(usize, 16), state.stream_bit);
}
