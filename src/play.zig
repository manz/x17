//! `x17 play`: run an x17 image the way the host does on silicon.  Each init pass is loaded and
//! armed at q0 in load order; then the decoder pass is loaded and every packet goes through
//! host.zig with the decode entry at q256 (fact 770).  P memory persists across passes, as on
//! silicon: a pass overwrites only its own rows.  An unmodelled form stops the run with its pc
//! and syllables (`Trap`) instead of guessing.
//!
//! Packet file: a u32 little-endian length, then that many bytes, repeated to the end.

const std = @import("std");
const emu = @import("emu");
const host_mod = @import("host");
const image = @import("image");
const asm_mod = @import("asm");
const disasm = @import("disasm.zig");

/// Decode-arm entry (fact 770).
pub const decode_entry: usize = 256;

pub const Options = struct {
    rate_hz: u32 = 44100,
    /// Y words written once after the image is loaded, before the first packet (the per-track
    /// configure words a host writes).
    y_writes: []const host_mod.Param = &.{},
    max_bundles_init: usize = 1 << 20,
    max_bundles_packet: usize = 1 << 26,
    /// Skip refused bundles as nops and count them here instead of trapping.
    trap_list: ?*TrapList = null,
    /// Print the first bundles run, with what each changed and read.
    tracer: ?*Tracer = null,
    /// Stop before the first decoder bundle at this q (`error.Breakpoint`).
    stop_at: ?usize = null,
    /// Stop once this many bundles have run, init passes included.
    stop_after: ?u64 = null,
    /// When the run stops short (a stop or a trap), the state goes here as JSON.
    dump: ?*std.Io.Writer = null,
};

/// The state as one JSON object: where the run stopped, the register files, and every X and Y
/// word written so far, by address.
pub fn writeDump(gpa: std.mem.Allocator, w: *std.Io.Writer, st: *const emu.State, t: Trap) !void {
    try w.print("{{\"stop\":\"{s}\",\"pass\":{d},\"packet\":", .{ @errorName(t.err), t.pass });
    if (t.packet) |i| try w.print("{d}", .{i}) else try w.writeAll("null");
    try w.print(",\"q\":{d},\"bundles\":{d}", .{ t.pc, st.bundles_total });
    try jsonArray(w, "s", &st.s);
    try jsonArray(w, "e", &st.e);
    try jsonArray(w, "R", &st.r);
    try jsonArray(w, "A", &st.a);
    try jsonArray(w, "p", &st.p);
    try jsonMem(gpa, w, "x", &st.x_mem);
    try jsonMem(gpa, w, "y", &st.y_mem);
    try w.writeAll("}\n");
}

fn jsonArray(w: *std.Io.Writer, name: []const u8, vals: anytype) !void {
    try w.print(",\"{s}\":[", .{name});
    for (vals, 0..) |v, k| try w.print("{s}{d}", .{ if (k == 0) "" else ",", v });
    try w.writeAll("]");
}

fn jsonMem(gpa: std.mem.Allocator, w: *std.Io.Writer, name: []const u8, mem: *const std.AutoHashMapUnmanaged(u32, u32)) !void {
    var addrs: std.ArrayList(u32) = .empty;
    defer addrs.deinit(gpa);
    var it = mem.keyIterator();
    while (it.next()) |k| try addrs.append(gpa, k.*);
    std.mem.sort(u32, addrs.items, {}, std.sort.asc(u32));
    try w.print(",\"{s}\":{{", .{name});
    for (addrs.items, 0..) |a, k| try w.print("{s}\"{d}\":{d}", .{ if (k == 0) "" else ",", a, mem.get(a).? });
    try w.writeAll("}");
}

/// One line per bundle run for the first `left` bundles: where it ran, its syllables and
/// disasm, then the registers and predicates it changed and its X / Y reads and writes.
pub const Tracer = struct {
    const Regs = struct { s: [16]u32, e: [16]u32, r: [16]u32, a: [32]u32, p: [16]u8, v: [4]u32 };
    const Mem = std.AutoHashMapUnmanaged(u32, u32);
    const Write = struct { addr: u32, old: ?u32, new: u32 };

    gpa: std.mem.Allocator,
    out: *std.Io.Writer,
    left: usize,
    /// Set by play: the packet being fed, null in an init pass.
    packet: ?usize = null,
    pass: u8 = 0,
    state: *emu.State = undefined,
    log: emu.ReadLog = .{},
    regs: Regs = undefined,
    y: Mem = .{},
    x: Mem = .{},
    /// A write or allocation failed; the trace is cut short.
    failed: bool = false,

    pub fn deinit(self: *Tracer) void {
        self.y.deinit(self.gpa);
        self.x.deinit(self.gpa);
    }

    fn attach(self: *Tracer, state: *emu.State) void {
        self.state = state;
        state.on_bundle = .{ .ctx = self, .f = hook };
    }

    fn regsOf(st: *const emu.State) Regs {
        return .{ .s = st.s, .e = st.e, .r = st.r, .a = st.a, .p = st.p, .v = st.v };
    }

    fn hook(ctx: *anyopaque, phase: emu.BundlePhase, pc: usize, bundle: []const u32) void {
        const self: *Tracer = @ptrCast(@alignCast(ctx));
        if (self.left == 0 or self.failed) {
            self.state.read_log = null;
            return;
        }
        switch (phase) {
            .before => self.before() catch {
                self.failed = true;
            },
            .after => {
                self.state.read_log = null;
                self.after(pc, bundle) catch {
                    self.failed = true;
                };
                self.left -= 1;
            },
        }
    }

    fn before(self: *Tracer) !void {
        self.regs = regsOf(self.state);
        self.y.deinit(self.gpa);
        self.x.deinit(self.gpa);
        self.y = .{};
        self.x = .{};
        self.y = try self.state.y_mem.clone(self.gpa);
        self.x = try self.state.x_mem.clone(self.gpa);
        self.log = .{};
        self.state.read_log = &self.log;
    }

    fn after(self: *Tracer, pc: usize, bundle: []const u32) !void {
        const w = self.out;
        if (self.packet) |i| try w.print("pkt {d}", .{i}) else try w.print("pass {d}", .{self.pass});
        try w.print(" q 0x{x}:", .{pc});
        for (bundle) |x| try w.print(" {x:0>8}", .{x});
        var buf: [256]u8 = undefined;
        const n = disasm.disasmBundle(&buf, bundle) catch 0;
        try w.print("  {s}\n", .{if (n == 0) "?" else buf[0..n]});
        var detail: std.Io.Writer.Allocating = .init(self.gpa);
        defer detail.deinit();
        try self.effects(&detail.writer);
        if (detail.written().len > 0) try w.print("   {s}\n", .{detail.written()});
    }

    fn effects(self: *Tracer, w: *std.Io.Writer) !void {
        try self.regDiffs(w);
        for (self.log.reads()) |r| {
            try w.print(" r{c}[0x{x}]=", .{ @as(u8, if (r.bank == .x) 'X' else 'Y'), r.addr });
            if (r.value) |v| try w.print("0x{x}", .{v}) else try w.writeAll("unset");
        }
        if (self.log.n > self.log.items.len) try w.print(" (+{d} reads)", .{self.log.n - self.log.items.len});
        try self.memDiffs(w, 'X', &self.x, &self.state.x_mem);
        try self.memDiffs(w, 'Y', &self.y, &self.state.y_mem);
        if (self.state.stream_starved) try w.writeAll(" (waits for the stream; did not run)");
    }

    fn regDiffs(self: *Tracer, w: *std.Io.Writer) !void {
        const now = regsOf(self.state);
        try diffFile(w, "s", &self.regs.s, &now.s);
        try diffFile(w, "e", &self.regs.e, &now.e);
        try diffFile(w, "R", &self.regs.r, &now.r);
        try diffFile(w, "A", &self.regs.a, &now.a);
        try diffFile(w, "p", &self.regs.p, &now.p);
        try diffFile(w, "V", &self.regs.v, &now.v);
    }

    fn diffFile(w: *std.Io.Writer, name: []const u8, old: anytype, new: anytype) !void {
        for (old, new, 0..) |o, n, k| if (o != n) try w.print(" {s}{d} 0x{x}->0x{x}", .{ name, k, o, n });
    }

    fn memDiffs(self: *Tracer, w: *std.Io.Writer, bank: u8, old: *const Mem, new: *const Mem) !void {
        var list: std.ArrayList(Write) = .empty;
        defer list.deinit(self.gpa);
        var it = new.iterator();
        while (it.next()) |kv| {
            const o = old.get(kv.key_ptr.*);
            if (o == null or o.? != kv.value_ptr.*) try list.append(self.gpa, .{ .addr = kv.key_ptr.*, .old = o, .new = kv.value_ptr.* });
        }
        std.mem.sort(Write, list.items, {}, struct {
            fn lt(_: void, a: Write, b: Write) bool {
                return a.addr < b.addr;
            }
        }.lt);
        for (list.items) |x| {
            try w.print(" w{c}[0x{x}]=0x{x}", .{ bank, x.addr, x.new });
            if (x.old) |o| try w.print(" (was 0x{x})", .{o});
        }
    }
};

/// Refused bundles by form shape: head low5, length and each syllable's class bits [31:29].
/// After a skip the state is a guess, so later entries are hints, not facts.
pub const TrapList = struct {
    pub const Entry = struct {
        hits: u64 = 0,
        first_pc: usize,
        syllables: [8]u32 = @splat(0),
        n: usize = 0,
    };
    gpa: std.mem.Allocator,
    map: std.AutoArrayHashMapUnmanaged(u32, Entry) = .empty,
    oom: bool = false,

    pub fn deinit(self: *TrapList) void {
        self.map.deinit(self.gpa);
    }

    pub fn key(bundle: []const u32) u32 {
        var k: u32 = (bundle[0] & 0x1F) | @as(u32, @intCast(@min(bundle.len, 15))) << 5;
        for (bundle[1..@min(bundle.len, 8)], 0..) |w, i| k |= (w >> 29) << @intCast(9 + 3 * i);
        return k;
    }

    fn hook(ctx: *anyopaque, pc: usize, bundle: []const u32) void {
        const self: *TrapList = @ptrCast(@alignCast(ctx));
        const gop = self.map.getOrPut(self.gpa, key(bundle)) catch {
            self.oom = true;
            return;
        };
        if (!gop.found_existing) {
            gop.value_ptr.* = .{ .first_pc = pc };
            const n = @min(bundle.len, 8);
            @memcpy(gop.value_ptr.syllables[0..n], bundle[0..n]);
            gop.value_ptr.n = n;
        }
        gop.value_ptr.hits += 1;
    }

    /// Entries, most hits first (caller frees).
    pub fn ranked(self: *const TrapList, gpa: std.mem.Allocator) ![]Entry {
        const out = try gpa.dupe(Entry, self.map.values());
        std.mem.sort(Entry, out, {}, struct {
            fn more(_: void, a: Entry, b: Entry) bool {
                return a.hits > b.hits or (a.hits == b.hits and a.first_pc < b.first_pc);
            }
        }.more);
        return out;
    }
};

/// Where and why a run stopped short.
pub const Trap = struct {
    err: anyerror,
    /// null: an init pass; otherwise the packet index.
    packet: ?usize,
    /// The init pass number, or image.decoder_pass.
    pass: u8,
    pc: usize,
    /// The bundle at pc (fewer when its length is unknown).
    syllables: [8]u32 = @splat(0),
    n_syllables: usize = 0,
};

pub const Result = struct {
    pcm: []i16,
    packets: usize,
    bundles: u64,
    /// Packets whose run waited on the stream (784) and resumed with the next one.
    waits: usize,
    trap: ?Trap,

    pub fn deinit(self: *Result, gpa: std.mem.Allocator) void {
        gpa.free(self.pcm);
    }
};

pub const PacketError = error{BadPacketFile};

/// Split a packet file into slices of `bytes`.
pub fn packets(gpa: std.mem.Allocator, bytes: []const u8) (PacketError || error{OutOfMemory})![][]const u8 {
    var list: std.ArrayList([]const u8) = .empty;
    errdefer list.deinit(gpa);
    var at: usize = 0;
    while (at < bytes.len) {
        if (at + 4 > bytes.len) return error.BadPacketFile;
        const n = std.mem.readInt(u32, bytes[at..][0..4], .little);
        at += 4;
        if (n > bytes.len - at) return error.BadPacketFile;
        try list.append(gpa, bytes[at..][0..n]);
        at += n;
    }
    return list.toOwnedSlice(gpa);
}

fn trapAt(err: anyerror, packet: ?usize, pass: u8, prog: []const u32, pc: usize) Trap {
    var t: Trap = .{ .err = err, .packet = packet, .pass = pass, .pc = pc };
    if (pc < prog.len) {
        const len = @min(emu.bundleLen(prog[pc]) orelse 1, t.syllables.len, prog.len - pc);
        @memcpy(t.syllables[0..len], prog[pc..][0..len]);
        t.n_syllables = len;
    }
    return t;
}

fn loadData(gpa: std.mem.Allocator, state: *emu.State, prog: []u32, img: *const image.Image, pass: u8) !void {
    for (img.sections) |sec| if (sec.pass == pass) switch (sec.kind) {
        .p => image.placeRows(prog, sec),
        .x => for (sec.words, 0..) |w, k| try state.x_mem.put(gpa, sec.addr + @as(u32, @intCast(k)), w & 0xFFFFFF),
        .y, .param => for (sec.words, 0..) |w, k| try state.y_mem.put(gpa, sec.addr + @as(u32, @intCast(k)), w & 0xFFFF),
    };
}

/// Run `img` over `pkts`.  The PCM the host drained comes back even when a trap ends the run.
pub fn play(gpa: std.mem.Allocator, img: *const image.Image, pkts: []const []const u8, opts: Options) !Result {
    const prog = try gpa.alloc(u32, asm_mod.layout.lanes * asm_mod.layout.p_rows);
    defer gpa.free(prog);
    @memset(prog, 0);

    var h = try host_mod.Host.init(gpa, .{
        .entry = decode_entry,
        .rate_hz = opts.rate_hz,
        .max_bundles = opts.max_bundles_packet,
    });
    defer h.deinit();
    if (opts.trap_list) |tl| h.state.on_unknown = .{ .ctx = tl, .f = TrapList.hook };
    if (opts.tracer) |t| t.attach(&h.state);
    h.state.break_after = opts.stop_after;

    var res: Result = .{ .pcm = &.{}, .packets = 0, .bundles = 0, .waits = 0, .trap = null };
    defer if (opts.dump) |w| if (res.trap) |t| writeDump(gpa, w, &h.state, t) catch {};

    // Init passes in load order: the sections come pass by pass.
    var last: ?u8 = null;
    for (img.sections) |sec| {
        if (sec.pass == image.decoder_pass or sec.pass == last) continue;
        last = sec.pass;
        if (opts.tracer) |t| t.pass = sec.pass;
        try loadData(gpa, &h.state, prog, img, sec.pass);
        h.state.cur_lane = 0;
        const n = emu.arm(gpa, &h.state, prog, .pass1, opts.max_bundles_init) catch |e| {
            res.trap = trapAt(e, null, sec.pass, prog, h.state.pc);
            res.pcm = try h.pcm.toOwnedSlice(gpa);
            return res;
        };
        res.bundles += n;
        if (!h.state.halted) {
            res.trap = trapAt(error.NoStop, null, sec.pass, prog, h.state.pc);
            res.pcm = try h.pcm.toOwnedSlice(gpa);
            return res;
        }
    }

    try loadData(gpa, &h.state, prog, img, image.decoder_pass);
    h.state.break_at = opts.stop_at;
    for (opts.y_writes) |w| try h.state.y_mem.put(gpa, w.addr, w.value & 0xFFFF);

    for (pkts, 0..) |pkt, i| {
        if (opts.tracer) |t| t.packet = i;
        const r = h.feed(prog, pkt) catch |e| {
            res.trap = trapAt(e, i, image.decoder_pass, prog, h.state.pc);
            break;
        };
        res.packets += 1;
        res.bundles += r.bundles;
        if (r.waiting) res.waits += 1;
    }
    res.pcm = try h.pcm.toOwnedSlice(gpa);
    return res;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

fn imageOf(gpa: std.mem.Allocator, src: []const u8) !image.Image {
    var prog = switch (try asm_mod.assembleProgram(gpa, src, .{})) {
        .ok => |p| p,
        .err => return error.TestUnexpectedResult,
    };
    defer prog.deinit();
    const bytes = try image.write(gpa, &prog);
    defer gpa.free(bytes);
    return image.parse(gpa, bytes);
}

test "packets splits length-prefixed records and refuses a short one" {
    const gpa = testing.allocator;
    const bytes = [_]u8{ 2, 0, 0, 0, 0xAA, 0xBB, 0, 0, 0, 0, 1, 0, 0, 0, 0xCC };
    const p = try packets(gpa, &bytes);
    defer gpa.free(p);
    try testing.expectEqual(@as(usize, 3), p.len);
    try testing.expectEqualSlices(u8, &.{ 0xAA, 0xBB }, p[0]);
    try testing.expectEqual(@as(usize, 0), p[1].len);
    try testing.expectEqualSlices(u8, &.{0xCC}, p[2]);
    try testing.expectError(error.BadPacketFile, packets(gpa, &.{ 5, 0, 0, 0, 1 }));
}

test "play runs the init pass, then hands each packet's ring slot to the host" {
    const gpa = testing.allocator;
    // Init writes a one-slot ring header; the decoder fills one sample per packet into the
    // slot, marks it owned with length 1, raises the status bit and stops.
    var img = try imageOf(gpa,
        \\.pass init
        \\        ldi s3, #1
        \\        lde e6, #1
        \\        x17.sty e6, #3, #0
        \\        ldi s3, #0x800
        \\        lde e6, #2
        \\        x17.sty e6, #3, #0
        \\        x17.halt
        \\.pass decoder
        \\        ldi s3, #0x1234
        \\        lde e6, #0x80
        \\        x17.sty e6, #3, #0
        \\        ldi s3, #1
        \\        lde e6, #5
        \\        x17.sty e6, #3, #0
        \\        lde e6, #6
        \\        x17.sty e6, #3, #0
        \\        lde e6, #0xFD94
        \\        x17.sty e6, #3, #0
        \\        x17.halt
    );
    defer img.deinit(gpa);
    var r = try play(gpa, &img, &.{ "a", "b" }, .{});
    defer r.deinit(gpa);
    try testing.expectEqual(@as(?Trap, null), r.trap);
    try testing.expectEqual(@as(usize, 2), r.packets);
    // Both runs write the one slot; the host released it after the first.
    try testing.expectEqualSlices(i16, &.{ 0x1234, 0x1234 }, r.pcm);
}

test "play traps an unmodelled form with its pass, pc and syllables" {
    const gpa = testing.allocator;
    var img = try imageOf(gpa,
        \\.pass init
        \\        x17.halt
        \\.pass decoder
        \\        ldi s3, #1
        \\        x19.ld s3, #4, e8, #2
        \\        x17.halt
    );
    defer img.deinit(gpa);
    // Set w1[20], which x19.ld refuses (793): the bundle sits at q257, its w1 at q258 = row 64
    // lane 1, the section's word 1.
    for (img.sections) |*sec| if (sec.kind == .p and sec.pass == image.decoder_pass) {
        @constCast(sec.words)[1] |= 1 << 20;
    };
    var r = try play(gpa, &img, &.{"x"}, .{});
    defer r.deinit(gpa);
    const t = r.trap orelse return error.TestUnexpectedResult;
    try testing.expectEqual(@as(anyerror, error.UnknownForm), t.err);
    try testing.expectEqual(@as(?usize, 0), t.packet);
    try testing.expectEqual(decode_entry + 1, t.pc);
    try testing.expectEqual(@as(usize, 4), t.n_syllables);
    try testing.expectEqual(@as(u32, 0x00000019), t.syllables[0]);
}

test "play --trap-list skips refused bundles and counts them by shape" {
    const gpa = testing.allocator;
    var img = try imageOf(gpa,
        \\.pass init
        \\        x17.halt
        \\.pass decoder
        \\        x19.ld s3, #4, e8, #2
        \\        ldi s3, #1
        \\        x17.halt
    );
    defer img.deinit(gpa);
    for (img.sections) |*sec| if (sec.kind == .p and sec.pass == image.decoder_pass) {
        @constCast(sec.words)[4] |= 1 << 20; // the x19.ld sits at q256; its w1 at q257 = row 65 lane 0
    };
    var tl: TrapList = .{ .gpa = gpa };
    defer tl.deinit();
    var r = try play(gpa, &img, &.{ "a", "b", "c" }, .{ .trap_list = &tl });
    defer r.deinit(gpa);
    try testing.expectEqual(@as(?Trap, null), r.trap);
    try testing.expectEqual(@as(usize, 3), r.packets);
    const ranked = try tl.ranked(gpa);
    defer gpa.free(ranked);
    try testing.expectEqual(@as(usize, 1), ranked.len);
    try testing.expectEqual(@as(u64, 3), ranked[0].hits);
    try testing.expectEqual(decode_entry, ranked[0].first_pc);
    try testing.expectEqual(@as(u32, 0x00000019), ranked[0].syllables[0]);
}

test "play --trace prints the first N bundles with their changes, reads and writes" {
    const gpa = testing.allocator;
    var img = try imageOf(gpa,
        \\.pass init
        \\        x17.halt
        \\.pass decoder
        \\        lde e8, #0x40
        \\        x19.ld s3, #4, e8, #2
        \\        lde e6, #0x80
        \\        x17.sty e6, #3, #0
        \\        x17.halt
    );
    defer img.deinit(gpa);
    var out: std.Io.Writer.Allocating = .init(gpa);
    defer out.deinit();
    var tr: Tracer = .{ .gpa = gpa, .out = &out.writer, .left = 7 };
    defer tr.deinit();
    var r = try play(gpa, &img, &.{"a"}, .{ .y_writes = &.{.{ .addr = 0x40, .value = 0x1234 }}, .tracer = &tr });
    defer r.deinit(gpa);
    const text = out.written();
    // Three init bundles (the halt and its two-bundle shadow), then four of the decoder.
    try testing.expectEqual(@as(usize, 7), std.mem.count(u8, text, " q 0x"));
    try testing.expect(std.mem.indexOf(u8, text, "pkt 0 q 0x101: 00000019") != null);
    try testing.expect(std.mem.indexOf(u8, text, "s3 0x0->0x1234 e8 0x40->0x41 rY[0x40]=0x1234\n") != null);
    try testing.expect(std.mem.indexOf(u8, text, "wY[0x80]=0x1234\n") != null);
    try testing.expect(std.mem.indexOf(u8, text, "q 0x109") == null);
}

test "play --stop-at stops before the first decoder bundle at q and dumps the state" {
    const gpa = testing.allocator;
    var img = try imageOf(gpa,
        \\.pass init
        \\        x17.halt
        \\.pass decoder
        \\        lde e8, #0x40
        \\        x19.ld s3, #4, e8, #2
        \\        lde e6, #0x80
        \\        x17.sty e6, #3, #0
        \\        x17.halt
    );
    defer img.deinit(gpa);
    var out: std.Io.Writer.Allocating = .init(gpa);
    defer out.deinit();
    // q 0x105 is the lde e6 after the 4-syllable x19.ld: the store after it has not run.
    var r = try play(gpa, &img, &.{"a"}, .{ .y_writes = &.{.{ .addr = 0x40, .value = 0x1234 }}, .stop_at = decode_entry + 5, .dump = &out.writer });
    defer r.deinit(gpa);
    const t = r.trap orelse return error.TestUnexpectedResult;
    try testing.expectEqual(@as(anyerror, error.Breakpoint), t.err);
    try testing.expectEqual(decode_entry + 5, t.pc);
    const text = out.written();
    try testing.expect(std.mem.startsWith(u8, text, "{\"stop\":\"Breakpoint\",\"pass\":255,\"packet\":0,\"q\":261,"));
    try testing.expect(std.mem.indexOf(u8, text, "\"s\":[0,0,0,4660,") != null);
    try testing.expect(std.mem.indexOf(u8, text, "\"128\":") == null);
}
