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

/// Decode-arm entry (fact 770).
pub const decode_entry: usize = 256;

pub const Options = struct {
    rate_hz: u32 = 44100,
    /// Y words written once after the image is loaded, before the first packet (the per-track
    /// configure words a host writes).
    y_writes: []const host_mod.Param = &.{},
    max_bundles_init: usize = 1 << 20,
    max_bundles_packet: usize = 1 << 26,
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

    var res: Result = .{ .pcm = &.{}, .packets = 0, .bundles = 0, .waits = 0, .trap = null };

    // Init passes in load order: the sections come pass by pass.
    var last: ?u8 = null;
    for (img.sections) |sec| {
        if (sec.pass == image.decoder_pass or sec.pass == last) continue;
        last = sec.pass;
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
    for (opts.y_writes) |w| try h.state.y_mem.put(gpa, w.addr, w.value & 0xFFFF);

    for (pkts, 0..) |pkt, i| {
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
