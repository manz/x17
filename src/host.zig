//! The host side of an sm1 run: what the ARM does around a codec program.
//!
//! Facts come from sosumi's specs/aac.md (input and output contracts) and the
//! answers in sosumi specs/questions.md.  What is still unknown is a named
//! stub, so a program written against this model shows what it assumed:
//!
//!   Q1 entry       `Config.entry`: syllable 0 of the program window
//!                  (MEASURED); kept settable for tests.
//!   Q2 packet in   the descriptor DMA feeds the input stream (fact 82); the
//!                  program enables it with ld0e.sen and reads it with
//!                  x16.getb (783).  A read past the queued bits waits for
//!                  the next packet, and the bits run on across packets (784):
//!                  `feed` then reports `waiting` and the next `feed` resumes
//!                  at that read.
//!   Q3 hand-back   one packet per run; the run ends at the stop form
//!                  (MEASURED).  No CPU code writes the status header, so the
//!                  hardware raises it (FIRMWARE); what makes it do so, and
//!                  the interrupt, are unknown: tests raise it by hand.
//!   Q4 clobber     STUB `Config.clobber`: regions poisoned between packets.
//!
//! What is modelled from measurement: the descriptor control word, the
//! parameter block (flags, rate index), and the output ring the host drains
//! (entry table, PCM buffers, status header), including the rule that the
//! program must not write a buffer the host still owns.
//!
//! Y addresses are words.  The parameter block is Y:0xFD80 (FIRMWARE).  The
//! output ring is measured (fact 785): two slots used alternately per packet,
//! slot k's PCM at Y:0x80 + 0x800k (2048 units), its entry owned at Y:2k+5 and
//! length at Y:2k+6, the status header at Y:0xFD94 bit 0, then the stop.  The
//! map stays a parameter.

const std = @import("std");
const emu = @import("emu");

/// Descriptor control word for an N-byte packet: N << 16 | 0x1F << 7
/// (MEASURED: 6 bytes -> 0x00060F80).
pub fn controlWord(n_bytes: u16) u32 {
    return @as(u32, n_bytes) << 16 | 0x1F << 7;
}

/// The one per-track parameter (FF06): the AAC sampling-rate index.  44100
/// and any unlisted rate give 4 (FIRMWARE, specs/aac.md configure step 3).
pub fn rateIndex(hz: u32) u16 {
    const rates = [_]u32{ 96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000 };
    for (rates, 0..) |r, i| if (r == hz) return @intCast(i);
    return 4;
}

pub const Map = struct {
    /// Entry table: +1 ring size, +2 units per buffer; entry i owned at
    /// +5 + 2i, length (16-bit units) at +6 + 2i.  MEASURED (byte offsets
    /// +2, +4, +0xA + 4i, +0xC + 4i); at Y:0 (fact 785).
    entry_table: u32 = 0x0000,
    /// PCM buffers, buffer i at + i * units, interleaved 16-bit; at Y:0x80
    /// with 0x800 units per buffer (fact 785).
    pcm: u32 = 0x0080,
    /// Status header: +0 bit 0 output ready, +1 must be 0; at Y:0xFD94
    /// (fact 785).
    status: u32 = 0xFD94,
    /// Parameter block (Y:0xFD80, FIRMWARE): +0 flags, +3 rate
    /// index (FF06).
    param: u32 = 0xFD80,
};

/// The parameter block's words before configure: ASCII "--" (fact 828).
pub const param_fill: u32 = 0x2D2D;
pub const param_words = 8;

pub const Region = struct { lo: u32, hi: u32 };

pub const Param = struct { addr: u32, value: u32 };

pub const Config = struct {
    map: Map = .{},
    /// Q1: execution enters at syllable 0 of the program window (MEASURED).
    entry: usize = 0,
    /// STUB: codec parameters (an ALAC cookie's frame length, depth, Rice
    /// constants, channels) as Y words.  How they reach the program on
    /// silicon is unknown; the host writes them before each packet, after
    /// the clobber.
    params: []const Param = &.{},
    /// Q4 STUB: Y regions the host or boot code overwrites between packets.
    clobber: []const Region = &.{},
    clobber_value: u32 = 0,
    /// Sampling rate written to the parameter block.
    rate_hz: u32 = 44100,
    /// Entries the host takes per packet; null drains every owned entry.
    /// A small value models a host that lags behind the program.
    drain_limit: ?usize = null,
    /// Bundle budget for one packet's run.
    max_bundles: usize = 1 << 20,
};

pub const Error = emu.RunError || error{
    /// The run ended without a stop: a wedge on silicon.
    NoStop,
    /// Ring size 0 with an entry owned.
    BadRing,
    /// An entry's length is over its buffer's units.
    LengthOverBuffer,
    /// The program wrote a buffer the host still owned (MEASURED failure:
    /// with the ring full the hand-back stalls).
    OverwroteOwnedBuffer,
};

pub const Host = struct {
    gpa: std.mem.Allocator,
    cfg: Config,
    state: emu.State = .{},
    /// The next entry the host takes.
    idx: u32 = 0,
    /// PCM the host has taken, in ring order.
    pcm: std.ArrayList(i16) = .empty,
    /// The entry that last tripped `OverwroteOwnedBuffer`.
    overwritten: ?u32 = null,
    /// Where a run waiting on the stream stopped: the next `feed` resumes there (784).
    resume_at: ?usize = null,

    pub fn init(gpa: std.mem.Allocator, cfg: Config) Error!Host {
        var h: Host = .{ .gpa = gpa, .cfg = cfg };
        try h.writeParams();
        return h;
    }

    /// The firmware leaves the parameter block as "----" fill (0x2D2D per word, fact 828)
    /// and configures over it: FF00 |= 1, mode 0 FF00 |= 6, FF06 = rate index.
    fn writeParams(self: *Host) Error!void {
        const m = self.cfg.map;
        for (0..param_words) |k| try self.putY(m.param + @as(u32, @intCast(k)), param_fill);
        try self.orY(m.param, 7);
        try self.putY(m.param + 3, rateIndex(self.cfg.rate_hz));
    }

    /// The next track's configure on the same engine: the host starts its ring over, drops
    /// the old track's unread stream and writes the parameter block again.  The stream enable
    /// drops too (fact 958): a decoder must enable it again.  Everything else the engine
    /// holds is left as the last track left it.
    pub fn reconfigure(self: *Host) Error!void {
        self.idx = 0;
        self.resume_at = null;
        self.overwritten = null;
        self.state.stream_on = false;
        self.state.stream.clearRetainingCapacity();
        self.state.stream_bit = 0;
        self.state.stream_starved = false;
        try self.writeParams();
    }

    pub fn deinit(self: *Host) void {
        self.state.deinit(self.gpa);
        self.pcm.deinit(self.gpa);
    }

    fn getY(self: *const Host, addr: u32) u32 {
        return self.state.peekY(addr) orelse 0;
    }

    fn putY(self: *Host, addr: u32, v: u32) Error!void {
        try self.state.y_mem.put(self.gpa, addr, v);
    }

    fn orY(self: *Host, addr: u32, v: u32) Error!void {
        try self.putY(addr, self.getY(addr) | v);
    }

    /// `waiting`: the run read past the queued bits and waits for the next packet (784).
    pub const Report = struct { bundles: usize, units: usize, waiting: bool = false };

    /// Queue a packet on the input stream without arming a run (945: feed 1 arms nothing).
    pub fn queue(self: *Host, packet: []const u8) Error!void {
        try self.state.queueStream(self.gpa, packet);
    }

    /// One packet: queue it on the input stream, run the program from the entry (or from
    /// the read a waiting run stopped at) to its stop, check the owned buffers survived,
    /// drain the ring.
    pub fn feed(self: *Host, program: []const u32, packet: []const u8) Error!Report {
        const m = self.cfg.map;
        for (self.cfg.clobber) |r| {
            var a = r.lo;
            while (a <= r.hi) : (a += 1) try self.putY(a, self.cfg.clobber_value);
        }
        for (self.cfg.params) |q| try self.putY(q.addr, q.value);
        try self.state.queueStream(self.gpa, packet);

        const guard = try self.snapshotOwned();
        defer self.gpa.free(guard);

        // Each run enters at the entry syllable (Q1), in its own lane, not the last run's halt
        // lane: runFrom counts lanes from cur_lane at syllable 0.
        self.state.cur_lane = 0;
        // 928: the decode arm's +0x988 entry is also its call base.
        self.state.call_base = self.cfg.entry;
        const start = self.resume_at orelse self.cfg.entry;
        self.resume_at = null;
        const bundles = try emu.runFrom(self.gpa, &self.state, program, start, self.cfg.max_bundles);
        if (self.state.stream_starved) self.resume_at = self.state.pc;
        if (!self.state.halted and self.resume_at == null) return error.NoStop;

        for (guard) |g| {
            if (self.state.peekY(g.addr) != g.value) {
                self.overwritten = g.entry;
                return error.OverwroteOwnedBuffer;
            }
        }
        if (self.resume_at != null) return .{ .bundles = bundles, .units = 0, .waiting = true };
        const before = self.pcm.items.len;
        if (self.getY(m.status) & 1 != 0 and self.getY(m.status + 1) == 0) try self.drain();
        return .{ .bundles = bundles, .units = self.pcm.items.len - before };
    }

    const Guard = struct { entry: u32, addr: u32, value: ?u32 };

    /// Every word of every buffer the host still owns, as it stands now.
    fn snapshotOwned(self: *Host) Error![]Guard {
        const m = self.cfg.map;
        var list: std.ArrayList(Guard) = .empty;
        errdefer list.deinit(self.gpa);
        const ring = self.getY(m.entry_table + 1);
        const units = self.getY(m.entry_table + 2);
        var e: u32 = 0;
        while (e < ring) : (e += 1) {
            if (self.getY(m.entry_table + 5 + 2 * e) == 0) continue;
            for (0..units) |k| {
                const addr = m.pcm + e * units + @as(u32, @intCast(k));
                try list.append(self.gpa, .{ .entry = e, .addr = addr, .value = self.state.peekY(addr) });
            }
        }
        return list.toOwnedSlice(self.gpa);
    }

    /// The host's hand-back (FIRMWARE, specs/aac.md output contract): take
    /// entry idx while it is owned, copy its length, clear the flag, advance
    /// idx modulo the ring size.
    fn drain(self: *Host) Error!void {
        const m = self.cfg.map;
        const ring = self.getY(m.entry_table + 1);
        const units = self.getY(m.entry_table + 2);
        var taken: usize = 0;
        while (self.cfg.drain_limit == null or taken < self.cfg.drain_limit.?) : (taken += 1) {
            const owned_at = m.entry_table + 5 + 2 * self.idx;
            if (self.getY(owned_at) == 0) break;
            if (ring == 0) return error.BadRing;
            const len = self.getY(owned_at + 1);
            if (len > units) return error.LengthOverBuffer;
            const base = m.pcm + self.idx * units;
            for (0..len) |k| {
                const w: u16 = @truncate(self.getY(base + @as(u32, @intCast(k))));
                try self.pcm.append(self.gpa, @bitCast(w));
            }
            try self.putY(owned_at, 0);
            self.idx = (self.idx + 1) % ring;
        }
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;
const asm_mod = @import("asm");

fn assemble(comptime lines: []const []const u8) ![]u32 {
    var words: std.ArrayList(u32) = .empty;
    errdefer words.deinit(testing.allocator);
    inline for (lines) |l| {
        const b = try asm_mod.assembleOne(l);
        try words.appendSlice(testing.allocator, b.slice());
    }
    return words.toOwnedSlice(testing.allocator);
}

/// What a program would leave: a ring of `ring` buffers of `units`, entry
/// `e` owned with `samples`, and the status header raised.
fn publish(h: *Host, ring: u32, units: u32, e: u32, samples: []const i16) !void {
    const m = h.cfg.map;
    try h.putY(m.entry_table + 1, ring);
    try h.putY(m.entry_table + 2, units);
    for (samples, 0..) |s, k| try h.putY(m.pcm + e * units + @as(u32, @intCast(k)), @as(u16, @bitCast(s)));
    try h.putY(m.entry_table + 5 + 2 * e, 1);
    try h.putY(m.entry_table + 6 + 2 * e, @intCast(samples.len));
    try h.putY(m.status, 1);
}

test "the descriptor control word matches the measured 6-byte packet" {
    try testing.expectEqual(@as(u32, 0x00060F80), controlWord(6));
}

test "rate index: listed rates, and unlisted ones fall back to 44100's" {
    try testing.expectEqual(@as(u16, 4), rateIndex(44100));
    try testing.expectEqual(@as(u16, 11), rateIndex(8000));
    try testing.expectEqual(@as(u16, 4), rateIndex(37800));
}

test "reconfigure drops the stream enable and the unread stream, and keeps registers (958)" {
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    h.state.stream_on = true;
    h.state.s[3] = 0x123;
    try h.queue("ab");
    h.state.stream_bit = 3;
    try h.reconfigure();
    try testing.expect(!h.state.stream_on);
    try testing.expectEqual(@as(usize, 0), h.state.stream.items.len);
    try testing.expectEqual(@as(usize, 0), h.state.stream_bit);
    try testing.expectEqual(@as(u32, 0x123), h.state.s[3]);
}

test "init writes the parameter block" {
    var h = try Host.init(testing.allocator, .{ .rate_hz = 48000 });
    defer h.deinit();
    try testing.expectEqual(@as(u32, 0x2D2F), h.getY(0xFD80));
    try testing.expectEqual(@as(u32, 0x2D2D), h.getY(0xFD82));
    try testing.expectEqual(@as(u32, 3), h.getY(0xFD83));
    try testing.expectEqual(@as(u32, 0x2D2D), h.getY(0xFD87));
}

test "packets join one input stream; a read past them waits for the next (783, 784)" {
    const prog = try assemble(&.{
        "ldi s12, #1",      "ldi s14, #4",     "ld0e.sen #0, #0",
        "x16.getb #7, #11", "x16.getb #6, #3", "x17.halt",
    });
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    const first = try h.feed(prog, &.{0xDE});
    try testing.expect(first.waiting);
    try testing.expectEqual(@as(?u32, null), h.state.getY(0x4000)); // nothing staged in Y
    const second = try h.feed(prog, &.{ 0xAD, 0xBE });
    try testing.expect(!second.waiting);
    try testing.expectEqual(@as(u32, 0xDEA), h.state.a[15]); // 12 bits across both packets
    try testing.expectEqual(@as(u32, 0xD), h.state.a[13]);
}

test "the host drains an owned entry once the status header is raised" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    try publish(&h, 2, 4, 0, &.{ 1, -2, 3 });
    const r = try h.feed(prog, &.{});
    try testing.expectEqual(@as(usize, 3), r.units);
    try testing.expectEqualSlices(i16, &.{ 1, -2, 3 }, h.pcm.items);
    try testing.expectEqual(@as(u32, 0), h.getY(5)); // owned flag cleared
    try testing.expectEqual(@as(u32, 1), h.idx);
}

test "the measured ring: two 0x800-unit slots taken alternately per packet (785)" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    for ([_]i16{ 10, 11, 12 }, 0..) |s, n| {
        const k: u32 = @intCast(n % 2);
        try publish(&h, 2, 0x800, k, &.{ s, -s });
        try testing.expectEqual(@as(u32, @as(u16, @bitCast(s))), h.getY(0x80 + 0x800 * k));
        try testing.expectEqual(@as(u32, 2), h.getY(2 * k + 6));
        try testing.expectEqual(@as(usize, 2), (try h.feed(prog, &.{})).units);
        try testing.expectEqual(@as(u32, 0), h.getY(2 * k + 5)); // released
    }
    try testing.expectEqualSlices(i16, &.{ 10, -10, 11, -11, 12, -12 }, h.pcm.items);
}

test "no status bit, or a nonzero second word, takes nothing" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    try publish(&h, 2, 4, 0, &.{ 1, 2 });
    try h.putY(h.cfg.map.status, 0);
    try testing.expectEqual(@as(usize, 0), (try h.feed(prog, &.{})).units);
    try h.putY(h.cfg.map.status, 1);
    try h.putY(h.cfg.map.status + 1, 1);
    try testing.expectEqual(@as(usize, 0), (try h.feed(prog, &.{})).units);
}

test "idx wraps modulo the ring size" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    try publish(&h, 2, 2, 0, &.{7});
    try publish(&h, 2, 2, 1, &.{8});
    _ = try h.feed(prog, &.{});
    try publish(&h, 2, 2, 0, &.{9});
    _ = try h.feed(prog, &.{});
    try testing.expectEqualSlices(i16, &.{ 7, 8, 9 }, h.pcm.items);
    try testing.expectEqual(@as(u32, 1), h.idx);
}

test "a program that writes a still-owned buffer is caught" {
    // st.y stores lane 0's V register to Y:0x80, buffer 0's first word.
    const prog = try assemble(&.{ "st.y Y:0x80", "x17.halt" });
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{ .drain_limit = 0 });
    defer h.deinit();
    try publish(&h, 2, 4, 0, &.{5});
    h.state.v[0] = 0x1234;
    try testing.expectError(error.OverwroteOwnedBuffer, h.feed(prog, &.{}));
    try testing.expectEqual(@as(?u32, 0), h.overwritten);
}

test "a run without a stop is a wedge" {
    const prog = try assemble(&.{"ldi s3, #1"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    try testing.expectError(error.NoStop, h.feed(prog, &.{}));
}

test "the entry stub starts the run past the first bundle" {
    const prog = try assemble(&.{ "ldi s3, #1", "ldi s4, #2", "x17.halt" });
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{ .entry = 1 });
    defer h.deinit();
    _ = try h.feed(prog, &.{});
    try testing.expectEqual(@as(u32, 0), h.state.s[3]);
    try testing.expectEqual(@as(u32, 2), h.state.s[4]);
}

test "clobber regions are poisoned before each packet" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{
        .clobber = &.{.{ .lo = 0x100, .hi = 0x101 }},
        .clobber_value = 0xAAAA,
    });
    defer h.deinit();
    try h.putY(0x100, 1);
    _ = try h.feed(prog, &.{});
    try testing.expectEqual(@as(u32, 0xAAAA), h.getY(0x100));
    try testing.expectEqual(@as(u32, 0xAAAA), h.getY(0x101));
}

test "params are written before each packet, over the clobber" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{
        .params = &.{ .{ .addr = 0x2E00, .value = 4096 }, .{ .addr = 0x2E05, .value = 2 } },
        .clobber = &.{.{ .lo = 0x2E00, .hi = 0x2E05 }},
        .clobber_value = 0xAAAA,
    });
    defer h.deinit();
    for (0..2) |_| {
        _ = try h.feed(prog, &.{});
        try testing.expectEqual(@as(u32, 4096), h.getY(0x2E00));
        try testing.expectEqual(@as(u32, 2), h.getY(0x2E05));
        try testing.expectEqual(@as(u32, 0xAAAA), h.getY(0x2E01));
    }
}

test "every packet starts in lane 0, not the last run's halt lane" {
    const prog = try assemble(&.{ "ldi s3, #1", "x17.halt" });
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    _ = try h.feed(prog, &.{});
    try testing.expect(h.state.cur_lane != 0);
    _ = try h.feed(prog, &.{});
    var fresh = try Host.init(testing.allocator, .{});
    defer fresh.deinit();
    _ = try fresh.feed(prog, &.{});
    try testing.expectEqual(fresh.state.cur_lane, h.state.cur_lane);
}
