//! The host side of an sm1 run: what the ARM does around a codec program.
//!
//! Facts come from sosumi's specs/aac.md (input and output contracts).  The
//! four interface questions still open there (sosumi specs/questions.md 1..4)
//! are explicit stubs below, each named after its question, so a program
//! written against this model shows exactly what it assumed:
//!
//!   Q1 entry       `Config.entry`: the syllable the program starts at.
//!   Q2 packet in   `Config.packet`: the host stages packet bytes in Y.
//!   Q3 hand-back   one packet per run; the run ends at the program's stop.
//!   Q4 clobber     `Config.clobber`: regions the host poisons between packets.
//!
//! What is modelled from measurement: the descriptor control word, the
//! parameter block (flags, rate index), and the output ring the host drains
//! (entry table, PCM buffers, status header), including the rule that the
//! program must not write a buffer the host still owns.
//!
//! Y addresses are words.  The default `Map` follows the spec's engine view,
//! two bytes per Y word from 0x22038000 (Y:0x80 at 0x22038100).  isa.toml's
//! `param` region puts the 0x2203FF00 page at Y:0xFD80 instead of this map's
//! Y:0x3F80; which one holds is OPEN, so the map is a parameter.

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
    /// +2, +4, +0xA + 4i, +0xC + 4i).
    entry_table: u32 = 0x0000,
    /// PCM buffers, buffer i at + i * units, interleaved 16-bit.  MEASURED.
    pcm: u32 = 0x0080,
    /// Status header (0x2203FF28): +0 bit 0 output ready, +1 must be 0.
    /// Address MEASURED, Y mapping INFERRED.
    status: u32 = 0x3F94,
    /// Parameter block (0x2203FF00): +0 flags, +3 rate index (FF06).
    param: u32 = 0x3F80,
};

pub const Region = struct { lo: u32, hi: u32 };

pub const Config = struct {
    map: Map = .{},
    /// Q1 STUB: where execution enters after the two-pass load.
    entry: usize = 0,
    /// Q2 STUB: how the program reads packet bytes.  Until measured, the host
    /// writes the length in bytes at `len_at` and the bytes from `base`,
    /// big-endian pairs, two bytes per Y word.
    packet: struct { base: u32 = 0x4000, len_at: u32 = 0x3FFF } = .{},
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
    PacketTooLong,
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

    pub fn init(gpa: std.mem.Allocator, cfg: Config) Error!Host {
        var h: Host = .{ .gpa = gpa, .cfg = cfg };
        // Configure step 3: FF00 |= 1, mode 0 FF00 |= 6, FF06 = rate index.
        try h.orY(cfg.map.param, 7);
        try h.putY(cfg.map.param + 3, rateIndex(cfg.rate_hz));
        return h;
    }

    pub fn deinit(self: *Host) void {
        self.state.deinit(self.gpa);
        self.pcm.deinit(self.gpa);
    }

    fn getY(self: *const Host, addr: u32) u32 {
        return self.state.y_mem.get(addr) orelse 0;
    }

    fn putY(self: *Host, addr: u32, v: u32) Error!void {
        try self.state.y_mem.put(self.gpa, addr, v);
    }

    fn orY(self: *Host, addr: u32, v: u32) Error!void {
        try self.putY(addr, self.getY(addr) | v);
    }

    pub const Report = struct { bundles: usize, units: usize };

    /// One packet: stage it, run the program from the entry to its stop,
    /// check the owned buffers survived, drain the ring.
    pub fn feed(self: *Host, program: []const u32, packet: []const u8) Error!Report {
        const m = self.cfg.map;
        for (self.cfg.clobber) |r| {
            var a = r.lo;
            while (a <= r.hi) : (a += 1) try self.putY(a, self.cfg.clobber_value);
        }
        try self.stage(packet);

        const guard = try self.snapshotOwned();
        defer self.gpa.free(guard);

        const bundles = try emu.runFrom(self.gpa, &self.state, program, self.cfg.entry, self.cfg.max_bundles);
        if (!self.state.halted) return error.NoStop;

        for (guard) |g| {
            if (self.state.y_mem.get(g.addr) != g.value) {
                self.overwritten = g.entry;
                return error.OverwroteOwnedBuffer;
            }
        }
        const before = self.pcm.items.len;
        if (self.getY(m.status) & 1 != 0 and self.getY(m.status + 1) == 0) try self.drain();
        return .{ .bundles = bundles, .units = self.pcm.items.len - before };
    }

    fn stage(self: *Host, packet: []const u8) Error!void {
        const p = self.cfg.packet;
        if (packet.len > 0xFFFF) return error.PacketTooLong;
        try self.putY(p.len_at, @intCast(packet.len));
        var i: usize = 0;
        while (i < packet.len) : (i += 2) {
            const hi: u32 = packet[i];
            const lo: u32 = if (i + 1 < packet.len) packet[i + 1] else 0;
            try self.putY(p.base + @as(u32, @intCast(i / 2)), hi << 8 | lo);
        }
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
                try list.append(self.gpa, .{ .entry = e, .addr = addr, .value = self.state.y_mem.get(addr) });
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

test "init writes the parameter block" {
    var h = try Host.init(testing.allocator, .{ .rate_hz = 48000 });
    defer h.deinit();
    try testing.expectEqual(@as(u32, 7), h.getY(0x3F80));
    try testing.expectEqual(@as(u32, 3), h.getY(0x3F83));
}

test "a packet is staged as big-endian pairs with its length" {
    const prog = try assemble(&.{"x17.halt"});
    defer testing.allocator.free(prog);
    var h = try Host.init(testing.allocator, .{});
    defer h.deinit();
    _ = try h.feed(prog, &.{ 0xDE, 0xAD, 0xBE });
    try testing.expectEqual(@as(u32, 3), h.getY(0x3FFF));
    try testing.expectEqual(@as(u32, 0xDEAD), h.getY(0x4000));
    try testing.expectEqual(@as(u32, 0xBE00), h.getY(0x4001));
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
