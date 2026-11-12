//! x17 image: the load file for an assembled `Program` (asm.zig), and its
//! debug sidecar.
//!
//! Container, little-endian, version 1:
//!
//!   0   "X17I"
//!   4   u16 version (1)
//!   6   u16 section count
//!   8   sections, back to back:
//!         u8  kind      1 P, 2 X, 3 Y, 4 param, 5 mode
//!         u8  pass      init passes 0..N-1 in load order, 0xFF the decoder
//!         u16 0
//!         u32 addr      P: first row; X, Y, param: first word address
//!         u32 count     P: rows; X, Y, param: words
//!         payload       P: count * 4 u32, row by row, lanes 0..3 in each row
//!                       (lane l of row r is the syllable at q = 8*(r/2) + 2*l + r%2);
//!                       X: count u32 (24-bit); Y, param: count u32 (16-bit)
//!
//! A mode section (`.mode N`, fact 960) has pass 0xFF, addr = the configure mode and count 0;
//! when present it is the first section.
//!
//! Sections come pass by pass in load order: each pass's P, then its X, Y and
//! param blocks in source order.  A P section runs from the pass's base row
//! (0 init, 0x40 decoder) through the two zero rows after its last row.
//!
//! The sidecar (`debugJson`) maps every bundle to its q, rows and lanes,
//! file:line, source text and macro call, and lists labels per pass.

const std = @import("std");
const asm_mod = @import("asm");

const layout = asm_mod.layout;

pub const magic = "X17I";
pub const version: u16 = 1;
pub const decoder_pass: u8 = 0xFF;

pub const SectionKind = enum(u8) { p = 1, x = 2, y = 3, param = 4, mode = 5 };

pub const Section = struct {
    kind: SectionKind,
    pass: u8,
    addr: u32,
    count: u32,
    /// P: count * 4 syllables; otherwise count words.
    words: []const u32,
};

fn passId(prog: *const asm_mod.Program, i: usize) u8 {
    return if (prog.passes[i].kind == .decoder) decoder_pass else @intCast(i);
}

fn sectionKind(space: asm_mod.Space) SectionKind {
    return switch (space) {
        .x => .x,
        .y => .y,
        .param => .param,
    };
}

/// The P rows of one pass, base row through the two zero rows, row-major over lanes.
pub fn rows(gpa: std.mem.Allocator, pass: *const asm_mod.Pass) ![]u32 {
    const n_rows = pass.endRow() + 1 - pass.base_row;
    const out = try gpa.alloc(u32, n_rows * layout.lanes);
    @memset(out, 0);
    const base_q = pass.baseQ();
    for (pass.syllables, 0..) |w, k| {
        const q = base_q + @as(u32, @intCast(k));
        out[(layout.rowOf(q) - pass.base_row) * layout.lanes + layout.laneOf(q)] = w;
    }
    return out;
}

/// Serialise a program (caller frees).
pub fn write(gpa: std.mem.Allocator, prog: *const asm_mod.Program) ![]u8 {
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(gpa);
    try out.appendSlice(gpa, magic);
    try putInt(gpa, &out, u16, version);
    try putInt(gpa, &out, u16, 0); // patched below
    var count: u16 = 0;
    if (prog.mode) |m| {
        try putSection(gpa, &out, .{ .kind = .mode, .pass = decoder_pass, .addr = m, .count = 0, .words = &.{} });
        count += 1;
    }
    for (prog.passes, 0..) |*pass, i| {
        const p_rows = try rows(gpa, pass);
        defer gpa.free(p_rows);
        try putSection(gpa, &out, .{ .kind = .p, .pass = passId(prog, i), .addr = pass.base_row, .count = @intCast(p_rows.len / layout.lanes), .words = p_rows });
        count += 1;
        for (pass.data) |blk| {
            try putSection(gpa, &out, .{ .kind = sectionKind(blk.space), .pass = passId(prog, i), .addr = blk.addr, .count = @intCast(blk.words.len), .words = blk.words });
            count += 1;
        }
    }
    std.mem.writeInt(u16, out.items[6..8], count, .little);
    return out.toOwnedSlice(gpa);
}

fn putInt(gpa: std.mem.Allocator, out: *std.ArrayList(u8), comptime T: type, v: T) !void {
    var b: [@sizeOf(T)]u8 = undefined;
    std.mem.writeInt(T, &b, v, .little);
    try out.appendSlice(gpa, &b);
}

fn putSection(gpa: std.mem.Allocator, out: *std.ArrayList(u8), s: Section) !void {
    try out.append(gpa, @backingInt(s.kind));
    try out.append(gpa, s.pass);
    try putInt(gpa, out, u16, 0);
    try putInt(gpa, out, u32, s.addr);
    try putInt(gpa, out, u32, s.count);
    for (s.words) |w| try putInt(gpa, out, u32, w);
}

pub const ParseError = error{ BadMagic, BadVersion, Truncated, BadSection, OutOfMemory };

/// A parsed image: `sections[i].words` point into `words`.
pub const Image = struct {
    sections: []Section,
    words: []u32,

    pub fn deinit(self: *Image, gpa: std.mem.Allocator) void {
        gpa.free(self.sections);
        gpa.free(self.words);
    }

    /// The configure mode the image was built for, if it says.
    pub fn mode(self: *const Image) ?u8 {
        for (self.sections) |s| if (s.kind == .mode) return std.math.cast(u8, s.addr);
        return null;
    }
};

pub fn parse(gpa: std.mem.Allocator, bytes: []const u8) ParseError!Image {
    if (bytes.len < 8 or !std.mem.eql(u8, bytes[0..4], magic)) return error.BadMagic;
    if (std.mem.readInt(u16, bytes[4..6], .little) != version) return error.BadVersion;
    const n = std.mem.readInt(u16, bytes[6..8], .little);
    // First walk: bounds and sizes.
    var total: usize = 0;
    var at: usize = 8;
    for (0..n) |_| {
        const h = try header(bytes, at);
        total += h.len;
        at += 12 + 4 * h.len;
    }
    if (at != bytes.len) return error.Truncated;
    const sections = try gpa.alloc(Section, n);
    errdefer gpa.free(sections);
    const words = try gpa.alloc(u32, total);
    var w: usize = 0;
    at = 8;
    for (sections) |*s| {
        const h = try header(bytes, at);
        at += 12;
        for (words[w..][0..h.len]) |*dst| {
            dst.* = std.mem.readInt(u32, bytes[at..][0..4], .little);
            at += 4;
        }
        s.* = .{ .kind = h.kind, .pass = h.pass, .addr = h.addr, .count = h.count, .words = words[w..][0..h.len] };
        w += h.len;
    }
    return .{ .sections = sections, .words = words };
}

const Header = struct { kind: SectionKind, pass: u8, addr: u32, count: u32, len: usize };

fn header(bytes: []const u8, at: usize) ParseError!Header {
    if (at + 12 > bytes.len) return error.Truncated;
    const kind = std.enums.fromInt(SectionKind, bytes[at]) orelse return error.BadSection;
    const count = std.mem.readInt(u32, bytes[at + 4 ..][4..8], .little);
    const len: usize = if (kind == .p) @as(usize, count) * layout.lanes else count;
    if (at + 12 + 4 * len > bytes.len) return error.Truncated;
    return .{
        .kind = kind,
        .pass = bytes[at + 1],
        .addr = std.mem.readInt(u32, bytes[at + 4 ..][0..4], .little),
        .count = count,
        .len = len,
    };
}

/// Lay a P section into a q-indexed program array (`prog.len` >= every q it touches).
pub fn placeRows(prog: []u32, s: Section) void {
    for (0..s.count) |r| for (0..layout.lanes) |l| {
        const row = s.addr + @as(u32, @intCast(r));
        prog[layout.qOf(row, @intCast(l))] = s.words[r * layout.lanes + l];
    };
}

/// Serialise a parsed image back to bytes (the inverse of `parse`).
pub fn serialize(gpa: std.mem.Allocator, img: *const Image) ![]u8 {
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(gpa);
    try out.appendSlice(gpa, magic);
    try putInt(gpa, &out, u16, version);
    try putInt(gpa, &out, u16, @intCast(img.sections.len));
    for (img.sections) |s| try putSection(gpa, &out, s);
    return out.toOwnedSlice(gpa);
}

/// x17.halt plus the two nops its stop shadow runs.
pub const halt_patch = [_]u32{ 0x00000017, 0x31308000, 0xA0000000, 0x40000005, 0x40000005 };

pub const PatchError = error{QOutsidePass};

/// Overwrite the decoder pass's syllables at q, q+1, ... with `patch`; returns the old ones.
pub fn patchDecoder(img: *Image, q: u32, patch: []const u32, old: []u32) PatchError!void {
    var hit: usize = 0;
    for (img.sections) |s| {
        if (s.kind != .p or s.pass != decoder_pass) continue;
        const base = @intFromPtr(s.words.ptr) - @intFromPtr(img.words.ptr);
        for (patch, 0..) |w, k| {
            const qk = q + @as(u32, @intCast(k));
            const row = layout.rowOf(qk);
            if (row < s.addr or row >= s.addr + s.count) continue;
            const at = base / 4 + (row - s.addr) * layout.lanes + layout.laneOf(qk);
            old[k] = img.words[at];
            img.words[at] = w;
            hit += 1;
        }
    }
    if (hit != patch.len) return error.QOutsidePass;
}

// ---------------------------------------------------------------------------
// Debug sidecar
// ---------------------------------------------------------------------------

/// The sidecar as JSON (caller frees).
pub fn debugJson(gpa: std.mem.Allocator, prog: *const asm_mod.Program) ![]u8 {
    var aw: std.Io.Writer.Allocating = .init(gpa);
    errdefer aw.deinit();
    const o = &aw.writer;
    try o.writeAll("{\"version\":1,\"passes\":[");
    for (prog.passes, 0..) |*pass, i| {
        if (i > 0) try o.writeAll(",");
        try o.print("\n{{\"pass\":{d},\"kind\":\"{s}\",\"base_row\":{d},\"end_row\":{d},\"bundles\":[", .{ passId(prog, i), @tagName(pass.kind), pass.base_row, pass.endRow() });
        for (pass.sites, 0..) |site, k| {
            if (k > 0) try o.writeAll(",");
            try o.print("\n {{\"q\":{d},\"len\":{d},\"slots\":[", .{ site.q, site.len });
            for (0..site.len) |j| {
                const q = site.q + @as(u32, @intCast(j));
                if (j > 0) try o.writeAll(",");
                try o.print("[{d},{d}]", .{ layout.rowOf(q), layout.laneOf(q) });
            }
            try o.writeAll("],\"file\":");
            try jsonStr(o, site.file);
            try o.print(",\"line\":{d},\"text\":", .{site.line});
            try jsonStr(o, site.text);
            if (site.call) |c| {
                try o.writeAll(",\"macro\":{\"name\":");
                try jsonStr(o, c.name);
                try o.writeAll(",\"file\":");
                try jsonStr(o, c.file);
                try o.print(",\"line\":{d}}}", .{c.line});
            }
            try o.writeAll("}");
        }
        try o.writeAll("],\"labels\":{");
        var first = true;
        for (prog.labels) |lab| if (lab.pass == i) {
            if (!first) try o.writeAll(",");
            first = false;
            try jsonStr(o, lab.name);
            try o.print(":{d}", .{lab.q});
        };
        try o.writeAll("}}");
    }
    try o.writeAll("\n]}\n");
    return aw.toOwnedSlice();
}

fn jsonStr(o: *std.Io.Writer, s: []const u8) !void {
    try o.writeByte('"');
    for (s) |c| switch (c) {
        '"' => try o.writeAll("\\\""),
        '\\' => try o.writeAll("\\\\"),
        '\n' => try o.writeAll("\\n"),
        '\t' => try o.writeAll("\\t"),
        0...8, 11...31, 127 => try o.print("\\u{x:0>4}", .{c}),
        else => try o.writeByte(c),
    };
    try o.writeByte('"');
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

fn testProgram(src: []const u8) !asm_mod.Program {
    return switch (try asm_mod.assembleProgram(testing.allocator, src, .{ .path = "t.sm" })) {
        .ok => |p| p,
        .err => return error.TestUnexpectedResult,
    };
}

const two_pass =
    \\.pass init
    \\.xdata 0x10, 0x123456
    \\x17.halt
    \\.pass decoder
    \\.ydata 0x20, 0xBEEF, 7
    \\.param 0xFD83, 4
    \\top: x17.halt
;

test "image: sections in load order, P through the two zero rows" {
    var prog = try testProgram(two_pass);
    defer prog.deinit();
    const bytes = try write(testing.allocator, &prog);
    defer testing.allocator.free(bytes);
    var img = try parse(testing.allocator, bytes);
    defer img.deinit(testing.allocator);
    try testing.expectEqual(@as(usize, 5), img.sections.len);
    const p0 = img.sections[0];
    try testing.expectEqual(SectionKind.p, p0.kind);
    try testing.expectEqual(@as(u8, 0), p0.pass);
    // q0..2 sit in rows 0 and 1; rows 2 and 3 are the zero rows.
    try testing.expectEqual(@as(u32, 4), p0.count);
    try testing.expectEqualSlices(u32, &.{ 0x00000017, 0xA0000000, 0, 0 }, p0.words[0..4]);
    try testing.expectEqual(@as(u32, 0x31308000), p0.words[4]);
    try testing.expectEqual(SectionKind.x, img.sections[1].kind);
    const p1 = img.sections[2];
    try testing.expectEqual(decoder_pass, p1.pass);
    try testing.expectEqual(@as(u32, 0x40), p1.addr);
    try testing.expectEqual(SectionKind.param, img.sections[4].kind);
    try testing.expectEqualSlices(u32, &.{4}, img.sections[4].words);
}

test "image: placeRows puts the decoder back at q256" {
    var prog = try testProgram(two_pass);
    defer prog.deinit();
    const bytes = try write(testing.allocator, &prog);
    defer testing.allocator.free(bytes);
    var img = try parse(testing.allocator, bytes);
    defer img.deinit(testing.allocator);
    var p: [0x4000]u32 = @splat(0);
    placeRows(&p, img.sections[2]);
    try testing.expectEqualSlices(u32, &.{ 0x00000017, 0x31308000, 0xA0000000 }, p[256..259]);
}

test "image: patchDecoder puts a halt at q and serialize round-trips" {
    var prog = try testProgram(two_pass);
    defer prog.deinit();
    const bytes = try write(testing.allocator, &prog);
    defer testing.allocator.free(bytes);
    var img = try parse(testing.allocator, bytes);
    defer img.deinit(testing.allocator);
    var old: [halt_patch.len]u32 = undefined;
    try patchDecoder(&img, 257, &halt_patch, &old);
    try testing.expectEqual(@as(u32, 0x31308000), old[0]);
    const again = try serialize(testing.allocator, &img);
    defer testing.allocator.free(again);
    var img2 = try parse(testing.allocator, again);
    defer img2.deinit(testing.allocator);
    var p: [0x4000]u32 = @splat(0);
    placeRows(&p, img2.sections[2]);
    try testing.expectEqualSlices(u32, &halt_patch, p[257..262]);
    try testing.expectError(error.QOutsidePass, patchDecoder(&img, 0x3FFF, &halt_patch, &old));
}

test "image: parse refuses a bad magic, version and a cut file" {
    var prog = try testProgram(two_pass);
    defer prog.deinit();
    const bytes = try write(testing.allocator, &prog);
    defer testing.allocator.free(bytes);
    try testing.expectError(error.Truncated, parse(testing.allocator, bytes[0 .. bytes.len - 1]));
    var bad = try testing.allocator.dupe(u8, bytes);
    defer testing.allocator.free(bad);
    bad[4] = 2;
    try testing.expectError(error.BadVersion, parse(testing.allocator, bad));
    bad[0] = 'Y';
    try testing.expectError(error.BadMagic, parse(testing.allocator, bad));
}

test "image: the sidecar maps bundles to rows, lanes, lines and labels" {
    var prog = try testProgram(two_pass);
    defer prog.deinit();
    const j = try debugJson(testing.allocator, &prog);
    defer testing.allocator.free(j);
    var parsed = try std.json.parseFromSlice(std.json.Value, testing.allocator, j, .{});
    defer parsed.deinit();
    const dec = parsed.value.object.get("passes").?.array.items[1].object;
    try testing.expectEqualStrings("decoder", dec.get("kind").?.string);
    const b0 = dec.get("bundles").?.array.items[0].object;
    try testing.expectEqual(@as(i64, 256), b0.get("q").?.integer);
    try testing.expectEqual(@as(i64, 7), b0.get("line").?.integer);
    // q258 is row 0x40, lane 1.
    const s2 = b0.get("slots").?.array.items[2].array.items;
    try testing.expectEqual(@as(i64, 0x40), s2[0].integer);
    try testing.expectEqual(@as(i64, 1), s2[1].integer);
    try testing.expectEqual(@as(i64, 256), dec.get("labels").?.object.get("top").?.integer);
}

test "image: the sidecar names the macro call a bundle came from" {
    var prog = try testProgram(
        \\.macro stop() {
        \\    x17.halt
        \\}
        \\stop()
    );
    defer prog.deinit();
    const j = try debugJson(testing.allocator, &prog);
    defer testing.allocator.free(j);
    var parsed = try std.json.parseFromSlice(std.json.Value, testing.allocator, j, .{});
    defer parsed.deinit();
    const b0 = parsed.value.object.get("passes").?.array.items[0].object.get("bundles").?.array.items[0].object;
    const m = b0.get("macro").?.object;
    try testing.expectEqualStrings("stop", m.get("name").?.string);
    try testing.expectEqual(@as(i64, 4), m.get("line").?.integer);
}

test "image: jsonStr escapes quotes, backslashes and control bytes" {
    var aw: std.Io.Writer.Allocating = .init(testing.allocator);
    defer aw.deinit();
    try jsonStr(&aw.writer, "a\"b\\c\x01");
    try testing.expectEqualStrings("\"a\\\"b\\\\c\\u0001\"", aw.written());
}
