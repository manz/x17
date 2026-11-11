//! ISA tests: each pins a measured fact in the emulator, the assembler or the docs.
const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");
const image = @import("image");

const testing = std.testing;

/// Sources `.include` can name in these tests: none since the AAC libs moved out of x17.
const libs = [_]struct { path: []const u8, text: []const u8 }{};

fn load(ctx: ?*anyopaque, gpa: std.mem.Allocator, path: []const u8) anyerror!?[]u8 {
    _ = ctx;
    for (libs) |l| if (std.mem.eql(u8, l.path, path)) return try gpa.dupe(u8, l.text);
    return null;
}

fn assemble(gpa: std.mem.Allocator, src: []const u8) ![]u32 {
    return switch (try asm_mod.assembleFileWith(gpa, src, .{ .path = "t.sm", .loader = .{ .load = load } })) {
        .ok => |p| p,
        .err => |fe| {
            std.debug.print("line {d}: {s}\n", .{ fe.line, fe.lineText() });
            return error.Assemble;
        },
    };
}

test "the libraries branch only on measured ptst rows" {
    // fact 387/389: the b-only rows (b = s4, s8, s12), each gated on silicon, are the
    // ones the libraries may use.
    // Row 000 (c0-c3) is a <= -b, which read as "a < 0" mis-branched on silicon.
    const allowed = [_][]const u8{ "#LT4", "#LT8", "#LT12", "#LE4", "#LE8", "#LE12", "#EQ4", "#EQ8", "#EQ12" };
    for (libs) |l| {
        var lines = std.mem.splitScalar(u8, l.text, '\n');
        var n: usize = 0;
        while (lines.next()) |line| {
            n += 1;
            const code = if (std.mem.indexOfScalar(u8, line, ';')) |c| line[0..c] else line;
            const at = std.mem.indexOf(u8, code, "ptst ") orelse continue;
            var fields = std.mem.tokenizeAny(u8, code[at + 5 ..], ", \t");
            _ = fields.next(); // p
            const a = fields.next() orelse return error.BadPtst;
            const cond = fields.next() orelse return error.BadPtst;
            var ok = std.mem.eql(u8, a, "s0");
            if (ok) {
                ok = false;
                for (allowed) |w| ok = ok or std.mem.eql(u8, cond, w);
            }
            if (!ok) {
                std.debug.print("{s}:{d}: ptst {s}, {s} is not a measured b-only row\n", .{ l.path, n, a, cond });
                return error.UnmeasuredPtst;
            }
        }
    }
}

const HCode = struct { code: u32, len: u5, v: u16 };

/// A random complete prefix code: split random leaves of a binary tree, lengths up to 19.
fn randomCode(rnd: std.Random, out: []HCode) []HCode {
    return randomCodeN(rnd, out, rnd.intRangeAtMost(usize, 2, out.len));
}

/// randomCode with exactly n codewords.
fn randomCodeN(rnd: std.Random, out: []HCode, n: usize) []HCode {
    var lens: [400]u5 = undefined;
    lens[0] = 1;
    lens[1] = 1;
    var have: usize = 2;
    while (have < n) {
        const i = rnd.uintLessThan(usize, have);
        if (lens[i] >= 19) continue;
        lens[i] += 1;
        lens[have] = lens[i];
        have += 1;
    }
    // canonical codes, shortest first
    std.mem.sort(u5, lens[0..n], {}, std.sort.asc(u5));
    var code: u32 = 0;
    var prev: u5 = lens[0];
    for (out[0..n], lens[0..n], 0..) |*c, l, i| {
        if (i > 0) code = (code + 1) << (l - prev);
        prev = l;
        c.* = .{ .code = code, .len = l, .v = rnd.uintLessThan(u16, 1024) }; // entries stay below 0x8000: ldy sign-extends
    }
    rnd.shuffle(HCode, out[0..n]);
    return out[0..n];
}

/// sosumi ref/dsp/aac/huffman.zig's table layout: leaf (v << 5) + len, pointer
/// (base << 5) + 16 + width.
fn buildTables(t: []u16, n: *usize, syms: []const HCode, depth: u5, width: u4) usize {
    const base = n.*;
    const size = @as(usize, 1) << width;
    n.* += size;
    @memset(t[base..][0..size], 0);
    for (syms) |s| {
        const r = s.len - depth;
        if (r > width) continue;
        const rem = s.code & ((@as(u32, 1) << r) - 1);
        const start = @as(usize, rem) << @intCast(width - r);
        for (0..@as(usize, 1) << @intCast(width - r)) |j| t[base + start + j] = (s.v << 5) + r;
    }
    for (0..size) |p| {
        var group: [400]HCode = undefined;
        var ng: usize = 0;
        var maxrem: u5 = 0;
        for (syms) |s| {
            const r = s.len - depth;
            if (r <= width) continue;
            const rem = s.code & ((@as(u32, 1) << r) - 1);
            if (rem >> @intCast(r - width) != p) continue;
            group[ng] = s;
            ng += 1;
            maxrem = @max(maxrem, r - width);
        }
        if (ng == 0) continue;
        const w2: u4 = @intCast(@min(8, maxrem));
        const sub = buildTables(t, n, group[0..ng], depth + width, w2);
        t[base + p] = @intCast((sub << 5) + 16 + w2);
    }
    return base;
}

test "no test program puts a label inside the halt's two-bundle shadow" {
    // fact 72: two bundles after x17.halt still run, so code placed there (a fail
    // marker, say) runs on the success path too.
    var lines = std.mem.splitScalar(u8, @embedFile("lib_test.zig"), '\n');
    var since: ?usize = null;
    while (lines.next()) |line| {
        // Only multiline-string program lines count; Zig code after a one-line program does not.
        if (!std.mem.startsWith(u8, std.mem.trimStart(u8, line, " "), "\\\\")) {
            since = null;
            continue;
        }
        const t = std.mem.trim(u8, line, " \\");
        if (std.mem.indexOf(u8, t, "x17.halt") != null) {
            since = 0;
            continue;
        }
        const n = since orelse continue;
        if (n >= 2 or t.len == 0) {
            if (n >= 2) since = null;
            continue;
        }
        if (std.mem.indexOfScalar(u8, t, ':') != null and std.mem.indexOf(u8, t, "nop()") == null) return error.LabelInHaltShadow;
        since = n + 1;
    }
}

const BitWriter = struct {
    buf: [1024]u8 = @splat(0),
    n: usize = 0,

    fn put(self: *BitWriter, value: u32, len: u5) void {
        var k = len;
        while (k > 0) {
            k -= 1;
            if ((value >> k) & 1 == 1) self.buf[self.n >> 3] |= @as(u8, 0x80) >> @intCast(self.n & 7);
            self.n += 1;
        }
    }
};

test "a Y store in x17.ms.stop's shadow is refused, not guessed" {
    // fact 465: in shadow slot 1 it is lost; in slot 2 it reads address 0 unless slot 1
    // wrote s4. Neither is modelled.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        x17.ms.stop s0, #0, s5, e3, #1, #3, #0, R2, #1
        \\        x17.ms s0, #0, s1, e6, #0, #0, #0, R0, #0
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[3] = 0x740;
    state.e[6] = 0x700;
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, prog, 0, 64));
}

test "x17.ms.stop with no store source is refused: silicon stored s1 there" {
    // fact 465: with S1[27:24] = 0 the bundle's own store wrote s1, not nothing.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        x17.ms.stop s0, #0, s0, e3, #1, #3, #0, R2, #1
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, prog, 0, 64));
}

test "an e0 address after a bundle no measurement covers is refused" {
    // fact 468: e0 follows some forms' [20:5] and not others'; no general rule.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        ldi s5, #0x730
        \\        x02.ainc #6, #6
        \\        st.wr #0x40, e0, s1
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, prog, 0, 64));
}

// ---------------------------------------------------------------------------
// AAC inverse quantisation: sosumi ref/dsp/aac/dequant.zig, plain integers.

fn rshr(a: i64, n: u6) i64 {
    if (n == 0) return a;
    return (a + (@as(i64, 1) << (n - 1))) >> n;
}

fn runSpliced(gpa: std.mem.Allocator, state: *emu.State, pre: []const u8, raw: []const u32, post: []const u8) !void {
    const a = try assemble(gpa, pre);
    defer gpa.free(a);
    const b = try assemble(gpa, post);
    defer gpa.free(b);
    const prog = try std.mem.concat(gpa, u32, &.{ a, raw, b });
    defer gpa.free(prog);
    _ = try emu.runFrom(gpa, state, prog, 0, 256);
}

test "a pc14 pair loading an immediate sets e0 to its [20:5]" {
    // fact 468: 16E29474 42382CD8 (s7 <- #0x14A3), then the st pair.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try runSpliced(gpa, &state, "        ldi s1, #0x55\n", &.{ 0x16E29474, 0x42382CD8 },
        \\        st.wr #0x40, e0, s1
        \\        x17.halt
        \\
    );
    try testing.expectEqual(@as(?u32, 0x55), state.getY(0x14A3));
}

test "lde e7 passes the e0 latch through" {
    // fact 470: ldi s1 <- 0x1234, lde e7 <- 0x710, st pair: Y:0x1234.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        ldi s1, #0x1234
        \\        lde e7, #0x710
        \\        st.wr #0x40, e0, s1
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog, 0, 64);
    try testing.expectEqual(@as(?u32, 0x1234), state.getY(0x1234));
}

fn pc14P(src: []const u8, a: u32) !u8 {
    const gpa = testing.allocator;
    const prog = try assemble(gpa, src);
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[3] = a & 0xFFFFFF;
    state.p[2] = 7;
    _ = try emu.runFrom(gpa, &state, prog, 0, 64);
    return state.p[2];
}

test "pc14 conditions: g12 cycles by c mod 6, g11 c12 tests v even" {
    // fact 380 (g12), 464 (g11 c12); m 5 passes a through, m 0 negates.
    const vals = [_]i32{ -5, -1, 0, 1, 6 };
    for (vals) |v| {
        const a: u32 = @bitCast(v);
        for (0..12) |c| {
            var buf: [96]u8 = undefined;
            const src = try std.fmt.bufPrint(&buf, "pc14 p2, s3, #12, #{d}, #5, s0, #0, #0\nx17.halt\n", .{c});
            const want: bool = switch (c % 6) {
                0 => v >= 0,
                1 => v <= 0,
                2 => v > 0,
                3 => v < 0,
                4 => v != 0,
                else => v == 0,
            };
            try testing.expectEqual(@as(u8, @intFromBool(want)), try pc14P(src, a));
        }
        // m 0 is -a: v <= 0 flips to a >= 0
        try testing.expectEqual(@as(u8, @intFromBool(-v <= 0)), try pc14P("pc14 p2, s3, #12, #1, #0, s0, #0, #0\nx17.halt\n", a));
        try testing.expectEqual(@as(u8, @intFromBool(@mod(v, 2) == 0)), try pc14P("pc14 p2, s3, #11, #12, #2, s0, #0, #0\nx17.halt\n", a));
    }
}

test "pc14 refuses a condition no measurement covers" {
    try testing.expectError(error.UnknownForm, pc14P("pc14 p2, s3, #5, #0, #5, s0, #0, #0\nx17.halt\n", 0));
    try testing.expectError(error.UnknownForm, pc14P("pc14 p2, s3, #12, #13, #5, s0, #0, #0\nx17.halt\n", 0));
}

fn pc14Sign(m: u32, a: u32) !u8 {
    var buf: [96]u8 = undefined;
    const zero = try pc14P(try std.fmt.bufPrint(&buf, "pc14 p2, s3, #12, #5, #{d}, s0, #0, #0\nx17.halt\n", .{m}), a);
    if (zero == 1) return '0';
    const neg = try pc14P(try std.fmt.bufPrint(&buf, "pc14 p2, s3, #12, #3, #{d}, s0, #0, #0\nx17.halt\n", .{m}), a);
    return if (neg == 1) '-' else '+';
}

test "pc14 operand modes give fact 381's sign table" {
    // a = -1, 0, 1, 0x7FFF per mode S[14:12].
    const rows = [8]*const [4]u8{ "+0--", "+0--", "+0--", "0---", "++0-", "-0++", "-0++", "-0--" };
    const as = [4]u32{ 0xFFFFFF, 0, 1, 0x7FFF };
    for (rows, 0..) |row, m| for (as, 0..) |a, i| {
        try testing.expectEqual(row[i], try pc14Sign(@intCast(m), a));
    };
}

test "pc14's head target takes !cond under mux 0 and the immediate under mux 1 and 2" {
    // fact 478 (mux 0, own-pc14not), 107 (mux 1, 2).
    const gpa = testing.allocator;
    for ([_]u32{ 0, 1, 2 }) |mux| for ([_]u32{ 0x77, 0x76 }) |a| {
        var buf: [96]u8 = undefined;
        const prog = try assemble(gpa, try std.fmt.bufPrint(&buf, "pc14 p13, s7, #11, #12, #2, s7, #{d}, #0x14A3\nx17.halt\n", .{mux}));
        defer gpa.free(prog);
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[7] = a;
        _ = try emu.runFrom(gpa, &state, prog, 0, 64);
        const even: u32 = @intFromBool(a & 1 == 0);
        try testing.expectEqual(@as(u32, if (mux == 0) 1 - even else 0x14A3), state.s[7]);
        try testing.expectEqual(@as(u8, @intCast(even)), state.p[13]);
    };
}

test "an x18 branch word writes its p slot beside the branch" {
    // measured on silicon: 00000018 00000000 70006000 C0000100, branch on p7 = 0: p3 <- 1.
    for ([_]u8{ 0, 1 }) |pre| {
        const gpa = testing.allocator;
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[3] = pre;
        const prog = [_]u32{ 0x00000018, 0x00000000, 0x70006000, 0xC0000100, 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(gpa, &state, &prog, 0, 16);
        try testing.expectEqual(@as(u8, 1), state.p[3]);
    }
}

test "pc14 group 0 is measured for codes 0-7 and 9 only" {
    try testing.expectEqual(@as(u8, 1), try pc14P("pc14 p2, s3, #0, #7, #5, s0, #0, #0\nx17.halt\n", 0));
    try testing.expectError(error.UnknownForm, pc14P("pc14 p2, s3, #0, #8, #5, s0, #0, #0\nx17.halt\n", 0));
}

test "pc14 refuses head-target mux 3, untested on silicon" {
    try testing.expectError(error.UnknownForm, pc14P("pc14 p2, s3, #0, #0, #5, s6, #3, #0\nx17.halt\n", 0));
}

fn runX18(words: [4]u32, p1: u8, p5: u8) !emu.State {
    const gpa = testing.allocator;
    var state = emu.State{};
    state.p[1] = p1;
    state.p[5] = p5;
    state.s[1] = 0x123;
    state.s[2] = 0x456;
    const prog = [_]u32{ words[0], words[1], words[2], words[3], 0x00000017, 0x31308000, 0xA0000000 };
    _ = try emu.runFrom(gpa, &state, &prog, 0, 16);
    return state;
}

test "x18 class-110 p write on the multiply base: the silicon gates" {
    // fact 469: own-x18pinv also writes s2 <- lo(s1 * s1 << 1), s1 <- hi.
    var st = try runX18(.{ 0x1C000018, 0x00000000, 0x02002222, 0xC4114001 }, 1, 1);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u8, 0), st.p[1]);
    try testing.expectEqual(@as(u32, 0x029592), st.s[2]); // 469 read [15:0]; 24 bits (663)
    try testing.expectEqual(@as(u32, 0), st.s[1]);
    // 470 own-x18p1: op 2 with w3[9:8] != 0 writes 1.
    var st2 = try runX18(.{ 0x1C000018, 0x00010000, 0x02002222, 0xC4114101 }, 0, 1);
    defer st2.deinit(testing.allocator);
    try testing.expectEqual(@as(u8, 1), st2.p[1]);
    // 471 own-x18c11: code 11, f 10 writes 0.
    var st3 = try runX18(.{ 0x16000018, 0x00010000, 0x02002222, 0xD4114001 }, 1, 1);
    defer st3.deinit(testing.allocator);
    try testing.expectEqual(@as(u8, 0), st3.p[1]);
}

test "x18 class-110 p write refuses an unmeasured cell" {
    // code 11, f 4: not measured.
    try testing.expectError(error.UnknownForm, runX18(.{ 0x16000018, 0x00000000, 0x00002022, 0xC8114001 }, 0, 0));
    // op 1: 451's comparisons, not this table.
    try testing.expectError(error.UnknownForm, runX18(.{ 0x1C000018, 0x00008000, 0x00002022, 0xC4114001 }, 0, 0));
}

test "x18 class-110 p write lands only while p<w2[31:28]> is set (837)" {
    const gpa = testing.allocator;
    // x18.pnot p1, p5 with the condition field w2[31:28] = 6.
    for ([_]struct { p6: u8, p1: u8 }{ .{ .p6 = 1, .p1 = 0 }, .{ .p6 = 0, .p1 = 1 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[1] = 1;
        state.p[5] = 1;
        state.p[6] = c.p6;
        const prog = [_]u32{ 0x1C000018, 0x00000000, 0x60002022, 0xC4114001, 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(gpa, &state, &prog, 0, 16);
        try testing.expectEqual(c.p1, state.p[1]);
    }
}

test "x18.pmov copies p5 = 0 too" {
    var st = try runX18(.{ 0x1C000018, 0x00000000, 0x00002022, 0xC2114001 }, 1, 0);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u8, 0), st.p[1]);
}

fn deadlineRun(armers: []const usize, gap_after: []const usize) !void {
    // An x17.cont (an armer) then n nops, per entry, then the halt.
    const gpa = testing.allocator;
    var src: std.ArrayList(u8) = .empty;
    defer src.deinit(gpa);
    for (armers, gap_after) |_, n| {
        try src.appendSlice(gpa, "x17.cont\n");
        for (0..n) |_| try src.appendSlice(gpa, "alu s0, s0, s0, #0\n");
    }
    try src.appendSlice(gpa, "x17.halt\n");
    const prog = try assemble(gpa, src.items);
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog, 0, 256);
}

test "a bare continuing x17 lets eight nops through; nine park" {
    // fact 484 (98's six counted a two-word st pair); the latest x17 restarts it.
    try deadlineRun(&.{0}, &.{8});
    try testing.expectError(error.HaltDeadline, deadlineRun(&.{0}, &.{9}));
    try deadlineRun(&.{ 0, 0 }, &.{ 12, 2 });
    try testing.expectError(error.HaltDeadline, deadlineRun(&.{ 0, 0 }, &.{ 8, 9 }));
}

test "x02 w2[30] arms the halt deadline" {
    // fact 477, 484: 00100002 0000A008 40000224 00000000 00022810 C0024200, g nops,
    // halt: g = 7 halts, g = 8 does not.
    const gpa = testing.allocator;
    for ([_]usize{ 7, 8 }) |g| {
        var prog: std.ArrayList(u32) = .empty;
        defer prog.deinit(gpa);
        try prog.appendSlice(gpa, &.{ 0x00100002, 0x0000A008, 0x40000224, 0x00000000, 0x00022810, 0xC0024200 });
        for (0..g) |_| try prog.append(gpa, 0x40000005);
        try prog.appendSlice(gpa, &.{ 0x00000017, 0x31308000, 0xA0000000 });
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, prog.items, 0, 64);
        if (g == 7) _ = try r else try testing.expectError(error.HaltDeadline, r);
    }
}

test "x17.sty.stop stores, runs two shadow bundles and stops unhalted" {
    // fact 478: any continue-shape x17 with S1[22:20] = 010 stops after its slots.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        ldi s13, #0x6FF
        \\        x17.mve e5, s13
        \\        ldi s3, #0x42
        \\        x17.sty.stop e5, #3, #0
        \\        ldi s1, #1
        \\        ldi s2, #2
        \\        ldi s4, #4
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog, 0, 64);
    try testing.expectEqual(@as(?u32, 0x42), state.getY(0x700));
    try testing.expectEqual(@as(u32, 2), state.s[2]);
    try testing.expectEqual(@as(u32, 0), state.s[4]);
    try testing.expect(!state.halted);
}

test "x03 with w2[31:29] = 010 arms the halt deadline" {
    // fact 479; the word is x03.agu's template with w2[30] set. The 7/8 boundary is
    // x02's and x18's (484), not measured on this armer.
    const gpa = testing.allocator;
    for ([_]usize{ 7, 8 }) |g| {
        var prog: std.ArrayList(u32) = .empty;
        defer prog.deinit(gpa);
        try prog.appendSlice(gpa, &.{ 0x00000003, 0x00000002, 0x4140E000, 0x04000C0E, 0x0000683C, 0x00140102 });
        for (0..g) |_| try prog.append(gpa, 0x40000005);
        try prog.appendSlice(gpa, &.{ 0x00000017, 0x31308000, 0xA0000000 });
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, prog.items, 0, 64);
        if (g == 7) _ = try r else try testing.expectError(error.HaltDeadline, r);
    }
}

test "the deadline counts words: a two-word st pair takes two" {
    // fact 485: x18 armer, n nops, st pair, halt: n = 5 completes, n = 6 parks.
    const gpa = testing.allocator;
    for ([_]usize{ 5, 6 }) |g| {
        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        for (0..g) |_| try src.appendSlice(gpa, "alu s0, s0, s0, #0\n");
        try src.appendSlice(gpa, "st.wr #0x40, e0, s4\nx17.halt\n");
        const head = try assemble(gpa, "ldi s4, #0x700\n");
        defer gpa.free(head);
        const tail = try assemble(gpa, src.items);
        defer gpa.free(tail);
        var prog: std.ArrayList(u32) = .empty;
        defer prog.deinit(gpa);
        // the ldi sets e0 for the pair before the armer, outside the counted gap
        try prog.appendSlice(gpa, head);
        try prog.appendSlice(gpa, &.{ 0x00000018, 0x00000004, 0x0004AC00, 0x00040000 });
        try prog.appendSlice(gpa, tail);
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, prog.items, 0, 64);
        if (g == 5) _ = try r else try testing.expectError(error.HaltDeadline, r);
    }
}

test "x18 class 000 with w2[17], w2[18] and w3[7] clear is refused: it may stop" {
    // fact 485, 487: x18.eagu's template with w2[17] cleared stops, with w3[8] set too;
    // w3[7] set runs on. own-x18e2nd-w17 (180: 80800018 80028024 0000A97B 00048100) runs on
    // silicon yet no field separates it, so it is refused as well.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x00000018, 0x00000004, 0x0000AC00, 0x00040000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    var ok = emu.State{};
    defer ok.deinit(gpa);
    _ = try emu.runFrom(gpa, &ok, &.{ 0x00000018, 0x00000004, 0x0000AC00, 0x00040080, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    var img = emu.State{};
    defer img.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &img, &.{ 0x80800018, 0x80028024, 0x0000A97B, 0x00048200, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    for ([_][4]u32{ .{ 0x00000018, 0x00000004, 0x0000AC00, 0x00040100 }, .{ 0x80800018, 0x80028024, 0x0000A97B, 0x00048100 } }) |b| {
        var st = emu.State{};
        defer st.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st, &(b ++ [_]u32{ 0x00000017, 0x31308000, 0xA0000000 }), 0, 16));
    }
}

test "an x17 store with an odd e does not arm the halt deadline" {
    // fact 479: x17sty 00000017 A902C800 66001202 (S1[15] = 1) halts 12 words out.
    const gpa = testing.allocator;
    var prog: std.ArrayList(u32) = .empty;
    defer prog.deinit(gpa);
    try prog.appendSlice(gpa, &.{ 0x00000017, 0xA902C800, 0x66001202 });
    for (0..12) |_| try prog.append(gpa, 0x40000005);
    try prog.appendSlice(gpa, &.{ 0x00000017, 0x31308000, 0xA0000000 });
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog.items, 0, 64);
    try testing.expect(state.halted);
}

test "lde.arm loads e8..e15 and arms the halt deadline" {
    // fact 479: lde class 101 is a plain lde plus the arm; the 7/8 boundary is x02's
    // and x18's (484), not measured on this armer.
    const gpa = testing.allocator;
    for ([_]usize{ 7, 8 }) |g| {
        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        try src.appendSlice(gpa, "lde.arm e9, #0x123\n");
        for (0..g) |_| try src.appendSlice(gpa, "alu s0, s0, s0, #0\n");
        try src.appendSlice(gpa, "x17.halt\n");
        const prog = try assemble(gpa, src.items);
        defer gpa.free(prog);
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, prog, 0, 64);
        if (g == 7) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0x123), state.e[9]);
        } else try testing.expectError(error.HaltDeadline, r);
    }
}

test "lde.arm passes the e0 latch through" {
    // fact 480: ldi s4 <- 0x700, AC00E806, st pair: Y:0x700.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        ldi s4, #0x700
        \\        lde.arm e11, #0x740
        \\        st.wr #0x40, e0, s4
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog, 0, 64);
    try testing.expectEqual(@as(?u32, 0x700), state.getY(0x700));
}

test "x17e with S1[23:20] set is refused: it may run, arm or stop" {
    // fact 481: no field predicts which.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x00000017, 0x00160200, 0x80000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "x18 class 000 with w2[18] = 1 and w2[17] = 0 arms the halt deadline" {
    // fact 483, measured on these words in 484: 7 nops complete, 8 park; w2[17] = 1
    // cancels the arm (12 complete).
    const gpa = testing.allocator;
    for ([_]struct { w2: u32, g: usize, ok: bool }{
        .{ .w2 = 0x0004AC00, .g = 7, .ok = true },
        .{ .w2 = 0x0004AC00, .g = 8, .ok = false },
        .{ .w2 = 0x0006AC00, .g = 12, .ok = true },
    }) |c| {
        var prog: std.ArrayList(u32) = .empty;
        defer prog.deinit(gpa);
        try prog.appendSlice(gpa, &.{ 0x00000018, 0x00000004, c.w2, 0x00040000 });
        for (0..c.g) |_| try prog.append(gpa, 0x40000005);
        try prog.appendSlice(gpa, &.{ 0x00000017, 0x31308000, 0xA0000000 });
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, prog.items, 0, 64);
        if (c.ok) _ = try r else try testing.expectError(error.HaltDeadline, r);
    }
}

test "x18 class 111 X-slot muxes 3, 4 and 5 read lanes of Y:[e]" {
    // fact 486: plant 0x12AB; mux 3 gives 0xAB00, mux 4 (Y << 7)[15:0], mux 5 Y.
    const gpa = testing.allocator;
    for ([_]struct { mux: u32, want: u32 }{
        .{ .mux = 3, .want = 0xAB00 },
        .{ .mux = 4, .want = 0x5580 },
        .{ .mux = 5, .want = 0x12AB },
        .{ .mux = 6, .want = 0x0000 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try state.y_mem.put(gpa, 0x700, 0x12AB);
        state.e[3] = 0x700;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00400018, c.mux << 12, 0x00200000, 0xE6000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.s[2]);
    }
}

test "x18 class 111 mux 3 takes yd[7:0] << 8 unless w2[22:21] = 01 (fact 597)" {
    const gpa = testing.allocator;
    // Y:[e3] = 0x12B6; w2[22:21] = 01 reads it, 00 / 10 / 11 take yd.
    for ([_]struct { mux: u32 = 3, w2: u32, yd: u32, want: ?u32 }{
        .{ .w2 = 0x00000000, .yd = 0x34C9, .want = 0xC900 },
        .{ .w2 = 0x00400000, .yd = 0x7E11, .want = 0x1100 },
        .{ .w2 = 0x00600000, .yd = 0x34C9, .want = 0xC900 }, // 11 (with 197's Y store of s0)
        .{ .w2 = 0x00200000, .yd = 0x34C9, .want = 0xB600 },
        .{ .mux = 4, .w2 = 0x00000000, .yd = 0x34C9, .want = null },
        .{ .mux = 5, .w2 = 0x00400000, .yd = 0x34C9, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try state.y_mem.put(gpa, 0x700, 0x12B6);
        state.e[3] = 0x700;
        state.yd = c.yd;
        const r = emu.runFrom(gpa, &state, &.{ 0x00400018, c.mux << 12, c.w2, 0xE6000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
            // The fresh read (01) sets yd; the yd copies leave it.
            try testing.expectEqual(@as(u32, if (c.w2 == 0x00200000) 0x12B6 else c.yd), state.yd);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "pc14 class-011 mux 10 writes the immediate, and group 0 code 0 sets its p" {
    // fact 486: 461's "0 after a st.y" was a readback artifact; the immediate is
    // unconditional. 487: group 0 code 0 sets its p (measured with p6); other p cells refuse.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[1] = 5;
    // h: g = 0, s3 <- #0x1234; second: class 011, mux 10, S[25], p0 dest, c 0
    const h: u32 = 0x14 | (3 << 21) | (0x1234 << 5);
    const sw: u32 = (0b011 << 29) | (0b10 << 27) | (1 << 25) | (1 << 19) | (5 << 12);
    _ = try emu.runFrom(gpa, &state, &.{ h, sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1234), state.s[3]);
    var img = emu.State{};
    defer img.deinit(gpa);
    // s4 <- #0x2B, p6 set (DERIVED from 487's measured pair: other s dest and immediate).
    _ = try emu.runFrom(gpa, &img, &.{ 0x40800574, 0x72000060, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x2B), img.s[4]);
    try testing.expectEqual(@as(u8, 1), img.p[6]);
    var c9 = emu.State{};
    defer c9.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &c9, &.{ 0x40800574, 0x72000960, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "rimm's s write and x16 mux 3 are refused: they depend on state not yet found" {
    // fact 488: rimm's s<head[24:21]> write read 0x0700 before any st.y and 0x5000 after;
    // x16 mux 3 varies from run to run.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x0060000F, 0xA1008000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    var r = emu.State{};
    defer r.deinit(gpa);
    _ = try emu.runFrom(gpa, &r, &.{ 0x0000000F, 0xA1008000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    const prog = try assemble(gpa, "x16.mux s3, R3, #1, #0, #6, #4, #3\nx17.halt\n");
    defer gpa.free(prog);
    var m = emu.State{};
    defer m.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &m, prog, 0, 16));
}

test "x17 S2 class 101 with S1[28] = 0 loads yd through the mux" {
    // fact 490 measured mux 4 giving 0 in plain programs: that was yd (550).  Here on own-x17st's shape with S1[28] cleared and a load into s2.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[2] = 0x55;
    state.s[7] = 0x4321;
    state.e[3] = 0x720;
    try state.y_mem.put(gpa, 0x720, 0x1234);
    _ = try emu.runFrom(gpa, &state, &.{ 0x00400017, 0x67018000, 0xA0000048, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0), state.s[2]);
    try testing.expectEqual(@as(?u32, 0x4321), state.getY(0x720));
    var m2 = emu.State{};
    defer m2.deinit(gpa);
    m2.yd = 0x9234;
    _ = try emu.runFrom(gpa, &m2, &.{ 0x00400017, 0x67018000, 0xA0000044, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xFF9234), m2.s[2]); // mux 2 sign-extends yd
}

test "x18 class 111 slots read every source before any write" {
    // fact 491 (x99 presets, 2 of 2): 02c00058 00046562 00670005 e2023000. s6 <- #2
    // (X slot), s5 <- (s7 + s6) << 6 with the old s6 = 0x4340, Y:[e1] <- the old s5.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[5] = 0x5505;
    state.s[6] = 0x6606;
    state.s[7] = 0x7707;
    state.e[1] = 0x740;
    _ = try emu.runFrom(gpa, &state, &.{ 0x02C00058, 0x00046562, 0x00670005, 0xE2023000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 2), state.s[6]);
    try testing.expectEqual(@as(u32, 0x4340), state.s[5]);
    try testing.expectEqual(@as(?u32, 0x5505), state.getY(0x740));
}

// --- IMDCT: a twin of sosumi's ref/dsp/aac/imdct.zig, every operation as the model has it ---

fn imMul(a: i64, b: i64) i64 {
    return (a * b + (1 << 22)) >> 23;
}

test "x02.ainc's template leaves s2 (954) and x18.rag's steps e7" {
    // Side writes of two templates (subforms.txt): ainc's w2 multiply points at s0 (954);
    // x18.rag101's A5 write is 955's test.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\ldi s1, #0x4000
        \\ldi s2, #0x6000
        \\x02.ainc #8, #8
        \\ldi s13, #0x23FF
        \\x17.mve e7, s13
        \\x18.rag R3, A6, R2, #2
        \\x18.rag101 s0, R3, A6, R2, #2
        \\x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, prog, 0, 64);
    try testing.expectEqual(@as(u32, 0x6000), state.s[2]);
    try testing.expectEqual(@as(u32, 0x2401), state.e[7]);
}

test "x18 class 101 also writes its R slot sum to A<2*w1[3:0] + head[31]> (955)" {
    // The AAC TNS loop's rag101 (80000018 00001002 60000000 A0262980): R3 and A5 <- A12 + R5.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\x18.rag101 s0, R3, A12, R5, #2
        \\x17.halt
        \\
    );
    defer gpa.free(prog);
    try testing.expectEqualSlices(u32, &.{ 0x80000018, 0x00001002, 0x60000000, 0xA0262980 }, prog[0..4]);
    var state = emu.State{};
    defer state.deinit(gpa);
    state.r[3] = 0x1234;
    state.r[5] = 5;
    state.a[12] = 0x100;
    _ = try emu.runFrom(gpa, &state, prog, 0, 16);
    try testing.expectEqual([2]u32{ 0x105, 0x105 }, [2]u32{ state.r[3], state.a[5] });
}

test "x18 class 000 writes A<2*w1[3:0] + head[31]> <- A8 + 1 beside its e step (956)" {
    // own-x18eagu-a: the im_pre loop's eagu, A8 = 0x17FB -> 0x17FC; w1[3:0] = 5 moves the dest to A10.
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, dest: usize }{ .{ .w1 = 0x09C00004, .dest = 8 }, .{ .w1 = 0x09C00005, .dest = 10 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[8] = 0x17FB;
        state.a[10] = 0x55;
        const halt = try assemble(gpa, "x17.halt\n");
        defer gpa.free(halt);
        const prog = try std.mem.concat(gpa, u32, &.{ &.{ 0x00000018, c.w1, 0x0002AC00, 0x0E040000 }, halt });
        defer gpa.free(prog);
        _ = try emu.runFrom(gpa, &state, prog, 0, 16);
        try testing.expectEqual(@as(u32, 0x17FC), state.a[c.dest]);
    }
}

test "x18 class 000's A write loads #head[20:5] when w2[30] = 1 (351)" {
    // 352's T-phase bundle (w2[17] = w2[18] = w3[7] = 0) is refused under 485/487.
    const gpa = testing.allocator;
    const halt = try assemble(gpa, "x17.halt\n");
    defer gpa.free(halt);
    var state = emu.State{};
    defer state.deinit(gpa);
    // 351: own-x18c0a, 0042A058 C0028017 4002A25D 00000000 writes A14 <- #head[20:5] = 0x1502.
    const imm = try std.mem.concat(gpa, u32, &.{ &.{ 0x0042A058, 0xC0028017, 0x4002A25D, 0x00000000 }, halt });
    defer gpa.free(imm);
    _ = try emu.runFrom(gpa, &state, imm, 0, 16);
    try testing.expectEqual(@as(u32, 0x1502), state.a[14]);
}

test "x18 classes 110 and 111 write A<2*w1[3:0] + head[31]> (345, 344)" {
    // own-x18c6a: 80615798 00005007 0058146C C0500001 gives A15 <- #0xABC.
    // own-x18c7a: 80600018 72343004 32300006 E60481D6 gives A9 <- A9 + 1.
    const gpa = testing.allocator;
    const halt = try assemble(gpa, "x17.halt\n");
    defer gpa.free(halt);
    for ([_]struct { w: [4]u32, a: usize, want: u32 }{
        .{ .w = .{ 0x80615798, 0x00005007, 0x0058146C, 0xC0500001 }, .a = 15, .want = 0xABC },
        .{ .w = .{ 0x80600018, 0x72343004, 0x32300006, 0xE60481D6 }, .a = 9, .want = 0x201 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x200;
        const prog = try std.mem.concat(gpa, u32, &.{ &c.w, halt });
        defer gpa.free(prog);
        _ = try emu.runFrom(gpa, &state, prog, 0, 16);
        try testing.expectEqual(c.want, state.a[c.a]);
    }
}

test "the 0x01 ALU slot is the class-05 adder with a = s<w6[23:20]> and m = w5[29:27]" {
    // fact 498-500: own-x01am (m1, b - a = 0x23), own-x01neg (m6 with a = s0: -b), m2
    // passes b; w5[3:2] = 10 is refused.
    const gpa = testing.allocator;
    for ([_]struct { w5: u32, w6: u32, want: ?u32 }{
        .{ .w5 = 0x08200900, .w6 = 0x00300090, .want = 0x23 },
        .{ .w5 = 0x30200900, .w6 = 0x00000090, .want = 0xFFFEDD },
        .{ .w5 = 0x10200900, .w6 = 0x00000090, .want = 0x123 },
        .{ .w5 = 0x08200908, .w6 = 0x00300090, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x100;
        state.s[8] = 0x123;
        const r = emu.runFrom(gpa, &state, &.{
            0x280011C1, 0x00000580, 0x28000020, 0xA00280A0, 0x00040002, // w4[12:9] cleared (669)
            c.w5,       c.w6,       0x00000017, 0x31308000, 0xA0000000,
        }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[9] & 0xFFFFFF);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "ldi's head[29:27] picks the source: 0 and 7 load 0, 1/2/6 the immediate, 3-5 refused" {
    // fact 501: ldi s5, #0xFFF8 with k = 0..7 loaded 0, FFF8, FFF8, D lanes, FFF8, 0.
    const gpa = testing.allocator;
    for ([_]struct { k: u32, want: ?u32 }{
        .{ .k = 0, .want = 0 },        .{ .k = 1, .want = 0xFFFFF8 }, .{ .k = 2, .want = 0xFFFFF8 },
        .{ .k = 3, .want = null },     .{ .k = 4, .want = null },     .{ .k = 5, .want = null },
        .{ .k = 6, .want = 0xFFFFF8 }, .{ .k = 7, .want = 0 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x55;
        const r = emu.runFrom(gpa, &state, &.{ 0xC2BFFF0B | (c.k << 27), 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // x17 assembles k = 1
    const prog = try assemble(gpa, "ldi s5, #-8\n");
    defer gpa.free(prog);
    try testing.expectEqual(@as(u32, 0xCABFFF0B), prog[0]);
}

test "the 0x01 slot's xori and asr codes" {
    // fact 502-506: own-x01norm, own-x01asr, own-x01asrneg, own-x01asrwrap, sweeps.
    const gpa = testing.allocator;
    for ([_]struct { code: u32, i: u32, m: u32, a: u32, b: u32, want: ?u32 }{
        .{ .code = 0x1C, .i = 0, .m = 5, .a = 0, .b = 0x100, .want = 14 },
        .{ .code = 0x1C, .i = 0, .m = 5, .a = 0, .b = 0xFF8808, .want = 8 },
        .{ .code = 0x1C, .i = 0, .m = 5, .a = 0, .b = 0, .want = 24 },
        .{ .code = 0x1C, .i = 3, .m = 0, .a = 7, .b = 0x123, .want = 0x120 },
        .{ .code = 0x1C, .i = 3, .m = 7, .a = 0, .b = 0x100, .want = 3 },
        .{ .code = 0x1C, .i = 3, .m = 7, .a = 0, .b = 0xFFFF00, .want = 0xFFFF00 },
        .{ .code = 0x06, .i = 3, .m = 0, .a = 1, .b = 0x4000, .want = 0x2001 },
        .{ .code = 0x06, .i = 3, .m = 0, .a = 0xFFFFFE, .b = 0x100, .want = 0x40C },
        .{ .code = 0x06, .i = 0, .m = 4, .a = 9, .b = 0xFF8000, .want = 0xFFFFBF },
        .{ .code = 0x06, .i = 0, .m = 6, .a = 9, .b = 0xFF8000, .want = 0x40 },
        .{ .code = 0x06, .i = 3, .m = 7, .a = 0, .b = 0x100, .want = 0x100 },
        .{ .code = 0x06, .i = 3, .m = 7, .a = 1, .b = 0x100, .want = 0x80 },
        .{ .code = 0x06, .i = 0, .m = 7, .a = 9, .b = 0xFF8000, .want = 0x40 },
        .{ .code = 0x06, .i = 0, .m = 0, .a = 24, .b = 0x4000, .want = 0 },
        .{ .code = 0x06, .i = 0, .m = 0, .a = 30, .b = 0xFF8000, .want = 0xFFFFFF },
        .{ .code = 0x06, .i = 0, .m = 0, .a = 0xFFFFF6, .b = 0x4000, .want = 0 },
        .{ .code = 0x06, .i = 0, .m = 0, .a = 0xFFFFF6, .b = 0xFF8000, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = c.a;
        state.s[8] = c.b;
        const head = (0x280011C1 & ~@as(u32, 0x3F << 5)) | (c.code << 5);
        const r = emu.runFrom(gpa, &state, &.{ head, 0x00000580, 0x28000020 | (c.i << 6), 0xA00280A0, 0x00040002, 0x00200900 | (c.m << 27), 0x00300090, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[9] & 0xFFFFFF);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x01 R slot loads R from Y:[e<w4[8:5]>], Y:[0] at index 0" {
    // fact 425, 505: own-x01ry (index 5) and own-x01ry0; e0 points elsewhere, so
    // index 0 is the address 0, not e0.
    const gpa = testing.allocator;
    const N: u32 = 0x40000005;
    for ([_]struct { w4: u32, w5: u32, want: ?u32 }{
        .{ .w4 = 0x00000000, .w5 = 0x00001800, .want = 0x1357 },
        .{ .w4 = 0x000000A0, .w5 = 0x00001800, .want = 0x2468 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[0] = 0x720;
        state.e[5] = 0x710;
        try state.y_mem.put(gpa, 0, 0x1357);
        try state.y_mem.put(gpa, 0x710, 0x2468);
        try state.y_mem.put(gpa, 0x720, 0x7777);
        const r = emu.runFrom(gpa, &state, &.{ 0x882151C1, 0x00000541, 0x0380C0C0, 0x40000000, c.w4, c.w5, 0x00000000, N, N, N, N, N, 0x44024007, N, N, N, 0x00000017, 0x31308000, 0xA0000000 }, 0, 64);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[1]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x01 R slot's AGU10 form: R<w3[31:29]> <- fn(R<head[30:27]>, head[26:11])" {
    // fact 320: own-x01ragu (head[31] = 1, y = R0, i = 0x700, fn 10 gives R1 = 0x700),
    // then y = R5 under fns 11, 8, 14, and an unmeasured fn refused.
    const gpa = testing.allocator;
    for ([_]struct { head: u32, fn_op: u32, w4: u32 = 0, dest: u32 = 1, want: ?u32 }{
        .{ .head = 0x80380001, .fn_op = 10, .want = 0x700 },
        .{ .head = 0xA8380001, .fn_op = 10, .want = 0x1700 },
        .{ .head = 0xA8380001, .fn_op = 11, .want = 0x0900 },
        .{ .head = 0xA8380001, .fn_op = 8, .want = 0x700 },
        .{ .head = 0xA8380001, .fn_op = 14, .want = 0x1000 },
        .{ .head = 0xC8380001, .fn_op = 14, .want = 0x2000 }, // y = R9
        .{ .head = 0xA8380001, .fn_op = 14, .w4 = 1, .dest = 9, .want = 0x1000 }, // 321: w4[0]
        .{ .head = 0xA8380001, .fn_op = 6, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[5] = 0x1000;
        state.r[9] = 0x2000;
        const r = emu.runFrom(gpa, &state, &.{ c.head, c.fn_op << 7, 0x00000000, 0x20000000, c.w4, 0x00200000, 0x00000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[c.dest]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "0x01 multiply mode 4 and the s slot's AGU10 rows" {
    // fact 507: own-x01m4w22 (w4[22] = 1, s8 = s11 = 0x4000 gives s8 = 0x20);
    // 508: own-x01ty (R7 = 0x100, head i = 3, op 11 gives s1 = 0xFD), op 10 the sum.
    const gpa = testing.allocator;
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[8] = 0x4000;
        state.s[11] = 0x4000;
        _ = try emu.runFrom(gpa, &state, &(.{ 0x28001001, 0x0000A580, 0x2800C000, 0xA00100E0, 0x00FE000C, 0x40200900, 0xB8080000 } ++ halt), 0, 16);
        try testing.expectEqual(@as(u32, 0x20), state.s[8]);
        // rounded (496): 0x4000 * 0x4100 = 2^28 + 2^22 gives 0x21
        state.s[8] = 0x4000;
        state.s[11] = 0x4100;
        state.pc = 0;
        state.halted = false;
        _ = try emu.runFrom(gpa, &state, &(.{ 0x28001001, 0x0000A580, 0x2800C000, 0xA00100E0, 0x00FE000C, 0x40200900, 0xB8080000 } ++ halt), 0, 16);
        try testing.expectEqual(@as(u32, 0x21), state.s[8]);
    }
    for ([_]struct { w1: u32, want: u32 }{ .{ .w1 = 0x0002A585, .want = 0xFD }, .{ .w1 = 0x0002A505, .want = 0x103 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[7] = 0x100;
        _ = try emu.runFrom(gpa, &state, &(.{ 0x38001801, c.w1, 0xE8010000, 0x00000020, 0x00000000, 0xC3000910, 0x35001002 } ++ halt), 0, 16);
        try testing.expectEqual(c.want, state.s[1]);
    }
}

test "the 0x01 s slot's sub-mux 10: AGU10 under 509's w5, 234's table under own-x01st's" {
    // fact 509 base (R5 = 0x1000, e1 = 0x100, e2 = 0x10, i = 2), 16 of 16 (own-x01agu:
    // t = 7 gives 0xF100); with own-x01st's w5 01003600, t = 2 gives b = 0x10.  w3's R slot
    // (dest R5, same fn field) is cleared: its fns past 8/10/11/14 are unmodelled.
    const gpa = testing.allocator;
    const want = [16]u32{ 0x0100, 0x1000, 0x0101, 0x1001, 0x00FF, 0x0FFF, 0x1100, 0xF100, 0x0002, 0x0000, 0x1002, 0x0FFE, 0x1000, 0x1100, 0x1000, 0x1000 };
    for (0..17) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[5] = 0x1000;
        state.e[1] = 0x100;
        state.e[2] = 0x10;
        const t: u32 = if (k == 16) 2 else @intCast(k);
        const w5: u32 = if (k == 16) 0x01003600 else 0x31200F00;
        _ = try emu.runFrom(gpa, &state, &.{ 0x280011E1, t << 7, 0xA8080804, 0x00300060, 0x00800024, w5, 0x00002012, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(if (k == 16) 0x10 else want[k], state.s[2]);
    }
    // 512 (own-x01tabsel): w5[11] alone picks; clearing 29/28/21/8 keeps AGU10, 11 gives b
    for ([_]struct { w5: u32, want: u32 }{
        .{ .w5 = 0x01000F00, .want = 0x0101 },
        .{ .w5 = 0x31200700, .want = 0x0010 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[5] = 0x1000;
        state.e[1] = 0x100;
        state.e[2] = 0x10;
        _ = try emu.runFrom(gpa, &state, &.{ 0x280011E1, 2 << 7, 0xA8080804, 0x00300060, 0x00800024, c.w5, 0x00002012, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.s[2]);
    }
}

test "0x00's s-slot sub-mux 10 is 234's table" {
    // fact 510: own-x00tab (e1 = 0x100, t = 3 gives -a = 0xFF00).
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0x100;
    _ = try emu.runFrom(gpa, &state, &.{ 0x28001000, 0x0002A005 | (3 << 7), 0xE8010000, 0x00000020, 0x00000020, 0xC3000900, 0x35001002, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xFF00), state.s[1]);
}

test "0x02's X store and immediate/yd R source" {
    // fact 513: own-x02rimm (R2 <- head[26:11] = 0x109F with w5[31:30] = 11); 00 takes
    // yd, 01 is refused; w5[10:9] = 11 stores s<{w2[0], w1[31:29]}> (s3 here) at X:[A11].
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, w5: u32, r2: ?u32, xst: bool }{
        .{ .w1 = 0x00000008, .w5 = 0xC0024204, .r2 = 0x109F, .xst = false },
        .{ .w1 = 0x00000008, .w5 = 0x00024204, .r2 = 0x4321, .xst = false },
        .{ .w1 = 0x00000008, .w5 = 0x40024204, .r2 = null, .xst = false },
        .{ .w1 = 0x60000008, .w5 = 0xC0024604, .r2 = 0x109F, .xst = true },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.yd = 0x4321;
        state.a[11] = 0x300;
        state.s[3] = 0x123456;
        const r = emu.runFrom(gpa, &state, &.{ 0x0084FC42, c.w1, 0x00000000, 0x00100000, 0x02562848, c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.r2) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[2]);
            try testing.expectEqual(@as(?u32, if (c.xst) 0x123456 else null), state.readX(0x300));
        } else try testing.expectError(error.UnknownForm, r);
    }
}

fn stDown(w: i64, n: i64) i64 {
    return if (n >= 24) 0 else rshr(w, @intCast(n));
}

fn stSpectral(b: i64) bool {
    return b >= 1 and b <= 11;
}

/// sosumi ref/dsp/aac/stereo.zig `applyPair` over the same Y lists the routine reads.
const StPair = struct {
    groups: usize,
    max_sfb: usize,
    win: usize,
    glen: [8]usize,
    msp: i64,
    off: [64]i64,
    lb: [128]i64,
    rb: [128]i64,
    rs: [128]i64,
    msu: [128]i64,
    l: [1024]i64,
    r: [1024]i64,
    el: i64,
    er: i64,

    fn kind(p: *const StPair, i: usize) u2 {
        if (p.rb[i] >= 14) {
            var neg = p.rb[i] == 14;
            if (p.msp == 1 and p.msu[i] != 0) neg = !neg;
            return if (neg) 3 else 2;
        }
        if (p.msp != 0 and p.msu[i] != 0 and p.lb[i] < 13 and p.rb[i] < 13) return 1;
        return 0;
    }

    fn eachRun(p: *const StPair, ctx: anytype, comptime f: anytype) void {
        var wb: usize = 0;
        for (0..p.groups) |g| {
            for (0..p.max_sfb) |sfb| {
                for (0..p.glen[g]) |w| {
                    const start = wb + w * p.win + @as(usize, @intCast(p.off[sfb]));
                    const len: usize = @intCast(p.off[sfb + 1] - p.off[sfb]);
                    f(ctx, start, len, g * p.max_sfb + sfb);
                }
            }
            wb += p.glen[g] * p.win;
        }
    }

    fn move(w: *[1024]i64, e: *i64, to: i64, live: bool) void {
        if (live) for (w) |*x| {
            x.* = stDown(x.*, to - e.*);
        };
        e.* = to;
    }

    fn apply(p: *StPair) void {
        const nb = p.groups * p.max_sfb;
        var ll = false;
        var rl = false;
        var ams = false;
        var ais = false;
        for (0..nb) |i| {
            ll = ll or stSpectral(p.lb[i]);
            rl = rl or stSpectral(p.rb[i]);
            ams = ams or p.kind(i) == 1;
            ais = ais or p.kind(i) >= 2;
        }
        if (ams and (ll or rl)) {
            var to: i64 = if (ll) p.el else p.er;
            if (rl and p.er > to) to = p.er;
            to += 1;
            move(&p.l, &p.el, to, ll);
            move(&p.r, &p.er, to, rl);
            rl = rl or ll;
            const Ms = struct {
                fn f(q: *StPair, start: usize, len: usize, band: usize) void {
                    if (q.kind(band) != 1) return;
                    for (start..start + len) |k| {
                        const m = q.l[k];
                        q.l[k] = m + q.r[k];
                        q.r[k] = m - q.r[k];
                    }
                }
            };
            p.eachRun(p, Ms.f);
        }
        if (ll and ais) {
            var to: ?i64 = if (rl) p.er else null;
            for (0..nb) |i| {
                if (p.kind(i) < 2) continue;
                const e = p.el + 1 - (p.rs[i] >> 2);
                if (to == null or e > to.?) to = e;
            }
            move(&p.r, &p.er, to.?, rl);
            const Is = struct {
                fn f(q: *StPair, start: usize, len: usize, band: usize) void {
                    const kd = q.kind(band);
                    if (kd < 2) return;
                    const pos = q.rs[band];
                    const qq = pos >> 2;
                    const t: i64 = @intFromFloat(@round(std.math.pow(f64, 2.0, 22.0 - @as(f64, @floatFromInt(pos - 4 * qq)) / 4.0)));
                    const n = q.er - (q.el + 1 - qq);
                    for (start..start + len) |k| {
                        const v = stDown(imMul(q.l[k], t), n);
                        q.r[k] = if (kd == 3) -v else v;
                    }
                }
            };
            p.eachRun(p, Is.f);
        }
    }
};

test "ystm: AGU10B or AGU10 by s[20] with the store on, a fresh Y read at s[17:16] = 10" {
    // fact 514: own-ystmB (e1 = 0x700, fn 3: AGU10B -x = 0xF900, AGU10 y + 1 = 1); with
    // h[26] = 0, s[17:16] = 10 reads Y:[e1] and s[17] = 0 keeps the stale latch.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, sw: u32, want: u32 }{
        .{ .h = 0x04060230, .sw = 0x44073000, .want = 0xF900 },
        .{ .h = 0x04060230, .sw = 0x44173000, .want = 0x0001 },
        .{ .h = 0x00060230, .sw = 0x44063000, .want = 0x1234 },
        .{ .h = 0x00060230, .sw = 0x44053000, .want = 0x5555 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x700;
        state.yd = 0x5555;
        try state.y_mem.put(gpa, 0x700, 0x1234);
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.r[2]);
    }
}

test "the 0x00 R slot: AGU10, the immediate, zero, the product's low half, or Y" {
    // fact 516: own-x00rimm (R2 = 0x280) and the other sources of the same slot.  The Y
    // source is fresh only with w5[4]; without it, yd (550).
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w1: u32, w4: u32, w5: u32, w6: u32, want: u32 }{
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0, .w5 = 0x06000000, .w6 = 0, .want = 0x280 },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0, .w5 = 0x04000000, .w6 = 0, .want = 0 },
        .{ .h = 0x00140000, .w1 = 0x2000, .w4 = 0, .w5 = 0x02000000, .w6 = 0x43000000, .want = (0x1234 * 0x0567 * 2) & 0xFFFF },
        .{ .h = 0x00140000, .w1 = 0x22000, .w4 = 0, .w5 = 0x02000000, .w6 = 0x43000000, .want = (0x1234 * 0x0567) & 0xFFFF },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0x000000A0, .w5 = 0x02000000, .w6 = 0x43000000, .want = 0xBEE },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0x000000A0, .w5 = 0, .w6 = 0, .want = 0xBEE },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0x000000A0, .w5 = 0x10, .w6 = 0, .want = 0x2468 },
        .{ .h = 0x08140000, .w1 = 0x500, .w4 = 0, .w5 = 0x00200000, .w6 = 0, .want = 0x380 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x710;
        state.r[1] = 0x100;
        state.s[3] = 0x1234;
        state.s[4] = 0x0567;
        state.yd = 0xBEE;
        try state.y_mem.put(gpa, 0x710, 0x2468);
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.w1, 0, 0x40000000, c.w4, c.w5, c.w6, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.r[2]);
    }
}

test "the 0x16 class-100 row's s ALU: shifts, masked shift, add and subtract by n" {
    // fact 517: own-b16shl (s3 = 0x101, n = 7: s5 = 0x8080), then cells of the grid at
    // a = 0x3303, n = 7 by (head[29:27], S2[2:0]); unmeasured cells refuse.
    const gpa = testing.allocator;
    for ([_]struct { s3: u32, mux: u32, code: u32, want: ?u32 }{
        .{ .s3 = 0x101, .mux = 0, .code = 0, .want = 0x8080 },
        .{ .s3 = 0x3303, .mux = 0, .code = 4, .want = 0x3303 << 7 },
        .{ .s3 = 0x3303, .mux = 0, .code = 1, .want = 0x818000 }, // 589: a << (n + 8), 24-bit wrap ASSUMED
        .{ .s3 = 0x3303, .mux = 1, .code = 2, .want = 0x3303 >> 7 },
        .{ .s3 = 0x3303, .mux = 2, .code = 1, .want = 0x3303 + 7 },
        .{ .s3 = 0x3303, .mux = 2, .code = 2, .want = 0x3303 - 7 },
        .{ .s3 = 0x3303, .mux = 2, .code = 5, .want = 0x3303 },
        .{ .s3 = 0x3303, .mux = 2, .code = 6, .want = 0x3304 },
        .{ .s3 = 0x3303, .mux = 0, .code = 6, .want = null },
        .{ .s3 = 0x3303, .mux = 1, .code = 1, .want = null },
        .{ .s3 = 0x3303, .mux = 3, .code = 0, .want = null },
        // 518: arithmetic >>, wrapping << and -
        .{ .s3 = 0xFF8000, .mux = 1, .code = 0, .want = 0xFFFF00 },
        .{ .s3 = 0xFF8000, .mux = 0, .code = 0, .want = 0xC00000 },
        .{ .s3 = 0x7FFF, .mux = 0, .code = 0, .want = 0x3FFF80 },
        .{ .s3 = 3, .mux = 2, .code = 2, .want = 0xFFFFFC },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = c.s3;
        const h: u32 = (0x801211d6 & ~@as(u32, 7 << 27)) | c.mux << 27;
        const r = emu.runFrom(gpa, &state, &.{ h, 0xeb000007, 0x86000000 | c.code, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // 518: own-b16h31 (head[31] = 0 runs the same ALU).  Outside the gate X16_EXTRA holds only
    // at head[26:25] = 11, where 585's mx 0 c 0 with n = 7 is (a + b) >> n.
    for ([_]struct { h: u32, s2: u32, want: ?u32 }{
        .{ .h = 0x001211d6, .s2 = 0x86000000, .want = 0x8080 },
        .{ .h = 0x861211d6, .s2 = 0x86000040, .want = 2 }, // (0x101 + s4 = 0) >> 7
        .{ .h = 0x861211d6, .s2 = 0x86000080, .want = 2 },
        .{ .h = 0x861211d6, .s2 = 0x86000081, .want = null },
        .{ .h = 0x801211d6, .s2 = 0x86000040, .want = null }, // head[26:25] = 00: OPEN
        .{ .h = 0x861211d6, .s2 = 0x86000000, .want = null }, // gate cell at 11: rows move
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x101;
        state.s[5] = 0x5555;
        const r = emu.runFrom(gpa, &state, &.{ c.h, 0xeb000007, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x03 R slot's Y read, and the mux-2 e source by w1[8]" {
    // fact 519: own-x03ry (Y:0x700 = 0x1234, e1 = 0x700: R2 = 0x1234); w5[21] = 0 leaves R2;
    // the open shapes (w5[20], index 0) refuse.
    const gpa = testing.allocator;
    for ([_]struct { w3: u32, w5: u32, want: ?u32 }{
        .{ .w3 = 0x08100000, .w5 = 0x00218000, .want = 0x1234 },
        .{ .w3 = 0x08100000, .w5 = 0x00018000, .want = 0x5555 },
        .{ .w3 = 0x08100000, .w5 = 0x00318000, .want = null },
        .{ .w3 = 0x00100000, .w5 = 0x00218000, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x700;
        state.r[2] = 0x5555;
        try state.y_mem.put(gpa, 0x700, 0x1234);
        const r = emu.runFrom(gpa, &state, &.{ 0x00000003, 0x08500000, 0, c.w3, 0, c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // mux 2 (w4[13:11] = 010), w5[29:28] = 10: s5 <- e<w3[30:27]> = e1, or e<w3[26:23]> = e3 with w1[8]
    for ([_]struct { w1: u32, want: u32 }{ .{ .w1 = 0, .want = 0x111 }, .{ .w1 = 0x100, .want = 0x333 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x111;
        state.e[3] = 0x333;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000003, c.w1, 0, 0x09800000, 0x00001028, 0x20000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.s[5]);
    }
    // w1[8] with w3[26:23] = 0: silicon read 0x3422, not e0; refused
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x00000003, 0x100, 0, 0x08000000, 0x00001028, 0x20000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

/// sosumi ref/dsp/aac/tns.zig over the same parsed fields the routine reads.
const TnsCase = struct {
    nwin: usize,
    wlen: usize,
    nswb: u32,
    top: u32,
    maxo: u32,
    off: [64]i64,
    rec: std.ArrayList(i64),
    w: [1024]i64,
    e: i64,
    halvings: u32 = 0,
    gave_up: u32 = 0,

    fn small(v: i64) bool {
        return v > -(1 << 22) and v < (1 << 22);
    }

    fn lattice(w: []i64, k: []const i64, backwards: bool) bool {
        var b: [12]i64 = @splat(0);
        for (0..w.len) |step| {
            const n = if (backwards) w.len - 1 - step else step;
            var f = w[n];
            if (!small(f)) return false;
            var m = k.len;
            while (m > 0) : (m -= 1) {
                f -= imMul(k[m - 1], b[m - 1]);
                if (!small(f)) return false;
                if (m < k.len) {
                    b[m] = imMul(k[m - 1], f) + b[m - 1];
                    if (!small(b[m])) return false;
                }
            }
            b[0] = f;
            w[n] = f;
        }
        return true;
    }

    fn apply(c: *TnsCase, refl: *const [24]i64) void {
        var r: usize = 0;
        const rec = c.rec.items;
        for (0..c.nwin) |wi| {
            const nf: usize = @intCast(rec[r]);
            r += 1;
            if (nf == 0) continue;
            const res = rec[r];
            r += 1;
            var top: u32 = c.nswb;
            for (0..nf) |_| {
                const len: u32 = @intCast(rec[r]);
                const order: usize = @intCast(rec[r + 1]);
                r += 2;
                const bottom = top -| len;
                defer top = bottom;
                var k: [12]i64 = undefined;
                var dir = false;
                const ord = @min(order, c.maxo);
                if (order > 0) {
                    dir = rec[r] != 0;
                    r += 1;
                    for (0..order) |i| {
                        if (i < ord) k[i] = refl[@intCast(rec[r] + (if (res == 1) @as(i64, 16) else 4))];
                        r += 1;
                    }
                }
                if (ord == 0) continue;
                const start: usize = wi * c.wlen + @as(usize, @intCast(c.off[@min(bottom, c.top)]));
                const end: usize = wi * c.wlen + @as(usize, @intCast(c.off[@min(top, c.top)]));
                if (end <= start) continue;
                const run = c.w[start..end];
                var saved: [1024]i64 = undefined;
                @memcpy(saved[0..run.len], run);
                var tries: u32 = 0;
                while (!lattice(run, k[0..ord], dir)) : (tries += 1) {
                    @memcpy(run, saved[0..run.len]);
                    if (tries == 48) {
                        @memset(run, 0);
                        c.gave_up += 1;
                        break;
                    }
                    for (&c.w) |*x| x.* = rshr(x.*, 1);
                    for (saved[0..run.len]) |*x| x.* = rshr(x.*, 1);
                    c.e += 1;
                    c.halvings += 1;
                }
            }
        }
    }
};

test "rop picks AGU10 by head[21]; x16alu mux 1/5 reads Y by S2[18]; x19mac takes w2 class 100" {
    // fact 521: own-ropagu (e3 = 0x700, fn 2: AGU10 x + 1 = 0x701, AGU10B 0)
    const gpa = testing.allocator;
    for ([_]struct { h: u32, want: u32 }{ .{ .h = 0x40a40667, .want = 0x701 }, .{ .h = 0x40840667, .want = 0 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[3] = 0x700;
        state.r[2] = 0x5555;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.r[2]);
    }
    // x16alu (S1 class 000, S2 class 111): mux 1 and 5 with S2[18] read Y:[e2] into s<head[24:21]> = s5
    for ([_]u32{ 1, 5 }) |mux| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x710;
        state.yd = 0x1111;
        try state.y_mem.put(gpa, 0x710, 0x2468);
        const h: u32 = 0x00000016 | 5 << 21;
        const s1: u32 = mux << 12 | 5 << 16; // op 5: no ALU write
        const s2: u32 = 0xE0000000 | 2 << 21 | 1 << 18;
        _ = try emu.runFrom(gpa, &state, &.{ h, s1, s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x2468), state.s[5]);
    }
    // x19mac, w2 class 100 as 001: s5 = rnd(s3 * s4 / 2^23); with w3 class 101 refused
    for ([_]struct { w2c: u32, w3c: u32, want: ?u32 }{
        .{ .w2c = 1, .w3c = 6, .want = 0x100000 },
        .{ .w2c = 4, .w3c = 6, .want = 0x100000 },
        .{ .w2c = 4, .w3c = 5, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x200000;
        state.s[4] = 0x400000;
        const w1: u32 = 4 << 11 | 1 << 15;
        const w2: u32 = c.w2c << 29 | 5 << 9 | 3 << 5 | 4 << 1;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000019, w1, w2, c.w3c << 29, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // the two wedging rows (w2 = 0x80C80000) refuse
    for ([_][4]u32{ .{ 0x80000019, 0xffe80002, 0x80c80000, 0xc0050003 }, .{ 0x00000019, 0x3fe80001, 0x80c80000, 0xc0020003 } }) |w| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ w[0], w[1], w[2], w[3], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
}

fn pnWrap(v: i64) i64 {
    return @as(i64, @as(i24, @truncate(v)));
}

fn pnT(b: usize) i64 {
    return @intFromFloat(@round(std.math.pow(f64, 2.0, 22.0 + @as(f64, @floatFromInt(b)) / 4.0)));
}

fn pnSeed(i: usize) i64 {
    return @intFromFloat(@round(0x1p23 / (4.0 * @sqrt((@as(f64, @floatFromInt(i)) + 2.5) / 8.0))));
}

/// sosumi ref/dsp/aac/pns.zig noiseBand.
fn pnNoise(x: *i64, out: []i64, nrg: i64) ?i64 {
    var s: i64 = 0;
    var cap: i64 = 4;
    while (@as(i64, @intCast(out.len)) >= cap) {
        cap <<= 2;
        s += 2;
    }
    var big: i64 = 0;
    for (out) |*o| {
        x.* = pnWrap(0x7655 * x.* + 0x3C6EF3);
        o.* = rshr(x.*, 1);
        big = @max(big, @as(i64, @intCast(@abs(o.*))));
    }
    if (big == 0) return null;
    var k: u6 = 0;
    while (big < 1 << 21) {
        big <<= 1;
        k += 1;
    }
    var acc: i64 = 0;
    for (out) |*o| {
        o.* <<= k;
        acc += rshr(imMul(o.*, o.*), @intCast(s));
    }
    if (acc == 0) {
        @memset(out, 0);
        return null;
    }
    var p: i64 = 0;
    while (acc < 1 << 21) {
        acc <<= 2;
        p += 1;
    }
    var z = pnSeed(@intCast((acc >> 20) - 2));
    for (0..3) |_| {
        const u = imMul(acc, imMul(z, z));
        z = imMul(z, (3 << 20) - 2 * u) << 2;
    }
    const q = nrg >> 2;
    const g = imMul(z, pnT(@intCast(nrg - 4 * q)));
    for (out) |*o| o.* = imMul(o.*, g);
    return p + q - 20 - @divExact(s, 2);
}

fn pnDown(w: i64, n: i64) i64 {
    return if (n >= 24) 0 else rshr(w, @intCast(n));
}

const PnCase = struct {
    mode: i64,
    groups: usize,
    max_sfb: usize,
    win: usize,
    glen: [8]usize,
    msp: i64,
    off: [64]i64,
    book: [128]i64,
    sf: [128]i64,
    lbook: [128]i64,
    lsf: [128]i64,
    msu: [128]i64,
    w: [1024]i64,
    l: [1024]i64,
    e: i64,
    el: i64,
    x: i64,

    fn apply(c: *PnCase) void {
        var scr: [1024]i64 = @splat(0);
        var runs: [256]struct { a: usize, n: usize, e: ?i64 } = undefined;
        var nr: usize = 0;
        var top: ?i64 = null;
        var wb: usize = 0;
        for (0..c.groups) |g| {
            for (0..c.max_sfb) |sfb| {
                const band = g * c.max_sfb + sfb;
                for (0..c.glen[g]) |w| {
                    const a = wb + w * c.win + @as(usize, @intCast(c.off[sfb]));
                    const n: usize = @intCast(c.off[sfb + 1] - c.off[sfb]);
                    var e: ?i64 = null;
                    if (c.book[band] == 13) {
                        if (c.mode == 1 and c.msp != 0 and c.msu[band] != 0 and c.lbook[band] == 13) {
                            const d = c.sf[band] - c.lsf[band];
                            const q = d >> 2;
                            const t = pnT(@intCast(d - 4 * q));
                            for (a..a + n) |k| scr[k] = imMul(c.l[k], t);
                            e = c.el + q + 1;
                        } else e = pnNoise(&c.x, scr[a .. a + n], c.sf[band]);
                    }
                    runs[nr] = .{ .a = a, .n = n, .e = e };
                    nr += 1;
                    if (e) |x| top = if (top) |t| @max(t, x) else x;
                }
            }
            wb += c.glen[g] * c.win;
        }
        const t = top orelse return;
        var live = false;
        for (c.book[0 .. c.groups * c.max_sfb]) |b| live = live or (b >= 1 and b <= 11);
        var to = t;
        if (live and to < c.e) to = c.e;
        if (live) for (&c.w) |*x| {
            x.* = pnDown(x.*, to - c.e);
        };
        c.e = to;
        for (runs[0..nr]) |r| if (r.e) |e| {
            for (r.a..r.a + r.n) |k| c.w[k] = pnDown(scr[k], to - e);
        };
    }
};

test "aset12 with S[10:8] = 101 also stores s at X:[A], the address read before the A write" {
    // fact 522: X:[A<S[19:15]>] <- sx24 s<S[7:4]>; source index 0 is OPEN.
    const gpa = testing.allocator;
    for ([_]struct { xa: u32, src: u32, want: ?u32 }{
        .{ .xa = 10, .src = 7, .want = 0x300 },
        .{ .xa = 17, .src = 7, .want = 0x500 },
        .{ .xa = 10, .src = 0, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[10] = 0x300;
        state.a[17] = 0x500;
        state.r[3] = 0x345;
        state.s[7] = 0x823456;
        const s1: u32 = 0xA0000000 | c.xa << 15 | 5 << 8 | c.src << 4 | 5;
        const r = emu.runFrom(gpa, &state, &.{ 0x02C00012 | 3 << 13, s1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |addr| {
            _ = try r;
            try testing.expectEqual(@as(?u32, 0x823456), state.readX(addr));
            try testing.expectEqual(@as(u32, 0x345), state.a[10]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x04 R slot: the Y read, or the product's low half with w5[30] in mode 4" {
    // fact 522: own-x04yd's R value is the w5[25] Y read; w5[30] = 1 with w1[13:11] = 4
    // gives (s<w2[8:5]> * s<w2[4:1]>)[15:0], doubled unless w1[17].
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, w5: u32, want: ?u32 }{
        .{ .w1 = 0x582, .w5 = 0x32060000, .want = 0x1357 },
        .{ .w1 = 0x582 | 4 << 11, .w5 = 0x70060000, .want = (0x1234 * 0x0567 * 2) & 0xFFFF },
        .{ .w1 = 0x582 | 4 << 11 | 1 << 17, .w5 = 0x70060000, .want = (0x1234 * 0x0567) & 0xFFFF },
        .{ .w1 = 0x582 | 3 << 11, .w5 = 0x70060000, .want = 0x1357 },
        .{ .w1 = 0x582, .w5 = 0x70060000, .want = 0x1357 },
        .{ .w1 = 0x582 | 4 << 11, .w5 = 0x72060000, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[4] = 0x700;
        state.s[3] = 0x1234;
        state.s[4] = 0x0567;
        try state.y_mem.put(gpa, 0x700, 0x1357);
        const r = emu.runFrom(gpa, &state, &.{ 0x10004004, c.w1, 0x00100000 | 3 << 5 | 4 << 1, 4 << 27 | 5 << 19, 0x00000801, c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x01 R slot takes the product route too; its immediate and zero bits read Y" {
    // fact 522: own-x01mp, s7 = 0x100, s1 = 3: R2 = 0x300 (mode 4, w1[17] = 1). The
    // 0x00 immediate (w5[26:25] = 11) and zero (10) read Y:[e<w4[8:5]>] on 0x01 (505).
    const gpa = testing.allocator;
    for ([_]struct { w5: u32, want: u32 }{
        .{ .w5 = 0xc2000900, .want = 0x300 },
        .{ .w5 = 0xc6000900, .want = 0x4242 },
        .{ .w5 = 0xc4000900, .want = 0x4242 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[7] = 0x100;
        state.s[1] = 3;
        try state.y_mem.put(gpa, 0, 0x4242);
        _ = try emu.runFrom(gpa, &state, &.{ 0x409dd001, 0x0002a502, 0xe8008000, 0x40000100, 0x00000000, c.w5, 0x17000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.r[2]);
    }
}

test "ldy also loads R<[25:22]>, with or without its s destination" {
    // fact 523: own-ldyr (ldy.r s0, e3, R2 is 4080026a) and the synthetic words, Y:[e5].
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        ldy.r s0, e3, R2
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    try testing.expectEqual(@as(u32, 0x4080026a), prog[0]);
    for ([_]struct { w: u32, r: u32, s: u32 }{
        .{ .w = 0x420002aa, .r = 8, .s = 0 },
        .{ .w = 0x7a0002aa, .r = 8, .s = 14 },
        .{ .w = 0x408002aa, .r = 2, .s = 0 },
        .{ .w = 0x41c002aa, .r = 7, .s = 0 },
        .{ .w = 0x4080026a, .r = 2, .s = 0 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[3] = 0x710;
        state.e[5] = 0x710;
        try state.y_mem.put(gpa, 0x710, 0x9357);
        _ = try emu.runFrom(gpa, &state, &.{ c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x9357), state.r[c.r]);
        if (c.s != 0) try testing.expectEqual(@as(u32, 0xFF9357), state.s[c.s]);
        for (0..16) |i| if (i != c.s) try testing.expectEqual(@as(u32, 0), state.s[i]);
    }
    // R with [30] = 0 (no s destination field) is unmeasured
    var st3 = emu.State{};
    defer st3.deinit(gpa);
    st3.e[5] = 0x710;
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st3, &.{ 0x020002aa, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "the class-0A parallel move writes R<[25:22]> too" {
    // fact 523: 7B368AAA writes R12 beside s14 and its X store.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[5] = 0x710;
    try state.y_mem.put(gpa, 0x710, 0x1357);
    _ = try emu.runFrom(gpa, &state, &.{ 0x7B368AAA, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1357), state.r[12]);
    try testing.expectEqual(@as(u32, 0x1357), state.s[14]);
    // without bit 9 both take the stale latch, unmodelled
    var st2 = emu.State{};
    defer st2.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st2, &.{ 0x7B368CAA & ~@as(u32, 0x200), 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    // an s0 target without bit 9 is let through, but not beside an R destination
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st2, &.{ 0x433000AA, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "dld loads s from X:[A] and s from Y:[e]" {
    // fact 524: own-dldx (40918009: s2 = X:[A3] = 0x3333), the Y half with [13], and
    // the refused cases.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        dld s2, A3, s0, e0
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    try testing.expectEqual(@as(u32, 0x40918009 | 1 << 13), prog[0]);
    for ([_]struct { w: u32, s2: ?u32, s5: u32 }{
        .{ .w = 0x40918009, .s2 = 0x3333, .s5 = 0 },
        .{ .w = 0x40918009 | 1 << 13 | 5 << 26 | 4 << 5, .s2 = 0x3333, .s5 = 0x1357 },
        .{ .w = 0x40918009 | 1 << 12 | 0xF << 9, .s2 = 0x3333, .s5 = 0 }, // [12:9] inert
        .{ .w = (0x40918009 & ~@as(u32, 0x1F << 15)) | 17 << 15, .s2 = 0x7777, .s5 = 0 }, // A17
        .{ .w = 0x40118009 | 1 << 13 | 5 << 26 | 4 << 5, .s2 = 0x0ABC, .s5 = 0x1357 }, // no X destination
        .{ .w = 0x40918009 | 1 << 13 | 5 << 26 | 4 << 5 | 1 << 9, .s2 = 0x3333, .s5 = 0x1357 },
        .{ .w = 0x40918009 & ~@as(u32, 1 << 20), .s2 = null, .s5 = 0 },
        .{ .w = 0x40918009 | 5 << 26 | 4 << 5, .s2 = null, .s5 = 0 }, // [13] = 0: the stale latch
        .{ .w = 0x40918009 | 1 << 13 | 2 << 26 | 4 << 5, .s2 = null, .s5 = 0 }, // one s for both
        .{ .w = 0x40918009 | 1 << 13 | 5 << 26 | 6 << 5, .s2 = 0x3333, .s5 = 0xFF9357 }, // sign-extended (54)
        .{ .w = 0x40918009 & ~@as(u32, 1 << 30), .s2 = null, .s5 = 0 }, // [30] = 0 wedges (54, 116)
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[2] = 0x0ABC;
        for ([_]u32{ 1, 2, 3, 4, 12 }, 0..) |a, i| {
            state.a[a] = 0x300 + @as(u32, @intCast(i));
            try state.x_mem.put(gpa, 0x300 + @as(u32, @intCast(i)), 0x1111 * a);
        }
        state.a[17] = 0x320;
        try state.x_mem.put(gpa, 0x320, 0x7777);
        state.e[4] = 0x700;
        state.e[6] = 0x704;
        try state.y_mem.put(gpa, 0x700, 0x1357);
        try state.y_mem.put(gpa, 0x704, 0x9357);
        const r = emu.runFrom(gpa, &state, &.{ c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.s2) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
            try testing.expectEqual(c.s5, state.s[5]);
            try testing.expectEqual(@as(u32, 0), state.s[0]); // s0 reads 0 (438)
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "st.x also performs rimm's writes" {
    // fact 525: own-stxr (0000246f ac501001: R2 = 0x123); op[14:11] = 0 leaves R2; the
    // s<head[24:21]> write (488's OPEN value) refuses.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, s1: u32, want: ?u32 }{
        .{ .h = 0x0000246f, .s1 = 0xac501001, .want = 0x123 },
        .{ .h = 0x0000246f, .s1 = 0xac500001, .want = 0xABC },
        .{ .h = 0x0020246f, .s1 = 0xac501001, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[2] = 0xABC;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the x18 class-110 ALU2 slot: the rounded shift under code 0x04" {
    // fact 527: own-x18rnd, s1 = 0x100, s2 = 7: n = 2 gives 0x42 (truncation 0x41),
    // n = 3 0x21, n = 0 0x107; codes 0x09 / 0x0F add; other codes and ops refuse.
    const gpa = testing.allocator;
    const w3: u32 = 0xc4742c09 & ~@as(u32, 0xF << 25);
    const h: u32 = 0x08000018 & ~@as(u32, 0x3F << 25);
    for ([_]struct { h: u32, w1: u32, n: u32, s1: u32, want: ?u32 }{
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 2, .s1 = 0x100, .want = 0x42 },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 3, .s1 = 0x100, .want = 0x21 },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 0, .s1 = 0x100, .want = 0x107 },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 9, .s1 = 0x10000, .want = 0x80 },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 2, .s1 = 0xFFFF00, .want = 0xFFFFC2 },
        .{ .h = h | 0x09 << 25, .w1 = 0x80005320, .n = 2, .s1 = 0x100, .want = 0x107 },
        .{ .h = h | 0x0F << 25, .w1 = 0x80005320, .n = 2, .s1 = 0x100, .want = 0x107 },
        .{ .h = h | 0x05 << 25, .w1 = 0x80005320, .n = 2, .s1 = 0x100, .want = null },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320 | 1 << 15, .n = 2, .s1 = 0x100, .want = null },
        .{ .h = h | 0x04 << 25, .w1 = 0x80005320, .n = 2, .s1 = 0x7FFFFE, .want = null },
        // the X slot writes b's register too: the slot reads b before it (s2 then 0x500)
        .{ .h = h | 0x04 << 25 | 2 << 21, .w1 = 0x80005320, .n = 2, .s1 = 0x100, .want = 0x42 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = c.s1;
        state.s[2] = 7;
        state.a[8] = 0x300;
        try state.x_mem.put(gpa, 0x300, 0x500);
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.w1, 0x00500029, w3 | c.n << 25, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "s0 reads 0 even when the state holds a value there" {
    // fact 438: s0 always reads 0; stx has no index-0 case of its own.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        stx A3, s0
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[0] = 0xDEAD;
    state.a[3] = 0x300;
    _ = try emu.runFrom(gpa, &state, prog, 0, 16);
    try testing.expectEqual(@as(?u32, 0), state.readX(0x300));
}

test "x17xld runs x17a05's head ALU slot beside its load" {
    // fact 528: own-x17xla, A9 = 0x380, s5 = 0x100 gives s3 = 0xFFFF0C (s5 = 0x40:
    // 0xFFFFCC); the load lands in s1 from X:[A<8*1 + 1>].
    const gpa = testing.allocator;
    for ([_]struct { s5: u32, h: u32, want: ?u32 }{
        .{ .s5 = 0x100, .h = 0x22e18137, .want = 0xFFFF0C },
        .{ .s5 = 0x40, .h = 0x22e18137, .want = 0xFFFFCC },
        .{ .s5 = 0x100, .h = 0x22e18137 & ~@as(u32, 0xF << 15), .want = 0xABC }, // no ALU write
        .{ .s5 = 0x100, .h = (0x22e18137 & ~@as(u32, 0xF << 15)) | 1 << 15, .want = null }, // one s for both
        .{ .s5 = 0x100, .h = (0x22e18137 & ~@as(u32, 0x3F << 5)) | 1 << 5, .want = null }, // op 0x01 m5 #12: OPEN
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x380;
        state.s[5] = c.s5;
        state.s[3] = 0xABC;
        try state.x_mem.put(gpa, 0x380, 0x777);
        const r = emu.runFrom(gpa, &state, &.{ c.h, 0x224aa500, 0xca204401, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
            try testing.expectEqual(@as(u32, 0x777), state.s[1]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19a3's s write goes through the w1[17:15] mux; A always takes AGU3" {
    // fact 529: own-x19a3m, A14 = 0x300: s3 <- A14 - 0x100 = 0x200 (mux 1); mux 0
    // gives 0; A5 takes the AGU3 value either way; mux 6 refuses (5 is 611's X read, below).
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, s3: ?u32 }{
        .{ .w1 = 0x0814bb00, .s3 = 0x200 },
        .{ .w1 = 0x08143b00, .s3 = 0 },
        .{ .w1 = 0x08153b00, .s3 = 0x200 }, // mux 2
        .{ .w1 = 0x08173b00, .s3 = null }, // mux 6
        .{ .w1 = 0x0014bb00, .s3 = null }, // w1[27] cleared wedges (559)
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[14] = 0x300;
        state.s[3] = 0xABC;
        state.r[5] = 0x10; // y = R<w1[21:18]>, read by AGU10 fn 6 (x + y) into R7
        const r = emu.runFrom(gpa, &state, &.{ 0x0a600019, c.w1, 0x00002008, 0x7b570080, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.s3) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
            try testing.expectEqual(@as(u32, 0x200), state.a[5]);
            try testing.expectEqual(@as(u32, 0x10), state.r[7]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "psel also performs shape A's head write" {
    // fact 531: own-pselh, s3 = 0xABC: 4660246e 3205e250 gives s3 = 0x123 (mux 6);
    // h[24:21] = 0 leaves s3; mux 2 is the same immediate; mux 0 writes 0 (152); mux 3 is
    // OPEN.  The select (p2 = 0, S[21] = 0) takes X16_ALU[1][5] = a + 1 with a = s3 read
    // before the head write: s2 = 0xABD.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, s1: u32, s3: ?u32 }{
        .{ .h = 0x4660246e, .s1 = 0x3205e250, .s3 = 0x123 },
        .{ .h = 0x4600246e, .s1 = 0x3205e250, .s3 = 0xABC },
        .{ .h = 0x4660246e, .s1 = 0x3205a250, .s3 = 0x123 },
        .{ .h = 0x4660246e, .s1 = 0x32058250, .s3 = 0 },
        .{ .h = 0x4660246e, .s1 = 0x3205b250, .s3 = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0xABC;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.s3) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
            try testing.expectEqual(@as(u32, 0xABD), state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19agu writes s<head[24:21]> through its w1[14:12] mux" {
    // fact 532: own-x19agx, X:[A3 = 0x310] = 0x567: mux 5 gives s5 = 0x567, mux 4
    // 0xB380, w3[1] = 0 the stale yd (0xA050); mux 0 zero, mux 1 the AGU value (R5's, e6 + 1);
    // mux 2 with w2[25] = 1 is AGU10 again (636).
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, w3: u32, s5: ?u32, w2: u32 = 0x02460000 }{
        .{ .w1 = 0x02045004, .w3 = 0x0c232a82, .s5 = 0x567 },
        .{ .w1 = 0x02044004, .w3 = 0x0c232a82, .s5 = 0xB380 },
        .{ .w1 = 0x02045004, .w3 = 0x0c232a80, .s5 = 0xA050 },
        .{ .w1 = 0x02040004, .w3 = 0x0c232a82, .s5 = 0 },
        .{ .w1 = 0x02041004, .w3 = 0x0c232a82, .s5 = 0x101 },
        .{ .w1 = 0x02042004, .w3 = 0x0c232a82, .s5 = 0x101 },
        .{ .w1 = 0x02045004, .w3 = 0x0c232a82, .s5 = 0x777, .w2 = 0x02660000 }, // A19
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[3] = 0x310;
        state.e[6] = 0x100;
        state.s[5] = 0xABC;
        state.yd = 0xA050;
        try state.x_mem.put(gpa, 0x310, 0x567);
        state.a[19] = 0x320;
        try state.x_mem.put(gpa, 0x320, 0x777);
        const r = emu.runFrom(gpa, &state, &.{ 0x80a00019, c.w1, c.w2, c.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.s5) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
            try testing.expectEqual(@as(u32, 0x101), state.r[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19alu's two X writes beside its ALU" {
    // fact 533: own-x19alx.  X:[A6 = 0x310] = 0x567 lands in s<w2[7:4]> = s2;
    // X:[A11 = 0x330] = 0x246 goes through mux w2[3:1] into s<w2[30:27]> = s3: 5 X, 4
    // 0x2300, 1 yd, 0 zero; mux 2 refuses, and so does s2 targeted by both writes.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w1: u32, w2: u32, dest: u32, want: ?u32, w3: u32 = 0x80000000 }{
        .{ .h = 0x00000019, .w1 = 0x00004000, .w2 = 0x01821220, .dest = 2, .want = 0x567, .w3 = 0x84000000 },
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac0220a, .dest = 3, .want = 0x246 },
        // 533 read 0x2300 through a 16-bit Y window; 884 keeps the full X[15:0] << 7.
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac02208, .dest = 3, .want = 0x12300 },
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac02202, .dest = 3, .want = 0xA050 },
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac02200, .dest = 3, .want = 0 },
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac02204, .dest = 3, .want = null },
        .{ .h = 0x00000019, .w1 = 0x00004000, .w2 = 0x11821220, .dest = 2, .want = null },
        .{ .h = 0x00000019, .w1 = 0x00004000, .w2 = 0x05821220, .dest = 2, .want = 0x777, .w3 = 0x84000000 }, // A22
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[6] = 0x310;
        state.a[11] = 0x330;
        state.yd = 0xA050;
        state.s[2] = 0xABC;
        state.s[3] = 0xABC;
        try state.x_mem.put(gpa, 0x310, 0x567);
        try state.x_mem.put(gpa, 0x330, 0x246);
        state.a[22] = 0x320;
        try state.x_mem.put(gpa, 0x320, 0x777);
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.w1, c.w2, c.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[c.dest]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19alu mode 8 writes op(#head[26:11]) to e<2*w1[2:0] + head[31]> and clears p (851, 852)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w1: u32, w2: u32 = 0x0003B000, e: u32, want: ?u32, p14: u32 = 0 }{
        .{ .h = 0xD8002819, .w1 = 0x00004502, .e = 5, .want = 5 },
        .{ .h = 0xD8002019, .w1 = 0x00004502, .e = 5, .want = 4 }, // head[11] cleared
        .{ .h = 0xDC002819, .w1 = 0x00004502, .e = 5, .want = 0x8005 }, // head[26] set
        .{ .h = 0x58002819, .w1 = 0x00004502, .e = 4, .want = 5 }, // head[31] = 0
        .{ .h = 0xD8002819, .w1 = 0x00004506, .e = 13, .want = 5 },
        .{ .h = 0xD8002819, .w1 = 0x00004582, .e = 5, .want = 0xFFFB }, // w1[7]
        .{ .h = 0xD8002819, .w1 = 0x00004702, .e = 5, .want = 0 }, // w1[9]
        .{ .h = 0xD8002819, .w1 = 0x00004102, .e = 5, .want = 1 }, // w1[10] cleared
        .{ .h = 0xD8002819, .w1 = 0x00004402, .e = 5, .want = 5 }, // w1[8] cleared: op 1000 (852)
        .{ .h = 0xD8002819, .w1 = 0x00004002, .e = 5, .want = 0 }, // op 0000
        .{ .h = 0xD8002819, .w1 = 0x00004282, .e = 5, .want = 0xFFFF }, // op 0101
        .{ .h = 0xD8002819, .w1 = 0x00004182, .e = 5, .want = 1 }, // op 0011
        .{ .h = 0xD8002819, .w1 = 0x00004602, .e = 5, .want = 0 }, // op 1100
        .{ .h = 0xD8002819, .w1 = 0x00004502, .w2 = 0x0003B001, .e = 5, .want = 5, .p14 = 9 }, // w2[0] stops the p write
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[4] = 0x444;
        state.e[5] = 0x555;
        state.e[13] = 0xDDD;
        state.p[14] = 9;
        try state.y_mem.put(gpa, 0, 1); // e8 = 0 reads an odd word: p14 <- 0 (858)
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.w1, c.w2, 0x90000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.e[c.e]);
            try testing.expectEqual(c.p14, state.p[14]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "xst writes the head imm by S[28:27] and p<S[14:11]> <- !(Y:[e<S[24:21]>] & 1) (106, 852)" {
    const gpa = testing.allocator;
    for ([_]struct { y: u32, p6: u32 }{
        .{ .y = 0x1234, .p6 = 1 }, .{ .y = 0x1235, .p6 = 0 }, .{ .y = 2, .p6 = 1 },
        .{ .y = 3, .p6 = 0 },      .{ .y = 0x8001, .p6 = 0 }, .{ .y = 0xFFFE, .p6 = 1 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x2C00;
        state.s[12] = 2;
        state.p[6] = 7;
        try state.y_mem.put(gpa, 0x2C00, c.y);
        _ = try emu.runFrom(gpa, &state, &.{ 0x01800034, 0x32A13000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 1), state.s[12]);
        try testing.expectEqual(c.p6, state.p[6]);
        try testing.expectEqual(@as(u32, 0), state.x_mem.count()); // S[3] = S[8] = 0: no store
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[5] = 0x8005; // e bit 15: OPEN
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x01800034, 0x32A13000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "dld's Y half also writes the raw word to e<[12:9]> (854)" {
    const gpa = testing.allocator;
    for ([_]struct { w: u32, y: u32 = 0x4321, s13: u32, e: u32, want: u32 }{
        .{ .w = 0x74002669, .s13 = 0x4321, .e = 3, .want = 0x4321 },
        .{ .w = 0x74002A69, .s13 = 0x4321, .e = 5, .want = 0x4321 }, // [12:9] = 5
        .{ .w = 0x74002629, .s13 = 0x1111, .e = 3, .want = 0x1111 }, // [8:5] = 1
        .{ .w = 0x74002669, .y = 0x8001, .s13 = 0xFF8001, .e = 3, .want = 0x8001 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x2C10;
        state.e[3] = 0x2C00;
        state.e[5] = 0x2C20;
        try state.y_mem.put(gpa, 0x2C10, 0x1111);
        try state.y_mem.put(gpa, 0x2C00, c.y);
        try state.y_mem.put(gpa, 0x2C20, 0x5555);
        _ = try emu.runFrom(gpa, &state, &.{ c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.s13, state.s[13]);
        try testing.expectEqual(c.want, state.e[c.e]);
    }
}

test "x16 b16 writes #head[20:5] to e<S1[23:20]> and A<2*S1[3:0] + head[31]> (855)" {
    const gpa = testing.allocator;
    const h: u32 = 0x01C21036; // s14 <- Y:[e4], imm 0x1081
    for ([_]struct { h: u32 = h, s1: u32 = 0xE0951C00, s2: u32 = 0xF8967100, e: u32 = 9, ev: u32 = 0x1081, a: u32 = 0 }{
        .{},
        .{ .s1 = 0xE0851C00, .e = 8 }, // S1[20] cleared
        .{ .s1 = 0xE0151C00, .e = 1 }, // S1[23] cleared
        .{ .s2 = 0xF8947100, .ev = 0 }, // S2[17] cleared
        .{ .s2 = 0xF8867100, .ev = 0 }, // S2[20] cleared
        .{ .h = h | 0x80000000, .a = 1 },
        .{ .s1 = 0xE0951C02, .a = 4 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[4] = 0xFD94;
        try state.y_mem.put(gpa, 0xFD94, 0x920D);
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.ev, state.e[c.e]);
        try testing.expectEqual(@as(u32, 0xFF920D), state.s[14]);
        if (c.a != 0) try testing.expectEqual(@as(u32, 0x1081), state.a[c.a]);
    }
}

test "x19alu mode 0xA: op 31 is a | b, written whatever p1, and p<w2[17:14]> cleared (857)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x000003F9, w1: u32 = 0x0C0C5000, w2: u32 = 0x003A9000, p1: u8 = 0, s12: u32, p10: u8 = 0 }{
        .{ .s12 = 0x177 },
        .{ .p1 = 1, .s12 = 0x177 },
        .{ .h = 0x00000019, .s12 = 0x123 }, // op 0: a + 0
        .{ .w1 = 0x0C6C5000, .p1 = 1, .s12 = 0x177 }, // cond 6
        .{ .w2 = 0x003A9001, .s12 = 0x177, .p10 = 1 }, // w2[0] stops the p write
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[14] = 0x123;
        state.s[12] = 0x77;
        state.p[1] = c.p1;
        state.p[10] = 1;
        try state.y_mem.put(gpa, 0, 1); // e9 = 0 reads an odd word: p10 <- 0 (858)
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.w1, c.w2, 0x92000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.s12, state.s[12]);
        try testing.expectEqual(c.p10, state.p[10]);
    }
}

test "x19alu's p write is !(Y:[e<w3[28:25]>] & 1) (858)" {
    const gpa = testing.allocator;
    for ([_]struct { e9: u32 = 0, e5: u32 = 0, w3: u32 = 0x92000000, p10: u8 }{
        .{ .e9 = 0x2C00, .p10 = 0 },
        .{ .e9 = 0x2C01, .p10 = 1 },
        .{ .e5 = 0x2C00, .w3 = 0x8A000000, .p10 = 0 },
        .{ .e5 = 0x2C01, .w3 = 0x8A000000, .p10 = 1 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[9] = c.e9;
        state.e[5] = c.e5;
        state.p[10] = 7;
        try state.y_mem.put(gpa, 0x2C00, 1);
        try state.y_mem.put(gpa, 0x2C01, 2);
        _ = try emu.runFrom(gpa, &state, &.{ 0x000003F9, 0x0C0C5000, 0x003A9000, c.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.p10, state.p[10]);
    }
}

test "x19k writes #head[20:5] to e<w1[23:20]> after its slots read e (859)" {
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, e: u32 }{
        .{ .w1 = 0x90941000, .e = 9 },
        .{ .w1 = 0x90841000, .e = 8 }, // w1[20] cleared
        .{ .w1 = 0x90141000, .e = 1 }, // w1[23] cleared
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[9] = 1;
        _ = try emu.runFrom(gpa, &state, &.{ 0x01821179, c.w1, 0x33E20000, 0x13A00801, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x108B), state.e[c.e]);
        try testing.expectEqual(@as(u32, 1), state.s[12]); // AGU fn 0 read e9 before the write
    }
}

test "x15 pt writes #head[20:5] to e<2*S[2:0] + head[31]> (860)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x1D821195, s: u32 = 0x21898069, e: u32, v: u32 = 0x108C }{
        .{ .e = 2 },
        .{ .s = 0x2189806B, .e = 6 }, // S[1]
        .{ .s = 0x2189806D, .e = 10 }, // S[2]
        .{ .h = 0x9D821195, .e = 3 }, // head[31]
        .{ .h = 0x1D8211B5, .e = 2, .v = 0x108D }, // head[5]
        .{ .h = 0x1D921195, .e = 2, .v = 0x908C }, // head[20]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[c.e] = 9;
        state.s[12] = 7;
        state.p[13] = 1;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.s, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.e[c.e]);
        try testing.expectEqual(@as(u32, 0), state.s[12]);
    }
}

test "aset0b also writes the raw word to e<[12:9]> and sign-extends s (861)" {
    const gpa = testing.allocator;
    for ([_]struct { w: u32 = 0x9C1A288B, y: u32 = 0x1234, s: u32 = 7, sv: u32 = 0x1234, e: u32 = 4 }{
        .{},
        .{ .w = 0x9C18288B, .y = 0x8001, .sv = 0xFF8001 }, // A dest 0: x17 refuses a bit-15 A copy
        .{ .w = 0x9C1A2A8B, .e = 5 }, // [9]
        .{ .w = 0x981A288B, .s = 6 }, // [26]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[4] = 0x100;
        try state.y_mem.put(gpa, 0x100, c.y);
        _ = try emu.runFrom(gpa, &state, &.{ c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.sv, state.s[c.s]);
        try testing.expectEqual(c.y, state.e[c.e]);
    }
}

test "aset0b's A write is off while [19] = 1 (862)" {
    const gpa = testing.allocator;
    for ([_]struct { w: u32, a4: u32, a6: u32 = 0x555 }{
        .{ .w = 0x9C1A288B, .a4 = 0x777 },
        .{ .w = 0x9C12288B, .a4 = 0x1234 }, // [19] = 0
        .{ .w = 0x9C13288B, .a4 = 0x777, .a6 = 0x1234 }, // [19] = 0, [16] set: A6
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[4] = 0x100;
        state.a[4] = 0x777;
        state.a[6] = 0x555;
        try state.y_mem.put(gpa, 0x100, 0x1234);
        _ = try emu.runFrom(gpa, &state, &.{ c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.a4, state.a[4]);
        try testing.expectEqual(c.a6, state.a[6]);
    }
}

test "x19k writes p<w2[30:27]> <- (R<w3[14:11]> != 0) under w3[23:20] = 10 (862)" {
    const gpa = testing.allocator;
    for ([_]struct { w3: u32 = 0x13A00801, r1: u32, r3: u32 = 0, p6: u8, p6_in: u8 = 1 }{
        .{ .r1 = 0, .p6 = 0 },
        .{ .r1 = 9, .p6 = 1 },
        .{ .r1 = 0xFFFF, .p6 = 1 },
        .{ .r1 = 9, .p6 = 1, .p6_in = 0 }, // a write, not a conditional clear
        .{ .w3 = 0x13A01801, .r1 = 0, .r3 = 5, .p6 = 1 }, // w3[12]: R3
        .{ .w3 = 0x13A01801, .r1 = 9, .r3 = 0, .p6 = 0 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[9] = 1;
        state.r[1] = c.r1;
        state.r[3] = c.r3;
        state.p[6] = c.p6_in;
        _ = try emu.runFrom(gpa, &state, &.{ 0x01821179, 0x90941000, 0x33E20000, c.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.p6, state.p[6]);
    }
}

test "x02's Y side also writes the raw word to e<w1[6:3]> with w5[17] (863)" {
    const gpa = testing.allocator;
    const b = [_]u32{ 0x000001C2, 0x80000040, 0x00014001, 0x40080000, 0x00240818, 0x30021628 };
    for ([_]struct { w1: u32 = b[1], w5: u32 = b[5], y: u32 = 0x1234, e: u32 = 8, ev: ?u32 = 0x1234, s3: u32 = 0x1234 }{
        .{},
        .{ .y = 0x8001, .ev = 0x8001, .s3 = 0xFF8001 },
        .{ .w1 = 0x80000048, .e = 9 }, // w1[3]
        .{ .w5 = 0x30001628, .ev = null }, // w5[17] cleared
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[8] = 0x100;
        state.e[9] = 0x100;
        try state.y_mem.put(gpa, 0x100, c.y);
        _ = try emu.runFrom(gpa, &state, &.{ b[0], c.w1, b[2], b[3], b[4], c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.ev orelse 0x100, state.e[c.e]);
        try testing.expectEqual(c.y, state.r[1]);
        try testing.expectEqual(c.s3, state.s[3]);
    }
}

test "x19k's p write: w3[23:20] conds on R<w3[14:11]> (864, 865, 867)" {
    const gpa = testing.allocator;
    // p6 for R1 = 0, 1, 9, 0x7FFF, 0x8000, 0xFFFF per cond.
    const want = [16][6]u8{
        .{ 1, 1, 1, 1, 1, 1 }, .{ 0, 0, 0, 0, 0, 0 }, .{ 1, 1, 1, 0, 0, 0 }, .{ 0, 0, 1, 1, 1, 0 },
        .{ 0, 0, 0, 0, 0, 1 }, .{ 0, 0, 0, 0, 0, 0 }, .{ 0, 1, 1, 1, 0, 0 }, .{ 1, 1, 1, 1, 0, 0 },
        .{ 1, 0, 0, 0, 1, 1 }, .{ 0, 0, 0, 0, 1, 1 }, .{ 0, 1, 1, 1, 1, 1 }, .{ 1, 0, 0, 0, 0, 0 },
        .{ 0, 0, 0, 0, 1, 1 }, .{ 1, 1, 1, 1, 0, 0 }, .{ 0, 1, 1, 1, 1, 1 }, .{ 1, 0, 0, 0, 0, 0 },
    };
    for (want, 0..) |row, cond| {
        for ([_]u32{ 0, 1, 9, 0x7FFF, 0x8000, 0xFFFF }, 0..) |r1, k| {
            for ([_]u8{ 0, 1 }) |p_in| {
                var state = emu.State{};
                defer state.deinit(gpa);
                state.e[9] = 1;
                state.r[1] = r1;
                state.p[6] = p_in;
                const w3: u32 = (0x13A00801 & ~@as(u32, 0xF << 20)) | (@as(u32, @intCast(cond)) << 20);
                _ = try emu.runFrom(gpa, &state, &.{ 0x01821179, 0x90941000, 0x33E20000, w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
                try testing.expectEqual(row[k], state.p[6]);
            }
        }
    }
}

test "x02 mode 001 runs the w2 slot: s<w2[12:9]> <- X:[A<w4[21:17]>] (866)" {
    const gpa = testing.allocator;
    const b = [_]u32{ 0x00000002, 0x00000000, 0x00001E00, 0x00400000, 0x00240860, 0xC0080200 };
    for ([_]struct { w2: u32 = b[2], w5: u32 = b[5], s: u32 = 15, v: u32 = 0xABCDEF }{
        .{},
        .{ .w2 = 0x00001C00, .s = 14 }, // w2[9]
        .{ .w2 = 0x00000E00, .s = 7 }, // w2[12]
        .{ .w5 = 0xC0000200, .v = 0 }, // w5[19] cleared: 0
        .{ .w2 = 0x00001EA6, .w5 = 0xC0000200, .v = 0 }, // w2[4:1] = 3, w2[8:5] = 5: still 0, no product
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[18] = 0x11C4;
        state.s[c.s] = 0x77;
        state.s[3] = 0x100;
        state.s[5] = 0x10;
        try state.x_mem.put(gpa, 0x11C4, 0xABCDEF);
        _ = try emu.runFrom(gpa, &state, &.{ b[0], b[1], c.w2, b[3], b[4], c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.s[c.s]);
    }
}

test "ystm with the store on and h[26:27] = 0 writes op(imm) to e<h[12:9]> (868)" {
    const gpa = testing.allocator;
    const sw: u32 = 0x4017B000;
    for ([_]struct { h: u32 = 0x60150230, sw: u32 = sw, e: u32 = 1, v: u32 }{
        .{ .v = 6 },
        .{ .h = 0x70150230, .v = 7 }, // h[28]
        .{ .sw = sw | 0x800, .v = 0x8006 }, // S[11]
        .{ .h = 0x60150630, .e = 3, .v = 6 }, // h[10]
        .{ .h = 0x60110230, .v = 6 }, // op 1000
        .{ .h = 0x60170230, .v = 0xFFFA }, // op 1011: -imm
        .{ .h = 0x60050230, .v = 0 }, // op 0010
        .{ .h = 0x601D0230, .v = 0x101 }, // op 1110: e1 + 1
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x100;
        state.s[11] = 0x55;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.e[c.e]);
    }
}

test "st.x 0x0F writes #head[20:5] to A<head[28:25]> while head[29] = 0 (869)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0xD60294EF, a: u32 = 11, v: u32 = 0x14A7 }{
        .{},
        .{ .h = 0xD40294EF, .a = 10 }, // head[25]
        .{ .h = 0xF60294EF, .v = 0x100 }, // head[29] stops it
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[10] = 0x100;
        state.a[11] = 0x100;
        state.s[10] = 0x42;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, 0xB4500002, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.a[c.a]);
        try testing.expectEqual(@as(?u32, 0x42), state.x_mem.get(0x100)); // the store used the old A
    }
}

test "alu.m 0x2B m 1 is unsigned min; 0x28 m 5 is max(#, b) (870, 871)" {
    const gpa = testing.allocator;
    for ([_][3]u32{
        .{ 0x10, 1, 1 },     .{ 0x10, 0, 0 },     .{ 1, 0x10, 1 },
        .{ 0xFFFFFF, 1, 1 }, .{ 0xFFFFF0, 1, 1 }, .{ 0xFFFFF0, 0xFFFFF8, 0xFFFFF0 },
        .{ 5, 0xFF8000, 5 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = c[0];
        state.s[8] = c[1];
        _ = try emu.runFrom(gpa, &state, &.{ 0x60853565, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[2], state.s[10]);
    }
    for ([_][2]u32{ .{ 0xFFFFF1, 0 }, .{ 0xFF8000, 0 }, .{ 0x10, 0x10 }, .{ 1, 1 }, .{ 0x7000, 0x7000 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[10] = c[0];
        _ = try emu.runFrom(gpa, &state, &.{ 0x6A850505, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], state.s[10]);
    }
    // 871: the other operand is #[22:19]: max(#, s10) with s10 = 0xFFFFF1
    for ([_]u32{ 1, 2, 4, 8 }, 19..) |want, bit| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[10] = 0xFFFFF1;
        _ = try emu.runFrom(gpa, &state, &.{ 0x6A850505 | (@as(u32, 1) << @intCast(bit)), 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(want, state.s[10]);
    }
}

test "x17 rsub op 0 writes #head[20:5] to A<{S1[2:0], head[31]}> (872)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x83629517, s1: u32 = 0x000000A7, s2: u32 = 0x20000008, a: u32 = 15, v: u32 = 0x14A8 }{
        .{},
        .{ .h = 0x03629517, .a = 14 }, // head[31]
        .{ .s1 = 0x000000A6, .a = 13 }, // S1[0]
        .{ .s1 = 0x000000A3, .a = 7 }, // S1[2]
        .{ .h = 0x83629537, .v = 0x14A9 }, // head[5]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.a[c.a]);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x83629517, 0x000000AF, 0x20000008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16)); // S1[3]
}

test "x19k's s-slot AGU follows w2[25]; AGU10B fn 2 reads e<w2[7:4]> (871, 874)" {
    const gpa = testing.allocator;
    const b = [_]u32{ 0x00C5D7F9, 0x92142000, 0x61D60030, 0x01B02801 };
    for ([_]struct { w1: u32 = b[1], w2: u32 = b[2], s6: u32 }{
        .{ .s6 = 0x103 },
        .{ .w2 = 0x61D60050, .s6 = 0x555 }, // w2[7:4] = 5
        .{ .w2 = 0x61D60000, .s6 = 0 }, // w2[7:4] = 0
        .{ .w1 = 0x96142000, .s6 = 0 }, // fn 6: x + y with x = e0, y = R5
        .{ .w1 = 0x91142000, .s6 = 0 }, // fn 1: y
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[3] = 0x103;
        state.e[5] = 0x555;
        state.r[6] = 0x23;
        _ = try emu.runFrom(gpa, &state, &.{ b[0], c.w1, c.w2, b[3], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.s6, state.s[6]);
    }
}

test "x00 writes the immediate to A<w3[23:20]> (873)" {
    const gpa = testing.allocator;
    const b = [_]u32{ 0x00A55000, 0x01D00080, 0x00540000, 0x00B00000, 0xA8600000, 0x0700000A, 0x000E8004 };
    for ([_]struct { w3: u32 = b[3], w4: u32 = b[4], w5: u32 = b[5], a: u32 = 11, v: u32 = 0x14AA }{
        .{},
        .{ .w3 = 0x00A00000, .a = 10 }, // w3[20]
        .{ .w3 = 0x01B00000, .v = 0x14AD }, // w3[24] stops it
        .{ .w4 = 0xA8400000, .v = 0x14AD }, // w4[21]
        .{ .w5 = 0x0500000A, .v = 0x14AD }, // w5[25]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[10] = 0x14AD;
        state.a[11] = 0x14AD;
        _ = try emu.runFrom(gpa, &state, &.{ b[0], b[1], b[2], c.w3, c.w4, c.w5, b[6], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.a[c.a]);
    }
}

test "ealu's s<w[10:7]> is an index: 0x24B writes s9 <- 0xFFFFFF (875)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, d: u32, v: ?u32 = 0xFFFFFF }{
        .{ .s1 = 0xC0404487, .d = 9 },
        .{ .s1 = 0xC0404407, .d = 8 }, // w[7]
        .{ .s1 = 0xC0404587, .d = 11 }, // w[8]
        .{ .s1 = 0xC0004487, .d = 9, .v = null }, // w[22] cleared stops it
        .{ .s1 = 0xC0C04487, .d = 9, .v = null }, // w[23] set stops it
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[c.d] = 0x77;
        try emu.executeBundle(gpa, &state, &.{ 0xFFFFF810, c.s1 });
        try testing.expectEqual(c.v orelse 0x77, state.s[c.d]);
    }
}

test "pair also loads s<S[10:7]> <- X:[A<4*S[2:0] + head[31:30]>] (876)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32 = 0x6024822A, d: u32 = 4, v: ?u32 = 0x123456 }{
        .{},
        .{ .s1 = 0x6014822A, .v = null }, // S[21:20] = 01: no load
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[11] = 0x14AA;
        state.s[c.d] = 0x77;
        try state.x_mem.put(gpa, 0x14AA, 0x123456);
        try emu.executeBundle(gpa, &state, &.{ 0xC005D9D0, c.s1 });
        try testing.expectEqual(c.v orelse 0x77, state.s[c.d]);
        try testing.expectEqual(@as(u32, 0x2ECE), state.e[5]);
    }
}

test "alu.m 0x20 with modifiers writes 0 (877)" {
    const gpa = testing.allocator;
    for ([_]u32{ 0x403C8405, 0x40BC8405, 0x403C8405 | 1 << 26 }) |w| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[9] = 0x923;
        state.s[1] = 0x55;
        try emu.executeBundle(gpa, &state, &.{w});
        try testing.expectEqual(@as(u32, 0), state.s[9]);
    }
}

test "x17 rsub op 0 writes p<S1[7:4]> <- cond<head[24:21]>(R<S2[4:1]>) (878)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x83629517, s1: u32 = 0x000000A7, s2: u32 = 0x20000008, r4: u32 = 1, pd: u32 = 10, v: u8 }{
        .{ .v = 0 }, // cond 11 (== 0) on R4 = 1
        .{ .r4 = 0, .v = 1 },
        .{ .h = 0x83429517, .v = 1 }, // head[21]: cond 10 (!= 0)
        .{ .s1 = 0x000000B7, .pd = 11, .v = 0 }, // S1[4]
        .{ .s2 = 0x2000000A, .v = 1 }, // R5 = 0
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[4] = c.r4;
        state.p[c.pd] = 7;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.v, state.p[c.pd]);
    }
}

test "x16 code 0x0E mux 110: s <- imm, e and A <- op(old s<S2[28:25]>) (879)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x1D201FF6, s1: u32 = 0x00656002, s2: u32 = 0xF2100000, e: u32 = 6, ev: u32 = 0x1234, a: u32 = 4, av: u32 = 0x1234 }{
        .{},
        .{ .s1 = 0x0065E002, .ev = 0x1235, .av = 0x1235 }, // S1[15]: a + 1
        .{ .s1 = 0x00676002, .ev = 0xEDCC, .av = 0xEDCC }, // S1[17]: -a
        .{ .s2 = 0xF2000000, .ev = 0 }, // S2[20] cleared: e <- 0
        .{ .s1 = 0x00456002, .e = 4 }, // S1[21]
        .{ .s1 = 0x00656003, .a = 6 }, // S1[0]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[9] = 0x1234;
        try emu.executeBundle(gpa, &state, &.{ c.h, c.s1, c.s2 });
        try testing.expectEqual(@as(u32, 0xFF), state.s[9]);
        try testing.expectEqual(c.ev, state.e[c.e]);
        try testing.expectEqual(c.av, state.a[c.a]);
    }
}

test "x16 code 0x0E mux 110: s<S1[11:8]> takes the e/A value (880)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, old: u32, v: u32, e6: u32 }{
        .{ .s1 = 0x00656102, .old = 0x5555, .v = 0x5555, .e6 = 0x5555 },
        .{ .s1 = 0x0065E102, .old = 0x5555, .v = 0x5556, .e6 = 0x5556 }, // S1[15]
        .{ .s1 = 0x00676102, .old = 0x5555, .v = 0xAAAB, .e6 = 0xAAAB }, // S1[17]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[9] = c.old;
        try emu.executeBundle(gpa, &state, &.{ 0x1D201FF6, c.s1, 0xF2100000 });
        try testing.expectEqual(c.v, state.s[1]);
        try testing.expectEqual(c.e6, state.e[6]);
        try testing.expectEqual(c.v, state.a[4]);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[9] = 0x2222;
    try emu.executeBundle(gpa, &state, &.{ 0x1D201FF6, 0x00656102, 0xF2000000 }); // S2[20] = 0
    try testing.expectEqual(@as(u32, 0x2222), state.s[1]);
    try testing.expectEqual(@as(u32, 0), state.e[6]);
}

test "x00 ctl 2 jumps by sx15 when p<w4[31:28]> is 0 (881)" {
    const gpa = testing.allocator;
    for ([_]struct { c: u32, w1: u32 = 0x01F80008, o: ?i32 = 0x7E }{
        .{ .c = 10 },
        .{ .c = 1 },
        .{ .c = 14 },
        .{ .c = 10, .w1 = 0x01FC0008, .o = 0x7F }, // S1[18]
        .{ .c = 0, .o = null }, // p0 reads 1
        .{ .c = 12, .o = null }, // p12 set
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[12] = 1;
        try emu.executeBundle(gpa, &state, &.{ 0x008E09C0, c.w1, 0x003F6680, 0x00000002, (c.c << 28) | 0x00100000, 0x06000006, 0x00009000 });
        try testing.expectEqual(c.o, state.pending_jmp_offset);
    }
}

test "x00 ctl 1 jumps whatever p<w4[31:28]> holds (882)" {
    const gpa = testing.allocator;
    for ([_]u32{ 0, 1, 10, 12, 14 }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[12] = 1;
        try emu.executeBundle(gpa, &state, &.{ 0x008E09C0, 0x01F80008, 0x003F6680, 0x00000001, (c << 28) | 0x00100000, 0x06000006, 0x00009000 });
        try testing.expectEqual(@as(?i32, 0x7E), state.pending_jmp_offset);
    }
}

test "lde class 011 writes A<[25:21]> with e<[29:26]> (882)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32, a: u32 }{
        .{ .h = 0x65600066, .a = 11 },
        .{ .h = 0x64A00066, .a = 5 }, // [24:21] = 5
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try emu.executeBundle(gpa, &state, &.{c.h});
        try testing.expectEqual(@as(u32, 3), state.e[9]);
        try testing.expectEqual(@as(u32, 3), state.a[c.a]);
    }
}

test "x19alu mode 0xE op 2 writes 0 beside the X load (883)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[9] = 0x1234;
    state.a[15] = 0x20F;
    state.p[7] = 1;
    try state.x_mem.put(gpa, 0x20F, 0x8978AE);
    try emu.executeBundle(gpa, &state, &.{ 0x00000459, 0x09007000, 0x2BC1C30B, 0x81E58000 });
    try testing.expectEqual(@as(u32, 0x8978AE), state.s[5]);
    try testing.expectEqual(@as(u32, 0), state.s[9]);
    try testing.expectEqual(@as(u8, 1), state.p[7]);
}

test "x19alu X mux 100 keeps X[15:0] << 7 over 24 bits (884)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.a[15] = 0x20F;
    try state.x_mem.put(gpa, 0x20F, 0x8978AE);
    // 0x2CE with w2[1] flipped: mux 101 -> 100.
    try emu.executeBundle(gpa, &state, &.{ 0x00000459, 0x09007000, 0x2BC1C309, 0x81E58000 });
    try testing.expectEqual(@as(u32, 0x3C5700), state.s[5]);
}

test "x18 class 011 stores the old s<w2[13:10]> at the old A<w2[21:18]> (885)" {
    const gpa = testing.allocator;
    for ([_]struct { w2: u32 = 0x00373240, w3: u32 = 0x60000039, addr: ?u32 = 0x305, v: u32 = 0x44 }{
        .{},
        .{ .w2 = 0x00373640, .v = 0x13 }, // w2[10]: s13
        .{ .w2 = 0x00333240, .addr = 0x30C }, // w2[18]: A12
        .{ .w2 = 0x00773240, .addr = null }, // w2[22] off
        .{ .w3 = 0x60000029, .addr = null }, // w3[4] off
        .{ .w2 = 0x00B73240 }, // w2[23] inert
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[13] = 0x305;
        state.a[12] = 0x30C;
        state.s[12] = 0x44;
        state.s[13] = 0x13;
        try emu.executeBundle(gpa, &state, &.{ 0x9D829438, 0x00000006, c.w2, c.w3 });
        if (c.addr) |a| try testing.expectEqual(@as(?u32, c.v), state.readX(a)) else {
            try testing.expectEqual(@as(?u32, null), state.readX(0x305));
            try testing.expectEqual(@as(?u32, null), state.readX(0x30C));
        }
        try testing.expectEqual(@as(u32, 0x14A1), state.a[13]);
        try testing.expectEqual(@as(u32, 1), state.s[12]);
    }
}

test "x19k: A<{w1[2:0], h31}> <- imm, e<w2[11:8]> <- e<w2[7:4]> - 1 (885, 886)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x80031019, w1: u32 = 0x90040007, w2: u32 = 0x52DE0210, a: u32 = 15, a_on: bool = true, e: u32 = 2, ev: u32 = 0x100 }{
        .{},
        .{ .h = 0x00031019, .a = 14 }, // h31
        .{ .w1 = 0x90040006, .a = 13 }, // w1[0]
        .{ .w1 = 0x9004000F, .a_on = false }, // w1[3] stops it
        .{ .w2 = 0x52DE0230, .ev = 0x102 }, // w2[5]: e3 - 1
        .{ .w2 = 0x52DE0200, .ev = 0xFFFF }, // w2[4]: e0 - 1
        .{ .w2 = 0x52DE0310, .e = 3 }, // w2[8]
        .{ .w2 = 0x50DE0210, .ev = 0 }, // w2[25]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |n| state.e[n] = @intCast(0x100 + n);
        for (1..31) |n| state.a[n] = @intCast(0x200 + n);
        try emu.executeBundle(gpa, &state, &.{ c.h, c.w1, c.w2, 0x00000004 });
        try testing.expectEqual(if (c.a_on) @as(u32, 0x1880) else 0x200 + c.a, state.a[c.a]);
        try testing.expectEqual(c.ev, state.e[c.e]);
    }
}

test "x19k cond 0 tests the e slot's result when it is live (887)" {
    const gpa = testing.allocator;
    for ([_]struct { e1: u32, p: u8 }{
        .{ .e1 = 1, .p = 0 },
        .{ .e1 = 2, .p = 1 },
        .{ .e1 = 0x10, .p = 1 },
        .{ .e1 = 0x8001, .p = 0 },
        .{ .e1 = 0, .p = 0 },
    }) |c| for ([_]u8{ 0, 1 }) |pre| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = c.e1;
        state.p[10] = pre;
        try emu.executeBundle(gpa, &state, &.{ 0x80031019, 0x90040007, 0x52DE0210, 0x00000004 });
        try testing.expectEqual(c.p, state.p[10]);
    };
}

test "x00 ctl 4 calls the absolute w1[31:18] (888)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.pc = 0x2D2;
    try emu.executeBundle(gpa, &state, &.{ 0x00A55000, 0x7DBC0000, 0x00000224, 0x00F00004, 0x00200000, 0x16000000, 0x00000090 });
    try testing.expectEqual(@as(?i32, 0x1F6F - 0x2D2), state.pending_jmp_offset);
}

test "class 0A reads z into R<[25:22]> and s<[29:26]> (889)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32, r: u32, s: u32 }{
        .{ .h = 0x4216010A, .r = 8, .s = 0 },
        .{ .h = 0x4256010A, .r = 9, .s = 0 },
        .{ .h = 0x4616010A, .r = 8, .s = 1 },
        .{ .h = 0xC216010A, .r = 8, .s = 0 }, // [31] inert
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.z = 0x920D;
        try emu.executeBundle(gpa, &state, &.{c.h});
        try testing.expectEqual(@as(u32, 0x920D), state.r[c.r]);
        if (c.s != 0) try testing.expectEqual(@as(u32, 0xFF920D), state.s[c.s]);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{0x0216010A})); // [30] = 0 wedges
}

test "ystm fn 10 puts y + i in e while R takes the Y word (890)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.r[1] = 9; // 891: the frame's R1; R2 = 0
    try state.y_mem.put(gpa, 0, 0x920D);
    _ = try emu.runFrom(gpa, &state, &.{ 0x10142670, 0x4616014D, 0x60146410, 0x4014012A, 0x20142450, 0x4416013C, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x920D), state.r[3]); // 0x1F70: Y:[e3 = 0]
    try testing.expectEqual(@as(u32, 0x14DA), state.e[3]); // R1 + 0x14D1
    try testing.expectEqual(@as(u32, 0x13CB), state.e[2]); // 0x1F72 gave R3 + 0x12A6, 0x1F74 R1 + 0x13C2
}

test "x19 imm loads R<w1[16:13]> from Y:[e<w3[28:25]>] (891)" {
    const gpa = testing.allocator;
    for ([_]struct { w1: u32 = 0x00044400, w2: u32 = 0x00000008, w3: u32 = 0x27B00000, r: u32 = 2, v: ?u32 = 0xE166 }{
        .{},
        .{ .w1 = 0x00046400, .r = 3 }, // w1[13]
        .{ .w1 = 0x00054400, .r = 10 }, // w1[16]
        .{ .w3 = 0x25B00000, .v = 0x4D12 }, // w3[25]: e2
        .{ .w2 = 0x00000000, .v = null }, // w2[3] kills it
        .{ .w2 = 0x00004008, .v = null }, // w2[14]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x13CB;
        state.e[3] = 0x14DA;
        try state.y_mem.put(gpa, 0x14DA, 0xE166);
        try state.y_mem.put(gpa, 0x13CB, 0x4D12);
        try emu.executeBundle(gpa, &state, &.{ 0x04000019, c.w1, c.w2, c.w3 });
        try testing.expectEqual(c.v orelse 0, state.r[c.r]);
    }
}

test "x14 class 010 with a class-000 second: A <- imm, s <- sx16 fn of e (891)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x42229314, s1: u32 = 0x0A200738, a: u32 = 1, s: u32 = 0xFFFDFF }{
        .{},
        .{ .h = 0x46229314, .a = 3 }, // h[26]: A3
        .{ .s1 = 0x0A640738, .s = 0x14DB }, // e3, fn 2: e + 1
        .{ .s1 = 0x0A280738, .s = 0xFFFDFE }, // fn 4: e - 1
        .{ .s1 = 0x0A300738, .s = 0 }, // fn 8
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0xFDFF;
        state.e[3] = 0x14DA;
        try emu.executeBundle(gpa, &state, &.{ c.h, c.s1 });
        try testing.expectEqual(@as(u32, 0x1498), state.a[c.a]);
        try testing.expectEqual(c.s, state.s[1]);
    }
}

test "x16.mul2 jumps by sx15(S2[23:9]) unless S1[14:13] != 0 (893)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32 = 0x01000040, s2: u32 = 0x20008A1C, off: ?i32 = 0x45 }{
        .{},
        .{ .s2 = 0x20FFE01C, .off = -0x10 }, // S2[23:9] = 0x7FF0
        .{ .s1 = 0x01002040, .off = null }, // S1[13]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[2] = 1;
        state.a[1] = 0x1498;
        state.s[4] = 4;
        try emu.executeBundle(gpa, &state, &.{ 0x00000016, c.s1, c.s2 });
        try testing.expectEqual(c.off, state.pending_jmp_offset);
        try testing.expectEqual(@as(?u32, 4), state.readX(0x1498));
    }
}

test "x16.mul2's jump needs p<S1[8:5]> unless it is 0 (905)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32 = 0x01000040, p: []const u4, off: ?i32 = null }{
        .{ .p = &.{2}, .off = 0x45 },
        .{ .p = &.{ 10, 12, 13 } },
        .{ .s1 = 0x01000060, .p = &.{2} }, // S1[8:5] = 3
        .{ .s1 = 0x01000060, .p = &.{3}, .off = 0x45 },
        .{ .s1 = 0x010000C0, .p = &.{6}, .off = 0x45 }, // S1[8:5] = 6
        .{ .s1 = 0x01000000, .p = &.{}, .off = 0x45 }, // S1[8:5] = 0
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (c.p) |i| state.p[i] = 1;
        try emu.executeBundle(gpa, &state, &.{ 0x00000016, c.s1, 0x20008A1C });
        try testing.expectEqual(c.off, state.pending_jmp_offset);
    }
}

test "x16.mul2 refuses S1[14] (905)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(emu.RunError.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x00000016, 0x01004040, 0x20008A1C }));
}

test "0x1F82 stores Y:[e<S[20:17]>] <- s<S[24:21]>; 0x1F84 loads s1 from Y:[e1] (894)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0xFDFF;
    state.e[2] = 0x13CB;
    state.e[3] = 0x14DA;
    state.s[1] = 0xFFFDFF;
    state.r[1] = 9;
    try state.y_mem.put(gpa, 0x14F7, 0x7B6E);
    try emu.executeBundle(gpa, &state, &.{ 0x9C029EF5, 0x24258500 });
    try testing.expectEqual(@as(?u32, 0xFDFF), state.getY(0x13CB));
    try testing.expectEqual(@as(u32, 0x14F7), state.e[1]);
    try emu.executeBundle(gpa, &state, &.{ 0x889DFC17, 0x28280501, 0x00200580 });
    try testing.expectEqual(@as(u32, 0x7B6E), state.s[1]);
    try testing.expectEqual(@as(u32, 0x13C8), state.e[3]);
    // S2[8] flipped: mux 2 is the AGU result.
    try emu.executeBundle(gpa, &state, &.{ 0x889DFC17, 0x28280501, 0x00200480 });
    try testing.expectEqual(@as(u32, 0x13C8), state.s[1]);
}

test "x16agu mux 000 writes AGU10 of R and the immediate to e<{S1[2:0], h31}> (895)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x18952416, s1: u32 = 0x03304501, e: u32 = 2, v: u32 = 0xA4B1 }{
        .{},
        .{ .h = 0x98952416, .e = 3 }, // h[31]
        .{ .s1 = 0x03304503, .e = 6 }, // S1[1]
        .{ .h = 0x10952416, .v = 0xF40A }, // h[27]: R2
        .{ .s1 = 0x03304581, .v = 0x7F69 }, // fn 11: y - i
        .{ .s1 = 0x03304401, .v = 0x12A4 }, // fn 8: i
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[2] = 0xE166;
        state.r[3] = 0x920D;
        state.s[3] = 0x77;
        try emu.executeBundle(gpa, &state, &.{ c.h, c.s1, 0x60002000 });
        try testing.expectEqual(c.v, state.e[c.e]);
        try testing.expectEqual(@as(u32, 0), state.s[3]); // clr 0x20 in s<S1[27:24]>
    }
}

test "x17mve writes p<h[14:11]> from cond h[25:22] of R<S2[4:1]> (896)" {
    const gpa = testing.allocator;
    for ([_]struct { s2: u32 = 0x46000006, h: u32 = 0x1F6009D7, p: u32 = 1, v: u8 = 1 }{
        .{},
        .{ .s2 = 0x46000004, .v = 0 }, // S2[1]: R2 = 0xE166 < 0
        .{ .h = 0x1F6011D7, .p = 2 }, // h[12:11]: p2
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[2] = 0xE166;
        state.r[3] = 1;
        state.p[c.p] = 1 - c.v;
        try emu.executeBundle(gpa, &state, &.{ c.h, 0x00050400, c.s2 });
        try testing.expectEqual(c.v, state.p[c.p]);
    }
}

test "ystm's A<h[25:21]> mirrors its e write (897)" {
    const gpa = testing.allocator;
    for ([_]u32{ 0, 0x1234, 0x920D }) |seed| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.r[k] = seed;
        for (1..16) |k| state.e[k] = seed;
        state.a[1] = 0x1498;
        try emu.executeBundle(gpa, &state, &.{ 0x0C224850, 0x44160000 });
        try testing.expectEqual(state.e[4], state.a[1]);
    }
}

test "st takes a class-011 second as its wr; [16:15] = 00 writes e1 alone (899)" {
    const gpa = testing.allocator;
    for ([_]struct { w: u32, store: ?u32 }{
        .{ .w = 0x60238000, .store = 0x1234 }, // s<w[24:21]> = s1 at Y:[old e1]
        .{ .w = 0x60638000, .store = 0x0333 }, // s3
        .{ .w = 0x60000000, .store = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x10AB;
        state.s[1] = 0x1234;
        state.s[3] = 0x0333;
        _ = try emu.runFrom(gpa, &state, &.{ 0x20029D8F, c.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.store, state.getY(0x10AB));
        try testing.expectEqual(@as(?u32, null), state.getY(0x14EC));
        try testing.expectEqual(@as(u32, 0x14EC), state.e[1]);
    }
}

test "pair's X load steps its A with head[25]; dld's e takes Y beside s0 (900)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.a[1] = 0x1496;
    state.s[2] = 5;
    try state.x_mem.put(gpa, 0x1495, 0x2136FA);
    try state.y_mem.put(gpa, 0x2EB1, 0x66A1);
    try emu.executeBundle(gpa, &state, &.{ 0x420292B0, 0x60248100 }); // 0x1FA2
    try testing.expectEqual(@as(u32, 0), state.s[2]); // X:[0x1496]
    try testing.expectEqual(@as(u32, 0x1495), state.a[1]);
    try testing.expectEqual(@as(u32, 0x1495), state.e[0]);
    state.e[1] = 0x2EB1;
    try emu.executeBundle(gpa, &state, &.{0x4050A629}); // 0x1FA5
    try testing.expectEqual(@as(u32, 0x2136FA), state.s[1]);
    try testing.expectEqual(@as(u32, 0x66A1), state.e[3]);
    var flat = emu.State{};
    defer flat.deinit(gpa);
    flat.a[1] = 0x1496;
    try emu.executeBundle(gpa, &flat, &.{ 0x400292B0, 0x60248100 }); // head[25] clear
    try testing.expectEqual(@as(u32, 0x1496), flat.a[1]);
}

test "x03 walks e from s and stores Y at the old e (902)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    for (1..16) |n| state.s[n] = @as(u32, @intCast(n)) * 0x100 + 0x23;
    state.e[3] = 0x13C8;
    const b = [_]u32{ 0x150001C3, 0x00280018, 0x60000000, 0x18000000, 0x00000000, 0x8020A610 }; // 0x1F94
    try emu.executeBundle(gpa, &state, &b);
    try testing.expectEqual(@as(u32, 0x523), state.e[3]);
    try testing.expectEqual(@as(?u32, 0x323), state.getY(0x13C8));
    var neg = emu.State{};
    defer neg.deinit(gpa);
    neg.s[5] = 0x10;
    var nb = b;
    nb[0] = 0x150001C3 ^ (4 << 23); // op 110
    try emu.executeBundle(gpa, &neg, &nb);
    try testing.expectEqual(@as(u32, 0xFFF0), neg.e[3]);
    var off = emu.State{};
    defer off.deinit(gpa);
    off.e[3] = 7;
    nb[0] = 0x150001C2; // w0[0] = 0
    try emu.executeBundle(gpa, &off, &nb);
    try testing.expectEqual(@as(u32, 7), off.e[3]);
    try testing.expectEqual(@as(?u32, null), off.getY(7));
}

test "zmv clears p1..p3 on a zero z (907)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    for (1..16) |i| state.p[i] = 1;
    try emu.executeBundle(gpa, &state, &.{0x4216010A}); // 0x1F6F
    try testing.expectEqual(@as(u8, 0), state.p[1] | state.p[2] | state.p[3]);
    try testing.expectEqual(@as(u8, 1), state.p[4]);
    var nz = emu.State{};
    defer nz.deinit(gpa);
    nz.z = 0x920D;
    nz.p[2] = 1;
    try emu.executeBundle(gpa, &nz, &.{0x4216010A});
    try testing.expectEqual(@as(u8, 1), nz.p[2]);
    try testing.expectError(emu.RunError.UnknownForm, emu.executeBundle(gpa, &state, &.{0x4216010A ^ (1 << 19)}));
}

test "x17sty's stop form walks e from s and stores Y at the old e (908)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[2] = 0x2D00;
    state.s[3] = 1;
    state.e[2] = 0x14D4;
    const b = [_]u32{ 0x000001D7, 0x60010009, 0x64021A0A }; // 0x1F8C
    try emu.executeBundle(gpa, &state, &b);
    try testing.expectEqual(@as(u32, 0x2D00), state.e[2]);
    try testing.expectEqual(@as(?u32, 1), state.getY(0x14D4));
    try testing.expect(!state.stop_now);
    var p1 = emu.State{};
    defer p1.deinit(gpa);
    p1.p[1] = 1;
    try emu.executeBundle(gpa, &p1, &b);
    try testing.expectEqual(@as(?i32, 8), p1.pending_jmp_offset); // 0x1F94 (910)
    try testing.expectEqual(@as(?i32, null), state.pending_jmp_offset);
}

test "x17sty's stop shape runs on with S2[22:18] = 11111 (916)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[2] = 0x700;
    state.s[3] = 0x555;
    try emu.executeBundle(gpa, &state, &.{ 0x28200017, 0x60012D50, 0x61FCB804 }); // 0x381
    try testing.expectEqual(@as(?u32, 0x555), state.getY(0x700));
    try testing.expect(!state.stop_now);
    try testing.expectError(emu.RunError.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x28200017, 0x60012D50, 0x61FCB804 ^ (1 << 19) }));
}

test "x16 class-000 jumps by S2[23:9] on p<S1[8:5]> (917)" {
    const gpa = testing.allocator;
    for ([_]struct { p2: u8, off: ?i32 }{ .{ .p2 = 1, .off = 0x5F }, .{ .p2 = 0, .off = null } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[2] = c.p2;
        try emu.executeBundle(gpa, &state, &.{ 0x44029556, 0x0A100058, 0x0000BE00 }); // 0x39C
        try testing.expectEqual(c.off, state.pending_jmp_offset);
    }
}

test "x18 class 011 code 0x0E ANDs its p with p<w2[31:28]> (921)" {
    const gpa = testing.allocator;
    for ([_]struct { p8: u8, p3: u8 }{ .{ .p8 = 0, .p3 = 0 }, .{ .p8 = 1, .p3 = 1 } }) |c| {
        for ([_]u8{ 0, 1 }) |pre| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.p[8] = c.p8;
            state.p[3] = pre;
            try emu.executeBundle(gpa, &state, &.{ 0x1C029538, 0x00000002, 0x8500C0C0, 0x60000009 }); // 0x42B
            try testing.expectEqual(c.p3, state.p[3]);
        }
    }
}

test "x15pt writes p<S[28:25]> <- p<S[6:3]> AND p<S[14:11]> (921, 911)" {
    const gpa = testing.allocator;
    for ([_][3]u8{ .{ 1, 1, 1 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 0 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[6] = c[0];
        state.p[7] = c[1];
        state.p[8] = 1 - c[2];
        try emu.executeBundle(gpa, &state, &.{ 0x1C05D835, 0x300039B2 }); // 0x427
        try testing.expectEqual(c[2], state.p[8]);
    }
    // 911: 0x1F82 (lut 101, sources p0) sets p2.
    var st2 = emu.State{};
    defer st2.deinit(gpa);
    try emu.executeBundle(gpa, &st2, &.{ 0x9C029EF5, 0x24258500 });
    try testing.expectEqual(@as(u8, 1), st2.p[2]);
}

test "x17 S2 class 101 with S1[22:19] = 0001 jumps on p8 (922)" {
    const gpa = testing.allocator;
    for ([_]struct { p8: u8, off: ?i32 }{ .{ .p8 = 1, .off = -0x1512 }, .{ .p8 = 0, .off = null } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[8] = c.p8;
        try emu.executeBundle(gpa, &state, &.{ 0x00021697, 0x31088000, 0xA1D5DC02 }); // 0x19BC
        try testing.expectEqual(c.off, state.pending_jmp_offset);
    }
}

test "ystm's fn 10 fresh-read form clears p1..p3 (925)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    @memset(state.p[1..5], 1);
    state.r[1] = 0x1000;
    try state.y_mem.put(gpa, 0, 0x920D);
    try emu.executeBundle(gpa, &state, &.{ 0xD0142650, 0x4216010A });
    try testing.expectEqualSlices(u8, &.{ 0, 0, 0, 1 }, state.p[1..5]);
}

test "ystm's fn 10 fresh-read form with h[28] = 0 leaves p (949)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    @memset(state.p[1..4], 1);
    try emu.executeBundle(gpa, &state, &.{ 0xC014C890, 0x4C1602A6 });
    try testing.expectEqualSlices(u8, &.{ 1, 1, 1 }, state.p[1..4]);
}

test "mul13's head ALU slot and X store" {
    // fact 535: own-mul13a, s6 = 0x100, s7 = 0x300: 80bbb1d3 a0283051 gives s7 = 0x200
    // (add m1, b - a); h[23] = 0 (m0) 0x400.  The multiply lands in s6 (0 * 0) after the ALU
    // read s6.  S[16:15] = 11 stores s<S[28:25]> = s5 to X:[A<h[30:26]>]; 01 refuses.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, sw: u32, s7: ?u32, stored: ?u32 = null }{
        .{ .h = 0x80bbb1d3, .sw = 0xa0283051, .s7 = 0x200 },
        .{ .h = 0x803bb1d3, .sw = 0xa0283051, .s7 = 0x400 },
        .{ .h = 0x80bbb1d3 | 25 << 26, .sw = 0xaa29b051, .s7 = 0x200, .stored = 0x555 },
        .{ .h = 0x80bbb1d3, .sw = 0xa028b051, .s7 = null },
        .{ .h = 0x80bbb1d3, .sw = 0xa0283851, .s7 = null }, // S[14:11] = s7, the ALU's dest too
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0x100;
        state.s[7] = 0x300;
        state.s[5] = 0x555;
        state.a[25] = 0x340;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.s7) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[7]);
            try testing.expectEqual(@as(u32, 0), state.s[6]);
            try testing.expectEqual(c.stored, state.readX(0x340));
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19k writes R<w3[10:7]> through AGU10 or AGU10B, beside its store" {
    // fact 534: own-x19kr, R11 = 0xABC: 9c000019 23040111 02c60000 00515d98 gives
    // R11 = y + 1 = 0xABD (w2[25] = 1, AGU10 fn 3); w2[25] = 0 picks AGU10B's -x = -e0 = 0.
    // w1[31:29] is the store source's high bits (s2 here, s10 in the class-101 variant), so
    // the X store to A3 fires alongside.
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, w2: u32, r11: u32, stored: u32 }{
        .{ .w1 = 0x23040111, .w2 = 0x02c60000, .r11 = 0xABD, .stored = 0x222 },
        .{ .w1 = 0x23040111, .w2 = 0x00c60000, .r11 = 0, .stored = 0x222 },
        .{ .w1 = 0xA3040111, .w2 = 0x02c60000, .r11 = 0xABD, .stored = 0xAAA },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[11] = 0xABC;
        state.s[2] = 0x222;
        state.s[10] = 0xAAA;
        state.a[3] = 0x130;
        _ = try emu.runFrom(gpa, &state, &.{ 0x9c000019, c.w1, c.w2, 0x00515d98, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.r11, state.r[11]);
        try testing.expectEqual(@as(?u32, c.stored), state.readX(0x130));
    }
}

test "x19k's ALU form still runs the R write and the store (all slots fire, 534)" {
    // x19.kalu's words with an R destination: s9 <- s4 - s5, R5 <- AGU10 fn 6 (e3 + R6),
    // X:[A12] <- s6.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[4] = 0x500;
    state.s[5] = 0x200;
    state.s[6] = 0x666;
    state.e[3] = 0x100;
    state.r[6] = 0x20;
    state.a[12] = 0x150;
    _ = try emu.runFrom(gpa, &state, &.{ 0x00000019, 0x66069000 | 9 << 8 | 4 << 4, 0x02D80000, 0x06543000 | 5 << 7 | 5 << 3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x300), state.s[9]);
    try testing.expectEqual(@as(u32, 0x120), state.r[5]);
    try testing.expectEqual(@as(?u32, 0x666), state.readX(0x150));
}

test "x19k's store: A<w2[21:17]> reaches A16-A31" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[2] = 0x222;
    state.a[19] = 0x140;
    _ = try emu.runFrom(gpa, &state, &.{ 0x9c000019, 0x23040111, 0x02e60000, 0x00515d98, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(?u32, 0x222), state.readX(0x140));
}

test "x17imm's ALU slot writes s<S1[27:24]> and R<S2[16:13]>" {
    // fact 537: own-x17ia, s2 = 0x100, s5 = 0x900: 000029d7 62024a80 ea012000 gives
    // s2 = s5 - s2 = 0x800 (add m1) from the pre-bundle s5, though the immediate writes
    // s5 <- 5 in the same bundle; R9 takes T too.  S1[11] = 0 (m0) gives 0xA00.  Refused:
    // sel m1 #4 (OPEN), the Y slot, and T aimed at the immediate's s.
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x000029d7, s1: u32, s2: u32 = 0xea012000, want: ?u32 }{
        .{ .s1 = 0x62024a80, .want = 0x800 },
        .{ .s1 = 0x62024280, .want = 0xA00 },
        .{ .h = 0x00002837, .s1 = 0x62224280, .want = 0x2800 }, // code 0x01, m0, #2: (a + b) << 2
        .{ .h = 0x00002d97, .s1 = 0x62424a80, .want = null }, // code 0x2C sel, m1, #4
        .{ .s1 = 0x62024a80, .s2 = 0xea212000, .want = null },
        .{ .s1 = 0x65024a80, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[2] = 0x100;
        state.s[5] = 0x900;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
            try testing.expectEqual(v, state.r[9]);
            try testing.expectEqual(@as(u32, 5), state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x16mul's immediate write and Y store" {
    // fact 538: own-x16imy, e1 = 0x700, s4 = 0xABC: 00500576 14046000 c0200200 gives
    // Y:0x700 = 0xABC and s2 = 0x802B.  S1[14:13] = 00 also writes; 10 refuses; the store
    // reads its source before the immediate lands (h[24:21] = s4); an s0 source refuses.
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x00500576, s1: u32 = 0x14046000, s2: u32 = 0xc0200200, dest: u32 = 2, imm: ?u32 = 0x802B, y: ?u32 = 0xABC }{
        .{},
        .{ .s1 = 0x14040000 },
        .{ .s1 = 0x14044000, .imm = null },
        .{ .h = 0x00900576, .dest = 4, .imm = 0x802B },
        .{ .s1 = 0x10046000, .imm = null },
        .{ .s2 = 0xc0200000, .y = null },
        .{ .s1 = 0x04046000, .y = null }, // S1[28] = 0: no store
        .{ .s1 = 0x14046100, .imm = null }, // S1[11:8]: OPEN
        .{ .s2 = 0xc0200800, .imm = null }, // mul into s2 as well
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x700;
        state.s[4] = 0xABC;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.imm) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[c.dest]);
            try testing.expectEqual(c.y, state.getY(0x700));
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x17agu2 writes its AGU value to R<S1[14:11]> too" {
    // fact 540: own-x17a2r, R11 = 0xABC: 58002c17 2de85d00 00000000 gives
    // R11 = R11 + 5 = 0xAC1; S1[14:11] = 0 leaves R11.
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, r11: u32 }{
        .{ .s1 = 0x2de85d00, .r11 = 0xAC1 },
        .{ .s1 = 0x2de80500, .r11 = 0xABC },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[11] = 0xABC;
        _ = try emu.runFrom(gpa, &state, &.{ 0x58002c17, c.s1, 0x00000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.r11, state.r[11]);
    }
}

test "yld10's S[6:3] is an OPEN s write, not 123's clears" {
    // fact 542: own-yld10k (1c026ff0 e00c2299, S[6:3] = 3) leaves s1 and s2 intact on
    // silicon, and writes some s from an OPEN source: refused.  S[6:3] = 0 still loads.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x1c026ff0, 0xe00c2299, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    var ok = emu.State{};
    defer ok.deinit(gpa);
    _ = try emu.runFrom(gpa, &ok, &.{ 0x1c026ff0, 0xe00c2281, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
}

test "x17sty's R mux 7 is the AGU over the store's own address register" {
    // fact 543: own-x17styr, e5 = 0x720: 500001d7 a802d804 78003a0e gives R11 = 0x720
    // (fn 0, x); fn 2 (a802d904) 0x721; the Y store to e5 runs alongside.  Fn 11 refuses.
    // Those words also write e8 (w1[2:0] = 4) to a value not yet known under mux 7 (658):
    // refused as measured, and run here with w1[2:0] cleared.
    const gpa = testing.allocator;
    for ([_]u32{ 0xa802d804, 0xa802d904 }) |s1| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x500001d7, s1, 0x78003a0e, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
    for ([_]struct { s1: u32, r11: ?u32 }{
        .{ .s1 = 0xa802d800, .r11 = 0x720 },
        .{ .s1 = 0xa802d900, .r11 = 0x721 },
        .{ .s1 = 0xa802dd80, .r11 = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x720;
        state.s[5] = 0x55;
        const r = emu.runFrom(gpa, &state, &.{ 0x500001d7, c.s1, 0x78003a0e, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.r11) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[11]);
            try testing.expectEqual(@as(?u32, 0x55), state.getY(0x720));
        } else try testing.expectError(error.UnknownForm, r);
    }
}

fn runX02(gpa: std.mem.Allocator, state: *emu.State, w: [6]u32) !usize {
    return emu.runFrom(gpa, state, &.{ w[0], w[1], w[2], w[3], w[4], w[5], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
}

test "x02's Y side: Y:[e<w3[30:27]>] with w5[12], else yd" {
    // fact 545: own-x02yd, yd = 0x246 (a yld10 of Y:0x710), Y:0x720 = 0x567, e8 = 0x720.
    // Mode 1 with the w5[29:28] = 11 override: s3 <- yd, or with w5[12] the Y word.  The
    // memory mode (5) without w5[14] takes the same Y side; with w5[14] X:[A18].
    const gpa = testing.allocator;
    for ([_]struct { w4: u32 = 0x00240818, w5: u32, s3: u32 }{
        .{ .w5 = 0x30020628, .s3 = 0x246 },
        .{ .w5 = 0x30021628, .s3 = 0x567 },
        .{ .w4 = 0x00242818, .w5 = 0x30020628, .s3 = 0x246 },
        .{ .w4 = 0x00242818, .w5 = 0x30021628, .s3 = 0x567 },
        // 736, 739: this row has w5[10:9] = 11, so its read slot is xd (0x999 from an earlier
        // read of X:0x340), not memory.
        .{ .w4 = 0x00242818, .w5 = 0x30024628, .s3 = 0x999 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.xd = 0x999;
        state.yd = 0x246;
        state.e[8] = 0x720;
        state.a[18] = 0x340;
        try state.y_mem.put(gpa, 0x720, 0x567);
        try state.x_mem.put(gpa, 0x340, 0x999);
        _ = try runX02(gpa, &state, .{ 0x000001c2, 0x80000040, 0x00014001, 0x40080000, c.w4, c.w5 });
        try testing.expectEqual(c.s3, state.s[3]);
        // R<w3[22:19]> = R1 takes the same Y side (w5[31:30] = 00, 547)
        try testing.expectEqual(@as(u32, if ((c.w5 >> 12) & 1 == 1) 0x567 else 0x246), state.r[1]);
    }
}

test "x02's R slot: AGU10 or AGU10B by w5[11]; the w5[27] carry refuses" {
    // fact 547: own-x02ra, e2 = 0x720: R11 <- AGU10 fn 2 (x + 1) = 0x721; w5[11] = 0
    // takes AGU10B's 0.  w5[27] = 1 is OPEN outside fresh state.
    const gpa = testing.allocator;
    for ([_]struct { w5: u32, r11: ?u32 }{
        .{ .w5 = 0x05800800, .r11 = 0x721 },
        .{ .w5 = 0x05800000, .r11 = 0 },
        .{ .w5 = 0x0D800800, .r11 = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x720;
        state.r[11] = 0xABC;
        const r = runX02(gpa, &state, .{ 0x00000002, 0x00000110, 0x00c00000, 0x105814a7, 0x00000810, c.w5 });
        if (c.r11) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[11]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x02's w5[29:28] = 10 override is x01's T table" {
    // fact 548: own-x02ov's registers, e5 = 0x730 (a), e2 = 0x720 (b): fn 0 gives a,
    // fn 2 b, fn 1 y = R6; fn 3 is unseen and refuses.
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, s3: ?u32 }{
        .{ .w1 = 0x80000040, .s3 = 0x730 },
        .{ .w1 = 0x80000140, .s3 = 0x720 },
        .{ .w1 = 0x800000C0, .s3 = 0x66 },
        .{ .w1 = 0x800001C0, .s3 = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x730;
        state.e[2] = 0x720;
        state.r[6] = 0x66;
        const r = runX02(gpa, &state, .{ 0x000001c2, c.w1, 0x00014001, 0x29080000, 0x00240818, 0x21820628 });
        if (c.s3) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x03's R slot refuses the w5[27] carry (fresh state only, 547)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.r[6] = 0x155;
    // own-x02ra's words as x03, AGU10B fn 14 (y + 1) on R6 into R11
    try testing.expectError(error.UnknownForm, runX02(gpa, &state, .{ 0x00000003, 0x00000710, 0x00c00000, 0x105814a7, 0x00000000, 0x09800000 }));
    var ok = emu.State{};
    defer ok.deinit(gpa);
    ok.r[6] = 0x155;
    _ = try runX02(gpa, &ok, .{ 0x00000003, 0x00000710, 0x00c00000, 0x105814a7, 0x00000000, 0x01800000 });
    try testing.expectEqual(@as(u32, 0x156), ok.r[11]);
}

test "x02's third slot shifts by the head ALU code" {
    // fact 549: own-x02t3, s3 = 0x1230: m = 2 (b) with n = 4 under head code 3 (asr)
    // gives 0x123; code 1 shifts left, read back through a 16-bit Y store as 0x2300 (the upper
    // byte is unpinned); code 6 with w5[4:2] != 0 refuses (628).
    const gpa = testing.allocator;
    for ([_]struct { h: u32, want: ?u32, w5: u32 = 0xc0020090 }{
        .{ .h = 0x01065062, .want = 0x123 },
        .{ .h = 0x01065022, .want = 0x2300 },
        .{ .h = 0x010650c2, .want = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x1230;
        const r = runX02(gpa, &state, .{ c.h, 0x00002048, 0x00001432, 0x00000000, 0xcc000480, c.w5 });
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3] & 0xFFFF);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x00's multiply slot swap reads memory in mode 0" {
    // fact 550: A16 = 0x350 (X = 0x2A4), A3 = 0x320 (X = 0x135).  w4[22] = 1, w4[18] = 0:
    // s8 <- X:[A16] (2 of 2) and s6 <- X:[A3]; with w4[18] = 1 and w5[4], Y:[e<w4[8:5]>]; with
    // w5[4] = 0 (yd, unmeasured) it refuses.  The first bundle's w3 (20000003: a ctl-3 jump
    // and an R9 write, other slots) is zeroed here.
    const gpa = testing.allocator;
    for ([_]struct { w: [7]u32, dest: u32, want: ?u32 }{
        .{ .w = .{ 0x000001c0, 0x00b80000, 0x00404280, 0x00000000, 0x10400001, 0x06000002, 0x00085002 }, .dest = 8, .want = 0x2A4 },
        .{ .w = .{ 0x0018b000, 0x00000000, 0x000c0000, 0x00d00000, 0x0060c0c0, 0x06000032, 0x00060000 }, .dest = 6, .want = 0x135 },
        .{ .w = .{ 0x0018b000, 0x00000000, 0x000c0000, 0x00d00000, 0x0064c0c0, 0x06000032, 0x00060000 }, .dest = 6, .want = 0x777 },
        .{ .w = .{ 0x0018b000, 0x00000000, 0x000c0000, 0x00d00000, 0x0064c0c0, 0x06000022, 0x00060000 }, .dest = 6, .want = null },
        .{ .w = .{ 0x0018b000, 0x00000000, 0x000c0000, 0x00d00000, 0x0020c0c0, 0x06000032, 0x00060000 }, .dest = 6, .want = 0 }, // plain mode 0 writes 0 (762)
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[16] = 0x350;
        state.a[3] = 0x320;
        state.e[6] = 0x700;
        try state.x_mem.put(gpa, 0x350, 0x2A4);
        try state.x_mem.put(gpa, 0x320, 0x135);
        try state.y_mem.put(gpa, 0x700, 0x777);
        const r = emu.runFrom(gpa, &state, &.{ c.w[0], c.w[1], c.w[2], c.w[3], c.w[4], c.w[5], c.w[6], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[c.dest]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x17sty's R mux 0 takes yd" {
    // fact 550: yd = 0x246 (a yld10 of Y:0x710), e3 = 0x720: 500001d7 a802d804
    // 78003a00 gives R11 = 0x246.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.yd = 0x246;
    state.e[5] = 0x720;
    _ = try emu.runFrom(gpa, &state, &.{ 0x500001d7, 0xa802d804, 0x78003a00, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x246), state.r[11]);
}

test "class-bit aliases run as their base class (fact 551)" {
    // Each row: a measured base bundle, and the syllable/mask whose class bits flip without
    // changing the result.  End states must match the base run's.
    const gpa = testing.allocator;
    const aset = try assemble(gpa, "        x12.aset #5, R3\n");
    defer gpa.free(aset);
    const Row = struct { words: []const u32, at: usize, set: u32, clear: u32 };
    const mac = [_]u32{ 0x00000019, 0x40000000, 0x20020000, 0xC0448000 };
    const kal = [_]u32{ 0x01821179, 0x90941000, 0x33e20000, 0x13a00801 };
    const a2r = [_]u32{ 0x58002c17, 0x2de85d00, 0x00000000 };
    for ([_]Row{
        .{ .words = &mac, .at = 2, .set = 0, .clear = 0xE0000000 }, // x19mac w2 class 000
        .{ .words = &mac, .at = 2, .set = 0xA0000000, .clear = 0xE0000000 }, // and 101
        .{ .words = &kal, .at = 2, .set = 0, .clear = 0x60000000 }, // x19k w2 class 001 -> 000
        .{ .words = &kal, .at = 2, .set = 0x40000000, .clear = 0x60000000 }, // -> 010
        .{ .words = &kal, .at = 2, .set = 0x60000000, .clear = 0x60000000 }, // -> 011
        .{ .words = &a2r, .at = 1, .set = 0x60000000, .clear = 0xE0000000 }, // x17agu2 S1 011
        .{ .words = &a2r, .at = 1, .set = 0xA0000000, .clear = 0xE0000000 }, // and 101
        .{ .words = aset[0..2], .at = 1, .set = 0xE0000000, .clear = 0xE0000000 }, // aset12 S 111
    }) |row| {
        var base = emu.State{};
        defer base.deinit(gpa);
        var alias = emu.State{};
        defer alias.deinit(gpa);
        for ([_]*emu.State{ &base, &alias }) |st| {
            st.a[9] = 0x240;
            st.e[9] = 0x790;
            st.r[3] = 0x345;
            st.r[11] = 0xABC;
            st.s[9] = 0x999;
            st.a[17] = 0x360;
        }
        var w: [8]u32 = undefined;
        @memcpy(w[0..row.words.len], row.words);
        const n = row.words.len;
        w[n] = 0x00000017;
        w[n + 1] = 0x31308000;
        w[n + 2] = 0xA0000000;
        _ = try emu.runFrom(gpa, &base, w[0 .. n + 3], 0, 16);
        w[row.at] = (w[row.at] & ~row.clear) | row.set;
        _ = try emu.runFrom(gpa, &alias, w[0 .. n + 3], 0, 16);
        try testing.expectEqual(base.s, alias.s);
        try testing.expectEqual(base.a, alias.a);
        try testing.expectEqual(base.r, alias.r);
        try testing.expectEqual(base.readX(0x360), alias.readX(0x360));
    }
}

test "own-x19kal: x19k with w2 class 001" {
    // fact 551: e9 = 0x790: 01821179 90941000 33e20000 13a00801 gives s12 = 0x790.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[9] = 0x790;
    _ = try emu.runFrom(gpa, &state, &.{ 0x01821179, 0x90941000, 0x33e20000, 0x13a00801, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x790), state.s[12]);
}

test "0x0F's e write at index 0 is an s <- e move instead" {
    // fact 552: e3 = 0x730: s3 <- e3 + 1 = 0x731 with w1[2], e3 itself without; a
    // nonzero index keeps 279's e <- #imm and no s write.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w1: u32, s3: u32, e2: u32 }{
        .{ .h = 0x0060000F, .w1 = 0x20060004, .s3 = 0x731, .e2 = 0 },
        .{ .h = 0x0060000F, .w1 = 0x20060000, .s3 = 0x730, .e2 = 0 },
        .{ .h = 0x4060246F, .w1 = 0x20060004, .s3 = 0xABC, .e2 = 0x123 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[3] = 0x730;
        state.s[3] = 0xABC;
        _ = try emu.runFrom(gpa, &state, &.{ c.h, c.w1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.s3, state.s[3]);
        try testing.expectEqual(c.e2, state.e[2]);
    }
}

test "0x0F with a class-001 second sets p<w1[28:25]> when R<head[28:25]> and R<w1[12:9]> are 0" {
    // fact 827: a zero test on both registers, not parity (2 and 0x100 give 0); other p untouched.
    const gpa = testing.allocator;
    for ([_]struct { ra: u32, rb: u32, p: u8 }{
        .{ .ra = 0, .rb = 0, .p = 1 },     .{ .ra = 1, .rb = 0, .p = 0 },
        .{ .ra = 0, .rb = 1, .p = 0 },     .{ .ra = 2, .rb = 0, .p = 0 },
        .{ .ra = 0, .rb = 0x100, .p = 0 }, .{ .ra = 0xFFFF, .rb = 0, .p = 0 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[5] = c.ra;
        state.r[7] = c.rb;
        state.p[12] = 1 - c.p;
        state.p[13] = 1;
        _ = try emu.runFrom(gpa, &state, &.{ 0x4A02110F, 0x38000E00, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.p, state.p[12]);
        try testing.expectEqual(@as(u8, 1), state.p[13]);
        try testing.expectEqual(@as(u32, 0x1088), state.e[2]);
    }
}

test "0x10's class-011 pair loads R6 from Y:[e<S[28:25]>] with S[19:16] = 6" {
    // fact 828: head[23:22] does not move the R; the e pointer does not step.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[3] = 0x2C01;
    try state.y_mem.put(gpa, 0x2C01, 0x5678);
    _ = try emu.runFrom(gpa, &state, &.{ 0x01400010, 0x66060000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x5678), state.r[6]);
    try testing.expectEqual(@as(u32, 0x2C01), state.e[3]);
}

test "0x10's class-011 pair writes e<S[6:3]> (fact 835)" {
    const gpa = testing.allocator;
    for ([_]u32{ 5, 13 }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        _ = try emu.runFrom(gpa, &state, &.{ 0x0020E1B0, 0x60448000 | (k << 3), 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x70D), state.e[k]);
        try testing.expectEqual(@as(u32, 0), state.e[if (k == 5) 13 else 5]);
    }
}

test "0x10's class-011 pair refuses S[19:16] = 5, whose R value is unmapped" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x01400010, 0x66050000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "lde with e index 0 loads A<[25:21]> (829)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    // [29:26] = 0, [25:21] = 13, imm 0x14A0; then an e-form with [25:21] = 13 leaves A13.
    const lda13: u32 = 0x40000006 | (13 << 21) | (0x14A0 << 5);
    const lde3: u32 = 0x40000006 | (3 << 26) | (13 << 21) | (0x0777 << 5);
    _ = try emu.runFrom(gpa, &state, &.{ lda13, lde3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x14A0), state.a[13]);
    try testing.expectEqual(@as(u32, 0x0777), state.e[3]);
}

test "x12agu runs the AGU for every head[31:29]" {
    // fact 562: e6 = 0x760; cbe012d2 23780001 (and with w1[28] set, 33780001) give e9,
    // s3 (2*1 + h[31]) and R11 all 0x760 (AGU10 fn 0, x = e6).  fn 7 (measured on another row):
    // s1 = R4 = e2 = e5 - R3, DERIVED word.
    const gpa = testing.allocator;
    for ([_]u32{ 0x23780001, 0x33780001 }) |w1| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[6] = 0x760;
        _ = try emu.runFrom(gpa, &state, &.{ 0xcbe012d2, w1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x760), state.e[9]);
        try testing.expectEqual(@as(u32, 0x760), state.s[3]);
        try testing.expectEqual(@as(u32, 0x760), state.r[11]);
    }
    var img = emu.State{};
    defer img.deinit(gpa);
    img.e[5] = 0x500;
    img.r[3] = 0x123;
    _ = try emu.runFrom(gpa, &img, &.{ 0xa3ee64b2, 0x36980000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x500 - 0x123), img.s[1]);
    try testing.expectEqual(@as(u32, 0x500 - 0x123), img.e[2]);
    try testing.expectEqual(@as(u32, 0x500 - 0x123), img.r[4]);
}

test "x17 S1 class 000 + S2 class 111: the move and its Y store" {
    // fact 554: S2[4:3] = 11 stores Y:[e<S2[24:21]>] <- s<2*S2[2:0]> (the value before
    // the move's write), beside s<S1[27:24]> <- s<S2[28:25]>.  S2[4:3] = 01 and source 0 are
    // OPEN.  Here s6 <- s5 and the store reads the old s6.
    const gpa = testing.allocator;
    for ([_]struct { s2: u32, stored: ?u32, ok: bool }{
        .{ .s2 = 0xEA80001B, .stored = 0x0666, .ok = true },
        .{ .s2 = 0xEA800003, .stored = null, .ok = true },
        .{ .s2 = 0xEA80000B, .stored = null, .ok = false },
        .{ .s2 = 0xEA800018, .stored = null, .ok = false },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x555;
        state.s[6] = 0x10666;
        state.e[4] = 0x740;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000617, 0x06000000, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (!c.ok) {
            try testing.expectError(error.UnknownForm, r);
            continue;
        }
        _ = try r;
        try testing.expectEqual(@as(u32, 0x555), state.s[6]);
        try testing.expectEqual(c.stored, state.getY(0x740));
    }
}

test "own-x19al5 and own-x19aa5: x19alu and x19aagu with w2 class 001" {
    // fact 555, 556: X:[A2] = 0x2A4 gives s5 = 0x2A4 through x19alu's X mux; Y:[e4] =
    // 0x1234 gives s5 = 0x3400 through x19aagu's byte load (342, w2[12:11] = 2).
    const gpa = testing.allocator;
    var al = emu.State{};
    defer al.deinit(gpa);
    al.a[2] = 0x340;
    try al.x_mem.put(gpa, 0x340, 0x2A4);
    _ = try emu.runFrom(gpa, &al, &.{ 0x80000479, 0x03055008, 0x28a8220a, 0x82000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x2A4), al.s[5]);

    var aa = emu.State{};
    defer aa.deinit(gpa);
    aa.e[4] = 0x720;
    try aa.y_mem.put(gpa, 0x720, 0x1234);
    _ = try emu.runFrom(gpa, &aa, &.{ 0x00000019, 0x06a30500, 0x28401000, 0x49508014, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x3400), aa.s[5]);
    // The other byte-load mux values are OPEN.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x00000019, 0x06a30500, 0x28400800, 0x49508014, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "own-b16R: the class-100 row's R write muxes on S1[24]" {
    // fact 558: s5 = 0x123; 0002a816 26005800 8a000000 gives R11 = 0x246 (the cell,
    // s5 << 1); with S1[24] set, R11 = #head[20:5] = 0x1540.  589: with no s destination the
    // cell still runs and feeds R (head[30] = 0, no extra S2 bits only).
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x0002a816, s1: u32, s2: u32 = 0x8a000000, want: ?u32 }{
        .{ .s1 = 0x26005800, .want = 0x246 },
        .{ .s1 = 0x27005800, .want = 0x1540 },
        .{ .s1 = 0x20005800, .want = 0x246 }, // mx 0 c 0, n = 1
        .{ .s1 = 0x00005800, .want = 0x123 }, // n = 0
        .{ .h = 0x0802a816, .s1 = 0x40005800, .want = 0x49 }, // mx 1 c 0, n = 2: rounded
        .{ .h = 0x1002a816, .s1 = 0x60005800, .want = 0x126 }, // mx 2 c 0, n = 3
        .{ .h = 0x1002a816, .s1 = 0x60005800, .s2 = 0x8a000002, .want = 0x120 }, // mx 2 c 2
        .{ .s1 = 0x20005800, .s2 = 0x8a000001, .want = 0x4600 }, // mx 0 c 1: a << (n + 8)
        .{ .h = 0x4002a816, .s1 = 0x20005800, .want = null }, // head[30] = 1: OPEN
        .{ .s1 = 0x20005800, .s2 = 0x8a000030, .want = null }, // extra S2 bits at head[26:25] = 0
        .{ .h = 0x0202a816, .s1 = 0x20005800, .want = null }, // head[26:25] = 01: OPEN
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x123;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[11]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "wedge gates and post-op codes refused (fact 559, 564)" {
    const gpa = testing.allocator;
    for ([_][]const u32{
        // xld10 with S1[18] cleared (own-xld10's 80a17dd0 60640181).
        &.{ 0x80A17DD0, 0x60600181 },
        // x16alu (S2 class 111) post-op codes 39 and 44 on op 4 with S1[15] = 1: OPEN (564).
        &.{ 0x4E600016, 0x0284d550, 0xe708a200 },
        &.{ 0x58600016, 0x0284d550, 0xe708a200 },
    }) |w| {
        var state = emu.State{};
        defer state.deinit(gpa);
        var b: [6]u32 = undefined;
        @memcpy(b[0..w.len], w);
        b[w.len] = 0x00000017;
        b[w.len + 1] = 0x31308000;
        b[w.len + 2] = 0xA0000000;
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, b[0 .. w.len + 3], 0, 16));
    }
    // The ungated xld10 still runs.
    var ok = emu.State{};
    defer ok.deinit(gpa);
    ok.a[6] = 0x330;
    try ok.x_mem.put(gpa, 0x330, 0x777);
    _ = try emu.runFrom(gpa, &ok, &.{ 0x80A17DD0, 0x60640181, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x777), ok.s[3]);
}

test "own-dld13: dld's h[13] is inert without a Y destination" {
    // fact 559: 4091a009 (h[13] set) reads X:[A3] = 0x135 like 40918009.
    const gpa = testing.allocator;
    for ([_]u32{ 0x4091a009, 0x40918009 }) |h| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[3] = 0x320;
        try state.x_mem.put(gpa, 0x320, 0x135);
        _ = try emu.runFrom(gpa, &state, &.{ h, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x135), state.s[2]);
    }
}

test "x16alu's post-op code (fact 563)" {
    const gpa = testing.allocator;
    // own-x16ovf: code 3, op 4, n = 4 on the exact sum: (0x5505 + 0x4404) asr 4 = 0x0990.
    var ovf = emu.State{};
    defer ovf.deinit(gpa);
    ovf.s[3] = 0x5505;
    ovf.s[5] = 0x4404;
    _ = try emu.runFrom(gpa, &ovf, &.{ 0x06600016, 0x02845550, 0xe708a240, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x0990), ovf.s[5]);
    // The operands are 16-bit signed: (0x8808 + 0x6606) asr 1 = 0xF707.
    var neg = emu.State{};
    defer neg.deinit(gpa);
    neg.s[3] = 0x8808;
    neg.s[5] = 0x6606;
    _ = try emu.runFrom(gpa, &neg, &.{ 0x06600016, 0x02845550, 0xe708a210, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xF707), neg.s[5]);
    // Plain code 15 keeps every op, here op 4 with S1[15] = 1 (561's bundle): a - b.
    var pl = emu.State{};
    defer pl.deinit(gpa);
    pl.s[3] = 0x300;
    pl.s[5] = 0x123;
    _ = try emu.runFrom(gpa, &pl, &.{ 0x1e600016, 0x0284d550, 0xe708a200, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1DD), pl.s[5]);
    // 561's miss row: code 4, op 5, n = 1: (0x3303 + 1) >> 1 = 0x1982 into s8.
    var miss = emu.State{};
    defer miss.deinit(gpa);
    miss.s[3] = 0x3303;
    _ = try emu.runFrom(gpa, &miss, &.{ 0x08c41076, 0x00455800, 0xe6960010, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1982), miss.s[8]);
    // 564's own program: code 28, op 5 with S1[15] = 1: a & n = 0x330E & 11 = 0xA.
    var and_n = emu.State{};
    defer and_n.deinit(gpa);
    and_n.s[3] = 0x330E;
    _ = try emu.runFrom(gpa, &and_n, &.{ 0x38600016, 0x0285d550, 0xe708a2b0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xA), and_n.s[5]);
}

test "x16Post table cells" {
    // Ops 4 (add) and 5 at S1[15] = 0.
    const P = struct {
        fn f(code: u32, add: bool, a: i64, b: i64, n: u6) ?i64 {
            return emu.x16PostForTest(if (add) 4 else 5, 0, code, a, b, n);
        }
    }.f;
    const Q = emu.x16PostForTest;
    try testing.expectEqual(@as(?i64, 0x4404), P(3, true, 0x3303, 0x5505, 1)); // exact sum
    try testing.expectEqual(@as(?i64, -0x08F9), P(3, true, -0x77F8, 0x6606, 1)); // 0xF707
    try testing.expectEqual(@as(?i64, (0x3303 ^ 0x5505) << 1), P(22, true, 0x3303, 0x5505, 1));
    try testing.expectEqual(@as(?i64, (0x3303 | 0x5505) << 1), P(22, false, 0x3303, 0x5505, 1));
    try testing.expectEqual(@as(?i64, 5), P(40, true, 1, 2, 5));
    try testing.expectEqual(@as(?i64, 0x3303), P(48, true, 0x3303, 2, 3));
    try testing.expectEqual(@as(?i64, 0x3303 << 3), P(48, false, 0x3303, 2, 3));
    try testing.expectEqual(@as(?i64, 0x3303 + 3), P(9, true, 0x3303, 2, 3));
    try testing.expectEqual(@as(?i64, 0x3303 - 3), P(28, true, 0x3303, 2, 3));
    try testing.expectEqual(@as(?i64, 2 - 0x3303), P(16, true, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, -0x3303), P(16, false, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, -(0x3303 + 2) - 2), P(20, true, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 0), P(7, true, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, null), P(7, false, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, null), P(62, false, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 0), P(62, true, 0x3303, 2, 0)); // 564 corrects 563
    try testing.expectEqual(@as(?i64, 0x3303 << 2), P(58, true, 0x3303, 2, 2)); // a, not a + b
    // 564's other rows, and n in place of b on 9, 10 and 28.
    try testing.expectEqual(@as(?i64, 0x3303 + 5), P(9, true, 0x3303, 2, 5));
    try testing.expectEqual(@as(?i64, 0x3303 + 1), Q(5, 1, 11, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 0x3303 & 0xB), Q(5, 1, 28, 0x3303, 2, 11));
    try testing.expectEqual(@as(?i64, 0x3303 - 2), Q(4, 1, 11, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, ~@as(i64, 0x3303 ^ 2)), Q(4, 1, 29, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, null), Q(4, 1, 39, 0x3303, 2, 0)); // x99 and own disagree
    try testing.expectEqual(@as(?i64, null), Q(4, 1, 44, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, -0x3303), Q(7, 0, 11, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 2), Q(7, 0, 29, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 2 - 0x3303), Q(6, 1, 11, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, null), Q(6, 1, 29, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 0x3303 >> 3), Q(6, 1, 58, 0x3303, 2, 3)); // 58/62 shift right
    try testing.expectEqual(@as(?i64, 0x3303 >> 3), Q(7, 0, 62, 0x3303, 2, 3));
    try testing.expectEqual(@as(?i64, null), Q(1, 0, 11, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, null), Q(6, 0, 0, 0x3303, 2, 0));
    try testing.expectEqual(@as(?i64, 3), P(45, true, 0, 5, 1)); // b asr n, rounded
    try testing.expectEqual(@as(?i64, 2), P(44, true, 0, 1, 1));
    try testing.expectEqual(@as(?i64, 0x3303 ^ 0x5505), P(29, true, 0x3303, 0x5505, 0));
    try testing.expectEqual(@as(?i64, 0x5505), P(35, true, 0x3303, 0x5505, 0));
    try testing.expectEqual(@as(?i64, 0x3303), P(51, true, 0x3303, 0x5505, 2));
    try testing.expectEqual(@as(?i64, 0x3303 >> 2), P(51, false, 0x3303, 0x5505, 2));
    try testing.expectEqual(@as(?i64, 0x3303 - 0x5505), P(18, true, 0x3303, 0x5505, 0));
    try testing.expectEqual(@as(?i64, 0x3303), P(18, false, 0x3303, 0x5505, 0));
    try testing.expectEqual(@as(?i64, null), P(54, false, 0x3303, 0x5505, 0));
}

test "x03 with w5[1] = 0: w5[21] picks a fresh Y read or yd (fact 565)" {
    // Y:0x710 = 0x1234, Y:0x720 = 0x0BEE: a read at 0x720, then w5[21] = 0 at e2 = 0x710 gives
    // s5 = 0x0BEE.  Modes 3/4 lane yd the same way (yd[7:0] << 8).
    const gpa = testing.allocator;
    const rd = try assemble(gpa, "        x03.yrd s5, e3, #5, #0\n");
    defer gpa.free(rd);
    try testing.expectEqual(@as(u32, 0), (rd[5] >> 1) & 1);
    try testing.expectEqual(@as(u32, 1), (rd[5] >> 21) & 1);
    var b: [15]u32 = undefined;
    @memcpy(b[0..6], rd[0..6]);
    @memcpy(b[6..12], rd[0..6]);
    b[6 + 3] = (b[6 + 3] & ~@as(u32, 0x78000000)) | (2 << 27); // e2
    b[6 + 5] &= ~@as(u32, 1 << 21);
    b[12] = 0x00000017;
    b[13] = 0x31308000;
    b[14] = 0xA0000000;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[3] = 0x720;
    state.e[2] = 0x710;
    try state.y_mem.put(gpa, 0x710, 0x1234);
    try state.y_mem.put(gpa, 0x720, 0x0BEE);
    _ = try emu.runFrom(gpa, &state, &b, 0, 16);
    try testing.expectEqual(@as(u32, 0x0BEE), state.s[5]);
    // With w5[1] = 1 the slot reads X whatever w5[21] says (own-x03xrd's shape).
    var x = emu.State{};
    defer x.deinit(gpa);
    x.e[3] = 0x720;
    x.a[0] = 0;
    try x.y_mem.put(gpa, 0x720, 0x0BEE);
    try x.x_mem.put(gpa, 0, 0x777);
    var bx: [9]u32 = undefined;
    @memcpy(bx[0..6], rd[0..6]);
    bx[5] |= 2;
    bx[6] = 0x00000017;
    bx[7] = 0x31308000;
    bx[8] = 0xA0000000;
    _ = try emu.runFrom(gpa, &x, &bx, 0, 16);
    try testing.expectEqual(@as(u32, 0x777), x.s[5]);
}

test "agu_xr's once-only wrap on x02's A sum and R slot (fact 566, 567)" {
    // A9 = 0x200, R9 = 0x50 (y), R3 = 0x40 (M, w3[18:15] = 3): fn 2 sum 0x250 wraps to 0x210 in
    // A1 (w5[27] = 0) and in R9 (w5[27:26] = 10); M field 0 leaves 0x250.
    const gpa = testing.allocator;
    for ([_]struct { w3: u32, w5: u32, a1: u32, r9: u32 }{
        .{ .w3 = 0x00018009, .w5 = 0x02400000, .a1 = 0x210, .r9 = 0x50 },
        .{ .w3 = 0x00000009, .w5 = 0x02400000, .a1 = 0x250, .r9 = 0x50 },
        .{ .w3 = 0x00498009, .w5 = 0x0A400000, .a1 = 0x210, .r9 = 0x210 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x200;
        state.r[9] = 0x50;
        state.r[3] = 0x40;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0, 0x00820000, c.w3, 0, c.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.a1, state.a[1]);
        try testing.expectEqual(c.r9, state.r[9]);
    }
    // M = 9: sum 0x10 gives 7, 0x800F gives 0x8018, 0, 6 and 8 stay.
    for ([_][2]u32{ .{ 0x10, 7 }, .{ 0x800F, 0x8018 }, .{ 0, 0 }, .{ 6, 6 }, .{ 8, 8 }, .{ 9, 0 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = c[0];
        state.r[3] = 9;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0, 0x00820000, 0x00018009, 0, 0x00000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], state.a[1]);
    }
    // own-x02wrap (626): A1 <- (sum mod M) + A9, the add unwrapped.  fn 8, A9 = 0x200, R9 = 0x50.
    for ([_][2]u32{ .{ 0x300, 0x450 }, .{ 0x40, 0x410 } }) |c| {
        var w = emu.State{};
        defer w.deinit(gpa);
        w.a[9] = 0x200;
        w.r[9] = 0x50;
        w.r[3] = c[0];
        _ = try emu.runFrom(gpa, &w, &.{ 0x00000002, 0, 0x02000000, 0x00018529, 0, 0x02400000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], w.a[1]);
    }
}

test "own-x16rmod: x16alu's R slot wraps by R<S2[5:2]> (fact 568)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.a[1] = 0x200;
    state.r[5] = 0x50;
    state.r[12] = 0x40;
    _ = try emu.runFrom(gpa, &state, &.{ 0x1c0047f6, 0x43948440, 0x50002231, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x210), state.r[1]);
    // -A1 (fn 4) with A1 = 0x110 and M = 0x300C adds M: 0x2EFC.
    var neg = emu.State{};
    defer neg.deinit(gpa);
    neg.a[1] = 0x110;
    neg.r[12] = 0x300C;
    _ = try emu.runFrom(gpa, &neg, &.{ 0x1c0047f6, 0x43948440, 0x50002131, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x2EFC), neg.r[1]);
}

test "the Y read forms that set yd (fact 571)" {
    // 571's template: yd primed to 0x1234, the form reads Y:0x720 = 0x0BEE, then an x03 with
    // w5[1] = w5[21] = 0 copies yd into s5.  Every e points at 0x720.
    const gpa = testing.allocator;
    const rd = try assemble(gpa, "        x03.yrd s5, e3, #5, #0\n");
    defer gpa.free(rd);
    var yd_rd: [6]u32 = rd[0..6].*;
    yd_rd[5] &= ~@as(u32, 1 << 21);
    for ([_][]const u32{
        &.{ 0x00a00010, 0xe6420080 }, // yld10
        &.{ 0x40429d57, 0x90028010, 0xa0000004 }, // x17 S2 class 101, S1[28] = 1
        &.{ 0x44400016, 0x00071700, 0xe0c40000 }, // x16alu mux Y read (own-ydx16)
        &.{ 0x00000002, 0, 0x0000a000, 0x91880000, 0x00004001, 0x00001800 }, // x02 R slot, w5[12]
        &.{ 0x10002019, 0xa4020800, 0x08003000, 0x44528000 }, // x19aagu byte load
    }) |form| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.e[k] = 0x720;
        state.yd = 0x1234;
        try state.y_mem.put(gpa, 0x720, 0x0BEE);
        var b: [16]u32 = undefined;
        @memcpy(b[0..form.len], form);
        @memcpy(b[form.len..][0..6], &yd_rd);
        b[form.len + 6] = 0x00000017;
        b[form.len + 7] = 0x31308000;
        b[form.len + 8] = 0xA0000000;
        _ = try emu.runFrom(gpa, &state, b[0 .. form.len + 9], 0, 32);
        try testing.expectEqual(@as(u32, 0x0BEE), state.s[5]);
    }
}

test "x00's jump controls take sx15({w2[0], w1[31:18]}) (fact 575, 772)" {
    const gpa = testing.allocator;
    for ([_]struct { ctl: u32, w1: u32, w2: u32 = 0, w4: u32 = 0, off: ?i32 }{
        .{ .ctl = 3, .w1 = 0xFFD80000, .w2 = 1, .off = -10 }, // the measured q - 10
        .{ .ctl = 3, .w1 = 0xFFD80000, .off = 16374 }, // without the sign: far forward
        .{ .ctl = 1, .w1 = 0x0C << 18, .off = 12 },
        .{ .ctl = 0xE, .w1 = (1 << 26) | (4 << 18), .off = 260 }, // +256 per w1[26]
        .{ .ctl = 0x17, .w1 = 0x3FFF << 18, .w2 = 1, .off = -1 },
        .{ .ctl = 3, .w1 = 0x0C << 18, .w4 = 1 << 28, .off = null }, // 772: p1 = 0, not taken
        .{ .ctl = 1, .w1 = 0x0C << 18, .w4 = 1 << 28, .off = 12 }, // 882: control 1 ignores c
        .{ .ctl = 0xE, .w1 = 0x0C << 18, .w4 = 1 << 28, .off = null }, // condition under control 0xE: unmeasured
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.executeBundle(gpa, &state, &.{ 0, c.w1, c.w2, c.ctl, c.w4, 0, 0 });
        if (c.off) |o| {
            try r;
            try testing.expectEqual(@as(?i32, o), state.pending_jmp_offset);
        } else if (c.ctl == 3) {
            try r;
            try testing.expectEqual(@as(?i32, null), state.pending_jmp_offset);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x00 writes its immediate to e<w1[6:3]> (fact 830, 831)" {
    const gpa = testing.allocator;
    for ([_]u32{ 4, 5 }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        // ctl 3 on p1 = 0: the jump falls through, as in the measured bundle.
        try emu.executeBundle(gpa, &state, &.{ 0xFD94 << 11, k << 3, 0, 3, 1 << 28, 0, 0 });
        try testing.expectEqual(@as(u32, 0xFD94), state.e[k]);
        try testing.expectEqual(@as(u32, 0), state.e[9 - k]);
    }
}

test "x00's 842 selector doubles ystm until the halt; rop runs once; the next run is single" {
    const gpa = testing.allocator;
    const sel = [_]u32{ 0, 0, 1 << 14, 0, 2 << 1, 1 << 3, 0 };
    const ystm = [_]u32{ 0x24142450, 0x42172000 };
    const inc_r1 = 0x405C2007; // rop s0, R1, e0, R1, #14: R1 <- R1 + 1
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // Under the selector: one ystm written, two run; one rop written, one runs.
    var a = emu.State{};
    defer a.deinit(gpa);
    a.e[2] = 3;
    a.r[2] = 5;
    const prog_a = sel ++ ystm ++ halt;
    _ = try emu.runFrom(gpa, &a, &prog_a, 0, 32);
    // Reference: the same ystm written twice, no selector.
    var b = emu.State{};
    defer b.deinit(gpa);
    b.e[2] = 3;
    b.r[2] = 5;
    const prog_b = ystm ++ ystm ++ halt;
    _ = try emu.runFrom(gpa, &b, &prog_b, 0, 32);
    try testing.expectEqual(b.e[2], a.e[2]);
    try testing.expectEqual(b.r[1], a.r[1]);
    // rop under the selector runs once.
    var c = emu.State{};
    defer c.deinit(gpa);
    c.r[1] = 5;
    const prog_c = sel ++ [_]u32{inc_r1} ++ halt;
    _ = try emu.runFrom(gpa, &c, &prog_c, 0, 32);
    try testing.expectEqual(@as(u32, 6), c.r[1]);
    // The flag ended with the halt: a new run of the ystm is single.
    var d = emu.State{};
    defer d.deinit(gpa);
    d.e[2] = 3;
    d.r[2] = 5;
    _ = try emu.runFrom(gpa, &d, &prog_a, 0, 32);
    _ = try emu.runFrom(gpa, &d, &prog_a, 7, 32);
    var e = emu.State{};
    defer e.deinit(gpa);
    e.e[2] = 3;
    e.r[2] = 5;
    const prog_e = ystm ++ ystm ++ ystm ++ halt;
    _ = try emu.runFrom(gpa, &e, &prog_e, 0, 32);
    try testing.expectEqual(e.e[2], d.e[2]);
}

test "x00 without the full 842 selector leaves the doubling off" {
    const gpa = testing.allocator;
    for ([_][3]u32{ .{ 1 << 14, 2 << 1, 1 << 3 }, .{ 3 << 14, 2 << 1, 1 << 3 }, .{ 1 << 14, 3 << 1, 1 << 3 }, .{ 1 << 14, 2 << 1, 0 } }, 0..) |f, k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try emu.executeBundle(gpa, &state, &.{ 0, 0, f[0], 0, f[1], f[2], 0 });
        try testing.expectEqual(k == 0, state.x00_double);
    }
}

test "yld10 also writes e<2*S[2:0] + head[31]> <- #head[20:5] (fact 844, 845)" {
    const gpa = testing.allocator;
    // own-yld10-imm-e5 / -e8: 0x1234 lands in exactly e5 / e8.
    for ([_]struct { h: u32, s: u32, e: usize }{
        .{ .h = 0x80A24690, .s = 0xE6420082, .e = 5 },
        .{ .h = 0x00A24690, .s = 0xE6420084, .e = 8 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try emu.executeBundle(gpa, &state, &.{ c.h, c.s });
        for (1..16) |k| try testing.expectEqual(@as(u32, if (k == c.e) 0x1234 else 0), state.e[k]);
    }
}

test "0x10 heads under S classes 100 and 000 run their ALU word into s<h[18:15]> (fact 846)" {
    const gpa = testing.allocator;
    // Class 100 (x10alu): s15 <- s12 << 2 (code 2, m 2, #2); S[24:21] = 0, no R write.
    for ([_][2]u32{ .{ 1, 4 }, .{ 3, 0xC }, .{ 5, 0x14 }, .{ 0x100, 0x400 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[12] = c[0];
        state.s[15] = 7;
        try emu.executeBundle(gpa, &state, &.{ 0xF1178050, 0x80000000 });
        try testing.expectEqual(c[1], state.s[15]);
    }
    // Class 000 (bcc on p13 = 0): falls through and s14 <- s15 (code 0x1F).
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[14] = 0xEEE;
    state.s[15] = 0xFFF;
    try emu.executeBundle(gpa, &state, &.{ 0x00077BF0, (13 << 15) | 1 });
    try testing.expectEqual(@as(u32, 0xFFF), state.s[14]);
    try testing.expectEqual(@as(?i32, null), state.pending_jmp_offset);
}

test "no A index is masked to 4 bits (A has 32 registers)" {
    const src = @embedFile("emu.zig");
    var i: usize = 0;
    while (std.mem.indexOfPos(u8, src, i, "state.a[")) |at| : (i = at + 1) {
        const close = std.mem.indexOfScalarPos(u8, src, at, ']') orelse break;
        try testing.expect(std.mem.indexOf(u8, src[at..close], "& 0xF)") == null);
        try testing.expect(!std.mem.endsWith(u8, src[at..close], "& 0xF"));
    }
}

test "A16 and above through xld10 and x19aagu" {
    const gpa = testing.allocator;
    // xld10: A<4*S1[2:0] + h[31:30]> = A31 (own-xld10's shape with both fields maxed).
    var x = emu.State{};
    defer x.deinit(gpa);
    x.a[31] = 0x330;
    try x.x_mem.put(gpa, 0x330, 0x777);
    _ = try emu.runFrom(gpa, &x, &.{ 0xC0A17DD0, 0x60640187, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x777), x.s[3]);
    // x19aagu: A<w1[7:3]> = A17 <- AGUA(A<w3[19:15]> = A18) (own-x19aagu's fn 2, i = 16).
    var g = emu.State{};
    defer g.deinit(gpa);
    g.a[18] = 0x100;
    _ = try emu.runFrom(gpa, &g, &.{ 0x00000019, 0x06000088, 0x00000000, 0x40290008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expect(g.a[17] != 0);
    try testing.expectEqual(@as(u32, 0), g.a[1]);
}

test "x19aagu's other slots (fact 271, 335, 341, 342)" {
    const gpa = testing.allocator;
    // own-x19aagp: A2 = 0x100, R2 = 0x20: A8 <- A2 + R2 + 1 (main, i = 1), A2 <- A2 + R2 (copy).
    var p = emu.State{};
    defer p.deinit(gpa);
    p.a[2] = 0x100;
    p.r[2] = 0x20;
    _ = try emu.runFrom(gpa, &p, &.{ 0x10000019, 0x06020040, 0x80000000, 0x40210000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x121), p.a[8]);
    try testing.expectEqual(@as(u32, 0x120), p.a[2]);
    // own-x19aagu5: w1[25] = 0 (no main write); the copy A2 <- A5 = 0x345.
    var c = emu.State{};
    defer c.deinit(gpa);
    c.a[5] = 0x345;
    _ = try emu.runFrom(gpa, &c, &.{ 0x093A5019, 0xA4020010, 0x00000007, 0x40528000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x345), c.a[2]);
    // own-x19ax: s2 = 0x0BEE, A10 = 0x345: X:[A<w2[26:22]> = A13] <- sx24 s2 with w1[8], w1[10].
    var x = emu.State{};
    defer x.deinit(gpa);
    x.s[2] = 0x0BEE;
    x.a[13] = 0x345;
    x.a[10] = 0x345;
    _ = try emu.runFrom(gpa, &x, &.{ 0x00000019, 0x06400548, 0x03400000, 0x40548001, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(?u32, 0x0BEE), x.readX(0x345));
    // own-x19aar: R2 = 0x500: R1 <- AGU10[10](y = R<h[30:27]> = R2, i = h[26:11] = 4) = 0x504.
    var r = emu.State{};
    defer r.deinit(gpa);
    r.r[2] = 0x500;
    r.e[2] = 0x700;
    _ = try emu.runFrom(gpa, &r, &.{ 0x10002019, 0xa4020800, 0x08003000, 0x44528000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x504), r.r[1]);
    // Without w1[25] the main write drops; with w1[15] the copy takes s<w2[21:18]>.
    for ([_]struct { w1: u32, w2: u32, a8: u32, a2: u32 }{
        .{ .w1 = 0x04020040, .w2 = 0x80000000, .a8 = 0, .a2 = 0x120 },
        .{ .w1 = 0x06028040, .w2 = 0x800C0000, .a8 = 0x121, .a2 = 0x777 },
    }) |k| {
        var q = emu.State{};
        defer q.deinit(gpa);
        q.a[2] = 0x100;
        q.r[2] = 0x20;
        q.s[3] = 0x777;
        _ = try emu.runFrom(gpa, &q, &.{ 0x10000019, k.w1, k.w2, 0x40210000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.a8, q.a[8]);
        try testing.expectEqual(k.a2, q.a[2]);
    }
    // e<w2[3:0]> = e3: w2[13] = 1 gives e<w2[7:4]> - 1 = e2 - 1 on fn 10; w2[13] = 0 table B's
    // y + i = R2 + 4.  (For R the two tables agree on every fn w1's class admits: 0, 1, 10, 11.)
    for ([_][2]u32{ .{ 0x08003023, 0x6FF }, .{ 0x08001023, 0x504 } }) |k| {
        var q = emu.State{};
        defer q.deinit(gpa);
        q.r[2] = 0x500;
        q.e[2] = 0x700;
        _ = try emu.runFrom(gpa, &q, &.{ 0x10002019, 0xa4020800, k[0], 0x44528000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k[1], q.e[3]);
    }
    // w1[26] = 0 wedges; one store gate alone is unmeasured.
    for ([_]u32{ 0x02000048, 0x06000148 }) |w1| {
        var o = emu.State{};
        defer o.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x00000019, w1, 0x00000000, 0x40248008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
}

test "the 0x16 class-100 s ALU copies s<S2[7:4]> outside its gate (fact 580, 581, 584)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x46000096, s1: u32 = 0x0b000012, s2: u32 = 0x8a0000b4, want: ?u32 }{
        .{ .want = 0xBB0B }, // the measured row: s5 <- s11
        .{ .s2 = 0x8a000034, .want = 0x333 }, // S2[7:4] = 3, code 4
        .{ .s2 = 0x8a0000b0, .want = 0xBB0B }, // code 0
        .{ .h = 0x46000096 | 2 << 27, .want = 0xBB0B }, // 584: mx 0-2 all move b
        .{ .s1 = 0x0b000012 | 1 << 29, .want = 0xBB0B }, // 588: n = 1 keeps the move
        .{ .s1 = 0x0b000012 | 2 << 29, .want = 0x5555 }, // 588: n = 2 runs mx 2's c 4 (a = s5)
        .{ .s1 = 0x0b000012 | 2 << 29, .s2 = 0x8a0000b1, .want = null }, // odd cell
        .{ .s2 = 0x8a0000b1, .want = 0xBB0B }, // 584: every cell
        .{ .h = 0x46000096 | 3 << 27, .want = 0 }, // 584: mx 3 writes 0
        .{ .s2 = 0x8a0000bc, .want = 0xBB0B }, // 588: cell 12 moves b too
        .{ .s2 = 0x8a0001b4, .want = null }, // S2[8] set
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[11] = 0xBB0B;
        state.s[3] = 0x333;
        state.s[5] = 0x5555;
        const r = emu.runFrom(gpa, &state, &.{ c.h, c.s1, c.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x16 class-100 s ALU shifts by s<S2[7:4]> (fact 582, 583, 586)" {
    const gpa = testing.allocator;
    // head[30] = 0, head[29:27] = 1, S2[2:0] = 4: s5 <- s3 >> s11, rounded (583); operands as
    // ldi leaves them, 24-bit (586).
    for ([_]struct { a: u32, k: u32, want: ?u32, code: u32 = 4 }{
        .{ .a = 0x1234, .k = 1, .want = 0x091A },
        .{ .a = 0x1234, .k = 2, .want = 0x048D },
        .{ .a = 0x0100, .k = 1, .want = 0x0080 },
        .{ .a = 0xFF8000, .k = 4, .want = 0xFFF800 },
        .{ .a = 0xFF8000, .k = 1, .want = 0xFFC000 },
        .{ .a = 0xFF8001, .k = 1, .want = 0xFFC001 }, // half up
        .{ .a = 0x1234, .k = 15, .want = 0 },
        .{ .a = 0x1234, .k = 0, .want = 0x1234 },
        .{ .a = 0x1234, .k = 0xFFFFFF, .want = 0x2468 }, // -1
        .{ .a = 0x1234, .k = 0xFFFFFE, .want = 0x48D0 },
        .{ .a = 0x1234, .k = 0xFFFFFC, .want = 0x12340 },
        .{ .a = 0xFF8000, .k = 0xFFFFFC, .want = 0xF80000 },
        .{ .a = 0x1234, .k = 0xFFFFF0, .want = 0x7FFFFF }, // saturates
        .{ .a = 0x1234, .k = 0xFF8000, .want = 0x7FFFFF },
        .{ .a = 0x0F0F, .k = 3, .want = 0x1E2 }, // rounds
        .{ .a = 0x10000, .k = 1, .want = 0x8000 }, // 24-bit operand
        .{ .a = 0xFF8000, .k = 0xFFFFF0, .want = null }, // negative overflow: OPEN
        .{ .a = 0xFF8000, .k = 15, .want = null }, // large amount on a negative value: OPEN
        .{ .a = 0x1234, .k = 0xFFFFFF, .code = 5, .want = null }, // a << -1: OPEN
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = c.a;
        state.s[11] = c.k;
        state.s[5] = 0x5555;
        const r = emu.runFrom(gpa, &state, &.{ 0x0E000096, 0x0b000012, 0x860000b0 | c.code, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x16 class-100 s ALU table outside its gate (fact 583)" {
    const gpa = testing.allocator;
    // s5 <- cell(head[29:27], S2[2:0]) of (a = s3, b = s11), head[30] = 0.
    for ([_]struct { mx: u32, c: u32, r1: u32, r2: u32 }{
        .{ .mx = 0, .c = 0, .r1 = 0x1236, .r2 = 0x0F12 },
        .{ .mx = 0, .c = 1, .r1 = 0x0012, .r2 = 0x000F },
        .{ .mx = 4, .c = 2, .r1 = 0x1237, .r2 = 0x0F13 },
        .{ .mx = 5, .c = 2, .r1 = 0xEDC9, .r2 = 0xF0F3 },
        .{ .mx = 6, .c = 3, .r1 = 0xB72C, .r2 = 0x8780 },
        .{ .mx = 1, .c = 7, .r1 = 0x48D4, .r2 = 0x7880 },
    }) |k| {
        for ([_][3]u32{ .{ 0x1234, 2, k.r1 }, .{ 0x0F0F, 3, k.r2 } }) |v| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[3] = v[0];
            state.s[11] = v[1];
            _ = try emu.runFrom(gpa, &state, &.{ 0x06000096 | k.mx << 27, 0x0b000012, 0x860000b0 | k.c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            try testing.expectEqual(v[2], state.s[5] & 0xFFFF);
        }
    }
    // Every cell, (a, b) = (0x1234, 2), low 16 bits.
    const want = [8][8]u32{
        .{ 0x1236, 0x12, 0x1232, 0x12, 0x1234, 0x12, 0x1235, 0x12 },
        .{ 0x48D, 0x48D0, 0x48D, 0x48D0, 0x48D, 0x48D0, 0x48D, 0x48D4 },
        .{ 0x1236, 0x1236, 0x1232, 0x1232, 0x1234, 0x1234, 0x1235, 0x1235 },
        .{ 0x1236, 0x1236, 0x1232, 0x1232, 0x1234, 0x1234, 0x1235, 0x1235 },
        .{ 0x1232, 0x1232, 0x1237, 0x1237, 0x1234, 0x1234, 0x1233, 0x1233 },
        .{ 0x1236, 0x3600, 0xEDC9, 0xC900, 0x1236, 0x3600, 0x0000, 0x0000 },
        .{ 0x48D, 0x48D0, 0xFB72, 0xB72C, 0x48D, 0x48D0, 0, 0 },
        .{ 0x1236, 0x1236, 0xEDC9, 0xEDC9, 0x1236, 0x1236, 0, 0 },
    };
    for (0..8) |mx| for (0..8) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x1234;
        state.s[11] = 2;
        _ = try emu.runFrom(gpa, &state, &.{ 0x06000096 | @as(u32, @intCast(mx)) << 27, 0x0b000012, 0x860000b0 | @as(u32, @intCast(c)), 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(want[mx][c], state.s[5] & 0xFFFF);
    };
}

test "the 0x16 class-100 s ALU table with head[30] = 1 (fact 584)" {
    const gpa = testing.allocator;
    // (a, b) = (s3, s11): vectors at (0x1234, 2) and (0x0F0F, 3), then every cell at (0x1234, 2).
    const run = struct {
        fn f(mx: u32, c: u32, a: u32, b: u32) !u32 {
            var state = emu.State{};
            defer state.deinit(testing.allocator);
            state.s[3] = a;
            state.s[11] = b;
            _ = try emu.runFrom(testing.allocator, &state, &.{ 0x46000096 | mx << 27, 0x0b000012, 0x860000b0 | c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            return state.s[5] & 0xFFFF;
        }
    }.f;
    _ = gpa;
    try testing.expectError(error.UnknownForm, run(5, 4, 0x1234, 0xFFFFFF)); // negative amount: OPEN here
    for ([_][4]u32{ .{ 7, 3, 0x048C, 0x01E1 }, .{ 7, 1, 0x048D, 0x01E2 }, .{ 5, 4, 0x048D, 0x01E2 }, .{ 4, 5, 0x0012, 0x000F } }) |v| {
        try testing.expectEqual(v[2], try run(v[0], v[1], 0x1234, 2));
        try testing.expectEqual(v[3], try run(v[0], v[1], 0x0F0F, 3));
    }
    const want = [8][8]u32{
        .{ 2, 2, 2, 2, 2, 2, 2, 2 },
        .{ 2, 2, 2, 2, 2, 2, 2, 2 },
        .{ 2, 2, 2, 2, 2, 2, 2, 2 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x12, 0x1234, 0x1234 },
        .{ 0x1234, 0x1234, 0x48D0, 0x48D0, 0x48D, 0x48D, 0x1234, 0x1234 },
        .{ 0, 0, 0, 0, 0, 0, 0, 0 },
        .{ 0x1236, 0x48D, 0x1232, 0x48C, 0x1234, 0x48D, 0x1235, 0x48D },
    };
    for (0..8) |mx| for (0..8) |c| {
        try testing.expectEqual(want[mx][c], try run(@intCast(mx), @intCast(c), 0x1234, 2));
    };
}

test "the 0x16 class-100 s ALU's n = S1[31:29] cells outside its gate (fact 585)" {
    const gpa = testing.allocator;
    for ([_]struct { mx: u32, c: u32, n: u32, want: ?u32, h30: u32 = 0 }{
        .{ .mx = 0, .c = 0, .n = 2, .want = 0x3C4 }, // (a + b) >> n
        .{ .mx = 5, .c = 0, .n = 5, .want = 0xE180 }, // (a ^ b) << n, low 16
        .{ .mx = 1, .c = 1, .n = 1, .want = 0x7880 }, // (a + n) << b
        .{ .mx = 2, .c = 2, .n = 7, .want = 0x0F0C }, // a - b, n ignored
        .{ .mx = 0, .c = 1, .n = 1, .want = null },
        .{ .mx = 7, .c = 7, .n = 3, .want = null },
        .{ .mx = 0, .c = 0, .n = 2, .h30 = 1, .want = null },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x0F0F;
        state.s[11] = 3;
        const r = emu.runFrom(gpa, &state, &.{ 0x06000096 | k.h30 << 30 | k.mx << 27, 0x0b000012 | k.n << 29, 0x860000b0 | k.c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5] & 0xFFFF);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "the 0x16 class-100 s ALU on 24-bit operands (fact 586)" {
    const gpa = testing.allocator;
    for ([_]struct { h30: u32 = 0, mx: u32, c: u32, a: u32, b: u32, want: ?u32 }{
        .{ .mx = 5, .c = 2, .a = 0x1234, .b = 2, .want = 0xFFEDC9 }, // ~(a ^ b)
        .{ .mx = 7, .c = 6, .a = 0xFF8F0F, .b = 0xFFFFF3, .want = 0xFF8F03 }, // a & b
        .{ .mx = 6, .c = 1, .a = 0x1234, .b = 4, .want = 0x012340 }, // a << b
        .{ .mx = 0, .c = 0, .a = 0x7FFF, .b = 0x3B0C, .want = 0x00BB0B }, // no 16-bit wrap
        .{ .mx = 6, .c = 0, .a = 0xFF8000, .b = 4, .want = 0x0FF800 }, // logical >>
        .{ .mx = 6, .c = 0, .a = 0xFF8001, .b = 1, .want = 0x7FC000 },
        .{ .mx = 6, .c = 2, .a = 0x1234, .b = 2, .want = 0x3FFB72 }, // ~a >> b, logical
        .{ .h30 = 1, .mx = 7, .c = 5, .a = 0xFF8000, .b = 4, .want = 0x0FF800 },
        .{ .h30 = 1, .mx = 5, .c = 4, .a = 0xFF8000, .b = 4, .want = 0xFFF800 }, // rounded: arithmetic
        .{ .h30 = 1, .mx = 0, .c = 0, .a = 0, .b = 0x00BB0B, .want = 0x00BB0B }, // raw b
        .{ .h30 = 1, .mx = 1, .c = 3, .a = 0, .b = 0xFF8000, .want = 0xFF8000 }, // all 24 bits
        // 587: >> 8 cells are arithmetic on the even cell's value; every truncating >> b logical
        .{ .mx = 0, .c = 1, .a = 0xFF8000, .b = 4, .want = 0xFFFF80 },
        .{ .mx = 0, .c = 3, .a = 0xFF8000, .b = 4, .want = 0xFFFF7F },
        .{ .mx = 0, .c = 1, .a = 0x12FF, .b = 2, .want = 0x13 },
        .{ .mx = 0, .c = 3, .a = 0x12FF, .b = 2, .want = 0x12 },
        .{ .mx = 0, .c = 5, .a = 0x12FF, .b = 2, .want = 0x12 },
        .{ .mx = 0, .c = 7, .a = 0x12FF, .b = 2, .want = 0x13 },
        .{ .mx = 0, .c = 5, .a = 0x1300, .b = 2, .want = 0x13 },
        .{ .h30 = 1, .mx = 4, .c = 5, .a = 0xFF8000, .b = 4, .want = 0xFFFF80 },
        .{ .h30 = 1, .mx = 7, .c = 1, .a = 0xFF8000, .b = 4, .want = 0x0FF800 },
        .{ .h30 = 1, .mx = 7, .c = 3, .a = 0xFF8000, .b = 4, .want = 0x0FF7FF },
        .{ .h30 = 1, .mx = 7, .c = 7, .a = 0xFF8000, .b = 4, .want = 0x0FF800 },
        .{ .mx = 6, .c = 4, .a = 0xFF8000, .b = 4, .want = 0x0FF800 },
        .{ .mx = 6, .c = 0, .a = 0xFF8000, .b = 0x20, .want = null }, // amount >= 0x20
        .{ .mx = 6, .c = 0, .a = 0xFF8000, .b = 15, .want = null }, // 15+ on a negative
        .{ .mx = 0, .c = 0, .a = 0x7FFFFF, .b = 1, .want = null }, // past 24 bits: OPEN
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = k.a;
        state.s[11] = k.b;
        const r = emu.runFrom(gpa, &state, &.{ 0x06000096 | k.h30 << 30 | k.mx << 27, 0x0b000012, 0x860000b0 | k.c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // 585's n post-shift on mx 0 c 0 is arithmetic.
    var st = emu.State{};
    defer st.deinit(gpa);
    st.s[3] = 0xFF8000;
    _ = try emu.runFrom(gpa, &st, &.{ 0x06000096, 0x8b000012, 0x860000b0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xFFF800), st.s[5]);
}

test "the 0x16 class-100 s ALU's cells 8-15 and S2[8] (fact 588)" {
    const gpa = testing.allocator;
    for ([_]struct { h30: u32 = 0, mx: u32, c: u32, a: u32 = 0x0F0F, b: u32 = 3, n: u32 = 0, b8: u32 = 0, want: ?u32 }{
        .{ .mx = 0, .c = 10, .want = 0xFFF0F4 }, // b - a
        .{ .mx = 0, .c = 10, .a = 0x1234, .b = 2, .want = 0xFFEDCE },
        .{ .mx = 2, .c = 10, .want = 0xFFF0F4 },
        .{ .mx = 0, .c = 12, .want = 0xFFF0F1 }, // -a
        .{ .mx = 3, .c = 12, .a = 0x1234, .b = 2, .want = 0xFFEDCC },
        .{ .mx = 1, .c = 12, .want = 0xFFFE1E }, // (-a) >> b rounded
        .{ .mx = 1, .c = 12, .a = 0x1234, .b = 2, .want = 0xFFFB73 },
        .{ .mx = 1, .c = 12, .a = 0x0F0C, .want = 0xFFFE1F }, // rounded, not 0xFFFE1E
        .{ .mx = 1, .c = 12, .a = 0x1235, .b = 2, .want = 0xFFFB73 }, // not 0xFFFB72
        .{ .mx = 5, .c = 8, .want = 0xFFFFFC }, // ~b
        .{ .mx = 5, .c = 8, .a = 0x1234, .b = 2, .want = 0xFFFFFD },
        .{ .mx = 0, .c = 8, .want = 0x0F0E }, // a - 1
        .{ .mx = 2, .c = 8, .want = 0x0F0E },
        .{ .mx = 0, .c = 14, .want = 0x0F0F }, // a
        .{ .mx = 7, .c = 12, .want = 3 }, // b
        .{ .mx = 7, .c = 14, .want = 3 },
        .{ .h30 = 1, .mx = 0, .c = 12, .want = 3 },
        .{ .mx = 4, .c = 8, .want = null }, // unmeasured cell 8-15
        .{ .mx = 0, .c = 9, .want = null },
        .{ .mx = 0, .c = 10, .n = 1, .want = null }, // S2[3] with n
        // S2[8] inert alone at its measured cells only (b = s0 there)
        .{ .mx = 1, .c = 4, .b8 = 1, .want = 0x0F0F }, // a >> 0 rounded
        .{ .h30 = 1, .mx = 0, .c = 4, .b8 = 1, .want = 0 }, // the move of s0
        .{ .mx = 2, .c = 0, .b8 = 1, .want = null },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = k.a;
        state.s[11] = k.b;
        const src: u32 = if (k.b8 == 1) 0 else 0xB0;
        const r = emu.runFrom(gpa, &state, &.{ 0x06000096 | k.h30 << 30 | k.mx << 27, 0x0b000012 | k.n << 29, 0x86000000 | k.b8 << 8 | src | k.c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // S2[8] with other extra bits: refused.
    var st = emu.State{};
    defer st.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st, &.{ 0x0E000096, 0x0b000012, 0x860001b4, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "the 0x16 class-100 gate cell's top byte (after fact 589)" {
    const gpa = testing.allocator;
    for ([_]struct { mx: u32, c: u32, a: u32, n: u32, want: u32 }{
        .{ .mx = 0, .c = 1, .a = 0x123, .n = 1, .want = 0x024600 },
        .{ .mx = 0, .c = 1, .a = 0x3303, .n = 7, .want = 0x818000 }, // wraps at 24
        .{ .mx = 0, .c = 5, .a = 0x123, .n = 0, .want = 0x012300 },
        .{ .mx = 0, .c = 0, .a = 0x3303, .n = 7, .want = 0x198180 },
        .{ .mx = 1, .c = 0, .a = 0xFF8000, .n = 4, .want = 0xFFF800 },
        .{ .mx = 1, .c = 0, .a = 0xFF8001, .n = 1, .want = 0xFFC001 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = k.a;
        // s3 <- cell(a = s5, n = S1[31:29]); head base 0x0002a816.
        _ = try emu.runFrom(gpa, &state, &.{ 0x0002a816 | k.mx << 27, 0x06000000 | k.n << 29, 0x8a000000 | k.c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.s[3]);
    }
}

test "the 0x16 class-100 row is head[29:25] (fact 590)" {
    const gpa = testing.allocator;
    // 581's move holds at head[26:25] = 2 too; other ops there are OPEN.
    for ([_]struct { h: u32, s1: u32 = 0x0b000012, s2: u32 = 0x8a0000b4, want: ?u32 }{
        .{ .h = 0x44000096, .want = 0xBB0B }, // head[30] = 1, mx 0, head[26:25] = 2, c 4
        .{ .h = 0x44000096, .s2 = 0x8a0000b0, .want = 0xBB0B }, // c 0
        .{ .h = 0x44000096, .s2 = 0x8a0000b2, .want = null }, // c 2
        .{ .h = 0x44000096, .s1 = 0x2b000012, .want = null }, // n = 1
        .{ .h = 0x44000096, .s2 = 0x8a0000bc, .want = 0 }, // S2[3]: cell 12 writes 0 (591)
        .{ .h = 0x4C000096, .want = null }, // mx 1
        .{ .h = 0x04000096, .want = null }, // head[30] = 0
        .{ .h = 0x42000096, .want = null }, // head[26:25] = 1
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[11] = 0xBB0B;
        state.s[5] = 0x5555;
        const r = emu.runFrom(gpa, &state, &.{ k.h, k.s1, k.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // 590's evidence rows at cell 0, a = 0x3303: the gate grid at head[26:25] = 0 (n = 3) and
    // X16_EXTRA at 3 (b = 2, n = 0).
    const grid0 = [8]u32{ 0x19818, 0x660, 0x3306, 0x3303, 0xFFCCFD, 0xFFCCFB, 0x660, 0x3300 };
    const extra3 = [8]u32{ 0x3305, 0xCC1, 0x3305, 0x3305, 0x3301, 0x3301, 0xCC0, 0x3301 };
    for (0..8) |mx| {
        const m: u32 = @intCast(mx);
        var g = emu.State{};
        defer g.deinit(gpa);
        g.s[5] = 0x3303;
        const rg = emu.runFrom(gpa, &g, &.{ 0x0002a816 | m << 27, 0x66000000, 0x8a000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (rg) |_| try testing.expectEqual(grid0[mx], g.s[3]) else |e| try testing.expectEqual(error.UnknownForm, e);
        var x = emu.State{};
        defer x.deinit(gpa);
        x.s[3] = 0x3303;
        x.s[11] = 2;
        _ = try emu.runFrom(gpa, &x, &.{ 0x06000096 | m << 27, 0x0b000012, 0x860000b0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(extra3[mx], x.s[5]);
    }
}

test "the class-100 R feed takes the extra-S2 cell too (fact 591)" {
    const gpa = testing.allocator;
    // R11 <- cell with no s destination, S1[24] = 0; a = s3, b = s<S2[7:4]> = s11.
    for ([_]struct { h: u32, s2: u32, want: ?[2]u32 }{
        .{ .h = 0x46000096, .s2 = 0x860000b4, .want = .{ 2, 3 } }, // the move b
        .{ .h = 0xC6000096, .s2 = 0x860000b4, .want = .{ 2, 3 } }, // head[31:30] = 11
        .{ .h = 0x06000096, .s2 = 0x860000b0, .want = .{ 0x1236, 0x0F12 } }, // a + b
        .{ .h = 0x40000096, .s2 = 0x860000b4, .want = null }, // head[26:25] = 0: OPEN row
        .{ .h = 0x40000096, .s2 = 0x86000004, .want = null }, // gate cell, head[30] = 1
    }) |k| {
        for ([_][2]u32{ .{ 0x1234, 2 }, .{ 0x0F0F, 3 } }, 0..) |v, i| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[3] = v[0];
            state.s[11] = v[1];
            const r = emu.runFrom(gpa, &state, &.{ k.h, 0x00005800, k.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            if (k.want) |w| {
                _ = try r;
                try testing.expectEqual(w[i], state.r[11]);
            } else try testing.expectError(error.UnknownForm, r);
        }
    }
}

test "the class-100 cells measured at head[26:25] = 2 and gate mx 4 c 4 (fact 591)" {
    const gpa = testing.allocator;
    // d = s5 here; a = s<S2[28:25]> = s3, b = s11.  Vectors (0x1234, 2) and (0xFF8F0F, 3).
    for ([_]struct { h: u32, s1: u32 = 0x0b000012, s2: u32, want: ?[2]u32 }{
        .{ .h = 0x1C000096, .s2 = 0x86000004, .want = .{ 0x1234, 0xFF8F0F } }, // gate, h2 mx 3 c 4
        .{ .h = 0x20000096, .s1 = 0xEb000012, .s2 = 0x86000004, .want = .{ 0x1234, 0xFF8F0F } }, // gate, h0 mx 4 c 4 n 7
        .{ .h = 0x0C000096, .s2 = 0x860000b5, .want = .{ 0x48D0, 0xFC7878 } }, // extra, h2 mx 1 c 5
        .{ .h = 0x1C000096, .s2 = 0x860001b4, .want = .{ 0x1234, 0xFF8F0F } }, // extra, h2 mx 3 c 4, S2[8]
        .{ .h = 0x1C000096, .s2 = 0x860000b4, .want = .{ 0x1234, 0xFF8F0F } }, // without S2[8]
        .{ .h = 0x44000096, .s2 = 0x860000bc, .want = .{ 0, 0 } }, // extra, h30 = 1, h2 mx 0 c 12
        .{ .h = 0x0C000096, .s2 = 0x860001b5, .want = null }, // S2[8] elsewhere at h2
        .{ .h = 0x0C000096, .s2 = 0x860000b4, .want = null }, // unmeasured h2 cell
        .{ .h = 0x1C000096, .s1 = 0x2b000012, .s2 = 0x86000004, .want = null }, // gate h2 mx 3 c 4, n 1
        .{ .h = 0x0C000096, .s1 = 0x2b000012, .s2 = 0x860000b5, .want = null }, // extra h2 with n
    }) |k| {
        for ([_][2]u32{ .{ 0x1234, 2 }, .{ 0xFF8F0F, 3 } }, 0..) |v, i| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[3] = v[0];
            state.s[11] = v[1];
            const r = emu.runFrom(gpa, &state, &.{ k.h, k.s1, k.s2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            if (k.want) |w| {
                _ = try r;
                try testing.expectEqual(w[i], state.s[5]);
            } else try testing.expectError(error.UnknownForm, r);
        }
    }
}

test "x16agu carries the 0x05 ALU word in S1 (fact 592)" {
    const gpa = testing.allocator;
    // s6 <- alu(a = s4 = 0x443, b = s11 = 0xBB3); S1 base 0x16045030 (m 2), S2 class 011, b = S2[28:25].
    for ([_]struct { code: u32, s1: u32 = 0x16045030, want: ?u32 }{
        .{ .code = 0x00, .want = 0xBB3 }, // add, m 2
        .{ .code = 0x10, .want = 0xFFF44D }, // sub
        .{ .code = 51, .want = 0xBB3 }, // movb
        .{ .code = 0x30, .want = null }, // another movb code: unmeasured here
        .{ .code = 0x23, .want = 0x443 }, // mova
        .{ .code = 42, .want = 0x443 }, // sel
        .{ .code = 1, .want = 0xBB3 }, // add code 1
        .{ .code = 2, .want = null }, // add 2: unmeasured here
        .{ .code = 0x06, .want = 0 }, // asr
        .{ .code = 0x20, .want = 0 }, // clr
        .{ .code = 0x22, .want = 0 },
        .{ .code = 0x00, .s1 = 0x16040030, .want = 0xFF6 }, // m 0: a + b
        .{ .code = 0x00, .s1 = 0x16041830, .want = 0xBB4 }, // m 3
        .{ .code = 0x00, .s1 = 0x16043030, .want = 0xFFF44D }, // m 6
        .{ .code = 0x00, .s1 = 0x16145030, .want = 0x1766 }, // #s = 1
        .{ .code = 0x00, .s1 = 0x16845030, .want = 0x0BB300 }, // #s = 8
        .{ .code = 0x1c, .want = null }, // xori: OPEN
        .{ .code = 0x10, .s1 = 0x16040030, .want = null }, // sub at m 0: unmeasured
        .{ .code = 0x28, .want = null }, // other clr code
        .{ .code = 0x00, .s1 = 0x16042030, .want = null }, // add at m 4: unmeasured
        .{ .code = 16, .s1 = 0x16043830, .want = null }, // sub at m 7: OPEN
        .{ .code = 0x00, .s1 = 0x16043830, .want = 0xBB3 }, // add at m 7: |b| (after 594)
        .{ .code = 0x00, .s1 = 0x16140030, .want = null }, // add #s at m 0: unmeasured
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[4] = 0x443;
        state.s[11] = 0xBB3;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000016 | k.code << 5, k.s1, 0x76000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[6]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x16agu's ALU and AGU writing one s is refused (fact 592)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    // AGU dest S2[6:3] = 6 and ALU dest S1[27:24] = 6.
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ 0x00000016, 0x16045030, 0x76000030, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "0x02's R slot takes the third slot's value with w5[31:30] = 10 (fact 594)" {
    const gpa = testing.allocator;
    // R2 <- alu(code h[10:5], m w5[8:6], #n w5[5:2]; a = s<w4[25:22]> = s3, b = s<w4[29:26]> = s5).
    for ([_]struct { code: u32 = 0, m: u32, sh: u32 = 0, b: u32 = 0x553, want: ?u32 }{
        .{ .m = 0, .want = 0x886 },
        .{ .m = 1, .want = 0x220 },
        .{ .m = 2, .want = 0x553 },
        .{ .m = 3, .want = 0x554 },
        .{ .m = 5, .want = 0xFDE0 },
        .{ .m = 7, .want = 0x553 },
        .{ .m = 0, .sh = 2, .want = 0x2218 },
        .{ .code = 3, .m = 0, .want = 0x886 },
        .{ .code = 0xE, .m = 0, .want = 0x886 },
        .{ .code = 0xE, .m = 4, .want = 0x552 }, // b - 1
        .{ .code = 0xE, .m = 6, .want = 0xFAAD }, // -b
        .{ .code = 0xE, .m = 7, .b = 0xFFF553, .want = 0x0AAD }, // |b|
        .{ .code = 3, .m = 0, .sh = 2, .want = null }, // shift with code 3: unmeasured
        .{ .code = 0x10, .m = 0, .want = null }, // other code
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x333;
        state.s[5] = k.b;
        const r = emu.runFrom(gpa, &state, &.{ 0x0084F802 | k.code << 5, 0x00000008, 0x00000000, 0x00100000, 0x14D62848, 0x80024200 | k.m << 6 | k.sh << 2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "aluWord: add at m 7 is |b|; sub at m 7 is refused (fact 238, after 594)" {
    try testing.expectEqual(@as(?u32, 0xBB3), try emu.aluWord((7 << 23), 0, 0xBB3));
    try testing.expectEqual(@as(?u32, 0xBB3), try emu.aluWord((7 << 23), 0, -0xBB3));
    try testing.expectError(error.UnknownForm, emu.aluWord((0x10 << 5) | (7 << 23), 0, 0xBB3));
}

test "0x00's s-slot sub-mux 11 reads D (fact 595)" {
    const gpa = testing.allocator;
    // own-x00tab's bundle with w5[24:23] = 11: s1 <- Y:[e<w4[8:5]> = e1] with w5[4], else yd.
    for ([_]struct { w4: u32 = 0x20, w5: u32, want: u32 }{
        .{ .w5 = 0xC3800910, .want = 0x1234 },
        .{ .w4 = 0x120, .w5 = 0xC3800910, .want = 0x7777 }, // w4[8:5] = 9
        .{ .w5 = 0xC3800900, .want = 0xA050 }, // yd (s1 gets it sign-extended, 726)
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x720;
        state.e[9] = 0x760;
        state.yd = 0xA050;
        try state.y_mem.put(gpa, 0x720, 0x1234);
        try state.y_mem.put(gpa, 0x760, 0x7777);
        _ = try emu.runFrom(gpa, &state, &.{ 0x28001000, 0x0002A005 | (3 << 7), 0xE8010000, 0x00000020, k.w4, k.w5, 0x35001002, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(if (k.want & 0x8000 != 0) k.want | 0xFF0000 else k.want, state.s[1]);
        // The fresh read sets yd; the yd copy leaves it.
        try testing.expectEqual(k.want, state.yd);
    }
}

fn runX00Mux(state: *emu.State, mux: u32, w5: u32) !void {
    // 595's bundle (s1 <- the s slot, D = Y:[e1] with w5[4]) with w6[3:1] = mux.
    const w6: u32 = (0x35001002 & ~@as(u32, 0xE)) | mux << 1;
    _ = try emu.runFrom(testing.allocator, state, &.{ 0x28001000, 0x0002A005 | (3 << 7), 0xE8010000, 0x00000020, 0x20, w5, w6, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
}

test "0x00's s-slot muxes read D: 1 sign-, 2/4/5 zero-extended, 3 D << 8 (facts 724, 726, 729)" {
    const gpa = testing.allocator;
    for ([_]struct { mux: u32, y: u32, want: u32 }{
        .{ .mux = 1, .y = 0x1234, .want = 0x001234 },
        .{ .mux = 3, .y = 0x1234, .want = 0x123400 },
        .{ .mux = 4, .y = 0x1234, .want = 0x091A00 },
        .{ .mux = 5, .y = 0x1234, .want = 0x001234 },
        .{ .mux = 3, .y = 0x00C5, .want = 0x00C500 },
        .{ .mux = 4, .y = 0x00C5, .want = 0x006280 },
        .{ .mux = 1, .y = 0x8000, .want = 0xFF8000 },
        .{ .mux = 3, .y = 0x8000, .want = 0x800000 },
        .{ .mux = 4, .y = 0x8000, .want = 0x400000 },
        .{ .mux = 5, .y = 0x8000, .want = 0x008000 },
        .{ .mux = 1, .y = 0xFF00, .want = 0xFFFF00 },
        .{ .mux = 3, .y = 0xFF00, .want = 0xFF0000 },
        .{ .mux = 4, .y = 0xFF00, .want = 0x7F8000 },
        .{ .mux = 5, .y = 0xFF00, .want = 0x00FF00 },
        .{ .mux = 2, .y = 0x8000, .want = 0x008000 }, // 729: mux 2 zero-extends
        .{ .mux = 2, .y = 0xFF00, .want = 0x00FF00 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x720;
        state.yd = 0xA050;
        try state.y_mem.put(gpa, 0x720, k.y);
        try runX00Mux(&state, k.mux, 0xC3800910);
        try testing.expectEqual(k.want, state.s[1]);
        try testing.expectEqual(k.y, state.yd); // the raw word
    }
}

test "0x00's s-slot muxes 3/4/5 ignore w5[24:23] and read yd with w5[4] = 0 (fact 726)" {
    const gpa = testing.allocator;
    for ([_]u32{ 3, 4, 5 }) |mux| {
        for ([_]u32{ 0, 1, 2, 3 }) |sub| {
            for ([_]u32{ 0, 1 }) |fresh| {
                var state = emu.State{};
                defer state.deinit(gpa);
                state.e[1] = 0x720;
                state.yd = 0x0BCD;
                try state.y_mem.put(gpa, 0x720, 0x1234);
                try runX00Mux(&state, mux, (0xC2000900 | sub << 23) | fresh << 4);
                const d: u32 = if (fresh == 1) 0x1234 else 0x0BCD;
                const want: u32 = switch (mux) {
                    3 => d << 8,
                    4 => d << 7,
                    else => d,
                };
                try testing.expectEqual(want, state.s[1]);
                try testing.expectEqual(d, state.yd);
            }
        }
    }
}

fn runX00Slot(state: *emu.State, code: u32, a: u32, m: u32, n: u32) !void {
    // own-x00tab's bundle, ALU slot s2 <- code(a = s<a>, b = s1), m, shift n.
    _ = try emu.runFrom(testing.allocator, state, &.{ 0x28001000 | code << 5, 0x0002A005, 0xE8010004 | n << 6, 0x00000020, 0x20, 0xC0000000 | m << 27, 0x00000020 | a << 20, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
}

test "0x00's class-05 slot takes a = s<w6[23:20]> and m = w5[29:27]; 0x23 adds (fact 725)" {
    const gpa = testing.allocator;
    // own-x00tab's bundle, ALU slot s2 <- code(a, b = s<w2[5:2]> = s1), shift 0.
    for ([_]struct { code: u32 = 0, a: u32, m: u32 = 0, want: ?u32 }{
        .{ .a = 3, .want = 0x0130 },
        .{ .a = 5, .want = 0x0080 },
        .{ .a = 0, .want = 0x0030 },
        .{ .a = 3, .m = 1, .want = 0xFFFF30 }, // b - a
        .{ .a = 3, .m = 2, .want = 0x0030 }, // a dropped
        .{ .code = 0x12, .a = 3, .want = null }, // rsub with a: OPEN
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = 0x30;
        state.s[3] = 0x100;
        state.s[5] = 0x50;
        const r = runX00Slot(&state, k.code, k.a, k.m, 0);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "0x00's class-05 slot: 0x14 adds, 0x06 asr by s<a>, 0x26 selects on the adder sign (facts 727, 728, 731)" {
    const gpa = testing.allocator;
    for ([_]struct { code: u32, a: u32, m: u32 = 0, n: u32 = 0, b: u32 = 0x130, sa: u32 = 2, want: ?u32 }{
        .{ .code = 0x14, .a = 3, .want = 0x0132 },
        .{ .code = 0x14, .a = 3, .m = 1, .want = 0x012E }, // b - a
        .{ .code = 0x14, .a = 5, .want = 0x0030 },
        .{ .code = 0x14, .a = 5, .m = 1, .want = 0x0230 },
        .{ .code = 0x14, .a = 0, .want = 0x0130 },
        .{ .code = 0x06, .a = 3, .want = 0x004C },
        .{ .code = 0x06, .a = 5, .want = 0x0000 }, // count -256
        .{ .code = 0x06, .a = 0, .want = 0x0130 },
        // 731: b = 1 / -1 over counts -8 .. 30
        .{ .code = 0x06, .a = 3, .b = 1, .sa = 0xFFFFF8, .want = 0x000100 },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 0xFFFFF8, .want = 0xFFFF00 },
        .{ .code = 0x06, .a = 3, .b = 1, .sa = 0xFFFFEA, .want = 0x400000 },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 0xFFFFEA, .want = 0xC00000 },
        .{ .code = 0x06, .a = 3, .b = 1, .sa = 0xFFFFE9, .want = 0x800000 },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 0xFFFFE9, .want = 0x800000 },
        .{ .code = 0x06, .a = 3, .b = 1, .sa = 0xFFFFE8, .want = 0 },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 0xFFFFE2, .want = 0 },
        .{ .code = 0x06, .a = 3, .b = 1, .sa = 23, .want = 0 },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 24, .want = 0xFFFFFF },
        .{ .code = 0x06, .a = 3, .b = 0xFFFFFF, .sa = 30, .want = 0xFFFFFF },
        .{ .code = 0x26, .a = 3, .m = 1, .b = 1, .want = 1 }, // min
        .{ .code = 0x26, .a = 3, .b = 0xFFFB00, .sa = 0x444, .want = 0xFFFB00 },
        .{ .code = 0x26, .a = 3, .m = 1, .b = 0x300, .sa = 0x444, .want = 0x300 },
        .{ .code = 0x26, .a = 3, .m = 1, .b = 0x500, .sa = 0x444, .want = 0x444 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = k.b;
        state.s[3] = k.sa;
        state.s[5] = 0xFFFF00;
        const r = runX00Slot(&state, k.code, k.a, k.m, k.n);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "0x00's slot with n set: asr adds i = w2[8:6] and w2[9] negates the count; 0x14 splits on n (fact 732)" {
    const gpa = testing.allocator;
    for ([_]struct { code: u32, m: u32 = 0, n: u32, sa: u32 = 1, b: u32 = 0x100, want: ?u32 }{
        .{ .code = 0x06, .n = 3, .want = 0x81 },
        .{ .code = 0x06, .m = 1, .n = 3, .want = 0x7E },
        .{ .code = 0x06, .m = 2, .n = 3, .want = 0x80 },
        .{ .code = 0x06, .n = 8, .want = 0x200 },
        .{ .code = 0x06, .m = 1, .n = 8, .want = 0x200 },
        .{ .code = 0x06, .n = 11, .want = 0x206 },
        .{ .code = 0x06, .m = 1, .n = 11, .want = 0x1FA },
        .{ .code = 0x06, .n = 9, .want = 0x202 },
        .{ .code = 0x14, .n = 1, .sa = 2, .b = 0x130, .want = 0x132 },
        .{ .code = 0x14, .m = 1, .n = 3, .sa = 2, .b = 0x130, .want = 0x12E },
        .{ .code = 0x14, .n = 2, .sa = 2, .b = 0x130, .want = 0xFFFECC }, // -(a + b) - 2
        .{ .code = 0x14, .n = 4, .sa = 0xFFFB00, .b = 0x130, .want = 0x0003CE },
        .{ .code = 0x14, .m = 1, .n = 8, .sa = 2, .b = 0x130, .want = 0xFFFED2 }, // a - b
        .{ .code = 0x14, .m = 1, .n = 2, .sa = 0xFFFB00, .b = 0x130, .want = 0xFFF9D0 },
        .{ .code = 0x14, .n = 5, .sa = 2, .b = 0x130, .want = 0xFFFECC }, // 749
        .{ .code = 0x14, .n = 6, .sa = 2, .b = 0x130, .want = 0x132 }, // 749: 6 adds
        .{ .code = 0x14, .m = 2, .n = 2, .sa = 2, .b = 0x130, .want = 0xFFFECE }, // 750: -b - 2
        .{ .code = 0x26, .m = 1, .n = 2, .sa = 2, .b = 1, .want = 1 },
        .{ .code = 0x26, .m = 1, .n = 2, .sa = 0x444, .b = 0x500, .want = 0x444 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = k.b;
        state.s[3] = k.sa;
        const r = runX00Slot(&state, k.code, 3, k.m, k.n);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "0x00's code 0x23 never shifts: n picks the adder or a (fact 729)" {
    const gpa = testing.allocator;
    for ([_]struct { n: u32, m: u32 = 0, want: ?u32 }{
        .{ .n = 0, .want = 0x130 },
        .{ .n = 1, .want = 0x130 },
        .{ .n = 3, .m = 1, .want = 0xFFFF30 },
        .{ .n = 9, .m = 2, .want = 0x030 },
        .{ .n = 2, .want = 0x100 },
        .{ .n = 4, .m = 1, .want = 0x100 },
        .{ .n = 8, .want = 0x100 },
        .{ .n = 15, .want = 0x100 },
        .{ .n = 5, .want = 0x100 }, // 749
        .{ .n = 6, .want = 0x130 }, // 749: 6 adds
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = 0x30;
        state.s[3] = 0x100;
        const r = runX00Slot(&state, 0x23, 3, k.m, k.n);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x18 class 111 muxes 4 and 5 fresh reads set yd too (after fact 597)" {
    const gpa = testing.allocator;
    for ([_][2]u32{ .{ 4, 0x5B00 }, .{ 5, 0x12B6 } }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try state.y_mem.put(gpa, 0x700, 0x12B6);
        state.e[3] = 0x700;
        state.yd = 0xA050;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00400018, k[0] << 12, 0x00200000, 0xE6000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k[1], state.s[2]);
        try testing.expectEqual(@as(u32, 0x12B6), state.yd); // the raw word, not the lane
    }
}

test "x18 class 010's R slot refuses w2[27:24] != 0 (fact 598)" {
    const gpa = testing.allocator;
    // R2 <- AGU3 fn 8 (x + y) of A9 = 0x20, R6 = 0x100.
    for ([_]u32{ 0, 1, 2, 6, 0xF }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x20;
        state.r[6] = 0x100;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000018, 0x00000000, k << 24, 0x4084B100, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k == 0) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0x120), state.r[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x18 class 110 stores s<{w2[0], w1[31:29]}> at X:[A<w2[21:17]>] when w2[23:22] = 11 (fact 599)" {
    const gpa = testing.allocator;
    // Head and w3 from own-x18c6y (01800058 A0001000 00D80001 C0000001, X:[A12] <- s13).
    for ([_]struct { h: u32 = 0x01800058, w1: u32, w2: u32, w3: u32 = 0xC0000001, ai: u5, addr: u32, si: u4, sv: u32, want: ?u32 }{
        .{ .w1 = 0xC0001000, .w2 = 0x00CA0000, .ai = 5, .addr = 0x300, .si = 6, .sv = 0x1234, .want = 0x001234 },
        .{ .w1 = 0xC0001000, .w2 = 0x00CA0000, .ai = 5, .addr = 0x320, .si = 6, .sv = 0x0567, .want = 0x000567 },
        .{ .w1 = 0xA0001000, .w2 = 0x00D80001, .ai = 12, .addr = 0x340, .si = 13, .sv = 0xFF8567, .want = 0xFF8567 },
        .{ .w1 = 0x20001000, .w2 = 0x008C0001, .ai = 6, .addr = 0x160, .si = 9, .sv = 0xFF9909, .want = 0 }, // 10: no store
        .{ .w1 = 0x20005000, .w2 = 0x00CC0001, .ai = 6, .addr = 0x160, .si = 9, .sv = 0xFF9909, .want = null }, // w1 mode 5
        .{ .w1 = 0x20001000, .w2 = 0x00CC0001, .w3 = 0xC0000101, .ai = 6, .addr = 0x160, .si = 9, .sv = 0xFF9909, .want = null }, // branch
        .{ .w1 = 0xC0001000, .w2 = 0x00E80000, .ai = 20, .addr = 0x380, .si = 6, .sv = 0x0042, .want = 0x000042 }, // A20: 5-bit index
        // the ALU2 slot writes s9 too: ordering unmeasured
        .{ .h = 0x13200418, .w1 = 0x20001900, .w2 = 0x00CC0001, .ai = 6, .addr = 0x160, .si = 9, .sv = 0xFF9909, .want = null },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[k.ai] = k.addr;
        state.s[k.si] = k.sv;
        const r = emu.runFrom(gpa, &state, &.{ k.h, k.w1, k.w2, k.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.readX(k.addr) orelse 0);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x18 class 110's doubled hi is P[39:24], rounded by w3[22] (fact 600)" {
    const gpa = testing.allocator;
    // x18.mpy's bundle with w3[24] = 0: s5 <- hi(s1 * s2 << 1); w3[22] picks round / floor.
    for ([_]struct { a: u32, b: u32, round: u32, floor: u32 }{
        .{ .a = 0x7000, .b = 0x0123, .round = 1, .floor = 0 },
        .{ .a = 0x1000, .b = 0x0C00, .round = 2, .floor = 1 },
        .{ .a = 0x1000, .b = 0x0BFF, .round = 1, .floor = 1 },
        .{ .a = 0x1234, .b = 0x5678, .round = 0xC, .floor = 0xC },
        .{ .a = 0x7000, .b = 0xFFFEDD, .round = 0xFFFF, .floor = 0xFFFF },
        .{ .a = 0x4000, .b = 0x4000, .round = 0x20, .floor = 0x20 },
    }) |k| {
        for ([_]u32{ 1, 0 }) |rnd| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[1] = k.a;
            state.s[2] = k.b;
            _ = try emu.runFrom(gpa, &state, &.{ 0x00000018, 0x00000000, 0x03000a42, 0xC0100001 | rnd << 22, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            try testing.expectEqual(if (rnd == 1) k.round else k.floor, state.s[5] & 0xFFFF);
        }
    }
}

test "x18 class 110 integer-mode hi: w3[22] is signedness (after fact 600)" {
    const gpa = testing.allocator;
    // x18.mpy's bundle (w3[24] = 1): s5 <- (s1 * s2) >> 24, no rounding.
    for ([_]struct { a: u32, b: u32, signed: u32, want: ?u32 }{
        .{ .a = 0x1000, .b = 0x1800, .signed = 1, .want = 1 },
        .{ .a = 0x1000, .b = 0x1800, .signed = 0, .want = 1 },
        .{ .a = 0x7000, .b = 0xFFFEDD, .signed = 1, .want = 0xFFFF },
        .{ .a = 0x7000, .b = 0xFFFEDD, .signed = 0, .want = 0x6FFF },
        .{ .a = 0xFFFEDD, .b = 0x7000, .signed = 0, .want = null }, // a negative: unmeasured
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = k.a;
        state.s[2] = k.b;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000018, 0x00000000, 0x03000a42, 0xC1100001 | k.signed << 22, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[5] & 0xFFFF);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x18 class 111 MAC high write rounds with w3[8] (fact 601)" {
    const gpa = testing.allocator;
    // 80600018 72343004 32300006 e60481d6: s6 <- ({s7 : sx24 s6} + s2 * s3 << 1)[39:24].
    for ([_]struct { a: u32, b: u32, s7: u32 = 0, round: ?u32, floor: u32 }{
        .{ .a = 0x1000, .b = 0x0C00, .round = 2, .floor = 1 },
        .{ .a = 0x1000, .b = 0x0BFF, .round = 1, .floor = 1 },
        .{ .a = 0x7000, .b = 0x0123, .round = 1, .floor = 0 },
        .{ .a = 0x7000, .b = 0xFFFEDD, .round = 0xFFFF, .floor = 0xFFFF }, // ldi sign-extends (665: 16-bit inputs)
        .{ .a = 0x1000, .b = 0x0C00, .s7 = 3, .round = 5, .floor = 4 },
    }) |k| {
        for ([_]u32{ 1, 0 }) |rnd| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[2] = k.a;
            state.s[3] = k.b;
            state.s[7] = k.s7;
            _ = try emu.runFrom(gpa, &state, &.{ 0x80600018, 0x72343004, 0x32300006, 0xE60480D6 | rnd << 8, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            try testing.expectEqual(if (rnd == 1) k.round.? else k.floor, state.s[6] & 0xFFFF);
        }
    }
    // Every mode, toward +inf (after 601): P = 0x1800000, X = 1 << 24, ties.
    for ([_][3]u32{ .{ 4, 2, 1 }, .{ 6, 1, 0 }, .{ 7, 0, 0xFFFF } }) |k| {
        for ([_]u32{ 1, 0 }) |rnd| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[2] = 0x1000;
            state.s[3] = 0x0C00;
            state.s[7] = 1;
            _ = try emu.runFrom(gpa, &state, &.{ 0x80600018, 0x72343004, 0x32300006, 0xE6048006 | k[0] << 4 | rnd << 8, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            try testing.expectEqual(if (rnd == 1) k[1] else k[2], state.s[6] & 0xFFFF);
        }
    }
}

test "x03 override 11 over mux 001/010 reads Y:[e] with w5[21], else yd (fact 602)" {
    const gpa = testing.allocator;
    for ([_]struct { w5: u32, y: u32, yd: u32, want: u32 }{
        .{ .w5 = 0x30300100, .y = 0x1234, .yd = 0x34C9, .want = 0x1234 },
        .{ .w5 = 0x30300100, .y = 0x0BCD, .yd = 0x34C9, .want = 0x0BCD },
        .{ .w5 = 0x30100100, .y = 0x1234, .yd = 0x34C9, .want = 0x34C9 },
        .{ .w5 = 0x30100100, .y = 0x1234, .yd = 0x7E11, .want = 0x7E11 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[10] = 0x720;
        state.yd = k.yd;
        try state.y_mem.put(gpa, 0x720, k.y);
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000003, 0x00000170, 0x0140a000, 0x57001405, 0x01004835, k.w5, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.s[6]);
        try testing.expectEqual(k.want, state.yd); // the fresh read sets yd (after 602)
    }
}

test "x03 product low half is (X op P) mod 2^24 (fact 603, 663)" {
    const gpa = testing.allocator;
    // 80000003 00006800 01441485 00000007 00002C90 00041402 (w4 mode 5): s9 <- low, s2 = 3, s4 = 5 (P = 0x1E),
    // s<k> = 0x110 k + 1, X = s<w5[12:9]>, mode w1[13:11].
    // w1[17] clears the w2[12:9] high dest (DERIVED: measured with 0xA, whose high write
    // there is unmodelled and refuses).
    for ([_]struct { w1: u32 = 0x00006800, w2: u32 = 0x01441485, k: u32 = 8, want: ?u32 }{
        .{ .want = 0x089F },
        .{ .k = 10, .want = 0x0ABF },
        .{ .k = 12, .want = 0x0CDF },
        .{ .k = 1, .want = 0x012F },
        .{ .w1 = 0x00007800, .want = 0x0863 },
        .{ .w1 = 0x00007000, .want = 0xFFF79D }, // 603 read 0xF79D: the slot keeps 24 bits (663)
        .{ .w1 = 0x00006000, .want = 0x001E },
        .{ .w1 = 0x00026800, .w2 = 0x01440085, .want = 0x0890 }, // w1[17]: P not doubled
        .{ .w1 = 0x00002800, .want = null }, // w1[14] = 0 with an X mode: OPEN
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |i| state.s[i] = 0x110 * @as(u32, @intCast(i)) + 1;
        state.s[2] = 3;
        state.s[4] = 5;
        const r = emu.runFrom(gpa, &state, &.{ 0x80000003, c.w1, c.w2, 0x00000007, 0x00002C90, 0x00040002 | c.k << 9, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[9]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x03 high dest beside w4 mode 5 is the 40-bit acc's [39:24] (fact 604, 605)" {
    const gpa = testing.allocator;
    // 603's row: s10 <- ({X_hi : s<w5[12:9]> = s10} + 2ab)[39:24], X_hi = s<{w2[0], w1[31:29]}>;
    // presets s<k> = 0x110 k + 1.
    for ([_]struct { w1: u32 = 0x00006800, w2: u32 = 0x01441485, a: u32 = 3, b: u32 = 5, want: ?u32 }{
        .{ .want = 0x0881 },
        .{ .a = 0x4000, .b = 0x4000, .want = 0x08A1 },
        .{ .a = 0x1000, .b = 0x0C00, .want = 0x0882 },
        .{ .a = 0x7000, .b = 0xFFFEDD, .want = 0x0880 },
        .{ .w1 = 0x60006800, .want = 0x0BB1 }, // w1[31:29] = 3, w2[0] = 1: s11
        .{ .w1 = 0x60006800, .w2 = 0x01441484, .want = 0x0331 }, // w2[0] = 0: s3
        .{ .w1 = 0xC0006800, .w2 = 0x01441484, .want = 0x0661 }, // s6
        .{ .w2 = 0x01441484, .want = null }, // X index 0: OPEN
        .{ .w1 = 0x00026800, .want = null }, // w1[17] (again, kept for the table shape)
        .{ .w1 = 0x00026800, .want = null }, // w1[17]
        .{ .w1 = 0x00006000, .a = 0x4000, .b = 0x4000, .want = 0x0020 }, // mode 4: P alone
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |i| state.s[i] = 0x110 * @as(u32, @intCast(i)) + 1;
        state.s[2] = c.a;
        state.s[4] = c.b;
        const r = emu.runFrom(gpa, &state, &.{ 0x80000003, c.w1, c.w2, 0x00000007, 0x00002C90, 0x00041402, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[10]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x03 mode-5-path high dest: modes 6/7 and w1[15] rounding (after fact 605)" {
    const gpa = testing.allocator;
    // X_hi = s8 = 0x881, X_lo = s10 = 0xAA1; 2ab = 0x1800000 (0x1000 * 0x0C00) or 0x1E (3 * 5).
    for ([_]struct { mode: u32, big: bool, floor: u32, round: u32 }{
        .{ .mode = 5, .big = true, .floor = 0x0882, .round = 0x0883 },
        .{ .mode = 5, .big = false, .floor = 0x0881, .round = 0x0881 },
        .{ .mode = 6, .big = true, .floor = 0xF780, .round = 0xF780 },
        .{ .mode = 6, .big = false, .floor = 0xF77E, .round = 0xF77F },
        .{ .mode = 7, .big = true, .floor = 0x087F, .round = 0x0880 },
        .{ .mode = 7, .big = false, .floor = 0x0881, .round = 0x0881 },
    }) |c| {
        for ([_]u32{ 0, 1 }) |rnd| {
            var state = emu.State{};
            defer state.deinit(gpa);
            for (1..16) |i| state.s[i] = 0x110 * @as(u32, @intCast(i)) + 1;
            state.s[2] = if (c.big) 0x1000 else 3;
            state.s[4] = if (c.big) 0x0C00 else 5;
            const w1: u32 = 0x00004000 | c.mode << 11 | rnd << 15;
            _ = try emu.runFrom(gpa, &state, &.{ 0x80000003, w1, 0x01441485, 0x00000007, 0x00002C90, 0x00041402, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            // 605's values are a 16-bit view (0xF780 for the 24-bit 0xFFF780 of 737's 48-bit acc).
            try testing.expectEqual(if (rnd == 1) c.round else c.floor, state.s[10] & 0xFFFF);
        }
    }
}

test "x03 head ALU refuses w5[16:14] != 0 (fact 569, 570, 606)" {
    const gpa = testing.allocator;
    // s1 <- add(s11, s12); w5[16:14] forces an add under a head[22:19] gate with no rule.
    for ([_]u32{ 0, 1, 2, 4 }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[11] = 0x0BB1;
        state.s[12] = 0x0CC1;
        const r = emu.runFrom(gpa, &state, &.{ 0x30008003 | 11 << 11, 0, 0, 0, 0, k << 14, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (k == 0) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0x1872), state.s[1]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "ld0e has psel shape A's ALU word and head mux (fact 607)" {
    const gpa = testing.allocator;
    // s9 <- a = s5 = 0x551 op b = s6 = 0x661; s3 <- mux S[14:12] (head dest 3).
    for ([_]struct { code: u32 = 14, m: u32 = 0, mux: u32 = 1, imm: u32 = 0x14A1, sp: u32 = 0, alu: ?u32, hd: u32 = 0x14A1 }{
        .{ .alu = 0xBB2 },
        .{ .m = 2, .alu = 0x0661 },
        .{ .code = 22, .alu = 0x0330 },
        .{ .code = 51, .alu = 0x0661 },
        .{ .code = 6, .alu = 0 },
        .{ .mux = 6, .imm = 0x0BCD, .alu = 0xBB2, .hd = 0x0BCD },
        .{ .mux = 0, .alu = 0xBB2, .hd = 0 }, // p0, modifier 0 (own-0e)
        .{ .code = 16, .alu = null }, // 569's override set: OPEN
        .{ .code = 35, .alu = null },
        .{ .m = 3, .alu = null },
        .{ .mux = 0, .sp = 4 << 21, .alu = null }, // cond modifier 4: OPEN
        .{ .mux = 2, .alu = null },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x551;
        state.s[6] = 0x661;
        const head: u32 = 0x8000000E | c.code << 25 | 3 << 21 | c.imm << 5;
        const sw: u32 = 6 << 28 | c.sp | c.m << 15 | c.mux << 12 | 9 << 8 | 5 << 4;
        const r = emu.runFrom(gpa, &state, &.{ head, sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.alu) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[9]);
            try testing.expectEqual(c.hd, state.s[3]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "ld0e mux 0 writes the cond on p<S[27:24]> by modifier S[23:21] (fact 608)" {
    const gpa = testing.allocator;
    for ([_]struct { p: u32, md: u32, want0: u32, want1: u32 }{
        .{ .p = 12, .md = 0, .want0 = 1, .want1 = 0 },
        .{ .p = 12, .md = 1, .want0 = 0, .want1 = 1 },
        .{ .p = 12, .md = 2, .want0 = 0, .want1 = 0 },
        .{ .p = 12, .md = 3, .want0 = 1, .want1 = 1 },
        .{ .p = 5, .md = 0, .want0 = 1, .want1 = 0 },
        .{ .p = 0, .md = 0, .want0 = 0, .want1 = 0 }, // p0 reads 1
        .{ .p = 0, .md = 1, .want0 = 1, .want1 = 1 },
    }) |c| {
        for ([_]u8{ 0, 1 }) |pv| {
            var state = emu.State{};
            defer state.deinit(gpa);
            if (c.p != 0) state.p[c.p] = pv;
            _ = try emu.runFrom(gpa, &state, &.{ 0x9C60000E, c.p << 24 | c.md << 21, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            try testing.expectEqual(if (pv == 1) c.want1 else c.want0, state.s[3]);
        }
    }
}

test "p0 reads 1 from reset (fact 608)" {
    const state = emu.State{};
    try testing.expectEqual(@as(u8, 1), state.p[0]);
}

test "writes to p0 refuse (after fact 608)" {
    const gpa = testing.allocator;
    // tst (low5 0x12) and x18 class 011 code 11: p3 writes, p0 refuses.
    for ([_]struct { prog: []const u32, ok: bool }{
        .{ .prog = &.{ 0x00000012, 0x26080000, 0x00000017, 0x31308000, 0xA0000000 }, .ok = true },
        .{ .prog = &.{ 0x00000012, 0x20080000, 0x00000017, 0x31308000, 0xA0000000 }, .ok = false },
        .{ .prog = &.{ 0x16000018, 0x00000000, 0x0000C000, 0x60000001, 0x00000017, 0x31308000, 0xA0000000 }, .ok = true },
        .{ .prog = &.{ 0x16000018, 0x00000000, 0x00000000, 0x60000001, 0x00000017, 0x31308000, 0xA0000000 }, .ok = false },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, c.prog, 0, 16);
        if (c.ok) _ = try r else try testing.expectError(error.UnknownForm, r);
    }
}

test "x17agu also copies its AGU value to s<S1[23:20]> (fact 609)" {
    const gpa = testing.allocator;
    // 00000017 50000100 402a8660: R3 <- AGU10[0](x = e8); s<S1[23:20]> too.
    for ([_]struct { sd: u32, e8: u32 = 0x0123, fn_op: u32 = 0, want: ?u32 }{
        .{ .sd = 5, .want = 0x0123 },
        .{ .sd = 5, .e8 = 0x0BCD, .want = 0x0BCD },
        .{ .sd = 9, .want = 0x0123 },
        .{ .sd = 0, .want = 0x0123 }, // no s write
        .{ .sd = 5, .fn_op = 2, .want = null }, // s copy measured for fn 0 only
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[8] = c.e8;
        const r = emu.runFrom(gpa, &state, &.{ 0x00000017, 0x50000100 | c.sd << 20 | c.fn_op << 24, 0x402a8660, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.r[3]);
            if (c.sd != 0) try testing.expectEqual(v, state.s[c.sd]);
            for (1..16) |i| if (i != c.sd) try testing.expectEqual(@as(u32, 0), state.s[i]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x16agu2 mux 0's s write is measured as 0 only at head code 14 (fact 610)" {
    const gpa = testing.allocator;
    // 002001d6 01002d20 a14c8f00: s6 <- 0 at code 14; code 12 gave 1 (OPEN).
    for ([_]struct { head: u32, ok: bool }{ .{ .head = 0x002001D6, .ok = true }, .{ .head = 0x00200196, .ok = false } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0x777;
        const r = emu.runFrom(gpa, &state, &.{ c.head, 0x01002D20, 0xA14C8F00, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.ok) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0), state.s[6]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19a3 s mux 5 reads X:[A<{w1[2:0], head[31:30]}>] before the A write; 4 and 7 (fact 611)" {
    const gpa = testing.allocator;
    // own-x19a3m's bundle; s3 <- mux; A<head[29:25]> <- AGU3 (A14 - 0x100).
    for ([_]struct { head: u32, w1: u32 = 0x0816bb00, ai: u5, addr: u32, x: u32, want: u32 }{
        .{ .head = 0x8a600019, .ai = 2, .addr = 0x300, .x = 0x1234, .want = 0x1234 },
        .{ .head = 0x8a600019, .ai = 2, .addr = 0x300, .x = 0xFF8BCD, .want = 0x8BCD },
        .{ .head = 0x8a600019, .ai = 2, .addr = 0x320, .x = 0x1234, .want = 0x1234 },
        .{ .head = 0xca600019, .ai = 3, .addr = 0x340, .x = 0x5A31, .want = 0x5A31 }, // head[30]
        .{ .head = 0x8a600019, .w1 = 0x0816bb02, .ai = 10, .addr = 0x360, .x = 0x0C3D, .want = 0x0C3D }, // w1[1]
        .{ .head = 0x84600019, .ai = 2, .addr = 0x300, .x = 0x1234, .want = 0x1234 }, // A2 is also the A dest
        .{ .head = 0x8a600019, .w1 = 0x08163b00, .ai = 2, .addr = 0x300, .x = 0x1234, .want = 0x1A00 }, // mux 4
        .{ .head = 0x8a600019, .w1 = 0x0817bb00, .ai = 2, .addr = 0x300, .x = 0x1234, .want = 0 }, // mux 7
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[14] = 0x300;
        state.a[c.ai] = c.addr;
        try state.x_mem.put(gpa, c.addr, c.x);
        _ = try emu.runFrom(gpa, &state, &.{ c.head, c.w1, 0x00002008, 0x7b570080, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.s[3]);
    }
}

test "x19a3st's s write takes 529's mux; the store reads the old s (fact 612)" {
    const gpa = testing.allocator;
    // Y:[e2] <- s<{w2[0], w1[31:29]}> = s15; s15 <- mux w1[17:15] (0 zero, 2 AGU3 fn 25 = 2 * 0xB0).
    for ([_]struct { mux: u32, extra: u32 = 0, want: ?u32 }{
        .{ .mux = 0, .want = 0 },
        .{ .mux = 2, .want = 0x160 },
        .{ .mux = 1, .want = null },
        .{ .mux = 3, .want = null },
        .{ .mux = 0, .extra = 3 << 22, .want = null }, // class 7's A<w1[26:22]> slot: unmodelled

    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x700;
        state.s[15] = 0x5555;
        const r = emu.runFrom(gpa, &state, &.{ 0x01E00019, 0xE0000000 | c.mux << 15 | c.extra, 0x00000203, 0x659000B0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[15]);
            try testing.expectEqual(@as(?u32, 0x5555), state.getY(0x700));
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x19a3st's R slot (w1[31:29] != 7) and class 7's e slot (fact 144, 148, 149, 613)" {
    const gpa = testing.allocator;
    // Store s5 or s7 at Y:[e2 = 0x700].  w1[31:29] = 5: R3 <- AGU10[2](e2) = 0x701.
    // w1[31:29] = 7: e3 <- #head[20:5] = 0x1234.
    {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x700;
        state.s[5] = 0x1357;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000019, 0xA0001900, 0x00000202, 0x659000B0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(?u32, 0x1357), state.getY(0x700));
        try testing.expectEqual(@as(u32, 0x701), state.r[3]);
    }
    {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x700;
        state.s[7] = 0x2468;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00024699, 0xE0000018, 0x00000202, 0x659000B0, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(?u32, 0x2468), state.getY(0x700));
        try testing.expectEqual(@as(u32, 0x1234), state.e[3]);
    }
}

test "x19a3 stores X with w2[13:9] = 01101 at the A read before the A write (fact 613)" {
    const gpa = testing.allocator;
    // 43e00019 08010000 00001b24 68580000: A1 is the store index and the dest (A1 <- A16).
    for ([_]struct { w2: u32 = 0x00001B24, a1: u32, s9: u32, want: ?u32 }{
        .{ .a1 = 0x380, .s9 = 0x4321, .want = 0x004321 },
        .{ .a1 = 0x3A0, .s9 = 0xFF8765, .want = 0xFF8765 },
        .{ .w2 = 0x00000B24, .a1 = 0x380, .s9 = 0x4321, .want = null }, // w2[12] = 0: no store
        .{ .w2 = 0x00003B24, .a1 = 0x380, .s9 = 0x4321, .want = null }, // w2[13:9] = 11101: not claimed
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[1] = c.a1;
        state.a[16] = 0x500;
        state.s[9] = c.s9;
        _ = try emu.runFrom(gpa, &state, &.{ 0x43E00019, 0x08010000, c.w2, 0x68580000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.readX(c.a1));
        try testing.expectEqual(@as(u32, 0x500), state.a[1]);
    }
}

test "x10alu ignores head[31:30] and has no w1[14] gate (fact 614, 615)" {
    const gpa = testing.allocator;
    // own-x10alu0 0A0001D0 80A20017: R5 <- b - 1, b = s<head[29:26]>.
    for ([_]struct { head: u32, w1: u32 = 0x80A20017, s2: u32 = 0x1234, s10: u32 = 0, want: u32 }{
        .{ .head = 0x0A0001D0, .want = 0x1233 },
        .{ .head = 0x4A0001D0, .want = 0x1233 },
        .{ .head = 0x8A0001D0, .want = 0x1233 },
        .{ .head = 0xCA0001D0, .want = 0x1233 },
        .{ .head = 0x8A0001D0, .s2 = 0x0800, .want = 0x07FF },
        // head[29] = 1: b = s10, w1[14] = 0 (615)
        .{ .head = 0x2A0001D0, .w1 = 0x80A20017 & ~@as(u32, 1 << 14), .s10 = 0x0500, .want = 0x04FF },
        .{ .head = 0xAA0001D0, .w1 = 0x80A20017 & ~@as(u32, 1 << 14), .s10 = 0x0500, .want = 0x04FF },
        .{ .head = 0xEA0001D0, .w1 = 0x80A20017 & ~@as(u32, 1 << 14), .s10 = 0x2222, .want = 0x2221 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[2] = c.s2;
        state.s[10] = c.s10;
        _ = try emu.runFrom(gpa, &state, &.{ c.head, c.w1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.r[5]);
    }
}

test "ypair class 110 admits head[31:29] 0-3; head[31] refuses (fact 616)" {
    const gpa = testing.allocator;
    // 3840c154 d2026001: s2 <- e10.
    for ([_]u32{ 0, 1, 2, 3, 4, 7 }) |top| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[10] = 0x0345;
        state.s[2] = 0x7777;
        const r = emu.runFrom(gpa, &state, &.{ (0x3840c154 & 0x1FFFFFFF) | top << 29, 0xd2026001, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (top < 4) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0x0345), state.s[2]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "jr2 runs its head ALU word and jumps to q + 2 * w1[18:5] after two slots (fact 302, 303, 617)" {
    const gpa = testing.allocator;
    // own-x11jr shape: at word 4, jr2 off 4 -> word 12; two slots set s2 = 0x333, 0x444; the
    // ldi s1, #0xBAD between the slots and the target is skipped.
    for ([_]struct { head: u32, s10: u32 = 0x1000, s3: u32 = 0x0234, want: u32 }{
        .{ .head = 0x0c00d1d1, .want = 0x1234 },
        .{ .head = 0x0c00d1d1, .s10 = 0x2000, .s3 = 0x0005, .want = 0x2005 },
        .{ .head = 0xcc00d1d1, .want = 0x1234 }, // head[31:30] inert
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[10] = c.s10;
        state.s[3] = c.s3;
        const prog = [_]u32{
            0xca80000b, 0xca80000b, 0xca80000b, 0xca80000b, // words 0-3: ldi s4, #0
            c.head, 0x80200080, // word 4: jr2, off 4 -> word 12
            0xca40666b, 0xca40888b, // slots: ldi s2, #0x333 / #0x444
            0xcaa175ab, 0xcaa175ab, 0xcaa175ab, 0xcaa175ab, // skipped: ldi s5, #0xBAD
            0x00000017, 0x31308000, 0xA0000000, // word 12: halt
        };
        _ = try emu.runFrom(gpa, &state, &prog, 0, 64);
        try testing.expectEqual(c.want, state.s[1]);
        try testing.expectEqual(@as(u32, 0x444), state.s[2]);
        try testing.expectEqual(@as(u32, 0), state.s[5]);
    }
}

test "jr2's offset field is wider than 3 bits (fact 303; derived)" {
    const gpa = testing.allocator;
    // jr2 at word 0 with off 8 -> word 16; slots at 2/3, words 4-15 skipped.
    var prog: [19]u32 = @splat(0xcaa175ab); // ldi s5, #0xBAD
    prog[0] = 0x0c00d1d1;
    prog[1] = 0x80200100;
    prog[2] = 0xca40666b;
    prog[3] = 0xca40888b;
    prog[16] = 0x00000017;
    prog[17] = 0x31308000;
    prog[18] = 0xA0000000;
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.runFrom(gpa, &state, &prog, 0, 64);
    try testing.expectEqual(@as(u32, 0x444), state.s[2]);
    try testing.expectEqual(@as(u32, 0), state.s[5]);
}

test "class aliases: x17agu2 S1 111, x17sub any S1, agu12 / agu13 S 011 (fact 618-621)" {
    const gpa = testing.allocator;
    const Case = struct { prog: []const u32, cls_word: usize, classes: []const u32 };
    const cases = [_]Case{
        // own-x17rsub: s5 <- 0xFFFCD7 from s3 = 0x333
        .{ .prog = &.{ 0x00000017, 0x04a85a00, 0x26000000, 0x00000017, 0x31308000, 0xA0000000 }, .cls_word = 1, .classes = &.{ 0, 1, 2, 3, 4, 5, 6, 7 } },
        // own-x17agu2: s5 = e2 = 0x702 from R3 = 0x700
        .{ .prog = &.{ 0x18001017, 0x28001d01, 0x00001400, 0x00000017, 0x31308000, 0xA0000000 }, .cls_word = 1, .classes = &.{ 1, 3, 5, 7 } },
        // own-x13agu: e9 = 0x350, R5 = 0x123
        .{ .prog = &.{ 0x00006a13, 0x2a6cf300, 0x00000017, 0x31308000, 0xA0000000 }, .cls_word = 1, .classes = &.{ 1, 3 } },
        // agu12 2bc000b2: s2 <- e5
        .{ .prog = &.{ 0x2bc000b2, 0x72080001, 0x00000017, 0x31308000, 0xA0000000 }, .cls_word = 1, .classes = &.{3} },
    };
    for (cases) |c| {
        var base: ?emu.State = null;
        defer if (base) |*b| b.deinit(gpa);
        for (c.classes) |cls| {
            var state = emu.State{};
            state.s[3] = 0x333;
            state.r[3] = 0x700;
            state.e[3] = 0x100;
            state.r[7] = 0x23;
            state.e[5] = 0x0345;
            var prog: [8]u32 = undefined;
            @memcpy(prog[0..c.prog.len], c.prog);
            prog[c.cls_word] = (prog[c.cls_word] & 0x1FFFFFFF) | cls << 29;
            _ = try emu.runFrom(gpa, &state, prog[0..c.prog.len], 0, 16);
            if (base) |b| {
                try testing.expectEqualSlices(u32, &b.s, &state.s);
                try testing.expectEqualSlices(u32, &b.e, &state.e);
                try testing.expectEqualSlices(u32, &b.r, &state.r);
                state.deinit(gpa);
            } else base = state;
        }
    }
    // agu12 value check
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[5] = 0x0BCD;
    _ = try emu.runFrom(gpa, &st, &.{ 0x2bc000b2, 0x72080001, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x0BCD), st.s[2]);
}

test "b16a's move reads a when S1[4:3] = 01 (fact 622, 623)" {
    const gpa = testing.allocator;
    // c6000016 0500201c 86000044: head[30] = 1, mx 0, cell 4; a = s3, b = s4, dest s2.
    for ([_]struct { s1lo: u32, want: ?u32 }{
        .{ .s1lo = 0x00, .want = 0x777 },
        .{ .s1lo = 0x04, .want = 0x777 },
        .{ .s1lo = 0x10, .want = 0x777 },
        .{ .s1lo = 0x14, .want = 0x777 },
        .{ .s1lo = 0x18, .want = 0x777 },
        .{ .s1lo = 0x1C, .want = 0x777 },
        .{ .s1lo = 0x08, .want = 0x333 },
        .{ .s1lo = 0x0C, .want = 0x333 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x333;
        state.s[4] = 0x777;
        _ = try emu.runFrom(gpa, &state, &.{ 0xc6000016, 0x05002000 | c.s1lo, 0x86000044, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want.?, state.s[2]);
    }
    // Another cell (head[30] = 1, mx 0, cell 2) with S1[4:3] = 01: unmeasured.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0xc6000016, 0x05002008, 0x86000042, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "ytst stores X with S1[8] and S1[3]; head[29] and head[24:21] refuse (fact 625)" {
    const gpa = testing.allocator;
    // own-x14st 00000014 0221016B: X:[A12] <- sx24 s6.
    for ([_]struct { head: u32 = 0x00000014, s1: u32 = 0x0221016B, at: u32 = 0x300, want: ?u32 }{
        .{ .want = 0xFF8123 },
        .{ .head = 0x40000014, .at = 0x400, .want = 0xFF8123 }, // head class 2: A index {011, 01} = 13
        .{ .s1 = 0x0221006B, .want = 0 }, // S1[8] = 0
        .{ .s1 = 0x02210163, .want = 0 }, // S1[3] = 0
        .{ .s1 = 0x02210763, .want = 0 }, // S1[10:9] alone
        .{ .head = 0x20000014, .want = null }, // head[29]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0xFF8123;
        state.a[12] = 0x300;
        state.a[13] = 0x400;
        const r = emu.runFrom(gpa, &state, &.{ c.head, c.s1, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.readX(c.at) orelse 0);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x02's third slot with head code 7 is asrrs, w5[5] negating the count (fact 627)" {
    const gpa = testing.allocator;
    // s3 <- asrrs(s5 = 0x3303 by s4); low 16 bits as measured.
    const neg = [_][2]i32{ .{ 0, 0x3303 }, .{ 1, 0x6606 }, .{ 2, 0xCC0C }, .{ 4, 0x3030 }, .{ 7, 0x8180 }, .{ 8, 0x0300 }, .{ 9, 0x0600 }, .{ 12, 0xFFFF }, .{ 20, 0xFFFF }, .{ -8, 0x33 }, .{ -1, 0x1982 } };
    const pos = [_][2]i32{ .{ 4, 0x0330 }, .{ -1, 0x6606 } };
    for ([_]struct { w5: u32, rows: []const [2]i32 }{
        .{ .w5 = 0xc00200B0, .rows = &neg },
        .{ .w5 = 0xc0020090, .rows = &pos },
        .{ .w5 = 0xc00201DC, .rows = &pos }, // w5[8:6], w5[4:2] inert
    }) |c| for (c.rows) |r| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x3303;
        state.s[4] = @as(u32, @bitCast(r[0])) & 0xFFFFFF;
        _ = try runX02(gpa, &state, .{ 0x010650E2, 0x00002048, 0x00001432, 0x00000000, 0xD5000480, c.w5 });
        try testing.expectEqual(@as(u32, @intCast(r[1])), state.s[3] & 0xFFFF);
    };
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, runX02(gpa, &o, .{ 0x010650E2, 0x00002048, 0x00001432, 0x00000000, 0xD5000480, 0xc0030090 }));
}

test "x02's third slot: code 6 asr, codes 22/31 XOR_M, codes 32/34 clear (fact 628)" {
    const gpa = testing.allocator;
    // s3 <- f(a = s4, b = s5); head code in h[10:5]; w5 base 0xc0020000 (dest s3, w5[16] = 0).
    const Row = struct { code: u32, w5lo: u32, a: u32, b: u32, want: ?u32 };
    for ([_]Row{
        .{ .code = 6, .w5lo = 0x20, .a = 0xFFFFFF, .b = 0x3303, .want = 0x1981 }, // no rounding
        .{ .code = 6, .w5lo = 0x20, .a = 30, .b = 0x3303, .want = 0 }, // huge left
        .{ .code = 6, .w5lo = 0x00, .a = 4, .b = 0x3303, .want = 0x330 },
        .{ .code = 6, .w5lo = 0x04, .a = 4, .b = 0x3303, .want = null }, // w5[4:2] OPEN
        .{ .code = 31, .w5lo = 0 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0x1D3B },
        .{ .code = 31, .w5lo = 1 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0xFFE2C4 },
        .{ .code = 31, .w5lo = 2 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0x1F3F },
        .{ .code = 31, .w5lo = 3 << 6 | 0x3C, .a = 0x1234, .b = 0x0F0F, .want = 0x0204 }, // w5[5:2] ignored
        .{ .code = 31, .w5lo = 4 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0xFFEDCB },
        .{ .code = 31, .w5lo = 5 << 6, .a = 0x1234, .b = 0x0F0F, .want = null }, // f5 OPEN
        .{ .code = 31, .w5lo = 6 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0x1234 },
        .{ .code = 31, .w5lo = 7 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0x1234 },
        .{ .code = 22, .w5lo = 3 << 6 | 4 << 2, .a = 0x1234, .b = 0x0F0F, .want = 0x2040 },
        .{ .code = 22, .w5lo = 3 << 6, .a = 0x1234, .b = 0x0F0F, .want = 0x0204 },
        .{ .code = 32, .w5lo = 0, .a = 0x1234, .b = 0x0F0F, .want = 0 },
        .{ .code = 34, .w5lo = 0, .a = 0x1234, .b = 0x0F0F, .want = 0 },
        .{ .code = 9, .w5lo = 0, .a = 0x1234, .b = 0x0F0F, .want = null },
        .{ .code = 0x17, .w5lo = 0, .a = 0x1234, .b = 0x0F0F, .want = null }, // other xor codes
        .{ .code = 31, .w5lo = 0x10000, .a = 0x1234, .b = 0x0F0F, .want = null }, // w5[16]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = 0x777;
        state.s[4] = c.a;
        state.s[5] = c.b;
        const r = runX02(gpa, &state, .{ 0x01065002 | c.code << 5, 0x00002048, 0x00001432, 0x00000000, 0xD5000480, 0xc0020000 | c.w5lo });
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[3]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x17agu2 writes s<S1[22:19]> with the bare alu05 op (fact 629)" {
    const gpa = testing.allocator;
    // own-x17a2s 18002a57 284b0d00 00000000: s9 <- rsub(a = s6, b = s0) = -0x8123.
    for ([_]struct { code: u32, want: ?u32 }{
        .{ .code = 0x12, .want = 0xFF7EDD },
        .{ .code = 0x23, .want = 0x8123 }, // mova
        .{ .code = 0x00, .want = 0x8123 }, // add, h[25:19] not applied
        .{ .code = 0x10, .want = 0x8123 },
        .{ .code = 0x16, .want = 0x8123 },
        .{ .code = 0x30, .want = 0 },
        .{ .code = 0x09, .want = 0 },
        .{ .code = 0x20, .want = 0 },
        .{ .code = 0x06, .want = 0 }, // asr(0 >> a)
        .{ .code = 0x14, .want = null }, // nadd
        .{ .code = 0x26, .want = null }, // sel
        .{ .code = 0x08, .want = null }, // lsr
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0x8123;
        const r = emu.runFrom(gpa, &state, &.{ (0x18002a57 & ~@as(u32, 0x7E0)) | c.code << 5, 0x284b0d00, 0x00000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[9]);
        } else try testing.expectError(error.UnknownForm, r);
    }
    // b = s5 = 0x0100 (S2[28:25] = 5), a = s6 = 0x0123.  Derived from 629's op list.
    for ([_][2]u32{ .{ 0x00, 0x0223 }, .{ 0x10, 0x0023 }, .{ 0x12, 0xFFFFDD }, .{ 0x16, 0x0023 }, .{ 0x30, 0x0100 }, .{ 0x09, 0x0100 }, .{ 0x23, 0x0123 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0x0123;
        state.s[5] = 0x0100;
        _ = try emu.runFrom(gpa, &state, &.{ (0x18002a57 & ~@as(u32, 0x7E0)) | c[0] << 5, 0x284b0d00, 0x0A000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], state.s[9]);
    }
    // The ALU and AGU s writes on one register (S2[13:10] = 9): unmeasured.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x18002a57, 0x284b0d00, 0x00002400, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "x17agu2's AGU x = e<S2[24:21]> and the e<k> <- e<j> - 1 write (fact 630, 631)" {
    const gpa = testing.allocator;
    // own-x17a2x 40002c17 2de83000 00a02400, e5 = 0x457: fn 0 gives s9 = 0x457, fn 2 0x458.
    for ([_][2]u32{ .{ 0, 0x457 }, .{ 2, 0x458 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x457;
        _ = try emu.runFrom(gpa, &state, &.{ 0x40002c17, (0x2de83000 & ~@as(u32, 0x780)) | c[0] << 7, 0x00a02400, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], state.s[9]);
    }
    // own-x17a2e 20001817 68000501 0014004d: e3 <- e5 - 1; S1[31:30] = 2 reads e6.  e2 (the
    // AGU's own e write) takes y + i = R4 + 3.
    for ([_]struct { s1: u32, want: u32 }{ .{ .s1 = 0x68000501, .want = 0x456 }, .{ .s1 = 0xA8000501, .want = 0x566 } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x457;
        state.e[6] = 0x567;
        state.r[4] = 0x100;
        _ = try emu.runFrom(gpa, &state, &.{ 0x20001817, c.s1, 0x0014004d, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.e[3]);
        try testing.expectEqual(@as(u32, 0x103), state.e[2]);
    }
    // j = 2 = the AGU's dest e2: the source is read before the write.
    var pre = emu.State{};
    defer pre.deinit(gpa);
    pre.e[2] = 0x222;
    _ = try emu.runFrom(gpa, &pre, &.{ 0x20001817, 0xA8000501, 0x0014004c, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x221), pre.e[3]);
    // k = dest e2: unmeasured.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x20001817, 0x68000501, 0x00140049, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "ptst op 0x1E tests a & b at c 14 (s10) and 15 (s12) (fact 632, 633)" {
    const gpa = testing.allocator;
    const Row = struct { head: u32, a_i: u32, b_i: u32, b: u32, want: ?u8 };
    for ([_]Row{
        .{ .head = 0x45d533cd, .a_i = 6, .b_i = 10, .b = 4, .want = 1 }, // own-ptst-and
        .{ .head = 0x45d533cd, .a_i = 6, .b_i = 10, .b = 2, .want = 0 },
        .{ .head = 0x45e55bcd, .a_i = 11, .b_i = 12, .b = 4, .want = 1 }, // own-ptst-and12
        .{ .head = 0x45e55bcd, .a_i = 11, .b_i = 12, .b = 2, .want = 0 },
        .{ .head = 0x45b533cd, .a_i = 6, .b_i = 10, .b = 4, .want = null }, // c 13
        .{ .head = 0x45d5b3cd, .a_i = 6, .b_i = 10, .b = 4, .want = null }, // h[18:15] = 11
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[1] = 7;
        state.s[c.a_i] = 5;
        state.s[c.b_i] = c.b;
        const r = emu.runFrom(gpa, &state, &.{ c.head, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.p[1]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "ldi2 writes s and A from the head and stores Y from the old s (fact 634)" {
    const gpa = testing.allocator;
    // own-ldi2y 18006010 2a980000: Y:[e5 = 0x710] <- s4 = 0x1234; A12 <- 0x300.
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[5] = 0x710;
    st.s[4] = 0x1234;
    _ = try emu.runFrom(gpa, &st, &.{ 0x18006010, 0x2a980000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1234), st.getY(0x710).?);
    try testing.expectEqual(@as(u32, 0x300), st.a[12]);
    // Head s4 <- 0x8001 (sx16 -> 0xFF8001), A17 (h[29]) <- 0x8001; the store reads the old s4.
    var o = emu.State{};
    defer o.deinit(gpa);
    o.e[5] = 0x710;
    o.s[4] = 0x1234;
    _ = try emu.runFrom(gpa, &o, &.{ 0x22900030, 0x2a980000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1234), o.getY(0x710).?);
    try testing.expectEqual(@as(u32, 0xFF8001), o.s[4]);
    try testing.expectEqual(@as(u32, 0x8001), o.a[17]);
    // Refused: h[31:30] != 0, S[18:0] != 0, S[20:19] = 00.
    for ([_][2]u32{ .{ 0x58006010, 0x2a980000 }, .{ 0x18006010, 0x2a980001 }, .{ 0x18006010, 0x2a800000 } }) |c| {
        var r = emu.State{};
        defer r.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &r, &.{ c[0], c[1], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
}

test "ytst's head writes s<h[24:21]> <- e<S1[24:21]> and A<h[28:25]> <- # (fact 635)" {
    const gpa = testing.allocator;
    // own-ytst-e 01224694 0aa00600: s9 <- e5 = 0x710 (not Y).
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[5] = 0x710;
    try st.y_mem.put(gpa, 0x710, 0x0BEE);
    _ = try emu.runFrom(gpa, &st, &.{ 0x01224694, 0x0aa00600, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x710), st.s[9]);
    // A7 (h[28:25] = 7) <- 0x1234.
    var a = emu.State{};
    defer a.deinit(gpa);
    _ = try emu.runFrom(gpa, &a, &.{ 0x0f224694, 0x0aa00600, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1234), a.a[7]);
    // The X store's A (A12 = {011, 00}) is the head's A dest: unmeasured.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x18000014, 0x0221016B, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "ldi2 admits the class-001 IMM10 seconds as no-store; A dest refused there (fact 74, 634)" {
    const gpa = testing.allocator;
    for ([_]u32{ 0x20040000, 0x20400000, 0x20440000, 0x22463000 }) |sw| {
        var st = emu.State{};
        defer st.deinit(gpa);
        _ = try emu.runFrom(gpa, &st, &.{ 0x01a02470, sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x123), st.s[13]);
        try testing.expectEqual(@as(usize, 0), st.y_mem.count());
        var a = emu.State{};
        defer a.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &a, &.{ 0x03a02470, sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
    // 20004001 / 20014000: not IMM10 seconds, refused.
    for ([_]u32{ 0x20004001, 0x20014000 }) |sw| {
        var st = emu.State{};
        defer st.deinit(gpa);
        try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &st, &.{ 0x01a02470, sw, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
}

test "x19agu s mux 2 runs AGU10Q with q = e<w2[7:4]> (fact 636)" {
    const gpa = testing.allocator;
    // own-x19q 01200019 02042000 01000050 00001804: e5 = q = 0x457, fn 2 -> s9 = 0x457; fn 12
    // gives y - q = 0x2BAC (R3 = 0x3003, derived from that row).  x = e0 reads 0.
    const want = [16]u32{ 0, 0x3003, 0x457, 0, 0xCFFD, 0x458, 0x3003, 0xCFFD, 0, 0, 0x3003, 0x3003, 0x2BAC, 0xD454, 0x2BAD, 0x2BAB };
    for (want, 0..) |v, f| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[5] = 0x457;
        st.r[3] = 0x3003;
        _ = try emu.runFrom(gpa, &st, &.{ 0x01200019, 0x00042000 | @as(u32, @intCast(f)) << 24, 0x01000050, 0x00001804, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(v, st.s[9]);
    }
    // x = e6 = 0x100 (w3[28:25] = 6): fn 0 x, fn 3 -x, fn 6 x + y, fn 7 x - y.
    for ([_][2]u32{ .{ 0, 0x100 }, .{ 3, 0xFF00 }, .{ 6, 0x3103 }, .{ 7, 0xD0FD } }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[5] = 0x457;
        st.e[6] = 0x100;
        st.r[3] = 0x3003;
        _ = try emu.runFrom(gpa, &st, &.{ 0x01200019, 0x00042000 | c[0] << 24, 0x01000050, 0x0c001804, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[1], st.s[9]);
    }
    // w2[25] = 1: AGU10 (fn 6 x + y).
    var a = emu.State{};
    defer a.deinit(gpa);
    a.e[6] = 0x100;
    a.r[3] = 0x3003;
    _ = try emu.runFrom(gpa, &a, &.{ 0x01200019, 0x06042000, 0x03000050, 0x0c001804, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x3103), a.s[9]);
}

test "x19agu s mux 3 loads Y[7:0] << 8 under w2[26] (fact 637)" {
    const gpa = testing.allocator;
    // own-x19m3 01200019 00043000 04000000 0a000000: s9 <- Y:[e5 = 0x710][7:0] << 8.
    for ([_]struct { y: u32, w2: u32 = 0x04000000, w3: u32 = 0x0a000000, want: ?u32 }{
        .{ .y = 0x12AB, .want = 0xAB00 },
        .{ .y = 0x3471, .want = 0x7100 },
        .{ .y = 0x12AB, .w2 = 0x00000000, .want = null }, // w2[26] = 0
        .{ .y = 0x12AB, .w2 = 0x04010000, .want = null }, // w2[16] = 1
        .{ .y = 0x12AB, .w3 = 0x0a000002, .want = null }, // w3[1] = 1
    }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[5] = 0x710;
        try st.y_mem.put(gpa, 0x710, c.y);
        const r = emu.runFrom(gpa, &st, &.{ 0x01200019, 0x00043000, c.w2, c.w3, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, st.s[9]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x16agu (120 form) s mux 3 loads Y[7:0] << 8 (fact 638)" {
    const gpa = testing.allocator;
    // own-x16y 50000016 8941b202 a0a00148 (agumul's S1 class 100, see below): s9 <- Y:[e5 =
    // 0x710][7:0] << 8; S1[31] cleared (class 000) runs the slot alone.
    for ([_]struct { y: u32, s1: u32 = 0x8941b202, want: u32 }{
        .{ .y = 0x12AB, .want = 0xAB00 },
        .{ .y = 0x3471, .want = 0x7100 },
        .{ .y = 0x12AB, .s1 = 0x0941b202, .want = 0xAB00 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[5] = 0x710;
        try st.y_mem.put(gpa, 0x710, c.y);
        _ = try emu.runFrom(gpa, &st, &.{ 0x50000016, c.s1, 0xa0a00148, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, st.s[9]);
    }
}

test "x16mul's head immediate lands with S1[14:13] = 01; 10 refuses (fact 639)" {
    const gpa = testing.allocator;
    // own-x16i 01224696 11042000 c0400200: s9 <- 0x1234.
    for ([_]struct { s1: u32, want: ?u32 }{ .{ .s1 = 0x11042000, .want = 0x1234 }, .{ .s1 = 0x11044000, .want = null } }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        const r = emu.runFrom(gpa, &st, &.{ 0x01224696, c.s1, 0xc0400200, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, st.s[9]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "agumul with S1[30:28] = 000 also runs the 120 slot; both read the old s (fact 638)" {
    const gpa = testing.allocator;
    // own-x16y: s2 <- s1 * s2 >> 23 beside s9 <- Y[7:0] << 8.
    for ([_][3]u32{ .{ 0x4000, 0x4000, 0x20 }, .{ 0x2000, 0x6000, 0x18 } }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[5] = 0x710;
        try st.y_mem.put(gpa, 0x710, 0x12AB);
        st.s[1] = c[0];
        st.s[2] = c[1];
        _ = try emu.runFrom(gpa, &st, &.{ 0x50000016, 0x8941b202, 0xa0a00148, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c[2], st.s[2]);
        try testing.expectEqual(@as(u32, 0xAB00), st.s[9]);
    }
    // own-x16y1 (own-x16y with S1[12:9] = 1, silicon 3 of 3): the slot writes s1 = 0x5F00, the product still
    // reads the old s1.
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[5] = 0x710;
    try st.y_mem.put(gpa, 0x710, 0x125F);
    st.s[1] = 0x4000;
    st.s[2] = 0x4000;
    _ = try emu.runFrom(gpa, &st, &.{ 0x50000016, 0x8941a202, 0xa0a00148, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x20), st.s[2]);
    try testing.expectEqual(@as(u32, 0x5F00), st.s[1]);
    // Slot dest = product dest s2: unmeasured.
    var o = emu.State{};
    defer o.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &o, &.{ 0x50000016, 0x8941a402, 0xa0a00148, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "x19agu: w2[24]'s immediate goes to e only, w2[30] inert, s mux 6 writes 0 (fact 640)" {
    const gpa = testing.allocator;
    // own-x19aimm 00002479 16540000 03c40000 06000980: e5 <- #0x123, R3 <- e3 + R1.
    var a = emu.State{};
    defer a.deinit(gpa);
    a.e[3] = 0x430;
    a.r[1] = 9;
    a.s[3] = 0x777;
    _ = try emu.runFrom(gpa, &a, &.{ 0x00002479, 0x16540000, 0x03c40000, 0x06000980, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x123), a.e[5]);
    try testing.expectEqual(@as(u32, 0x439), a.r[3]);
    // own-x19r 00000199 00e40000 53000440 0e001380 (w2 class 010): R7 <- e7 = 0x123.
    var r = emu.State{};
    defer r.deinit(gpa);
    r.e[7] = 0x123;
    _ = try emu.runFrom(gpa, &r, &.{ 0x00000199, 0x00e40000, 0x53000440, 0x0e001380, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x123), r.r[7]);
    // s mux 6 (w1[14:12] = 110) writes 0 to s<h[24:21]>.
    var m = emu.State{};
    defer m.deinit(gpa);
    m.e[7] = 0x123;
    m.s[9] = 0x777;
    _ = try emu.runFrom(gpa, &m, &.{ 0x01200199, 0x00e46000, 0x53000440, 0x0e001380, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0), m.s[9]);
}

const ScanFile = struct { path: []const u8, text: []const u8 };
const libs_scan = blk: {
    var out: [libs.len]ScanFile = undefined;
    for (libs, 0..) |l, i| out[i] = .{ .path = l.path, .text = l.text };
    break :blk out;
};

test "the ISA sources quote no image provenance" {
    // Facts here are bit semantics and our own programs.  Words copied from the image, or text
    // naming where a row came from, stay in the RE tree.  Phrases are split so this test does not
    // match itself.
    const banned = [_][]const u8{
        "App" ++ "le",   "image " ++ "row",     "image " ++ "pair", "image " ++ "word", "image" ++ "'s",
        "sm1_" ++ "isa", "sm1_" ++ "spec",      "Chik" ++ "uma",    "chik" ++ "uma",    "ipod" ++ "-re",
        "iPod" ++ "-re", "programs" ++ ".toml", "reports" ++ "/",
    };
    const files = [_]ScanFile{
        .{ .path = "src/isa.toml", .text = @embedFile("isa.toml") },
        .{ .path = "src/subforms.txt", .text = @embedFile("subforms.txt") },
        .{ .path = "src/operands.txt", .text = @embedFile("operands.txt") },
        .{ .path = "src/emu.zig", .text = @embedFile("emu.zig") },
        .{ .path = "src/isa.zig", .text = @embedFile("isa.zig") },
        .{ .path = "src/lib_test.zig", .text = @embedFile("lib_test.zig") },
        .{ .path = "src/silicon_gates.zig", .text = @embedFile("silicon_gates.zig") },
        .{ .path = "docs/sm1/handbook.md", .text = @embedFile("handbook.md") },
        .{ .path = "docs/sm1/reference.md", .text = @embedFile("reference.md") },
        .{ .path = "README.md", .text = @embedFile("README.md") },
        .{ .path = "ARCHITECTURE.md", .text = @embedFile("ARCHITECTURE.md") },
        .{ .path = "ASSEMBLY.md", .text = @embedFile("ASSEMBLY.md") },
        .{ .path = "PROBES.md", .text = @embedFile("PROBES.md") },
        .{ .path = "src/host.zig", .text = @embedFile("host.zig") },
        .{ .path = "src/asm.zig", .text = @embedFile("asm.zig") },
        .{ .path = "src/disasm.zig", .text = @embedFile("disasm.zig") },
        .{ .path = "src/image.zig", .text = @embedFile("image.zig") },
        .{ .path = "src/doc.zig", .text = @embedFile("doc.zig") },
        .{ .path = "src/summaries.txt", .text = @embedFile("summaries.txt") },
        .{ .path = "src/dispatch.zig", .text = @embedFile("dispatch.zig") },
        .{ .path = "src/expand.zig", .text = @embedFile("expand.zig") },
        .{ .path = "src/fmt.zig", .text = @embedFile("fmt.zig") },
        .{ .path = "src/main.zig", .text = @embedFile("main.zig") },
    } ++ libs_scan;
    for (files) |f| for (banned) |b| if (std.mem.indexOf(u8, f.text, b)) |at| {
        std.debug.print("{s}: offset {d} carries '{s}'\n", .{ f.path, at, b });
        return error.ProvenanceText;
    };
}

fn x18c3Run(head: u32, w1: u32, w2: u32, s5: u32) !emu.State {
    var state = emu.State{};
    errdefer state.deinit(testing.allocator);
    // The 641-645 silicon rows ran with a stale p3 = 1, which the S codes read (752).
    state.p[3] = 1;
    state.s[5] = s5;
    state.s[7] = 3;
    _ = try emu.runFrom(testing.allocator, &state, &.{ head, w1, w2, 0x74000149, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    return state;
}

fn x18c3Head(code: u32) u32 {
    return (0x21c00018 & ~@as(u32, 0x3F << 25)) | (code << 25);
}

test "x18 class 011 codes 0x10 and 0x23 run the ALU only for n in {2,5,6,7,9}" {
    // own-x18b (21c00018 00001970 30000014 74000149): code 0x10, op 0, n = 0, a = s5 = 5, b = s7 = 3: table B op 0 = b - a.
    const want_b = [8]u32{ 0xFFFFFE, 0xFFFFF7, 0xFFFFFB, 0xFFFFFB, 0xFFFFFA, 9, 6, 0xFFFFFB };
    for (want_b, 0..) |want, op| {
        var st = try x18c3Run(x18c3Head(0x10), 0x00001970 | (@as(u32, @intCast(op)) << 15), 0x30000014 | (1 << 6), 5);
        defer st.deinit(testing.allocator);
        try testing.expectEqual(want, st.s[9]);
    }
    for ([_]u32{ 2, 5, 6, 7, 9 }) |n| {
        var st = try x18c3Run(x18c3Head(0x10), 0x00001970, 0x30000014 | (n << 6), 5);
        defer st.deinit(testing.allocator);
        try testing.expectEqual(@as(u32, 8), st.s[9]); // slot op 0: a + b
        var s2 = try x18c3Run(x18c3Head(0x23), 0x00001970, 0x30000014 | (n << 6), 5);
        defer s2.deinit(testing.allocator);
        try testing.expectEqual(@as(u32, 8), s2.s[9]);
    }
    for ([_]u32{ 0, 1, 3, 4, 8, 10, 15 }) |n| {
        var st = try x18c3Run(x18c3Head(0x23), 0x00001970, 0x30000014 | (n << 6), 5);
        defer st.deinit(testing.allocator);
        try testing.expectEqual(@as(u32, 3), st.s[9]); // writes b
    }
    // op 7: a < 0 picks a + 1, else -a; a where 16- and 24-bit signs disagree refuses
    var neg = try x18c3Run(x18c3Head(0x10), 0x00001970 | (7 << 15), 0x30000014, 0xFFFFFB);
    defer neg.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xFFFFFC), neg.s[9]);
    try testing.expectError(error.UnknownForm, x18c3Run(x18c3Head(0x10), 0x00001970 | (7 << 15), 0x30000014, 0x8000));
    // other codes: binary op with n != 0 refuses
    try testing.expectError(error.UnknownForm, x18c3Run(x18c3Head(0x09), 0x00001970, 0x30000014 | (1 << 6), 5));
}

test "x18 class 011 unary ops shift the result by n under codes 0x00-0x05" {
    // own-x18s (01c00018 00019970 30000054 74000149): code 0x00, op 3 (a + 1), n = 1: (5 + 1) << 1 = 12.
    var st = try x18c3Run(0x01c00018, 0x00019970, 0x30000054, 5);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xC), st.s[9]);
    var l2 = try x18c3Run(x18c3Head(0x02), 0x00019970, 0x30000014 | (3 << 6), 5);
    defer l2.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 48), l2.s[9]);
    var r = try x18c3Run(x18c3Head(0x03), 0x00019970, 0x30000014 | (1 << 6), 5);
    defer r.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 3), r.s[9]);
    // floor: (-5 + 1) >> 3 = -1
    var rn = try x18c3Run(x18c3Head(0x05), 0x00019970, 0x30000014 | (3 << 6), 0xFFFFFB);
    defer rn.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xFFFFFF), rn.s[9]);
    // left shift of a negative result keeps its sign; past 24 bits is refused
    var w = try x18c3Run(x18c3Head(0x01), 0x00011970, 0x30000014 | (4 << 6), 0xFFFFFB);
    defer w.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xFFFFB0), w.s[9]);
    try testing.expectError(error.UnknownForm, x18c3Run(x18c3Head(0x00), 0x00011970, 0x30000014 | (4 << 6), 0x7FFFFF));
    // a binary op under 0x16 runs the slot ALU at n = 0
    var b = try x18c3Run(x18c3Head(0x16), 0x00001970, 0x30000014, 5);
    defer b.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 8), b.s[9]);
}

/// Runs one class-011 bundle, s9 <- the slot.  p1 is the p destination, for codes whose
/// head[28:25] is the p-write condition 11.
fn x18c3Unary(code: u32, op: u32, n: u32, s5: u32, s7: u32) !u32 {
    var state = emu.State{};
    defer state.deinit(testing.allocator);
    state.p[3] = 1; // the silicon rows' stale p3 (752)
    state.s[5] = s5;
    state.s[7] = s7;
    _ = try emu.runFrom(testing.allocator, &state, &.{ x18c3Head(code), 0x00001970 | (op << 15), 0x30000014 | (n << 6) | (1 << 14), 0x74000149, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    return state.s[9];
}

test "x18 class 011 unary ops under codes 0x16-0x19, 0x30-0x35, 0x3A (643)" {
    // own-x18l (2dc00018 00019970 30000054 74000149): s5 = 7, s7 = -2, code 0x16, op 3 (a & b), n = 1: (7 & -2) << 1 = 12.
    try testing.expectEqual(@as(u32, 12), try x18c3Unary(0x16, 3, 1, 7, 0xFFFFFE));
    const a: u32 = 0x000C35; // 3125
    const b: u32 = 0x0000F0;
    const C = struct { code: u32, op: u32, n: u32, want: u32 };
    const M24: u32 = 0xFFFFFF;
    for ([_]C{
        // LOGIC << n, >> n
        .{ .code = 0x16, .op = 2, .n = 2, .want = (a | b) << 2 },
        .{ .code = 0x17, .op = 4, .n = 0, .want = ~b & M24 },
        .{ .code = 0x17, .op = 6, .n = 3, .want = b << 3 },
        .{ .code = 0x18, .op = 7, .n = 4, .want = b >> 4 },
        .{ .code = 0x19, .op = 3, .n = 1, .want = (a & b) >> 1 },
        .{ .code = 0x19, .op = 4, .n = 4, .want = 0xFFFFF0 }, // ~0xF0 >> 4, floor
        // 0x30-0x35: ops 2/6/7 shift a, ops 3/4 give a
        .{ .code = 0x30, .op = 2, .n = 3, .want = a << 3 },
        .{ .code = 0x31, .op = 3, .n = 3, .want = a },
        .{ .code = 0x32, .op = 4, .n = 5, .want = a },
        .{ .code = 0x33, .op = 6, .n = 3, .want = a >> 3 },
        .{ .code = 0x35, .op = 7, .n = 2, .want = a >> 2 },
        .{ .code = 0x35, .op = 3, .n = 2, .want = a },
        .{ .code = 0x34, .op = 2, .n = 3, .want = (a + 4) >> 3 }, // 391
        .{ .code = 0x34, .op = 2, .n = 0, .want = a },
        .{ .code = 0x34, .op = 4, .n = 3, .want = a },
        // 0x3A: ops 2/3 a << n, ops 4/6/7 a >> n
        .{ .code = 0x3A, .op = 3, .n = 2, .want = a << 2 },
        .{ .code = 0x3A, .op = 6, .n = 2, .want = a >> 2 },
    }) |c| try testing.expectEqual(c.want, try x18c3Unary(c.code, c.op, c.n, a, b));
    // floor and rounding on a negative a: -5
    try testing.expectEqual(@as(u32, 0xFFFFFE), try x18c3Unary(0x33, 2, 2, 0xFFFFFB, 0)); // floor(-5/4) = -2
    try testing.expectEqual(@as(u32, 0xFFFFFF), try x18c3Unary(0x34, 2, 2, 0xFFFFFB, 0)); // (-5+2)>>2 = -1
    try testing.expectEqual(@as(u32, 390), try x18c3Unary(0x34, 7, 3, 3121, 0)); // 390.125 rounds down
    try testing.expectEqual(@as(u32, 391), try x18c3Unary(0x34, 6, 3, 3124, 0)); // 390.5 rounds up
    try testing.expectEqual(@as(u32, 0xFFFFF6), try x18c3Unary(0x3A, 2, 1, 0xFFFFFB, 0));
    // left shifts past 24 bits refuse
    try testing.expectError(error.UnknownForm, x18c3Unary(0x30, 2, 2, 0x300000, 0));
    try testing.expectError(error.UnknownForm, x18c3Unary(0x16, 6, 1, 0, 0x400000));
}

test "x18 class 011 slot ops 4 and 6: a - 1 and -a (fact 641 rows)" {
    // code 0x10, n = 2 runs the slot ALU: a = 5 -> 4 / -5; 7 -> 6 / -7; -4 -> -5 / 4.
    for ([_][3]u32{ .{ 5, 4, 0xFFFFFB }, .{ 7, 6, 0xFFFFF9 }, .{ 0xFFFFFC, 0xFFFFFB, 4 } }) |c| {
        try testing.expectEqual(c[1], try x18c3Unary(0x10, 4, 2, c[0], 0));
        try testing.expectEqual(c[2], try x18c3Unary(0x10, 6, 2, c[0], 0));
    }
    // unary under 0x00-0x02 shifts the op result: (5 - 1) << 1
    try testing.expectEqual(@as(u32, 8), try x18c3Unary(0x00, 4, 1, 5, 0));
}

test "x18 class 011 unary ops under the 45 remaining codes (644)" {
    // own-x18m (4dc00018 00011970 30000014 74000149): s5 = -4, s7 = 9, code 0x26, op 2: min(-4, 9) = -4.
    var st = try x18c3Run(0x4dc00018, 0x00011970, 0x30000014, 0xFFFFFC);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xFFFFFC), st.s[9]); // the exact words, s7 = 3 here
    try testing.expectEqual(@as(u32, 0xFFFFFC), try x18c3Unary(0x26, 2, 0, 0xFFFFFC, 9));
    const N4: u32 = 0xFFFFFC; // -4
    const M2: u32 = 0xFFFFFE; // -2
    const C = struct { code: u32, op: u32, n: u32, a: u32, b: u32, want: u32 };
    for ([_]C{
        // 0x06-0x08: shr / rshr / lsr of the slot result by b
        .{ .code = 0x06, .op = 2, .n = 0, .a = N4, .b = 9, .want = 0xFFFFFF },
        .{ .code = 0x06, .op = 2, .n = 1, .a = 7, .b = M2, .want = 28 },
        .{ .code = 0x07, .op = 4, .n = 0, .a = 7, .b = 3, .want = 1 }, // (6 + 4) >> 3
        .{ .code = 0x08, .op = 2, .n = 2, .a = N4, .b = 9, .want = 0x7FFF },
        // 0x09-0x0F: the slot ALU, op 7 abs
        .{ .code = 0x09, .op = 7, .n = 0, .a = N4, .b = 0, .want = 4 },
        .{ .code = 0x0C, .op = 4, .n = 2, .a = 5, .b = 0, .want = 4 },
        // 0x11 = 0x10
        .{ .code = 0x11, .op = 2, .n = 0, .a = 5, .b = 3, .want = 0xFFFFFB },
        .{ .code = 0x11, .op = 2, .n = 2, .a = 5, .b = 3, .want = 5 },
        // 0x12 / 0x13, 0x14 / 0x15: S picks the ALU
        .{ .code = 0x12, .op = 3, .n = 0, .a = 5, .b = 3, .want = 4 },
        .{ .code = 0x13, .op = 4, .n = 1, .a = 5, .b = 3, .want = 6 },
        .{ .code = 0x12, .op = 6, .n = 0, .a = 5, .b = 3, .want = 0xFFFFFA },
        .{ .code = 0x12, .op = 3, .n = 5, .a = 5, .b = 3, .want = 6 },
        .{ .code = 0x14, .op = 2, .n = 1, .a = 5, .b = 3, .want = 0xFFFFF9 },
        .{ .code = 0x15, .op = 3, .n = 0, .a = 5, .b = 3, .want = 0xFFFFFA },
        .{ .code = 0x14, .op = 6, .n = 0, .a = 5, .b = 3, .want = 5 },
        .{ .code = 0x15, .op = 6, .n = 9, .a = 5, .b = 3, .want = 0xFFFFFB },
        // 0x1A-0x1F: LOGIC with #n or b
        .{ .code = 0x1A, .op = 2, .n = 2, .a = 5, .b = M2, .want = 28 },
        .{ .code = 0x1B, .op = 4, .n = 1, .a = 5, .b = 9, .want = 0x7FFF },
        .{ .code = 0x1C, .op = 3, .n = 2, .a = 7, .b = 3, .want = 2 },
        .{ .code = 0x1D, .op = 2, .n = 0, .a = 5, .b = 3, .want = 7 },
        .{ .code = 0x1F, .op = 4, .n = 1, .a = 5, .b = 3, .want = 0xFFFFFC },
        .{ .code = 0x1E, .op = 6, .n = 0, .a = 5, .b = 3, .want = 3 },
        // 0x20-0x22: 0
        .{ .code = 0x20, .op = 2, .n = 0, .a = 5, .b = 3, .want = 0 },
        .{ .code = 0x21, .op = 3, .n = 1, .a = 5, .b = 3, .want = 0 },
        .{ .code = 0x22, .op = 6, .n = 2, .a = 5, .b = 3, .want = 0 },
        // 0x24 / 0x25
        .{ .code = 0x24, .op = 3, .n = 0, .a = 5, .b = 3, .want = 3 },
        .{ .code = 0x24, .op = 3, .n = 2, .a = 5, .b = 3, .want = 6 },
        .{ .code = 0x25, .op = 3, .n = 6, .a = 5, .b = 3, .want = 5 },
        .{ .code = 0x25, .op = 3, .n = 0, .a = 5, .b = 3, .want = 1 },
        // min / max
        .{ .code = 0x26, .op = 6, .n = 0, .a = N4, .b = 9, .want = 9 },
        .{ .code = 0x2A, .op = 3, .n = 1, .a = N4, .b = 9, .want = N4 },
        .{ .code = 0x2A, .op = 4, .n = 0, .a = 7, .b = M2, .want = M2 },
        .{ .code = 0x27, .op = 2, .n = 0, .a = 5, .b = 3, .want = 3 },
        .{ .code = 0x2B, .op = 6, .n = 0, .a = 5, .b = 3, .want = 5 },
        .{ .code = 0x28, .op = 2, .n = 2, .a = 5, .b = 3, .want = 2 },
        .{ .code = 0x28, .op = 6, .n = 1, .a = N4, .b = 3, .want = 1 },
        .{ .code = 0x29, .op = 3, .n = 2, .a = 5, .b = 3, .want = 2 },
        .{ .code = 0x29, .op = 6, .n = 2, .a = 5, .b = 3, .want = 5 },
        .{ .code = 0x2C, .op = 6, .n = 2, .a = 5, .b = 3, .want = 20 },
        .{ .code = 0x2D, .op = 2, .n = 1, .a = 5, .b = 7, .want = 3 }, // (5 + 1) >> 1
        .{ .code = 0x2D, .op = 2, .n = 0, .a = 5, .b = 7, .want = 5 },
        .{ .code = 0x2E, .op = 2, .n = 2, .a = N4, .b = 9, .want = 0xFFFFFF },
        .{ .code = 0x2E, .op = 2, .n = 2, .a = N4, .b = M2, .want = 0xFFFFF0 },
        .{ .code = 0x2E, .op = 6, .n = 1, .a = N4, .b = M2, .want = 4 }, // max(-4, #1) << 2, not max(-4, -2) << 2
        .{ .code = 0x2F, .op = 6, .n = 0, .a = N4, .b = 9, .want = 0x7FFF },
        .{ .code = 0x2F, .op = 2, .n = 2, .a = 5, .b = M2, .want = 8 },
        // 0x3B-0x3E
        .{ .code = 0x3B, .op = 4, .n = 0, .a = 5, .b = 3, .want = 3 },
        .{ .code = 0x3B, .op = 2, .n = 0, .a = 5, .b = 3, .want = 0 },
        .{ .code = 0x3C, .op = 3, .n = 1, .a = 5, .b = 3, .want = 6 },
        .{ .code = 0x3C, .op = 3, .n = 2, .a = 5, .b = 3, .want = 3 },
        .{ .code = 0x3D, .op = 4, .n = 0, .a = 5, .b = 3, .want = 4 },
        .{ .code = 0x3F, .op = 6, .n = 1, .a = 5, .b = 3, .want = 0xFFFFFB },
        .{ .code = 0x3E, .op = 4, .n = 2, .a = N4, .b = 3, .want = 0xFFFFFF },
    }) |c| {
        const got = x18c3Unary(c.code, c.op, c.n, c.a, c.b) catch |e| {
            std.debug.print("code {x} op {d} n {d}: {}\n", .{ c.code, c.op, c.n, e });
            return e;
        };
        if (got != c.want) std.debug.print("code {x} op {d} n {d}: got {x}\n", .{ c.code, c.op, c.n, got });
        try testing.expectEqual(c.want, got);
    }
    const R = struct { code: u32, op: u32, n: u32, a: u32 = 5, b: u32 = 3 };
    for ([_]R{
        .{ .code = 0x3E, .op = 2, .n = 0 },          .{ .code = 0x3C, .op = 3, .n = 3 },
        .{ .code = 0x3D, .op = 3, .n = 2 },          .{ .code = 0x3F, .op = 3, .n = 2 },
        .{ .code = 0x26, .op = 7, .n = 0 },          .{ .code = 0x26, .op = 2, .n = 3 },
        .{ .code = 0x08, .op = 6, .n = 0 },          .{ .code = 0x21, .op = 4, .n = 0 },
        .{ .code = 0x06, .op = 2, .n = 0, .b = 13 }, .{ .code = 0x06, .op = 2, .n = 0, .b = 0xFFFFF3 },
    }) |r| try testing.expectError(error.UnknownForm, x18c3Unary(r.code, r.op, r.n, r.a, r.b));
}

test "x18 class 011 codes 0x36-0x39 and the head's Y load (645)" {
    // own-x18rot: 71c00018 00031970 30000014 74000149, s5 = -0x123, s7 = 4: rot24 left by 4.
    var st = try x18c3Run(0x71c00018, 0x00031970, 0x30000014, 0xFFFEDD);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u32, 0xFFFEDD), st.s[5]);
    st.s[7] = 4;
    try testing.expectEqual(@as(u32, 0xFFEDDF), try x18c3Unary(0x38, 6, 0, 0xFFFEDD, 4));
    const N: u32 = 0xFFFEDD; // -0x123
    const C = struct { code: u32, op: u32, n: u32, a: u32, b: u32, want: u32 };
    for ([_]C{
        .{ .code = 0x36, .op = 2, .n = 0, .a = 0x123, .b = 4, .want = 0x12 },
        .{ .code = 0x37, .op = 2, .n = 0, .a = 0x123, .b = 4, .want = 0x12 }, // (0x123 + 8) >> 4
        .{ .code = 0x37, .op = 2, .n = 0, .a = 0x128, .b = 4, .want = 0x13 },
        .{ .code = 0x36, .op = 2, .n = 0, .a = 0x128, .b = 4, .want = 0x12 },
        .{ .code = 0x38, .op = 2, .n = 0, .a = N, .b = 8, .want = 0xFFFE },
        .{ .code = 0x36, .op = 2, .n = 1, .a = N, .b = 8, .want = 0xFFFFFE },
        .{ .code = 0x36, .op = 2, .n = 0, .a = 0x123, .b = 0xFFFFFF, .want = 0x246 }, // b < 0 shifts left
        .{ .code = 0x36, .op = 3, .n = 2, .a = 0x123, .b = 4, .want = 0x1230 },
        .{ .code = 0x37, .op = 3, .n = 2, .a = 0x128, .b = 0xFFFFFC, .want = 0x13 }, // rounds the right shift
        .{ .code = 0x36, .op = 3, .n = 2, .a = 0x128, .b = 0xFFFFFC, .want = 0x12 },
        .{ .code = 0x36, .op = 3, .n = 1, .a = 0x123, .b = 4, .want = 0x123 },
        .{ .code = 0x38, .op = 4, .n = 2, .a = N, .b = 8, .want = 0xFFFE },
        .{ .code = 0x37, .op = 4, .n = 0, .a = 0x128, .b = 4, .want = 0x128 },
        .{ .code = 0x36, .op = 6, .n = 0, .a = N, .b = 4, .want = 0xFFEDDF }, // (a | 1) rotated: 0xFFFEDD already odd
        .{ .code = 0x37, .op = 6, .n = 0, .a = 0xFFFFFC, .b = 4, .want = 0xFFFFDF }, // -4 | 1 = -3, rotated
        .{ .code = 0x38, .op = 6, .n = 0, .a = 0xFFFFFC, .b = 4, .want = 0xFFFFCF }, // -4 rotated plain
        .{ .code = 0x38, .op = 6, .n = 0, .a = 0x123, .b = 0xFFFFF8, .want = 0x230001 }, // right by 8
        .{ .code = 0x39, .op = 2, .n = 0, .a = 0x123, .b = 4, .want = 0x1230 },
        .{ .code = 0x39, .op = 6, .n = 1, .a = 0x123, .b = 0xFFFFFC, .want = 0 },
        .{ .code = 0x39, .op = 3, .n = 2, .a = N, .b = 4, .want = 0xFFFFED },
        .{ .code = 0x39, .op = 3, .n = 2, .a = N, .b = 0xFFFFFC, .want = 0xFFFFFF },
        .{ .code = 0x39, .op = 3, .n = 2, .a = 0x123, .b = 0xFFFFFC, .want = 0 },
        .{ .code = 0x39, .op = 2, .n = 2, .a = N, .b = 4, .want = 0x0FFFED },
        .{ .code = 0x39, .op = 4, .n = 2, .a = N, .b = 0xFFFFFC, .want = 0 },
    }) |c| {
        const got = x18c3Unary(c.code, c.op, c.n, c.a, c.b) catch |e| {
            std.debug.print("code {x} op {d} n {d}: {}\n", .{ c.code, c.op, c.n, e });
            return e;
        };
        if (got != c.want) std.debug.print("code {x} op {d} n {d}: got {x}\n", .{ c.code, c.op, c.n, got });
        try testing.expectEqual(c.want, got);
    }
    for ([_]C{
        .{ .code = 0x36, .op = 2, .n = 0, .a = 1, .b = 16, .want = 0 },
        .{ .code = 0x38, .op = 6, .n = 0, .a = 1, .b = 13, .want = 0 },
        .{ .code = 0x39, .op = 2, .n = 0, .a = 1, .b = 0xFFFFF3, .want = 0 },
        .{ .code = 0x37, .op = 7, .n = 0, .a = 1, .b = 1, .want = 0 },
        .{ .code = 0x39, .op = 2, .n = 3, .a = 1, .b = 1, .want = 0 },
    }) |c| try testing.expectError(error.UnknownForm, x18c3Unary(c.code, c.op, c.n, c.a, c.b));
    // the head loads s14 <- Y:[e10]; a word with bit 15 set is refused (sign unmeasured)
    var ld = emu.State{};
    defer ld.deinit(testing.allocator);
    ld.e[10] = 0x700;
    try ld.y_mem.put(testing.allocator, 0x700, 0x1234);
    ld.p[3] = 1; // own-x18b's stale p3 (752)
    ld.s[5] = 5;
    ld.s[7] = 3;
    _ = try emu.runFrom(testing.allocator, &ld, &.{ 0x21c00018, 0x00001970, 0x30000014, 0x74000149, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1234), ld.s[14]);
    try testing.expectEqual(@as(u32, 0xFFFFFE), ld.s[9]);
    try ld.y_mem.put(testing.allocator, 0x700, 0x8000);
    try testing.expectError(error.UnknownForm, emu.runFrom(testing.allocator, &ld, &.{ 0x21c00018, 0x00001970, 0x30000014, 0x74000149, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "x19alu writes its ALU result per cond w1[23:20] and p1 (646)" {
    // The probe bundle: x19alu s4 <- s4 << s2 (000006f9 0402_5808 00108101 81e18008) with
    // s4 = 0x12, s2 = 8.  Cond 0 writes only while p1 = 0.
    const Case = struct { cond: u32, p1: u8, want: u32 };
    for ([_]Case{
        .{ .cond = 0, .p1 = 0, .want = 0x1200 }, .{ .cond = 0, .p1 = 1, .want = 0x12 },
        .{ .cond = 1, .p1 = 1, .want = 0x12 },   .{ .cond = 3, .p1 = 0, .want = 0x1200 },
        .{ .cond = 2, .p1 = 0, .want = 0x12 },   .{ .cond = 2, .p1 = 1, .want = 0x1200 },
        .{ .cond = 5, .p1 = 1, .want = 0x1200 }, .{ .cond = 7, .p1 = 0, .want = 0x12 },
        .{ .cond = 6, .p1 = 0, .want = 0x1200 }, .{ .cond = 6, .p1 = 1, .want = 0x1200 },
        .{ .cond = 9, .p1 = 1, .want = 0x1200 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[4] = 0x12;
        st.s[2] = 8;
        st.p[1] = c.p1;
        st.p[3] = 1; // other p never matter
        _ = try emu.runFrom(testing.allocator, &st, &.{ 0x000006f9, 0x04025808 | (c.cond << 20), 0x00108101, 0x81e18008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, st.s[4]);
    }
    for ([_]u32{ 4, 8, 10, 15 }) |cond| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        try testing.expectError(error.UnknownForm, emu.runFrom(testing.allocator, &st, &.{ 0x000006f9, 0x04025808 | (cond << 20), 0x00108101, 0x81e18008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    }
    // The assembler emits the always-write cond 6.
    const w = try asm_mod.assembleOne("x19.alu s4, s4, s2, #23");
    try testing.expectEqual(@as(u32, 6), (w.slice()[1] >> 20) & 0xF);
}

test "x19.xrd reads A<{w1[2:0], head[31:30]}> (651)" {
    // the old template word, its A write (head[29:25] = 4) cleared, reads A11
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    for (1..31) |k| st.a[k] = 0x40 + @as(u32, @intCast(k));
    for (1..31) |k| try st.x_mem.put(testing.allocator, 0x40 + @as(u32, @intCast(k)), 0x100 + @as(u32, @intCast(k)));
    _ = try emu.runFrom(testing.allocator, &st, &.{ 0xc0e00019, 0x08028002, 0x80003000, 0x61510554, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
    try testing.expectEqual(@as(u32, 0x10B), st.s[7]);
    // the assembler splits k across w1[2:0] and head[31:30]
    for ([_]u32{ 1, 2, 8, 11, 19, 30 }) |k| {
        var buf: [32]u8 = undefined;
        const w = try asm_mod.assembleOne(try std.fmt.bufPrint(&buf, "x19.xrd s7, #{d}", .{k}));
        try testing.expectEqual(k, ((w.slice()[1] & 7) << 2) | (w.slice()[0] >> 30));
    }
    try testing.expectError(error.AssemblyFailed, asm_mod.assembleOne("x19.xrd s7, #0"));
    try testing.expectError(error.AssemblyFailed, asm_mod.assembleOne("x19.xrd s7, #31"));
    // the assembled word writes no A (654); one that does is refused
    const w = try asm_mod.assembleOne("x19.xrd s7, #11");
    try testing.expectEqual(@as(u32, 0), (w.slice()[0] >> 25) & 0x1F);
    var st2 = emu.State{};
    defer st2.deinit(testing.allocator);
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st2, &.{ 0xc8e00019, 0x08028002, 0x80003000, 0x61510554, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8));
}

test "an x02.v X read into s0 reaches no other slot (652)" {
    for ([_]u32{ 0, 8 }) |d| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.a[1] = 0;
        try st.x_mem.put(testing.allocator, 0, 0x123456);
        st.s[3] = 0x40;
        st.s[7] = 0x100;
        st.s[4] = 0x77;
        st.s[8] = 0x55;
        var buf: [96]u8 = undefined;
        const src = try std.fmt.bufPrint(&buf, "x02.v s{d}, A1, A0, A0, A0, s4, s0, s0, s0, #5, #0, #2, s3, s7, #5, #0", .{d});
        const w = try asm_mod.assembleOne(src);
        var prog: [9]u32 = undefined;
        @memcpy(prog[0..6], w.slice()[0..6]);
        prog[6..9].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(u32, 0), st.s[0]);
        try testing.expectEqual(@as(u32, 0), st.s[4]); // the multiply saw s0 = 0
        try testing.expectEqual(@as(u32, if (d == 8) 0x123456 else 0x55), st.s[8]);
    }
}

test "stx refuses A0 and A31 (649)" {
    for ([_]u32{ 0, 31, 7 }) |k| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.a[k] = 0x123;
        st.s[3] = 0x55;
        const w: u32 = 0x4030000A | (k << 11) | (3 << 16);
        const r = emu.runFrom(testing.allocator, &st, &.{ w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 4);
        if (k == 7) {
            _ = try r;
            try testing.expectEqual(@as(?u32, 0x55), st.x_mem.get(0x123));
        } else try testing.expectError(emu.RunError.UnknownForm, r);
    }
}

test "x02's low products hold P and 2P mod 2^24 (655, 657); x18.and masks them (644, 345)" {
    for ([_]struct { c: u32, d: u32, want: u32 }{
        .{ .c = 0x2AC, .d = 0xFFFF80, .want = 0xFEAA00 },
        .{ .c = 0x2AC, .d = 0x80, .want = 0x015600 },
        .{ .c = 0x7FFF, .d = 0xFFEDCC, .want = 0xE61234 },
    }) |t| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = t.c;
        st.s[2] = t.d;
        st.s[11] = 0xFFFF;
        const w = try asm_mod.assembleOne("x02.v s0, A1, A0, A0, A0, s0, s1, s2, s12, #5, #1, #0, s0, s0, #0, #0");
        const m = try asm_mod.assembleOne("x18.and s13, s12, s11");
        var prog: [13]u32 = undefined;
        @memcpy(prog[0..6], w.slice()[0..6]);
        @memcpy(prog[6..10], m.slice()[0..4]);
        prog[10..13].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(t.want, st.s[12]);
        try testing.expectEqual(t.want & 0xFFFF, st.s[13]);
        try testing.expectEqual(@as(u32, 0), st.a[2]); // the template writes no A (345)
    }
    // w1[17] = 0: (2P) mod 2^24, also unmasked (657)
    {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x2AC;
        st.s[2] = 0x80;
        const w = try asm_mod.assembleOne("x02.x3 #0, #0, s2, s3, #5, #2, s2, #1, s6");
        var prog: [9]u32 = undefined;
        @memcpy(prog[0..6], w.slice()[0..6]);
        prog[6..9].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(u32, 0x02AC00), st.s[6]);
    }
    // w1[3:0] = 1 also writes A2 <- the result (345)
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.a[2] = 0x42;
    st.s[12] = 0x1255;
    st.s[11] = 0xFF;
    _ = try emu.runFrom(testing.allocator, &st, &.{ 0x3a000018, 0x00018cb1, 0x00004030, 0x61d53805, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
    try testing.expectEqual(@as(u32, 0x55), st.a[2]);
    try testing.expectEqual(@as(u32, 0x55), st.s[12]);
}

test "x03's mode-5 A copy counts e<2*w4[2:0] + w3[31]> down; the template does not (349)" {
    const t = try asm_mod.assembleOne("x03.agu A6, A14, #0, #0, A0");
    for ([_]u32{ 4, 0 }) |lo| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[8] = 0x710;
        st.a[14] = 0x123;
        var prog: [9]u32 = undefined;
        @memcpy(prog[0..6], t.slice()[0..6]);
        prog[4] = (prog[4] & ~@as(u32, 7)) | lo;
        prog[6..9].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(u32, if (lo == 4) 0x70F else 0x710), st.e[8]);
        try testing.expectEqual(@as(u32, 0x123), st.a[6]);
    }
    try testing.expectEqual(@as(u32, 0), t.slice()[4] & 7);
}

test "x17.sty writes e<2*w1[2:0] + h[31]> <- e<k> + 1; the template does not; ldy reads e2 (658)" {
    const t = try asm_mod.assembleOne("x17.sty e3, #3, #0");
    try testing.expectEqual(@as(u32, 0), t.slice()[1] & 7);
    try testing.expectEqual(@as(u32, 0), t.slice()[0] >> 31);
    for ([_]struct { lo: u32, dest: usize }{ .{ .lo = 0, .dest = 0 }, .{ .lo = 1, .dest = 2 }, .{ .lo = 2, .dest = 4 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[3] = 0x2E10;
        st.e[2] = 0x55;
        st.e[4] = 0x66;
        st.s[3] = 0x1234;
        var prog: [6]u32 = undefined;
        @memcpy(prog[0..3], t.slice()[0..3]);
        prog[1] |= c.lo;
        prog[3..6].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(?u32, 0x1234), st.y_mem.get(0x2E10));
        try testing.expectEqual(@as(u32, if (c.dest == 2) 0x2E11 else 0x55), st.e[2]);
        try testing.expectEqual(@as(u32, if (c.dest == 4) 0x2E11 else 0x66), st.e[4]);
    }
    // through e2 with dest e2: two stores land at e2 and e2 + 1, and ldy reads e2
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.e[2] = 0x2E20;
    st.s[3] = 0x77;
    const w = try asm_mod.assembleOne("x17.sty e2, #3, #0");
    var one: [3]u32 = undefined;
    @memcpy(&one, w.slice()[0..3]);
    one[1] |= 1;
    const l = try asm_mod.assembleOne("ldy s9, e2");
    _ = try emu.runFrom(testing.allocator, &st, &.{ one[0], one[1], one[2], one[0], one[1], one[2], l.slice()[0], 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(?u32, 0x77), st.y_mem.get(0x2E20));
    try testing.expectEqual(@as(?u32, 0x77), st.y_mem.get(0x2E21));
    try testing.expectEqual(@as(u32, 0x2E22), st.e[2]);
}

test "x17.sty keeps S1[27]; S1[28:27] = 00 is a stop form keyed by S1[2:0] (648, 667)" {
    const t = try asm_mod.assembleOne("x17.sty e3, #3, #0");
    try testing.expectEqual(@as(u32, 1), (t.slice()[1] >> 27) & 3);
    const ti = try asm_mod.assembleOne("x17.styi e3, #3, #0, e3");
    try testing.expectEqual(@as(u32, 1), (ti.slice()[1] >> 27) & 3);
    for ([_]u8{ 0, 1 }) |p1| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[3] = 0x2E40;
        st.s[3] = 0x99;
        st.p[1] = p1;
        const prog = [_]u32{ t.slice()[0], (t.slice()[1] & ~@as(u32, 1 << 27)) | 1, t.slice()[2], 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        // The store lands either way; with p1 the run then stops without a halt (667).
        try testing.expectEqual(@as(?u32, 0x99), st.y_mem.get(0x2E40));
        try testing.expectEqual(p1 == 1, st.silent_stop);
        try testing.expectEqual(p1 == 0, st.halted);
    }
    // k = 0, 2, 4..7 stop whatever p is (667); k = 3 runs away: refused.
    for ([_]u32{ 0, 2, 3, 7 }) |k| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        const prog = [_]u32{ t.slice()[0], (t.slice()[1] & ~@as(u32, 1 << 27)) | k, t.slice()[2], 0x00000017, 0x31308000, 0xA0000000 };
        const r = emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        if (k == 3) try testing.expectError(emu.RunError.UnknownForm, r) else {
            _ = try r;
            try testing.expect(st.silent_stop);
        }
    }
}

test "x18.macf chains (hi, lo) exactly; x18.mac rounds hi only (665, 668)" {
    const gpa = testing.allocator;
    // 3 and 5 taps of 2 * 0x300 * 0x1000 = 0x600000 from X = 0
    for ([_]struct { taps: u32, hi: u32, lo: u32 }{ .{ .taps = 3, .hi = 1, .lo = 0x200000 }, .{ .taps = 5, .hi = 1, .lo = 0xE00000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(gpa);
        st.s[1] = 0x300;
        st.s[2] = 0x1000;
        const m = try asm_mod.assembleOne("x18.macf s4, #5, s1, s2, s4, s6, s6");
        var prog: [23]u32 = undefined;
        var n: usize = 0;
        for (0..c.taps) |_| {
            @memcpy(prog[n .. n + 4], m.slice()[0..4]);
            n += 4;
        }
        prog[n..][0..3].* = .{ 0x00000017, 0x31308000, 0xA0000000 };
        _ = try emu.runFrom(gpa, &st, prog[0 .. n + 3], 0, 16);
        try testing.expectEqual(c.hi, st.s[4]);
        try testing.expectEqual(c.lo, st.s[6]);
    }
    // X = {0, 0x900000}, P = 0: lo is unsigned, hi 0 (668)
    var st = emu.State{};
    defer st.deinit(gpa);
    st.s[6] = 0x900000;
    const m = try asm_mod.assembleOne("x18.macf s4, #5, s1, s2, s4, s6, s6");
    _ = try emu.runFrom(gpa, &st, &.{ m.slice()[0], m.slice()[1], m.slice()[2], m.slice()[3], 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
    try testing.expectEqual(@as(u32, 0), st.s[4]);
    try testing.expectEqual(@as(u32, 0x900000), st.s[6]);
    // the rounded form's word is 665's own word
    const r = try asm_mod.assembleOne("x18.mac s4, #5, s1, s2, s4, s6, s6");
    try testing.expectEqualSlices(u32, &.{ 0x1C000018, 0x42070007, 0x21326006, 0xE60701D4 }, r.slice()[0..4]);
}

test "x17.styi steps its pointer in the store bundle (658)" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        x17.styi e3, #3, #0, e3
        \\        x17.styi e3, #5, #0, e3
        \\        x17.styi e3, #3, #0, e12
        \\        x17.halt
    );
    defer gpa.free(prog);
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[3] = 0x2E30;
    st.s[3] = 0x11;
    st.s[5] = 0x22;
    _ = try emu.runFrom(gpa, &st, prog, 0, 16);
    try testing.expectEqual(@as(?u32, 0x11), st.y_mem.get(0x2E30));
    try testing.expectEqual(@as(?u32, 0x22), st.y_mem.get(0x2E31));
    try testing.expectEqual(@as(?u32, 0x11), st.y_mem.get(0x2E32));
    try testing.expectEqual(@as(u32, 0x2E32), st.e[3]);
    try testing.expectEqual(@as(u32, 0x2E33), st.e[12]);
}

test "x17.stys steps its pointer by R<h[30:27]> after the store (fact 773)" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\        x17.stys e3, #3, #0, e3, R1
        \\        x17.stys e3, #5, #0, e3, R2
        \\        x17.stys e3, #3, #0, e2, R4
        \\        x17.stys e5, #3, #0, e5, R5
        \\        x17.halt
    );
    defer gpa.free(prog);
    var st = emu.State{};
    defer st.deinit(gpa);
    st.e[3] = 0x2E10;
    st.e[5] = 0xFFFE;
    st.r[1] = 1;
    st.r[2] = 2;
    st.r[4] = 0xFFFF;
    st.r[5] = 3;
    st.s[3] = 0x11;
    st.s[5] = 0x22;
    _ = try emu.runFrom(gpa, &st, prog, 0, 16);
    try testing.expectEqual(@as(?u32, 0x11), st.y_mem.get(0x2E10));
    try testing.expectEqual(@as(?u32, 0x22), st.y_mem.get(0x2E11));
    try testing.expectEqual(@as(?u32, 0x11), st.y_mem.get(0x2E13));
    try testing.expectEqual(@as(u32, 0x2E13), st.e[3]);
    try testing.expectEqual(@as(u32, 0x2E12), st.e[2]); // -1 via R4 = 0xFFFF
    try testing.expectEqual(@as(?u32, 0x11), st.y_mem.get(0xFFFE));
    try testing.expectEqual(@as(u32, 0x0001), st.e[5]); // 16-bit wrap
}

test "x19alu sets p<w2[17:14]> under w3[23:20] = 0xE (647)" {
    for ([_]struct { pk: u32, s4: u32 }{ .{ .pk = 2, .s4 = 0x12 }, .{ .pk = 3, .s4 = 0 }, .{ .pk = 15, .s4 = 0x12 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[4] = c.s4;
        st.s[2] = 8;
        const w2 = (0x00108101 & ~@as(u32, 0xF << 14)) | (c.pk << 14);
        _ = try emu.runFrom(testing.allocator, &st, &.{ 0x000006f9, 0x04625808, w2, 0x81e18008, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u8, @intFromBool(c.pk == 2)), st.p[2]);
        try testing.expectEqual(@as(u8, @intFromBool(c.pk == 3)), st.p[3]);
    }
    // the assembler's x19.alu touches no predicate
    const w = try asm_mod.assembleOne("x19.alu s4, s4, s2, #23");
    try testing.expectEqual(@as(u32, 15), (w.slice()[2] >> 14) & 0xF);
}

test "x01 norm: the clean word leaves R5 and e alone; e<w4[12:9]> is refused (669)" {
    const clean = [_]u32{ 0x28001381, 0x00000580, 0x280000e0, 0x000280a0, 0x00040002, 0x28200900, 0x00300090 };
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]struct { b: u32, n: u32 }{ .{ .b = 0x100, .n = 14 }, .{ .b = 0xFFFF00, .n = 15 }, .{ .b = 0x7FFF, .n = 8 }, .{ .b = 0, .n = 24 }, .{ .b = 0xFFFFFF, .n = 23 }, .{ .b = 0xFFFFFE, .n = 22 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[8] = c.b;
        st.r[5] = 0x40;
        _ = try emu.runFrom(testing.allocator, &st, &(clean ++ halt), 0, 8);
        try testing.expectEqual(c.n, st.s[9]);
        try testing.expectEqual(@as(u32, 0x40), st.r[5]);
    }
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    var w = clean;
    w[4] |= 6 << 9;
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st, &(w ++ halt), 0, 8));
}

test "x03 mode 5 chains {hi:lo} exactly, and loads the next operand behind the multiply (670, 671)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const mac = [_]u32{ 0x80000003, 0x40006800, 0x01441485, 0x00000007, 0x00002c80, 0x00041202 };
    for ([_]struct { n: usize, hi: u32, lo: u32 }{ .{ .n = 3, .hi = 1, .lo = 0x200000 }, .{ .n = 5, .hi = 1, .lo = 0xE00000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[2] = 0x300;
        st.s[4] = 0x1000;
        var prog: [33]u32 = undefined;
        for (0..c.n) |k| @memcpy(prog[6 * k .. 6 * k + 6], &mac);
        @memcpy(prog[6 * c.n .. 6 * c.n + 3], &halt);
        _ = try emu.runFrom(testing.allocator, &st, prog[0 .. 6 * c.n + 3], 0, 16);
        try testing.expectEqual(c.hi, st.s[10]);
        try testing.expectEqual(c.lo, st.s[9]);
    }
    {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[9] = 0x900000;
        _ = try emu.runFrom(testing.allocator, &st, &(mac ++ halt), 0, 16);
        try testing.expectEqual(@as(u32, 0), st.s[10]);
    }
    // w4[4] = 1: s2 <- X:[A2] after the multiply has read the old s2 (671)
    const ld = [_]u32{ 0x80000003, 0x40006800, 0x01441485, 0x00000007, 0x00002c90, 0x00041202 };
    for ([_]struct { n: usize, lo: u32 }{ .{ .n = 1, .lo = 0x600000 }, .{ .n = 2, .lo = 0x800000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[2] = 0x300;
        st.s[4] = 0x1000;
        st.a[2] = 0x40;
        st.a[7] = 0x40;
        try st.x_mem.put(testing.allocator, 0x40, 0x100);
        var prog: [15]u32 = undefined;
        for (0..c.n) |k| @memcpy(prog[6 * k .. 6 * k + 6], &ld);
        @memcpy(prog[6 * c.n .. 6 * c.n + 3], &halt);
        _ = try emu.runFrom(testing.allocator, &st, prog[0 .. 6 * c.n + 3], 0, 16);
        try testing.expectEqual(c.lo, st.s[9]);
        try testing.expectEqual(@as(u32, 0x100), st.s[2]);
    }
}

test "x03 one-bundle tap: X read, A post-step and running MAC (672)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const w = [_]u32{ 0x80000003, 0x40006800, 0x00041485, 0x00000002, 0x00002c90, 0x00041202 };
    // the third bundle's read (X:0x102) is the dummy load; s2 is not checked there
    for ([_]struct { n: usize, s2: ?u32, lo: u32 }{ .{ .n = 1, .s2 = 0x100, .lo = 0x600000 }, .{ .n = 2, .s2 = 0x80, .lo = 0x800000 }, .{ .n = 3, .s2 = null, .lo = 0x900000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[2] = 0x300;
        st.s[4] = 0x1000;
        st.a[2] = 0x100;
        try st.x_mem.put(testing.allocator, 0x100, 0x100);
        try st.x_mem.put(testing.allocator, 0x101, 0x80);
        var prog: [21]u32 = undefined;
        for (0..c.n) |k| @memcpy(prog[6 * k .. 6 * k + 6], &w);
        @memcpy(prog[6 * c.n .. 6 * c.n + 3], &halt);
        _ = try emu.runFrom(testing.allocator, &st, prog[0 .. 6 * c.n + 3], 0, 16);
        if (c.s2) |v| try testing.expectEqual(v, st.s[2]);
        try testing.expectEqual(c.lo, st.s[9]);
    }
}

test "x03 MAC row refuses the repeat class w2[31:29] = 011 (673)" {
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    const w = [_]u32{ 0x80000003, 0x40006800, 0x60041485, 0x00000002, 0x00002c90, 0x00041202, 0x00000017, 0x31308000, 0xA0000000 };
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st, &w, 0, 8));
}

test "x03 counted loop: jump to itself on p3, both slots every pass, p3 one pass late (675, 679, 680)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const w = [_]u32{ 0x00000003, 0x60000002, 0x18106000, 0x85800008, 0x00004005, 0x001CC000 };
    const add = [_]u32{0x5802a9c5}; // alu s5, s5, s6, #OP_ADD
    for ([_]struct { e: u32, p3: u32, passes: u32 }{ .{ .e = 5, .p3 = 1, .passes = 6 }, .{ .e = 1, .p3 = 1, .passes = 2 }, .{ .e = 0, .p3 = 1, .passes = 2 }, .{ .e = 15, .p3 = 1, .passes = 16 }, .{ .e = 5, .p3 = 0, .passes = 1 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[11] = c.e;
        st.p[3] = @intCast(c.p3);
        st.a[8] = 0x200;
        st.s[3] = 0x55;
        st.s[6] = 1;
        _ = try emu.runFrom(testing.allocator, &st, &(w ++ add ++ add ++ halt), 0, 200);
        try testing.expectEqual(c.passes, st.x_mem.count());
        try testing.expectEqual(@as(?u32, 0x55), st.x_mem.get(0x200 + c.passes - 1));
        try testing.expectEqual(0x200 + c.passes, st.a[8]);
        // both slots ran on every pass, the last included (680)
        try testing.expectEqual(2 * c.passes, st.s[5]);
        if (c.p3 == 1) {
            try testing.expectEqual(@as(u8, 0), st.p[3]);
            try testing.expectEqual(@as(u32, (c.e -% (c.passes)) & 0xFFFF), st.e[11]);
        }
    }
    // w2[27] = 0: 394's sense, jump while clear: p3 = 0 at entry gives 2 passes, p3 = 1 one
    for ([_][2]u32{ .{ 0, 2 }, .{ 1, 1 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[11] = 5;
        st.p[3] = @intCast(c[0]);
        st.a[8] = 0x200;
        st.s[3] = 0x55;
        const b = [_]u32{ w[0], w[1], w[2] & ~@as(u32, 1 << 27), w[3], w[4], w[5] } ++ add ++ add ++ halt;
        _ = try emu.runFrom(testing.allocator, &st, &b, 0, 200);
        try testing.expectEqual(c[1], st.x_mem.count());
    }
    // w2[28] = 0 with w2[27] = 1 is unconditional (329): to itself it runs away, refused.
    for ([_][2]u32{.{ 0x08106000, 0x001CC000 }}) |v| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.p[3] = 1;
        st.p[4] = 1;
        const bad = [_]u32{ w[0], w[1], v[0], w[3], w[4], v[1] } ++ add ++ add ++ halt;
        if (emu.runFrom(testing.allocator, &st, &bad, 0, 200)) |_| return error.TestUnexpectedResult else |_| {}
    }
}

test "x19k Y store: Y:[e<w3[28:25]>] <- s<w2[3:0]> on w2[26] and w2[16]; w1[18] = 0 refused (677)" {
    const k = try asm_mod.assembleOne("x19.k00 s9, e3, R6, #1, #6, #0");
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]u32{ 1, 0 }) |b18| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[3] = 0x100;
        st.r[6] = 0x23;
        st.s[5] = 0x4567;
        const w1 = (k.slice()[1] & ~@as(u32, 1 << 18)) | (b18 << 18);
        const w2 = (k.slice()[2] & ~@as(u32, 0xF)) | (1 << 26) | (1 << 16) | 5;
        const prog = [_]u32{ k.slice()[0], w1, w2, k.slice()[3] } ++ halt;
        const r = emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        if (b18 == 1) {
            _ = try r;
            try testing.expectEqual(@as(?u32, 0x4567), st.y_mem.get(0x100));
            try testing.expectEqual(@as(u32, 0x123), st.s[9]);
        } else try testing.expectError(emu.RunError.UnknownForm, r);
    }
}

test "a halt in a taken jump's delay slot runs the target once, then stops (681)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const nop = [_]u32{0x40000005};
    const w = [_]u32{ 0x00000003, 0x60000002, 0x18106000, 0x85800008, 0x00004005, 0x001CC000 };
    for ([_]u32{ 3, 5 }) |e| {
        for ([_]bool{ false, true }) |slot2| {
            var st = emu.State{};
            defer st.deinit(testing.allocator);
            st.e[11] = e;
            st.p[3] = 1;
            st.a[8] = 0x300;
            st.s[3] = 0x77;
            const r = if (slot2) emu.runFrom(testing.allocator, &st, &(w ++ nop ++ halt), 0, 50) else emu.runFrom(testing.allocator, &st, &(w ++ halt), 0, 50);
            _ = try r;
            try testing.expectEqual(@as(usize, 2), st.x_mem.count());
            try testing.expectEqual(@as(?u32, 0x77), st.x_mem.get(0x301));
            try testing.expect(st.halted);
        }
    }
}

test "x03 jumps: p0 reads 1; the tap row jumps on p3 with a nonzero offset, refuses jr +0 (682)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const nop: u32 = 0x40000005;
    const add: u32 = 0x5802a9c5; // alu s5, s5, s6, #OP_ADD: runs only on the fall-through
    const tap = [_]u32{ 0x80000003, 0x40006800 | (9 << 18), 0x00041485 | (3 << 27), 0x00000002, 0x00002c90, 0x00041202 | (3 << 14) };
    const st0 = [_]u32{ 0x00000003, 0x60000002 | (9 << 18), 0x18106000, 0x85800008, 0x00004005, 0x001C0000 };
    for ([_]struct { w: [6]u32, p: u32, k: usize, jumps: bool }{
        .{ .w = tap, .p = 1, .k = 3, .jumps = true },
        .{ .w = tap, .p = 0, .k = 3, .jumps = false },
        .{ .w = st0, .p = 0, .k = 0, .jumps = true },
        // w2[28:27] = 10 on the tap row: jump if clear; 01: unconditional
        .{ .w = .{ tap[0], tap[1], tap[2] & ~@as(u32, 1 << 27), tap[3], tap[4], tap[5] }, .p = 0, .k = 3, .jumps = true },
        .{ .w = .{ tap[0], tap[1], tap[2] & ~@as(u32, 1 << 27), tap[3], tap[4], tap[5] }, .p = 1, .k = 3, .jumps = false },
        .{ .w = .{ tap[0], tap[1], tap[2] & ~@as(u32, 1 << 28), tap[3], tap[4], tap[5] }, .p = 1, .k = 3, .jumps = true },
        .{ .w = .{ tap[0], tap[1], tap[2] & ~@as(u32, 1 << 28), tap[3], tap[4], tap[5] }, .p = 0, .k = 3, .jumps = true },
        .{ .w = .{ st0[0], st0[1], st0[2] & ~@as(u32, 1 << 28), st0[3], st0[4], st0[5] | (3 << 14) }, .p = 0, .k = 3, .jumps = true },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.p[c.k] = @intCast(c.p);
        st.s[6] = 1;
        st.e[11] = 5;
        st.a[2] = 0x100;
        st.a[8] = 0x300;
        _ = try emu.runFrom(testing.allocator, &st, &(c.w ++ [_]u32{ nop, nop, add } ++ halt), 0, 50);
        try testing.expectEqual(@as(u32, if (c.jumps) 0 else 1), st.s[5]);
    }
    // the tap row's jump to itself never ends: refused
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.p[3] = 1;
    var w = tap;
    w[1] &= ~@as(u32, 0x7FF << 18);
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st, &(w ++ [_]u32{ nop, nop } ++ halt), 0, 50));
}

test "x18 class 011: w3[3] loads #head[20:5] into R; the load mux's 000 gives 0 (683, 688)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]struct { w1: u32, w3: u32, s2: u32, r1: u32 }{
        .{ .w1 = 0x00001970, .w3 = 0x740000c9, .s2 = 0x1234, .r1 = 0 },
        .{ .w1 = 0x00001970, .w3 = 0x740000c1, .s2 = 0x1234, .r1 = 0xFFFE },
        .{ .w1 = 0x00000970, .w3 = 0x740000c9, .s2 = 0, .r1 = 0 },
        // 010 and 101 read Y too (369)
        .{ .w1 = 0x00002970, .w3 = 0x740000c9, .s2 = 0x1234, .r1 = 0 },
        .{ .w1 = 0x00005970, .w3 = 0x740000c9, .s2 = 0x1234, .r1 = 0 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.p[3] = 1; // the silicon rows' stale p3 (752)
        st.s[4] = 5;
        st.s[7] = 3;
        st.s[2] = 0x99;
        st.r[1] = 0x77;
        st.e[10] = 0x50;
        try st.y_mem.put(testing.allocator, 0x50, 0x1234);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x20400018, c.w1, 0x30000010, c.w3 } ++ halt), 0, 8);
        try testing.expectEqual(c.s2, st.s[2]);
        try testing.expectEqual(c.r1, st.r[1]);
    } // a nonzero immediate lands in R (688): 0x1234 into R1
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.s[4] = 5;
    st.s[7] = 3;
    _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x20400018 | (0x1234 << 5), 0x00000970, 0x30000010, 0x740000c9 } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x1234), st.r[1]);
}

test "x18 class 011: mux 000 loads !cond; w3[3] sends #head[20:5] to A too (849, 850)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // code 0x0E, load slot s12, imm 0x14A1, A<2*6 + 1> = A13; p<w2[17:14]> = p12
    const h: u32 = 0x9D829438;
    for ([_]struct { n: u32, w3: u32, p12: u32, s12: u32, a13: u32 }{
        .{ .n = 2, .w3 = 0x60000009, .p12 = 0, .s12 = 1, .a13 = 0x14A1 },
        .{ .n = 0, .w3 = 0x60000009, .p12 = 1, .s12 = 0, .a13 = 0x14A1 },
        .{ .n = 2, .w3 = 0x60000001, .p12 = 0, .s12 = 1, .a13 = 0x22 }, // w3[3] clear: A gets v = s1 + s0
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x22;
        st.p[12] = 7;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ h, 0x00000006, 0x30004 | (c.n << 6), c.w3 } ++ halt), 0, 8);
        try testing.expectEqual(c.p12, st.p[12]);
        try testing.expectEqual(c.s12, st.s[12]);
        try testing.expectEqual(c.a13, st.a[13]);
    }
}

test "x18 class 010 Q23 multiply: w3[0] = 1 rounds, 0 truncates (684)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const q = try asm_mod.assembleOne("x18.q23 s3, s1, s2, #4");
    for ([_]struct { b: u32, rnd: u32, want: u32 }{
        .{ .b = 8141, .rnd = 1, .want = 0x10 }, // 15.9
        .{ .b = 8141, .rnd = 0, .want = 0xF },
        .{ .b = 6292, .rnd = 1, .want = 0xC }, // 12.29
        .{ .b = 6292, .rnd = 0, .want = 0xC },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x4000;
        st.s[2] = c.b;
        const w3 = (q.slice()[3] & ~@as(u32, 1)) | c.rnd;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ q.slice()[0], q.slice()[1], q.slice()[2], w3 } ++ halt), 0, 8);
        try testing.expectEqual(c.want, st.s[3]);
    }
}

test "x18 class 010 X mux 011: Y[7:0] << 8 setting yd with w1[28] = 1, yd[7:0] << 8 with 0 (685, 686)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]struct { w1: u32, e: u32, want: ?u32 }{
        .{ .w1 = 0x10003000, .e = 3, .want = 0x3300 },
        .{ .w1 = 0x10003000, .e = 9, .want = 0x9900 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[3] = 0x40;
        st.e[9] = 0x80;
        try st.y_mem.put(testing.allocator, 0x40, 0x0A33);
        try st.y_mem.put(testing.allocator, 0x80, 0x0A99);
        const prog = [_]u32{ 0x00A00018, c.w1, 0x00000000, 0x40000001 | (c.e << 25) } ++ halt;
        const r = emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, st.s[5]);
            try testing.expectEqual(v >> 8 | 0x0A00, st.yd);
        } else try testing.expectError(emu.RunError.UnknownForm, r);
    }
    // w1[28] = 0 takes yd: a fresh read, then the copy, gives the same value
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.e[3] = 0x40;
    st.e[5] = 0x99;
    try st.y_mem.put(testing.allocator, 0x40, 0x0A33);
    const prog = [_]u32{ 0x00A00018, 0x10003000, 0, 0x40000001 | (3 << 25), 0x00C00018, 0x00003000, 0, 0x40000001 | (5 << 25) } ++ halt;
    _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
    try testing.expectEqual(@as(u32, 0x3300), st.s[6]);
}

test "x18 class 110 hi output follows the mode w3[20:18], X the previous read (687, 689)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]struct { mode: u32, want: ?u32 }{
        .{ .mode = 4, .want = 0x0C },
        .{ .mode = 5, .want = 0x1A0C },
        .{ .mode = 6, .want = 0xE60C },
        .{ .mode = 7, .want = 0x19F4 },
        .{ .mode = 0, .want = 0 },
        .{ .mode = 2, .want = 0x4600 },
        // mode 3: 689's ~X cell is withdrawn (720); pinned below with a known latch.
        .{ .mode = 1, .want = 0x7A00 }, // a + X: 0x6000 + 0x1A00
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x6000;
        st.s[2] = 0x1000;
        st.xd = 0xE01A;
        // a = s1 (w2[4:1]), b = s2 (w2[8:5]), hi -> s4 (w2[12:9])
        const w2: u32 = (1 << 1) | (2 << 5) | (4 << 9);
        // doubled form (w3[24] = 0), rounding (w3[22] = 1)
        const w3: u32 = (0xC1500001 & ~@as(u32, (7 << 18) | (1 << 24))) | (c.mode << 18);
        const r = emu.runFrom(testing.allocator, &st, &([_]u32{ 0x00000018, 0, w2, w3 } ++ halt), 0, 8);
        if (c.want) |v| {
            _ = try r;
            try testing.expectEqual(v, st.s[4]);
        } else try testing.expectError(emu.RunError.UnknownForm, r);
    }
    // beside its own X read (w1 mode 5) the multiply sees the old xd; the read sets xd (689)
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.s[1] = 0x6000;
    st.s[2] = 0x1000;
    st.xd = 0x55;
    st.a[3] = 0x40;
    try st.x_mem.put(testing.allocator, 0x40, 0xE01A);
    const w2: u32 = (1 << 1) | (2 << 5) | (4 << 9) | (3 << 17) | (1 << 22);
    const w3: u32 = (0xC1500001 & ~@as(u32, (7 << 18) | (1 << 24))) | (5 << 18);
    _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x00000018, 0x00005000, w2, w3 } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x550C), st.s[4]);
    try testing.expectEqual(@as(u32, 0xE01A), st.xd);
    // 720: this word's mode 3 with the latch at 0x12 is +(xd << 8), not inverted.
    var st3 = emu.State{};
    defer st3.deinit(testing.allocator);
    st3.s[1] = 0x6000;
    st3.s[2] = 0x1000;
    st3.xd = 0x12;
    const w3m: u32 = (0xC1500001 & ~@as(u32, (7 << 18) | (1 << 24))) | (3 << 18);
    _ = try emu.runFrom(testing.allocator, &st3, &([_]u32{ 0x00000018, 0, (1 << 1) | (2 << 5) | (4 << 9), w3m } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x1200), st3.s[4]);
}

test "one X latch: an x03 read sets the X that x18 class 110's mode uses; stores leave it (690)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.s[1] = 0x6000;
    st.s[2] = 0x1000;
    st.s[4] = 0x1000;
    st.a[2] = 0x40;
    st.a[7] = 0x40;
    st.xd = 0x99;
    try st.x_mem.put(testing.allocator, 0x40, 0x55);
    // x03 tap read s2 <- X:[A2] (671), then x18 class 110 mode 5 into s4
    const tap = [_]u32{ 0x80000003, 0x40006800, 0x01441485, 0x00000007, 0x00002c90, 0x00041202 };
    const w3: u32 = (0xC1500001 & ~@as(u32, (7 << 18) | (1 << 24))) | (5 << 18);
    const mpy = [_]u32{ 0x00000018, 0, (6 << 1) | (2 << 5) | (11 << 9), w3 };
    st.s[6] = 0x6000;
    _ = try emu.runFrom(testing.allocator, &st, &(tap ++ mpy ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x55), st.xd);
    // X part 0x55 << 8; P = 2 * 0x6000 * 0x55 = 0x3FC000 rounds to 0 at bit 24
    try testing.expectEqual(@as(u32, 0x5500), st.s[11]);
}

test "x18 X read returns all 24 bits of the word (693)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_][2]u32{ .{ 0xFF8303, 0xFF8303 }, .{ 0x600000, 0x600000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.a[3] = 0x40;
        try st.x_mem.put(testing.allocator, 0x40, c[0]);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x00A00018, 0x00005000, (3 << 17) | (1 << 22), 0xC0000000 } ++ halt), 0, 8);
        try testing.expectEqual(c[1], st.s[5]);
    }
}

test "class-05 add is 24 bits wide and wraps at 2^24 (695)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const add = try asm_mod.assembleOne("alu s3, s1, s2, #14");
    for ([_][3]u32{ .{ 0x7FFF, 0x7FFF, 0x00FFFE }, .{ 0x7FFF, 1, 0x008000 }, .{ 0xFFFFFF, 1, 0 }, .{ 0x600000, 0x600000, 0xC00000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = c[0];
        st.s[2] = c[1];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{add.slice()[0]} ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[3]);
    }
}

test "stx0f through A0 is refused: its address is the previous word's field (694)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st, &([_]u32{ 0x0000000F, 0xA0500000 } ++ halt), 0, 8));
}

test "class-05 shifts, the shifted add and Q23 at 24 bits (696)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // head: 0x40000005 | code << 5 | a << 11 | dest << 15 | s << 19 | b << 26
    const w = struct {
        fn alu(code: u32, s: u32) u32 {
            return 0x40000005 | (code << 5) | (1 << 11) | (3 << 15) | (s << 19) | (2 << 26);
        }
    };
    for ([_][4]u32{
        .{ 6, 4, 0x800000, 0xF80000 }, .{ 6, 20, 0x800000, 0xFFFFF8 }, .{ 6, 0xFFFFFC, 0x800000, 0 },
        .{ 8, 4, 0x800000, 0x080000 }, .{ 8, 20, 0x800000, 0x000008 }, .{ 8, 0xFFFFFC, 0x800000, 0 },
        .{ 7, 4, 0x800000, 0xF80000 }, .{ 7, 20, 0x800000, 0xFFFFF8 }, .{ 7, 0xFFFFFC, 0x800000, 0x800000 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = c[1];
        st.s[2] = c[2];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{w.alu(c[0], 0)} ++ halt), 0, 8);
        try testing.expectEqual(c[3], st.s[3]);
    }
    // add code 0 with s = 4: 0x0F0000 << 4 keeps 24 bits
    {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x0F0000;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{w.alu(0, 4)} ++ halt), 0, 8);
        try testing.expectEqual(@as(u32, 0xF00000), st.s[3]);
    }
    // shift codes with s != 0 do not shift: refused
    {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        try testing.expectError(emu.RunError.UnknownForm, emu.runFrom(testing.allocator, &st, &([_]u32{w.alu(6, 4)} ++ halt), 0, 8));
    }
    // x18 class 010 Q23, rounded: 0x400000 * 0x400000 >> 23 = 0x200000
    {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x400000;
        st.s[2] = 0x400000;
        const q = try asm_mod.assembleOne("x18.q23 s3, s1, s2, #4");
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ q.slice()[0], q.slice()[1], q.slice()[2], q.slice()[3] } ++ halt), 0, 8);
        try testing.expectEqual(@as(u32, 0x200000), st.s[3]);
    }
}

test "mul13 multiplies 24x24 and the x16 ALU keeps 24 bits (697)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const sub = try asm_mod.assembleOne("x16.alu2 s3, s1, s2, #4, #1");
    for ([_][3]u32{ .{ 0x400000, 1, 0x3FFFFF }, .{ 0x007FFF, 0xFF8000, 0x00FFFF } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = c[0];
        st.s[2] = c[1];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ sub.slice()[0], sub.slice()[1], sub.slice()[2] } ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[3]);
    }
}

test "x16mul, x16mul2 and x19alu's saturating shift work at 24 bits (698)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_][]const u8{ "x16.mul s3, s1, s2, #5", "x16.mul s3, s1, s2, #1", "x16.mul2 #3, #0, s1, s2, #0, #1" }) |src| {
        const m = try asm_mod.assembleOne(src);
        for ([_][3]u32{ .{ 0x400000, 0x400000, 0x200000 }, .{ 0x400000, 0x4000, 0x2000 }, .{ 0x4000, 0x4000, 0x20 } }) |c| {
            var st = emu.State{};
            defer st.deinit(testing.allocator);
            st.s[1] = c[0];
            st.s[2] = c[1];
            _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ m.slice()[0], m.slice()[1], m.slice()[2] } ++ halt), 0, 8);
            try testing.expectEqual(c[2], st.s[3]);
        }
    }
    const sh = try asm_mod.assembleOne("x19.alu s7, s3, s6, #23");
    for ([_][3]u32{ .{ 0x400000, 1, 0x7FFFFF }, .{ 0x100000, 2, 0x400000 }, .{ 0x400000, 0xFFFFFF, 0x200000 }, .{ 0x400000, 0xFFFFEC, 4 }, .{ 0x7FFF, 0x010001, 0x7FFFFF } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[3] = c[0];
        st.s[6] = c[1];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ sh.slice()[0], sh.slice()[1], sh.slice()[2], sh.slice()[3] } ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[7]);
    }
}

test "x19alu's 32 ops are 24 bits wide (699)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    var want: [32]u32 = undefined;
    for (&want) |*w| w.* = 0;
    for ([_]u32{ 3, 4, 28, 29 }) |c| want[c] = 0x400000;
    for ([_]u32{ 5, 16, 17, 18, 19, 20, 21, 26 }) |c| want[c] = 0x3FFFFF;
    for ([_]u32{ 6, 7, 10, 11, 12, 13 }) |c| want[c] = 0x4000FF;
    want[31] = 0x400001;
    want[23] = 0x7FFFFF;
    var bad: u32 = 0;
    for (0..32) |code| {
        var buf: [64]u8 = undefined;
        const src = try std.fmt.bufPrint(&buf, "x19.alu s7, s3, s6, #{d}", .{code});
        const m = try asm_mod.assembleOne(src);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[3] = 0x3FFFFF;
        st.s[6] = 0x4000FF;
        st.s[7] = 0xABCDEF;
        const r = emu.runFrom(testing.allocator, &st, &([_]u32{ m.slice()[0], m.slice()[1], m.slice()[2], m.slice()[3] } ++ halt), 0, 8);
        if (r) |_| {
            if (st.s[7] != want[code]) {
                std.debug.print("code {d}: got {x} want {x}\n", .{ code, st.s[7], want[code] });
                bad += 1;
            }
        } else |e| {
            std.debug.print("code {d}: {any}\n", .{ code, e });
            bad += 1;
        }
    }
    try testing.expectEqual(@as(u32, 0), bad);
}

test "the running MAC is 24x24 into a 48-bit saturating acc on both forms (701)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // x03 row: a s2, b s4, hi s10 (fed back), lo s9 (fed back).
    const x03 = [_]u32{ 0x80000003, 0x40006800, 0x01441485, 0x00000007, 0x00002c80, 0x00041202 };
    const m18 = try asm_mod.assembleOne("x18.macf s4, #5, s1, s2, s4, s6, s6");
    const cases = [_]struct { a: i64, b: i64, n: usize }{
        .{ .a = -0x7FFF, .b = 0x1FFFF, .n = 32 },
        .{ .a = -0x7FFF, .b = -0x1FFFF, .n = 32 },
        .{ .a = 0x123456, .b = 0x100, .n = 1 },
        .{ .a = -0x200000, .b = 0x7FF, .n = 4 },
        .{ .a = 0x7FFFFF, .b = 0x7FFFFF, .n = 1 },
        .{ .a = -0x800000, .b = 0x400000, .n = 1 },
        .{ .a = 0x7FFFFF, .b = 0x7FFFFF, .n = 2 }, // saturates
        .{ .a = -0x800000, .b = 0x7FFFFF, .n = 2 }, // saturates low (702)
        .{ .a = -0x800000, .b = -0x800000, .n = 1 }, // +2^47 saturates (702)
    };
    for (cases) |c| {
        const true_sum: i64 = std.math.clamp(@as(i64, @intCast(c.n)) * 2 * c.a * c.b, -(@as(i64, 1) << 47), (@as(i64, 1) << 47) - 1);
        const want_hi: u32 = @truncate(@as(u64, @bitCast(true_sum >> 24)) & 0xFFFFFF);
        const want_lo: u32 = @truncate(@as(u64, @bitCast(true_sum)) & 0xFFFFFF);
        for (0..2) |form| {
            var st = emu.State{};
            defer st.deinit(testing.allocator);
            var prog: [32 * 6 + 3]u32 = undefined;
            const blen: usize = if (form == 0) 6 else m18.slice().len;
            for (0..c.n) |k| @memcpy(prog[k * blen ..][0..blen], if (form == 0) x03[0..] else m18.slice());
            @memcpy(prog[c.n * blen ..][0..3], &halt);
            const ra: usize = if (form == 0) 2 else 1;
            const rb: usize = if (form == 0) 4 else 2;
            st.s[ra] = @as(u32, @truncate(@as(u64, @bitCast(c.a)))) & 0xFFFFFF;
            st.s[rb] = @as(u32, @truncate(@as(u64, @bitCast(c.b)))) & 0xFFFFFF;
            _ = try emu.runFrom(testing.allocator, &st, prog[0 .. c.n * blen + 3], 0, 256);
            try testing.expectEqual(want_hi, st.s[if (form == 0) 10 else 4]);
            try testing.expectEqual(want_lo, st.s[if (form == 0) 9 else 6]);
        }
    }
}

test "x01.norm and x01.selneg on any registers (502, 666, 669)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const n = try asm_mod.assembleOne("x01.norm s3, s12");
    for ([_][2]u32{ .{ 0x100, 14 }, .{ 0xFFFF00, 15 }, .{ 0x7FFF, 8 }, .{ 0, 24 }, .{ 0xFFFFFF, 23 }, .{ 0x400000, 0 }, .{ 1, 22 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[12] = c[0];
        st.r[5] = 0x40;
        st.e[6] = 0x77;
        var prog: [10]u32 = undefined;
        @memcpy(prog[0..7], n.slice()[0..7]);
        @memcpy(prog[7..10], &halt);
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(c[1], st.s[3]);
        try testing.expectEqual(@as(u32, 0x40), st.r[5]);
        try testing.expectEqual(@as(u32, 0x77), st.e[6]);
    }
    const sel = try asm_mod.assembleOne("x01.selneg s4, s7, #9");
    for ([_][2]u32{ .{ 0xFFFFF0, 0xFFFFF0 }, .{ 5, 9 }, .{ 0, 9 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[7] = c[0];
        var prog: [10]u32 = undefined;
        @memcpy(prog[0..7], sel.slice()[0..7]);
        @memcpy(prog[7..10], &halt);
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(c[1], st.s[4]);
    }
}

test "unrolled 672 taps against coefficient registers give the exact dot product" {
    // Each tap: s2 <- X:[A2], A2 += 1, {s10:s9} += 2 * s2 * s<coef>; the load lands after
    // the multiply, so s2 is primed with d[0] and the last load is a dummy.
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const coef_regs = [_]u32{ 1, 3, 4, 5, 6, 7, 8, 11 };
    var prng = std.Random.DefaultPrng.init(0x672);
    const rnd = prng.random();
    for (0..200) |_| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        var c: [8]i64 = undefined;
        var d: [9]i64 = undefined;
        for (&c, 0..) |*v, j| {
            v.* = rnd.intRangeAtMost(i64, -0x8000, 0x7FFF);
            st.s[coef_regs[j]] = @as(u32, @bitCast(@as(i32, @intCast(v.*)))) & 0xFFFFFF;
        }
        for (&d, 0..) |*v, j| {
            v.* = rnd.intRangeAtMost(i64, -0x8000, 0x7FFF);
            try st.x_mem.put(testing.allocator, @intCast(0x100 + j), @as(u32, @bitCast(@as(i32, @intCast(v.*)))) & 0xFFFFFF);
        }
        st.s[2] = st.x_mem.get(0x100).?; // prime
        st.a[2] = 0x101;
        var prog: [8 * 6 + 3]u32 = undefined;
        for (coef_regs, 0..) |r, j| {
            const w2: u32 = (0x00041485 & ~@as(u32, 0xF << 5)) | (r << 5);
            prog[6 * j ..][0..6].* = .{ 0x80000003, 0x40006800, w2, 0x00000002, 0x00002c90, 0x00041202 };
        }
        prog[48..51].* = halt;
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 64);
        var want: i64 = 0;
        for (0..8) |j| want += 2 * c[j] * d[j];
        const hi: i64 = @as(i16, @bitCast(@as(u16, @truncate(st.s[10]))));
        const got: i64 = hi * (1 << 24) + st.s[9];
        try testing.expectEqual(want, got);
    }
}

test "Y words are 16 bits and ldy sign-extends Y[15] (700)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const ld = try asm_mod.assembleOne("ldy s14, e5");
    for ([_][2]u32{ .{ 0x401234, 0x001234 }, .{ 0x8000, 0xFF8000 }, .{ 0xFF7FFF, 0x007FFF } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[5] = 0x300;
        try st.y_mem.put(testing.allocator, 0x300, c[0]);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ld.slice()[0]} ++ halt), 0, 8);
        try testing.expectEqual(c[1], st.s[14]);
    }
}

test "x03.tap: a 32-tap 16 x 18-bit dot product, exact (672, 701)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const coef_regs = [_]u32{ 1, 2, 4, 5, 6, 7, 8, 9 };
    var taps: [8][6]u32 = undefined;
    for (coef_regs, 0..) |r, j| {
        var buf: [64]u8 = undefined;
        const b = try asm_mod.assembleOne(try std.fmt.bufPrint(&buf, "x03.tap s3, #5, s{d}, s12, s11", .{r}));
        @memcpy(&taps[j], b.slice()[0..6]);
    }
    var prng = std.Random.DefaultPrng.init(701);
    const rnd = prng.random();
    for (0..50) |_| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        var c: [8]i64 = undefined;
        for (&c, 0..) |*v, j| {
            v.* = rnd.intRangeAtMost(i64, -0x8000, 0x7FFF);
            st.s[coef_regs[j]] = @as(u32, @truncate(@as(u64, @bitCast(v.*)))) & 0xFFFFFF;
        }
        var d: [33]i64 = undefined;
        for (&d, 0..) |*v, j| {
            v.* = rnd.intRangeAtMost(i64, -0x20000, 0x1FFFF);
            try st.x_mem.put(testing.allocator, @intCast(0x200 + j), @as(u32, @truncate(@as(u64, @bitCast(v.*)))) & 0xFFFFFF);
        }
        st.s[3] = st.x_mem.get(0x200).?; // prime
        st.a[5] = 0x201;
        var prog: [32 * 6 + 3]u32 = undefined;
        for (0..32) |k| prog[6 * k ..][0..6].* = taps[k % 8];
        prog[192..195].* = halt;
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 256);
        var want: i64 = 0;
        for (0..32) |k| want += 2 * c[k % 8] * d[k];
        const got: i64 = (@as(i64, @as(i24, @bitCast(@as(u24, @truncate(st.s[12]))))) << 24) + st.s[11];
        try testing.expectEqual(want, got);
        try testing.expectEqual(@as(u32, 0x221), st.a[5]);
    }
}

test "x03.tap0 starts the sum: 2ab, the X registers ignored (702)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const m = try asm_mod.assembleOne("x03.tap0 s2, #2, s4, s10, s9");
    try testing.expectEqual(@as(u32, 0x40006000), m.slice()[1]);
    for ([_][4]u32{
        .{ 0x7FFFFF, 0x7FFFFF, 0x7FFFFE, 0x000002 },
        .{ 0x800000, 0x7FFFFF, 0x800001, 0x000000 },
        .{ 0x123456, 0xFFFF00, 0xFFFFDB, 0x975400 },
        .{ 0xFF8001, 0x01FFFF, 0xFFFE00, 0x04FFFE },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[2] = c[0];
        st.s[4] = c[1];
        st.s[10] = 0x654321;
        st.s[9] = 0x123456;
        st.a[2] = 0x40;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ m.slice()[0], m.slice()[1], m.slice()[2], m.slice()[3], m.slice()[4], m.slice()[5] } ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[10]);
        try testing.expectEqual(c[3], st.s[9]);
    }
}

test "x18 class 111 Y-side tap: load, e step and exact MAC in one bundle (703)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const t = try asm_mod.assembleOne("x18.ytap s1, e3, s2, s4, s6");
    try testing.expectEqualSlices(u32, &.{ 0x1C200018, 0x42375007, 0x21326006, 0xE60700D4 }, t.slice()[0..4]);
    const tap = t.slice()[0..4].*;
    const want = [_][2]u32{ .{ 0x111, 0 }, .{ 0x222, 0x222000 }, .{ 0x333, 0x666000 } };
    for (1..4) |n| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        for ([_]u32{ 0x111, 0x222, 0x333 }, 0..) |v, i| try st.y_mem.put(testing.allocator, @intCast(0x100 + i), v);
        st.e[3] = 0x100;
        st.s[2] = 0x1000;
        var prog: [3 * 4 + 3]u32 = undefined;
        for (0..n) |k| prog[4 * k ..][0..4].* = tap;
        prog[4 * n ..][0..3].* = halt;
        _ = try emu.runFrom(testing.allocator, &st, prog[0 .. 4 * n + 3], 0, 16);
        try testing.expectEqual(want[n - 1][0], st.s[1]);
        try testing.expectEqual(want[n - 1][1], st.s[6]);
        try testing.expectEqual(@as(u32, 0), st.s[4]);
        try testing.expectEqual(@as(u32, 0x100 + @as(u32, @intCast(n))), st.e[3]);
    }
}

test "two-pointer taps: x19ld + x03.tap and x02 load + x18.ytap, primed, exact (705)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    var prng = std.Random.DefaultPrng.init(705);
    const rnd = prng.random();
    // Pair B: s4 <- Y:[e2], e2 += 1 (sign-extended); then x03.tap s2, #2, s4, s10, s9.
    const yl = try asm_mod.assembleOne("x19.ld s4, #1, e2, #2");
    try testing.expectEqualSlices(u32, &.{ 0x00000019, 0x00000109, 0x00001000, 0xE4000403 }, yl.slice()[0..4]);
    const yld = yl.slice()[0..4].*;
    const tb = try asm_mod.assembleOne("x03.tap s2, #2, s4, s10, s9");
    // Pair A: s2 <- X:[A9], A9 += 1; then x18.ytap s1, e3, s2, s4, s6 (Y zero-extends).
    const xld = [_]u32{ 0x00000002, 0x0002A000, 0x00120000, 0x00000009, 0x40522810, 0x00004223 };
    const ta = try asm_mod.assembleOne("x18.ytap s1, e3, s2, s4, s6");
    for (0..2) |pair| for (0..20) |_| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        var c: [32]i64 = undefined;
        var d: [33]i64 = undefined;
        for (0..33) |k| {
            d[k] = if (pair == 0) rnd.intRangeAtMost(i64, -0x20000, 0x1FFFF) else rnd.intRangeAtMost(i64, 0, 0xFFFF);
            if (pair == 0)
                try st.x_mem.put(testing.allocator, @intCast(0x300 + k), @as(u32, @truncate(@as(u64, @bitCast(d[k])))) & 0xFFFFFF)
            else
                try st.y_mem.put(testing.allocator, @intCast(0x300 + k), @intCast(d[k]));
        }
        for (0..32) |k| {
            c[k] = if (pair == 0) rnd.intRangeAtMost(i64, -0x8000, 0x7FFF) else rnd.intRangeAtMost(i64, -0x8000, 0x7FFF);
            if (pair == 0)
                try st.y_mem.put(testing.allocator, @intCast(0x500 + k), @as(u32, @truncate(@as(u64, @bitCast(c[k])))) & 0xFFFF)
            else
                try st.x_mem.put(testing.allocator, @intCast(0x500 + k), @as(u32, @truncate(@as(u64, @bitCast(c[k])))) & 0xFFFFFF);
        }
        var prog: [33 * 10 + 3]u32 = undefined;
        var n: usize = 0;
        const tap = if (pair == 0) tb.slice() else ta.slice();
        const ld: []const u32 = if (pair == 0) &yld else &xld;
        @memcpy(prog[n..][0..tap.len], tap); // prime: MAC 0 * 0, loads d[0]
        n += tap.len;
        for (0..32) |_| {
            @memcpy(prog[n..][0..ld.len], ld);
            n += ld.len;
            @memcpy(prog[n..][0..tap.len], tap);
            n += tap.len;
        }
        @memcpy(prog[n..][0..3], &halt);
        if (pair == 0) {
            st.a[2] = 0x300;
            st.e[2] = 0x500;
        } else {
            st.e[3] = 0x300;
            st.a[9] = 0x500;
        }
        _ = try emu.runFrom(testing.allocator, &st, prog[0 .. n + 3], 0, 512);
        var want: i64 = 0;
        for (0..32) |k| want += 2 * c[k] * d[k];
        const hi: u32 = if (pair == 0) st.s[10] else st.s[4];
        const lo: u32 = if (pair == 0) st.s[9] else st.s[6];
        try testing.expectEqual(want, (@as(i64, @as(i24, @bitCast(@as(u24, @truncate(hi))))) << 24) + lo);
        if (pair == 0) {
            try testing.expectEqual(@as(u32, 0x300 + 33), st.a[2]);
            try testing.expectEqual(@as(u32, 0x500 + 32), st.e[2]);
        }
    };
}

test "Y 0x8000: x18.ytap zero-extends, x19.ld sign-extends and stores nothing (705)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const yt = try asm_mod.assembleOne("x18.ytap s1, e3, s2, s4, s6");
    const ln = try asm_mod.assembleOne("x19.ld s4, #1, e2, #2");
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    try st.y_mem.put(testing.allocator, 0x40, 0x8000);
    try st.y_mem.put(testing.allocator, 0x41, 0x1234);
    st.e[3] = 0x40;
    st.e[2] = 0x40;
    st.v[0] = 0x777;
    _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ yt.slice()[0], yt.slice()[1], yt.slice()[2], yt.slice()[3], ln.slice()[0], ln.slice()[1], ln.slice()[2], ln.slice()[3] } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x008000), st.s[1]);
    try testing.expectEqual(@as(u32, 0xFF8000), st.s[4]);
    try testing.expectEqual(@as(?u32, 0x1234), st.getY(0x41));
}

test "x18 class 110 code 11 op 1: w3[26:25] picks the relation (706)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const pts = [_][2]u32{ .{ 0x100, 0x200 }, .{ 0x300, 0x200 }, .{ 0x200, 0x200 }, .{ 0xFF8100, 0x200 } };
    // rel 0 !=, 1 ==, 2 <=, 3 >=, per point
    const want = [4][4]u8{ .{ 1, 1, 0, 1 }, .{ 0, 0, 1, 0 }, .{ 1, 0, 1, 1 }, .{ 0, 1, 1, 0 } };
    for (0..4) |rel| for (pts, 0..) |pt, i| for (0..2) |pre| {
        const w3: u32 = (0xCC500009 & ~@as(u32, 3 << 25)) | (@as(u32, @intCast(rel)) << 25);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = pt[0];
        st.s[5] = pt[1];
        st.p[2] = @intCast(pre);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x16200018, 0x0000D050, 0x00444C46, w3 } ++ halt), 0, 8);
        try testing.expectEqual(want[rel][i], st.p[2]);
    };
}

test "x19.ld stores nothing (707); w1[20] or w1[5] is refused (793)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const base = try asm_mod.assembleOne("x19.ld s5, #5, e10, #2");
    for ([_]u32{ 0, 1 << 20, 1 << 5, (1 << 20) | (1 << 5) }) |extra| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[10] = 0x700;
        st.v[0] = 0x333;
        try st.y_mem.put(testing.allocator, 0x700, 0x111);
        try st.y_mem.put(testing.allocator, 0x701, 0xAAAA);
        const w = base.slice();
        const prog = [_]u32{ w[0], w[1] | extra, w[2], w[3] } ++ halt;
        if (extra != 0) {
            try testing.expectError(error.UnknownForm, emu.runFrom(testing.allocator, &st, &prog, 0, 8));
            continue;
        }
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(u32, 0x111), st.s[5]);
        try testing.expectEqual(@as(u32, 0x701), st.e[10]);
        try testing.expectEqual(@as(?u32, 0xAAAA), st.getY(0x701));
    }
}

test "x18 class 011 or, max, min, abs and x03's X store at 24 bits (708)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const Case = struct { src: []const u8, a: u32, b: u32, want: u32 };
    const cases = [_]Case{
        .{ .src = "x18.or s3, s1, s2", .a = 0x400001, .b = 0x020000, .want = 0x420001 },
        .{ .src = "x18.or s3, s1, s2", .a = 0x00C000, .b = 0x001000, .want = 0x00D000 },
        .{ .src = "x18.max s3, s1, s2", .a = 0x00C000, .b = 0x001000, .want = 0x00C000 },
        .{ .src = "x18.min s3, s1, s2", .a = 0x00C000, .b = 0x001000, .want = 0x001000 },
        .{ .src = "x18.max s3, s1, s2", .a = 0xFF4000, .b = 0x000100, .want = 0x000100 },
        .{ .src = "x18.min s3, s1, s2", .a = 0xFF4000, .b = 0x000100, .want = 0xFF4000 },
        .{ .src = "x18.max s3, s1, s2", .a = 0x400000, .b = 0xC00000, .want = 0x400000 },
        .{ .src = "x18.min s3, s1, s2", .a = 0x400000, .b = 0xC00000, .want = 0xC00000 },
        .{ .src = "x18.una s3, s1, #7, R0, s0", .a = 0x00C000, .b = 0, .want = 0x00C000 },
        .{ .src = "x18.una s3, s1, #7, R0, s0", .a = 0xFF4000, .b = 0, .want = 0x00C000 },
        .{ .src = "x18.una s3, s1, #7, R0, s0", .a = 0xC00000, .b = 0, .want = 0x400000 },
        .{ .src = "x18.una s3, s1, #7, R0, s0", .a = 0x800000, .b = 0, .want = 0x800000 },
    };
    for (cases) |c| {
        const m = try asm_mod.assembleOne(c.src);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = c.a;
        st.s[2] = c.b;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ m.slice()[0], m.slice()[1], m.slice()[2], m.slice()[3] } ++ halt), 0, 8);
        try testing.expectEqual(c.want, st.s[3]);
    }
}

test "x18 min and max select on a's sign, not on b (951)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const Case = struct { src: []const u8, a: u32, b: u32, want: u32 };
    const cases = [_]Case{
        .{ .src = "x18.min s3, s1, s2", .a = 0, .b = 0x7FFF, .want = 0x7FFF },
        .{ .src = "x18.min s3, s1, s2", .a = 0, .b = 1, .want = 1 },
        .{ .src = "x18.min s3, s1, s2", .a = 0xFFFFFF, .b = 0xFFFF9C, .want = 0xFFFFFF },
        .{ .src = "x18.max s3, s1, s2", .a = 1, .b = 0x7FFFFF, .want = 1 },
        .{ .src = "x18.max s3, s1, s2", .a = 0, .b = 0xFFFFFF, .want = 0xFFFFFF },
        .{ .src = "x18.max s3, s1, s2", .a = 0, .b = 0xFF8000, .want = 0xFF8000 },
    };
    for (cases) |c| {
        const m = try asm_mod.assembleOne(c.src);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = c.a;
        st.s[2] = c.b;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ m.slice()[0], m.slice()[1], m.slice()[2], m.slice()[3] } ++ halt), 0, 8);
        try testing.expectEqual(c.want, st.s[3]);
    }
}

test "x03's X store writes all 24 bits (708)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const m = try asm_mod.assembleOne("x03.xst #3, #1, #2, #0, #3");
    for ([_]u32{ 0x401234, 0x00C000, 0xFFBB0B }) |v| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.a[8] = 0x3A0;
        st.s[11] = v;
        var prog: [9]u32 = undefined;
        @memcpy(prog[0..6], m.slice()[0..6]);
        @memcpy(prog[6..9], &halt);
        _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
        try testing.expectEqual(@as(?u32, v), st.readX(0x3A0));
    }
}

test "x03.tap0 into s0 is a pure X load-and-step; stx from s0 stores xd << 8 (709)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // 709's word: x03.tap0 s3, #5, s0, s0, s0 but with the multiplicand w2[4:1] = s0 (the
    // mnemonic ties it to sL = s3; both discard the product, only this one is measured).
    const word = [_]u32{ 0x40000003, 0x00006001, 0x000A0000, 0x00000005, 0x00002818, 0x00040002 };
    const stx = try asm_mod.assembleOne("stx A6, s0");
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.a[5] = 0x40;
    st.a[6] = 0x80;
    try st.x_mem.put(testing.allocator, 0x40, 0x111);
    try st.x_mem.put(testing.allocator, 0x41, 0x222);
    for (1..15) |k| st.s[k] = @intCast(0x10 * k);
    var prog: [16]u32 = undefined;
    @memcpy(prog[0..6], &word);
    @memcpy(prog[6..12], &word);
    prog[12] = stx.slice()[0];
    @memcpy(prog[13..16], &halt);
    _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
    try testing.expectEqual(@as(u32, 0x222), st.s[3]);
    try testing.expectEqual(@as(u32, 0x42), st.a[5]);
    try testing.expectEqual(@as(u32, 0), st.s[0]);
    for (1..15) |k| if (k != 3) try testing.expectEqual(@as(u32, @intCast(0x10 * k)), st.s[k]);
    try testing.expectEqual(@as(?u32, 0x022200), st.readX(0x80));
}

test "x18 class 011 codes 0x22 and 0x23 ignore the op field (710)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const base = try asm_mod.assembleOne("x18.and s3, s1, s2");
    var bad: u32 = 0;
    for ([_]u32{ 0x22, 0x23 }) |code| for ([_]u32{ 0, 2, 4 }) |n| for (0..8) |op| {
        if (code == 0x22 and n == 4) continue;
        if (code == 0x23 and n == 2) continue;
        const w = base.slice();
        const h = (w[0] & ~@as(u32, 0x3F << 25)) | (code << 25);
        const w1 = (w[1] & ~@as(u32, 7 << 15)) | (@as(u32, @intCast(op)) << 15);
        const w2 = (w[2] & ~@as(u32, 0xF << 6)) | (n << 6);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.p[3] = 1; // the silicon rows' stale p3 (752)
        st.s[1] = 0x123;
        st.s[2] = 0x456;
        st.s[3] = 0xABCDEF;
        const want: u32 = if (code == 0x22) 0 else 0x456;
        if (emu.runFrom(testing.allocator, &st, &([_]u32{ h, w1, w2, w[3] } ++ halt), 0, 8)) |_| {
            if (st.s[3] != want) {
                std.debug.print("code {x} n {d} op {d}: {x}\n", .{ code, n, op, st.s[3] });
                bad += 1;
            }
        } else |e| {
            std.debug.print("code {x} n {d} op {d}: {any}\n", .{ code, n, op, e });
            bad += 1;
        }
    };
    try testing.expectEqual(@as(u32, 0), bad);
}

test "x18 class 110: w3[21] = 0 takes the hi MAC's X from s<w3[13:10]> (711)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const base = [_]u32{ 0x1E200018, 0x80005AA0, 0x00500E2B, 0xC0542C39 };
    for ([_][2]u32{ .{ 5, 0x686 }, .{ 6, 0xF9BA }, .{ 7, 0x646 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[1] = 0x4000;
        st.s[5] = 0x4000;
        st.s[6] = 0x666;
        st.xd = 0x111;
        const w3 = (base[3] & ~@as(u32, (7 << 18) | (0xF << 10))) | (c[0] << 18) | (6 << 10);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ base[0], base[1], base[2], w3 } ++ halt), 0, 8);
        try testing.expectEqual(c[1], st.s[7]);
    }
    // w3[21] = 1 ignores the field: X is xd again.
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.s[1] = 0x4000;
    st.s[5] = 0x4000;
    st.s[6] = 0x666;
    const w3 = (base[3] & ~@as(u32, 0xF << 10)) | (6 << 10) | (1 << 21);
    _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ base[0], base[1], base[2], w3 } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0x20), st.s[7]);
}

test "x18 class 110 field X in modes 1 and 2; mode 3 keeps xd; mode 0 is 0 (712)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const base = [_]u32{ 0x1E200018, 0x80005AA0, 0x00500E2B, 0xC0542C39 };
    for ([_][2]u32{ .{ 1, 0x789 }, .{ 2, 0xFABD }, .{ 0, 0 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x123;
        st.s[1] = 0x4000;
        st.s[6] = 0x666;
        const w3 = (base[3] & ~@as(u32, (7 << 18) | (0xF << 10))) | (c[0] << 18) | (6 << 10);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ base[0], base[1], base[2], w3 } ++ halt), 0, 8);
        try testing.expectEqual(c[1], st.s[7]);
    }
    // Mode 3 gives the same with and without the field.
    var got: [2]u32 = undefined;
    for (0..2) |k| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[6] = 0x666;
        st.xd = 0x55;
        const fld: u32 = if (k == 0) 6 << 10 else 0;
        const w3 = (base[3] & ~@as(u32, (7 << 18) | (0xF << 10))) | (3 << 18) | fld;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ base[0], base[1], base[2], w3 } ++ halt), 0, 8);
        got[k] = st.s[7];
    }
    try testing.expectEqual(got[1], got[0]);
}

test "x18 class 110 lo X term: s<w3[13:10]> in modes 5/6, xd << 8 for field 0 (713)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const h = [_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B };
    const base3: u32 = 0xC0542C39 & ~@as(u32, (1 << 24) | (1 << 21) | (7 << 18) | (0xF << 10));
    const Case = struct { int: bool, w21: bool, mode: u32, field: u32, want: u32 };
    for ([_]Case{
        .{ .int = false, .w21 = true, .mode = 5, .field = 6, .want = 0x2AC6 },
        .{ .int = false, .w21 = true, .mode = 6, .field = 6, .want = 0x1DFA },
        .{ .int = false, .w21 = true, .mode = 5, .field = 13, .want = 0x323D },
        .{ .int = false, .w21 = false, .mode = 5, .field = 6, .want = 0x2460 },
        .{ .int = true, .w21 = true, .mode = 5, .field = 6, .want = 0x1896 },
        .{ .int = true, .w21 = false, .mode = 6, .field = 6, .want = 0x0BCA },
        .{ .int = true, .w21 = true, .mode = 5, .field = 13, .want = 0x200D },
        .{ .int = false, .w21 = true, .mode = 5, .field = 0, .want = 0x2460 + 0x6B00 },
    }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x123;
        st.s[1] = 0x10;
        st.s[6] = 0x666;
        st.s[13] = 0xDDD;
        st.xd = 0x6B;
        const w3 = base3 | (@as(u32, @intFromBool(c.int)) << 24) | (@as(u32, @intFromBool(c.w21)) << 21) | (c.mode << 18) | (c.field << 10);
        // Integer hi with w3[21] = 1 takes an X term not yet pinned: drop the hi dest there.
        const w2 = if (c.int and c.w21) h[2] & ~@as(u32, 0xF << 9) else h[2];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ h[0], h[1], w2, w3 } ++ halt), 0, 8);
        try testing.expectEqual(c.want, st.s[12]);
        if (c.int and !c.w21) try testing.expectEqual(@as(u32, 0), st.s[7]);
    }
}

test "x18 class 110 hi X under w3[21] = 1 is the s at index 12 here, not xd; lo mode 7 is X - P (714)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const h = [_]u32{ 0x1E200018, 0x80005AA0, 0x00500E2B };
    const Case = struct { w3: u32, s12: u32, want: u32 };
    for ([_]Case{
        .{ .w3 = 0xC1741839, .s12 = 0x42, .want = 0x42 }, // integer mode 5
        .{ .w3 = 0xC0741839, .s12 = 0xC2, .want = 0xC2 }, // fractional mode 5
        .{ .w3 = 0xC1781839, .s12 = 1, .want = 0xFFFF }, // integer mode 6: -s12
    }) |c| for ([_]u32{ 0x100, 0x1234, 0x8000 }) |xd| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x10;
        st.s[1] = 0x10;
        st.s[12] = c.s12;
        st.xd = xd;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ h[0], h[1], h[2], c.w3 } ++ halt), 0, 8);
        try testing.expectEqual(c.want, st.s[7]);
    };
    // lo mode 7, fractional, w3[21] = 1, field 6: lo = s6 - P.
    var st2 = emu.State{};
    defer st2.deinit(testing.allocator);
    st2.s[5] = 0x123;
    st2.s[1] = 0x10;
    st2.s[6] = 0x666;
    const w3 = (0xC0741839 & ~@as(u32, (7 << 18) | (0xF << 10))) | (7 << 18) | (6 << 10);
    _ = try emu.runFrom(testing.allocator, &st2, &([_]u32{ h[0], h[1], 0x0C500E2B & ~@as(u32, 0xF << 9), w3 } ++ halt), 0, 8);
    try testing.expectEqual(@as(u32, 0xE206), st2.s[12] & 0xFFFF);
}

test "x18 class 110 hi X under w3[21] = 1 is s<{w2[0], w1[31:29]}> (715)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // 714's word has index 12; w1[29] -> 13, w1[30] -> 14, w2[0] -> 4.
    for ([_][3]u32{ .{ 0, 0, 12 }, .{ 1 << 29, 0, 13 }, .{ 1 << 30, 0, 14 }, .{ 0, 1, 4 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x10;
        st.s[1] = 0x10;
        for ([_]u32{ 4, 12, 13, 14 }) |k| st.s[k] = 0x100 + k;
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0 ^ c[0], 0x00500E2B ^ c[1], 0xC0741839 } ++ halt), 0, 8);
        try testing.expectEqual(0x100 + c[2], st.s[7]);
    }
}

test "x18 class 110 hi X index 0 under w3[21] = 1 is the incoming xd << 8 (716)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // 714's word with index 0: clear w1[31:29] and w2[0].
    const w1: u32 = 0x80005AA0 & 0x1FFFFFFF;
    const w2: u32 = 0x00500E2B & ~@as(u32, 1);
    for ([_]u32{ 0xC0741839, 0xC1741839 }) |w3| for ([_][2]u32{ .{ 0x12, 0x1200 }, .{ 0x34, 0x3400 }, .{ 0xF0, 0xF000 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x10;
        st.s[1] = 0x10;
        st.xd = c[0];
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, w1, w2, w3 } ++ halt), 0, 8);
        try testing.expectEqual(c[1], st.s[7]);
    };
}

test "x18 class 110 lo field 0 adds the incoming xd << 8, both forms (717)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const base3: u32 = 0xC0541839 & ~@as(u32, (1 << 24) | (7 << 18) | (0xF << 10));
    for ([_][3]u32{ .{ 0, 0x12, 0x3660 }, .{ 0, 0xF0, 0x11460 }, .{ 1, 0x12, 0x2430 }, .{ 1, 0xF0, 0x0230 } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x123;
        st.s[1] = 0x10;
        st.xd = c[1];
        try st.x_mem.put(testing.allocator, 0, 0x34); // the bundle's own read: must not reach lo
        const w3 = base3 | (1 << 21) | (c[0] << 24) | (5 << 18);
        // hi dest cleared: integer hi under w3[21] = 1 needs its own X index, not under test here.
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B & ~@as(u32, 0xF << 9), w3 } ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[12] & (if (c[0] == 1) @as(u32, 0xFFFF) else 0xFFFFFF));
    }
}

test "x18 class 110's two mode tables, w3[23] = 0 and 1, hi and lo (718)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // a = s5 = 0x4000, b = s1 = 0x2000 (P hi 0x10, lo 0); index X s12 = 0xC2; field X s6 = 0x66.
    // hi dest s7, lo dest s12 (read at the bundle's start).
    const t1 = [8][2]u32{ .{ 0xB2, 0x66 }, .{ 0x4000, 0x2000 }, .{ 0x4000, 0x2000 }, .{ 0x40C2, 0x2066 }, .{ 0x40C2, 0x2066 }, .{ 0x3F3E, 0x1F9A }, .{ 0xD2, 0x66 }, .{ 0xD2, 0x66 } };
    const t0 = [_][3]u32{ .{ 0, 0, 0 }, .{ 1, 0x40C2, 0x2066 }, .{ 2, 0x3F3E, 0x1F9A }, .{ 3, 0xC2, 0x66 }, .{ 4, 0x10, 0 } };
    for (0..2) |tab| for (0..8) |mode| {
        var want: [2]u32 = undefined;
        if (tab == 1) want = t1[mode] else {
            var found = false;
            for (t0) |r| if (r[0] == mode) {
                want = .{ r[1], r[2] };
                found = true;
            };
            if (!found) continue;
        }
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x4000;
        st.s[1] = 0x2000;
        st.s[12] = 0xC2;
        st.s[6] = 0x66;
        const w3 = (0xC0741839 & ~@as(u32, (1 << 23) | (7 << 18))) | (@as(u32, @intCast(tab)) << 23) | (@as(u32, @intCast(mode)) << 18);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B, w3 } ++ halt), 0, 8);
        try testing.expectEqual(want[0], st.s[7]);
        try testing.expectEqual(want[1], st.s[12]);
    };
}

test "x18 class 110 w3[23] = 1: integer w3[21] = 1, fractional w3[21] = 0 (719)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // As 718: a = 0x4000, b = 0x2000, index X s12 = 0xC2, field X s6 = 0x66; hi s7, lo s12.
    const int_t = [8][2]u32{ .{ 0, 0 }, .{ 0x4000, 0x2000 }, .{ 0x4000, 0x2000 }, .{ 0x40C2, 0x2066 }, .{ 8, 0 }, .{ 0xCA, 0x66 }, .{ 0xCA, 0x66 }, .{ 0xCA, 0x66 } };
    const frac_t = [8][2]u32{ .{ 0xFF4D, 0xFF9A }, .{ 0x4000, 0x2000 }, .{ 0x4000, 0x2000 }, .{ 0x4066, 0x2000 }, .{ 0x40C2, 0x2066 }, .{ 0x3F3E, 0x1F9A }, .{ 0x76, 0 }, .{ 0x76, 0 } };
    for (0..2) |form| for (0..8) |mode| {
        if (form == 0 and mode == 0) continue; // integer mode 0: open
        const want = if (form == 0) int_t[mode] else frac_t[mode];
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x4000;
        st.s[1] = 0x2000;
        st.s[12] = 0xC2;
        st.s[6] = 0x66;
        const ctl: u32 = if (form == 0) (1 << 24) | (1 << 21) else 0;
        const w3 = (0xC0741839 & ~@as(u32, (1 << 24) | (1 << 21) | (7 << 18))) | (1 << 23) | ctl | (@as(u32, @intCast(mode)) << 18);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B, w3 } ++ halt), 0, 8);
        try testing.expectEqual(want[0], st.s[7]);
        try testing.expectEqual(want[1], st.s[12] & 0xFFFF);
    };
}

test "x18 class 110 mode 3 under w3[23] = 0 is +X for index 0 too (719)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const w1: u32 = 0x80005AA0 & 0x1FFFFFFF;
    const w2: u32 = 0x0C500E2B & ~@as(u32, 1);
    for ([_]u32{ 0, 1 }) |w21| for ([_][2]u32{ .{ 6, 0x66 }, .{ 0, 0x1200 } }) |f| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x4000;
        st.s[1] = 0x2000;
        st.s[6] = 0x66;
        st.xd = 0x12;
        const w3 = (0xC0741839 & ~@as(u32, (1 << 21) | (7 << 18) | (0xF << 10))) | (w21 << 21) | (3 << 18) | (f[0] << 10);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, w1, w2, w3 } ++ halt), 0, 8);
        try testing.expectEqual(@as(u32, 0x1200), st.s[7]);
        try testing.expectEqual(f[1], st.s[12]);
    };
}

test "x18 class 110 w3[23] = 1 integer: w3[21] = 0, and mode 0 (721)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // As 719: integer P hi 8, lo 0; Xi = s12 = 0xC2, Xf = s6 = 0x66; hi s7, lo s12.
    const t = [8][2]u32{ .{ 8, 0x66 }, .{ 0x4000, 0x2000 }, .{ 0x4000, 0x2000 }, .{ 0x4000, 0x2066 }, .{ 8, 0 }, .{ 0xCA, 0x66 }, .{ 8, 0x66 }, .{ 8, 0x66 } };
    for (0..9) |k| {
        const w21: u32 = if (k == 8) 1 else 0;
        const mode: u32 = if (k == 8) 0 else @intCast(k);
        const want = if (k == 8) [2]u32{ 0x10, 0xCC } else t[k];
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = 0x4000;
        st.s[1] = 0x2000;
        st.s[12] = 0xC2;
        st.s[6] = 0x66;
        const w3 = (0xC0741839 & ~@as(u32, (1 << 21) | (7 << 18))) | (1 << 24) | (1 << 23) | (w21 << 21) | (mode << 18);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B, w3 } ++ halt), 0, 8);
        try testing.expectEqual(want[0], st.s[7]);
        try testing.expectEqual(want[1], st.s[12] & 0xFFFF);
    }
}

test "x18 class 110 w3[23] = 1 integer mode 0 lo is 2 * (P + Xf) (722)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_][3]u32{ .{ 0x123, 0x10, 0x252C }, .{ 0x123, 0x20, 0x498C }, .{ 0xC0, 0x10, 0x18CC } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.s[5] = c[0];
        st.s[1] = c[1];
        st.s[12] = 0xC2;
        st.s[6] = 0x66;
        const w3 = (0xC0741839 & ~@as(u32, 7 << 18)) | (1 << 24) | (1 << 23);
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ 0x1E200018, 0x80005AA0, 0x0C500E2B, w3 } ++ halt), 0, 8);
        try testing.expectEqual(c[2], st.s[12]);
        try testing.expectEqual(@as(u32, 0), st.s[7]);
    }
}

test "st.x's head immediate is inert with no e, R or s consumer (723)" {
    const gpa = testing.allocator;
    for ([_]u32{ 0, 1 << 9, 1 << 10, 1 << 11, 1 << 14, 1 << 18 }) |flip| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.s[k] = 0x100 + @as(u32, @intCast(k));
        state.a[4] = 0x40;
        // op[14:11] = 0, op[6:3] = 0, head[24:21] = 0: X:[A4] <- s6 and nothing else.
        _ = try emu.runFrom(gpa, &state, &.{ 0x0000246f ^ flip, 0xac500001, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(@as(u32, 0x106), state.x_mem.get(0x40).?);
        for (1..16) |k| try testing.expectEqual(0x100 + @as(u32, @intCast(k)), state.s[k]);
        for (state.r) |r| try testing.expectEqual(@as(u32, 0), r);
        for (state.e) |e| try testing.expectEqual(@as(u32, 0), e);
    }
}

test "x02.v's third slot codes 0/4/7 and plain product low half (fact 730)" {
    const gpa = testing.allocator;
    // s3 <- X:[A9], A9 + 1, s12 <- low product (w1[17] = 1: P, not 2P), s6 <- code(a = s4, b = s9).
    for ([_]struct { code: u32, n: u32 = 0, a: u32, b: u32, want: u32 }{
        .{ .code = 0, .a = 1, .b = 0x102, .want = 0x103 },
        .{ .code = 4, .n = 2, .a = 1, .b = 0x102, .want = 0x41 },
        .{ .code = 7, .a = 1, .b = 0x103, .want = 0x82 },
        .{ .code = 7, .a = 1, .b = 0xFFFEFD, .want = 0xFFFF7F }, // -0x103 -> -129
        .{ .code = 7, .a = 1, .b = 0xFFFFFD, .want = 0xFFFFFF }, // -3 -> -1
        .{ .code = 7, .a = 0xFFFFFF, .b = 0x103, .want = 0x206 },
        .{ .code = 7, .a = 0xFFFFFF, .b = 0xFFFEFD, .want = 0xFFFDFA }, // -518
        .{ .code = 7, .a = 0xFFFFFF, .b = 0xFFFFFD, .want = 0xFFFFFA }, // -6
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x60;
        try state.x_mem.put(gpa, 0x60, 0x5000);
        state.s[1] = 0x300;
        state.s[2] = 0x11;
        state.s[4] = k.a;
        state.s[9] = k.b;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002 | k.code << 5, 0x0002A000, 0x00120042, 0x00000009, 0xA5122E18, 0x00004201 | k.n << 2, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.s[6]);
        try testing.expectEqual(@as(u32, 0x5000), state.s[3]);
        try testing.expectEqual(@as(u32, 0x61), state.a[9]);
        try testing.expectEqual(@as(u32, 0x3300), state.s[12]);
    }
}

test "x02.v with the X store: the store lands at the old A, the read slot takes xd (fact 730)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.a[9] = 0x60;
    try state.x_mem.put(gpa, 0x60, 0x5000);
    state.xd = 0x3000;
    state.s[7] = 0x1357;
    _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0x0002A000 | 7 << 29, 0x00120042, 0x00000009, 0xA5122E18, 0x00004201 | 3 << 9, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(?u32, 0x1357), state.x_mem.get(0x60));
    try testing.expectEqual(@as(u32, 0x3000), state.s[3]);
    try testing.expectEqual(@as(u32, 0x61), state.a[9]);
}

test "x02.vc assembles x02.v with the third slot's head code" {
    const w = try asm_mod.assembleOne("x02.vc s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #2, #0, s4, s9, #0, #0, #7, #0, R0");
    try testing.expectEqual(@as(u32, 0x00000002 | 7 << 5), w.slice()[0]);
    const v = try asm_mod.assembleOne("x02.v s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #2, #0, s4, s9, #0, #0");
    try testing.expectEqualSlices(u32, v.slice()[1..], w.slice()[1..]);
}

test "x02.vs stores at the old A while A steps; the read slot yields xd (fact 730)" {
    const gpa = testing.allocator;
    const w = try asm_mod.assembleOne("x02.vs s3, A9, A9, A0, A9, s12, s1, s2, s0, #5, #2, #0, s4, s9, #0, #0, #7, #0, #0, R0");
    var prog: [9]u32 = undefined;
    @memcpy(prog[0..6], w.slice()[0..6]);
    prog[6] = 0x00000017;
    prog[7] = 0x31308000;
    prog[8] = 0xA0000000;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.a[9] = 0x60;
    try state.x_mem.put(gpa, 0x60, 0x5000);
    state.xd = 0x3000;
    state.s[7] = 0x1357;
    _ = try emu.runFrom(gpa, &state, &prog, 0, 16);
    try testing.expectEqual(@as(?u32, 0x1357), state.x_mem.get(0x60));
    try testing.expectEqual(@as(u32, 0x3000), state.s[3]);
    try testing.expectEqual(@as(u32, 0x61), state.a[9]);
}

test "x02 w2-slot modes 1 / 2 are x +/- (xd << 8) at 24 bits; w1[17] keeps the sign (facts 733, 756)" {
    const gpa = testing.allocator;
    // s3 <- X:[A9], A9 + 1, s10 <- mode(x = s4, X = xd << 8), s6 <- s4 + s9; xd = v from the read before.
    for ([_]struct { mode: u32, w17: u32 = 0, v: u32, want: ?u32 }{
        .{ .mode = 1, .v = 0x1234, .want = 0x123900 },
        .{ .mode = 1, .v = 0xFFF234, .want = 0xF23900 },
        .{ .mode = 2, .v = 0x1234, .want = 0xEDD100 },
        .{ .mode = 2, .v = 0xFFF234, .want = 0x0DD100 },
        // 756: w1[17] = 1 adds sx16(xd[15:0]) << 8 >> 24 (mode 2: of its negation).
        .{ .mode = 1, .w17 = 1, .v = 0x001234, .want = 0x500 },
        .{ .mode = 2, .w17 = 1, .v = 0x001234, .want = 0x4FF },
        .{ .mode = 1, .w17 = 1, .v = 0x008000, .want = 0x4FF },
        .{ .mode = 2, .w17 = 1, .v = 0x008000, .want = 0x500 },
        .{ .mode = 1, .w17 = 1, .v = 0x010000, .want = 0x500 },
        .{ .mode = 2, .w17 = 1, .v = 0x010000, .want = 0x500 },
        .{ .mode = 1, .w17 = 1, .v = 0x400000, .want = 0x500 },
        .{ .mode = 2, .w17 = 1, .v = 0x400000, .want = 0x500 },
        .{ .mode = 1, .w17 = 1, .v = 0xC00000, .want = 0x500 },
        .{ .mode = 2, .w17 = 1, .v = 0xC00000, .want = 0x500 },
        .{ .mode = 1, .w17 = 1, .v = 0x7FFF00, .want = 0x4FF },
        .{ .mode = 2, .w17 = 1, .v = 0x7FFF00, .want = 0x500 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x60;
        try state.x_mem.put(gpa, 0x60, k.v);
        try state.x_mem.put(gpa, 0x61, 0x5000);
        state.s[4] = 0x500;
        const w1: u32 = 0x00008000 | k.mode << 11 | k.w17 << 17;
        const bundle = [6]u32{ 0x00000002, w1, 0x00121448, 0x00000009, 0xA5122E18, 0x00004201 };
        // The plain read before: s9 <- X:[A9] = v (sets xd), A9 + 1 undone by the A slot target.
        const r = emu.runFrom(gpa, &state, &.{ 0x00000002, 0x00002000, 0x00120000, 0x00000009, 0xA5122E48, 0x00004200, bundle[0], bundle[1], bundle[2], bundle[3], bundle[4], bundle[5], 0x00000017, 0x31308000, 0xA0000000 }, 0, 32);
        if (k.want) |want| {
            _ = try r;
            try testing.expectEqual(want, state.s[10]);
            try testing.expectEqual((0x500 + k.v) & 0xFFFFFF, state.s[6]); // u + v beside it
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x02 mode 1 under the 11 override reads yd beside the X store (fact 734)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.yd = 0xA050;
    state.xd = 0x777;
    state.e[8] = 0x720;
    state.a[18] = 0x340;
    state.s[12] = 0xABC;
    try state.x_mem.put(gpa, 0x340, 0x999);
    try state.x_mem.put(gpa, 0x350, 0x777);
    _ = try runX02(gpa, &state, .{ 0x000001c2, 0x80000040, 0x00014001, 0x40080000, 0x00240818, 0x30024628 });
    try testing.expectEqual(@as(u32, 0xA050), state.s[3]);
    try testing.expectEqual(@as(?u32, 0xABC), state.x_mem.get(0x340));
}

test "x02 w2-slot modes 4-7 take X = xd << 8 at 24 bits (fact 735)" {
    const gpa = testing.allocator;
    // 733's setup, x = s4 = 0x4000, s2 = 0x7FFF: P = 0x40 (the warm-up read's third slot off).
    for ([_]struct { mode: u32, v: u32, want: u32 }{
        .{ .mode = 4, .v = 0x1234, .want = 0x40 },
        .{ .mode = 5, .v = 0x1234, .want = 0x123440 },
        .{ .mode = 6, .v = 0x1234, .want = 0xEDCC40 },
        .{ .mode = 7, .v = 0x1234, .want = 0x1233C0 },
        .{ .mode = 4, .v = 0xFFF234, .want = 0x40 },
        .{ .mode = 5, .v = 0xFFF234, .want = 0xF23440 },
        .{ .mode = 6, .v = 0xFFF234, .want = 0x0DCC40 },
        .{ .mode = 7, .v = 0xFFF234, .want = 0xF233C0 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x60;
        try state.x_mem.put(gpa, 0x60, k.v);
        try state.x_mem.put(gpa, 0x61, 0x5000);
        state.s[4] = 0x4000;
        state.s[2] = 0x7FFF;
        const w1: u32 = 0x00008000 | k.mode << 11;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0x00002000, 0x00120000, 0x00000009, 0x25122E48, 0x00004200, 0x00000002, w1, 0x00121448, 0x00000009, 0xA5122E18, 0x00004201, 0x00000017, 0x31308000, 0xA0000000 }, 0, 32);
        try testing.expectEqual(k.want, state.s[10]);
    }
}

test "x02's w5[10:9] gates the mode-5 read: only 01 reads X memory (fact 739)" {
    const gpa = testing.allocator;
    for ([_]struct { g: u32, want: u32 }{
        .{ .g = 1, .want = 0x999 },
        .{ .g = 0, .want = 0x777 },
        .{ .g = 2, .want = 0x777 },
        .{ .g = 3, .want = 0x777 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x60;
        try state.x_mem.put(gpa, 0x60, 0x999);
        state.xd = 0x777;
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0x0002A000, 0x00120042, 0x00000009, 0xA5122E18, 0x00004001 | k.g << 9, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.s[3]);
        try testing.expectEqual(@as(u32, 0x61), state.a[9]);
    }
}

test "x02's A slot steps by agu_xr fns in the full bundle (fact 738)" {
    const gpa = testing.allocator;
    // 730's bundle (read + multiply + third slot) with fn w2[26:22], R<w5[25:22]> = R5.
    for ([_]struct { fn_op: u32, r: u32 = 0, want: u32 }{
        .{ .fn_op = 0, .want = 0x61 },
        .{ .fn_op = 1, .want = 0x5F },
        .{ .fn_op = 2, .r = 0xFFFE, .want = 0x5E },
        .{ .fn_op = 2, .r = 0xFFFF, .want = 0x5F },
        .{ .fn_op = 3, .r = 2, .want = 0x5E },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[9] = 0x60;
        state.r[5] = k.r;
        try state.x_mem.put(gpa, 0x60, 0x5000);
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000002, 0x0002A000, 0x00120042 | k.fn_op << 22, 0x00000009, 0xA5122E18, 0x00004201 | 5 << 22, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.a[9]);
        try testing.expectEqual(@as(u32, 0x5000), state.s[3]);
    }
}

test "x03.tap's A step takes agu_xr fns 0-3 (fact 740)" {
    const gpa = testing.allocator;
    // Two taps from A5 = 0x60, X:[a] = a: the second tap's load s3 shows where A5 landed.
    for ([_]struct { fn_op: u32, r: u32 = 0, want: u32 }{
        .{ .fn_op = 0, .want = 0x61 },
        .{ .fn_op = 1, .want = 0x5F },
        .{ .fn_op = 2, .r = 0xFFFE, .want = 0x5E },
        .{ .fn_op = 2, .r = 0xFFFF, .want = 0x5F },
        .{ .fn_op = 3, .r = 2, .want = 0x5E },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[5] = 0x60;
        state.r[6] = k.r;
        for (0x5E..0x62) |a| try state.x_mem.put(gpa, @intCast(a), @intCast(a));
        const tap = [6]u32{ 0x40000003, 0x80006801, 0x000A18E7 | k.fn_op << 22, 0x00000005, 0x00002D98, 0x00041602 | 6 << 22 };
        _ = try emu.runFrom(gpa, &state, &(tap ++ tap ++ [3]u32{ 0x00000017, 0x31308000, 0xA0000000 }), 0, 32);
        try testing.expectEqual(k.want, state.s[3]);
    }
}

test "x03.rep's store pointer takes agu_xr fns 0-3 (fact 741)" {
    const gpa = testing.allocator;
    // own-x03rep's program with fn w2[26:22] and R<w5[25:22]> = R6.
    for ([_]struct { fn_op: u32, r: u32 = 0, second: u32 }{
        .{ .fn_op = 0, .second = 0x301 },
        .{ .fn_op = 1, .second = 0x2FF },
        .{ .fn_op = 2, .r = 0xFFFE, .second = 0x2FE },
        .{ .fn_op = 3, .r = 2, .second = 0x2FE },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[3] = 1;
        state.s[3] = 0x3303;
        state.a[8] = 0x300;
        state.r[6] = k.r;
        const words = switch (try asm_mod.assembleFile(gpa, "x03.rep p3\nalu s0, s0, s0, #0\nalu s0, s0, s0, #0\nx17.halt\n")) {
            .ok => |w| w,
            .err => return error.AssembleFailed,
        };
        defer gpa.free(words);
        words[2] |= k.fn_op << 22;
        words[5] |= 6 << 22;
        _ = try emu.runLinear(gpa, &state, words);
        try testing.expectEqual(@as(?u32, 0x3303), state.x_mem.get(0x300));
        try testing.expectEqual(@as(?u32, 0x3303), state.x_mem.get(k.second));
    }
}

test "x03 modes 7 / 6 on the 48-bit acc: sat48(X - 2cd) / sat48(2cd - X) (fact 737)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    // 701's x03 row: a s2, b s4, hi s10, lo s9.  Mode 4 (2ab), reload, then mode 7 or 6 (2cd).
    const row = [_]u32{ 0x80000003, 0x40006800, 0x01441485, 0x00000007, 0x00002c80, 0x00041202 };
    const cases = [_]struct { a: i64, b: i64, c: i64, d: i64 }{
        .{ .a = 0x123456, .b = 0x0ABCDE, .c = 0x654321, .d = 0x00FEDC },
        .{ .a = 0x100, .b = 0x200, .c = 0x300, .d = 0x400 },
        .{ .a = -0x800000, .b = 0x7FFFFF, .c = 0x7FFFFF, .d = 0x7FFFFF }, // saturates low on 7
        .{ .a = 0x7FFFFF, .b = 0x7FFFFF, .c = -0x800000, .d = 0x7FFFFF }, // saturates high on 7
    };
    for (cases) |c| {
        for ([_]u32{ 7, 6 }) |mode| {
            var st = emu.State{};
            defer st.deinit(testing.allocator);
            var prog: [9]u32 = undefined;
            @memcpy(prog[0..6], &row);
            prog[1] = 0x40006000; // mode 4
            @memcpy(prog[6..9], &halt);
            st.s[2] = @as(u32, @truncate(@as(u64, @bitCast(c.a)))) & 0xFFFFFF;
            st.s[4] = @as(u32, @truncate(@as(u64, @bitCast(c.b)))) & 0xFFFFFF;
            _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
            prog[1] = 0x40006000 | mode << 11;
            st.s[2] = @as(u32, @truncate(@as(u64, @bitCast(c.c)))) & 0xFFFFFF;
            st.s[4] = @as(u32, @truncate(@as(u64, @bitCast(c.d)))) & 0xFFFFFF;
            _ = try emu.runFrom(testing.allocator, &st, &prog, 0, 8);
            const lo48 = -(@as(i64, 1) << 47);
            const hi48 = (@as(i64, 1) << 47) - 1;
            const x = std.math.clamp(2 * c.a * c.b, lo48, hi48);
            const p = 2 * c.c * c.d;
            const want = std.math.clamp(if (mode == 7) x - p else p - x, lo48, hi48);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(want >> 24)) & 0xFFFFFF)), st.s[10]);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(want)) & 0xFFFFFF)), st.s[9]);
        }
    }
}

test "dld does both loads in one word, with no pointer step (fact 742)" {
    const gpa = testing.allocator;
    for ([_]struct { w: u32, s2: u32, s11: u32 }{
        .{ .w = 0x40918009, .s2 = 0x3333, .s11 = 0x1111 },
        .{ .w = 0x6C0020C9, .s2 = 0x2222, .s11 = 0x0BCD },
        .{ .w = 0x6C91A0C9, .s2 = 0x3333, .s11 = 0x0BCD },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[3] = 0x330;
        state.e[6] = 0x720;
        state.s[2] = 0x2222;
        state.s[11] = 0x1111;
        try state.x_mem.put(gpa, 0x330, 0x3333);
        try state.y_mem.put(gpa, 0x720, 0x0BCD);
        _ = try emu.runFrom(gpa, &state, &.{ k.w, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
        try testing.expectEqual(k.s2, state.s[2]);
        try testing.expectEqual(k.s11, state.s[11]);
        try testing.expectEqual(@as(u32, 0x330), state.a[3]);
        try testing.expectEqual(@as(u32, 0x720), state.e[6]);
    }
}

test "x18 class 011's p write: cond 11 v <= 0, 13 v == 0, 0 true; w3[21] inert (fact 743)" {
    const gpa = testing.allocator;
    // 16000018 00000000 0000800C 60000101: p2 <- cond head[28:25] of op w1[17:15](a = s3).
    const vs = [_]u32{ 1, 0xFFFFFF, 0, 0x4000 };
    for ([_]struct { cond: u32, want: [4]u8 }{
        .{ .cond = 11, .want = .{ 0, 1, 1, 0 } },
        .{ .cond = 13, .want = .{ 0, 0, 1, 0 } },
        .{ .cond = 0, .want = .{ 1, 1, 1, 1 } },
    }) |k| {
        for ([_]u32{ 0, 2 }) |op| {
            for ([_]u32{ 0, 1 }) |w21| {
                for (vs, k.want) |v, want| {
                    var state = emu.State{};
                    defer state.deinit(gpa);
                    state.s[3] = v;
                    state.p[2] = 1 - want;
                    const h: u32 = (0x16000018 & ~@as(u32, 0xF << 25)) | k.cond << 25;
                    _ = try emu.runFrom(gpa, &state, &.{ h, op << 15, 0x0000800C, 0x60000101 | w21 << 21, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
                    try testing.expectEqual(want, state.p[2]);
                }
            }
        }
    }
}

test "the A-step operand forms set fn w2[26:22] and R<w5[25:22]> over their bases (738, 740, 741)" {
    for ([_][2][]const u8{
        .{ "x02.vc s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #2, #0, s4, s9, #0, #0, #7, #2, R6", "x02.vc s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #2, #0, s4, s9, #0, #0, #7, #0, R0" },
        .{ "x02.vs s3, A9, A9, A0, A9, s12, s1, s2, s0, #5, #2, #0, s4, s9, #0, #0, #7, #0, #2, R6", "x02.vs s3, A9, A9, A0, A9, s12, s1, s2, s0, #5, #2, #0, s4, s9, #0, #0, #7, #0, #0, R0" },
        .{ "x03.tapa s3, #5, s7, s12, s11, #2, R6", "x03.tap s3, #5, s7, s12, s11" },
        .{ "x03.tap0a s3, #5, s7, s12, s11, #2, R6", "x03.tap0 s3, #5, s7, s12, s11" },
        .{ "x03.repa p3, #2, R6", "x03.rep p3" },
    }) |k| {
        const w = try asm_mod.assembleOne(k[0]);
        const base = try asm_mod.assembleOne(k[1]);
        const ws = w.slice();
        const bs = base.slice();
        try testing.expectEqual(bs[2] | 2 << 22, ws[2]);
        try testing.expectEqual(bs[5] | 6 << 22, ws[5]);
        for ([_]usize{ 0, 1, 3, 4 }) |i| try testing.expectEqual(bs[i], ws[i]);
    }
}

test "x18 class-101 branch: w1[18] is only the offset's bit 0 (fact 744)" {
    const gpa = testing.allocator;
    const add = try asm_mod.assembleOne("alu s1, s1, s2, #0");
    // Branch on p3 at q = 0, then eight 1-word adds (two of them delay slots), then the halt.
    for ([_]struct { w1: u32, p3: u8, want: u32 }{
        .{ .w1 = 0x00200000, .p3 = 1, .want = 6 }, // +8
        .{ .w1 = 0x00240000, .p3 = 1, .want = 5 }, // +9: one add fewer
        .{ .w1 = 0x00240000, .p3 = 0, .want = 8 }, // not taken
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[3] = k.p3;
        state.s[2] = 1;
        var prog: [15]u32 = undefined;
        @memcpy(prog[0..4], &[4]u32{ 0x00000018, k.w1, 0x80000000, 0xA0000001 });
        for (4..12) |i| prog[i] = add.slice()[0];
        @memcpy(prog[12..15], &[3]u32{ 0x00000017, 0x31308000, 0xA0000000 });
        _ = try emu.runFrom(gpa, &state, &prog, 0, 32);
        try testing.expectEqual(k.want, state.s[1]);
    }
}

test "x18 class-001 multiply rounds on w3[0]; w1[26] = 0 refused (facts 745, 747)" {
    const gpa = testing.allocator;
    // 04000418 04000000 40B88000 2000000x: s8 <- s8 * s11 (mode P), product / 2^23 = 64.63.
    for ([_]struct { w1: u32 = 0x04000000, w3lo: u32, want: ?u32 }{
        .{ .w3lo = 1, .want = 0x41 },
        .{ .w3lo = 0, .want = 0x40 },
        .{ .w3lo = 5, .want = 0x20 },
        .{ .w3lo = 4, .want = 0xFF4333 }, // 747: unsigned x unsigned >> 24
        .{ .w1 = 0x00000000, .w3lo = 1, .want = null }, // w1[26] = 0 wedges
        .{ .w1 = 0x08000000, .w3lo = 1, .want = null },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[8] = 0xFF8808; // -0x77F8
        state.s[11] = 0xFFBB0B; // -0x44F5
        const r = emu.runFrom(gpa, &state, &.{ 0x04000418, k.w1, 0x40B88000, 0x20000000 | k.w3lo, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
        if (k.want) |v| {
            _ = try r;
            try testing.expectEqual(v, state.s[8]);
        } else try testing.expectError(error.UnknownForm, r);
    }
}

test "x18 class-001 multiply reads signed 24-bit operands (fact 746)" {
    const gpa = testing.allocator;
    for ([_][3]u32{
        .{ 0x00C000, 0x004000, 0x000060 },
        .{ 0x004000, 0x00C000, 0x000060 },
        .{ 0x010000, 0x004000, 0x000080 },
        .{ 0x004000, 0x004000, 0x000020 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[8] = k[0];
        state.s[11] = k[1];
        _ = try emu.runFrom(gpa, &state, &.{ 0x04000418, 0x04000000, 0x40B88000, 0x20000001, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
        try testing.expectEqual(k[2], state.s[8]);
    }
}

test "x18 class-001 multiply w3[2:0] = 100 is unsigned 24 x unsigned 24 >> 24, truncated (fact 747)" {
    const gpa = testing.allocator;
    for ([_][3]u32{
        .{ 0x004000, 0xFFC000, 0x003FF0 },
        .{ 0xFF8808, 0xFFBB0B, 0xFF4333 },
        .{ 0x007000, 0x007000, 0x000031 },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[8] = k[0];
        state.s[11] = k[1];
        _ = try emu.runFrom(gpa, &state, &.{ 0x04000418, 0x04000000, 0x40B88000, 0x20000004, 0x00000017, 0x31308000, 0xA0000000 }, 0, 8);
        try testing.expectEqual(k[2], state.s[8]);
    }
}

test "x03.acp's template read slot also loads s7 and yd <- Y:[e<y_e>] under #0 and #1 (748)" {
    const gpa = testing.allocator;
    for ([_][]const u8{ "x03.acp A9, A5, e3, #0", "x03.acp A9, A5, e3, #1" }, [_]u32{ 0x100, 0x0BCD }) |src, a9| {
        const w = try asm_mod.assembleOne(src);
        var prog: [9]u32 = undefined;
        @memcpy(prog[0..6], w.slice()[0..6]);
        @memcpy(prog[6..9], &[3]u32{ 0x00000017, 0x31308000, 0xA0000000 });
        var state = emu.State{};
        defer state.deinit(gpa);
        state.a[5] = 0x100;
        state.e[3] = 0x720;
        state.s[7] = 0x5555;
        state.yd = 0xA050;
        try state.y_mem.put(gpa, 0x720, 0x0BCD);
        _ = try emu.runFrom(gpa, &state, &prog, 0, 8);
        try testing.expectEqual(a9, state.a[9]);
        try testing.expectEqual(@as(u32, 0x0BCD), state.s[7]);
        try testing.expectEqual(@as(u32, 0x0BCD), state.yd);
    }
}

test "0x00's codes 0x14 / 0x23 pick the adder by n's condition on p4, all 16 n (facts 749, 751)" {
    const gpa = testing.allocator;
    for (0..32) |nk| {
        const n: u32 = @intCast(nk & 15);
        const p4 = nk >= 16;
        const adds = switch (n) {
            0, 1, 3 => !p4,
            2, 5, 7 => p4,
            6, 9 => true,
            else => false,
        };
        for ([_]u32{ 0, 1 }) |m| {
            for ([_]u32{ 0x14, 0x23 }) |code| {
                var state = emu.State{};
                defer state.deinit(gpa);
                state.p[4] = @intFromBool(p4);
                state.s[1] = 0x130;
                state.s[3] = 2;
                try runX00Slot(&state, code, 3, m, n);
                const want: u32 = if (adds) (if (m == 0) 0x132 else 0x12E) else if (code == 0x23) 2 else if (m == 0) 0xFFFECC else 0xFFFED2;
                try testing.expectEqual(want, state.s[2]);
            }
        }
    }
}

test "0x00's code 0x14 over every m: x + y + c on the adder n, ~x + ~y + c otherwise (fact 750)" {
    const gpa = testing.allocator;
    const b: i64 = 0x130;
    for ([_]i64{ 2, -0x500 }) |a| {
        for (0..8) |mm| {
            const m: u32 = @intCast(mm);
            // second result by m (750); the adder side is ADDER_M[m](a, b)'s sum
            const second: i64 = switch (m) {
                0 => -(a + b) - 2,
                1 => a - b,
                2 => -b - 2,
                3, 4 => -b - 1,
                5 => b - a,
                6 => b,
                else => -b - 2, // -|b| - 2, b > 0
            };
            const first: i64 = switch (m) {
                0 => a + b,
                1 => b - a,
                2 => b,
                3 => b + 1,
                4 => b - 1,
                5 => a - b,
                6 => -b,
                else => b,
            };
            for ([_]u32{ 0, 2 }) |n| {
                var state = emu.State{};
                defer state.deinit(gpa);
                state.s[1] = 0x130;
                state.s[3] = @as(u32, @truncate(@as(u64, @bitCast(a)))) & 0xFFFFFF;
                try runX00Slot(&state, 0x14, 3, m, n);
                const want = if (n == 0) first else second;
                try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(want)))) & 0xFFFFFF, state.s[2]);
            }
        }
    }
}

test "x18 class 011's S codes read 646's condition on p3 (fact 752)" {
    const gpa = testing.allocator;
    // own-x18b's shape, op 0, a = s5 = 5, b = s7 = 3: ALU2 a + b = 8, else 0x10 table B b - a, 0x23 b.
    for (0..32) |nk| {
        const n: u32 = @intCast(nk & 15);
        const p3 = nk >= 16;
        const alu = switch (n) {
            0, 1, 3 => !p3,
            2, 5, 7 => p3,
            6, 9 => true,
            else => false,
        };
        for ([_]u32{ 0x10, 0x23 }) |code| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.p[3] = @intFromBool(p3);
            state.s[5] = 5;
            state.s[7] = 3;
            _ = try emu.runFrom(gpa, &state, &.{ x18c3Head(code), 0x00001970, 0x30000014 | n << 6, 0x74000149, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
            const want: u32 = if (alu) 8 else if (code == 0x10) 0xFFFFFE else 3;
            try testing.expectEqual(want, state.s[9]);
        }
    }
}

test "0x00's sel ignores n and p4 and takes every m; asr takes every m (facts 753, 754)" {
    const gpa = testing.allocator;
    for ([_]i64{ 2, -0x500 }) |a| for (0..8) |mm| for ([_]u32{ 0, 2, 4, 6 }) |n| for ([_]u8{ 0, 1 }) |p4| {
        const m: u32 = @intCast(mm);
        const b: i64 = 0x130;
        const sum: i64 = switch (m) {
            0 => a + b,
            1 => b - a,
            2 => b,
            3 => b + 1,
            4 => b - 1,
            5 => a - b,
            6 => -b,
            else => b,
        };
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[4] = p4;
        state.s[1] = 0x130;
        state.s[3] = @as(u32, @truncate(@as(u64, @bitCast(a)))) & 0xFFFFFF;
        try runX00Slot(&state, 0x26, 3, m, n);
        const want: i64 = if (sum < 0) b else a;
        try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(want)))) & 0xFFFFFF, state.s[2]);
    };
    // 754: i = 3, b = 0x100, count 1: n 3 >> 1, n 11 << 1.
    for ([_][3]u32{ .{ 3, 0x80, 0x202 }, .{ 4, 0x7F, 0x1FE }, .{ 5, 0xFFFF81, 0xFFFE06 }, .{ 6, 0xFFFF80, 0xFFFE00 }, .{ 7, 0x80, 0x200 } }) |k| {
        for ([_]u32{ 3, 11 }, [_]u32{ k[1], k[2] }) |n, want| {
            var state = emu.State{};
            defer state.deinit(gpa);
            state.s[1] = 0x100;
            state.s[3] = 1;
            try runX00Slot(&state, 0x06, 3, k[0], n);
            try testing.expectEqual(want, state.s[2]);
        }
    }
}

test "0x00's MAC low half reads xd << 8 when w4[15:13] = 0 (fact 755)" {
    const gpa = testing.allocator;
    // mode 7 (X - 2P): s6 = 0x100, s3 = 0x4000, s5 = 0x2345, dest s4.
    for ([_][2]u32{ .{ 0x3333, 0xB300 }, .{ 0x0012, 0x9200 }, .{ 0x008A, 0x0A00 } }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[6] = 0x100;
        state.s[3] = 0x4000;
        state.s[5] = 0x2345;
        state.xd = k[0];
        _ = try emu.runFrom(gpa, &state, &.{ 0x00000000, 0x0000F828, 0x00001800, 0x00000000, 0x00100000, 0x06000000, 0x53090400 }, 0, 1);
        try testing.expectEqual(k[1], state.s[4]);
    }
}

test "st.y with a class-011 second writes e and the 326 mux, no store (fact 757)" {
    const gpa = testing.allocator;
    // Own words: head 4002468F (e2 <- #0x1234, s0), w1[28:1] inert with w1[16:15] = 00.
    for ([_]u32{ 0x6A4C2468, 0x75B20900, 0x60000000, 0x7FFE7FFE }) |w1| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try emu.executeBundle(gpa, &state, &.{ 0x4002468F, w1 & ~@as(u32, 0x00018001) });
        try testing.expectEqual(@as(u32, 0x1234), state.e[2]);
        try testing.expectEqual(@as(u32, 0), state.y_mem.count());
    }
    // s<head[24:21]> by w1[2:1]: 0, R<head[28:25]>, e<w1[8:5]>, 0.
    for ([_][2]u32{ .{ 0, 0 }, .{ 1, 0x777 }, .{ 2, 0x555 }, .{ 3, 0 } }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x999;
        state.r[3] = 0x777;
        state.e[4] = 0x555;
        try emu.executeBundle(gpa, &state, &.{ 0x4002468F | 3 << 25 | 5 << 21, 0x60000000 | 4 << 5 | k[0] << 1 });
        try testing.expectEqual(k[1], state.s[5]);
    }
    // Store forms (w1[16:15] != 00) and index 0 are unmeasured.
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x4002468F, 0x60008000 }));
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x0002468F, 0x60000000 }));
}

test "0x00 control 6 / 7 ends the program after two delay slots (fact 759)" {
    const gpa = testing.allocator;
    const lde = struct {
        fn f(k: u32, v: u32) u32 {
            return 0x40000006 | k << 26 | v << 5;
        }
    }.f;
    for ([_]u32{ 6, 7 }) |ctl| for ([_][2]u32{ .{ 0, 0 }, .{ 0x1068, 1 }, .{ 0x40, 0 } }) |t| {
        var state = emu.State{};
        defer state.deinit(gpa);
        const n = try emu.runFrom(gpa, &state, &.{ 0x00000000, t[0] << 18, t[1], ctl, 0, 0, 0, lde(1, 0x11), lde(2, 0x22), lde(3, 0x33) }, 0, 16);
        try testing.expectEqual(@as(usize, 3), n);
        try testing.expect(state.halted);
        try testing.expectEqual([3]u32{ 0x11, 0x22, 0 }, state.e[1..4].*);
    };
    // 0x01's control 6 is unmeasured.
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x00000001, 0, 0, 6, 0, 0, 0 }));
}

test "host arms: 0x08 / 0x0D park the next arm once; other controls enter q0 (facts 315, 768, 769)" {
    const gpa = testing.allocator;
    // END (control 6) then two nop slots: no control-0 bundle, so the next entry is q0.
    const halt = [_]u32{ 0x00000000, 0, 0, 6, 0, 0, 0, 0x40000005, 0x40000005 };
    for ([_]u32{ 8, 0xD }) |ctl| {
        var state = emu.State{};
        defer state.deinit(gpa);
        // s4 <- #1 (mux 6) beside the control; the run goes on to its END.
        const prog = [_]u32{ 0x00000800, 0, 0, ctl, 0, 0, 0x0000400C } ++ halt;
        _ = try emu.arm(gpa, &state, &prog, .pass1, 16);
        try testing.expect(state.halted);
        try testing.expectEqual(@as(u32, 1), state.s[4]);
        try testing.expectError(error.Parked, emu.arm(gpa, &state, &halt, .pass1, 16));
        _ = try emu.arm(gpa, &state, &halt, .pass1, 16);
        try testing.expect(state.halted);
    }
    // 769: control 0 / 0x10 / 0x18 leave the next arm entering at q0.
    for ([_]u32{ 0, 0x10, 0x18 }) |ctl| {
        var state = emu.State{};
        defer state.deinit(gpa);
        const prog = [_]u32{ 0x00000800, 0, 0, ctl, 0, 0, 0x0000400C } ++ halt;
        _ = try emu.arm(gpa, &state, &prog, .pass1, 16);
        _ = try emu.arm(gpa, &state, &prog, .pass1, 16);
        try testing.expect(state.halted);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    // 230: a bare 0x08 in an empty bundle wedged.
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0, 0, 0, 8, 0, 0, 0 }));
}

test "the decode arm enters at the host's +0x988 syllable; pass-1 at q0 (fact 770)" {
    const gpa = testing.allocator;
    var prog: [0x120]u32 = @splat(0x40000005);
    // q0: e1 <- #1 (st.y-free lde), END; q0x100: e2 <- #2, END.
    const blk = [_]u32{ 0x00000000, 0, 0, 6, 0, 0, 0, 0x40000005, 0x40000005 };
    prog[0] = 0x40000006 | 1 << 26 | 1 << 5;
    @memcpy(prog[1..10], &blk);
    prog[0x100] = 0x40000006 | 2 << 26 | 2 << 5;
    @memcpy(prog[0x101..0x10A], &blk);
    var state = emu.State{};
    defer state.deinit(gpa);
    _ = try emu.arm(gpa, &state, &prog, .pass1, 16);
    try testing.expectEqual([2]u32{ 1, 0 }, state.e[1..3].*);
    _ = try emu.arm(gpa, &state, &prog, .{ .decode = 0x100 }, 16);
    try testing.expectEqual([2]u32{ 1, 2 }, state.e[1..3].*);
}

test "0x00 multiply mode 0 without w4[22] writes 0 and wins over the MAC slot (fact 762)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[2] = 0x40;
    state.s[3] = 0x300;
    state.s[4] = 0x20;
    state.a[9] = 0x10;
    try state.x_mem.put(gpa, 0x10, 0x777);
    // multiply dest s3 (s4 * s2, A9 in w2[22:18]), mode 0, w1[17:14] = 0.
    try emu.executeBundle(gpa, &state, &.{ 0, 0, 9 << 18, 0, 0, 0, 0x24030000 });
    try testing.expectEqual(@as(u32, 0), state.s[3]);
    // Both slots name s4: the multiply slot's X word lands (w4[22], w4[18] = 0), the MAC write is lost.
    try emu.executeBundle(gpa, &state, &.{ 0, 0, 9 << 18, 0, 1 << 22, 0, 0x24040400 });
    try testing.expectEqual(@as(u32, 0x777), state.s[4]);
}

test "st.y with a class-001 store second stores s<w[24:21]> at the old e, w[25] inert (fact 765)" {
    const gpa = testing.allocator;
    for ([_][2]u32{ .{ 1, 0x111 }, .{ 2, 0x222 }, .{ 3, 0x333 }, .{ 17, 0x111 }, .{ 18, 0x222 } }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = 0x111;
        state.s[2] = 0x222;
        state.s[3] = 0x333;
        state.e[1] = 0x700;
        state.v = @splat(0x999);
        // head: e1 <- #0x1090; second: [16:15] = 11, Y:[e1] <- s<src>.
        try emu.executeBundle(gpa, &state, &.{ 0x2002120F, 0x20038000 | k[0] << 21 });
        try testing.expectEqual(@as(?u32, k[1]), state.getY(0x700));
        try testing.expectEqual(@as(?u32, null), state.getY(0x1090));
        try testing.expectEqual(@as(u32, 0x1090), state.e[1]);
    }
}

test "ealu10 op 4 copies a into e; s<w[10:7]> <- #sx16(head[26:11]) (facts 766, 767, 875)" {
    const gpa = testing.allocator;
    for ([_]struct { mask: u32, dest: ?u32 }{
        .{ .mask = 1, .dest = 1 },
        .{ .mask = 2, .dest = 2 },
        .{ .mask = 4, .dest = 4 },
        .{ .mask = 8, .dest = 8 },
        .{ .mask = 9, .dest = null }, // s9: none of s1 / s2 / s4 / s8 (487); an index (875)
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[5] = 0x505;
        for ([_]u32{ 1, 2, 4, 8 }) |r| state.s[r] = 0x77;
        // a = s5, e4, op 4, w[28:22] = 1 and w[2:0] = 7 inert; head[26:11] = 0xADE1.
        try emu.executeBundle(gpa, &state, &.{ 0xFD6F0810, 0xC0404027 | 5 << 18 | k.mask << 7 });
        try testing.expectEqual(@as(u32, 0x505), state.e[4]);
        for ([_]u32{ 1, 2, 4, 8 }) |r|
            try testing.expectEqual(if (k.dest == r) @as(u32, 0xFFADE1) else 0x77, state.s[r]);
    }
}

test "0x00 jump control 3 is taken iff p<w4[31:28]>, code 0 always (fact 772)" {
    const gpa = testing.allocator;
    // control 3 at q0, offset +32, two nop slots; the fall-through ENDs (q9) before q32's e1 write.
    var prog: [40]u32 = @splat(0x40000005);
    @memcpy(prog[0..7], &[_]u32{ 0, 32 << 18, 0, 3, 0, 0, 0 });
    @memcpy(prog[9..16], &[_]u32{ 0, 0, 0, 6, 0, 0, 0 });
    prog[32] = 0x40000006 | 1 << 26 | 0x77 << 5;
    for (0..16) |c| for ([_]u8{ 0, 1 }) |pv| {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.p[k] = 1 - pv;
        if (c != 0) state.p[c] = pv;
        prog[4] = @as(u32, @intCast(c)) << 28;
        _ = try emu.runFrom(gpa, &state, &prog, 0, 16);
        const taken = c == 0 or pv == 1;
        try testing.expectEqual(@as(u32, if (taken) 0x77 else 0), state.e[1]);
    };
    // Other jump controls with a nonzero condition stay unmeasured.
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0, 16 << 18, 0, 0xE, 0xD0000000, 0, 0 }));
}

test "alu head shifts asr / asrrs / lsr over every count (fact 774); clr 0x22 with modifiers (775)" {
    const gpa = testing.allocator;
    const Row = struct { n: i32, b: u32, asr: u32, lsr: u32, rs: u32 };
    for ([_]Row{
        .{ .n = 0, .b = 0x005A5A, .asr = 0x005A5A, .lsr = 0x005A5A, .rs = 0x005A5A },
        .{ .n = 7, .b = 0x005A5A, .asr = 0xB4, .lsr = 0xB4, .rs = 0xB5 },
        .{ .n = 12, .b = 0x005A5A, .asr = 5, .lsr = 5, .rs = 6 },
        .{ .n = 1, .b = 0xFFA5A5, .asr = 0xFFD2D2, .lsr = 0x7FD2D2, .rs = 0xFFD2D3 },
        .{ .n = 23, .b = 0xFFA5A5, .asr = 0xFFFFFF, .lsr = 1, .rs = 0 },
        .{ .n = 24, .b = 0xFFA5A5, .asr = 0xFFFFFF, .lsr = 0, .rs = 0 },
        .{ .n = 48, .b = 0xFFA5A5, .asr = 0xFFFFFF, .lsr = 0, .rs = 0 },
        .{ .n = 32, .b = 0x005A5A, .asr = 0, .lsr = 0, .rs = 0 },
        .{ .n = -8, .b = 0x005A5A, .asr = 0x5A5A00, .lsr = 0x5A5A00, .rs = 0x5A5A00 },
        .{ .n = -12, .b = 0x005A5A, .asr = 0xA5A000, .lsr = 0xA5A000, .rs = 0x7FFFFF },
        .{ .n = -12, .b = 0xFFA5A5, .asr = 0x5A5000, .lsr = 0x5A5000, .rs = 0x800000 },
        .{ .n = -23, .b = 0xFFA5A5, .asr = 0x800000, .lsr = 0x800000, .rs = 0x800000 },
        .{ .n = -24, .b = 0x005A5A, .asr = 0, .lsr = 0, .rs = 0x7FFFFF },
    }) |k| for ([_]u32{ 6, 8, 7 }, [_]u32{ k.asr, k.lsr, k.rs }) |code, want| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = @as(u32, @bitCast(k.n)) & 0xFFFFFF;
        state.s[2] = k.b;
        // dest s3, n = s1, b = s2, [25:19] = 0.
        try emu.executeBundle(gpa, &state, &.{0x40000005 | 2 << 26 | 3 << 15 | 1 << 11 | code << 5});
        try testing.expectEqual(want, state.s[3]);
    };
    // 775: clr 0x22 with m = 3, # = 2 writes 0.
    var state = emu.State{};
    defer state.deinit(gpa);
    state.s[14] = 0x123;
    state.s[12] = 0x55;
    try emu.executeBundle(gpa, &state, &.{0x41976445});
    try testing.expectEqual(@as(u32, 0), state.s[14]);
}

test "x18 class 011 codes below 0x10 always write p (fact 776)" {
    const gpa = testing.allocator;
    // code / cond head[28:25], op 0, n = 0, a = s3, b = s0, p dest p12, w3 shape 011 with w3[3].
    for ([_]struct { code: u32, v: u32, want: u8 }{
        .{ .code = 2, .v = 5, .want = 1 },
        .{ .code = 2, .v = 0xFFFFFB, .want = 0 },
        .{ .code = 8, .v = 0xFFFFFB, .want = 0 },
        .{ .code = 11, .v = 0, .want = 1 },
        .{ .code = 12, .v = 5, .want = 0 },
        .{ .code = 13, .v = 0, .want = 1 },
        .{ .code = 13, .v = 5, .want = 0 },
        .{ .code = 4, .v = 0xFFFFFB, .want = 1 },
        .{ .code = 15, .v = 0, .want = 1 },
    }) |k| for ([_]u8{ 0, 1 }) |pre| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = k.v;
        state.p[12] = pre;
        try emu.executeBundle(gpa, &state, &.{ 0x00000018 | k.code << 25, 0, 12 << 14 | 3 << 2, 0x60000009 });
        try testing.expectEqual(k.want, state.p[12]);
    };
    // 0x0E: p <- n not in {2, 5, 6, 7, 9}, operand-free (n = 0 here; other n await the value).
    var state = emu.State{};
    defer state.deinit(gpa);
    state.p[12] = 0;
    try emu.executeBundle(gpa, &state, &.{ 0x0E << 25 | 0x18, 0, 12 << 14, 0x60000009 });
    try testing.expectEqual(@as(u8, 1), state.p[12]);
}

test "x18 class 011 code 0x0E op 0 is a + b at every n (fact 777)" {
    const gpa = testing.allocator;
    for ([_][3]u32{ .{ 0, 0, 0 }, .{ 0x123, 0x10, 0x133 }, .{ 0xFFFFFF, 0x10, 0xF }, .{ 0x123, 0xFFFFF0, 0x113 } }) |k| for (0..16) |nn| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = k[0];
        state.s[5] = k[1];
        const n: u32 = @intCast(nn);
        // dest s9 (w1[11:8]), b = s5 (w1[7:4]), a = s3 (w2[5:2]), p dest p12.
        try emu.executeBundle(gpa, &state, &.{ 0x0E << 25 | 0x18, 9 << 8 | 5 << 4, 12 << 14 | n << 6 | 3 << 2, 0x60000009 });
        try testing.expectEqual(k[2], state.s[9]);
    };
}

test "0x0B head[31:30] = 10 loads Y:[e] into s, R and A (facts 258, 259, 324)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0x700;
    state.yd = 0x0BCD;
    try state.y_mem.put(gpa, 0x700, 0x1234);
    // class 101: s13 <- v, R dest 0, A13 <- v ([14:13] = 01), e1.
    try emu.executeBundle(gpa, &state, &.{0xB416A02B});
    try testing.expectEqual(@as(u32, 0x1234), state.s[13]);
    try testing.expectEqual(@as(u32, 0x1234), state.a[13]);
    // class 100 with R5 and A4 from yd ([14:13] = 00); [20] = 0 wedges.
    try emu.executeBundle(gpa, &state, &.{0x8000000B | 3 << 26 | 5 << 22 | 1 << 20 | 4 << 15 | 1 << 5});
    try testing.expectEqual([3]u32{ 0x1234, 0x1234, 0x0BCD }, [3]u32{ state.s[3], state.r[5], state.a[4] });
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{0x8000000B | 3 << 26 | 1 << 5}));
    // 16-bit value with bit 15 into A: widening unmeasured.
    try state.y_mem.put(gpa, 0x700, 0x8001);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{0xB416A02B}));
}

test "x19mac's X store: X:[A<{w1[2:0], h[31:30]}>] <- s<{w2[0], w1[31:29]}> before the writes (facts 312, 384)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    for (0..32) |k| state.a[k] = 0x100 + 0x10 * @as(u32, @intCast(k));
    state.s[13] = 0x00ABCD;
    state.s[12] = 3;
    // The decode stop's shape: w1 class 101 + w2[0] = 1 -> s13; head ALU off; k = (w1[2:0] << 2) | h[31:30].
    const w1: u32 = 0xA002A008 | 5; // w1[2:0] = 5, op 4, e1 <- P
    try emu.executeBundle(gpa, &state, &.{ 0x40000019, w1, 0x21C0019B, 0xC0000580 });
    try testing.expectEqual(@as(?u32, 0x00ABCD), state.getX(0x100 + 0x10 * ((5 << 2) | 1)));
    // Head ALU rewriting s13 (s13 <- s0 + s0): the store keeps the old s13.
    state.s[13] = 0x000777;
    try emu.executeBundle(gpa, &state, &.{ 0x40000019 | 13 << 15, w1, 0x21C0019B, 0xC0000580 });
    try testing.expectEqual(@as(?u32, 0x000777), state.getX(0x100 + 0x10 * ((5 << 2) | 1)));
    try testing.expectEqual(@as(u32, 0), state.s[13]);
}

test "rop passes e and R to s sign-extended from 16 bits; e keeps 16 (810)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[10] = 0xA914;
    state.e[11] = 0x1234;
    state.r[2] = 0xA914;
    for ([_]struct { src: []const u8, want: u32 }{
        .{ .src = "rop s5, R0, e10, R0, #0", .want = 0xFFA914 },
        .{ .src = "rop s5, R0, e0, R2, #1", .want = 0xFFA914 },
        .{ .src = "rop s5, R0, e11, R0, #0", .want = 0x1234 },
    }) |k| {
        const b = try asm_mod.assembleOne(k.src);
        try emu.executeBundle(gpa, &state, b.slice());
        try testing.expectEqual(k.want, state.s[5]);
    }
    // x19.e mux 2 copies e as stored: 16 bits.
    const xe = try asm_mod.assembleOne("x19.e e15, #0x1, s6, e10, #2");
    try emu.executeBundle(gpa, &state, xe.slice());
    try testing.expectEqual(@as(u32, 0xA914), state.s[6]);
}

test "x19mac: class 011 runs the head ALU and stores s<{w2[0], 011}> (807)" {
    const gpa = testing.allocator;
    for ([_]struct { w1: u32, x: u32, alu: bool }{
        .{ .w1 = 0x2002A050, .x = 0x999, .alu = false },
        .{ .w1 = 0x4002A050, .x = 0xAAA, .alu = true },
        .{ .w1 = 0x6002A050, .x = 0xFFFA99, .alu = true },
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[12] = 0x1234;
        state.s[11] = 0xFFFA99; // -0x567
        state.s[9] = 0x999;
        state.s[10] = 0xAAA;
        state.s[3] = 0x111;
        state.s[4] = 0x222;
        state.s[13] = 0xDD;
        state.a[1] = 0x700;
        // Head ALU s13 <- s3 op 14 / s_mod 10 s4; wide op 4 x = s12, y = s11, e10 <- p, R2 <- p,
        // X:[A1] <- s<{w2[0], w1[31:29]}>.
        try emu.executeBundle(gpa, &state, &.{ 0x505699D9, k.w1, 0x21C0E197, 0xC0000100 });
        try testing.expectEqual(@as(u32, 0xA914), state.e[10]);
        try testing.expectEqual(@as(u32, 0xA914), state.r[2]);
        try testing.expectEqual(k.x, state.x_mem.get(0x700).?);
        try testing.expectEqual(@as(u32, if (k.alu) 0x333 else 0xDD), state.s[13]);
    }
}

test "x19mac with w1[17] = 0: w1[15] rounds or truncates the high part; w1[16] is refused (312)" {
    const gpa = testing.allocator;
    for ([_]struct { x: u32, y: u32, w15: u32, want: u32 }{
        .{ .x = 0x7FFF, .y = 0x7FFF, .w15 = 1, .want = 0x80 },
        .{ .x = 0x7FFF, .y = 0x7FFF, .w15 = 0, .want = 0x7F },
        .{ .x = 0xFFFFFF, .y = 0x7FFF, .w15 = 0, .want = 0xFFFFFF },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[3] = c.x;
        state.s[4] = c.y;
        const w1: u32 = 4 << 11 | c.w15 << 15;
        try emu.executeBundle(gpa, &state, &.{ 0x00000019, w1, 1 << 29 | 5 << 9 | 3 << 5 | 4 << 1, 6 << 29 });
        try testing.expectEqual(c.want, state.s[5]);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x00000019, 4 << 11 | 1 << 16, 1 << 29 | 5 << 9 | 3 << 5 | 4 << 1, 6 << 29 }));
}

test "unset_reads counts X / Y reads of never-written addresses" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    var unset = emu.UnsetReads{};
    state.unset_reads = &unset;
    try state.x_mem.put(gpa, 1, 5);
    try state.y_mem.put(gpa, 1, 5);
    _ = state.readX(1);
    _ = state.readX(2);
    _ = state.getY(1);
    _ = state.getY(2);
    _ = state.getY(3);
    try testing.expectEqual(emu.UnsetReads{ .x = 1, .y = 2, .first_x = 2, .first_y = 2 }, unset);
}

// Compatibility run of an external program read in place from X17_COMPAT_PROG (a little-endian
// u32 syllable stream).  Skipped when unset.  Reports only ran / refused / wedged and the
// bundle count, plus counts of X / Y reads before any write.
test "compat: external program from reset state" {
    const path = testing.environ.getPosix("X17_COMPAT_PROG") orelse return error.SkipZigTest;
    const gpa = testing.allocator;
    const bytes = std.Io.Dir.cwd().readFileAlloc(testing.io, path, gpa, .limited(1 << 20)) catch return error.SkipZigTest;
    defer gpa.free(bytes);
    const words = try gpa.alloc(u32, bytes.len / 4);
    defer gpa.free(words);
    for (words, 0..) |*w, i| w.* = std.mem.readInt(u32, bytes[i * 4 ..][0..4], .little);
    var state = emu.State{};
    defer state.deinit(gpa);
    // The profile only feeds the total bundle count; nothing per syllable is kept.
    const prof = try gpa.alloc(u64, words.len);
    defer gpa.free(prof);
    @memset(prof, 0);
    state.profile = prof;
    var unset = emu.UnsetReads{};
    state.unset_reads = &unset;
    const r = emu.runFrom(gpa, &state, words, 0, 50_000_000);
    if (r) |n| {
        std.debug.print("compat: ran, {d} bundles, halted={}, unset reads X {d} Y {d}\n", .{ n, state.halted, unset.x, unset.y });
    } else |e| {
        var n: u64 = 0;
        for (prof) |c| n += c;
        std.debug.print("compat: {s} after {d} bundles, unset reads X {d} Y {d}\n", .{ @errorName(e), n, unset.x, unset.y });
    }
}

// Compatibility run of a set image read in place from X17_COMPAT_IMAGE: a sequence of sets, each
// u32 run_count then runs { u32 cpu_addr, u32 len_bytes | width << 28, bytes }; P lane l row r
// at 0x22020000 + l * 0x4000 + 4r (syllable 8 * (r / 2) + 2l + r % 2, fact 45's order), X word a at 0x22030000 + 4a, Y word a at
// 0x22038000 + 2a.  Set 1 is armed from syllable 0; set 2 is then loaded, Y:0xFD80..0xFD87
// seeded with X17_COMPAT_PARAMS (eight hex halfwords), and the decode arm enters at
// X17_COMPAT_DECODE_ENTRY (hex engine-order syllable, the host's +0x988 value; default 0).
// Skipped when unset.  Reports only ran / refused / wedged, bundle counts and unset reads.
const CompatImage = struct {
    bytes: []const u8,
    off: usize = 0,

    fn word(self: *CompatImage) !u32 {
        if (self.off + 4 > self.bytes.len) return error.Truncated;
        defer self.off += 4;
        return std.mem.readInt(u32, self.bytes[self.off..][0..4], .little);
    }

    /// Apply the next set to `prog` and `state`'s X / Y.
    fn applySet(self: *CompatImage, gpa: std.mem.Allocator, prog: []u32, state: *emu.State) !void {
        const runs = try self.word();
        for (0..runs) |_| {
            const addr = try self.word();
            const len: usize = (try self.word()) & 0x0FFFFFFF;
            if (self.off + len > self.bytes.len) return error.Truncated;
            const data = self.bytes[self.off..][0..len];
            self.off += len;
            try storeRun(gpa, prog, state, addr, data);
        }
    }

    fn storeRun(gpa: std.mem.Allocator, prog: []u32, state: *emu.State, addr: u32, data: []const u8) !void {
        if (addr >= 0x22038000) {
            var i: usize = 0;
            while (i + 2 <= data.len) : (i += 2)
                try state.y_mem.put(gpa, (addr - 0x22038000 + @as(u32, @intCast(i))) / 2, std.mem.readInt(u16, data[i..][0..2], .little));
        } else if (addr >= 0x22030000) {
            var i: usize = 0;
            while (i + 4 <= data.len) : (i += 4)
                try state.x_mem.put(gpa, (addr - 0x22030000 + @as(u32, @intCast(i))) / 4, std.mem.readInt(u32, data[i..][0..4], .little) & 0xFFFFFF);
        } else {
            const rel = addr - 0x22020000;
            const lane = rel / 0x4000;
            var i: usize = 0;
            while (i + 4 <= data.len) : (i += 4) {
                const row = (rel % 0x4000 + i) / 4;
                prog[8 * (row / 2) + 2 * lane + row % 2] = std.mem.readInt(u32, data[i..][0..4], .little);
            }
        }
    }
};

fn compatArm(gpa: std.mem.Allocator, state: *emu.State, prog: []const u32, label: []const u8, kind: emu.Arm) void {
    var unset = emu.UnsetReads{};
    state.unset_reads = &unset;
    defer state.unset_reads = null;
    const prof = gpa.alloc(u64, prog.len) catch return;
    defer gpa.free(prof);
    @memset(prof, 0);
    state.profile = prof;
    defer state.profile = null;
    // Syllable i sits in lane i mod 4 of the whole image, so each arm counts lanes from 0.
    state.cur_lane = 0;
    const r = emu.arm(gpa, state, prog, kind, 50_000_000);
    var n: u64 = 0;
    for (prof) |c| n += c;
    const verdict: []const u8 = if (r) |_| (if (state.halted) "ran, halted" else if (state.silent_stop) "ran, stopped without completion" else "ran off the end") else |e| @errorName(e);
    std.debug.print("compat {s}: {s} after {d} bundles, unset reads X {d} Y {d}, OPEN p skips {d}\n", .{ label, verdict, n, unset.x, unset.y, unset.open_p });
}

test "compat: set image, configure then decode arm" {
    const path = testing.environ.getPosix("X17_COMPAT_IMAGE") orelse return error.SkipZigTest;
    const gpa = testing.allocator;
    const bytes = std.Io.Dir.cwd().readFileAlloc(testing.io, path, gpa, .limited(1 << 22)) catch return error.SkipZigTest;
    defer gpa.free(bytes);
    const prog = try gpa.alloc(u32, 0x4000);
    defer gpa.free(prog);
    @memset(prog, 0);
    var state = emu.State{};
    defer state.deinit(gpa);
    var img = CompatImage{ .bytes = bytes };
    try img.applySet(gpa, prog, &state);
    compatArm(gpa, &state, prog, "set 1", .pass1);
    try img.applySet(gpa, prog, &state);
    if (testing.environ.getPosix("X17_COMPAT_PARAMS")) |params| {
        var it = std.mem.tokenizeAny(u8, params, " ,");
        var a: u32 = 0xFD80;
        while (it.next()) |hw| : (a += 1) try state.y_mem.put(gpa, a, try std.fmt.parseInt(u32, hw, 16));
    }
    const entry_hex = testing.environ.getPosix("X17_COMPAT_DECODE_ENTRY") orelse "0";
    compatArm(gpa, &state, prog, "set 2 decode arm", .{ .decode = try std.fmt.parseInt(usize, entry_hex, 16) });
}

test "x16 b16 under any S1 class with S1[18] and S2 0xE004 (fact 778)" {
    const gpa = testing.allocator;
    // S1 class 111: s14 <- sx(Y:[e4]), X:[xa] <- old s14, s12 <- old s3.
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[4] = 0x300;
    state.xa = 0x40;
    state.s[14] = 0x123456;
    state.s[3] = 0x000777;
    try state.y_mem.put(gpa, 0x300, 0xB454);
    _ = try emu.runFrom(gpa, &state, &.{ 0x01C00016, 0xE0041C00, 0xE6846000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0xFFB454), state.s[14]);
    try testing.expectEqual(@as(?u32, 0x123456), state.x_mem.get(0x40));
    try testing.expectEqual(@as(u32, 0x000777), state.s[12]);
}

test "x16 b16's xa load also writes A17 (fact 840)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    // The x16.b16 base under S1 class 001 (head[31], S1[3] set), head[20:5] = 0x11C3.
    try emu.executeBundle(gpa, &state, &.{ 0x80000016 | (0x11C3 << 5), 0x20041008, 0xE0067100 });
    try testing.expectEqual(@as(u32, 0x11C3), state.a[17]);
    try testing.expectEqual(@as(?u32, 0x11C3), state.xa);
}

test "x16 b16 placement 10 zero-extends the Y word (fact 800)" {
    const gpa = testing.allocator;
    // 778's word with S1[13:12] = 10: s14 <- Y:[e4] zero-extended.
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[4] = 0x300;
    state.xa = 0x40;
    try state.y_mem.put(gpa, 0x300, 0xB454);
    try emu.executeBundle(gpa, &state, &.{ 0x01C00016, 0xE0042C00, 0xE6846000 });
    try testing.expectEqual(@as(u32, 0x00B454), state.s[14]);
}

test "x17sty stop shape under R mux 2: e<k> takes the AGU result and runs on (801, 916)" {
    const gpa = testing.allocator;
    // 667's bundle with k = S1[2:0] = 1: e<2> is both the store's address and the side write.
    const word = [_]u32{ 0x28200017, 0x60012d51, 0x61fcb804 };
    for ([_]u8{ 0, 1 }) |p1| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[2] = 0x420;
        state.r[5] = 0x123;
        state.s[3] = 0x4567;
        state.p[1] = p1;
        const run = emu.runFrom(gpa, &state, &(word ++ [_]u32{ 0x00000017, 0x31308000, 0xA0000000 }), 0, 16);
        if (p1 == 1) {
            // 801's p1 = 1 row was a stop, retracted by 916 and not rechecked: refused.
            try testing.expectError(emu.RunError.UnknownForm, run);
            continue;
        }
        const n = try run;
        try testing.expectEqual(@as(?u32, 0x4567), state.y_mem.get(0x420));
        try testing.expectEqual(@as(u32, 0x523), state.r[5]);
        try testing.expectEqual(@as(u32, 0x523), state.e[2]);
        try testing.expectEqual(@as(usize, 2), n);
        try testing.expect(state.halted);
        try testing.expectEqual(@as(?u32, null), state.y_mem.get(0x523));
    }
}

test "ealu op 5 is a, op 7 is -a, head[9] makes op 5 -a (fact 132)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w: u32, want: u32 }{
        .{ .h = 0x000001d0, .w = 0xc0305048, .want = 0x000123 }, // 132's own word: e9 <- s12
        .{ .h = 0x000001d0, .w = 0xc0307048, .want = 0xFFFEDD }, // op 7: -a
        .{ .h = 0x000003d0, .w = 0xc0305048, .want = 0xFFFEDD }, // head[9]: op 5 gives -a
    }) |row| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[12] = 0x123;
        try emu.executeBundle(gpa, &state, &.{ row.h, row.w });
        try testing.expectEqual(row.want, state.e[9]);
    }
}

test "x19.ld stalls with 0..2 bundles after an st.wr; 7 after completes (fact 804)" {
    const gpa = testing.allocator;
    const st_wr = [_]u32{ 0x2000000F, 0x22AD8000 }; // st.wr #0, e6, s5
    const ld = [_]u32{ 0x00000019, 0x0000010C, 0x00000C00, 0xF0000403 }; // x19.ld s3, #4, e8, #2
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    for ([_]struct { gap: usize, ok: bool }{
        .{ .gap = 0, .ok = false },
        .{ .gap = 1, .ok = false },
        .{ .gap = 2, .ok = false },
        .{ .gap = 6, .ok = false }, // 3..6 unmeasured: refused
        .{ .gap = 7, .ok = true },
    }) |k| {
        var words: std.ArrayList(u32) = .empty;
        defer words.deinit(gpa);
        try words.appendSlice(gpa, &st_wr);
        try words.appendNTimes(gpa, 0x40000005, k.gap);
        try words.appendSlice(gpa, &ld);
        try words.appendSlice(gpa, &halt);
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.runFrom(gpa, &state, words.items, 0, 64);
        if (k.ok) _ = try r else try testing.expectError(error.UnknownForm, r);
    }
}

test "image: init pass then decode arm from one x17 image" {
    const gpa = testing.allocator;
    var prog = switch (try asm_mod.assembleProgram(gpa,
        \\.pass init
        \\.xdata 0x10, 0x123456
        \\x17.halt
        \\.pass decoder
        \\.ydata 0x20, 0xBEEF
        \\.param 0xFD83, 4
        \\x17.halt
    , .{})) {
        .ok => |p| p,
        .err => return error.TestUnexpectedResult,
    };
    defer prog.deinit();
    const bytes = try image.write(gpa, &prog);
    defer gpa.free(bytes);
    var img = try image.parse(gpa, bytes);
    defer img.deinit(gpa);

    var state = emu.State{};
    defer state.deinit(gpa);
    const p = try gpa.alloc(u32, 0x4000);
    defer gpa.free(p);
    @memset(p, 0);
    var pass: ?u8 = null;
    for (img.sections) |sec| {
        if (pass != null and pass.? != sec.pass) try runImagePass(gpa, &state, p, pass.?);
        pass = sec.pass;
        switch (sec.kind) {
            .p => image.placeRows(p, sec),
            .x => for (sec.words, 0..) |w, k| try state.x_mem.put(gpa, sec.addr + @as(u32, @intCast(k)), w),
            .y, .param => for (sec.words, 0..) |w, k| try state.y_mem.put(gpa, sec.addr + @as(u32, @intCast(k)), w),
        }
    }
    try runImagePass(gpa, &state, p, pass.?);
    try testing.expectEqual(@as(?u32, 0x123456), state.x_mem.get(0x10));
    try testing.expectEqual(@as(?u32, 4), state.y_mem.get(0xFD83));
}

fn runImagePass(gpa: std.mem.Allocator, state: *emu.State, p: []const u32, pass: u8) !void {
    state.halted = false;
    state.cur_lane = 0;
    const kind: emu.Arm = if (pass == image.decoder_pass) .{ .decode = 256 } else .pass1;
    _ = try emu.arm(gpa, state, p, kind, 64);
    try testing.expect(state.halted);
}

test "alu code 0x28 moves s<b> or # by m, a inert (fact 779)" {
    const gpa = testing.allocator;
    // head = b<<26 | m<<23 | #<<19 | d<<15 | a<<11 | 0x28<<5 | 0x05, class [31:30] = 01.
    for ([_]struct { b: u32, m: u32, n: u32, want: u32 }{
        .{ .b = 10, .m = 5, .n = 0, .want = 0x00ABCD }, // 6a850505: s10 <- s10
        .{ .b = 2, .m = 6, .n = 5, .want = 0x000222 }, // m 6: s<b> whatever # is
        .{ .b = 0, .m = 5, .n = 8, .want = 8 }, // b index 0: #
        .{ .b = 2, .m = 4, .n = 15, .want = 15 }, // m 4: #
        .{ .b = 0, .m = 1, .n = 5, .want = 0 }, // m 1, b index 0: 0
        .{ .b = 2, .m = 7, .n = 1, .want = 1 }, // m 7: #
    }) |row| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[2] = 0x222;
        state.s[10] = 0xABCD;
        state.s[12] = 0x777;
        const head: u32 = 0x40000000 | row.b << 26 | row.m << 23 | row.n << 19 | 10 << 15 | 12 << 11 | 0x28 << 5 | 0x05;
        try emu.executeBundle(gpa, &state, &.{head});
        try testing.expectEqual(row.want, state.s[10]);
    }
}

test "x17agu2 fn 11 is y - i into e<2*S1[2:0] + h[31]> (fact 780)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, want: u32 }{
        .{ .s1 = 0x68000581, .want = 0x57D }, // the measured bundle: fn 11
        .{ .s1 = 0x68000501, .want = 0x683 }, // fn 10: y + i
        .{ .s1 = 0x68000601, .want = 0x600 }, // fn 12: y
    }) |row| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[8] = 0x600;
        state.e[9] = 0x490;
        try emu.executeBundle(gpa, &state, &.{ 0x40041817, row.s1, 0x00140066 });
        try testing.expectEqual(row.want, state.e[2]);
        try testing.expectEqual(@as(u32, 0x48F), state.e[9]);
    }
    // h[31] = 1 moves the dest to e3.
    var hi = emu.State{};
    defer hi.deinit(gpa);
    hi.r[8] = 0x600;
    try emu.executeBundle(gpa, &hi, &.{ 0xC0041817, 0x68000581, 0x00140066 });
    try testing.expectEqual(@as(u32, 0x57D), hi.e[3]);
}

test "x17sty stop form: the store lands, then the run ends with no completion (fact 667)" {
    const gpa = testing.allocator;
    // 667's row (k = 0, S2[22:18] = 00000; 916: the decoder's 61fcb804 runs on), then an ldi
    // that must not run.
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[6] = 0x420;
    state.s[3] = 0x4567;
    const n = try emu.runFrom(gpa, &state, &.{ 0x28200017, 0x60030100, 0x60003800, 0x00a00010, 0xe6420080 }, 0, 16);
    try testing.expectEqual(@as(?u32, 0x4567), state.y_mem.get(0x420));
    try testing.expectEqual(@as(usize, 1), n);
    try testing.expect(state.silent_stop);
    try testing.expect(!state.halted);
    // k = 3 runs away: refused.
    var away = emu.State{};
    defer away.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.executeBundle(gpa, &away, &.{ 0x28200017, 0x60030103, 0x60003800 }));
}

test "x02.asrc and x02.esrc also write A1 (157, 795)" {
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    const Case = struct { src: []const u8, s5: u32, a1: u32 };
    for ([_]Case{
        .{ .src = "x02.asrc s5, A9", .s5 = 0x41, .a1 = 0x41 },
        .{ .src = "x02.esrc s5, e3", .s5 = 0x77, .a1 = 0x101 },
    }) |c| {
        const b = try asm_mod.assembleOne(c.src);
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.a[1] = 0x100;
        st.a[9] = 0x40;
        st.e[3] = 0x77;
        const w = b.slice();
        _ = try emu.runFrom(testing.allocator, &st, &([_]u32{ w[0], w[1], w[2], w[3], w[4], w[5] } ++ halt), 0, 8);
        try testing.expectEqual(c.s5, st.s[5]);
        try testing.expectEqual(c.a1, st.a[1]);
        try testing.expectEqual(@as(u32, 0x40), st.a[9]);
    }
}

fn ldiS(n: u32, v: u32) u32 {
    return 0xC800000B | ((16 + n) << 21) | (v << 5);
}

test "x00 control 5 returns past the call's delay bundles (911)" {
    const gpa = testing.allocator;
    const halt = try assemble(gpa, "x17.halt\n");
    defer gpa.free(halt);
    const nop: u32 = 0x40000005;
    var prog: std.ArrayList(u32) = .empty;
    defer prog.deinit(gpa);
    // 0: call 13; 7, 8: its delay bundles; 9: the link.
    try prog.appendSlice(gpa, &.{ 0, 13 << 18, 0, 4, 0, 0, 0, ldiS(1, 1), nop, ldiS(2, 2) });
    try prog.appendSlice(gpa, halt);
    while (prog.items.len < 13) try prog.append(gpa, nop);
    // 13: the callee, then its return and two delay bundles.
    try prog.appendSlice(gpa, &.{ ldiS(3, 3), 0, 0, 0, 5, 0, 0, 0, ldiS(4, 4), nop, ldiS(5, 5) });
    var st = emu.State{};
    defer st.deinit(gpa);
    _ = try emu.runLinear(gpa, &st, prog.items);
    try testing.expectEqual([_]u32{ 1, 2, 3, 4, 0 }, st.s[1..6].*);
    try testing.expectEqual(@as(?usize, null), st.link);
}

test "x00 control 5's return clears p<w2[17:14]> under w4[5], w5[4], w5[31] (938)" {
    const gpa = testing.allocator;
    for ([_]struct { w2: u32, w5: u32, cleared: ?usize }{
        .{ .w2 = 0x00004000, .w5 = 0x80000010, .cleared = 1 }, // 0x1FBB
        .{ .w2 = 0x00020000, .w5 = 0x80000010, .cleared = 8 },
        .{ .w2 = 0x00004000, .w5 = 0x00000010, .cleared = null }, // w5[31] clear
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        @memset(state.p[1..], 1);
        state.link = 0x4D2;
        state.pc = 0x1FBB;
        try emu.executeBundle(gpa, &state, &.{ 0, 0, c.w2, 5, 0x20, c.w5, 0 });
        for (1..16) |k| try testing.expectEqual(@as(u8, if (c.cleared == k) 0 else 1), state.p[k]);
    }
}

test "x04 w2[28] jumps q + w1[31:18] (940)" {
    const gpa = testing.allocator;
    for ([_]struct { w2: u32, want: ?i32 }{ .{ .w2 = 0x10000000, .want = 0x60 }, .{ .w2 = 0, .want = null } }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        try emu.executeBundle(gpa, &state, &.{ 0x88001804, 0x01800518, c.w2, 0x08200000, 0x00000000, 0x03000000 }); // 0x506
        try testing.expectEqual(c.want, state.pending_jmp_offset);
    }
}

test "x16agu's e slot sets p<S2[10:7]> under head[6], !head[9], S1[23] (942)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0xB0A689D6, s1: u32 = 0x00A04500, want: u8 = 1 }{
        .{},
        .{ .h = 0xB0A68996, .want = 0 }, // head[6]
        .{ .s1 = 0x00204500, .want = 0 }, // S1[23]
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        _ = emu.executeBundle(gpa, &state, &.{ c.h, c.s1, 0x60002180 }) catch {}; // 0x596
        try testing.expectEqual(c.want, state.p[3]);
    }
}

test "x10's class-100 second with S1[15:14] = 00 jumps q + 4 * S1[12:0] (948)" {
    const gpa = testing.allocator;
    for ([_]struct { h: u32 = 0x0A0001D0, s1: u32, want: ?i32 }{
        .{ .s1 = 0x80A20017, .want = 0x5C }, // 0x5F2 -> 0x64E
        .{ .s1 = 0x80A20037, .want = 0xDC }, // S1[5]
        .{ .s1 = 0x80A24017, .want = null }, // S1[14]: no jump
        .{ .h = 0xF1178050, .s1 = 0x801A0658, .want = null }, // 0x12F: head[31:30] = 11 runs on
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        emu.executeBundle(gpa, &state, &.{ c.h, c.s1 }) catch {};
        try testing.expectEqual(c.want, state.pending_jmp_offset);
    }
}

test "x01 tests p<w1[5:3]> with x00's sense (943, 946)" {
    const gpa = testing.allocator;
    for ([_]struct { w1: u32 = 0x4E54058A, ctl: u32, p: u8, want: ?i32 }{
        .{ .ctl = 3, .p = 0, .want = null }, // 0x5D9: c = 1, p1 = 0
        .{ .ctl = 2, .p = 0, .want = 0x1395 },
        .{ .ctl = 3, .p = 1, .want = 0x1395 },
        .{ .w1 = 0x4E54059A, .ctl = 3, .p = 1, .want = 0x1395 }, // c = 3
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[(c.w1 >> 3) & 7] = c.p;
        state.pc = 0x5D9;
        try emu.executeBundle(gpa, &state, &.{ 0x100031C1, c.w1, 0x00024280, 0x80000000 | c.ctl, 0x00000081, 0x00001800, 0 });
        try testing.expectEqual(c.want, state.pending_jmp_offset);
    }
    var st = emu.State{};
    defer st.deinit(gpa);
    try testing.expectError(emu.RunError.UnknownForm, emu.executeBundle(gpa, &st, &.{ 0x100031C1, 0x4E5405A2, 0x00024280, 0x80000003, 0x00000081, 0x00001800, 0 })); // c = 4
}

test "x02.asrc also runs the w2 multiply into s<w2[12:9]> (953)" {
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, s2: u32, want: u32 }{
        .{ .s1 = 0x7FFF, .s2 = 0x6000, .want = 0x60 },
        .{ .s1 = 0x4000, .s2 = 0x4000, .want = 0x20 },
        .{ .s1 = 0x111, .s2 = 0x222, .want = 0 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[1] = c.s1;
        state.s[2] = c.s2;
        state.a[15] = 0x40;
        try emu.executeBundle(gpa, &state, &.{ 0x00000002, 0x0000A000, 0x00020442, 0x0000000F, 0x00041068, 0x10004200 });
        try testing.expectEqual(c.want, state.s[2]);
        try testing.expectEqual(@as(u32, 0x41), state.s[13]);
        try testing.expectEqual(@as(u32, 0x40), state.a[15]);
    }
}

test "x02 esrc multiplies too; the templates' w2 dest s0 writes no s (954)" {
    const gpa = testing.allocator;
    {
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.s[k] = 0x111 * @as(u32, @intCast(k));
        state.e[3] = 0x490;
        try emu.executeBundle(gpa, &state, &.{ 0x00000002, 0x0000A000, 0x00020442, 0x18000001, 0x00041008, 0x20004200 });
        try testing.expectEqual(@as(u32, 0x490), state.s[1]);
        try testing.expectEqual(@as(u32, 0), state.s[2]);
    }
    for ([_][]const u8{ "x02.asrc s13, A15", "x02.esrc s13, e3", "x02.ainc #6, #6" }) |src| {
        const m = try asm_mod.assembleOne(src);
        try testing.expectEqual(@as(u32, 0), (m.slice()[2] >> 9) & 0xF);
        var state = emu.State{};
        defer state.deinit(gpa);
        for (1..16) |k| state.s[k] = 0x111 * @as(u32, @intCast(k));
        try emu.executeBundle(gpa, &state, m.slice());
        for (1..16) |k| if (k != 13) try testing.expectEqual(0x111 * @as(u32, @intCast(k)), state.s[k]);
    }
}

test "x03's counted jr writes the p it tests (915)" {
    const gpa = testing.allocator;
    const halt = try assemble(gpa, "x17.halt\n");
    defer gpa.free(halt);
    const nop: u32 = 0x40000005;
    for ([_]struct { e2: u32, p10: u8, n: u32 }{
        .{ .e2 = 5, .p10 = 1, .n = 6 },
        .{ .e2 = 1, .p10 = 1, .n = 2 },
        .{ .e2 = 0, .p10 = 1, .n = 2 },
        .{ .e2 = 5, .p10 = 0, .n = 1 },
    }) |c| {
        var prog: std.ArrayList(u32) = .empty;
        defer prog.deinit(gpa);
        try prog.appendSlice(gpa, &.{ 0xC0000003, 0x60000003, 0x181F4000, 0x0100000F, 0x00004001, 0x001E8000, nop, nop }); // 0x2F0
        try prog.appendSlice(gpa, halt);
        var st = emu.State{};
        defer st.deinit(gpa);
        st.e[2] = c.e2;
        st.p[10] = c.p10;
        st.s[3] = 0x777;
        st.a[15] = 0x100;
        _ = try emu.runLinear(gpa, &st, prog.items);
        if (c.p10 == 1) try testing.expectEqual(@as(u8, 0), st.p[10]);
        try testing.expectEqual(0x100 + c.n, st.a[15]);
        try testing.expectEqual(@as(?u32, 0x777), st.readX(0x100));
        try testing.expectEqual(@as(?u32, 0x777), st.readX(0x100 + c.n - 1));
        try testing.expectEqual(@as(?u32, null), st.readX(0x100 + c.n));
    }
}

test "x03's counter writes p<w2[16:13]>, not the tested p (922)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[0] = 5;
    state.p[2] = 1;
    state.s[3] = 0x123;
    try emu.executeBundle(gpa, &state, &.{ 0x0D5001C3, 0x00240040, 0x18190000, 0, 0, 0x80008030 }); // 0x464
    try testing.expectEqual(@as(u32, 0x123), state.e[8]); // 902's copy, w0[22] inert here
    try testing.expectEqual(@as(u8, 1), state.p[8]);
    try testing.expectEqual(@as(u8, 1), state.p[2]);
    try testing.expectEqual(@as(?i32, 9), state.pending_jmp_offset);
    // p8 is 1 with e0 = 0 and s3 = 0 too.
    var zero = emu.State{};
    defer zero.deinit(gpa);
    try emu.executeBundle(gpa, &zero, &.{ 0x0D5001C3, 0x00240040, 0x18190000, 0, 0, 0x80008030 });
    try testing.expectEqual(@as(u8, 1), zero.p[8]);
    try testing.expectEqual(@as(u32, 0), zero.e[8]);
}

test "x16 calls the arm base + S2[21:9] under S1[23:21] = 001 (923, 924, 928)" {
    const gpa = testing.allocator;
    var state = emu.State{ .call_base = 0x100 };
    defer state.deinit(gpa);
    state.pc = 0x4CD;
    try emu.executeBundle(gpa, &state, &.{ 0x80029D76, 0x01200000, 0x803C9400 }); // 0x4CD
    try testing.expectEqual(@as(?i32, 0x1F4A - 0x4CD), state.pending_jmp_offset);
    try testing.expect(state.call_pending);
    var plain = emu.State{};
    defer plain.deinit(gpa);
    plain.pc = 0x4CD;
    emu.executeBundle(gpa, &plain, &.{ 0x80029D76, 0x01000000, 0x803C9400 }) catch {}; // S1[21] clear
    try testing.expect(!plain.call_pending);
}

test "ctl 4 calls land at the arm's base + field (926, 928)" {
    const gpa = testing.allocator;
    for ([_]struct { base: usize, want: i32 }{ .{ .base = 0, .want = 0x1F6F }, .{ .base = 0x100, .want = 0x206F } }) |c| {
        var state = emu.State{ .call_base = c.base };
        defer state.deinit(gpa);
        state.pc = 0x2D2;
        try emu.executeBundle(gpa, &state, &.{ 0x00A55000, 0x7DBC0000, 0x00000224, 0x00F00004, 0x00200000, 0x16000000, 0x00000090 });
        try testing.expectEqual(@as(?i32, c.want - 0x2D2), state.pending_jmp_offset);
    }
}

test "x17 S1[23:18] = 0x3F jumps q + 0xE iff p<S2[15:12]> (927)" {
    const gpa = testing.allocator;
    for ([_]struct { s2: u32 = 0x84013108, p3: u8 = 1, want: ?i32 = 0xE }{
        .{},
        .{ .p3 = 0, .want = null },
        .{ .s2 = 0x84010108, .p3 = 0 }, // sel 0: always
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.p[3] = c.p3;
        try emu.executeBundle(gpa, &state, &.{ 0x80A5D7F7, 0x00FC0003, c.s2 });
        try testing.expectEqual(c.want, state.pending_jmp_offset);
    }
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(emu.RunError.UnknownForm, emu.executeBundle(gpa, &state, &.{ 0x80A5D7F7, 0x00FC0007, 0x84010108 })); // S1[4:0] = 7 wedges
}

test "xld10 writes head[20:5] to e<S[6:3]> (930)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0x2EAE;
    try emu.executeBundle(gpa, &state, &.{ 0x4005D6D0, 0x60248188 }); // 0x1FA9
    try testing.expectEqual(@as(u32, 0x2EB6), state.e[1]);
}

test "x16 0xE037 saves s<S2[3:0]> at Y:[e1] and loads e<S1[23:20]> (933, 934)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0x2EB4;
    state.s[5] = 0x14EC;
    try emu.executeBundle(gpa, &state, &.{ 0x4005D6B6, 0x00140000, 0xE0370005 }); // 0x1F5D
    try testing.expectEqual(@as(?u32, 0x14EC), state.getY(0x2EB4));
    try testing.expectEqual(@as(u32, 0x2EB5), state.e[1]);
}

test "st.y's class-001 pair with S[3:0] = 4 copies e<S[8:5]> into s<head[24:21]> (935)" {
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    state.e[1] = 0x2EB3;
    state.e[5] = 0x14EC;
    try emu.executeBundle(gpa, &state, &.{ 0x20A5D68F, 0x20A380A4 }); // 0x1F5B
    try testing.expectEqual(@as(u32, 0x14EC), state.s[5]);
    try testing.expectEqual(@as(u32, 0x2EB4), state.e[1]);
    try testing.expectEqual(@as(?u32, 0), state.getY(0x2EB3)); // the old s5
}

test "x02 mode 1 writes p<w2[16:13]> <- R<w5[25:22]> != 0 under A fn 26 (936)" {
    const gpa = testing.allocator;
    for ([_]struct { w5: u32 = 0x30821000, r2: u32 = 1, want: u8 = 1 }{
        .{},
        .{ .r2 = 0, .want = 0 },
        .{ .w5 = 0x30021000, .want = 0 }, // R0
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.r[2] = c.r2;
        state.p[3] = 1 - c.want;
        try emu.executeBundle(gpa, &state, &.{ 0x00000002, 0x00000008, 0x06806000, 0x18000000, 0x0000C868, c.w5 }); // 0x4D5
        try testing.expectEqual(c.want, state.p[3]);
    }
}

test "ptst writes p13 through the class-011 dest bit (fact 219)" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa, "ldi s12, #0\nptst p13, s0, #11, #0x0B\nx17.halt\n");
    defer gpa.free(prog);
    var st = emu.State{};
    _ = try emu.runLinear(gpa, &st, prog);
    try testing.expectEqual(@as(u8, 1), st.p[13]);
}

fn ptstP13(word: u32, s2: u32, s12: u32) !u8 {
    var st = emu.State{};
    defer st.deinit(testing.allocator);
    st.s[2] = s2;
    st.s[12] = s12;
    const prog = [_]u32{ word, 0x00000017, 0x31308000, 0xA0000000 };
    _ = try emu.runLinear(testing.allocator, &st, &prog);
    return st.p[13];
}

test "ptst h[18:15] picks row 010's relation of s12 against 0 (823)" {
    // s12 = -3, 0, 5, 0x008000; silicon p13 per field value.
    const vals = [_]u32{ 0xFFFFFD, 0, 5, 0x008000 };
    const rows = [_]struct { f: u32, p: [4]u8 }{
        .{ .f = 0, .p = .{ 1, 1, 0, 0 } },  .{ .f = 1, .p = .{ 0, 1, 1, 1 } },
        .{ .f = 2, .p = .{ 1, 0, 0, 0 } },  .{ .f = 3, .p = .{ 0, 0, 1, 1 } },
        .{ .f = 4, .p = .{ 1, 0, 1, 1 } },  .{ .f = 5, .p = .{ 0, 1, 0, 0 } },
        .{ .f = 9, .p = .{ 0, 0, 1, 1 } },  .{ .f = 13, .p = .{ 0, 1, 1, 1 } },
        .{ .f = 15, .p = .{ 1, 1, 1, 1 } },
    };
    for (rows) |r| for (vals, 0..) |v, i| {
        try testing.expectEqual(r.p[i], try ptstP13(0x7560016D | r.f << 15, 0, v));
    };
}

test "ptst h[18:15] picks row 101's relation of a against b (823)" {
    // a = s2 = 5, b = s12 = -3, 0, 5, 0x008000.
    const vals = [_]u32{ 0xFFFFFD, 0, 5, 0x008000 };
    const rows = [_]struct { f: u32, p: [4]u8 }{
        .{ .f = 0, .p = .{ 0, 0, 1, 1 } }, .{ .f = 1, .p = .{ 1, 1, 1, 0 } },
        .{ .f = 2, .p = .{ 0, 0, 0, 1 } }, .{ .f = 4, .p = .{ 1, 1, 0, 1 } },
    };
    for (rows) |r| for (vals, 0..) |v, i| {
        try testing.expectEqual(r.p[i], try ptstP13(0x76E0116D | r.f << 15, 5, v));
    };
}

test "ptst h[18:15] = 12 sets p when s12 is even (824)" {
    const vals = [_]u32{ 0xFFFFFD, 0, 5, 0x008000, 1, 2, 4, 8, 6, 0x10, 0xFFFFFE, 0x100 };
    const want = [_]u8{ 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1 };
    for (vals, want) |v, w| try testing.expectEqual(w, try ptstP13(0x7560016D | 12 << 15, 0, v));
    // Row 101 with 12 is unmeasured.
    try testing.expectError(error.UnknownForm, ptstP13(0x76E0116D | 12 << 15, 5, 0));
}
