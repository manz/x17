//! The shared sources in examples/lib run in the emulator against plain Zig models.
const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");

const testing = std.testing;

const libs = [_]struct { path: []const u8, text: []const u8 }{
    .{ .path = "lib/bits.sm", .text = @embedFile("bits.sm") },
    .{ .path = "lib/rice.sm", .text = @embedFile("rice.sm") },
};

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

/// MSB-first reference reader.
fn refRead(data: []const u8, pos: *usize, k: u5) u32 {
    var v: u32 = 0;
    for (0..k) |_| {
        const bit = (data[pos.* / 8] >> @intCast(7 - pos.* % 8)) & 1;
        v = v << 1 | bit;
        pos.* += 1;
    }
    return v;
}

const dsts = [_]u4{ 1, 2, 3, 4, 5, 7 };

test "getbits reads what an MSB-first reader reads, across refills" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0xB175);
    const rnd = prng.random();
    for (0..300) |_| {
        var data: [64]u8 = undefined;
        rnd.bytes(&data);
        const skip = rnd.intRangeAtMost(usize, 0, 20);
        var widths: [26]u5 = undefined;
        for (widths[0 .. skip + dsts.len]) |*w| w.* = rnd.intRangeAtMost(u5, 1, 8);

        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        try src.appendSlice(gpa, ".include \"lib/bits.sm\"\n        bits_init(0x4000)\n");
        for (widths[0..skip]) |w| try src.print(gpa, "        getbits(s8, {d})\n", .{w});
        for (dsts, widths[skip .. skip + dsts.len]) |d, w| try src.print(gpa, "        getbits(s{d}, {d})\n", .{ d, w });
        try src.appendSlice(gpa, "        x17.halt\n");
        const prog = try assemble(gpa, src.items);
        defer gpa.free(prog);

        var state = emu.State{};
        defer state.deinit(gpa);
        for (0..data.len / 2) |i| try state.y_mem.put(gpa, 0x4000 + @as(u32, @intCast(i)), @as(u32, data[2 * i]) << 8 | data[2 * i + 1]);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 16);
        try testing.expect(state.halted);

        var pos: usize = 0;
        for (widths[0..skip]) |w| _ = refRead(&data, &pos, w);
        for (dsts, widths[skip .. skip + dsts.len]) |d, w| {
            try testing.expectEqual(refRead(&data, &pos, w), state.s[d]);
        }
    }
}

/// sosumi ref/dsp/alac/rice.zig `get`, in plain integers.
fn refGet(data: []const u8, pos: *usize, m: u32, k: u5, esc: u5) u32 {
    var pre: u32 = 0;
    while (pre < 9) : (pre += 1) {
        if (refRead(data, pos, 1) == 0) break;
    }
    if (pre == 9) return refRead(data, pos, esc);
    const hi = refRead(data, pos, k - 1);
    if (hi == 0) return pre * m;
    return pre * m + 2 * hi + refRead(data, pos, 1) - 1;
}

fn stage(gpa: std.mem.Allocator, state: *emu.State, data: []const u8) !void {
    for (0..data.len / 2) |i| try state.y_mem.put(gpa, 0x4000 + @as(u32, @intCast(i)), @as(u32, data[2 * i]) << 8 | data[2 * i + 1]);
}

test "rice_get decodes what the model decodes: plain, zero-hi and escaped codewords" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x41CE);
    const rnd = prng.random();
    var escapes: usize = 0;
    for (0..400) |_| {
        var data: [64]u8 = undefined;
        const density = rnd.intRangeAtMost(u8, 1, 255); // biased bits reach every prefix length
        for (&data) |*b| {
            var v: u8 = 0;
            for (0..8) |_| v = (v << 1) | @intFromBool(rnd.int(u8) < density);
            b.* = v;
        }
        const skip = rnd.intRangeAtMost(u5, 0, 23);
        const k = rnd.intRangeAtMost(u5, 1, 15);
        const m: u32 = if (rnd.boolean()) (@as(u32, 1) << k) - 1 else rnd.intRangeAtMost(u32, 0, 1 << 14);
        const esc = rnd.intRangeAtMost(u5, 1, 22);

        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        try src.print(gpa,
            \\.include "lib/bits.sm"
            \\.include "lib/rice.sm"
            \\        bits_init(0x4000)
            \\        ldi s7, #{d}
            \\        readn(s4)
            \\        ldi s1, #{d}
            \\        ldi s2, #{d}
            \\        ldi s3, #{d}
            \\        rice_get(s4)
            \\        getbits(s8, 8)
            \\        x17.halt
            \\
        , .{ skip, m, k, esc });
        const prog = try assemble(gpa, src.items);
        defer gpa.free(prog);

        var state = emu.State{};
        defer state.deinit(gpa);
        try stage(gpa, &state, &data);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 16);
        try testing.expect(state.halted);

        var pos: usize = skip;
        var peek: usize = skip;
        if (refRead(&data, &peek, 9) == 0x1FF) escapes += 1;
        try testing.expectEqual(refGet(&data, &pos, m, k, esc), state.s[4]);
        try testing.expectEqual(refRead(&data, &pos, 8), state.s[8]); // and left the reader there
    }
    try testing.expect(escapes > 20);
}
