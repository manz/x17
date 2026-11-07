//! The shared sources in examples/lib run in the emulator against plain Zig models.
const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");

const testing = std.testing;

const libs = [_]struct { path: []const u8, text: []const u8 }{
    .{ .path = "lib/bits.sm", .text = @embedFile("bits.sm") },
    .{ .path = "lib/rice.sm", .text = @embedFile("rice.sm") },
    .{ .path = "lib/predictor.sm", .text = @embedFile("predictor.sm") },
    .{ .path = "lib/unmix.sm", .text = @embedFile("unmix.sm") },
    .{ .path = "lib/alac.sm", .text = @embedFile("alac.sm") },
    .{ .path = "lib/huffman.sm", .text = @embedFile("huffman.sm") },
    .{ .path = "lib/sections.sm", .text = @embedFile("sections.sm") },
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

const RefError = error{ EndOfStream, RunTooLong };

fn refReadChecked(data: []const u8, pos: *usize, k: u5) RefError!u32 {
    if (pos.* + k > data.len * 8) return error.EndOfStream;
    return refRead(data, pos, k);
}

fn refGetChecked(data: []const u8, pos: *usize, m: u32, k: u5, esc: u5) RefError!u32 {
    var pre: u32 = 0;
    while (pre < 9) : (pre += 1) {
        if (try refReadChecked(data, pos, 1) == 0) break;
    }
    if (pre == 9) return refReadChecked(data, pos, esc);
    const hi = try refReadChecked(data, pos, k - 1);
    if (hi == 0) return pre * m;
    return pre * m + 2 * hi + try refReadChecked(data, pos, 1) - 1;
}

/// sosumi ref/alac/rice.zig `decompress`, 32-bit as Apple's reference computes it.
fn refDecompress(data: []const u8, pos: *usize, out: []i32, pb: u32, kb: u5, mb0: u32, max_bits: u5) RefError!void {
    var mb = mb0;
    const wb: u32 = (@as(u32, 1) << kb) - 1;
    var zmode: u32 = 0;
    var c: usize = 0;
    while (c < out.len) {
        const y = (mb >> 9) + 3;
        var k: u5 = @intCast(31 - @clz(y));
        k = @min(k, kb);
        const n = try refGetChecked(data, pos, (@as(u32, 1) << k) - 1, k, max_bits);
        const nd = n +% zmode;
        const mag: i32 = @bitCast((nd +% 1) >> 1);
        out[c] = if (nd & 1 != 0) -%mag else mag;
        c += 1;
        mb = pb *% nd +% mb -% ((pb *% mb) >> 9);
        if (n > 0xffff) mb = 0xffff;
        zmode = 0;
        if (mb *% 4 < 512 and c < out.len) {
            zmode = 1;
            const kz: u5 = @intCast(@clz(mb) - 24 + ((mb + 16) >> 6));
            const mz = ((@as(u32, 1) << kz) - 1) & wb;
            const run = try refGetChecked(data, pos, mz, kz, 16);
            if (c + run > out.len) return error.RunTooLong;
            @memset(out[c .. c + run], 0);
            c += run;
            if (run >= 65535) zmode = 0;
            mb = 0;
        }
    }
}

test "rice_decompress decodes what the reference decodes, means, zero runs and overruns" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0xDEC0);
    const rnd = prng.random();
    const slots: u32 = 0x3000;
    const xbase: u32 = 0x200;
    var checked: usize = 0;
    var overruns: usize = 0;
    var runs: usize = 0;
    for (0..300) |_| {
        var data: [512]u8 = undefined;
        const density = rnd.intRangeAtMost(u8, 1, 255);
        for (&data) |*b| {
            var v: u8 = 0;
            for (0..8) |_| v = (v << 1) | @intFromBool(rnd.int(u8) < density);
            b.* = v;
        }
        const pb = rnd.intRangeAtMost(u32, 0, 127);
        const kb = rnd.intRangeAtMost(u5, 1, 15);
        const mb = if (rnd.boolean()) rnd.intRangeAtMost(u32, 0, 255) else rnd.intRangeAtMost(u32, 0, (1 << 22) - 1);
        const max_bits = rnd.intRangeAtMost(u5, 8, 22);
        const n = rnd.intRangeAtMost(usize, 1, 200);

        var want: [200]i32 = undefined;
        var pos: usize = 0;
        const res = refDecompress(&data, &pos, want[0..n], pb, kb, mb, max_bits);
        if (res == error.EndOfStream) continue;

        const prog = try assemble(gpa,
            \\.include "lib/bits.sm"
            \\.include "lib/rice.sm"
            \\        bits_init(0x4000)
            \\        rice_slots(0x3000)
            \\        rice_decompress(0x3000)
            \\        getbits(s8, 8)
            \\        x17.halt
            \\
        );
        defer gpa.free(prog);

        var state = emu.State{};
        defer state.deinit(gpa);
        try stage(gpa, &state, &data);
        const init = [_]u32{ mb >> 15, mb & 0x7FFF, 0, @intCast(n), pb, kb, max_bits, 0 };
        for (init, 0..) |v, i| try state.y_mem.put(gpa, slots + @as(u32, @intCast(i)), v);
        state.a[6] = xbase;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 24);
        try testing.expect(state.halted);

        const err = state.y_mem.get(slots + 7).?;
        if (res) |_| {
            try testing.expectEqual(@as(u32, 0), err);
            for (want[0..n], 0..) |w, i| {
                const got = state.x_mem.get(xbase + @as(u32, @intCast(i))) orelse return error.Missing;
                try testing.expectEqual(@as(u32, @bitCast(w)) & 0xFFFFFF, got);
            }
            try testing.expectEqual(xbase + @as(u32, @intCast(n)), state.a[6]);
            if (pos + 8 <= data.len * 8) try testing.expectEqual(refRead(&data, &pos, 8), state.s[8]);
            checked += 1;
            for (want[0..n]) |w| runs += @intFromBool(w == 0);
        } else |_| {
            try testing.expectEqual(@as(u32, 1), err);
            overruns += 1;
        }
    }
    try testing.expect(checked > 100 and overruns > 5 and runs > 100);
}

fn wrapCb(x: i32, cb: u6) i32 {
    const s: u5 = @intCast(32 - @as(u32, cb));
    return (x << s) >> s;
}

fn sign32(x: i32) i32 {
    return @as(i32, @intFromBool(x > 0)) - @intFromBool(x < 0);
}

/// sosumi ref/alac/predictor.zig `unpredict`, 32-bit as Apple's reference computes it.
fn refUnpredict(pc: []const i32, out: []i32, coefs: []i16, cb: u6, ds: u4) void {
    const n = pc.len;
    out[0] = pc[0];
    const order = coefs.len;
    if (order == 0) {
        @memcpy(out[1..n], pc[1..n]);
        return;
    }
    const half: i32 = if (ds == 0) 0 else @as(i32, 1) << (ds - 1);
    var j: usize = 1;
    while (j <= order and j < n) : (j += 1) out[j] = wrapCb(pc[j] +% out[j - 1], cb);
    const lim = order + 1;
    while (j < n) : (j += 1) {
        const top = out[j - lim];
        var sum: i32 = 0;
        for (0..order) |k| sum +%= @as(i32, coefs[k]) *% (out[j - 1 - k] -% top);
        var del = pc[j];
        var del0 = del;
        const sg = sign32(del);
        del +%= top +% ((sum +% half) >> ds);
        out[j] = wrapCb(del, cb);
        if (sg == 0) continue;
        var k = order;
        while (k > 0) {
            k -= 1;
            const dd = top -% out[j - 1 - k];
            const sgn = sign32(dd);
            const w: i32 = @intCast(order - k);
            if (sg > 0) {
                coefs[k] -%= @intCast(sgn);
                del0 -%= w *% ((sgn *% dd) >> ds);
                if (del0 <= 0) break;
            } else {
                coefs[k] +%= @intCast(sgn);
                del0 -%= w *% ((-sgn *% dd) >> ds);
                if (del0 >= 0) break;
            }
        }
    }
}

fn supportedPred(cb: u6, ds: u4) bool {
    return cb <= 23 and @as(u32, ds) + cb <= 32 and cb <= 18 + @as(u32, ds);
}

test "unpredict matches the reference: samples and adapted coefficients" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x9ED1);
    const rnd = prng.random();
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\        bits_init(0x4000)
        \\        ldi s9, #0x5A5A
        \\        ldi s6, #6
        \\        x19.alu s9, s9, s6, #SHIFT
        \\        ldi s10, #13
        \\        pred_slots(0x3100)
        \\        unpredict()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    const pc_base: u32 = 0x1000;
    const out_base: u32 = 0x2000;
    const c_base: u32 = 0x0F00;
    var cases: usize = 0;
    while (cases < 250) {
        const cb = rnd.intRangeAtMost(u6, 8, 23);
        const ds = rnd.int(u4);
        if (!supportedPred(cb, ds)) continue;
        cases += 1;
        const order = if (cases < 10) cases - 1 else rnd.intRangeAtMost(usize, 0, 31);
        const n = rnd.intRangeAtMost(usize, 1, 96);
        const spread = rnd.intRangeAtMost(u5, 0, @intCast(cb - 1));
        var pc: [96]i32 = undefined;
        const lim: i32 = @as(i32, 1) << spread;
        for (pc[0..n]) |*x| x.* = if (rnd.int(u8) < 40) 0 else rnd.intRangeAtMost(i32, -lim, lim);
        var coefs: [31]i16 = undefined;
        for (coefs[0..order]) |*c| c.* = rnd.int(i16) >> rnd.intRangeAtMost(u4, 0, 12);
        // the 16-bit wrap of the adaptation: coefficients at the edges
        for (coefs[0..order]) |*c| {
            if (rnd.int(u8) < 40) c.* = if (rnd.boolean()) std.math.maxInt(i16) else std.math.minInt(i16);
        }
        const c_asm = coefs;

        var want: [96]i32 = undefined;
        refUnpredict(pc[0..n], want[0..n], coefs[0..order], cb, ds);

        var state = emu.State{};
        defer state.deinit(gpa);
        for (pc[0..n], 0..) |x, i| try state.x_mem.put(gpa, pc_base + @as(u32, @intCast(i)), @as(u32, @bitCast(x)) & 0xFFFFFF);
        for (0..order) |i| try state.x_mem.put(gpa, c_base + @as(u32, @intCast(i)), @as(u32, @bitCast(@as(i32, c_asm[order - 1 - i]))) & 0xFFFFFF);
        const init = [_]u32{ @intCast(n), @intCast(order), ds, cb, 0 };
        for (init, 0..) |v, i| try state.y_mem.put(gpa, 0x3100 + @as(u32, @intCast(i)), v);
        state.a[2] = pc_base;
        state.a[3] = out_base;
        state.a[10] = c_base;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 26);
        try testing.expect(state.halted);
        // the bit reader survives (set after bits_init): unpredict parks it past the coefficients
        try testing.expectEqual(@as(u32, 0x5A5A << 6), state.s[9]);
        try testing.expectEqual(@as(u32, 13), state.s[10]);

        for (want[0..n], 0..) |w, i| {
            const got = state.x_mem.get(out_base + @as(u32, @intCast(i))) orelse return error.Missing;
            if (got != @as(u32, @bitCast(w)) & 0xFFFFFF) {
                std.debug.print("sample {d}: order={d} n={d} cb={d} ds={d}\n", .{ i, order, n, cb, ds });
                try testing.expectEqual(@as(u32, @bitCast(w)) & 0xFFFFFF, got);
            }
        }
        for (0..order) |i| {
            const got = state.x_mem.get(c_base + @as(u32, @intCast(i))).?;
            try testing.expectEqual(@as(u32, @bitCast(@as(i32, coefs[order - 1 - i]))) & 0xFFFFFF, got);
        }
    }
}

test "unmix matches the reference: matrixing, low bytes, depth wrap, gaps" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x0A1C);
    const rnd = prng.random();
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\.include "lib/unmix.sm"
        \\        bits_init(0x4000)
        \\        unmix_slots(0x3200)
        \\        unmix()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    const u_base: u32 = 0x1000;
    const v_base: u32 = 0x1400;
    const low_base: u32 = 0x1800;
    const out_base: u32 = 0x2000;
    var cases: usize = 0;
    while (cases < 400) {
        const lanes: u32 = if (rnd.int(u8) < 200) 2 else 1;
        const depth = rnd.intRangeAtMost(u32, 8, 24);
        const shifted: u32 = 8 * rnd.intRangeAtMost(u32, 0, 2);
        if (shifted >= depth) continue;
        const cb = depth - shifted + @intFromBool(lanes == 2);
        if (cb > 23) continue;
        cases += 1;
        const n = rnd.intRangeAtMost(usize, 1, 40);
        const mix_bits = rnd.intRangeAtMost(u32, 0, 31);
        const mix_res: i8 = if (rnd.int(u8) < 30) 0 else rnd.int(i8);
        const gap = rnd.intRangeAtMost(u32, 0, 3);
        const nch = lanes + gap;

        var mix: [2][40]i32 = undefined;
        var low: [80]u32 = undefined;
        for (0..lanes) |l| for (mix[l][0..n]) |*x| {
            x.* = wrapCb(rnd.int(i32), @intCast(cb));
        };
        for (low[0 .. n * lanes]) |*x| x.* = rnd.int(u32) & ((@as(u32, 1) << @intCast(shifted)) - 1);

        var want: [200]i32 = @splat(0);
        const out_shift: u5 = @intCast(32 - depth);
        for (0..n) |i| {
            var sv: [2]i32 = undefined;
            if (lanes == 2 and mix_res != 0) {
                const u = mix[0][i];
                const v = mix[1][i];
                sv[0] = u +% v -% ((@as(i32, mix_res) *% v) >> @intCast(mix_bits));
                sv[1] = sv[0] -% v;
            } else {
                for (0..lanes) |l| sv[l] = mix[l][i];
            }
            for (0..lanes) |l| {
                var ux: u32 = @bitCast(sv[l]);
                if (shifted != 0) ux = (ux << @intCast(shifted)) | low[i * lanes + l];
                want[i * nch + l] = @as(i32, @bitCast(ux << out_shift)) >> out_shift;
            }
        }

        var state = emu.State{};
        defer state.deinit(gpa);
        for (0..n) |i| {
            const k: u32 = @intCast(i);
            try state.x_mem.put(gpa, u_base + k, @as(u32, @bitCast(mix[0][i])) & 0xFFFFFF);
            if (lanes == 2) try state.x_mem.put(gpa, v_base + k, @as(u32, @bitCast(mix[1][i])) & 0xFFFFFF);
        }
        for (low[0 .. n * lanes], 0..) |x, i| try state.x_mem.put(gpa, low_base + @as(u32, @intCast(i)), x);
        for (0..n * nch) |i| try state.x_mem.put(gpa, out_base + @as(u32, @intCast(i)), 0);
        const init = [_]u32{ @intCast(n), mix_bits, @as(u8, @bitCast(mix_res)), shifted, depth, lanes, 0, gap };
        for (init, 0..) |v, i| try state.y_mem.put(gpa, 0x3200 + @as(u32, @intCast(i)), v);
        state.a[2] = u_base;
        state.a[3] = v_base;
        state.a[4] = low_base;
        state.a[7] = out_base;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 24);
        try testing.expect(state.halted);
        for (want[0 .. n * nch], 0..) |w, i| {
            const got = state.x_mem.get(out_base + @as(u32, @intCast(i))).?;
            if (got != @as(u32, @bitCast(w)) & 0xFFFFFF) {
                std.debug.print("word {d}: lanes={d} depth={d} shifted={d} mix_bits={d} mix_res={d} gap={d}\n", .{ i, lanes, depth, shifted, mix_bits, mix_res, gap });
                try testing.expectEqual(@as(u32, @bitCast(w)) & 0xFFFFFF, got);
            }
        }
    }
}

fn le32(b: []const u8, at: usize) u32 {
    return std.mem.readInt(u32, b[at..][0..4], .little);
}

/// Packets dumped by sosumi's `alacdec --dump` (the reference decoder's PCM alongside).
fn checkPackets(gpa: std.mem.Allocator, prog: []const u32, dump: []const u8, name: []const u8) !void {
    try testing.expectEqualSlices(u8, "ALPK", dump[0..4]);
    const cfg = [6]u32{ le32(dump, 4), le32(dump, 8), le32(dump, 12), le32(dump, 16), le32(dump, 20), le32(dump, 24) };
    const count = le32(dump, 28);
    var p: usize = 32;
    for (0..count) |k| {
        const len = le32(dump, p);
        const pkt = dump[p + 4 .. p + 4 + len];
        p += 4 + len;
        const n = le32(dump, p);
        p += 4;
        const nch = cfg[5];
        const pcm = dump[p .. p + 4 * n * nch];
        p += 4 * n * nch;

        var state = emu.State{};
        defer state.deinit(gpa);
        const padded = try gpa.alloc(u8, len + (len & 1));
        defer gpa.free(padded);
        @memcpy(padded[0..len], pkt);
        if (len & 1 != 0) padded[len] = 0;
        try stage(gpa, &state, padded);
        for (cfg, 0..) |v, i| try state.y_mem.put(gpa, 0x3300 + @as(u32, @intCast(i)), v);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 30);
        try testing.expect(state.halted);
        const status = state.y_mem.get(0x3307).?;
        if (status != 0) std.debug.print("{s} packet {d}: status {d}\n", .{ name, k, status });
        try testing.expectEqual(@as(u32, 0), status);
        try testing.expectEqual(n, state.y_mem.get(0x3306).?);
        for (0..n * nch) |i| {
            const want = le32(pcm, 4 * i) & 0xFFFFFF;
            const got = state.x_mem.get(0x6000 + @as(u32, @intCast(i))) orelse 0xDEAD0000;
            if (got != want) {
                std.debug.print("{s} packet {d}: word {d} (frame {d} ch {d})\n", .{ name, k, i, i / nch, i % nch });
                try testing.expectEqual(want, got);
            }
        }
    }
}

test "alac_packet decodes fixture packets as the reference does" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\.include "lib/unmix.sm"
        \\.include "lib/alac.sm"
        \\        bits_init(0x4000)
        \\        alac_packet()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    inline for (.{ "mono.at", "tone.at", "white24.at", "clicks.c0.ff", "clicks.o30.ff", "mono.at.int", "tone.at.int" }) |name| {
        try checkPackets(gpa, prog, @embedFile(name ++ ".pkts"), name);
    }
}

test "integrate matches the reference: running sum wrapped to chan_bits" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x1A7E);
    const rnd = prng.random();
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\        bits_init(0x4000)
        \\        pred_slots(0x3100)
        \\        integrate()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    for (0..200) |case| {
        const cb = rnd.intRangeAtMost(u6, 8, 23);
        const n: usize = if (case < 3) case else rnd.intRangeAtMost(usize, 1, 80);
        var buf: [80]i32 = undefined;
        for (buf[0..n]) |*x| x.* = wrapCb(rnd.int(i32), cb);
        var state = emu.State{};
        defer state.deinit(gpa);
        for (buf[0..n], 0..) |x, i| try state.x_mem.put(gpa, 0x1000 + @as(u32, @intCast(i)), @as(u32, @bitCast(x)) & 0xFFFFFF);
        const init = [_]u32{ @intCast(n), 0, 0, cb, 0 };
        for (init, 0..) |v, i| try state.y_mem.put(gpa, 0x3100 + @as(u32, @intCast(i)), v);
        state.a[2] = 0x1000;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 22);
        try testing.expect(state.halted);
        var j: usize = 1;
        while (j < n) : (j += 1) buf[j] = wrapCb(buf[j] +% buf[j - 1], cb);
        for (buf[0..n], 0..) |w, i| try testing.expectEqual(@as(u32, @bitCast(w)) & 0xFFFFFF, state.x_mem.get(0x1000 + @as(u32, @intCast(i))).?);
    }
}

test "sxe sign-extends any 24-bit word from k bits, the edges included" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x5E8E);
    const rnd = prng.random();
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\        bits_init(0x4000)
        \\        pred_slots(0x3100)
        \\        x19.xrd s3, #2
        \\        sxcb(s3)
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    const edges = [_]u32{ 0, 1, 0x7FFFFF, 0x800000, 0x800001, 0xFFFFFF, 0x7FFF80, 0x7FFF7F, 0x400000, 0xC00000 };
    for (8..25) |k| {
        for (0..60) |t| {
            const x: u32 = if (t < edges.len) edges[t] else rnd.int(u24);
            var state = emu.State{};
            defer state.deinit(gpa);
            try state.x_mem.put(gpa, 0x1000, x);
            try state.y_mem.put(gpa, 0x3103, @intCast(k));
            state.a[2] = 0x1000;
            _ = try emu.runFrom(gpa, &state, prog, 0, 1000);
            const sh: u5 = @intCast(32 - k);
            const want: u32 = @as(u32, @bitCast(@as(i32, @bitCast(x << sh)) >> sh)) & 0xFFFFFF;
            if (want != state.s[3]) std.debug.print("k={d} x={x}\n", .{ k, x });
            try testing.expectEqual(want, state.s[3]);
        }
    }
}

test "the packet program has no long runs of 0x05 words" {
    // Silicon: 32 consecutive low5-0x05 words never halted (4 of 4); 8 in a row ran.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/rice.sm"
        \\.include "lib/predictor.sm"
        \\.include "lib/unmix.sm"
        \\.include "lib/alac.sm"
        \\        bits_init(0x4000)
        \\        alac_packet()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var run: usize = 0;
    for (prog) |w| {
        run = if (w & 0x1F == 0x05) run + 1 else 0;
        try testing.expect(run <= 8);
    }
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

/// Bit-by-bit match against the codewords.
fn refHuff(data: []const u8, pos: *usize, syms: []const HCode) u32 {
    var code: u32 = 0;
    var len: u5 = 0;
    while (true) {
        code = code << 1 | refRead(data, pos, 1);
        len += 1;
        for (syms) |s| if (s.len == len and s.code == code) return s.v;
    }
}

test "huff_decode decodes what a bit-by-bit matcher decodes, through subtables" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x4AFF);
    const rnd = prng.random();
    var deep: usize = 0;
    for (0..60) |_| {
        var cbuf: [300]HCode = undefined;
        const syms = randomCode(rnd, &cbuf);
        const root = rnd.intRangeAtMost(u4, 1, 8);
        var t: [8192]u16 = undefined;
        var n: usize = 0;
        _ = buildTables(&t, &n, syms, 0, root);
        if (n > 1024) continue; // pointer bases must stay below 1024 too
        var data: [256]u8 = undefined;
        rnd.bytes(&data);
        const skip = rnd.intRangeAtMost(u5, 0, 23);
        const count = 12;

        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        try src.print(gpa,
            \\.include "lib/bits.sm"
            \\.include "lib/huffman.sm"
            \\        bits_init(0x4000)
            \\        ldi s7, #{d}
            \\        readn(s4)
            \\
        , .{skip});
        for (0..count) |_| try src.print(gpa,
            \\        ldi s1, #0x1000
            \\        ldi s11, #{d}
            \\        huff_decode(s1)
            \\        stx A6, s1
            \\        x02.ainc #6, #6
            \\
        , .{root});
        try src.appendSlice(gpa, "        getbits(s8, 8)\n        x17.halt\n");
        const prog = try assemble(gpa, src.items);
        defer gpa.free(prog);

        var state = emu.State{};
        defer state.deinit(gpa);
        try stage(gpa, &state, &data);
        for (t[0..n], 0..) |w, i| try state.y_mem.put(gpa, 0x1000 + @as(u32, @intCast(i)), w);
        state.a[6] = 0x100;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 20);
        try testing.expect(state.halted);

        var pos: usize = skip;
        for (0..count) |i| {
            const before = pos;
            const want = refHuff(&data, &pos, syms);
            if (pos - before > root) deep += 1;
            try testing.expectEqual(want, state.readX(0x100 + @as(u32, @intCast(i))) orelse 0xDEAD);
        }
        try testing.expectEqual(refRead(&data, &pos, 8), state.s[8]);
    }
    try testing.expect(deep > 50);
}

const Family = struct { name: []const u8, dim: u3, fbits: u3, off: u5, max: u5, signs: bool, esc: bool };

const families = [_]Family{
    .{ .name = "spec_quad_s", .dim = 4, .fbits = 2, .off = 1, .max = 2, .signs = false, .esc = false },
    .{ .name = "spec_quad_u", .dim = 4, .fbits = 2, .off = 0, .max = 2, .signs = true, .esc = false },
    .{ .name = "spec_pair_s", .dim = 2, .fbits = 5, .off = 4, .max = 8, .signs = false, .esc = false },
    .{ .name = "spec_pair_u", .dim = 2, .fbits = 5, .off = 0, .max = 12, .signs = true, .esc = false },
    .{ .name = "spec_pair_esc", .dim = 2, .fbits = 5, .off = 0, .max = 16, .signs = true, .esc = true },
};

/// sosumi ref/dsp/aac/huffman.zig `decodeSpectral` for one family, plain integers.
fn refSpectral(data: []const u8, pos: *usize, syms: []const HCode, f: Family, out: []i32) error{EscapeTooLong}!void {
    var v: u32 = refHuff(data, pos, syms);
    var k: usize = f.dim;
    while (k > 0) {
        k -= 1;
        out[k] = @as(i32, @intCast(v & ((@as(u32, 1) << f.fbits) - 1))) - f.off;
        v >>= f.fbits;
    }
    if (f.signs) for (out) |*x| {
        if (x.* != 0 and refRead(data, pos, 1) == 1) x.* = -x.*;
    };
    if (f.esc) for (out) |*x| {
        if (@abs(x.*) != 16) continue;
        var n: u5 = 0;
        while (refRead(data, pos, 1) == 1) {
            n += 1;
            if (n > 8) return error.EscapeTooLong;
        }
        const mag: i32 = @intCast((@as(u32, 1) << (n + 4)) + refRead(data, pos, n + 4));
        x.* = if (x.* < 0) -mag else mag;
    };
}

test "the spectral macros unpack, sign and escape as the model does" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x5BEC);
    const rnd = prng.random();
    var escapes: usize = 0;
    var fails: usize = 0;
    var exact9: usize = 0;
    for (families) |f| {
        for (0..@as(usize, if (f.esc) 200 else 30)) |run| {
            // a random complete code over random packed values in the family's range
            var cbuf: [300]HCode = undefined;
            const syms = randomCode(rnd, cbuf[0..rnd.intRangeAtMost(usize, 2, 120)]);
            for (syms, 0..) |*s, i| {
                var v: u16 = 0;
                for (0..f.dim) |_| v = (v << f.fbits) + rnd.intRangeAtMost(u16, 0, f.max);
                if (f.esc and i < 3) v = (16 << 5) + @as(u16, @intCast(i)) * 16; // force some escapes
                s.v = v;
            }
            var t: [8192]u16 = undefined;
            var n: usize = 0;
            _ = buildTables(&t, &n, syms, 0, 8);
            if (n > 1024) continue;
            var data: [256]u8 = undefined;
            rnd.bytes(&data);
            // mostly short escape prefixes; dense ones on half the escape runs reach N > 8
            const dense = f.esc and run % 2 == 1;
            for (&data) |*b| b.* = if (dense) b.* | rnd.int(u8) else b.* & (rnd.int(u8) | rnd.int(u8));
            const count = 8;

            var src: std.ArrayList(u8) = .empty;
            defer src.deinit(gpa);
            try src.appendSlice(gpa, ".include \"lib/bits.sm\"\n.include \"lib/huffman.sm\"\n        bits_init(0x4000)\n");
            for (0..count) |_| {
                try src.appendSlice(gpa, "        ldi s1, #0x1000\n        ldi s11, #8\n");
                if (f.esc) try src.print(gpa, "        {s}(fail)\n", .{f.name}) else try src.print(gpa, "        {s}()\n", .{f.name});
            }
            try src.appendSlice(gpa,
                \\        getbits(s8, 8)
                \\        x17.halt
                \\        nop()
                \\        nop()
                \\fail:   ldi s1, #0x777
                \\        stx A6, s1
                \\        x17.halt
                \\
            );
            const prog = try assemble(gpa, src.items);
            defer gpa.free(prog);

            var state = emu.State{};
            defer state.deinit(gpa);
            try stage(gpa, &state, &data);
            for (t[0..n], 0..) |w, i| try state.y_mem.put(gpa, 0x1000 + @as(u32, @intCast(i)), w);
            state.a[6] = 0x100;
            _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 20);
            try testing.expect(state.halted);

            var pos: usize = 0;
            var at: u32 = 0x100;
            const failed = for (0..count) |_| {
                var out: [4]i32 = undefined;
                refSpectral(&data, &pos, syms, f, out[0..f.dim]) catch {
                    var peek = pos;
                    if (refRead(&data, &peek, 1) == 0) exact9 += 1; // nine ones, then the zero
                    break true;
                };
                for (out[0..f.dim]) |x| {
                    if (@abs(x) > 16) escapes += 1;
                    try testing.expectEqual(@as(u32, @bitCast(x)) & 0xFFFFFF, state.readX(at) orelse 0xDEAD);
                    at += 1;
                }
            } else false;
            if (failed) {
                fails += 1;
                try testing.expectEqual(@as(u32, 0x777), state.readX(at) orelse 0xDEAD);
            } else try testing.expectEqual(refRead(&data, &pos, 8), state.s[8]);
        }
    }
    try testing.expect(escapes > 10);
    try testing.expect(fails > 2);
    try testing.expect(exact9 > 0); // N = 9 itself fails, not only N >= 10
}

test "no test program puts a label inside the halt's two-bundle shadow" {
    // fact 72: two bundles after x17.halt still run, so code placed there (a fail
    // marker, say) runs on the success path too.
    var lines = std.mem.splitScalar(u8, @embedFile("lib_test.zig"), '\n');
    var since: ?usize = null;
    while (lines.next()) |line| {
        const t = std.mem.trim(u8, line, " \\");
        if (std.mem.indexOf(u8, t, "x17.halt") != null and std.mem.indexOf(u8, t, "indexOf") == null) {
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

const SecShape = struct { short: bool, groups: u4, max_sfb: u6, gain: u8 };

/// sosumi ref/dsp/aac/sections.zig, plain integers, and where it fails: section data (0x777)
/// or a scale factor (0x778).
fn refSections(data: []const u8, pos: *usize, s: SecShape, syms: []const HCode, books: []i32, sf: []i32) u32 {
    const sb: u5 = if (s.short) 3 else 5;
    const esc: u32 = if (s.short) 7 else 31;
    var n: usize = 0;
    for (0..s.groups) |_| {
        var left: i32 = s.max_sfb;
        while (left != 0) {
            const cb: i32 = @intCast(refRead(data, pos, 4));
            if (cb == 12) return 0x777;
            var len: i32 = 0;
            while (true) {
                const incr = refRead(data, pos, sb);
                len += @intCast(incr);
                if (incr != esc) break;
            }
            left -= len;
            if (left < 0) return 0x777;
            for (0..@intCast(len)) |_| {
                books[n] = cb;
                n += 1;
            }
        }
    }
    books[n] = -1;
    var fac: i32 = s.gain;
    var noise: i32 = @as(i32, s.gain) - 90;
    var is_pos: i32 = 0;
    var pcm = true;
    for (books[0..n], sf[0..n]) |cb, *out| {
        if (cb == 0) {
            out.* = 0;
        } else if (cb == 13 and pcm) {
            pcm = false;
            noise += @as(i32, @intCast(refRead(data, pos, 9))) - 256;
            out.* = noise;
        } else {
            const d = @as(i32, @intCast(refHuff(data, pos, syms))) - 60;
            if (cb < 13) {
                fac += d;
                if (fac < 0 or fac > 255) return 0x778;
                out.* = fac;
            } else if (cb == 13) {
                noise += d;
                out.* = noise;
            } else {
                is_pos += d;
                out.* = is_pos;
            }
        }
    }
    return 0;
}

/// A plausible stream: sections, then scale factor codewords, now and then a reserved book, an
/// overrun or a scale factor outside 0..255; random bytes after.
fn writeSections(w: *BitWriter, rnd: std.Random, s: SecShape, code_of: []const HCode) void {
    const sb: u5 = if (s.short) 3 else 5;
    const esc: u32 = if (s.short) 7 else 31;
    var books: [121]u4 = undefined;
    var n: usize = 0;
    for (0..s.groups) |_| {
        var left: u32 = s.max_sfb;
        while (left > 0) {
            var cb = rnd.intRangeAtMost(u4, 0, 15);
            if (cb == 12 and rnd.uintLessThan(u8, 8) != 0) cb = 1;
            var len = rnd.intRangeAtMost(u32, 0, left);
            if (rnd.uintLessThan(u8, 60) == 0) len = left + 1 + rnd.uintLessThan(u32, 40);
            w.put(cb, 4);
            var l = len;
            while (l >= esc) : (l -= esc) w.put(esc, sb);
            w.put(l, sb);
            for (0..@min(len, left)) |_| {
                books[n] = cb;
                n += 1;
            }
            left -|= len;
        }
    }
    var fac: i32 = s.gain;
    var pcm = true;
    for (books[0..n]) |cb| {
        var d: i32 = rnd.intRangeAtMost(i32, -60, 60);
        switch (cb) {
            0 => continue,
            13 => if (pcm) {
                pcm = false;
                w.put(rnd.int(u9), 9);
                continue;
            },
            14, 15 => {},
            else => {
                d = rnd.intRangeAtMost(i32, -10, 10);
                if (rnd.uintLessThan(u8, 100) == 0) d = if (fac > 127) 60 else -60;
                if ((fac + d < 0 or fac + d > 255) and rnd.boolean()) d = -d;
                fac += d;
            },
        }
        const c = code_of[@intCast(d + 60)];
        w.put(c.code, c.len);
    }
    for (0..32) |_| w.put(rnd.int(u8), 8);
}

test "aac_sections and aac_scalefactors decode as the model does" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x5EC5);
    const rnd = prng.random();
    var ok: usize = 0;
    var failed: usize = 0;
    var grouped: usize = 0;
    var sf_fails: usize = 0;
    var runs: usize = 0;
    while (runs < 150) {
        var cbuf: [121]HCode = undefined;
        const syms = randomCodeN(rnd, &cbuf, 121);
        var code_of: [121]HCode = undefined;
        for (syms, 0..) |*c, i| {
            c.v = @intCast(i);
            code_of[i] = c.*;
        }
        var t: [8192]u16 = undefined;
        var n: usize = 0;
        _ = buildTables(&t, &n, syms, 0, 8);
        if (n > 1024) continue;
        runs += 1;
        const short = rnd.boolean();
        const s: SecShape = .{
            .short = short,
            .groups = if (short) rnd.intRangeAtMost(u4, 1, 8) else 1,
            .max_sfb = if (short) rnd.intRangeAtMost(u6, 0, 15) else rnd.intRangeAtMost(u6, 0, 51),
            .gain = rnd.int(u8),
        };
        var w: BitWriter = .{};
        writeSections(&w, rnd, s, &code_of);

        var src: std.ArrayList(u8) = .empty;
        defer src.deinit(gpa);
        try src.print(gpa,
            \\.include "lib/bits.sm"
            \\.include "lib/huffman.sm"
            \\.include "lib/sections.sm"
            \\        bits_init(0x4000)
            \\        ldi s1, #{d}
            \\        ldi s2, #{d}
            \\        aac_sections({d}, {d}, fail1)
            \\        ldi s1, #{d}
            \\        aac_scalefactors(0x1000, fail)
            \\        getbits(s8, 8)
            \\        x17.halt
            \\        nop()
            \\        nop()
            \\fail1:  ldi s8, #0x777
            \\        x17.halt
            \\        nop()
            \\        nop()
            \\fail:   ldi s8, #0x778
            \\        x17.halt
            \\
        , .{ s.groups, s.max_sfb, @as(u32, if (short) 3 else 5), @as(u32, if (short) 7 else 31), s.gain });
        const prog = try assemble(gpa, src.items);
        defer gpa.free(prog);

        var state = emu.State{};
        defer state.deinit(gpa);
        try stage(gpa, &state, &w.buf);
        for (t[0..n], 0..) |x, i| try state.y_mem.put(gpa, 0x1000 + @as(u32, @intCast(i)), x);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 22);
        try testing.expect(state.halted);

        var pos: usize = 0;
        var books: [121]i32 = undefined;
        var sf: [120]i32 = undefined;
        const fail = refSections(&w.buf, &pos, s, syms, &books, &sf);
        if (fail != 0) {
            if (fail == 0x778) sf_fails += 1;
            failed += 1;
            try testing.expectEqual(fail, state.s[8]);
            continue;
        }
        ok += 1;
        if (short and s.groups > 1) grouped += 1;
        const bands = @as(usize, s.groups) * s.max_sfb;
        for (books[0 .. bands + 1], 0..) |b, i| {
            try testing.expectEqual(@as(u32, @bitCast(b)) & 0xFFFF, state.getY(0x2000 + @as(u32, @intCast(i))) orelse 0xDEAD);
        }
        for (sf[0..bands], 0..) |v, i| {
            try testing.expectEqual(@as(u32, @bitCast(v)) & 0xFFFF, state.getY(0x2080 + @as(u32, @intCast(i))) orelse 0xDEAD);
        }
        try testing.expectEqual(refRead(&w.buf, &pos, 8), state.s[8]);
    }
    try testing.expect(ok > 90);
    try testing.expect(failed > 10);
    try testing.expect(grouped > 20);
    try testing.expect(sf_fails > 3);
}
