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
            \\        rice_decompress()
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
