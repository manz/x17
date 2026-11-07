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
    .{ .path = "lib/mem.sm", .text = @embedFile("mem.sm") },
    .{ .path = "lib/alac.sm", .text = @embedFile("alac.sm") },
    .{ .path = "lib/huffman.sm", .text = @embedFile("huffman.sm") },
    .{ .path = "lib/sections.sm", .text = @embedFile("sections.sm") },
    .{ .path = "lib/dequant.sm", .text = @embedFile("dequant.sm") },
    .{ .path = "lib/imdct.sm", .text = @embedFile("imdct.sm") },
    .{ .path = "lib/window.sm", .text = @embedFile("window.sm") },
    .{ .path = "lib/stereo.sm", .text = @embedFile("stereo.sm") },
    .{ .path = "lib/tns.sm", .text = @embedFile("tns.sm") },
    .{ .path = "lib/pns.sm", .text = @embedFile("pns.sm") },
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
        \\.include "lib/mem.sm"
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
        \\.include "lib/mem.sm"
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

const DqTables = struct {
    dm: [129]i32,
    de: [129]i32,
    it: [65]i32,
    g: [12]i32,
};

fn dqTables() DqTables {
    var t: DqTables = undefined;
    t.dm[0] = 0;
    t.de[0] = 0;
    for (1..129) |k| {
        const v = std.math.pow(f64, @floatFromInt(k), 4.0 / 3.0);
        var ex: i32 = @intFromFloat(@floor(std.math.log2(v)));
        var r = @round(std.math.ldexp(v, 22 - ex));
        if (r >= 1 << 23) {
            r /= 2;
            ex += 1;
        }
        t.dm[k] = @intFromFloat(r);
        t.de[k] = ex - 22;
    }
    for (&t.it, 64..) |*x, j| x.* = @intFromFloat(@round(std.math.pow(f64, @floatFromInt(j), 4.0 / 3.0) * 8192.0));
    for (0..3) |rem| for (0..4) |b| {
        const x = @as(f64, @floatFromInt(rem)) / 3.0 + @as(f64, @floatFromInt(b)) / 4.0;
        t.g[rem * 4 + b] = @intFromFloat(@round(std.math.pow(f64, 2.0, x) * (1 << 21)));
    };
    return t;
}

const dq_ie = [8]i32{ 0, 1 - 13, 2 - 13, 4 - 13, 5 - 13, 6 - 13, 8 - 13, 9 - 13 };
const dq_ir = [8]i32{ 0, 1, 2, 0, 1, 2, 0, 1 };

fn rshr(a: i64, n: u6) i64 {
    if (n == 0) return a;
    return (a + (@as(i64, 1) << (n - 1))) >> n;
}

const Mant = struct { m: i64, e: i32, rem: i32 };

fn dqMant(t: *const DqTables, mag: i32) Mant {
    if (mag <= 128) return .{ .m = t.dm[@intCast(mag)], .e = t.de[@intCast(mag)], .rem = 0 };
    var s: u6 = 1;
    var top: i32 = 256;
    while (mag >= top) : (top *= 2) s += 1;
    const j = rshr(mag - (@as(i64, 1) << (s - 1)), s);
    const f = mag - (j << s);
    const lo: i64 = t.it[@intCast(j - 64)];
    const d: i64 = t.it[@intCast(j - 63)] - lo;
    const fs = f << (23 - s);
    return .{ .m = lo + ((d * fs + (1 << 22)) >> 23), .e = dq_ie[s], .rem = dq_ir[s] };
}

const DqShape = struct { short: bool, groups: u4, group_len: [8]u4, max_sfb: u6 };

const swb_long_44 = [_]u16{ 0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 1024 };
const swb_short_44 = [_]u16{ 0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128 };

/// Words and exponent, as the model's dequantChannel.
fn refDequant(t: *const DqTables, spec: []const i32, s: DqShape, books: []const i32, sfs: []const i32, out: []i32) i32 {
    const offs: []const u16 = if (s.short) &swb_short_44 else &swb_long_44;
    const win: usize = if (s.short) 128 else 1024;
    var e_max: ?i32 = null;
    var fw: usize = 0;
    for (0..s.groups) |g| {
        for (0..s.max_sfb) |sfb| {
            const i = g * s.max_sfb + sfb;
            if (books[i] < 1 or books[i] > 11) continue;
            var mx: i32 = 0;
            for (fw..fw + s.group_len[g]) |w| for (w * win + offs[sfb]..w * win + offs[sfb + 1]) |k| {
                mx = @max(mx, @as(i32, @intCast(@abs(spec[k]))));
            };
            if (mx == 0) continue;
            const sh: i32 = @intCast(rshr(sfs[i] - 100 - 2, 2));
            const e = dqMant(t, mx).e + 2 + sh;
            if (e_max == null or e_max.? < e) e_max = e;
        }
        fw += s.group_len[g];
    }
    @memset(out, 0);
    const big = e_max orelse return 0;
    fw = 0;
    for (0..s.groups) |g| {
        for (0..s.max_sfb) |sfb| {
            const i = g * s.max_sfb + sfb;
            if (books[i] < 1 or books[i] > 11) continue;
            const tt = sfs[i] - 100;
            const sh: i32 = @intCast(rshr(tt - 2, 2));
            const b = tt - 4 * sh;
            for (fw..fw + s.group_len[g]) |w| for (w * win + offs[sfb]..w * win + offs[sfb + 1]) |k| {
                const q = spec[k];
                if (q == 0) continue;
                const mt = dqMant(t, @intCast(@abs(q)));
                const p = (mt.m * t.g[@intCast(mt.rem * 4 + b)] + (1 << 22)) >> 23;
                const n: u6 = @intCast(@min(big - (mt.e + 2 + sh), 23));
                const v: i32 = @intCast(rshr(p, n));
                out[k] = if (q < 0) -v else v;
            };
        }
        fw += s.group_len[g];
    }
    return big;
}

test "dq_channel dequantises as the model does: words and exponent" {
    const gpa = testing.allocator;
    var prng = std.Random.DefaultPrng.init(0xD0A);
    const rnd = prng.random();
    const t = dqTables();
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/huffman.sm"
        \\.include "lib/sections.sm"
        \\.include "lib/dequant.sm"
        \\        bits_init(0x4000)
        \\        dq_channel()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var nonzero: usize = 0;
    var interp: usize = 0;
    var shorts: usize = 0;
    for (0..40) |run| {
        const short = rnd.boolean();
        var s: DqShape = .{ .short = short, .groups = 1, .group_len = @splat(0), .max_sfb = 0 };
        if (short) {
            var left: u4 = 8;
            s.groups = 0;
            while (left > 0) {
                const l = rnd.intRangeAtMost(u4, 1, left);
                s.group_len[s.groups] = l;
                s.groups += 1;
                left -= l;
            }
            s.max_sfb = rnd.intRangeAtMost(u6, 0, 14);
            shorts += 1;
        } else {
            s.group_len[0] = 1;
            s.max_sfb = rnd.intRangeAtMost(u6, 0, 49);
        }
        const nb = @as(usize, s.groups) * s.max_sfb;
        var books: [120]i32 = undefined;
        var sfs: [120]i32 = undefined;
        const base_sf = rnd.intRangeAtMost(i32, 60, 200);
        for (0..nb) |i| {
            const cb = rnd.intRangeAtMost(i32, 0, 15);
            books[i] = if (cb == 12) 0 else cb;
            sfs[i] = std.math.clamp(base_sf + rnd.intRangeAtMost(i32, -30, 30), 0, 255);
        }
        var spec: [1024]i32 = undefined;
        for (&spec) |*q| {
            const r = rnd.uintLessThan(u8, 100);
            const mag: i32 = if (r < 40) 0 else if (r < 90) rnd.intRangeAtMost(i32, 1, 16) else if (r < 98) rnd.intRangeAtMost(i32, 17, 1000) else rnd.intRangeAtMost(i32, 1001, 8191);
            q.* = if (rnd.boolean()) -mag else mag;
            if (mag > 128) interp += 1;
        }
        // one run with nothing to dequantise
        if (run == 0) @memset(&spec, 0);
        var want: [1024]i32 = undefined;
        const want_e = refDequant(&t, &spec, s, books[0..nb], sfs[0..nb], &want);

        var state = emu.State{};
        defer state.deinit(gpa);
        const putX = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: u32, v: i64) !void {
                try st.x_mem.put(a, addr, @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        const putY = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: u32, v: i64) !void {
                try st.y_mem.put(a, addr, @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
            }
        }.f;
        for (0..1024) |k| try putX(&state, gpa, @intCast(k), spec[k]);
        for (0..129) |k| {
            try putX(&state, gpa, 0x1800 + 2 * @as(u32, @intCast(k)), t.dm[k]);
            try putX(&state, gpa, 0x1801 + 2 * @as(u32, @intCast(k)), t.de[k]);
        }
        for (t.it, 0..) |v, j| try putX(&state, gpa, 0x1910 + @as(u32, @intCast(j)), v);
        for (t.g, 0..) |v, j| try putX(&state, gpa, 0x1960 + @as(u32, @intCast(j)), v);
        for (0..8) |j| {
            try putY(&state, gpa, 0x2140 + @as(u32, @intCast(j)), dq_ie[j]);
            try putY(&state, gpa, 0x2148 + @as(u32, @intCast(j)), dq_ir[j] * 4);
        }
        const offs: []const u16 = if (short) &swb_short_44 else &swb_long_44;
        for (offs, 0..) |o, j| try putY(&state, gpa, 0x2180 + @as(u32, @intCast(j)), o);
        try putY(&state, gpa, 0x2150, s.groups);
        try putY(&state, gpa, 0x2151, s.max_sfb);
        try putY(&state, gpa, 0x2152, if (short) 128 else 1024);
        for (0..8) |j| try putY(&state, gpa, 0x2153 + @as(u32, @intCast(j)), s.group_len[j]);
        for (0..nb) |i| {
            try putY(&state, gpa, 0x2000 + @as(u32, @intCast(i)), books[i]);
            try putY(&state, gpa, 0x2080 + @as(u32, @intCast(i)), sfs[i]);
        }
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 24);
        try testing.expect(state.halted);
        try testing.expectEqual(@as(u32, @bitCast(want_e)) & 0xFFFF, state.getY(0x215B) orelse 0xDEAD);
        for (0..1024) |k| {
            if (want[k] != 0) nonzero += 1;
            try testing.expectEqual(@as(u32, @bitCast(want[k])) & 0xFFFFFF, state.readX(0x400 + @as(u32, @intCast(k))) orelse 0xDEAD);
        }
    }
    try testing.expect(nonzero > 5000);
    try testing.expect(interp > 1000);
    try testing.expect(shorts > 10);
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
    // Upstream, fact 468: 16E29474 42382CD8 (s7 <- #0x14A3), then the st pair.
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
    // Upstream, fact 470: ldi s1 <- 0x1234, lde e7 <- 0x710, st pair: Y:0x1234.
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
    // fact, Upstream: 00000018 00000000 70006000 C0000100, branch on p7 = 0: p3 <- 1.
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

test "x18 class-110 p write on the multiply base: Upstream's gates" {
    // fact 469: own-x18pinv also writes s2 <- lo(s1 * s1 << 1), s1 <- hi.
    var st = try runX18(.{ 0x1C000018, 0x00000000, 0x02002222, 0xC4114001 }, 1, 1);
    defer st.deinit(testing.allocator);
    try testing.expectEqual(@as(u8, 0), st.p[1]);
    try testing.expectEqual(@as(u32, 0x9592), st.s[2]);
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
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &img, &.{ 0x80800018, XXXXXXXX, 0x0000A97B, 0x00048200, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
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
        _ = try emu.runFrom(gpa, &state, &.{ 0x00400018, c.mux << 12, 0x00000000, 0xE6000000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(c.want, state.s[2]);
    }
}

test "pc14 class-011 mux 10 writes the immediate, and group 0 code 0 sets its p" {
    // fact 486: 461's "0 after a st.y" was a readback artifact; the immediate is
    // unconditional. 487: the pair XXXXXXXX 72000060 sets p6; other p cells are refused.
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
    _ = try emu.runFrom(gpa, &img, &.{ XXXXXXXX, 0x72000060, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0x1A), img.s[3]);
    try testing.expectEqual(@as(u8, 1), img.p[6]);
    var c9 = emu.State{};
    defer c9.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &c9, &.{ XXXXXXXX, 0x72000960, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
}

test "rimm's s write and x16 mux 3 are refused: they depend on state not yet found" {
    // fact 488: rimm XXXXXXXX A1008000 writes s2 = 0x0700 before any st.y and 0x5000
    // after; x16 mux 3 varies from run to run.
    const gpa = testing.allocator;
    var state = emu.State{};
    defer state.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &state, &.{ XXXXXXXX, 0xA1008000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    var r = emu.State{};
    defer r.deinit(gpa);
    _ = try emu.runFrom(gpa, &r, &.{ 0x0000000F, 0xA1008000, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    const prog = try assemble(gpa, "x16.mux s3, R3, #1, #0, #6, #4, #3\nx17.halt\n");
    defer gpa.free(prog);
    var m = emu.State{};
    defer m.deinit(gpa);
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &m, prog, 0, 16));
}

test "x17 S2 class 101 with S1[28] = 0 and mux 4 loads 0; other muxes are refused" {
    // fact 490 measured mux 4 (XXXXXXXX XXXXXXXX XXXXXXXX) in plain programs; here on
    // own-x17st's shape with S1[28] cleared and a load into s2.
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
    try testing.expectError(error.UnknownForm, emu.runFrom(gpa, &m2, &.{ 0x00400017, 0x67018000, 0xA0000044, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16));
    var w = emu.State{};
    defer w.deinit(gpa);
    _ = try emu.runFrom(gpa, &w, &.{ XXXXXXXX, XXXXXXXX, XXXXXXXX, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
    try testing.expectEqual(@as(u32, 0), w.s[15]);
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

const ImCx = struct { re: i64, im: i64 };

fn imQ23(x: f64) i64 {
    return @intFromFloat(std.math.clamp(@round(x * 0x1p23), -0x1p23, 0x1p23 - 1));
}

fn imExpj(theta: f64) ImCx {
    return .{ .re = imQ23(@cos(theta)), .im = imQ23(@sin(theta)) };
}

fn ImTables(comptime n2: usize) type {
    return struct {
        const n4 = n2 / 2;
        const stages = std.math.log2_int(usize, n4);
        pre: [n4]ImCx,
        post: [n4]ImCx,
        fft: [n4 / 2]ImCx,
        /// the x17 layout: stage by stage, size / 2 twiddles each
        tw: [n4 - 1]ImCx,
        rev: [n4]u32,
        fn init() @This() {
            var t: @This() = undefined;
            const pi = std.math.pi;
            const fn2: f64 = @floatFromInt(n2);
            for (&t.pre, 0..) |*e, n| e.* = imExpj(-pi * (@as(f64, @floatFromInt(n)) + 0.25) / fn2);
            for (&t.post, 0..) |*e, k| e.* = imExpj(-pi * @as(f64, @floatFromInt(k)) / fn2);
            for (&t.fft, 0..) |*e, k| e.* = imExpj(-2 * pi * @as(f64, @floatFromInt(k)) / @as(f64, @floatFromInt(n4)));
            var i: usize = 0;
            var size: usize = 2;
            while (size <= n4) : (size *= 2) {
                for (0..size / 2) |j| {
                    t.tw[i] = t.fft[j * (n4 / size)];
                    i += 1;
                }
            }
            for (&t.rev, 0..) |*e, n| e.* = @bitReverse(@as(@Int(.unsigned, stages), @intCast(n)));
            return t;
        }
    };
}

fn imMul(a: i64, b: i64) i64 {
    return (a * b + (1 << 22)) >> 23;
}

fn imCmul(a: ImCx, b: ImCx) ImCx {
    return .{ .re = imMul(a.re, b.re) - imMul(a.im, b.im), .im = imMul(a.re, b.im) + imMul(a.im, b.re) };
}

fn imAtLeast(a: i64, k: u6) bool {
    return @abs(a) >= @as(u64, 1) << k;
}

/// The model's shiftSat: v * 2^sh, rounded, saturated by comparisons.
fn imShiftSat(v: i64, sh: i64) i64 {
    if (v == 0 or sh <= -24) return 0;
    if (sh < 0) return rshr(v, @intCast(-sh));
    const top: i64 = if (v < 0) -(1 << 23) else (1 << 23) - 1;
    if (sh >= 23 or imAtLeast(v, @intCast(23 - sh))) return top;
    return v << @intCast(sh);
}

fn refImdct(comptime n2: usize, t: *const ImTables(n2), w: []const i64, e: i64, out: []i64) void {
    const n4 = n2 / 2;
    var big: u64 = 0;
    for (w) |v| big = @max(big, @abs(v));
    if (big == 0) {
        @memset(out, 0);
        return;
    }
    var x: [n2]i64 = undefined;
    var ex = e;
    if (big >= 1 << 22) {
        for (&x, w) |*d, v| d.* = rshr(v, 1);
        ex += 1;
    } else {
        var s: u6 = 0;
        while ((big << s) < 1 << 21) s += 1;
        for (&x, w) |*d, v| d.* = v << s;
        ex -= s;
    }
    var z: [n4]ImCx = undefined;
    for (0..n4) |n| z[t.rev[n]] = imCmul(.{ .re = x[2 * n], .im = x[n2 - 1 - 2 * n] }, t.pre[n]);
    var scaled: i64 = 0;
    var size: usize = 2;
    while (size <= n4) : (size *= 2) {
        var sh: u6 = 0;
        for (z) |c| {
            if (imAtLeast(c.re, 22) or imAtLeast(c.im, 22)) {
                sh = 2;
                break;
            }
            if (imAtLeast(c.re, 21) or imAtLeast(c.im, 21)) sh = 1;
        }
        if (sh != 0) for (&z) |*c| {
            c.* = .{ .re = rshr(c.re, sh), .im = rshr(c.im, sh) };
        };
        scaled += sh;
        const half = size / 2;
        var start: usize = 0;
        while (start < n4) : (start += size) {
            for (0..half) |j| {
                const a = z[start + j];
                const b = imCmul(z[start + j + half], t.fft[j * (n4 / size)]);
                z[start + j] = .{ .re = a.re + b.re, .im = a.im + b.im };
                z[start + j + half] = .{ .re = a.re - b.re, .im = a.im - b.im };
            }
        }
    }
    var u: [n2]i64 = undefined;
    for (z, 0..) |c0, k| {
        const c = imCmul(c0, t.post[k]);
        u[2 * k] = c.re;
        u[n2 - 1 - 2 * k] = -c.im;
    }
    const sh = scaled + ex - @as(i64, std.math.log2_int(usize, n2)) + 7;
    for (out, 0..) |*o, n| {
        const m = n + n4;
        const v = if (m < n2) u[m] else if (m < 2 * n2) -u[2 * n2 - 1 - m] else -u[m - 2 * n2];
        o.* = imShiftSat(v, sh);
    }
}

fn imdctRuns(comptime n2: usize, call: []const u8, seed: u64) !void {
    const gpa = testing.allocator;
    const T = ImTables(n2);
    const t = T.init();
    const src = try std.fmt.allocPrint(gpa, ".include \"lib/bits.sm\"\n.include \"lib/mem.sm\"\n.include \"lib/imdct.sm\"\n        bits_init(0x4000)\n        {s}\n        x17.halt\n", .{call});
    defer gpa.free(src);
    const prog = try assemble(gpa, src);
    defer gpa.free(prog);
    var prng = std.Random.DefaultPrng.init(seed);
    const rnd = prng.random();
    for (0..6) |run| {
        var w: [n2]i64 = undefined;
        // a falling spectrum; runs 0 and 3 near 2^22.3 (the halving), run 1 small
        // (doublings), run 2 silence, run 4 one positive peak, run 5 a large last coefficient
        const top: f64 = if (run == 1) 9 else 22.3;
        for (&w, 0..) |*v, k| {
            const amp = std.math.pow(f64, 2, top) / (1.0 + @as(f64, @floatFromInt(k)) / 8.0);
            v.* = @intFromFloat(@round((rnd.float(f64) * 2 - 1) * amp));
        }
        if (run == 2) @memset(&w, 0);
        // run 4: the largest word positive and alone, so the max decides the normalisation
        if (run == 4) {
            for (&w) |*v| v.* = @divTrunc(v.*, 64);
            w[3] = (1 << 22) + 12345;
        }
        // run 5: small noise under one large coefficient at n2 - 1, so the pre-twiddle's
        // imaginary part decides the first stage
        if (run == 5) {
            for (&w) |*v| v.* = @divTrunc(v.*, 64);
            w[n2 - 1] = (1 << 22) - 3;
        }
        const e: i64 = -20 + @as(i64, @intCast(run)) * 3;
        var want: [2 * n2]i64 = undefined;
        refImdct(n2, &t, &w, e, &want);

        var state = emu.State{};
        defer state.deinit(gpa);
        const px = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        const py = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.y_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
            }
        }.f;
        for (w, 0..) |v, k| try px(&state, gpa, 0x400 + k, v);
        for (t.pre, 0..) |c, k| {
            try px(&state, gpa, 0x1000 + 2 * k, c.re);
            try px(&state, gpa, 0x1001 + 2 * k, c.im);
        }
        for (t.post, 0..) |c, k| {
            try px(&state, gpa, 0x1400 + 2 * k, c.re);
            try px(&state, gpa, 0x1401 + 2 * k, c.im);
        }
        for (t.tw, 0..) |c, k| {
            try px(&state, gpa, 0x1A00 + 2 * k, c.re);
            try px(&state, gpa, 0x1A01 + 2 * k, c.im);
        }
        for (t.rev, 0..) |r, n| try py(&state, gpa, 0x2400 + n, 2 * @as(i64, r) - 2);
        try py(&state, gpa, 0x2390, e);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 26);
        try testing.expect(state.halted);
        for (want, 0..) |v, k| {
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF, state.readX(0x800 + @as(u32, @intCast(k))) orelse 0xDEAD);
        }
    }
}

test "imdct transforms a long block as the model does" {
    try imdctRuns(1024, "imdct(1024, 512, 10, 9, 0x0400, 0x0000, 0x0800, 0x1000, 0x1400, 0x1A00, 0x2400, 0x2390)", 0x1DC7);
}

test "imdct transforms a short block as the model does" {
    // the same layout; tables and buffers only use their first n2 / 4 .. 2 n2 words
    try imdctRuns(128, "imdct(128, 64, 7, 6, 0x0400, 0x0000, 0x0800, 0x1000, 0x1400, 0x1A00, 0x2400, 0x2390)", 0x5B7);
}

test "x02.ainc's template writes s2 and x18.rag's steps e7" {
    // Side writes of two image-row templates (subforms.txt); x18.rag101 has none.
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
    try testing.expectEqual(@as(u32, 0x30), state.s[2]);
    try testing.expectEqual(@as(u32, 0x2401), state.e[7]);
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
        const r = emu.runFrom(gpa, &state, &.{ 0x280011C1, 0x00000580, 0x28000020, 0xA00280A0, XXXXXXXX, c.w5, c.w6, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
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
        const r = emu.runFrom(gpa, &state, &.{ head, 0x00000580, 0x28000020 | (c.i << 6), 0xA00280A0, XXXXXXXX, 0x00200900 | (c.m << 27), 0x00300090, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
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
    // fact 507: own-x01m4w22 (row, w4[22] = 1, s8 = s11 = 0x4000 gives s8 = 0x20);
    // 508: own-x01ty (R7 = 0x100, head i = 3, op 11 gives s1 = 0xFD), op 10 the sum.
    const gpa = testing.allocator;
    const halt = [_]u32{ 0x00000017, 0x31308000, 0xA0000000 };
    {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[8] = 0x4000;
        state.s[11] = 0x4000;
        _ = try emu.runFrom(gpa, &state, &(.{ 0x28001001, 0x0000A580, 0x2800C000, 0xA00100E0, XXXXXXXX, 0x40200900, 0xB8080000 } ++ halt), 0, 16);
        try testing.expectEqual(@as(u32, 0x20), state.s[8]);
        // rounded (496): 0x4000 * 0x4100 = 2^28 + 2^22 gives 0x21
        state.s[8] = 0x4000;
        state.s[11] = 0x4100;
        state.pc = 0;
        state.halted = false;
        _ = try emu.runFrom(gpa, &state, &(.{ 0x28001001, 0x0000A580, 0x2800C000, 0xA00100E0, XXXXXXXX, 0x40200900, 0xB8080000 } ++ halt), 0, 16);
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

test "win_long windows, overlaps and clamps as the model does" {
    // sosumi ref/dsp/aac/window.zig, only_long: block = rnd(t w / 2^23) with w / 4 tables,
    // pcm = clamp(rshr(overlap + block, 5)), overlap = the second half.
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/window.sm"
        \\        bits_init(0x4000)
        \\        win_long(0x0000, 0x0800, 0x0C00, 0x1000, 0x2000)
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var prng = std.Random.DefaultPrng.init(0x3D2);
    const rnd = prng.random();
    for (0..3) |run| {
        var state = emu.State{};
        defer state.deinit(gpa);
        // run 0 quiet, 1 loud enough to clip both ways, 2 at the clamp edges
        const amp: i64 = if (run == 0) 1 << 20 else (1 << 23) - 1;
        var t: [2048]i64 = undefined;
        var wp: [1024]i64 = undefined;
        var wc: [1024]i64 = undefined;
        var ovl: [1024]i64 = undefined;
        for (&t) |*v| v.* = rnd.intRangeAtMost(i64, -amp, amp);
        for (&wp) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&wc) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&ovl) |*v| v.* = rnd.intRangeAtMost(i64, -(1 << 21), 1 << 21);
        if (run == 2) for (0..1024) |n| {
            // block 0, overlap straddling the rounding edges of 32767 and -32768
            wp[n] = 0;
            ovl[n] = if (n % 2 == 0) (32767 << 5) + 15 + @as(i64, @intCast(n % 4)) else -(32768 << 5) - 16 - @as(i64, @intCast(n % 4));
        };
        const put = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        for (t, 0..) |v, k| try put(&state, gpa, k, v);
        for (wp, 0..) |v, k| try put(&state, gpa, 0x800 + k, v);
        for (wc, 0..) |v, k| try put(&state, gpa, 0xC00 + k, v);
        for (ovl, 0..) |v, k| try put(&state, gpa, 0x1000 + k, v);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 22);
        try testing.expect(state.halted);
        for (0..1024) |n| {
            const b = imMul(t[n], wp[n]);
            const v = std.math.clamp(rshr(ovl[n] + b, 5), -32768, 32767);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF, (state.getY(0x2000 + @as(u32, @intCast(n))) orelse 0xDEAD) & 0xFFFF);
            const o = imMul(t[1024 + n], wc[1023 - n]);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(o)))) & 0xFFFFFF, state.readX(0x1000 + @as(u32, @intCast(n))) orelse 0xDEAD);
        }
    }
}

test "win_start, win_stop and win_short build the model's blocks" {
    // sosumi ref/dsp/aac/window.zig `block` and `synth` for long_start, long_stop and
    // eight_short, with arbitrary w / 4 tables (the routines do not depend on their values).
    const gpa = testing.allocator;
    const calls = [_][]const u8{
        "win_start(0x0000, 0x0800, 0x1080, 0x1100, 0x1900, 0x2000)",
        "win_stop(0x0000, 0x1000, 0x0C00, 0x1100, 0x1900, 0x2000)",
        "win_short(0x0000, 0x1000, 0x1080, 0x1100, 0x1900, 0x2000)",
    };
    var prng = std.Random.DefaultPrng.init(0x3D3);
    const rnd = prng.random();
    for (calls, 0..) |call, seq| for (0..2) |run| {
        const src = try std.fmt.allocPrint(gpa, ".include \"lib/bits.sm\"\n.include \"lib/mem.sm\"\n.include \"lib/window.sm\"\n        bits_init(0x4000)\n        {s}\n        x17.halt\n", .{call});
        defer gpa.free(src);
        const prog = try assemble(gpa, src);
        defer gpa.free(prog);
        const amp: i64 = if (run == 0) 1 << 20 else (1 << 23) - 1;
        var t: [2048]i64 = undefined;
        var wpl: [1024]i64 = undefined;
        var wcl: [1024]i64 = undefined;
        var wps: [128]i64 = undefined;
        var wcs: [128]i64 = undefined;
        var ovl: [1024]i64 = undefined;
        for (&t) |*v| v.* = rnd.intRangeAtMost(i64, -amp, amp);
        for (&wpl) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&wcl) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&wps) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&wcs) |*v| v.* = rnd.intRangeAtMost(i64, 0, 1 << 21);
        for (&ovl) |*v| v.* = rnd.intRangeAtMost(i64, -(1 << 21), 1 << 21);
        var b: [2048]i64 = @splat(0);
        switch (seq) {
            0 => for (0..2048) |n| {
                b[n] = if (n < 1024) imMul(t[n], wpl[n]) else if (n < 1472) rshr(t[n], 2) else if (n < 1600) imMul(t[n], wcs[1599 - n]) else 0;
            },
            1 => for (0..2048) |n| {
                b[n] = if (n < 448) 0 else if (n < 576) imMul(t[n], wps[n - 448]) else if (n < 1024) rshr(t[n], 2) else imMul(t[n], wcl[2047 - n]);
            },
            else => for (0..8) |w| for (0..128) |n| {
                const left = if (w == 0) &wps else &wcs;
                b[448 + 128 * w + n] += imMul(t[256 * w + n], left[n]);
                b[576 + 128 * w + n] += imMul(t[256 * w + 128 + n], wcs[127 - n]);
            },
        }
        var state = emu.State{};
        defer state.deinit(gpa);
        const put = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        for (t, 0..) |v, k| try put(&state, gpa, k, v);
        for (wpl, 0..) |v, k| try put(&state, gpa, 0x800 + k, v);
        for (wcl, 0..) |v, k| try put(&state, gpa, 0xC00 + k, v);
        for (wps, 0..) |v, k| try put(&state, gpa, 0x1000 + k, v);
        for (wcs, 0..) |v, k| try put(&state, gpa, 0x1080 + k, v);
        for (ovl, 0..) |v, k| try put(&state, gpa, 0x1900 + k, v);
        // B starts as garbage: every word must be written
        for (0..2048) |k| try put(&state, gpa, 0x1100 + k, 0x5A5A5A);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 23);
        try testing.expect(state.halted);
        for (0..1024) |n| {
            const v = std.math.clamp(rshr(ovl[n] + b[n], 5), -32768, 32767);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF, (state.getY(0x2000 + @as(u32, @intCast(n))) orelse 0xDEAD) & 0xFFFF);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(b[1024 + n])))) & 0xFFFFFF, state.readX(0x1900 + @as(u32, @intCast(n))) orelse 0xDEAD);
        }
    };
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

test "stereo_pair applies M/S and intensity as the model does" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/stereo.sm"
        \\        bits_init(0x4000)
        \\        stereo_pair()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var prng = std.Random.DefaultPrng.init(0x57E2);
    const rnd = prng.random();
    var seen_ms: usize = 0;
    var seen_is: usize = 0;
    for (0..40) |_| {
        var p: StPair = undefined;
        const short = rnd.boolean();
        p.win = if (short) 128 else 1024;
        p.glen = @splat(0);
        if (short) {
            var left: usize = 8;
            p.groups = 0;
            while (left > 0) {
                const l = rnd.intRangeAtMost(usize, 1, left);
                p.glen[p.groups] = l;
                p.groups += 1;
                left -= l;
            }
            p.max_sfb = rnd.intRangeAtMost(usize, 1, 15);
        } else {
            p.groups = 1;
            p.glen[0] = 1;
            p.max_sfb = rnd.intRangeAtMost(usize, 1, 49);
        }
        for (&p.off, 0..) |*o, k| o.* = @intCast(k * @as(usize, if (short) 8 else 20));
        p.msp = rnd.intRangeAtMost(i64, 0, 2);
        // books: mostly spectral, some zero, intensity on the right; sometimes a silent side
        const quiet_l = rnd.uintLessThan(u8, 8) == 0;
        const quiet_r = rnd.uintLessThan(u8, 6) == 0;
        for (0..128) |i| {
            const a = rnd.intRangeAtMost(i64, 0, 11);
            p.lb[i] = if (quiet_l) 0 else a;
            const c = rnd.uintLessThan(u8, 10);
            p.rb[i] = if (c < 2) 14 + @as(i64, rnd.intRangeAtMost(u1, 0, 1)) else if (quiet_r) 0 else rnd.intRangeAtMost(i64, 0, 11);
            p.rs[i] = rnd.intRangeAtMost(i64, -24, 60);
            p.msu[i] = if (p.msp == 2) 1 else rnd.intRangeAtMost(i64, 0, 1);
        }
        p.l = @splat(0);
        p.r = @splat(0);
        const Fill = struct {
            q: *StPair,
            rnd: std.Random,
            fn f(c: *@This(), start: usize, len: usize, band: usize) void {
                for (start..start + len) |k| {
                    if (stSpectral(c.q.lb[band])) c.q.l[k] = c.rnd.intRangeAtMost(i64, -5600000, 5600000);
                    if (stSpectral(c.q.rb[band])) c.q.r[k] = c.rnd.intRangeAtMost(i64, -5600000, 5600000);
                }
            }
        };
        var fill: Fill = .{ .q = &p, .rnd = rnd };
        p.eachRun(&fill, Fill.f);
        p.el = rnd.intRangeAtMost(i64, -20, 20);
        p.er = p.el + rnd.intRangeAtMost(i64, -30, 30);
        for (0..p.groups * p.max_sfb) |i| {
            if (p.kind(i) == 1) seen_ms += 1;
            if (p.kind(i) >= 2) seen_is += 1;
        }

        var state = emu.State{};
        defer state.deinit(gpa);
        const py = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.y_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
            }
        }.f;
        const px = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        const Y = 0x2600;
        try py(&state, gpa, Y + 0, @intCast(p.groups * p.max_sfb));
        try py(&state, gpa, Y + 1, @intCast(p.groups));
        try py(&state, gpa, Y + 2, @intCast(p.max_sfb));
        try py(&state, gpa, Y + 3, @intCast(p.win));
        try py(&state, gpa, Y + 4, p.msp);
        try py(&state, gpa, Y + 5, p.el);
        try py(&state, gpa, Y + 6, p.er);
        for (0..8) |g| try py(&state, gpa, Y + 0x10 + g, @intCast(p.glen[g]));
        for (p.off, 0..) |o, k| try py(&state, gpa, Y + 0x20 + k, o);
        for (0..128) |i| {
            try py(&state, gpa, Y + 0x60 + i, p.lb[i]);
            try py(&state, gpa, Y + 0xE0 + i, p.rb[i]);
            try py(&state, gpa, Y + 0x160 + i, p.rs[i]);
            try py(&state, gpa, Y + 0x1E0 + i, p.msu[i]);
        }
        for (0..4) |b| try px(&state, gpa, 0x1F00 + b, @intFromFloat(@round(std.math.pow(f64, 2.0, 22.0 - @as(f64, @floatFromInt(b)) / 4.0))));
        for (p.l, p.r, 0..) |a, b, k| {
            try px(&state, gpa, 0x400 + k, a);
            try px(&state, gpa, 0x800 + k, b);
        }
        p.apply();
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 24);
        try testing.expect(state.halted);
        const sx16 = struct {
            fn f(v: ?u32) i64 {
                return @as(i16, @bitCast(@as(u16, @truncate(v orelse 0xDEAD))));
            }
        }.f;
        try testing.expectEqual(p.el, sx16(state.getY(Y + 5)));
        try testing.expectEqual(p.er, sx16(state.getY(Y + 6)));
        for (0..1024) |k| {
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(p.l[k])))) & 0xFFFFFF, state.readX(0x400 + @as(u32, @intCast(k))) orelse 0xDEAD);
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(p.r[k])))) & 0xFFFFFF, state.readX(0x800 + @as(u32, @intCast(k))) orelse 0xDEAD);
        }
    }
    try testing.expect(seen_ms > 200);
    try testing.expect(seen_is > 100);
}

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
    // fact 516: own-x00rimm (R2 = 0x280) and the other sources of the same slot.
    const gpa = testing.allocator;
    for ([_]struct { h: u32, w1: u32, w4: u32, w5: u32, w6: u32, want: u32 }{
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0, .w5 = 0x06000000, .w6 = 0, .want = 0x280 },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0, .w5 = 0x04000000, .w6 = 0, .want = 0 },
        .{ .h = 0x00140000, .w1 = 0x2000, .w4 = 0, .w5 = 0x02000000, .w6 = 0x43000000, .want = (0x1234 * 0x0567 * 2) & 0xFFFF },
        .{ .h = 0x00140000, .w1 = 0x22000, .w4 = 0, .w5 = 0x02000000, .w6 = 0x43000000, .want = (0x1234 * 0x0567) & 0xFFFF },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0x000000A0, .w5 = 0x02000000, .w6 = 0x43000000, .want = 0x2468 },
        .{ .h = 0x00140000, .w1 = 0, .w4 = 0x000000A0, .w5 = 0, .w6 = 0, .want = 0x2468 },
        .{ .h = 0x08140000, .w1 = 0x500, .w4 = 0, .w5 = 0x00200000, .w6 = 0, .want = 0x380 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[5] = 0x710;
        state.r[1] = 0x100;
        state.s[3] = 0x1234;
        state.s[4] = 0x0567;
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
        .{ .s3 = 0x3303, .mux = 0, .code = 1, .want = 0x300 << 7 },
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
    // 518: own-b16h31 (head[31] = 0 runs the same ALU); S2[6] alone writes 0; other shapes refuse
    for ([_]struct { h: u32, s2: u32, want: ?u32 }{
        .{ .h = 0x001211d6, .s2 = 0x86000000, .want = 0x8080 },
        .{ .h = 0x801211d6, .s2 = 0x86000040, .want = 0 },
        .{ .h = 0x801211d6, .s2 = 0x86000080, .want = null },
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
    // w1[8] with w3[26:23] = 0: the image read 0x3422, not e0; refused
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

fn tnsRefl() [24]i64 {
    var t: [24]i64 = @splat(0);
    for (0..2) |res| {
        const bits: u32 = @as(u32, @intCast(res)) + 3;
        const half: i32 = @as(i32, 1) << @intCast(bits - 1);
        var c: i32 = -half;
        while (c < half) : (c += 1) {
            const p: f64 = std.math.pow(f64, 2, @floatFromInt(bits));
            const denom: f64 = if (c >= 0) p - 1 else p + 1;
            const at: i32 = if (res == 1) 16 + c else 4 + c;
            t[@intCast(at)] = @intFromFloat(@round(@sin(@as(f64, @floatFromInt(c)) * std.math.pi / denom) * 0x1p23));
        }
    }
    return t;
}

test "tns_channel filters every window as the model does" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/tns.sm"
        \\        bits_init(0x4000)
        \\        tns_channel()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    const refl = tnsRefl();
    var prng = std.Random.DefaultPrng.init(0x7E5C);
    const rnd = prng.random();
    var halvings: u32 = 0;
    var gave_up: u32 = 0;
    for (0..31) |trial| {
        const short = trial % 3 == 0 and trial != 30;
        var c: TnsCase = .{
            .nwin = if (short) 8 else 1,
            .wlen = if (short) 128 else 1024,
            .nswb = if (short) 14 else 49,
            .top = 0,
            .maxo = if (short) 7 else 12,
            .off = undefined,
            .rec = .empty,
            .w = undefined,
            .e = rnd.intRangeAtMost(i64, -30, 10),
        };
        defer c.rec.deinit(gpa);
        for (&c.off, 0..) |*o, k| o.* = @intCast(@min(k * @as(usize, if (short) 9 else 21), c.wlen));
        c.top = rnd.intRangeAtMost(u32, 1, c.nswb);
        const amp: i64 = rnd.intRangeAtMost(i64, 1 << 10, 5600000);
        for (&c.w) |*x| x.* = rnd.intRangeAtMost(i64, -amp, amp);
        if (trial == 30) {
            // twelve stages at the largest code on a constant run: past TN_MAXH, the run zeroed
            c.top = 49;
            c.w = @splat(1 << 21);
            try c.rec.appendSlice(gpa, &.{ 1, 1, 49, 12, 0 });
            for (0..12) |_| try c.rec.append(gpa, -8);
        } else for (0..c.nwin) |_| {
            const nf: usize = rnd.intRangeAtMost(usize, 0, if (short) 1 else 3);
            try c.rec.append(gpa, @intCast(nf));
            if (nf == 0) continue;
            const res = rnd.intRangeAtMost(i64, 0, 1);
            try c.rec.append(gpa, res);
            for (0..nf) |_| {
                const order = rnd.intRangeAtMost(i64, 0, if (short) 7 else 31);
                try c.rec.append(gpa, rnd.intRangeAtMost(i64, 0, if (short) 15 else 63));
                try c.rec.append(gpa, order);
                if (order == 0) continue;
                try c.rec.append(gpa, rnd.intRangeAtMost(i64, 0, 1));
                const lo: i64 = if (res == 1) -8 else -4;
                const wide = trial % 5 == 4;
                for (0..@intCast(order)) |_| try c.rec.append(gpa, if (wide) rnd.intRangeAtMost(i64, lo, -lo - 1) else rnd.intRangeAtMost(i64, -2, 2));
            }
        }

        var state = emu.State{};
        defer state.deinit(gpa);
        const py = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.y_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
            }
        }.f;
        const px = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        const Y = 0x2600;
        try py(&state, gpa, Y + 0, c.e);
        try py(&state, gpa, Y + 1, @intCast(c.nwin));
        try py(&state, gpa, Y + 2, @intCast(c.wlen));
        try py(&state, gpa, Y + 3, c.nswb);
        try py(&state, gpa, Y + 4, c.top);
        try py(&state, gpa, Y + 5, c.maxo);
        for (c.off, 0..) |o, k| try py(&state, gpa, Y + 0x20 + k, o);
        for (c.rec.items, 0..) |v, k| try py(&state, gpa, Y + 0x60 + k, v);
        for (refl, 0..) |v, k| try px(&state, gpa, 0x1F00 + k, v);
        for (c.w, 0..) |v, k| try px(&state, gpa, 0x400 + k, v);
        c.apply(&refl);
        halvings += c.halvings;
        gave_up += c.gave_up;
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 28);
        try testing.expect(state.halted);
        const sx16 = struct {
            fn f(v: ?u32) i64 {
                return @as(i16, @bitCast(@as(u16, @truncate(v orelse 0xDEAD))));
            }
        }.f;
        try testing.expectEqual(c.e, sx16(state.getY(Y + 0)));
        try testing.expectEqual(@as(i64, c.halvings), sx16(state.getY(Y + 6)));
        try testing.expectEqual(@as(i64, c.gave_up), sx16(state.getY(Y + 7)));
        for (0..1024) |k| {
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(c.w[k])))) & 0xFFFFFF, state.readX(0x400 + @as(u32, @intCast(k))) orelse 0xDEAD);
        }
        // codes past TNS_MAX_ORDER are skipped: the table and the words below K' untouched
        for (0..0x20) |k| try testing.expectEqual(if (k < 24) @as(?u32, @as(u32, @truncate(@as(u64, @bitCast(refl[k])))) & 0xFFFFFF) else null, state.readX(0x1F00 + @as(u32, @intCast(k))));
    }
    try testing.expect(halvings > 0);
    try testing.expectEqual(@as(u32, 1), gave_up);
}

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

test "pns_channel fills noise bands as the model does, both modes" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/pns.sm"
        \\        bits_init(0x4000)
        \\        pns_channel()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var prng = std.Random.DefaultPrng.init(0x9E5C);
    const rnd = prng.random();
    var noisy: usize = 0;
    var corr: usize = 0;
    for (0..40) |trial| {
        var c: PnCase = undefined;
        const short = trial % 3 == 0;
        c.mode = @intCast(trial % 2);
        c.win = if (short) 128 else 1024;
        c.glen = @splat(0);
        if (short) {
            var left: usize = 8;
            c.groups = 0;
            while (left > 0) {
                const l = rnd.intRangeAtMost(usize, 1, left);
                c.glen[c.groups] = l;
                c.groups += 1;
                left -= l;
            }
            c.max_sfb = rnd.intRangeAtMost(usize, 1, 14);
        } else {
            c.groups = 1;
            c.glen[0] = 1;
            c.max_sfb = rnd.intRangeAtMost(usize, 1, 49);
        }
        // widths of 1 to 31 so the band lengths cross every shift step
        c.off[0] = 0;
        for (1..64) |k| c.off[k] = @min(c.off[k - 1] + rnd.intRangeAtMost(i64, 1, if (short) 9 else 31), @as(i64, @intCast(c.win)));
        if (!short and trial % 7 == 2) {
            // one band of the whole window: the largest shift
            c.max_sfb = 1;
            c.off[1] = 1024;
        }
        c.msp = rnd.intRangeAtMost(i64, 0, 2);
        const quiet = rnd.uintLessThan(u8, 5) == 0;
        for (0..128) |i| {
            const r = rnd.uintLessThan(u8, 3);
            c.book[i] = if (r == 0) 13 else if (r == 1 or quiet) 0 else rnd.intRangeAtMost(i64, 1, 11);
            c.sf[i] = rnd.intRangeAtMost(i64, -60, 120);
            c.lbook[i] = if (rnd.boolean()) 13 else rnd.intRangeAtMost(i64, 0, 11);
            c.lsf[i] = rnd.intRangeAtMost(i64, -60, 120);
            c.msu[i] = if (c.msp == 2) 1 else rnd.intRangeAtMost(i64, 0, 1);
            if (c.book[i] == 13 and i < c.groups * c.max_sfb) {
                noisy += 1;
                if (c.mode == 1 and c.msp != 0 and c.msu[i] != 0 and c.lbook[i] == 13) corr += 1;
            }
        }
        c.w = @splat(0);
        for (&c.l) |*x| x.* = rnd.intRangeAtMost(i64, -5600000, 5600000);
        var wb: usize = 0;
        for (0..c.groups) |g| {
            for (0..c.max_sfb) |sfb| {
                const b = c.book[g * c.max_sfb + sfb];
                if (b < 1 or b > 11) continue;
                for (0..c.glen[g]) |w| {
                    for (@as(usize, @intCast(c.off[sfb]))..@as(usize, @intCast(c.off[sfb + 1]))) |k| c.w[wb + w * c.win + k] = rnd.intRangeAtMost(i64, -5600000, 5600000);
                }
            }
            wb += c.glen[g] * c.win;
        }
        c.e = rnd.intRangeAtMost(i64, -40, 30);
        c.el = rnd.intRangeAtMost(i64, -40, 30);
        c.x = pnWrap(@intCast(rnd.int(u24)));

        var state = emu.State{};
        defer state.deinit(gpa);
        const py = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.y_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
            }
        }.f;
        const px = struct {
            fn f(st: *emu.State, a: std.mem.Allocator, addr: usize, v: i64) !void {
                try st.x_mem.put(a, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
            }
        }.f;
        const Y = 0x2600;
        try py(&state, gpa, Y + 0, @intCast(c.groups * c.max_sfb));
        try py(&state, gpa, Y + 1, @intCast(c.groups));
        try py(&state, gpa, Y + 2, @intCast(c.max_sfb));
        try py(&state, gpa, Y + 3, @intCast(c.win));
        try py(&state, gpa, Y + 4, c.mode);
        try py(&state, gpa, Y + 5, c.msp);
        try py(&state, gpa, Y + 6, c.e);
        try py(&state, gpa, Y + 7, c.el);
        for (0..8) |g| try py(&state, gpa, Y + 0x10 + g, @intCast(c.glen[g]));
        for (c.off, 0..) |o, k| try py(&state, gpa, Y + 0x20 + k, o);
        for (0..128) |i| {
            try py(&state, gpa, Y + 0x60 + i, c.book[i]);
            try py(&state, gpa, Y + 0xE0 + i, c.sf[i]);
            try py(&state, gpa, Y + 0x160 + i, c.lbook[i]);
            try py(&state, gpa, Y + 0x1E0 + i, c.lsf[i]);
            try py(&state, gpa, Y + 0x260 + i, c.msu[i]);
        }
        for (0..4) |b| try px(&state, gpa, 0x1F00 + b, pnT(b));
        for (0..6) |i| try px(&state, gpa, 0x1F04 + i, pnSeed(i));
        try px(&state, gpa, 0x1F10, c.x);
        for (c.w, c.l, 0..) |a, b, k| {
            try px(&state, gpa, 0x400 + k, a);
            try px(&state, gpa, 0x800 + k, b);
        }
        c.apply();
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 28);
        try testing.expect(state.halted);
        const sx16 = struct {
            fn f(v: ?u32) i64 {
                return @as(i16, @bitCast(@as(u16, @truncate(v orelse 0xDEAD))));
            }
        }.f;
        try testing.expectEqual(c.e, sx16(state.getY(Y + 6)));
        try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(c.x)))) & 0xFFFFFF, state.readX(0x1F10) orelse 0xDEAD);
        for (0..1024) |k| {
            try testing.expectEqual(@as(u32, @truncate(@as(u64, @bitCast(c.w[k])))) & 0xFFFFFF, state.readX(0x400 + @as(u32, @intCast(k))) orelse 0xDEAD);
        }
    }
    try testing.expect(noisy > 200);
    try testing.expect(corr > 15);
}

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
        .{ .w = 0x40918009 | 1 << 13 | 5 << 26 | 6 << 5, .s2 = null, .s5 = 0 }, // Y bit 15 set
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.s[2] = 0x0ABC;
        state.s[0] = 0x55;
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
            try testing.expectEqual(@as(u32, 0x55), state.s[0]); // destination 0 writes nothing
        } else try testing.expectError(error.UnknownForm, r);
    }
}
