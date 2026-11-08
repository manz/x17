//! The shared sources in examples/lib run in the emulator against plain Zig models.
const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");

const testing = std.testing;

const build_options = @import("build_options");

/// AAC stage costs for `zig build aac-bench`: the worst bundles per unit seen by each stage's
/// model test.  Units: one code or codeword for the Huffman stages, one call for the IMDCT, one
/// 1024-coefficient channel (or channel pair, stereo) for the rest.
const StageCost = struct { name: []const u8, worst: u64 = 0 };
var stage_costs = [_]StageCost{
    .{ .name = "sections" },    .{ .name = "huffman" },  .{ .name = "spectral" },  .{ .name = "dequant" },
    .{ .name = "stereo" },      .{ .name = "pns" },      .{ .name = "tns" },       .{ .name = "imdct long" },
    .{ .name = "imdct short" }, .{ .name = "win_long" }, .{ .name = "win_other" },
};

fn stageBench(name: []const u8, bundles: u64, units: u64) void {
    for (&stage_costs) |*c| if (std.mem.eql(u8, c.name, name)) {
        c.worst = @max(c.worst, bundles / units);
        return;
    };
    unreachable;
}

fn stageCost(name: []const u8) u64 {
    for (stage_costs) |c| if (std.mem.eql(u8, c.name, name)) return c.worst;
    unreachable;
}

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
    .{ .path = "lib/aac.sm", .text = @embedFile("aac.sm") },
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
    try stageAt(gpa, state, data, 0x4000);
}

fn stageAt(gpa: std.mem.Allocator, state: *emu.State, data: []const u8, base: u32) !void {
    for (0..data.len / 2) |i| try state.y_mem.put(gpa, base + @as(u32, @intCast(i)), @as(u32, data[2 * i]) << 8 | data[2 * i + 1]);
}

/// Every word the run touched lies in the mode-0 windows: X 0..0x1FFF (silicon stores only to 0x1BFF, fact 649; ALAC's stereo lanes still need the room), Y 0..0x2EFF and
/// 0xFD80..0xFDFF (writes and the host's staging; the emulator's memory is sparse).
fn expectMode0(state: *emu.State) !void {
    var xi = state.x_mem.keyIterator();
    while (xi.next()) |k| if (k.* >= 0x2000) {
        std.debug.print("X:{x} outside the mode-0 window\n", .{k.*});
        return error.OutsideWindow;
    };
    var yi = state.y_mem.keyIterator();
    while (yi.next()) |k| if (k.* >= 0x2F00 and !(k.* >= 0xFD80 and k.* <= 0xFDFF)) {
        std.debug.print("Y:{x} outside the mode-0 windows\n", .{k.*});
        return error.OutsideWindow;
    };
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

/// sosumi ref/alac/rice.zig `decompress`, 32-bit as the ALAC reference algorithm computes it.
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

/// sosumi ref/alac/predictor.zig `unpredict`, 32-bit as the ALAC reference algorithm computes it.
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
        try stageAt(gpa, &state, padded, 0x80);
        for (cfg, 0..) |v, i| try state.y_mem.put(gpa, 0x2E00 + @as(u32, @intCast(i)), v);
        _ = try emu.runFrom(gpa, &state, prog, 0, 1 << 30);
        try testing.expect(state.halted);
        try expectMode0(&state);
        const status = state.y_mem.get(0x2E07).?;
        if (cfg[1] > 16) {
            // 16-bit output: deeper streams are refused, and nothing is written
            try testing.expectEqual(@as(u32, 4), status);
            try testing.expectEqual(@as(?u32, null), state.y_mem.get(0xFD94));
            continue;
        }
        if (status != 0) std.debug.print("{s} packet {d}: status {d}\n", .{ name, k, status });
        try testing.expectEqual(@as(u32, 0), status);
        try testing.expectEqual(n, state.y_mem.get(0x2E06).?);
        try testing.expectEqual(@as(?u32, 1), state.y_mem.get(0xFD94));
        try testing.expectEqual(@as(?u32, @intCast(n * nch)), state.y_mem.get(6));
        for (0..n * nch) |i| {
            const want = le32(pcm, 4 * i) & 0xFFFF;
            const got = state.y_mem.get(0x80 + @as(u32, @intCast(i))) orelse 0xDEAD0000;
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
        \\        bits_init(0x80)
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
        \\        bits_init(0x80)
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
        stageBench("huffman", try emu.runFrom(gpa, &state, prog, 0, 1 << 20), 12);
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
            stageBench("spectral", try emu.runFrom(gpa, &state, prog, 0, 1 << 20), 8);
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
        stageBench("sections", try emu.runFrom(gpa, &state, prog, 0, 1 << 22), 1);
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
        stageBench("dequant", try emu.runFrom(gpa, &state, prog, 0, 1 << 24), 1);
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
        stageBench(if (n2 == 1024) "imdct long" else "imdct short", try emu.runFrom(gpa, &state, prog, 0, 1 << 26), 1);
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
    // Side writes of two templates (subforms.txt); x18.rag101 has none.
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
    // fact 507: own-x01m4w22 (w4[22] = 1, s8 = s11 = 0x4000 gives s8 = 0x20);
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
        stageBench("win_long", try emu.runFrom(gpa, &state, prog, 0, 1 << 22), 1);
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
        stageBench("win_other", try emu.runFrom(gpa, &state, prog, 0, 1 << 23), 1);
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
        stageBench("stereo", try emu.runFrom(gpa, &state, prog, 0, 1 << 24), 1);
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
        stageBench("tns", try emu.runFrom(gpa, &state, prog, 0, 1 << 28), 1);
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
        stageBench("pns", try emu.runFrom(gpa, &state, prog, 0, 1 << 28), 1);
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
        .{ .h = 0xa8a40019, .w1 = 0x00004503, .w2 = 0x1ac02208, .dest = 3, .want = 0x2300 },
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
    const gpa = testing.allocator;
    for ([_]struct { s1: u32, r11: ?u32 }{
        .{ .s1 = 0xa802d804, .r11 = 0x720 },
        .{ .s1 = 0xa802d904, .r11 = 0x721 },
        .{ .s1 = 0xa802dd84, .r11 = null },
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
        .{ .w4 = 0x00242818, .w5 = 0x30024628, .s3 = 0x999 },
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
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
        .{ .w = .{ 0x0018b000, 0x00000000, 0x000c0000, 0x00d00000, 0x0020c0c0, 0x06000032, 0x00060000 }, .dest = 6, .want = null }, // plain mode 0
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

test "x00's jump controls take sx15({w2[0], w1[31:18]}) (fact 575, 578)" {
    const gpa = testing.allocator;
    for ([_]struct { ctl: u32, w1: u32, w2: u32 = 0, w4: u32 = 0, off: ?i32 }{
        .{ .ctl = 3, .w1 = 0xFFD80000, .w2 = 1, .off = -10 }, // the measured q - 10
        .{ .ctl = 3, .w1 = 0xFFD80000, .off = 16374 }, // without the sign: far forward
        .{ .ctl = 1, .w1 = 0x0C << 18, .off = 12 },
        .{ .ctl = 0xE, .w1 = (1 << 26) | (4 << 18), .off = 260 }, // +256 per w1[26]
        .{ .ctl = 0x17, .w1 = 0x3FFF << 18, .w2 = 1, .off = -1 },
        .{ .ctl = 3, .w1 = 0x0C << 18, .w4 = 1 << 28, .off = null }, // hidden state: OPEN
        .{ .ctl = 3, .w1 = 0x0C << 18, .w4 = 0xA << 28, .off = null },
        .{ .ctl = 3, .w1 = 0x0C << 18, .w4 = 2 << 28, .off = null }, // state-dependent
    }) |c| {
        var state = emu.State{};
        defer state.deinit(gpa);
        const r = emu.executeBundle(gpa, &state, &.{ 0, c.w1, c.w2, c.ctl, c.w4, 0, 0 });
        if (c.off) |o| {
            try r;
            try testing.expectEqual(@as(?i32, o), state.pending_jmp_offset);
        } else try testing.expectError(error.UnknownForm, r);
    }
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
        .{ .w5 = 0xC3800900, .want = 0xA050 }, // yd
    }) |k| {
        var state = emu.State{};
        defer state.deinit(gpa);
        state.e[1] = 0x720;
        state.e[9] = 0x760;
        state.yd = 0xA050;
        try state.y_mem.put(gpa, 0x720, 0x1234);
        try state.y_mem.put(gpa, 0x760, 0x7777);
        _ = try emu.runFrom(gpa, &state, &.{ 0x28001000, 0x0002A005 | (3 << 7), 0xE8010000, 0x00000020, k.w4, k.w5, 0x35001002, 0x00000017, 0x31308000, 0xA0000000 }, 0, 16);
        try testing.expectEqual(k.want, state.s[1]);
        // The fresh read sets yd; the yd copy leaves it.
        try testing.expectEqual(k.want, state.yd);
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
        .{ .a = 0x7000, .b = 0xFEDD, .round = 0xFFFF, .floor = 0xFFFF },
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

test "x03 product low half is (X op P)[15:0] (fact 603)" {
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
        .{ .w1 = 0x00007000, .want = 0xF79D },
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
            try testing.expectEqual(if (rnd == 1) c.round else c.floor, state.s[10]);
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

/// sosumi ref/aac/huffman.zig `books`: values per codeword, the radix, the field width x17's
/// spectral macros unpack (2 bits for quads, 5 for pairs).
const AacBook = struct { dim: u3, mod: u5, fbits: u4 };
const aac_books = [11]AacBook{
    .{ .dim = 4, .mod = 3, .fbits = 2 },  .{ .dim = 4, .mod = 3, .fbits = 2 },  .{ .dim = 4, .mod = 3, .fbits = 2 },
    .{ .dim = 4, .mod = 3, .fbits = 2 },  .{ .dim = 2, .mod = 9, .fbits = 5 },  .{ .dim = 2, .mod = 9, .fbits = 5 },
    .{ .dim = 2, .mod = 8, .fbits = 5 },  .{ .dim = 2, .mod = 8, .fbits = 5 },  .{ .dim = 2, .mod = 13, .fbits = 5 },
    .{ .dim = 2, .mod = 13, .fbits = 5 }, .{ .dim = 2, .mod = 17, .fbits = 5 },
};

/// The leaf a codeword index decodes to: its base-mod digits as fields, the first value highest;
/// the scale factor code's leaf is the index.
fn aacLeaf(book: usize, idx: usize) u16 {
    if (book == 0) return @intCast(idx);
    const b = aac_books[book - 1];
    var v = idx;
    var out: u16 = 0;
    var shift: u4 = 0;
    for (0..b.dim) |_| {
        out |= @as(u16, @intCast(v % b.mod)) << shift;
        v /= b.mod;
        shift += b.fbits;
    }
    return out;
}

fn aacQ23(x: f64) i64 {
    return @intFromFloat(std.math.clamp(@round(x * 0x1p23), -0x1p23, 0x1p23 - 1));
}

/// sosumi ref/aac/window.zig: the sine and KBD rising halves in Q23.
fn aacSine(out: []i64) void {
    const n: f64 = @floatFromInt(2 * out.len);
    for (out, 0..) |*e, i| e.* = aacQ23(@sin(std.math.pi / n * (@as(f64, @floatFromInt(i)) + 0.5)));
}

fn aacBesselI0(x: f64) f64 {
    var sum: f64 = 1;
    var term: f64 = 1;
    var k: f64 = 1;
    while (term > 1e-17 * sum) : (k += 1) {
        const r = x / (2 * k);
        term *= r * r;
        sum += term;
    }
    return sum;
}

fn aacKbd(out: []i64, alpha: f64) void {
    var kernel: [1025]f64 = undefined;
    const half = out.len;
    const q: f64 = @floatFromInt(half / 2);
    for (kernel[0 .. half + 1], 0..) |*k, p| {
        const r = (@as(f64, @floatFromInt(p)) - q) / q;
        k.* = aacBesselI0(std.math.pi * alpha * @sqrt(1 - r * r));
    }
    var total: f64 = 0;
    for (kernel[0 .. half + 1]) |k| total += k;
    var acc: f64 = 0;
    for (out, 0..) |*e, i| {
        acc += kernel[i];
        e.* = aacQ23(@sqrt(acc / total));
    }
}

fn aacPutX(st: *emu.State, gpa: std.mem.Allocator, addr: usize, v: i64) !void {
    try st.x_mem.put(gpa, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFFFF);
}

fn aacPutY(st: *emu.State, gpa: std.mem.Allocator, addr: usize, v: i64) !void {
    try st.y_mem.put(gpa, @intCast(addr), @as(u32, @truncate(@as(u64, @bitCast(v)))) & 0xFFFF);
}

/// Every table aac.sm reads, at its addresses (44.1 kHz).
fn aacTables(gpa: std.mem.Allocator, st: *emu.State) !void {
    // Huffman codes from Y 0x4800 on, the scale factor code first; AK_BOOKS + b points at book b
    var codes: [12][300]HCode = undefined;
    var counts: [12]usize = @splat(0);
    var lines = std.mem.tokenizeScalar(u8, @embedFile("books.txt"), '\n');
    while (lines.next()) |line| {
        if (line[0] == '#') continue;
        var f = std.mem.tokenizeScalar(u8, line, ' ');
        const b = try std.fmt.parseInt(usize, f.next().?, 10);
        const code = try std.fmt.parseInt(u32, f.next().?, 16);
        const len = try std.fmt.parseInt(u5, f.next().?, 10);
        codes[b][counts[b]] = .{ .code = code, .len = len, .v = aacLeaf(b, counts[b]) };
        counts[b] += 1;
    }
    var addr: usize = 0x4800;
    for (0..12) |b| {
        var t: [8192]u16 = undefined;
        var n: usize = 0;
        _ = buildTables(&t, &n, codes[b][0..counts[b]], 0, 8);
        try testing.expect(n <= 1024);
        for (t[0..n], 0..) |w, i| try aacPutY(st, gpa, addr + i, w);
        if (b > 0) try aacPutY(st, gpa, 0x3C40 + b, @intCast(addr));
        addr += n;
    }
    try testing.expect(addr <= 0x5C00);
    // dequantiser
    const dq = dqTables();
    for (0..129) |k| {
        try aacPutX(st, gpa, 0x1800 + 2 * k, dq.dm[k]);
        try aacPutX(st, gpa, 0x1801 + 2 * k, dq.de[k]);
    }
    for (dq.it, 0..) |v, j| try aacPutX(st, gpa, 0x1910 + j, v);
    for (dq.g, 0..) |v, j| try aacPutX(st, gpa, 0x1960 + j, v);
    for (0..8) |j| {
        try aacPutY(st, gpa, 0x2140 + j, dq_ie[j]);
        try aacPutY(st, gpa, 0x2148 + j, dq_ir[j] * 4);
    }
    // IMDCT, long and short, Z at X 0
    const Tl = ImTables(1024);
    const tl = try gpa.create(Tl);
    defer gpa.destroy(tl);
    tl.* = Tl.init();
    for (tl.pre, tl.post, 0..) |a, c, k| {
        try aacPutX(st, gpa, 0x1000 + 2 * k, a.re);
        try aacPutX(st, gpa, 0x1001 + 2 * k, a.im);
        try aacPutX(st, gpa, 0x1400 + 2 * k, c.re);
        try aacPutX(st, gpa, 0x1401 + 2 * k, c.im);
    }
    for (tl.tw, 0..) |c, k| {
        try aacPutX(st, gpa, 0x3000 + 2 * k, c.re);
        try aacPutX(st, gpa, 0x3001 + 2 * k, c.im);
    }
    for (tl.rev, 0..) |r, n| try aacPutY(st, gpa, 0x2400 + n, 2 * @as(i64, r) - 2);
    const ts = ImTables(128).init();
    for (ts.pre, ts.post, 0..) |a, c, k| {
        try aacPutX(st, gpa, 0x1A00 + 2 * k, a.re);
        try aacPutX(st, gpa, 0x1A01 + 2 * k, a.im);
        try aacPutX(st, gpa, 0x1A80 + 2 * k, c.re);
        try aacPutX(st, gpa, 0x1A81 + 2 * k, c.im);
    }
    for (ts.tw, 0..) |c, k| {
        try aacPutX(st, gpa, 0x1B00 + 2 * k, c.re);
        try aacPutX(st, gpa, 0x1B01 + 2 * k, c.im);
    }
    for (ts.rev, 0..) |r, n| try aacPutY(st, gpa, 0x2D00 + n, 2 * @as(i64, r) - 2);
    // window halves as rshr(w, 2): sine and KBD (alpha 4 long, 6 short)
    var wl: [2][1024]i64 = undefined;
    var ws: [2][128]i64 = undefined;
    aacSine(&wl[0]);
    aacKbd(&wl[1], 4);
    aacSine(&ws[0]);
    aacKbd(&ws[1], 6);
    for (0..2) |s| {
        for (wl[s], 0..) |w, k| try aacPutX(st, gpa, 0x4800 + 0x400 * s + k, rshr(w, 2));
        for (ws[s], 0..) |w, k| try aacPutX(st, gpa, 0x5000 + 0x80 * s + k, rshr(w, 2));
    }
    // stereo T, PNS T and seeds, TNS reflection coefficients
    for (0..4) |b| try aacPutX(st, gpa, 0x5100 + b, @intFromFloat(@round(std.math.pow(f64, 2.0, 22.0 - @as(f64, @floatFromInt(b)) / 4.0))));
    for (0..4) |b| try aacPutX(st, gpa, 0x5104 + b, pnT(b));
    for (0..6) |i| try aacPutX(st, gpa, 0x5108 + i, pnSeed(i));
    for (tnsRefl(), 0..) |v, k| try aacPutX(st, gpa, 0x5110 + k, v);
    // swb_offset, band counts, TNS_MAX_BANDS
    for (0..64) |k| {
        try aacPutY(st, gpa, 0x3D00 + k, swb_long_44[@min(k, swb_long_44.len - 1)]);
        try aacPutY(st, gpa, 0x3D40 + k, swb_short_44[@min(k, swb_short_44.len - 1)]);
    }
    try aacPutY(st, gpa, 0x3C0A, swb_long_44.len - 1);
    try aacPutY(st, gpa, 0x3C0B, swb_short_44.len - 1);
    try aacPutY(st, gpa, 0x3C0C, 42);
    try aacPutY(st, gpa, 0x3C0D, 14);
}

const AacRun = struct { ok: usize = 0, n: usize = 0, long_bundles: u64 = 0, short_bundles: u64 = 0 };

/// Access units dumped from sosumi's fixed-point decoder (`aacdec`): the state snapshot first,
/// then each unit in turn in one emulator state, its PCM against the reference's.
fn checkAac(gpa: std.mem.Allocator, prog: []const u32, dump: []const u8, name: []const u8, limit: usize) !AacRun {
    try testing.expectEqualSlices(u8, "AAPK", dump[0..4]);
    try testing.expectEqual(@as(u32, 4), le32(dump, 4));
    const nch = le32(dump, 8);
    const count = le32(dump, 16);
    var state = emu.State{};
    defer state.deinit(gpa);
    try aacTables(gpa, &state);
    try aacPutY(&state, gpa, 0x3C00, nch);
    var p: usize = 20;
    for (0..nch) |c| {
        try aacPutY(&state, gpa, 0x3C0E + c, le32(dump, p));
        p += 4;
        for (0..1024) |k| try state.x_mem.put(gpa, @intCast(0x3C00 + 0x400 * c + k), le32(dump, p + 4 * k) & 0xFFFFFF);
        p += 4096;
    }
    try state.x_mem.put(gpa, 0x5130, le32(dump, p) & 0xFFFFFF);
    p += 4;
    var run: AacRun = .{ .n = count };
    for (0..@min(count, limit)) |k| {
        const len = le32(dump, p);
        const au = dump[p + 4 .. p + 4 + len];
        p += 4 + len;
        const pcm = dump[p .. p + 4 * 1024 * nch];
        p += 4 * 1024 * nch;
        for (0..(len + 1) / 2) |i| {
            const hi: u32 = au[2 * i];
            const lo: u32 = if (2 * i + 1 < len) au[2 * i + 1] else 0;
            try state.y_mem.put(gpa, 0x4000 + @as(u32, @intCast(i)), hi << 8 | lo);
        }
        state.cur_lane = 0;
        const bundles = try emu.runFrom(gpa, &state, prog, 0, 1 << 30);
        try testing.expect(state.halted);
        const status = state.getY(0x3C01).?;
        if (status != 0) {
            std.debug.print("{s} unit {d}: status {d}\n", .{ name, k, status });
            return run;
        }
        // eight_short somewhere in the unit: the slot's FIN record keeps the last sequence
        if (state.getY(0x6401) == 2) run.short_bundles = @max(run.short_bundles, bundles) else run.long_bundles = @max(run.long_bundles, bundles);
        var bad: ?usize = null;
        for (0..1024 * nch) |i| {
            const want = le32(pcm, 4 * i) & 0xFFFF;
            const got = state.getY(0x3400 + @as(u32, @intCast(i))) orelse 0xDEAD0000;
            if (got != want and bad == null) {
                bad = i;
                std.debug.print("{s} unit {d}: sample {d} (frame {d} ch {d}) want {x} got {x}\n", .{ name, k, i, i / nch, i % nch, want, got });
            }
        }
        if (bad == null) run.ok += 1;
    }
    return run;
}

test "aac_packet decodes fixture access units as the reference does" {
    const gpa = testing.allocator;
    const prog = try assemble(gpa,
        \\.include "lib/bits.sm"
        \\.include "lib/mem.sm"
        \\.include "lib/huffman.sm"
        \\.include "lib/sections.sm"
        \\.include "lib/dequant.sm"
        \\.include "lib/stereo.sm"
        \\.include "lib/pns.sm"
        \\.include "lib/tns.sm"
        \\.include "lib/imdct.sm"
        \\.include "lib/window.sm"
        \\.include "lib/aac.sm"
        \\        bits_init(0x4000)
        \\        aac_packet()
        \\        x17.halt
        \\
    );
    defer gpa.free(prog);
    var all = true;
    inline for (.{ "tone.aac_at", "sweep.ff_tns", "pink.ff_pns", "pink.ff_is", "clicks.aac_at" }) |name| {
        const r = try checkAac(gpa, prog, @embedFile(name ++ ".pkts"), name, 1000);
        if (build_options.bench or r.ok != r.n) std.debug.print("{s}: {d}/{d} units bit for bit; worst bundles per unit: long {d}, short {d}\n", .{ name, r.ok, r.n, r.long_bundles, r.short_bundles });
        all = all and r.ok == r.n;
    }
    try testing.expect(all);
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
        .{ .path = "src/bench.zig", .text = @embedFile("bench.zig") },
    } ++ libs_scan;
    for (files) |f| for (banned) |b| if (std.mem.indexOf(u8, f.text, b)) |at| {
        std.debug.print("{s}: offset {d} carries '{s}'\n", .{ f.path, at, b });
        return error.ProvenanceText;
    };
}

test "aac stage bench report" {
    if (!build_options.bench) return;
    // Runs last (declaration order) under `zig build aac-bench`, which filters to the AAC stage
    // tests.  A long-block channel frame: sections + spectral codewords + dequant + tns + pns +
    // imdct long + window, plus half a stereo pair.  Spectral is costed at 512 codewords (pair
    // books, the worst case; quad books halve it).  Load at 26.7 M bundles/s (fact 379).
    const p = std.debug.print;
    p("\n{s:<12} {s:>10}  unit\n", .{ "stage", "bundles" });
    for (stage_costs) |c| p("{s:<12} {d:>10}  {s}\n", .{ c.name, c.worst, if (std.mem.eql(u8, c.name, "huffman")) "code" else if (std.mem.eql(u8, c.name, "spectral")) "codeword" else if (std.mem.startsWith(u8, c.name, "imdct")) "call" else if (std.mem.eql(u8, c.name, "stereo")) "pair" else "channel" });
    const frame: u64 = stageCost("sections") + 512 * stageCost("spectral") + stageCost("dequant") + stageCost("tns") + stageCost("pns") + stageCost("imdct long") + stageCost("win_long") + stageCost("stereo") / 2;
    // TNS and PNS are optional tools, costed at their tests' worst (TNS order 12 over the whole
    // block): the plain frame leaves them out.
    const plain = frame - stageCost("tns") - stageCost("pns");
    for ([_]struct { []const u8, u64 }{ .{ "worst", frame }, .{ "plain", plain } }) |f| {
        p("{s} long-block channel frame: {d} bundles\n", .{ f[0], f[1] });
        for ([_]u32{ 32000, 44100, 48000 }) |hz| for ([_]u32{ 1, 2 }) |ch| {
            const ld = @as(f64, @floatFromInt(f[1] * ch)) * @as(f64, @floatFromInt(hz)) / 1024.0 * 37.4e-9;
            p("  {d} Hz x{d}: {d:.0}%\n", .{ hz, ch, 100 * ld });
        };
    }
}

fn x18c3Run(head: u32, w1: u32, w2: u32, s5: u32) !emu.State {
    var state = emu.State{};
    errdefer state.deinit(testing.allocator);
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

test "x02's undoubled low product holds P mod 2^24 (655); x18.and masks it (644)" {
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
    }
}

test "ldy refuses e2 and reads e3 (650)" {
    for ([_]struct { e: u32, ok: bool }{ .{ .e = 2, .ok = false }, .{ .e = 3, .ok = true } }) |c| {
        var st = emu.State{};
        defer st.deinit(testing.allocator);
        st.e[c.e] = 0x2DB0;
        try st.y_mem.put(testing.allocator, 0x2DB0, 0x1234);
        var buf: [32]u8 = undefined;
        const w = try asm_mod.assembleOne(try std.fmt.bufPrint(&buf, "ldy s9, e{d}", .{c.e}));
        const r = emu.runFrom(testing.allocator, &st, &.{ w.slice()[0], 0x00000017, 0x31308000, 0xA0000000 }, 0, 4);
        if (c.ok) {
            _ = try r;
            try testing.expectEqual(@as(u32, 0x1234), st.s[9]);
        } else try testing.expectError(emu.RunError.UnknownForm, r);
    }
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
