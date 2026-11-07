//! Bundles per ALAC packet on the x17 emulator, split by stage: `zig build bench`.
//! Each stage call is bracketed by marker bundles (alu s0, s0, s0: writes 0 to s0, a no-op)
//! and the emulator's per-syllable profile is summed between them. Load is against the
//! measured rate: one bundle of any form takes about 37.4 ns on silicon (26.7 M/s; Upstream,
//! fact 379), so 100% is real time at the fixture's sample rate.
const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");

const lib_paths = [_][]const u8{ "lib/bits.sm", "lib/rice.sm", "lib/predictor.sm", "lib/unmix.sm", "lib/mem.sm", "lib/alac.sm" };
const lib_texts = [_][]const u8{ @embedFile("bits.sm"), @embedFile("rice.sm"), @embedFile("predictor.sm"), @embedFile("unmix.sm"), @embedFile("mem.sm"), @embedFile("alac.sm") };

/// The stage calls in alac.sm, in program order.
const stages = [_][]const u8{ "rice_decompress(RICE)", "integrate()", "unpredict()", "unmix()" };
const begin_mark = "alu s0, s0, s0, #OP_ADD";
const end_mark = "alu s0, s0, s0, #OP_SUB";

fn load(ctx: ?*anyopaque, gpa: std.mem.Allocator, path: []const u8) anyerror!?[]u8 {
    _ = ctx;
    for (lib_paths, lib_texts) |p, t| {
        if (!std.mem.eql(u8, p, path)) continue;
        if (!std.mem.eql(u8, p, "lib/alac.sm")) return try gpa.dupe(u8, t);
        var text: []u8 = try gpa.dupe(u8, t);
        for (stages) |s| {
            const call = try std.fmt.allocPrint(gpa, "        {s}\n", .{s});
            const marked = try std.fmt.allocPrint(gpa, "        {s}\n        {s}\n        {s}\n", .{ begin_mark, s, end_mark });
            if (std.mem.count(u8, text, call) != 1) return error.StageCall;
            text = try std.mem.replaceOwned(u8, gpa, text, call, marked);
        }
        return text;
    }
    return null;
}

fn assemble(gpa: std.mem.Allocator, src: []const u8) ![]u32 {
    return switch (try asm_mod.assembleFileWith(gpa, src, .{ .path = "bench.sm", .loader = .{ .load = load } })) {
        .ok => |p| p,
        .err => return error.Assemble,
    };
}

/// Syllable indices of every bundle equal to `mark`.
fn find(gpa: std.mem.Allocator, prog: []const u32, mark: []const u32) ![]usize {
    var at: std.ArrayList(usize) = .empty;
    var i: usize = 0;
    while (i + mark.len <= prog.len) : (i += 1) {
        if (std.mem.eql(u32, prog[i .. i + mark.len], mark)) try at.append(gpa, i);
    }
    return at.items;
}

fn le32(b: []const u8, at: usize) u32 {
    return std.mem.readInt(u32, b[at..][0..4], .little);
}

const fixtures = .{
    .{ "mono.at", 44100 },      .{ "tone.at", 44100 },       .{ "white24.at", 48000 },
    .{ "clicks.c0.ff", 44100 }, .{ "clicks.o30.ff", 44100 }, .{ "mono.at.int", 44100 },
    .{ "tone.at.int", 44100 },
};
const bundles_per_s = 1.0e9 / 37.4;

pub fn main(init: std.process.Init) !void {
    var arena = std.heap.ArenaAllocator.init(init.gpa);
    defer arena.deinit();
    const gpa = arena.allocator();
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
    const begins = try find(gpa, prog, try assemble(gpa, ".const OP_ADD = 14\n" ++ begin_mark ++ "\n"));
    const ends = try find(gpa, prog, try assemble(gpa, ".const OP_SUB = 0x10\n" ++ end_mark ++ "\n"));
    if (begins.len != stages.len or ends.len != stages.len) return error.Markers;
    std.debug.print("program: {d} syllables\n", .{prog.len});
    std.debug.print("{s:<14} {s:>3} {s:>5} {s:>3} {s:>10} {s:>8} {s:>7} {s:>7} {s:>7} {s:>7} {s:>7} {s:>6}\n", .{ "fixture", "pkt", "n", "ch", "bundles", "/sample", "rice", "integr", "predict", "unmix", "other", "load" });
    const profile = try gpa.alloc(u64, prog.len);
    inline for (fixtures) |fx| {
        const name = fx[0];
        const dump = @embedFile(name ++ ".pkts");
        const cfg = [6]u32{ le32(dump, 4), le32(dump, 8), le32(dump, 12), le32(dump, 16), le32(dump, 20), le32(dump, 24) };
        var p: usize = 32;
        for (0..le32(dump, 28)) |k| {
            const len = le32(dump, p);
            const pkt = dump[p + 4 .. p + 4 + len];
            p += 4 + len;
            const n = le32(dump, p);
            p += 4 + 4 * n * cfg[5];

            var state = emu.State{};
            @memset(profile, 0);
            state.profile = profile;
            for (0..(pkt.len + 1) / 2) |i| {
                const lo: u32 = if (2 * i + 1 < pkt.len) pkt[2 * i + 1] else 0;
                try state.y_mem.put(gpa, 0x4000 + @as(u32, @intCast(i)), @as(u32, pkt[2 * i]) << 8 | lo);
            }
            for (cfg, 0..) |v, i| try state.y_mem.put(gpa, 0x3300 + @as(u32, @intCast(i)), v);
            const total = try emu.runFrom(gpa, &state, prog, 0, 1 << 30);
            if (!state.halted or state.y_mem.get(0x3307).? != 0) return error.Decode;

            var share: [stages.len + 1]u64 = @splat(0);
            var other: u64 = total;
            for (begins, ends, 0..) |b, e, s| {
                for (profile[b..e]) |c| share[s] += c;
                other -= share[s];
            }
            share[stages.len] = other;
            const per: f64 = @floatFromInt(n * cfg[5]);
            std.debug.print("{s:<14} {d:>3} {d:>5} {d:>3} {d:>10} {d:>8.1}", .{ name, k, n, cfg[5], total, @as(f64, @floatFromInt(total)) / per });
            for (share) |c| std.debug.print(" {d:>7.1}", .{@as(f64, @floatFromInt(c)) / per});
            const rt = @as(f64, @floatFromInt(total)) / per * @as(f64, @floatFromInt(fx[1] * cfg[5])) / bundles_per_s;
            std.debug.print(" {d:>5.0}%\n", .{100 * rt});
        }
    }
}
