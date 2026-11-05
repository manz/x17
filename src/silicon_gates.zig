//! Silicon-measured gates: pre-state + .sm source + expected post-state.
//!
//! Each entry mirrors a program shipped on real hardware in the RE tree
//! tree's `the probe list`.  The harness assembles the source with x17's
//! assembler, runs the bundles through x17's emulator, and asserts the
//! final state matches what silicon wrote.  A failure here means one of:
//!   * assembler regression,
//!   * emulator rule missing or wrong,
//!   * isa.toml refresh changed a base word out from under us.
//!
//! Gates live inline so the test is self-contained; later, we can migrate
//! them into a shared `.toml` file as the catalog grows.

const std = @import("std");
const asm_mod = @import("asm");
const emu = @import("emu");

const RegBank = enum { s, v };
const PreSet = struct { bank: RegBank, index: u8, value: u32 };

const PostCheck = union(enum) {
    s_reg: struct { index: u8, value: u32 },
    y_mem: struct { addr: u32, value: u32 },
};

const Gate = struct {
    name: []const u8,
    pre: []const PreSet = &.{},
    src: []const u8,
    post: []const PostCheck,
    lane: u8 = 0,
};

const GATES = [_]Gate{
    .{
        .name = "own-x17alu-shl",
        // the probe list own-x17alu-shl: s8 = 0x246, s2 = 1.
        .pre = &.{
            .{ .bank = .s, .index = 8, .value = 0x246 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x17.alu  s1, s8, s2, #0, #3
        \\x17.halt
        ,
        // Silicon stored 0x1238 at Y:0x700; the emu writes s1 directly.
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x1238 } }},
    },
    .{
        .name = "own-x17alu-rsub",
        .pre = &.{
            .{ .bank = .s, .index = 8, .value = 0x246 },
            .{ .bank = .s, .index = 4, .value = 0x701 },
        },
        .src =
        \\x17.alu  s1, s8, s4, #4, #3
        \\x17.halt
        ,
        // b - a = 0x701 - 0x246 = 0x4BB.  Silicon: Y:0x700 = 0x4BB.
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x4BB } }},
    },
    .{
        .name = "own-x17imm",
        .pre = &.{},
        .src =
        \\x17.imm  s5, #0x0BEE
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0xBEE } }},
    },
    .{
        .name = "own-st3-lane2",
        // the probe list own-st3 at lane 2: V[2] = 2 (fact 21).
        .pre = &.{.{ .bank = .v, .index = 2, .value = 2 }},
        .src =
        \\st.y  Y:0x200
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x200, .value = 2 } }},
        .lane = 2,
    },
    .{
        .name = "alu-add-roundtrip",
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x100 },
            .{ .bank = .s, .index = 9, .value = 0x200 },
        },
        .src =
        \\alu  s7, s2, s9, #0x00
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x300 } }},
    },
};

fn runGate(gpa: std.mem.Allocator, g: Gate) !void {
    var state = emu.State{};
    defer state.deinit(gpa);
    for (g.pre) |p| switch (p.bank) {
        .s => state.s[p.index] = p.value,
        .v => state.v[p.index & 3] = p.value,
    };
    state.cur_lane = g.lane;

    const result = try asm_mod.assembleFile(gpa, g.src);
    const words = switch (result) {
        .ok => |w| w,
        .err => |fe| {
            std.debug.print("gate '{s}' assembler error L{d}: {s}\n", .{ g.name, fe.line, fe.diag.msg() });
            return error.AssembleFailed;
        },
    };
    defer gpa.free(words);

    _ = try emu.runLinear(gpa, &state, words);

    for (g.post) |check| switch (check) {
        .s_reg => |sr| {
            if (state.s[sr.index] != sr.value) {
                std.debug.print("gate '{s}': s{d} = 0x{x}, expected 0x{x}\n", .{ g.name, sr.index, state.s[sr.index], sr.value });
                return error.GateFailed;
            }
        },
        .y_mem => |ym| {
            const got = state.getY(ym.addr) orelse {
                std.debug.print("gate '{s}': Y:0x{x} not written, expected 0x{x}\n", .{ g.name, ym.addr, ym.value });
                return error.GateFailed;
            };
            if (got != ym.value) {
                std.debug.print("gate '{s}': Y:0x{x} = 0x{x}, expected 0x{x}\n", .{ g.name, ym.addr, got, ym.value });
                return error.GateFailed;
            }
        },
    };
}

test "silicon gates: all pass" {
    for (GATES) |g| try runGate(std.testing.allocator, g);
}

test "silicon gate count is reported" {
    // Low-value sanity check so a dropped entry gets caught.
    try std.testing.expect(GATES.len >= 5);
}
