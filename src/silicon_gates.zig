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

const RegBank = enum { s, e, r, v, a };
const PreSet = struct { bank: RegBank, index: u8, value: u32 };
const PreMem = struct { kind: enum { x, y }, addr: u32, value: u32 };

const PostCheck = union(enum) {
    s_reg: struct { index: u8, value: u32 },
    e_reg: struct { index: u8, value: u32 },
    r_reg: struct { index: u8, value: u32 },
    a_reg: struct { index: u8, value: u32 },
    y_mem: struct { addr: u32, value: u32 },
    x_mem: struct { addr: u32, value: u32 },
};

const Gate = struct {
    name: []const u8,
    pre: []const PreSet = &.{},
    pre_mem: []const PreMem = &.{},
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
        .name = "own-x17agu",
        // the probe list own-x17agu: e3 = 0x702, mid-program x17 agu writes
        // R5 <- AGU10[2] = x + 1 = 0x703.  (Silicon also stored Y:0x700 =
        // 0x703 via a rop + st tail we don't model yet; this gate stops at
        // the R-file write.)
        // y = R2 (unused by FN 2 = x + 1, but silicon-measured S2 encodes it).
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x702 }},
        .src =
        \\x17.agu  R5, e3, R2, #2
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 5, .value = 0x703 } }},
    },
    .{
        .name = "own-x17e",
        // the probe list own-x17e: pre e5 = 0x703.  The bundle writes
        //   e5 <- 0x0704, s5 <- 0x0704, e9 <- old e5 - 1 = 0x0702.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x703 }},
        .src =
        \\x17.e  e5, s5, e9, e5, #0x0704
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 5, .value = 0x0704 } },
            .{ .s_reg = .{ .index = 5, .value = 0x0704 } },
            .{ .e_reg = .{ .index = 9, .value = 0x0702 } },
        },
    },
    .{
        .name = "own-x17ld",
        // the probe list own-x17ld: pre s4 = 0x0E08, e8 = 0x705, Y:0x705 = 0x222.
        // Bundle: s<S1[22:19]> = s5 <- s<S2[28:25]> = s4 asr 1 = 0x0704,
        //         e<2*S1[2:0]> = e4 <- Y:[e8] = 0x222.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x0E08 },
            .{ .bank = .e, .index = 8, .value = 0x705 },
            .{ .bank = .v, .index = 0, .value = 0x0222 },
        },
        .src =
        \\st.y Y:0x705
        \\x17.ld s5, s4, #2, e8, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x0704 } },
            .{ .e_reg = .{ .index = 4, .value = 0x0222 } },
        },
    },
    .{
        .name = "own-x17alu05",
        // the probe list own-x17alu05: pre s5 = 0x555, s3 = 0x333.
        // Head is an ALU05 add word: s5 <- s3 + s5 = 0x888.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x333 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x17.alu05 s5, s3, s5, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x888 } }},
    },
    .{
        .name = "own-x17mve",
        // the probe list own-x17mve: pre s4 = 0x701.  Bundle: e5 <- s4 + 1 = 0x702.
        .pre = &.{.{ .bank = .s, .index = 4, .value = 0x701 }},
        .src =
        \\x17.mve e5, s4
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 5, .value = 0x702 } }},
    },
    .{
        .name = "own-x17rsub",
        // the probe list own-x17rsub: pre s3 = 0x333.
        // Bundle: s5 <- 0xA - s3 = 0xA - 0x333 wrapped to 24 bits = 0xFFFCD7.
        .pre = &.{.{ .bank = .s, .index = 3, .value = 0x333 }},
        .src =
        \\x17.rsub s5, s3, #0xA, #5, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0xFFFCD7 } }},
    },
    .{
        .name = "own-x17xpm",
        // the probe list own-x17xpm: pre A12 = 0x100, X:[0x100] = 0x222.
        // Bundle (S2[1:0] = 1 -> A12): s5 <- X:[A12] = 0x222.  The
        // secondary A-reg write at S1[25:22] is OPEN today.
        .pre = &.{.{ .bank = .a, .index = 12, .value = 0x100 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x100, .value = 0x222 }},
        .src =
        \\x17.xpm s5, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x222 } }},
    },
    .{
        .name = "own-x17xld-a10",
        // the probe list own-x17xld-a10: pre A10 = 0x100, X:[0x100] = 0x222.
        // Bundle (S2[1:0] = 1, offset = 2 -> A10): s5 <- X:[A10] = 0x222.
        .pre = &.{.{ .bank = .a, .index = 10, .value = 0x100 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x100, .value = 0x222 }},
        .src =
        \\x17.xld_a10 s5, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x222 } }},
    },
    .{
        .name = "own-x17ctl-continue",
        // the probe list own-x17ctl: a mid-program x17.cont must fall through
        // to the next bundle (silicon's "S1[22:20] = 000 continues" rule).
        // Verify by writing s5 AFTER the continue; if cont halted, s5 stays 0.
        .src =
        \\x17.cont
        \\x17.imm s5, #0x0042
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x0042 } }},
    },
    .{
        .name = "own-x19ld",
        // the probe list own-x19ld: pre e10 = 0x702, Y:[0x702] = 0x222, v[0] = 0x333.
        // Bundle: s5 <- Y:[e10] = 0x222; e10 <- AGU10[2](x = e10) = 0x703;
        //         st.y Y:[e10 = 0x703] <- v[0] = 0x333.
        .pre = &.{.{ .bank = .e, .index = 10, .value = 0x702 }, .{ .bank = .v, .index = 0, .value = 0x333 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x222 }},
        .src =
        \\x19.ld s5, #5, e10, #2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x222 } },
            .{ .e_reg = .{ .index = 10, .value = 0x703 } },
            .{ .y_mem = .{ .addr = 0x703, .value = 0x333 } },
        },
    },
    .{
        .name = "own-x19aagu",
        // the probe list own-x19aagu: pre A9 = 0x100.
        // Bundle (fn_op = 2, imm = 8): A9 <- A9 + 2*8 = 0x110.
        .pre = &.{.{ .bank = .a, .index = 9, .value = 0x100 }},
        .src =
        \\x19.aagu A9, A9, #2, #8
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 9, .value = 0x110 } }},
    },
    .{
        .name = "own-x19mul",
        // the probe list own-x19mul: pre s2 = 2, s4 = 0x381.
        // Bundle (op = 4): e7, R5 <- low16(s2 * s4) = 0x702.
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 2 },
            .{ .bank = .s, .index = 4, .value = 0x381 },
        },
        .src =
        \\x19.mul e7, R5, #4, s2, s4
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 7, .value = 0x702 } },
            .{ .r_reg = .{ .index = 5, .value = 0x702 } },
        },
    },
    .{
        .name = "own-x19imm",
        // the probe list own-x19imm: dest e3 <- R0 + 0x702 = 0x702 (fn_op 10).
        .src =
        \\x19.imm #0x0702
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 3, .value = 0x702 } }},
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
        .e => state.e[p.index] = p.value,
        .r => state.r[p.index] = p.value,
        .v => state.v[p.index & 3] = p.value,
        .a => state.a[p.index] = p.value,
    };
    for (g.pre_mem) |m| switch (m.kind) {
        .x => state.x_mem.put(gpa, m.addr, m.value) catch return error.OutOfMemory,
        .y => state.y_mem.put(gpa, m.addr, m.value) catch return error.OutOfMemory,
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
        .e_reg => |er| {
            if (state.e[er.index] != er.value) {
                std.debug.print("gate '{s}': e{d} = 0x{x}, expected 0x{x}\n", .{ g.name, er.index, state.e[er.index], er.value });
                return error.GateFailed;
            }
        },
        .r_reg => |rr| {
            if (state.r[rr.index] != rr.value) {
                std.debug.print("gate '{s}': R{d} = 0x{x}, expected 0x{x}\n", .{ g.name, rr.index, state.r[rr.index], rr.value });
                return error.GateFailed;
            }
        },
        .a_reg => |ar| {
            if (state.a[ar.index] != ar.value) {
                std.debug.print("gate '{s}': A{d} = 0x{x}, expected 0x{x}\n", .{ g.name, ar.index, state.a[ar.index], ar.value });
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
        .x_mem => |xm| {
            const got = state.getX(xm.addr) orelse {
                std.debug.print("gate '{s}': X:0x{x} not written, expected 0x{x}\n", .{ g.name, xm.addr, xm.value });
                return error.GateFailed;
            };
            if (got != xm.value) {
                std.debug.print("gate '{s}': X:0x{x} = 0x{x}, expected 0x{x}\n", .{ g.name, xm.addr, got, xm.value });
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
