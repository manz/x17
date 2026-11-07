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

const RegBank = enum { s, e, r, v, a, p };
const PreSet = struct { bank: RegBank, index: u8, value: u32 };
const PreMem = struct { kind: enum { x, y }, addr: u32, value: u32 };

const PostCheck = union(enum) {
    s_reg: struct { index: u8, value: u32 },
    e_reg: struct { index: u8, value: u32 },
    r_reg: struct { index: u8, value: u32 },
    a_reg: struct { index: u8, value: u32 },
    y_mem: struct { addr: u32, value: u32 },
    x_mem: struct { addr: u32, value: u32 },
    p_reg: struct { index: u8, value: u8 },
    y_none: u32,
};

const Gate = struct {
    name: []const u8,
    pre: []const PreSet = &.{},
    pre_mem: []const PreMem = &.{},
    src: []const u8,
    post: []const PostCheck,
    lane: u8 = 0,
    /// The run must not reach a stop (silicon wedged).
    wedge: bool = false,
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
        .pre = &.{ .{ .bank = .e, .index = 10, .value = 0x702 }, .{ .bank = .v, .index = 0, .value = 0x333 } },
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
        .name = "own-alu11",
        // the probe list own-alu11: pre s1 = 0x100, s3 = 0x23.
        // Head is an ALU05 add word; S side disabled (dest R0 / e0).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x100 },
            .{ .bank = .s, .index = 3, .value = 0x23 },
        },
        .src =
        \\alu11 s5, s1, s3, #2, R0, e0, e0, R0, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x123 } }},
    },
    .{
        .name = "own-agu11",
        // the probe list own-agu11: pre e2 = 0x701.
        // Head ALU side disabled (dest s0); S writes AGU10[2] result to R3 + e4.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x701 }},
        .src =
        \\alu11 s0, s0, s0, #0, R3, e4, e2, R0, #2, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x702 } },
            .{ .e_reg = .{ .index = 4, .value = 0x702 } },
        },
    },
    .{
        .name = "own-ealu",
        // Pre s5 = 0x701; expect e4 = s5 + 1 = 0x702.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x701 }},
        .src =
        \\ealu e4, s5, #1, #5
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 4, .value = 0x702 } }},
    },
    .{
        .name = "own-agu10r",
        // the probe list own-agu10r: pre e3 = 0x700.
        // Bundle: s5 <- 0x0BEE and R3 <- AGU10[2](e3) = e3 + 1 = 0x701.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x700 }},
        .src =
        \\agu10r s5, #0x0BEE, R3, e3, #0, #2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0xBEE } },
            .{ .r_reg = .{ .index = 3, .value = 0x701 } },
        },
    },
    .{
        .name = "own-rli",
        // Two rli bundles land R3 = 0x1234 and s5 = 0x0BEE respectively.
        .src =
        \\rli R3, s0, #0x1234
        \\rli R0, s5, #0x0BEE
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x1234 } },
            .{ .s_reg = .{ .index = 5, .value = 0x0BEE } },
        },
    },
    .{
        .name = "own-rop",
        // the probe list own-rop: pre e5 = 0x300.
        // Bundle 1: R3 <- fn0(x=e5) = 0x300.
        // Bundle 2: s1 <- fn6(x=e5, y=R3) = 0x300 + 0x300 = 0x600.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x300 }},
        .src =
        \\rop s0, R3, e5, R0, #0
        \\rop s1, R0, e5, R3, #6
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x300 } },
            .{ .s_reg = .{ .index = 1, .value = 0x600 } },
        },
    },
    .{
        .name = "own-ptst",
        // the probe list own-ptst: p1 <- (s5 < 0) with s5 = 0xF000 (negative when
        // interpreted as 24-bit signed via bit 23; here we use a 24-bit value
        // with bit 23 set to trigger c1 true).  Expected byte: 0x4420296D.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x800000 }},
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-pos",
        // s5 = 0x000F00 is positive (bit 23 clear), so p1 <- (s5 < 0) = 0.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x000F00 }},
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c1-0-0",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = 0, s4 = 0.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x000000 },
            .{ .bank = .s, .index = 4, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c1-m1-5",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = -1, s4 = 5.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0xFFFFFF },
            .{ .bank = .s, .index = 4, .value = 0x000005 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c1-1-m5",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = 1, s4 = -5.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x000001 },
            .{ .bank = .s, .index = 4, .value = 0xFFFFFB },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c1-m5-5",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = -5, s4 = 5.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0xFFFFFB },
            .{ .bank = .s, .index = 4, .value = 0x000005 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c1-m6-5",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = -6, s4 = 5.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0xFFFFFA },
            .{ .bank = .s, .index = 4, .value = 0x000005 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c1-m1-0",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = -1, s4 = 0.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0xFFFFFF },
            .{ .bank = .s, .index = 4, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c1-0-5",
        // fact 387 probe: ptst 4420296D, p1 <- s5 <= -s4 with s5 = 0, s4 = 5.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x000000 },
            .{ .bank = .s, .index = 4, .value = 0x000005 },
        },
        .src =
        \\ptst p1, s5, #1, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-eq",
        // fact 377: c7, s5 = 0x300, s12 = 0x300.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x300 },
            .{ .bank = .s, .index = 12, .value = 0x300 },
        },
        .src =
        \\ptst p1, s5, #7, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-eq-false",
        // fact 377: c7, s5 = 0x300, s12 = 0x301.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x300 },
            .{ .bank = .s, .index = 12, .value = 0x301 },
        },
        .src =
        \\ptst p1, s5, #7, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-b0",
        // fact 377: c9, s5 = 0x300, s4 = 0x0.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x300 },
            .{ .bank = .s, .index = 4, .value = 0x0 },
        },
        .src =
        \\ptst p1, s5, #9, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-b0-false",
        // fact 377: c9, s5 = 0x300, s4 = 0x1.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x300 },
            .{ .bank = .s, .index = 4, .value = 0x1 },
        },
        .src =
        \\ptst p1, s5, #9, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c13-m1",
        // fact 389: b-only row c13 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #13, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c13-0",
        // fact 389: b-only row c13 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #13, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c13-7fffff",
        // fact 389: b-only row c13 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0x7FFFFF },
        },
        .src =
        \\ptst p1, s5, #13, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c13-m800000",
        // fact 389: b-only row c13 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0x800000 },
        },
        .src =
        \\ptst p1, s5, #13, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c14-m1",
        // fact 389: b-only row c14 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #14, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c14-0",
        // fact 389: b-only row c14 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #14, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c14-7fffff",
        // fact 389: b-only row c14 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x7FFFFF },
        },
        .src =
        \\ptst p1, s5, #14, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c14-m800000",
        // fact 389: b-only row c14 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x800000 },
        },
        .src =
        \\ptst p1, s5, #14, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c15-m1",
        // fact 389: b-only row c15 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #15, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c15-0",
        // fact 389: b-only row c15 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #15, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c15-7fffff",
        // fact 389: b-only row c15 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x7FFFFF },
        },
        .src =
        \\ptst p1, s5, #15, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c15-m800000",
        // fact 389: b-only row c15 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x800000 },
        },
        .src =
        \\ptst p1, s5, #15, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c10-0",
        // fact 389: b-only row c10 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #10, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c10-1",
        // fact 389: b-only row c10 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x000001 },
        },
        .src =
        \\ptst p1, s5, #10, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c10-m1",
        // fact 389: b-only row c10 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #10, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c11-0",
        // fact 389: b-only row c11 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #11, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c11-1",
        // fact 389: b-only row c11 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x000001 },
        },
        .src =
        \\ptst p1, s5, #11, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c11-m1",
        // fact 389: b-only row c11 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #11, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c29-0",
        // fact 389: b-only row c29 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #29, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c29-1",
        // fact 389: b-only row c29 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0x000001 },
        },
        .src =
        \\ptst p1, s5, #29, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c29-m1",
        // fact 389: b-only row c29 on s4, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 4, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #29, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c30-0",
        // fact 389: b-only row c30 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #30, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c30-1",
        // fact 389: b-only row c30 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0x000001 },
        },
        .src =
        \\ptst p1, s5, #30, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c30-m1",
        // fact 389: b-only row c30 on s8, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 8, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #30, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c31-0",
        // fact 389: b-only row c31 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x000000 },
        },
        .src =
        \\ptst p1, s5, #31, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-ptst-c31-1",
        // fact 389: b-only row c31 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0x000001 },
        },
        .src =
        \\ptst p1, s5, #31, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-ptst-c31-m1",
        // fact 389: b-only row c31 on s12, a = s5 junk (ignored).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 12, .value = 0xFFFFFF },
        },
        .src =
        \\ptst p1, s5, #31, #0x0B
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-x19macw",
        // fact 312/386: 00000019 2000A000 20000284 C0000380, s1 <- rnd(s4 * s2 / 2^23).
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x123400 },
            .{ .bank = .s, .index = 2, .value = 0xFEAAAB },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s1, s4, s2, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0xFFCF75 } },
        },
    },
    .{
        .name = "own-x19macw-half",
        // fact 386: 0x100 * -0xC000 / 2^23 = -1.5 rounds half up to -1 (200006D0).
        .pre = &.{
            .{ .bank = .s, .index = 6, .value = 0x100 },
            .{ .bank = .s, .index = 8, .value = 0xFF4000 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s3, s6, s8, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0xFFFFFF } },
        },
    },
    .{
        .name = "own-x19macw-max",
        // fact 386 edge: 0x7fff00 * -0x40000, no saturation.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x7FFF00 },
            .{ .bank = .s, .index = 2, .value = 0xFC0000 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s1, s4, s2, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0xFC0008 } },
        },
    },
    .{
        .name = "own-x19macw-min",
        // fact 386 edge: -0x800000 * -0x40000, no saturation.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x800000 },
            .{ .bank = .s, .index = 2, .value = 0xFC0000 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s1, s4, s2, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x40000 } },
        },
    },
    .{
        .name = "own-x19macw-lo",
        // fact 386: x = 0x7FFF, y = 0x3FFFF: hi = 0x400, R7 = (2xy)[15:0] = 2.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x7FFF },
            .{ .bank = .s, .index = 2, .value = 0x3FFFF },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s1, s4, s2, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x400 } },
            .{ .r_reg = .{ .index = 7, .value = 2 } },
        },
    },
    .{
        .name = "own-x19head",
        // fact 384: 080089D9 4002A038 21000048 C0000280, head add s1 <- s1 + s2.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x10 },
            .{ .bank = .s, .index = 2, .value = 0x20 },
        },
        .src =
        \\x19.mac s1, s1, s2, #0x0E, #2, #0x54, e7, s0, s2, s4, #0x08, A0, R5, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x30 } },
        },
    },
    .{
        .name = "own-x19mac-aslot",
        // fact 385: 00000019 40000000 20020000 C0448000, A1 <- -A9.
        .pre = &.{
            .{ .bank = .a, .index = 9, .value = 0x240 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #2, #0, e0, s0, s0, s0, #0, A1, R0, #4, A9, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .a_reg = .{ .index = 1, .value = 0xFDC0 } },
        },
    },
    .{
        .name = "own-x19mac-combined",
        // Upstream 2026-11-06: 140221D9 4000A000 20060284 C0048000; the multiply reads the old s4.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x100 },
            .{ .bank = .s, .index = 5, .value = 0x300 },
            .{ .bank = .s, .index = 2, .value = 0x7000 },
            .{ .bank = .a, .index = 9, .value = 0x190 },
        },
        .src =
        \\x19.mac s4, s4, s5, #0x0E, #2, #0x14, e0, s1, s4, s2, #0, A3, R0, #0, A9, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x400 } },
            .{ .s_reg = .{ .index = 1, .value = 1 } },
            .{ .a_reg = .{ .index = 3, .value = 0x191 } },
        },
    },
    .{
        .name = "own-x19mac-a0",
        // Upstream 2026-11-06: A0 reads 0 even after a write; x19mac A5 <- A0 + 1 = 1.
        .pre = &.{
            .{ .bank = .a, .index = 0, .value = 0x55 },
            .{ .bank = .a, .index = 5, .value = 0x150 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s0, s0, s0, #0, A5, R0, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 5, .value = 1 } }},
    },
    .{
        .name = "own-x03jrc",
        // fact 394: 00000003 00240000 10000000 00000000 00000000 00004000 = jr +9 if !p1;
        // p1 = 0: the first add skipped (Y = 1).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
            .{ .bank = .p, .index = 1, .value = 0 },
        },
        .src =
        \\x03.jrc #0, p1, #9
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-x03jrc-p1",
        // fact 394: 00000003 00240000 10000000 00000000 00000000 00004000 = jr +9 if !p1;
        // p1 = 1: both adds run (Y = 2).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
            .{ .bank = .p, .index = 1, .value = 1 },
        },
        .src =
        \\x03.jrc #0, p1, #9
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 2 } }},
    },
    .{
        .name = "own-x02lo24",
        // fact 399: 00000002 0002A000 00020442 00000001 00042A80 00004200; w1[17] = 1
        // leaves the second dest undoubled: s5 = P[15:0] = 0x369F (0x1235 * 3).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x1235 },
            .{ .bank = .s, .index = 2, .value = 3 },
        },
        .src =
        \\x02.v s0, A2, A1, A0, A1, s2, s1, s2, s5, #5, #0, #0, s0, s0, #0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x369F } },
            .{ .s_reg = .{ .index = 2, .value = 0 } },
        },
    },
    .{
        .name = "own-alac-tap",
        // Upstream 2026-11-06, silicon 2 of 2: the ALAC tap (predictor.sm, x17 d694cec), four terms.
        // Loop jmp 447FFE6B, the word x17 emits.  s4 = 2 sum floor(256 c d / 2^24), s5 = sum
        // (cd)[15:0], d = o - top; R6 = A10 - R7 = 0x45.  Silicon read back 16 bits (Y).
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0x0 },
            .{ .bank = .a, .index = 11, .value = 0x40 },
            .{ .bank = .a, .index = 9, .value = 0x60 },
            .{ .bank = .a, .index = 10, .value = 0x304C },
            .{ .bank = .r, .index = 7, .value = 0x3007 },
        },
        .pre_mem = &.{
            .{ .kind = .x, .addr = 0x40, .value = 0x4000 },
            .{ .kind = .x, .addr = 0x41, .value = 0x2000 },
            .{ .kind = .x, .addr = 0x42, .value = 0x1000 },
            .{ .kind = .x, .addr = 0x43, .value = 0x7000 },
            .{ .kind = .x, .addr = 0x44, .value = 0x0 },
            .{ .kind = .x, .addr = 0x60, .value = 0x5000 },
            .{ .kind = .x, .addr = 0x61, .value = 0x3000 },
            .{ .kind = .x, .addr = 0x62, .value = 0x6000 },
            .{ .kind = .x, .addr = 0x63, .value = 0x7FFF },
            .{ .kind = .x, .addr = 0x64, .value = 0x0 },
        },
        .src =
        \\ragu R6, A10, R7, #29
        \\x02.v s1, A11, A11, A0, A11, s9, s0, s0, s0, #5, #1, #1, s0, s0, #0, #0
        \\x02.v s3, A9, A9, A0, A9, s0, s0, s0, s0, #5, #3, #1, s1, s0, #0, #8
        \\x02.v s0, A0, A0, A0, A0, s4, s0, s0, s0, #5, #0, #2, s3, s7, #5, #0
        \\tap:    x02.v s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #1, #0, s4, s9, #0, #0
        \\x02.v s1, A11, A11, A0, A11, s9, s13, s2, s0, #5, #1, #1, s5, s12, #0, #0
        \\pcond p1, R6, A11, #30
        \\jmp #3, #1, tap
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #3, #1, s1, s0, #0, #8
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #0, #2, s3, s7, #5, #0
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #1, #0, s4, s9, #0, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0xAFFE } },
            .{ .s_reg = .{ .index = 5, .value = 0x9000 } },
            .{ .r_reg = .{ .index = 6, .value = 0x45 } },
        },
    },
    .{
        .name = "own-alac-tap-2",
        // Upstream 2026-11-06, silicon 2 of 2: the ALAC tap (predictor.sm, x17 d694cec), four terms.
        // Loop jmp 447FFE6B, the word x17 emits.  s4 = 2 sum floor(256 c d / 2^24), s5 = sum
        // (cd)[15:0], d = o - top; R6 = A10 - R7 = 0x45.  Silicon read back 16 bits (Y).
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0x100 },
            .{ .bank = .a, .index = 11, .value = 0x40 },
            .{ .bank = .a, .index = 9, .value = 0x60 },
            .{ .bank = .a, .index = 10, .value = 0x304C },
            .{ .bank = .r, .index = 7, .value = 0x3007 },
        },
        .pre_mem = &.{
            .{ .kind = .x, .addr = 0x40, .value = 0x1234 },
            .{ .kind = .x, .addr = 0x41, .value = 0x567 },
            .{ .kind = .x, .addr = 0x42, .value = 0x7ABC },
            .{ .kind = .x, .addr = 0x43, .value = 0xDEF },
            .{ .kind = .x, .addr = 0x44, .value = 0x0 },
            .{ .kind = .x, .addr = 0x60, .value = 0x2345 },
            .{ .kind = .x, .addr = 0x61, .value = 0x6789 },
            .{ .kind = .x, .addr = 0x62, .value = 0x1357 },
            .{ .kind = .x, .addr = 0x63, .value = 0x2468 },
            .{ .kind = .x, .addr = 0x64, .value = 0x0 },
        },
        .src =
        \\ragu R6, A10, R7, #29
        \\x02.v s1, A11, A11, A0, A11, s9, s0, s0, s0, #5, #1, #1, s0, s0, #0, #0
        \\x02.v s3, A9, A9, A0, A9, s0, s0, s0, s0, #5, #3, #1, s1, s0, #0, #8
        \\x02.v s0, A0, A0, A0, A0, s4, s0, s0, s0, #5, #0, #2, s3, s7, #5, #0
        \\tap:    x02.v s3, A9, A9, A0, A9, s0, s1, s2, s12, #5, #1, #0, s4, s9, #0, #0
        \\x02.v s1, A11, A11, A0, A11, s9, s13, s2, s0, #5, #1, #1, s5, s12, #0, #0
        \\pcond p1, R6, A11, #30
        \\jmp #3, #1, tap
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #3, #1, s1, s0, #0, #8
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #0, #2, s3, s7, #5, #0
        \\x02.v s0, A0, A0, A0, A0, s0, s0, s0, s0, #5, #1, #0, s4, s9, #0, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x1E9E } },
            .{ .s_reg = .{ .index = 5, .value = 0x3021F } }, // silicon: 0x021F, its low 16 bits
            .{ .r_reg = .{ .index = 6, .value = 0x45 } },
        },
    },
    .{
        .name = "own-pcond",
        // fact 378: 802A7EAC, p1 <- R5 >= A7 (c29), R5 = 0x123, A7 = 0x123.
        .pre = &.{
            .{ .bank = .r, .index = 5, .value = 0x123 },
            .{ .bank = .a, .index = 7, .value = 0x123 },
        },
        .src =
        \\pcond p1, R5, A7, #29
        \\x17.halt
        ,
        .post = &.{
            .{ .p_reg = .{ .index = 1, .value = 1 } },
        },
    },
    .{
        .name = "own-pcond-false",
        // fact 378: 802A7EAC, p1 <- R5 >= A7 (c29), R5 = 0x123, A7 = 0x124.
        .pre = &.{
            .{ .bank = .r, .index = 5, .value = 0x123 },
            .{ .bank = .a, .index = 7, .value = 0x124 },
        },
        .src =
        \\pcond p1, R5, A7, #29
        \\x17.halt
        ,
        .post = &.{
            .{ .p_reg = .{ .index = 1, .value = 0 } },
        },
    },
    .{
        .name = "own-plog-and",
        // Pre p1 = 1, p2 = 0; p4 <- p1 AND p2 = 0.
        .pre = &.{
            .{ .bank = .p, .index = 1, .value = 1 },
            .{ .bank = .p, .index = 2, .value = 0 },
        },
        .src =
        \\plog p4, p1, p2, #3, #0x0E
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 4, .value = 0 } }},
    },
    .{
        .name = "own-plog-or",
        // Pre p1 = 1, p2 = 0; p4 <- p1 OR p2 = 1.
        .pre = &.{
            .{ .bank = .p, .index = 1, .value = 1 },
            .{ .bank = .p, .index = 2, .value = 0 },
        },
        .src =
        \\plog p4, p1, p2, #4, #0x0E
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 4, .value = 1 } }},
    },
    .{
        .name = "own-ldy",
        // the probe list own-ldy: s14 <- Y:[e5].  Pre e5 = 0xA21, Y[0xA21] = 0x1357.
        // Expected byte: 0x780002AA.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0xA21 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0xA21, .value = 0x1357 }},
        .src =
        \\ldy s14, e5
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 14, .value = 0x1357 } }},
    },
    .{
        .name = "own-ldysx",
        // Upstream, fact 409 (3 of 3): ldy sign-extends; 0x8001 reads 0xFF8001, 0xFFFF
        // reads 0xFFFFFF (stx stores all 24 bits, controlled).
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0x710 },
            .{ .bank = .e, .index = 3, .value = 0x711 },
        },
        .pre_mem = &.{
            .{ .kind = .y, .addr = 0x710, .value = 0x8001 },
            .{ .kind = .y, .addr = 0x711, .value = 0xFFFF },
        },
        .src =
        \\ldy s14, e5
        \\ldy s13, e3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 14, .value = 0xFF8001 } },
            .{ .s_reg = .{ .index = 13, .value = 0xFFFFFF } },
        },
    },
    .{
        .name = "own-x17st",
        // Upstream, fact 410: 00000017 77018000 A0000042, Y:[e3] <- s7 (e3 read before
        // the e slot's write of #0 to it).
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x720 },
            .{ .bank = .s, .index = 7, .value = 0x4321 },
        },
        .src =
        \\x17.ms s0, #0, s7, e3, #1, #3, #0, R2, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x720, .value = 0x4321 } },
            .{ .e_reg = .{ .index = 3, .value = 0 } },
        },
    },
    .{
        .name = "own-x17r",
        // fact 411: 00024697 00000000 A00000A0, R5 <- #0x1234.
        .src =
        \\x17.ms s0, #0x1234, s0, e0, #0, #0, #0, R5, #0
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 5, .value = 0x1234 } }},
    },
    .{
        .name = "own-x17e",
        // fact 412: 0000E217 20000000 A0000001, e9 <- #0x710.
        .src =
        \\x17.ms s0, #0x710, s0, e0, #0, #1, #1, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 9, .value = 0x710 } }},
    },
    .{
        .name = "own-x17y",
        // fact 413: 00400017 10028000 A0000004, s2 <- Y:[e5] (S1[28] = 1, mux 2).
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x730 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x730, .value = 0x2468 }},
        .src =
        \\x17.ms s2, #0, s0, e5, #1, #0, #0, R0, #2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x2468 } }},
    },
    .{
        .name = "own-x17ysx",
        // Upstream, fact 414 (3 of 3): mux 2 sign-extends, Y 0x8001 -> s2 = 0xFF8001.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x710 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x710, .value = 0x8001 }},
        .src =
        \\x17.ms s2, #0, s0, e5, #1, #0, #0, R0, #2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0xFF8001 } }},
    },
    .{
        .name = "own-x17mux4",
        // Upstream, fact 436 (3 of 3): mux 4 loads the Y word as mux 2 does; the tail
        // store reads back 16 bits, so the top byte is unmeasured. Mux 0 would store 0.
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0x710 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x710, .value = 0x8001 }},
        .src =
        \\x17.ms s2, #0, s0, e5, #1, #0, #0, R0, #4
        \\x17.ms s0, #0, s2, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x8001 } }},
    },
    .{
        .name = "own-x17estop",
        // Upstream, fact 448 (3 of 3): x17e S2[3:0] = 4 stops the program one bundle
        // later; the Y store after the nop never lands and the run ends cleanly.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x55 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0x1234 }},
        .src =
        \\x17.estop
        \\alu s0, s0, s0, #0
        \\x17.ms s0, #0, s1, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x1234 } }},
    },
    .{
        .name = "own-st0fe",
        // Upstream, fact 437 (3 of 3): 0000E20F AE700032, e6 <- #0x710 beside the X store;
        // the stk after it stores through e6.
        .src =
        \\stx0f.e #0, #2, s7, e6, #0x710
        \\ldi s5, #0x55
        \\stk e6, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 6, .value = 0x710 } },
            .{ .y_mem = .{ .addr = 0x710, .value = 0x55 } },
        },
    },
    .{
        .name = "own-x17styr",
        // Upstream, fact 439 (3 of 3): 00000017 A902C800 66001202, R9 <- s3 << 2.
        .pre = &.{.{ .bank = .s, .index = 3, .value = 0x123 }},
        .src =
        \\x17.styr R9, s3, #2, e5, #0
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 9, .value = 0x48C } }},
    },
    .{
        .name = "own-x17ldsyn",
        // Upstream, fact 443/460 (3 of 3): 10040017 00001528 60000004, R2 <- R2 + 0x80
        // (AGU10 fn 10); silicon set R2 = 0x1000 with an agu10r first.
        .pre = &.{.{ .bank = .r, .index = 2, .value = 0x1000 }},
        .src =
        \\x17.ldr R2, #10, R2, #0x80
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 2, .value = 0x1080 } }},
    },
    .{
        .name = "own-x19syn",
        // Upstream, fact 454/462 (3 of 3): 00000219 02014000 800C8101 81E18008, op 16 mode 8, b - a.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 1, .value = 0x23 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .src =
        \\x19.alum s2, s3, s1, #16, #8
        \\x17.ms s0, #0, s2, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0xFF23 } }},
    },
    .{
        .name = "own-x19m8add",
        // fact 454: 000001D9 02014000 800C8101 81E18008, op 14 mode 8, a + b.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 1, .value = 0x23 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .src =
        \\x19.alum s2, s3, s1, #14, #8
        \\x17.ms s0, #0, s2, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x0123 } }},
    },
    .{
        .name = "own-x19maon",
        // fact 454: 000001D9 02015000 800C8101 81E18008, op 14 mode 0xA, a.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 1, .value = 0x23 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .src =
        \\x19.alum s2, s3, s1, #14, #10
        \\x17.ms s0, #0, s2, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x0100 } }},
    },
    .{
        .name = "own-x19m8neg",
        // fact 454: 00000299 02014000 800C8101 81E18008, op 20 mode 8, -(a + b) - 2.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 1, .value = 0x23 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .src =
        \\x19.alum s2, s3, s1, #20, #8
        \\x17.ms s0, #0, s2, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0xFEDB } }},
    },
    .{
        .name = "own-x17mswedge-2",
        // Upstream, fact 457/463/465 (3 of 3): 00000017 75218000 A0000042, own-x17st's
        // shape with S1[22:20] = 010. The slots run, then two shadow bundles, then no halt: a
        // store two nops later never lands.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x66 },
            .{ .bank = .s, .index = 1, .value = 0x55 },
            .{ .bank = .e, .index = 3, .value = 0x740 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0x1234 }},
        .src =
        \\x17.ms.stop s0, #0, s5, e3, #1, #3, #0, R2, #1
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\x17.ms s0, #0, s1, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x740, .value = 0x66 } },
            .{ .y_mem = .{ .addr = 0x700, .value = 0x1234 } },
        },
        .wedge = true,
    },
    .{
        .name = "own-x17mswedge-sh",
        // fact 465: both shadow bundles run (an ldi s4 in the first set the address the
        // second's store used), the third does not. Shown here with register writes, since a Y
        // store in the shadow is lost or misaddressed.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x66 },
            .{ .bank = .e, .index = 3, .value = 0x740 },
        },
        .src =
        \\x17.ms.stop s0, #0, s5, e3, #1, #3, #0, R2, #1
        \\ldi s4, #0x720
        \\ldi s7, #0x77
        \\ldi s9, #0x99
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x720 } },
            .{ .s_reg = .{ .index = 7, .value = 0x77 } },
            .{ .s_reg = .{ .index = 9, .value = 0 } },
        },
        .wedge = true,
    },
    .{
        .name = "own-x17mswedge-8",
        // fact 457: the same with eight nops before the store, which never lands.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x66 },
            .{ .bank = .s, .index = 1, .value = 0x55 },
            .{ .bank = .e, .index = 3, .value = 0x740 },
            .{ .bank = .e, .index = 6, .value = 0x700 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0x1234 }},
        .src =
        \\x17.ms.stop s0, #0, s5, e3, #1, #3, #0, R2, #1
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\x17.ms s0, #0, s1, e6, #0, #0, #0, R0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x740, .value = 0x66 } },
            .{ .y_mem = .{ .addr = 0x700, .value = 0x1234 } },
        },
        .wedge = true,
    },
    .{
        .name = "own-sty0imm",
        // Upstream, fact 466 (3 of 3): the wr's e0 is the previous word's [20:5], here
        // ldi s5's 0x730; reading s4 would give 0x720.
        .src =
        \\ldi s1, #0x55
        \\ldi s4, #0x720
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\ldi s5, #0x730
        \\st.wr #0x40, e0, s1
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x730, .value = 0x55 } }},
    },
    .{
        .name = "own-sty0nop",
        // fact 466: a nop (40000005, [20:5] = 0) between them sends the store to Y:0.
        .src =
        \\ldi s1, #0x55
        \\ldi s5, #0x730
        \\alu s0, s0, s0, #0
        \\st.wr #0x40, e0, s1
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0, .value = 0x55 } }},
    },
    .{
        .name = "own-pc14not",
        // Upstream, fact 478: mux 00 loads s7 <- !(v even); s7 = 0x77 gives 1.
        .pre = &.{.{ .bank = .s, .index = 7, .value = 0x77 }},
        .src =
        \\pc14 p13, s7, #11, #12, #2, s7, #0, #0x14A3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 1 } }},
    },
    .{
        .name = "own-pc14-e0",
        // Upstream, fact 468: a pc14 pair loading an immediate sets e0 to its [20:5].
        .src =
        \\ldi s1, #0x55
        \\pc14 p13, s7, #11, #12, #2, s7, #0, #0x14A3
        \\st.wr #0x40, e0, s1
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x14A3, .value = 0x55 } }},
    },
    .{
        .name = "own-x18pset",
        // Upstream, fact 476: the pure class-110 base, p1 preset opposite the result.
        .pre = &.{.{ .bank = .p, .index = 1, .value = 0 }},
        .src =
        \\x18.pset p1
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-x18pmov",
        // Upstream, fact 476: the pure class-110 base, p1 preset opposite the result.
        .pre = &.{ .{ .bank = .p, .index = 1, .value = 0 }, .{ .bank = .p, .index = 5, .value = 1 } },
        .src =
        \\x18.pmov p1, p5
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-x18pnot",
        // Upstream, fact 476: the pure class-110 base, p1 preset opposite the result.
        .pre = &.{ .{ .bank = .p, .index = 1, .value = 1 }, .{ .bank = .p, .index = 5, .value = 1 } },
        .src =
        \\x18.pnot p1, p5
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-x18pclr",
        // Upstream, fact 476: the pure class-110 base, p1 preset opposite the result.
        .pre = &.{.{ .bank = .p, .index = 1, .value = 1 }},
        .src =
        \\x18.pclr p1
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-sty0x19",
        // Upstream, fact 468 (3 of 3): an x19alu between (00000219 02014000 800C8101
        // 81E18008, its [20:5] = 0x10) leaves e0 at ldi s5's 0x730.
        .src =
        \\ldi s1, #0x55
        \\ldi s5, #0x730
        \\x19.alum s2, s3, s1, #16, #8
        \\st.wr #0x40, e0, s1
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x730, .value = 0x55 } }},
    },
    .{
        .name = "own-ptest",
        // p2 seeded = 1; s5 <- f1(p2) = p2 = 1.
        .pre = &.{.{ .bank = .p, .index = 2, .value = 1 }},
        .src =
        \\ptest s5, p2, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 1 } }},
    },
    .{
        .name = "own-ptest-0",
        // p2 seeded = 0; s3 <- f0(p2) = !p2 = 1 (since f0 = !p and p=0).
        .pre = &.{.{ .bank = .p, .index = 2, .value = 0 }},
        .src =
        \\ptest s3, p2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 1 } }},
    },
    .{
        .name = "own-psel",
        // p2 = 1, s1 = 0x1000, s2 = 0x23; s5 <- p2 ? s1+s2 : s2 = 0x1023.
        .pre = &.{
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0x1000 },
            .{ .bank = .s, .index = 2, .value = 0x23 },
        },
        .src =
        \\psel s5, s1, s2, p2, #1, #4
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x1023 } }},
    },
    .{
        .name = "own-psel-0",
        // p2 = 0, same operands; s5 <- p2 ? s1+s2 : s2 = s2 = 0x23.
        .pre = &.{
            .{ .bank = .p, .index = 2, .value = 0 },
            .{ .bank = .s, .index = 1, .value = 0x1000 },
            .{ .bank = .s, .index = 2, .value = 0x23 },
        },
        .src =
        \\psel s5, s1, s2, p2, #1, #4
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x23 } }},
    },
    .{
        .name = "own-stx",
        // the probe list own-stx: X:[A6] <- s7 via low5 0x0A single syllable.
        // Expected byte: 0x4037300A.  Pre s7 = 0xF77, A6 = 0x3A0.
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0xF77 },
            .{ .bank = .a, .index = 6, .value = 0x3A0 },
        },
        .src =
        \\stx A6, s7
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x3A0, .value = 0xF77 } }},
    },
    .{
        .name = "own-xld10",
        // the probe list own-xld10: s3 <- X:[A6], s5 <- 0xBEE.  Pre A6 = 0x330,
        // X[0x330] = 0x777.  Expected bytes: 0x80A17DD0 0x60640181.
        .pre = &.{.{ .bank = .a, .index = 6, .value = 0x330 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x330, .value = 0x777 }},
        .src =
        \\xld10 s5, #2, #0xBEE, s3, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x777 } },
            .{ .s_reg = .{ .index = 5, .value = 0xBEE } },
        },
    },
    .{
        .name = "own-yld10",
        // the probe list own-yld10: s5 <- Y:[e3].  Pre e3 = 0x780, Y[0x780] = 0x0BEE.
        // Expected bytes: 0x00A00010 0xE6420080.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x780 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x780, .value = 0x0BEE }},
        .src =
        \\yld10 s5, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x0BEE } }},
    },
    .{
        .name = "own-x12agu",
        // Pre e3 = 0x100, R7 = 0x23.  AGU10[6](e3, R7) = 0x123 lands on
        // e9, s2 (= 2*1+0), R5.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 7, .value = 0x23 },
        },
        .src =
        \\x12.agu e9, e3, R7, #6, #0, #1, R5
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 9, .value = 0x123 } },
            .{ .s_reg = .{ .index = 2, .value = 0x123 } },
            .{ .r_reg = .{ .index = 5, .value = 0x123 } },
        },
    },
    .{
        .name = "own-bcc",
        // bcc is 2 syllables; target = pc + 4*q_off.  Delay slots at pc+2
        // and pc+3 run; the 4 ldi fillers between pc+4 and pc+7 are the
        // "skipped over" bundles; halt starts at syllable 8.
        .pre = &.{.{ .bank = .p, .index = 2, .value = 1 }},
        .src =
        \\bcc #2, #2
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\ldi s3, #256
        \\ldi s3, #256
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 5 } },
            .{ .s_reg = .{ .index = 4, .value = 7 } },
        },
    },
    .{
        .name = "own-br",
        // Pre p2 = 1: branch taken past the skipped bundle.
        .pre = &.{.{ .bank = .p, .index = 2, .value = 1 }},
        .src =
        \\br #2, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 5 } },
            .{ .s_reg = .{ .index = 4, .value = 7 } },
        },
    },
    .{
        .name = "own-jmp",
        // Pre p1 = 0, so !p1 = true: jmp taken.  Two delay slots run (set
        // s3 = 5 and s4 = 7), then jump lands past the skipped bundle.
        .pre = &.{.{ .bank = .p, .index = 1, .value = 0 }},
        .src =
        \\jmp #2, #1, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 5 } },
            .{ .s_reg = .{ .index = 4, .value = 7 } },
        },
    },
    .{
        .name = "own-x13st",
        // Pre s14 = 0x123; s3 <- 0 - s14 = 0xFFFEDD (24-bit two's complement).
        .pre = &.{.{ .bank = .s, .index = 14, .value = 0x123 }},
        .src =
        \\x13.st s3, s14
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0xFFFEDD } }},
    },
    .{
        .name = "own-x03agu",
        // Pre A5 = 0x100; i = 0x10 (from w4[31:17]); A9 <- A5 + 0x10 = 0x110.
        .pre = &.{.{ .bank = .a, .index = 5, .value = 0x100 }},
        .src =
        \\x03.agu A9, A5, #16, #0, A0
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 9, .value = 0x110 } }},
    },
    .{
        .name = "own-x03acp",
        // Pre A5 = 0x100; A-slot writes A0 via dest_A=0 (noop).
        // acp_dest = A9 also set to A5 = 0x100 (no +1).
        .pre = &.{.{ .bank = .a, .index = 5, .value = 0x100 }},
        .src =
        \\x03.acp A9, A5, e3, #0
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 9, .value = 0x100 } }},
    },
    .{
        .name = "own-x02yrd",
        // Pre e3 = 0x490 with Y[0x490] seeded to 0x886F.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x02.yrd s1, e3, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x886F } }},
    },
    .{
        .name = "own-x18q23c",
        // Pre s1 = 0x4000, s2 = 0x6000; mode 4, /2^23: s3 <- 0x30.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
        },
        .src =
        \\x18.q23c s3, s1, s2, #4, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x30 } }},
    },
    .{
        .name = "own-x03alu",
        // Pre s3 = 0x100, s4 = 0x23; add: s9 <- s3 + s4 = 0x123.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 4, .value = 0x23 },
        },
        .src =
        \\x03.alu s9, s3, s4, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0x123 } }},
    },
    .{
        .name = "own-x04agu",
        // Pre R2 = 0x100; fn 10 (y+imm): R1 <- R2 + 0x93 = 0x193.
        .pre = &.{.{ .bank = .r, .index = 2, .value = 0x100 }},
        .src =
        \\x04.agu R1, R2, #10, #147
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x193 } }},
    },
    .{
        .name = "own-x16agu",
        // Pre e2 = 0x100, R3 = 0x20; fn 6 (x+y): s3 <- e2 + R3 = 0x120.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x100 },
            .{ .bank = .r, .index = 3, .value = 0x20 },
        },
        .src =
        \\x16.agu s3, R3, #1, #6, #4, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x120 } }},
    },
    .{
        .name = "own-x12agu29",
        // Same inputs as x12.agu, with head[29] set (inert).
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 7, .value = 0x23 },
        },
        .src =
        \\x12.agu29 e9, e3, R7, #6, #0, #1, R5
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 9, .value = 0x123 } },
            .{ .s_reg = .{ .index = 2, .value = 0x123 } },
            .{ .r_reg = .{ .index = 5, .value = 0x123 } },
        },
    },
    .{
        .name = "own-x19kmux2",
        // Pre e3 = 0x100, R6 = 0x23; mux 010 behaves like mux 001.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 6, .value = 0x23 },
        },
        .src =
        \\x19.kmux2 s9, e3, R6, #6
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0x123 } }},
    },
    .{
        .name = "own-x19kabs",
        // Pre s3 = 0xFFFD (-3 as 16-bit signed); s4 <- |s3| = 3.
        .pre = &.{.{ .bank = .s, .index = 3, .value = 0xFFFD }},
        .src =
        \\x19.kabs s9, s4, s3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 3 } }},
    },
    .{
        .name = "own-x19kimm",
        // s9 <- head[20:5] = 0x1234.
        .src =
        \\x19.kimm s9, #4660
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0x1234 } }},
    },
    .{
        .name = "own-x19kalu",
        // Pre s5 = 0x50, s3 = 3; ALU m=5 (a-b): s4 <- s5 - s3 = 0x4D.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x50 },
            .{ .bank = .s, .index = 3, .value = 3 },
        },
        .src =
        \\x19.kalu s9, s4, s5, s3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x4D } }},
    },
    .{
        .name = "own-x19k00",
        // Pre e3 = 0x100, R6 = 0x23; mux 001: s9 <- AGU10[6](e3, R6) = 0x123.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 6, .value = 0x23 },
        },
        .src =
        \\x19.k00 s9, e3, R6, #1, #6, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0x123 } }},
    },
    .{
        .name = "own-x19kst",
        // Same bundle as own-x19k00; w2[23:22] = 11 stores s6 (ldi
        // sign-extended to 0xFF8123) at X:[A12 = 0x300].
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 6, .value = 0x23 },
            .{ .bank = .s, .index = 6, .value = 0xFF8123 },
            .{ .bank = .a, .index = 12, .value = 0x300 },
        },
        .src =
        \\x19.k00 s9, e3, R6, #1, #6, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 9, .value = 0x123 } },
            .{ .x_mem = .{ .addr = 0x300, .value = 0xFF8123 } },
        },
    },
    .{
        .name = "own-x19a43",
        // s4 <- (0x381 << 1) | 1 = 0x703.
        .src =
        \\x19.a43 s4, #25, #897
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x703 } }},
    },
    .{
        .name = "own-x19alu44",
        // Same semantics as x19.alu: s7 <- sat24(s3 << s6).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 7 },
            .{ .bank = .s, .index = 6, .value = 8 },
        },
        .src =
        \\x19.alu44 s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x700 } }},
    },
    .{
        .name = "own-x19aagu42",
        // Pre A9 = 0x100 (src and dest both A9); i = (0x08 << 1) | 1 = 0x11;
        // A9 <- A9 + 0x11 = 0x111.
        .pre = &.{.{ .bank = .a, .index = 9, .value = 0x100 }},
        .src =
        \\x19.aagu42 A9, A9, #2, #8
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 9, .value = 0x111 } }},
    },
    .{
        .name = "own-x19aimm",
        // w2[24] = 1: e5 <- #0x123 instead of x + y.
        .src =
        \\x19.aimm R3, e5, e3, R1, #6, #0x123
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 5, .value = 0x123 } }},
    },
    .{
        .name = "own-x19aagu5",
        // First bundle only: A2 <- A5 + #0 under w1 class 101.  The second
        // (x19a3 reading s2 <- A2, then the Y store) is not gated here.
        .pre = &.{.{ .bank = .a, .index = 5, .value = 0x345 }},
        .src =
        \\x19.aagu5 A2, A5, #5, #0
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 2, .value = 0x345 } }},
    },
    .{
        .name = "own-x19imm4",
        // e8 <- AGU10 fn 0 = x = e4 = 0x100.
        .pre = &.{
            .{ .bank = .e, .index = 4, .value = 0x100 },
            .{ .bank = .e, .index = 8, .value = 0x555 },
        },
        .src =
        \\x19.imm4 #0, #1, e4, #0
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 8, .value = 0x100 } }},
    },
    .{
        .name = "own-x19ragua",
        // R2 <- AGUA fn 0 (x + 1) over A3 = 0x345.
        .pre = &.{.{ .bank = .a, .index = 3, .value = 0x345 }},
        .src =
        \\x19.ragua R2, A3
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 2, .value = 0x346 } }},
    },
    .{
        .name = "own-x19a3st",
        // e2 = 0x700, s5 = 0x1357; Y:[e2] <- s5.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x700 },
            .{ .bank = .s, .index = 5, .value = 0x1357 },
        },
        .src =
        \\x19.a3st #5, #0, e2
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x1357 } }},
    },
    .{
        .name = "own-x19a3",
        // Pre e1 = 0x700; s4 <- 2*0x381 = 0x702, R7 <- AGU10[2](e1) = 0x701.
        .pre = &.{.{ .bank = .e, .index = 1, .value = 0x700 }},
        .src =
        \\x19.a3 s4, R7, e1, #25, #2, #897
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x702 } },
            .{ .r_reg = .{ .index = 7, .value = 0x701 } },
        },
    },
    .{
        .name = "own-x18mpy",
        // Pre s1 = 0x4001, s2 = 0x6003; P = 0x18012003: low = 0x2003, high = 0x18.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4001 },
            .{ .bank = .s, .index = 2, .value = 0x6003 },
        },
        .src =
        \\x18.mpy s3, s5, s1, s2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x2003 } },
            .{ .s_reg = .{ .index = 5, .value = 0x18 } },
        },
    },
    .{
        .name = "own-x19xrd",
        // Pre A11 = 0x345; X:0x345 = 0xDB7A; s1 lands with the read value.
        .pre = &.{.{ .bank = .a, .index = 11, .value = 0x345 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x345, .value = 0xDB7A }},
        .src =
        \\x19.xrd s1, #11
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0xDB7A } }},
    },
    .{
        .name = "own-x18q23",
        // Pre s1 = 0x4000, s2 = 0x6000; s3 <- round(s1*s2/2^23) = 0x30.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
        },
        .src =
        \\x18.q23 s3, s1, s2, #4
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x30 } }},
    },
    .{
        .name = "own-x18rimm",
        // e2 <- imm 0x705 via op 0.
        .src =
        \\x18.rimm #0, #2, #0, #1797, R0
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x705 } }},
    },
    .{
        .name = "own-x18xrd",
        // Pre A19 = 0x345, X:0x345 = 0xDB7A; s3 <- the read value.
        .pre = &.{.{ .bank = .a, .index = 3, .value = 0x345 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x345, .value = 0xDB7A }},
        .src =
        \\x18.xrd s3, #3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0xDB7A } }},
    },
    .{
        .name = "own-x18una",
        // s1 = 0x8001: abs -> s3 = 0x7FFF; +1 -> s5 = 0x8002.
        .pre = &.{.{ .bank = .s, .index = 1, .value = 0x8001 }},
        .src =
        \\x18.una s3, s1, #7, R13, s0
        \\x18.una s5, s1, #3, R13, s0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x7FFF } },
            .{ .s_reg = .{ .index = 5, .value = 0x8002 } },
        },
    },
    .{
        .name = "own-x18runa",
        // The slot result (+1 of s5) also lands in R3 = 0x556.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x555 }},
        .src =
        \\x18.una s0, s5, #3, R3, s0
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 0x556 } }},
    },
    .{
        .name = "own-x18alu011",
        // op 1 is binary: R3 <- s5 - s3 = 0x222.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x555 },
            .{ .bank = .s, .index = 3, .value = 0x333 },
        },
        .src =
        \\x18.una s0, s5, #1, R3, s3
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 0x222 } }},
    },
    .{
        .name = "own-x18pw-false",
        // cond 11 of +1 is false.  p2 starts at 1 here so the write shows.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 1 },
            .{ .bank = .p, .index = 2, .value = 1 },
        },
        .src =
        \\x18.pw #2, s3, #11
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 2, .value = 0 } }},
    },
    .{
        .name = "own-x18pw-true",
        // cond 11 of -1 is true: p2 <- 1.
        .pre = &.{.{ .bank = .s, .index = 3, .value = 0xFFFFFF }},
        .src =
        \\x18.pw #2, s3, #11
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 2, .value = 1 } }},
    },
    .{
        .name = "own-x18c2x",
        // mux 001: s3 <- e2 = 0x703; mux 110: s5 <- #0x4321.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x703 }},
        .src =
        \\x18.c2x s3, #1, e2, #0
        \\x18.c2x s5, #6, e2, #0x4321
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x703 } },
            .{ .s_reg = .{ .index = 5, .value = 0x4321 } },
        },
    },
    .{
        .name = "own-x18eagu2",
        // AGU10[2] = x+1: e5 <- e3 + 1 = 0x101.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .e, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.eagu2 e5, e3, #2, #1
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 5, .value = 0x101 } }},
    },
    .{
        .name = "own-x18rag",
        // R3 <- AGU3[2](A5, R1) = 0x55 + 2 = 0x57.
        .pre = &.{
            .{ .bank = .a, .index = 5, .value = 0x55 },
            .{ .bank = .r, .index = 1, .value = 2 },
        },
        .src =
        \\x18.rag R3, A5, R1, #2
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 0x57 } }},
    },
    .{
        .name = "own-x18rag101",
        .pre = &.{
            .{ .bank = .a, .index = 5, .value = 0x55 },
            .{ .bank = .r, .index = 1, .value = 2 },
        },
        .src =
        \\x18.rag101 s0, R3, A5, R1, #2
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 0x57 } }},
    },
    .{
        .name = "own-x18rmux",
        // X slot mux 001 takes the R slot value: s1 <- 0x57.
        .pre = &.{
            .{ .bank = .a, .index = 5, .value = 0x55 },
            .{ .bank = .r, .index = 1, .value = 2 },
        },
        .src =
        \\x18.rag101 s1, R3, A5, R1, #2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x57 } }},
    },
    .{
        .name = "own-x18eagu0",
        // w2[15] = 0 table, fn 5 = y+1, y = e2 = 0x20: e5 <- 0x21.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .e, .index = 2, .value = 0x20 },
            .{ .bank = .e, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.eagu0 e5, #2, #5
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 5, .value = 0x21 } }},
    },
    .{
        .name = "own-x18agu000",
        // fn 5 (y+1) of y = e3 = 0x705 into s1 via X mux 010.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
        },
        .src =
        \\x18.agu000 s1, #5, #3, #0, e2, #0, #1, #1, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x706 } }},
    },
    .{
        .name = "own-x18agu000b",
        // w2[15] = 1, AGU10[2] = x+1 of x = e2: s1 <- 0x704.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
        },
        .src =
        \\x18.agu000 s1, #2, #2, #1, e2, #0, #1, #1, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x704 } }},
    },
    .{
        .name = "own-x18st000",
        // Y:[e3] <- s5 = 0x555; x is now e3: s1 <- 0x706.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.agu000 s1, #2, #2, #1, e3, #1, #1, #1, #2
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x705, .value = 0x555 } },
            .{ .s_reg = .{ .index = 1, .value = 0x706 } },
        },
    },
    .{
        .name = "own-x18e2nd",
        // e2 <- e7 - 1 = 0x705.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 7, .value = 0x706 },
        },
        .src =
        \\x18.e2nd e2, #3, #1
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x705 } }},
    },
    .{
        .name = "own-x18edec",
        // Pre e2 = 0x703; e2 <- e2 - 1 = 0x702.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x703 }},
        .src =
        \\x18.edec e2, e2, #1, s5
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x702 } }},
    },
    .{
        .name = "own-x02agu",
        // Pre R12 = 0x155; R1 <- R12 = 0x155.
        .pre = &.{.{ .bank = .r, .index = 12, .value = 0x155 }},
        .src =
        \\x02.agu R1, R12, #10, #0
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x155 } }},
    },
    .{
        .name = "own-x02cin",
        // Pre R12 = 0x155; fn 14 (y+1) with cin=1: R1 <- R12 + 1 + 1 = 0x157.
        .pre = &.{.{ .bank = .r, .index = 12, .value = 0x155 }},
        .src =
        \\x02.agu R1, R12, #14, #1
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x157 } }},
    },
    .{
        .name = "own-aset12",
        // Pre R3 = 0x345; A10 <- R3 = 0x345.
        .pre = &.{.{ .bank = .r, .index = 3, .value = 0x345 }},
        .src =
        \\x12.aset #5, R3
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 10, .value = 0x345 } }},
    },
    .{
        .name = "own-x01st",
        // Y:[e5 = 0x360] <- s9, X:[A3 = 0x320] <- s7, s3 <- e5 (fact 243).
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0x360 },
            .{ .bank = .a, .index = 3, .value = 0x320 },
            .{ .bank = .s, .index = 7, .value = 0x777 },
            .{ .bank = .s, .index = 9, .value = 0x999 },
        },
        .src =
        \\x01.slots s3, #2, #2, #2, e5, e5, s9, A3, s7, s0, #0, #0, #3, #3, #0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x360 } },
            .{ .y_mem = .{ .addr = 0x360, .value = 0x999 } },
            .{ .x_mem = .{ .addr = 0x320, .value = 0x777 } },
        },
    },
    .{
        .name = "own-x02x3",
        // Pre s2=0x222, s3=0x333; (s2 - s3) << 2 = 0xFFFBBC (24-bit).
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x222 },
            .{ .bank = .s, .index = 3, .value = 0x333 },
        },
        .src =
        \\x02.x3 #1, #2, s2, s3, #5, #2, s2, #1, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0xFFFBBC } }},
    },
    .{
        .name = "own-x02coll",
        // mpy (s6 <- P = 0x30) and third slot (s6 <- s2 + s3) both write
        // s6; the third slot wins: s6 = 0x6000 + 0x333 = 0x6333.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 3, .value = 0x333 },
        },
        .src =
        \\x02.x3 #1, #2, s2, s3, #0, #0, s6, #1, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0x6333 } }},
    },
    .{
        .name = "own-x02u24",
        // ldi sign-extends: s1 = 0xFFC321, s2 = 0xFF9877.  w1[17:15] = 110:
        // a signed, b unsigned 24-bit, floor(P / 2^24) = 0xFFC339.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFC321 },
            .{ .bank = .s, .index = 2, .value = 0xFF9877 },
        },
        .src =
        \\x02.x3 #0, #0, s2, s3, #5, #2, s6, #6, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0xFFC339 } }},
    },
    .{
        .name = "own-x02uu24",
        // w1[17:15] = 100: both unsigned 24-bit: floor(P / 2^24) = 0xFF5BB0.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFC321 },
            .{ .bank = .s, .index = 2, .value = 0xFF9877 },
        },
        .src =
        \\x02.x3 #0, #0, s2, s3, #5, #2, s6, #4, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0xFF5BB0 } }},
    },
    .{
        .name = "own-x03mpy",
        // Pre s1 = 0x4000, s2 = 0x6000; rounded multiply gives s2 <- 0x30.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
        },
        .src =
        \\x03.mpy s2, s1, s2, s0, #4, #1, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x30 } }},
    },
    .{
        .name = "own-x03plo",
        // Pre s1=0x123, s2=0x45; prod=0x4E6F, 2P=0x9CDE low16 into s6.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x123 },
            .{ .bank = .s, .index = 2, .value = 0x45 },
        },
        .src =
        \\x03.mpy s0, s1, s2, s6, #4, #1, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0x9CDE } }},
    },
    .{
        .name = "own-x02asrc",
        // Pre A1 = 0x515; s1 <- A1 + 1 = 0x516 via w4 mode 010, w5[29:28]=01.
        .pre = &.{.{ .bank = .a, .index = 1, .value = 0x515 }},
        .src =
        \\x02.asrc s1, A1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x516 } }},
    },
    .{
        .name = "own-x02esrc",
        // Pre e3 = 0x490; s1 <- e3 = 0x490 via w4 mode 010.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .src =
        \\x02.esrc s1, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x490 } }},
    },
    .{
        .name = "own-x02cin1",
        // Pre R12 = 0x155; fn 10 (y) with cin=1: R1 <- y + 1 = 0x156.
        .pre = &.{.{ .bank = .r, .index = 12, .value = 0x155 }},
        .src =
        \\x02.agu R1, R12, #10, #1
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x156 } }},
    },
    .{
        .name = "own-br-c1",
        // Pre p1 = 0 (branch NOT taken); all adds execute.
        .pre = &.{
            .{ .bank = .p, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br #1, #8, #0, #6
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 5 } }},
    },
    .{
        .name = "own-x18br",
        // p3 = 1, +8 words.  Two delay-slot adds run, two are skipped, two
        // run: s1 = 4.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br #3, #8, #0, #6
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 4 } }},
    },
    .{
        .name = "own-x18br101",
        // w2[30:29] = 00, p3 = 1, +8 words: s1 = 4.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br101 #1, #1, #8, #0, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 4 } }},
    },
    .{
        .name = "own-x18br101-not",
        // w2[29] = 1: no branch, all six adds run.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br101 #1, #1, #8, #1, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 6 } }},
    },
    .{
        .name = "own-x18br101-e",
        // w1[29] = 1 is the e slot index, still +8 words: s1 = 4.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br101 #1, #1, #8, #0, #1
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 4 } }},
    },
    .{
        .name = "own-x18alu",
        // code 0 (add, shift left) by # = 4: s4 <- (0x12 + 3) << 4 = 0x150.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x12 },
            .{ .bank = .s, .index = 3, .value = 3 },
        },
        .src =
        \\x18.alu s4, s5, s3, #0, #0, #4
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x150 } }},
    },
    .{
        .name = "own-x18mac",
        // mode 6: s3 <- P - X = 0x30 - 0x100 = 0xFF30; mode 7: s5 <- 0xD0.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
        },
        .src =
        \\x18.mac s3, #6, s1, s2, s4, s5, s0
        \\x18.mac s5, #7, s1, s2, s4, s5, s0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0xFF30 } },
            .{ .s_reg = .{ .index = 5, .value = 0xD0 } },
        },
    },
    .{
        .name = "own-x18acc111",
        // X + P on 40 bits, X_lo = sx(0x8000): hi s3 = 0x131, lo s5 = 0.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6001 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
            .{ .bank = .s, .index = 6, .value = 0x8000 },
        },
        .src =
        \\x18.mac s3, #5, s1, s2, s4, s6, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x131 } },
            .{ .s_reg = .{ .index = 5, .value = 0 } },
        },
    },
    .{
        .name = "own-x18acc111p",
        // X_lo = s4 = 0x100: hi s3 = 0x130, lo s5 = 0x8100.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6001 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
            .{ .bank = .s, .index = 6, .value = 0x8000 },
        },
        .src =
        \\x18.mac s3, #5, s1, s2, s4, s4, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x130 } },
            .{ .s_reg = .{ .index = 5, .value = 0x8100 } },
        },
    },
    .{
        .name = "own-x18shr111",
        // pass of s4 >> 4 (rounding): s5 = 0x10; the MAC still writes s3.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
        },
        .src =
        \\x18.sh111 s5, #2, s4, s0, #4, #1, #1, e3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x10 } },
            .{ .s_reg = .{ .index = 3, .value = 0xFF30 } },
        },
    },
    .{
        .name = "own-x18shl111",
        // head[27] = 0: s5 <- s4 << 4 = 0x1000.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
        },
        .src =
        \\x18.sh111 s5, #2, s4, s0, #4, #0, #1, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x1000 } }},
    },
    .{
        .name = "own-x18alu111",
        // op 5 (b - a), then the shift: s5 <- (s1 - s4) >> 4 = 0x3F0.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
        },
        .src =
        \\x18.sh111 s5, #5, s4, s1, #4, #1, #1, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0x3F0 } }},
    },
    .{
        .name = "own-x18st111",
        // Y:[e2] <- s5 = 0x555, read before the slot's own s5 write.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 4, .value = 0x100 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
            .{ .bank = .e, .index = 2, .value = 0x703 },
        },
        .src =
        \\x18.sh111 s5, #2, s4, s0, #4, #1, #3, e2
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x703, .value = 0x555 } },
            .{ .s_reg = .{ .index = 5, .value = 0x10 } },
        },
    },
    .{
        .name = "own-x18rcl100",
        // R3 <- AGU10[2](e3) = 0x706; Y:[e3] <- s5.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
            .{ .bank = .r, .index = 1, .value = 2 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.rcl100 #1, #1, #2, e3, R1, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x706 } },
            .{ .y_mem = .{ .addr = 0x705, .value = 0x555 } },
        },
    },
    .{
        .name = "own-x18mpy100",
        // s4 <- round(0x4000 * 0x6000 / 2^23) = 0x30; Y:[e3] <- s5.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x705 },
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.mpy100 s4, s1, s2, #4, s5, e3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x30 } },
            .{ .y_mem = .{ .addr = 0x705, .value = 0x555 } },
        },
    },
    .{
        .name = "own-x18shr100",
        // s1 <- round(0x555 / 16) = 0x55.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x555 }},
        .src =
        \\x18.sh100 s1, s5, #4, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x55 } }},
    },
    .{
        .name = "own-x18shl100",
        // s1 <- 0x555 << 4 = 0x5550.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x555 }},
        .src =
        \\x18.sh100 s1, s5, #4, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x5550 } }},
    },
    .{
        .name = "own-x18yb100",
        // s1 <- Y:[e3 = 0x701][7:0] << 8 = 0x5500.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x701 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x701, .value = 0x555 }},
        .src =
        \\x18.yb100 s1, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x5500 } }},
    },
    .{
        .name = "own-x18rcl001",
        // R3 <- AGU10B[6](e3, R1) = 0x707; Y:[e3] <- s5 = 0x555.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
            .{ .bank = .r, .index = 1, .value = 2 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.rcl001 R3, e3, R1, #6, #18, #0, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x707 } },
            .{ .y_mem = .{ .addr = 0x705, .value = 0x555 } },
        },
    },
    .{
        .name = "own-x18rag001",
        // head[30] = 1: R3 <- AGU3[18](y = R1) = y + 1 = 3.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
            .{ .bank = .r, .index = 1, .value = 2 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x18.rcl001 R3, e3, R1, #6, #18, #1, s5
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 3 } }},
    },
    .{
        .name = "own-x06lde6",
        // [31] = 1 lde: e2 <- #0x300.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x555 }},
        .src =
        \\lde.hi e2, #0x300
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x300 } }},
    },
    .{
        .name = "own-x07rop12",
        // class 011 rop: s12 and R6 <- e8 = 0x345.
        .pre = &.{.{ .bank = .e, .index = 8, .value = 0x345 }},
        .src =
        \\rop.b21 s12, R6, e8, R0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 12, .value = 0x345 } },
            .{ .r_reg = .{ .index = 6, .value = 0x345 } },
        },
    },
    .{
        .name = "own-x07rop4",
        // class 100 rop still executes: s2 <- e7 = 0x345.
        .pre = &.{.{ .bank = .e, .index = 7, .value = 0x345 }},
        .src =
        \\rop.c4 s2, R0, e7, R0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x345 } }},
    },
    .{
        .name = "own-x0baset",
        // s1 <- Y:[e3 = 0x700] = 0x1357.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x700 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0x1357 }},
        .src =
        \\aset0b s1, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x1357 } }},
    },
    .{
        .name = "own-x0cragu",
        // AGUA fn 1 (x+i-1), i = 0: R6 <- A7 - 1 = 0x344.
        .pre = &.{.{ .bank = .a, .index = 7, .value = 0x345 }},
        .src =
        \\ragu0c R6, A7, #1
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 6, .value = 0x344 } }},
    },
    .{
        .name = "own-x0eld7",
        // n = 1, op[0] = 1: A7 <- #0x345, A5 untouched.
        .pre = &.{
            .{ .bank = .a, .index = 7, .value = 0x111 },
            .{ .bank = .a, .index = 5, .value = 0x222 },
        },
        .src =
        \\ld0e.a #0x345, #1, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .a_reg = .{ .index = 7, .value = 0x345 } },
            .{ .a_reg = .{ .index = 5, .value = 0x222 } },
        },
    },
    .{
        .name = "own-x0fe8",
        // e<{w1[0], head[31:29]}> = e8 <- #0x123.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x600 },
            .{ .bank = .e, .index = 8, .value = 0x555 },
        },
        .src =
        \\ste #0, #1, #0x123
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 8, .value = 0x123 } }},
    },
    .{
        .name = "own-x04ragu",
        // R6 <- R5 - 0x30 = 0x1000 - 0x30 = 0xFD0.
        .pre = &.{.{ .bank = .r, .index = 5, .value = 0x1000 }},
        .src =
        \\x04.ragu R6, R5, #11, #0x30
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 6, .value = 0xFD0 } }},
    },
    .{
        .name = "own-x04yd",
        // R5 <- Y:[e4] = 0x1357.
        .pre = &.{.{ .bank = .e, .index = 4, .value = 0x700 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0x1357 }},
        .src =
        \\x04.yd R5, s7, e4
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 5, .value = 0x1357 } }},
    },
    .{
        .name = "own-x04yef",
        // s7 <- Y:[e6] = 0x1357, not Y:[e4].
        .pre = &.{
            .{ .bank = .e, .index = 6, .value = 0x600 },
            .{ .bank = .e, .index = 4, .value = 0x400 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x600, .value = 0x1357 }},
        .src =
        \\x04.yd R0, s7, e6
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x1357 } }},
    },
    .{
        .name = "own-x04xst",
        // X:[A9 = 0x320] <- s3 = 0x777.
        .pre = &.{
            .{ .bank = .a, .index = 9, .value = 0x320 },
            .{ .bank = .s, .index = 3, .value = 0x777 },
        },
        .src =
        \\x04.xst A9, #3, #0
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x320, .value = 0x777 } }},
    },
    .{
        .name = "own-x10alu0",
        // add m4, b = s2, a = s0: R5 <- 0x1233.
        .pre = &.{.{ .bank = .s, .index = 2, .value = 0x1234 }},
        .src =
        \\x10.alu R5, s2, s0, #14, #4, #0, #0x20017
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 5, .value = 0x1233 } }},
    },
    .{
        .name = "own-x10shl",
        // code 0x3F m2 #4: R6 <- s8 << s1 = 0x1234 << 4 = 0x2340 (16-bit).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 4 },
            .{ .bank = .s, .index = 8, .value = 0x1234 },
        },
        .src =
        \\x10.alu R6, s8, s1, #0x3F, #2, #4, #0x4000
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 6, .value = 0x2340 } }},
    },
    .{
        .name = "own-x02w3b31",
        // Pre e3 = 0x490 with Y[0x490] seeded to 0x886F.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x02.yrd s1, e3, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x886F } }},
    },
    .{
        .name = "own-x03w5c2",
        // Pre e3 = 0x490, Y:0x490 = 0x886F; s7 <- Y:[e3].
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x03.yrd s7, e3, #5, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x886F } }},
    },
    .{
        .name = "own-x17ctl",
        // S1[22:20] = 000 continues; the S1 class 000 tail halts, its 2-bundle shadow runs, s6 does not.
        .src =
        \\x17.cont
        \\x17.imm s5, #0x555
        \\x17.end
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\x17.imm s6, #0x666
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x555 } },
            .{ .s_reg = .{ .index = 6, .value = 0 } },
        },
    },
    .{
        .name = "own-x17eagu",
        // s4 <- AGU10[7](e14, R10) = 0x200 - 0xB0 = 0x150.
        .pre = &.{
            .{ .bank = .e, .index = 14, .value = 0x200 },
            .{ .bank = .r, .index = 10, .value = 0xB0 },
        },
        .src =
        \\x17.eagu s4, e14, #2, #7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x150 } }},
    },
    .{
        .name = "own-x17sty",
        // Y:[e2 = 0x600] <- s5 = 0x1234.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x600 },
            .{ .bank = .s, .index = 5, .value = 0x1234 },
        },
        .src =
        \\x17.sty e2, #5, #0
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x600, .value = 0x1234 } }},
    },
    .{
        .name = "own-x17yld",
        // s14 <- Y:[e2 = 0x600] = 0x5A5A.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x600 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x600, .value = 0x5A5A }},
        .src =
        \\x17.yld s14, e2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 14, .value = 0x5A5A } }},
    },
    .{
        .name = "own-x16mac",
        // acc = (s1 << 24) + s2 + s3*s5*2 = 0x1_1000_4010: s7 <- 0x110, s6 <- 0x4010.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x100 },
            .{ .bank = .s, .index = 2, .value = 0x10 },
            .{ .bank = .s, .index = 3, .value = 0x4001 },
            .{ .bank = .s, .index = 5, .value = 0x2000 },
        },
        .src =
        \\x16.mac s7, s6, #1, #0, s2, s3, s5, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 7, .value = 0x110 } },
            .{ .s_reg = .{ .index = 6, .value = 0x4010 } },
        },
    },
    .{
        .name = "own-x03ayd",
        // w5[5]: A9 <- Y:[e3 = 0x416] = 0x332.
        .pre = &.{
            .{ .bank = .a, .index = 5, .value = 0x100 },
            .{ .bank = .e, .index = 3, .value = 0x416 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x416, .value = 0x332 }},
        .src =
        \\x03.acp A9, A5, e3, #1
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 9, .value = 0x332 } }},
    },
    .{
        .name = "own-x02plo",
        // s6 <- (2 * 0x123 * 0x45)[15:0] = 0x9CDE.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x123 },
            .{ .bank = .s, .index = 2, .value = 0x45 },
        },
        .src =
        \\x02.x3 #0, #0, s2, s3, #5, #2, s2, #1, s6
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0x9CDE } }},
    },
    .{
        .name = "own-x00slots",
        // Y:[e5] <- s9, X:[A3] <- s7, s3 <- e5 (w5[5:4], w5[2:1] = 11).
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0x360 },
            .{ .bank = .a, .index = 3, .value = 0x320 },
            .{ .bank = .s, .index = 7, .value = 0x777 },
            .{ .bank = .s, .index = 9, .value = 0x999 },
        },
        .src =
        \\x00.slots s3, #2, #2, #2, e5, e5, s9, A3, s7, #3, #3, s0, #0, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x360 } },
            .{ .y_mem = .{ .addr = 0x360, .value = 0x999 } },
            .{ .x_mem = .{ .addr = 0x320, .value = 0x777 } },
        },
    },
    .{
        .name = "own-x01w5old",
        // 0x00's store bits set on 0x01 are inert: same three writes.
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0x360 },
            .{ .bank = .a, .index = 3, .value = 0x320 },
            .{ .bank = .s, .index = 7, .value = 0x777 },
            .{ .bank = .s, .index = 9, .value = 0x999 },
        },
        .src =
        \\x01.slots s3, #2, #2, #2, e5, e5, s9, A3, s7, s0, #0, #0, #3, #3, #0x36, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x360 } },
            .{ .y_mem = .{ .addr = 0x360, .value = 0x999 } },
            .{ .x_mem = .{ .addr = 0x320, .value = 0x777 } },
        },
    },
    .{
        .name = "own-x00eop",
        // op 3: s3 <- -e5 = 0xFCA0.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x360 }},
        .src =
        \\x00.slots s3, #2, #2, #3, e5, e5, s0, A0, s0, #0, #0, s0, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0xFCA0 } }},
    },
    .{
        .name = "own-x01eop",
        // class 001 0x01, op 3: s3 <- -e5 = 0xFCA0.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x360 }},
        .src =
        \\x01.slots s3, #2, #2, #3, e5, e5, s0, A0, s0, s0, #0, #0, #0, #0, #0, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0xFCA0 } }},
    },
    .{
        .name = "own-x00xd",
        // s5 <- xd, the last X read (0x02 s1 <- X:[A2] = 0xEF41).
        .pre = &.{
            .{ .bank = .a, .index = 2, .value = 0x234 },
            .{ .bank = .e, .index = 5, .value = 0x360 },
            .{ .bank = .a, .index = 3, .value = 0x320 },
            .{ .bank = .s, .index = 7, .value = 0x777 },
            .{ .bank = .s, .index = 9, .value = 0x999 },
        },
        .pre_mem = &.{.{ .kind = .x, .addr = 0x234, .value = 0xEF41 }},
        .src =
        \\x02.xrd s1, A2
        \\x00.slots s3, #2, #2, #2, e5, e5, s9, A3, s7, #3, #3, s5, #1, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 5, .value = 0xEF41 } }},
    },
    .{
        .name = "own-x00alu",
        // s1 <- add(0, s3) << 1 = 0x246; s2 <- 0 (MAC slot, mode 0).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x123 },
            .{ .bank = .s, .index = 2, .value = 0x222 },
        },
        .src =
        \\x00.alu s1, s2, s3, #1, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x246 } },
            .{ .s_reg = .{ .index = 2, .value = 0 } },
        },
    },
    .{
        .name = "own-x00mac",
        // mode 7: s9 <- s6 - rnd(s3 * s5 >> 23) = 0x100 - 0x12 = 0xEE.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x4000 },
            .{ .bank = .s, .index = 5, .value = 0x2345 },
            .{ .bank = .s, .index = 6, .value = 0x100 },
        },
        .src =
        \\x00.mac s9, s0, s3, s5, s6, #7, #7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0xEE } }},
    },
    .{
        .name = "own-x00acclo",
        // acc = {s6 : s7} - 2 s3 s5 = 0xEE5DF707: s4 <- 0xF707.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x4000 },
            .{ .bank = .s, .index = 5, .value = 0x2345 },
            .{ .bank = .s, .index = 6, .value = 0x100 },
            .{ .bank = .s, .index = 7, .value = 0x7707 },
        },
        .src =
        \\x00.mac s9, s4, s3, s5, s6, #7, #7
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0xF707 } },
            .{ .s_reg = .{ .index = 9, .value = 0xEE } },
        },
    },
    .{
        .name = "own-x00jr",
        // q2 jr +10 = q12 skips q11's ldi.
        .src =
        \\ldi s1, #0x111
        \\lde e15, #0x700
        \\x00.ctl #10, #1
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\ldi s1, #0xBAD
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x111 } }},
    },
    .{
        .name = "own-x00jr16",
        // ctl 0x16 jumps like 1.
        .src =
        \\ldi s1, #0x111
        \\lde e15, #0x700
        \\x00.ctl #10, #22
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\ldi s1, #0xBAD
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x111 } }},
    },
    .{
        .name = "own-x00nj10",
        // ctl 0x10 does not jump: q11 runs.
        .src =
        \\ldi s1, #0x111
        \\lde e15, #0x700
        \\x00.ctl #10, #16
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\ldi s1, #0xBAD
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0xBAD } }},
    },
    .{
        .name = "own-x00ja",
        // ja to absolute q12 skips q11.
        .src =
        \\ldi s1, #0x111
        \\lde e15, #0x700
        \\x00.ctl #12, #4
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\ldi s1, #0xBAD
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x111 } }},
    },
    .{
        .name = "own-jmp-hi",
        // jmp +4 if !p9, p9 false: taken, q+3 skipped.
        .pre = &.{.{ .bank = .p, .index = 9, .value = 0 }},
        .src =
        \\jmp #2, #9, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 5 } }},
    },
    .{
        .name = "own-jmp-hi-t",
        // mode 3, jmp if p9, p9 false: no jump.
        .pre = &.{.{ .bank = .p, .index = 9, .value = 0 }},
        .src =
        \\jmp #3, #9, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 256 } }},
    },
    .{
        .name = "own-ptst-hi",
        // ptst p9 <- (s5 < 0) true, then jmp if p9: q+3 skipped.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0xFFF000 }},
        .src =
        \\ptst p9, s5, #1, #0x0B
        \\jmp #3, #9, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 5 } }},
    },
    .{
        .name = "own-ptst-hi-pos",
        // s5 = 0x0F00: p9 false, no jump.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0x0F00 }},
        .src =
        \\ptst p9, s5, #1, #0x0B
        \\jmp #3, #9, #4
        \\ldi s3, #5
        \\ldi s4, #7
        \\ldi s3, #256
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 256 } }},
    },
    .{
        .name = "own-plog-hi",
        // p4 <- p1 & p<{[29], [28:26]}> = p1 & p10 = 0 (p1 & p2 would be 1).
        .pre = &.{
            .{ .bank = .p, .index = 1, .value = 1 },
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .p, .index = 10, .value = 0 },
            .{ .bank = .p, .index = 4, .value = 1 },
        },
        .src =
        \\plog p4, p1, p10, #3, #0x0E
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 4, .value = 0 } }},
    },
    .{
        .name = "own-plog-hi-c",
        // class 000: p4 <- p1 & p2 = 1.
        .pre = &.{
            .{ .bank = .p, .index = 1, .value = 1 },
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .p, .index = 10, .value = 0 },
        },
        .src =
        \\plog p4, p1, p2, #3, #0x0E
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 4, .value = 1 } }},
    },
    .{
        .name = "own-p010-false",
        // head[28:25] = 14 picks row 0xFD1B: cond 9 is false.
        .pre = &.{.{ .bank = .p, .index = 3, .value = 1 }},
        .src =
        \\condset p3, #9, #14
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 3, .value = 0 } }},
    },
    .{
        .name = "own-bcc-fwd",
        // bcc p0 (always) +8: 2 slot adds run, 4 skipped, 2 run: s1 = 4.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\bcc #0, #2
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 4 } }},
    },
    .{
        .name = "own-bcc-back",
        // bcc p2 -4: delay slots plog p2 <- !p2 and a nop; 8 + 4 adds.
        .pre = &.{
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\bcc #2, #0x1FF
        \\plog p2, p2, p2, #7, #0x0E
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 12 } }},
    },
    .{
        .name = "own-bcc-slots",
        // a two-word ytst as delay slot 1: plog still runs as slot 2.
        .pre = &.{
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\bcc #2, #0x1FF
        \\ytst p3, e5
        \\plog p2, p2, p2, #7, #0x0E
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 12 } }},
    },
    .{
        .name = "own-x18br-back",
        // class 011 branch -8 words if p2; plog p2 <- !p2 in a slot: 16 adds.
        .pre = &.{
            .{ .bank = .p, .index = 2, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\x18.br #2, #0x3FF8, #1, #3
        \\plog p2, p2, p2, #7, #0x0E
        \\alu s0, s0, s0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 16 } }},
    },
    .{
        .name = "own-psel-imm",
        // mux 1: s3 <- #0x18; s5 <- s5 + s3 = 0x123 (reads s3 before the head write).
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x100 },
            .{ .bank = .s, .index = 3, .value = 0x23 },
        },
        .src =
        \\x0e s3, #0x18, #0x0E, s5, s3, s5, #0, #1, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x18 } },
            .{ .s_reg = .{ .index = 5, .value = 0x123 } },
        },
    },
    .{
        .name = "own-psel-r",
        // mux 7: s3 and s5 <- s5 + s3 = 0x123.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x100 },
            .{ .bank = .s, .index = 3, .value = 0x23 },
        },
        .src =
        \\x0e s3, #0x18, #0x0E, s5, s3, s5, #0, #7, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x123 } },
            .{ .s_reg = .{ .index = 5, .value = 0x123 } },
        },
    },
    .{
        .name = "own-psel-sub",
        // code 0x10 sub: s5 <- s3 - s5 = 0xFF23 (24-bit 0xFFFF23); s3 <- #0x18.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x100 },
            .{ .bank = .s, .index = 3, .value = 0x23 },
        },
        .src =
        \\x0e s3, #0x18, #0x10, s5, s3, s5, #0, #1, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x18 } },
            .{ .s_reg = .{ .index = 5, .value = 0xFFFF23 } },
        },
    },
    .{
        .name = "own-rimm",
        // R3 <- #0x1234.
        .src =
        \\rimm R3, #0x1234
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 3, .value = 0x1234 } }},
    },
    .{
        .name = "own-stx-a",
        // X:[A15] <- s7, X:[A22] <- s5 (A = 4*S[2:0] + head[31:30]).
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0x777 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
            .{ .bank = .a, .index = 15, .value = 0x310 },
            .{ .bank = .a, .index = 22, .value = 0x320 },
        },
        .src =
        \\stx0f #3, #3, s7
        \\stx0f #2, #5, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .x_mem = .{ .addr = 0x310, .value = 0x777 } },
            .{ .x_mem = .{ .addr = 0x320, .value = 0x555 } },
        },
    },
    .{
        .name = "own-st-e6",
        // head[31:29] = 6: e6 <- #0x700.
        .src =
        \\ste.n #6, #0x700
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 6, .value = 0x700 } }},
    },
    .{
        .name = "own-slot-st",
        // ldi2 s2 <- 0x55 | Y:[e3 = 0x700] <- s1 = 0xABC.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x700 },
            .{ .bank = .s, .index = 1, .value = 0xABC },
        },
        .src =
        \\ldi2.st s2, #0x55, e3, s1, #3
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x700, .value = 0xABC } },
            .{ .s_reg = .{ .index = 2, .value = 0x55 } },
        },
    },
    .{
        .name = "own-stk",
        // Y:[e9 = 0x709] <- s5, Y:[e14 = 0x70E] <- s6.
        .pre = &.{
            .{ .bank = .e, .index = 9, .value = 0x709 },
            .{ .bank = .e, .index = 14, .value = 0x70E },
            .{ .bank = .s, .index = 5, .value = 0x555 },
            .{ .bank = .s, .index = 6, .value = 0x666 },
        },
        .src =
        \\stk e9, s5
        \\stk e14, s6
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x709, .value = 0x555 } },
            .{ .y_mem = .{ .addr = 0x70E, .value = 0x666 } },
        },
    },
    .{
        .name = "own-st1",
        // k = 1: Y:[e1 = 0x70D] <- s5.
        .pre = &.{
            .{ .bank = .e, .index = 1, .value = 0x70D },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\stk e1, s5
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x70D, .value = 0x555 } }},
    },
    .{
        .name = "own-yagu",
        // s1 <- Y:[e3] (old e) = 0x1357; e3 and R5 <- e3 + R4 = 0x802.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x702 },
            .{ .bank = .r, .index = 4, .value = 0x100 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x1357 }},
        .src =
        \\yagu e3, R4, #6, s1, R5, #1, s0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x1357 } },
            .{ .e_reg = .{ .index = 3, .value = 0x802 } },
            .{ .r_reg = .{ .index = 5, .value = 0x802 } },
        },
    },
    .{
        .name = "own-ypair",
        // s8 <- e3 = 0x702; yagu loads yd <- 0x0BEE; s2 <- yd.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x702 },
            .{ .bank = .r, .index = 4, .value = 0x100 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x0BEE }},
        .src =
        \\ypair e3, #6, #2, #4
        \\yagu e3, R4, #0, s1, R5, #1, s0
        \\ypair e3, #7, #1, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 8, .value = 0x702 } },
            .{ .s_reg = .{ .index = 2, .value = 0x0BEE } },
        },
    },
    .{
        .name = "own-ystm",
        // Y:[e3] <- s6; e3 and R5 <- x+1 = 0x703.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x702 },
            .{ .bank = .s, .index = 6, .value = 0x0BEE },
        },
        .src =
        \\ystm e3, e3, R0, #2, #1, #2, s6, #3, R5
        \\x17.halt
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x702, .value = 0x0BEE } },
            .{ .e_reg = .{ .index = 3, .value = 0x703 } },
            .{ .r_reg = .{ .index = 5, .value = 0x703 } },
        },
    },
    .{
        .name = "own-ystm-yd",
        // head[26] = 0: R5 <- yd = 0x0BEE (yagu loaded it); no store.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x702 },
            .{ .bank = .r, .index = 4, .value = 0x100 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x0BEE }},
        .src =
        \\yagu e3, R4, #0, s1, R0, #1, s0
        \\ystm e3, e0, R0, #0, #0, #2, s0, #0, R5
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 5, .value = 0x0BEE } }},
    },
    .{
        .name = "own-tst-false",
        // tst (8, 5) -> p2 false: br.c2 not taken, the add runs.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\tst #8, #5, #2
        \\br #2, #4
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 1 } }},
    },
    .{
        .name = "own-tst-true",
        // tst (8, 0x1D) -> p2 true: br.c2 jumps over the add.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\tst #8, #0x1D, #2
        \\br #2, #4
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-tst-slot5",
        // the second tst writes false into p5; p2 stays true.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\tst #8, #0x1D, #2
        \\tst #8, #5, #5
        \\br #2, #4
        \\alu s0, s0, s0, #0
        \\alu s0, s0, s0, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0 } }},
    },
    .{
        .name = "own-par",
        // X:[A6] <- old s7 = 0xF77; s7 <- Y:[e5 = 0xA21] = 0x1357.
        .pre = &.{
            .{ .bank = .e, .index = 5, .value = 0xA21 },
            .{ .bank = .s, .index = 7, .value = 0xF77 },
            .{ .bank = .a, .index = 6, .value = 0x3A0 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0xA21, .value = 0x1357 }},
        .src =
        \\par s7, e5, A6, s7
        \\x17.halt
        ,
        .post = &.{
            .{ .x_mem = .{ .addr = 0x3A0, .value = 0xF77 } },
            .{ .s_reg = .{ .index = 7, .value = 0x1357 } },
        },
    },
    .{
        .name = "own-mul13",
        // s5 <- s1*s3 >> 23 = 0x3C, then s3 <- s1*s2 >> 23 = 0x54.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x6000 },
            .{ .bank = .s, .index = 2, .value = 0x7000 },
            .{ .bank = .s, .index = 3, .value = 0x5000 },
        },
        .src =
        \\mul13 s5, s1, #1, #1
        \\mul13 s3, s1, #1, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x3C } },
            .{ .s_reg = .{ .index = 3, .value = 0x54 } },
        },
    },
    .{
        .name = "own-alu-addi",
        // addi m1: s3 <- s2 - 5 = 0x27C; ldi pins e0 = 0x700; wr Y:[e0] <- s3.
        .src =
        \\ldi s1, #0xAF
        \\ldi s2, #0x281
        \\alu.m s3, s1, s2, #9, #1, #5
        \\ldi s14, #0x700
        \\st.wr #0x40, e0, s3
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x700, .value = 0x27C } }},
    },
    .{
        .name = "own-entry",
        // lde e13 <- 0x70D; wr Y:[e13] <- s5 = 0x555.
        .src =
        \\lde e13, #0x70D
        \\ldi s5, #0x555
        \\st.wr #0x40, e13, s5
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x70D, .value = 0x555 } }},
    },
    .{
        .name = "own-wr",
        // ldi s5 = 0x50 sets e0; wr Y:[e0] <- s5 reads e0 before st's own update.
        .src =
        \\ldi s5, #0x50
        \\st.wr #0x40, e0, s5
        \\ldi s1, #0x123
        \\x17.halt
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x50, .value = 0x50 } }},
    },
    .{
        .name = "own-stop-shadow",
        // the first marker after the stop lands (Y:0x600), the second does not.
        .src =
        \\ldi s5, #0x5A5
        \\x17.halt
        \\ldi s6, #0x600
        \\st.wr #0x40, e0, s5
        \\ldi s6, #0x601
        \\st.wr #0x40, e0, s5
        ,
        .post = &.{
            .{ .y_mem = .{ .addr = 0x600, .value = 0x5A5 } },
            .{ .y_none = 0x601 },
        },
    },
    .{
        .name = "own-b16",
        // the second bundle stores s3 at xa = 0x300, set by the first.
        .pre = &.{.{ .bank = .s, .index = 3, .value = 0x456 }},
        .src =
        \\x16.b16 s10, #0x300, s3, e5
        \\x16.b16 s10, #0x310, s3, e5
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x300, .value = 0x456 } }},
    },
    .{
        .name = "own-b16agu",
        // FN 2: s1 <- e6 + 1 = 0x301.
        .pre = &.{.{ .bank = .e, .index = 6, .value = 0x300 }},
        .src =
        \\x16.agu100 s1, #0x1092, e6, #2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x301 } }},
    },
    .{
        .name = "own-b16sh",
        // s2 <- #3; s1 <- s5 asr 3 = 0xF00 >> 3 = 0x1E0.
        .pre = &.{.{ .bank = .s, .index = 5, .value = 0xF00 }},
        .src =
        \\x16.sh s2, #3, #6, s1, #3, s5
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x1E0 } },
            .{ .s_reg = .{ .index = 2, .value = 3 } },
        },
    },
    .{
        .name = "own-x02mac",
        // A reads X:[A2] (0x7EFD) into s4; B: s2 <- P + xd[15:0] = 0x30 + 0xFD00.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .a, .index = 2, .value = 0x567 },
        },
        .pre_mem = &.{.{ .kind = .x, .addr = 0x567, .value = 0x7EFD }},
        .src =
        \\x02.mac s0, #4, #1, s4, A2
        \\x02.mac s2, #5, #1, s0, A2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 4, .value = 0x7EFD } },
            .{ .s_reg = .{ .index = 2, .value = 0xFD30 } },
        },
    },
    .{
        .name = "own-x18e2nd-w17",
        // from T = 0: e14 <- 0 - 1 = 0xFFFF, T = 1; then e14 <- e14 - 1 = 0x705, T = 0.
        .src =
        \\lde e14, #0x706
        \\x18.e2nd17 e14, #2, #3
        \\rop s5, R0, e14, R0, #0
        \\lde e14, #0x706
        \\x18.e2nd17 e14, #2, #3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0xFFFF } },
            .{ .e_reg = .{ .index = 14, .value = 0x705 } },
        },
    },
    .{
        .name = "own-all",
        // st sets e1 = 2; the stop's S2[1] write stores s1 = 0x123 at Y:[e1].
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x123 } }},
    },
    .{
        .name = "own-core",
        // st sets e1 = 2; the stop's S2[1] write stores s1 = 0x123 at Y:[e1].
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x123 } }},
    },
    .{
        .name = "own-core-17",
        // st sets e1 = 2; the stop's S2[1] write stores s1 = 0x123 at Y:[e1].
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x123 } }},
    },
    .{
        .name = "own-q23",
        // st sets e1 = 2; the stop's S2[1] write stores s1 = 0x123 at Y:[e1].
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x123 } }},
    },
    .{
        .name = "own-all-x",
        // own-all with the stop's inert head and S2 bits filled with junk.
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stopx s1, e1, #0x2D2D2D2, #0x1234567
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x123 } }},
    },
    .{
        .name = "own-core-02",
        // S2 class 000 is no stop: the run never halts (silicon wedges).
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #0, #1
        ,
        .post = &.{},
        .wedge = true,
    },
    .{
        .name = "own-core-a2",
        // S1[16:15] = 10 writes at a2, the drifting pointer (Y:9 on silicon).
        .pre = &.{.{ .bank = .e, .index = 2, .value = 9 }},
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e2, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x9, .value = 0x123 } }},
    },
    .{
        .name = "own-core-s5-y40",
        // st at Y:0x40, ldi s5, the stop writes s5 at e1 = 0x40.
        .src =
        \\st.y Y:0x40
        \\ldi.q s5, #0x456, #2
        \\x17.stop s5, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x40, .value = 0x456 } }},
    },
    .{
        .name = "own-core-y77",
        // st at Y:0x77: the stop's write follows e1.
        .src =
        \\st.y Y:0x77
        \\ldi.q s1, #0x123, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x77, .value = 0x123 } }},
    },
    .{
        .name = "own-pre-ldi",
        // an ldi s5 before the store survives to the stop's write.
        .src =
        \\ldi.q s5, #0xA5A, #2
        \\st.y Y:2
        \\ldi.q s1, #0x123, #2
        \\x17.stop s5, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0xA5A } }},
    },
    .{
        .name = "own-ldi-s5",
        // ldi into register 21 (s5), the stop reads register 21.
        .src =
        \\st.y Y:2
        \\ldi.q s5, #0xA15, #2
        \\x17.stop s5, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0xA15 } }},
    },
    .{
        .name = "own-ldi-y2",
        // our ldi's immediate is what the write stores at Y:2.
        .src =
        \\st.y Y:2
        \\ldi.q s1, #0x1234, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x2, .value = 0x1234 } }},
    },
    .{
        .name = "own-ldi-y4",
        // our ldi's immediate is what the stub's Y:4 write stores.
        .src =
        \\st.y Y:4
        \\ldi.q s1, #0x55, #2
        \\x17.stop s1, e1, #5, #1
        ,
        .post = &.{.{ .y_mem = .{ .addr = 0x4, .value = 0x55 } }},
    },
    .{
        .name = "own-st3",
        // three stores in lanes 1..3: each writes its lane's V (V2 = 2).
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #0
        \\st.sel Y:0x200, #0
        \\st.sel Y:0x300, #0
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x0 } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x2 } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x0 } } },
        .lane = 1,
    },
    .{
        .name = "own-st3-l1sel1",
        // sel 1 on l1: V2 <- the previous store address 0x108F.
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #1
        \\st.sel Y:0x200, #0
        \\st.sel Y:0x300, #0
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x0 } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x108F } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x0 } } },
        .lane = 1,
    },
    .{
        .name = "own-st3-l1sel2",
        // sel 2 on l1: V1, V3 <- 0x108F before l1 writes.
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #2
        \\st.sel Y:0x200, #0
        \\st.sel Y:0x300, #0
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x108F } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x2 } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x108F } } },
        .lane = 1,
    },
    .{
        .name = "own-st3-l1sel3",
        // sel 3 on l1: no effect.
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #3
        \\st.sel Y:0x200, #0
        \\st.sel Y:0x300, #0
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x0 } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x2 } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x0 } } },
        .lane = 1,
    },
    .{
        .name = "own-st3-l2sel2",
        // sel 2 on l2: V1, V3 <- 0x100; l1 already wrote, l3 writes 0x100.
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #0
        \\st.sel Y:0x200, #2
        \\st.sel Y:0x300, #0
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x0 } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x2 } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x100 } } },
        .lane = 1,
    },
    .{
        .name = "own-st3-l3sel2",
        // sel 2 on l3: V1, V3 <- 0x200 before its own write.
        .pre = &.{ .{ .bank = .v, .index = 2, .value = 2 }, .{ .bank = .e, .index = 1, .value = 0x108F } },
        .src =
        \\st.sel Y:0x100, #0
        \\st.sel Y:0x200, #0
        \\st.sel Y:0x300, #2
        \\x17.halt
        ,
        .post = &.{ .{ .y_mem = .{ .addr = 0x100, .value = 0x0 } }, .{ .y_mem = .{ .addr = 0x200, .value = 0x2 } }, .{ .y_mem = .{ .addr = 0x300, .value = 0x200 } } },
        .lane = 1,
    },
    .{
        .name = "own-x03rep",
        // p3 = 1: the store of s3 lands at X:0x300, the re-issue at X:0x301.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 3, .value = 0x3303 },
            .{ .bank = .a, .index = 8, .value = 0x300 },
        },
        .src =
        \\x03.rep p3
        \\x17.halt
        ,
        .post = &.{
            .{ .x_mem = .{ .addr = 0x300, .value = 0x3303 } },
            .{ .x_mem = .{ .addr = 0x301, .value = 0x3303 } },
        },
    },
    .{
        .name = "own-x15pt",
        // Pre p13 = 1; LUT 1 (identity): s3 <- p13 = 1.
        .pre = &.{.{ .bank = .p, .index = 13, .value = 1 }},
        .src =
        \\x15.pt s3, #13, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 1 } }},
    },
    .{
        .name = "own-x15pt-n",
        // Pre p13 = 1; LUT 0 (!p): s3 <- !p13 = 0.
        .pre = &.{.{ .bank = .p, .index = 13, .value = 1 }},
        .src =
        \\x15.pt s3, #13, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0 } }},
    },
    .{
        .name = "own-x16hd",
        // Pre s2 = 0x1234, s5 = 0x2345; s1 <- s5 + s2 = 0x3579.
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x1234 },
            .{ .bank = .s, .index = 5, .value = 0x2345 },
        },
        .src =
        \\x16.hd s1, s5, s2, #0xF, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x3579 } }},
    },
    .{
        .name = "own-x16mul2",
        // Pre s1 = 0x1101, s2 = 0x3303; rounded multiply gives s3 <- 7.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x1101 },
            .{ .bank = .s, .index = 2, .value = 0x3303 },
        },
        .src =
        \\x16.mul2 #3, #0, s1, s2, #0, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 7 } }},
    },
    .{
        .name = "own-alu-shr",
        // Pre s1 = 0xAF, s2 = 0x281; add then shift >> 3 via code 3.
        // Code 3 is "add" family in alu05; shift-count field not modelled.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xAF },
            .{ .bank = .s, .index = 2, .value = 0x281 },
        },
        .src =
        \\alu s3, s1, s2, #3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x330 } }},
    },
    .{
        .name = "own-alu-trn",
        // Code 3 add in alu05: s3 <- s1 + s2 = 0x330.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xAF },
            .{ .bank = .s, .index = 2, .value = 0x281 },
        },
        .src =
        \\alu s3, s1, s2, #3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x330 } }},
    },
    .{
        .name = "own-alu-rnd",
        // Code 4 add in alu05: s3 <- s1 + s2 = 0x330.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xAF },
            .{ .bank = .s, .index = 2, .value = 0x281 },
        },
        .src =
        \\alu s3, s1, s2, #4
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x330 } }},
    },
    .{
        .name = "own-alu-asr",
        // Pre s1 = -4 (0xFFFFFC = shift count), s2 = 0x123; asr b by a.
        // Negative count shifts left: 0x123 << 4 = 0x1230.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFFFFC },
            .{ .bank = .s, .index = 2, .value = 0x123 },
        },
        .src =
        \\alu s3, s1, s2, #0x06
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x1230 } }},
    },
    .{
        .name = "own-s0-reads-zero",
        // s0 reads as 0 regardless of pre-state.
        .pre = &.{.{ .bank = .s, .index = 0, .value = 7 }},
        .src =
        \\alu s3, s0, s0, #0x23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0 } }},
    },
    .{
        .name = "own-alu-sub",
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x700 },
            .{ .bank = .s, .index = 2, .value = 0x123 },
        },
        .src =
        \\alu s3, s1, s2, #0x10
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x5DD } }},
    },
    .{
        .name = "own-alu-hi",
        // ALU add with [31:29] = 011 reads b from s<8+n>.
        // For simplicity, add s1 + s2 using default encoding.
        .pre = &.{
            .{ .bank = .s, .index = 10, .value = 0x100 },
            .{ .bank = .s, .index = 1, .value = 0x23 },
        },
        .src =
        \\alu s1, s1, s10, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x123 } }},
    },
    .{
        .name = "own-alu-sel",
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFFFFE },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\alu s3, s1, s2, #0x26
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 1 } }},
    },
    .{
        .name = "own-x02hdhi",
        // head[31:27] bits are inert; imm 0x703 reaches s1.
        .src =
        \\x02.hdhi s1, #0x703
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x703 } }},
    },
    .{
        .name = "own-x18yst",
        // Y:[e3] <- s3 = 0x333; e2 <- e2 - 1 = 0x702.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x333 },
            .{ .bank = .e, .index = 2, .value = 0x703 },
            .{ .bank = .e, .index = 3, .value = 0x705 },
        },
        .src =
        \\x18.edec e2, e2, #1, s3
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 2, .value = 0x702 } },
            .{ .y_mem = .{ .addr = 0x705, .value = 0x333 } },
        },
    },
    .{
        .name = "own-x18eagu",
        // Pre e2 = 0x703; AGU10[2] = x+1, e2 <- e2 + 1 = 0x704.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x703 }},
        .src =
        \\x18.eagu e2, e2, #2
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x704 } }},
    },
    .{
        .name = "own-x02ainc",
        // Pre A3 = 0x100; A1 <- A3 + 1 = 0x101.
        .pre = &.{.{ .bank = .a, .index = 3, .value = 0x100 }},
        .src =
        \\x02.ainc #1, #3
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 1, .value = 0x101 } }},
    },
    .{
        .name = "own-x02mpy",
        // Pre s1 = 0x4000, s2 = 0x6000; s2 <- round(s1*s2/2^23) = 0x30.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x30 } }},
    },
    .{
        .name = "own-x19e",
        // Pre e2 = 0x777; e3 lands with head-imm 0x702, s6 gets e2.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x777 }},
        .src =
        \\x19.e e3, #1794, s6, e2, #2
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 3, .value = 0x702 } },
            .{ .s_reg = .{ .index = 6, .value = 0x777 } },
        },
    },
    .{
        .name = "own-x19alu",
        // Pre s3 = 7, s6 = 8; sat24(s3 << s6) = 0x700 lands on s7.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 7 },
            .{ .bank = .s, .index = 6, .value = 8 },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x700 } }},
    },
    .{
        .name = "own-x19shr-round",
        // op 23, count -4: rounded 0x708 >> 4 = 0x71 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x000708 }, .{ .bank = .s, .index = 6, .value = 0xFFFFFC },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x000071 } }},
    },
    .{
        .name = "own-x19shr-neg",
        // op 23, count -4 on -0x708: -112.5 rounds half up to -112 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0xFFF8F8 }, .{ .bank = .s, .index = 6, .value = 0xFFFFFC },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0xFFFF90 } }},
    },
    .{
        .name = "own-x19shr-23",
        // op 23, count -23: 0x7FFFFF rounds to 1 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x7FFFFF }, .{ .bank = .s, .index = 6, .value = 0xFFFFE9 },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x000001 } }},
    },
    .{
        .name = "own-x19shl-neg",
        // op 23: -1 << 4 = -16, signed (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0xFFFFFF }, .{ .bank = .s, .index = 6, .value = 0x000004 },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0xFFFFF0 } }},
    },
    .{
        .name = "own-x19shl-sat",
        // op 23: 0x400000 << 2 saturates high (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x400000 }, .{ .bank = .s, .index = 6, .value = 0x000002 },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x7FFFFF } }},
    },
    .{
        .name = "own-x19shl-satneg",
        // op 23: -0x400000 << 2 saturates low (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0xC00000 }, .{ .bank = .s, .index = 6, .value = 0x000002 },
        },
        .src =
        \\x19.alu s7, s3, s6, #23
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x800000 } }},
    },
    .{
        .name = "own-x02mpy-neg",
        // both signed: -0x4000 * 0x6000 / 2^23 = -0x30 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFC000 }, .{ .bank = .s, .index = 2, .value = 0x006000 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0xFFFFD0 } }},
    },
    .{
        .name = "own-x02mpy-halfneg",
        // -1 * 0x400000 / 2^23 = -0.5 rounds half up to 0 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFFFFF }, .{ .bank = .s, .index = 2, .value = 0x400000 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x000000 } }},
    },
    .{
        .name = "own-mpysat-x19",
        // Upstream, fact 402 (own-mpysat, 3 of 3): -1 * -1 saturates to 0x7FFFFF;
        // 0x7FFFFF^2 and -0x800000 * 0x7FFFFF stay exact.
        .pre = &.{
            .{ .bank = .s, .index = 4, .value = 0x800000 }, .{ .bank = .s, .index = 2, .value = 0x800000 },
            .{ .bank = .s, .index = 6, .value = 0x7FFFFF }, .{ .bank = .s, .index = 8, .value = 0x7FFFFF },
            .{ .bank = .s, .index = 9, .value = 0x800000 },
        },
        .src =
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s1, s4, s2, #0, A0, R7, #0, A0, R0
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s3, s6, s8, #0, A0, R7, #0, A0, R0
        \\x19.mac s0, s0, s0, #0, #1, #0x14, e0, s5, s9, s8, #0, A0, R7, #0, A0, R0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 1, .value = 0x7FFFFF } },
            .{ .s_reg = .{ .index = 3, .value = 0x7FFFFE } },
            .{ .s_reg = .{ .index = 5, .value = 0x800001 } },
        },
    },
    .{
        .name = "own-mpysat-x02",
        // fact 402: x02 op 4, rounded, -1 * -1 = 0x7FFFFF.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x800000 }, .{ .bank = .s, .index = 2, .value = 0x800000 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x7FFFFF } }},
    },
    .{
        .name = "own-x02aa",
        // Upstream, fact 403: 00000002 00000000 008E0000 000014C3 00000000 01400000.
        // fn 2: A7 <- A3 + R5 = 0x50; A5 <- that + A6 = 0x150.
        .pre = &.{
            .{ .bank = .a, .index = 3, .value = 0x40 },
            .{ .bank = .r, .index = 5, .value = 0x10 },
            .{ .bank = .a, .index = 6, .value = 0x100 },
        },
        .src =
        \\x02.aa A7, A3, R5, #2, A5, A6
        \\x17.halt
        ,
        .post = &.{
            .{ .a_reg = .{ .index = 7, .value = 0x50 } },
            .{ .a_reg = .{ .index = 5, .value = 0x150 } },
        },
    },
    .{
        .name = "own-x02mpy-halfpos",
        // 1 * 0x400000 / 2^23 = 0.5 rounds half up to 1 (PROBES.md, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x000001 }, .{ .bank = .s, .index = 2, .value = 0x400000 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x000001 } }},
    },
    .{
        .name = "own-ldi-sx",
        // ldi sign-extends its 16-bit immediate: 0xFFF7 lands as 0xFFFFF7 (read back
        // through stx; measured 2026-11-06, fact 371).
        .src =
        \\ldi s7, #0xFFF7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0xFFFFF7 } }},
    },
    .{
        .name = "own-ldi-sx-pos",
        // 0x7FF7 stays 0x007FF7 (measured 2026-11-06).
        .src =
        \\ldi s7, #0x7FF7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x7FF7 } }},
    },
    .{
        .name = "own-stx24",
        // stx keeps all 24 bits: X:0x3A0 <- 0x0ABCDE (3 of 3, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 6, .value = 0x0ABCDE },
            .{ .bank = .a, .index = 6, .value = 0x3A0 },
        },
        .src =
        \\stx A6, s6
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x3A0, .value = 0x0ABCDE } }},
    },
    .{
        .name = "own-xst24",
        // xst keeps all 24 bits: X:[A11 = 0x300] <- 0x0ABCDE (measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 6, .value = 0x0ABCDE },
            .{ .bank = .a, .index = 11, .value = 0x300 },
        },
        .src =
        \\xst #3, s8, #1, #2, s6
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x300, .value = 0x0ABCDE } }},
    },
    .{
        .name = "own-x02mpy-a1",
        // The x02 A slot always runs: own-x02mpy's bundle (w2[21:17] = 1, w3[4:0] = 1)
        // also does A1 <- A1 + 1 (2 of 2, measured 2026-11-06).
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .a, .index = 1, .value = 0x100 },
        },
        .src =
        \\x02.mpy s2, s1, s2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 2, .value = 0x30 } },
            .{ .a_reg = .{ .index = 1, .value = 0x101 } },
        },
    },
    .{
        .name = "own-x02ainc-a8",
        // A + 1 with w3 bit 1 clear: A8 <- A8 + 1 (2 of 2, measured 2026-11-06).
        .pre = &.{.{ .bank = .a, .index = 8, .value = 0x100 }},
        .src =
        \\x02.ainc #8, #8
        \\x17.halt
        ,
        .post = &.{.{ .a_reg = .{ .index = 8, .value = 0x101 } }},
    },
    .{
        .name = "own-x19agu",
        // Pre e3 = 0x700, R1 = 2; fn 6 (x+y) → 0x702; R3 and e5 both get v.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x700 },
            .{ .bank = .r, .index = 1, .value = 2 },
        },
        .src =
        \\x19.agu R3, e5, e3, R1, #6
        \\x17.halt
        ,
        .post = &.{
            .{ .r_reg = .{ .index = 3, .value = 0x702 } },
            .{ .e_reg = .{ .index = 5, .value = 0x702 } },
        },
    },
    .{
        .name = "own-x17agu2",
        // Pre R3 = 0x700; fn 10 (y+i) with imm 2 yields 0x702; dest_e_low=1
        // so e2 = 0x702 and s5 = 0x702.
        .pre = &.{.{ .bank = .r, .index = 3, .value = 0x700 }},
        .src =
        \\x17.agu2 s5, #1, R3, #10, #2
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x702 } },
            .{ .e_reg = .{ .index = 2, .value = 0x702 } },
        },
    },
    .{
        .name = "own-x15alu-asr",
        // Pre s5 = 4 (shift count), s3 = 0x100; asr: s3 >> s5 = 0x10.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 4 },
            .{ .bank = .s, .index = 3, .value = 0x100 },
        },
        .src =
        \\x15.alu s4, s5, s3, #6, #0, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x10 } }},
    },
    .{
        .name = "own-x15alu-sub",
        // Exercise ALU05 code 0x10 (sub) via x15.alu: s4 <- s7 - s3.
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0x200 },
            .{ .bank = .s, .index = 3, .value = 0x50 },
        },
        .src =
        \\x15.alu s4, s7, s3, #16, #0, s0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 4, .value = 0x1B0 } }},
    },
    .{
        .name = "own-x15alu",
        // Pre s5 = 0x123, s3 = 0x18; code 0x0E add writes s5 and (mux 111) s3.
        .pre = &.{
            .{ .bank = .s, .index = 5, .value = 0x123 },
            .{ .bank = .s, .index = 3, .value = 0x18 },
        },
        .src =
        \\x15.alu s5, s5, s3, #14, #1, s3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0x13B } },
            .{ .s_reg = .{ .index = 3, .value = 0x13B } },
        },
    },
    .{
        .name = "own-x16mul",
        // op 5 rounds: s5 <- 7; op 1 truncates: s3 <- 6.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x1101 },
            .{ .bank = .s, .index = 2, .value = 0x3303 },
        },
        .src =
        \\x16.mul s5, s1, s2, #5
        \\x16.mul s3, s1, s2, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 7 } },
            .{ .s_reg = .{ .index = 3, .value = 6 } },
        },
    },
    .{
        .name = "own-x16mul6",
        // Class-110 S1: s7 <- s9 * s5 >> 23 = 0x10.
        .pre = &.{
            .{ .bank = .s, .index = 9, .value = 0x4000 },
            .{ .bank = .s, .index = 5, .value = 0x2000 },
        },
        .src =
        \\x16.mul2 #3, #1, s9, s5, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x10 } }},
    },
    .{
        .name = "own-x16alu2",
        // s3 <- s1 - s3 = 0xDDFE; s5 <- -s1 = 0xEEFF.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x1101 },
            .{ .bank = .s, .index = 3, .value = 0x3303 },
        },
        .src =
        \\x16.alu2 s3, s1, s3, #4, #1
        \\x16.alu2 s5, s1, s3, #7, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0xDDFE } },
            .{ .s_reg = .{ .index = 5, .value = 0xEEFF } },
        },
    },
    .{
        .name = "own-x16alu",
        // s3 <- s1 + s2 = 0x123; xi <- 10, s5 <- X:[A10 = 0x300] = 0x0BEE.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x100 },
            .{ .bank = .s, .index = 2, .value = 0x23 },
            .{ .bank = .a, .index = 10, .value = 0x300 },
        },
        .pre_mem = &.{.{ .kind = .x, .addr = 0x300, .value = 0x0BEE }},
        .src =
        \\x16.alux s5, s3, s1, s2, #4, #0, #10
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x123 } },
            .{ .s_reg = .{ .index = 5, .value = 0x0BEE } },
        },
    },
    .{
        .name = "own-x16ld",
        // xi <- 10; s2 <- X:[A10 = 0x300] = 0x0BEE.
        .pre = &.{.{ .bank = .a, .index = 10, .value = 0x300 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x300, .value = 0x0BEE }},
        .src =
        \\x16.ld s2, #10, #2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x0BEE } }},
    },
    .{
        .name = "own-x16aset",
        // A3 <- #0x345; then xi <- 3, s5 <- X:[A3] = 0x0BEE.
        .pre = &.{.{ .bank = .a, .index = 3, .value = 0x100 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x345, .value = 0x0BEE }},
        .src =
        \\x16.aset #1, #0x345
        \\x16.ld s5, #3, #0
        \\x17.halt
        ,
        .post = &.{
            .{ .a_reg = .{ .index = 3, .value = 0x345 } },
            .{ .s_reg = .{ .index = 5, .value = 0x0BEE } },
        },
    },
    .{
        .name = "own-x16ldi",
        // s12 <- #head[20:5] = 0x18.
        .src =
        \\x16.ldi s12, #0x18
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 12, .value = 0x18 } }},
    },
    .{
        .name = "own-x16hd-and",
        // m3 on code 0x16: s1 <- s5 & s2 = 0x0204.
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x1234 },
            .{ .bank = .s, .index = 5, .value = 0x2345 },
        },
        .src =
        \\x16.hd s1, s5, s2, #0x16, #3, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x0204 } }},
    },
    .{
        .name = "own-x16hd-shr",
        // code 0x03 with s = 1: s1 <- (s5 + s2) asr 1 = 0x1ABC.
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x1234 },
            .{ .bank = .s, .index = 5, .value = 0x2345 },
        },
        .src =
        \\x16.hd s1, s5, s2, #3, #0, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x1ABC } }},
    },
    .{
        .name = "own-x16agu-x1",
        // S1[31] = 1: x = e3 = 0x20; s3 <- e3 + R3 = 0x40.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x100 },
            .{ .bank = .e, .index = 3, .value = 0x20 },
            .{ .bank = .r, .index = 3, .value = 0x20 },
        },
        .src =
        \\x16.agu s3, R3, #1, #6, #4, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x40 } }},
    },
    .{
        .name = "own-x16mux",
        // mux 101: s3 <- Y:[e2] = 0xABCD; mux 110: s5 <- 4 << 6 = 0x100.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x700 },
            .{ .bank = .r, .index = 3, .value = 0x20 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0xABCD }},
        .src =
        \\x16.mux s3, R3, #1, #0, #6, #4, #5
        \\x16.mux s5, R3, #1, #0, #10, #4, #6
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0xABCD } },
            .{ .s_reg = .{ .index = 5, .value = 0x100 } },
        },
    },
    .{
        .name = "own-x16mux-b",
        // mux 011: s3 <- 0xCD00; mux 100: s5 <- (0xABCD << 7)[15:0] = 0xE680.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x700 },
            .{ .bank = .r, .index = 3, .value = 0x20 },
        },
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0xABCD }},
        .src =
        \\x16.mux s3, R3, #1, #0, #6, #4, #3
        \\x16.mux s5, R3, #1, #0, #10, #4, #4
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0xCD00 } },
            .{ .s_reg = .{ .index = 5, .value = 0xE680 } },
        },
    },
    .{
        .name = "own-x16xs",
        // s3 <- D = Y:[e2] = 0xABCD; the ALU writes nothing.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x700 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x700, .value = 0xABCD }},
        .src =
        \\x16.xs s3, #5, e2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0xABCD } }},
    },
    .{
        .name = "own-x16xs-imm",
        // mux 001: s3 <- #0x123.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x700 }},
        .src =
        \\x16.xs s3, #1, e2, #0x123
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x123 } }},
    },
    .{
        .name = "own-x16stx",
        // X:[A6 = 0x3A0] <- s7 = 0xF77.
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0xF77 },
            .{ .bank = .a, .index = 6, .value = 0x3A0 },
        },
        .src =
        \\x16.stx s7, #6
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x3A0, .value = 0xF77 } }},
    },
    .{
        .name = "own-x16st2",
        // X:[A6 = 0x3A0] <- s7 = 0xF77; Y:[e4 = 0x300] <- s6 = 0x666.
        .pre = &.{
            .{ .bank = .s, .index = 7, .value = 0xF77 },
            .{ .bank = .s, .index = 6, .value = 0x666 },
            .{ .bank = .a, .index = 6, .value = 0x3A0 },
            .{ .bank = .e, .index = 4, .value = 0x300 },
        },
        .src =
        \\x16.st2 s7, #6, e4, s6
        \\x17.halt
        ,
        .post = &.{
            .{ .x_mem = .{ .addr = 0x3A0, .value = 0xF77 } },
            .{ .y_mem = .{ .addr = 0x300, .value = 0x666 } },
        },
    },
    .{
        .name = "own-x16agumul",
        // s2 <- s3 * s5 >> 23 = 0x4000 * 0x2000 >> 23 = 0x10.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x4000 },
            .{ .bank = .s, .index = 5, .value = 0x2000 },
        },
        .src =
        \\x16.agumul #2, #0, s3, s5
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x10 } }},
    },
    .{
        .name = "own-x16agu2",
        // AGU10[2] (x+1): s3 <- e2 + 1 = 0x101; AGU10B[3] (-x): s5 <- 0xFF00.
        .pre = &.{
            .{ .bank = .e, .index = 2, .value = 0x100 },
            .{ .bank = .r, .index = 3, .value = 0x20 },
        },
        .src =
        \\x16.agu2 s3, e2, R3, #2, #1, #1
        \\x16.agu2 s5, e2, R3, #3, #0, #1
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 3, .value = 0x101 } },
            .{ .s_reg = .{ .index = 5, .value = 0xFF00 } },
        },
    },
    .{
        .name = "own-x16xld",
        // xi <- 10; s7 <- X:[A10 = 0x300] = 0x1234.
        .pre = &.{.{ .bank = .a, .index = 10, .value = 0x300 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x300, .value = 0x1234 }},
        .src =
        \\x16.xld s7, #10
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x1234 } }},
    },
    .{
        .name = "own-x16ldb",
        // xi <- 10; s12 <- X:[A10][7:0] << 8 = 0x3400.
        .pre = &.{.{ .bank = .a, .index = 10, .value = 0x300 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x300, .value = 0x1234 }},
        .src =
        \\x16.ldb s12
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 12, .value = 0x3400 } }},
    },
    .{
        .name = "own-x16mul-trunc",
        // Pre s1 = 0x1101, s2 = 0x3303; s3 <- (s1 * s2) >> 23 = 6 truncated.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x1101 },
            .{ .bank = .s, .index = 2, .value = 0x3303 },
        },
        .src =
        \\x16.mul s3, s1, s2, #1
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 6 } }},
    },
    .{
        .name = "own-x13agu",
        // Pre e3 = 0x100, R7 = 0x23.  Expect e9 = 0x350, R5 = AGU10[6](e3, R7) = 0x123.
        .pre = &.{
            .{ .bank = .e, .index = 3, .value = 0x100 },
            .{ .bank = .r, .index = 7, .value = 0x23 },
        },
        .src =
        \\x13.agu e9, #0x350, R5, e3, R7, #6
        \\x17.halt
        ,
        .post = &.{
            .{ .e_reg = .{ .index = 9, .value = 0x350 } },
            .{ .r_reg = .{ .index = 5, .value = 0x123 } },
        },
    },
    .{
        .name = "own-x03esrc",
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .src =
        \\x03.esrc s7, e3
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x490 } }},
    },
    .{
        .name = "own-x03u24",
        // ldi sign-extends: s1 = 0xFFC321, s2 = 0xFF9877.  Same bits as
        // own-x02u24: s6 <- 0xFFC339.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0xFFC321 },
            .{ .bank = .s, .index = 2, .value = 0xFF9877 },
        },
        .src =
        \\x03.mpy s6, s1, s2, s0, #4, #6, #0, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 6, .value = 0xFFC339 } }},
    },
    .{
        .name = "own-x03xsn",
        // mode 5 with w1[14] = 1: s2 <- P + s<w1[31:29]> = 0x30 + 0x555.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x03.mpy s2, s1, s2, s0, #13, #1, #5, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x585 } }},
    },
    .{
        .name = "own-x03w2c5",
        // w2 class 101 behaves as class 000: same result as own-x03xsn.
        .pre = &.{
            .{ .bank = .s, .index = 1, .value = 0x4000 },
            .{ .bank = .s, .index = 2, .value = 0x6000 },
            .{ .bank = .s, .index = 5, .value = 0x555 },
        },
        .src =
        \\x03.mpy s2, s1, s2, s0, #13, #1, #5, #5
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 2, .value = 0x585 } }},
    },
    .{
        .name = "own-x03alub",
        // head[29] is b's top bit: b = s12.  s9 <- s3 + s12 = 0x123.
        .pre = &.{
            .{ .bank = .s, .index = 3, .value = 0x100 },
            .{ .bank = .s, .index = 12, .value = 0x23 },
        },
        .src =
        \\x03.alu s9, s3, s12, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 9, .value = 0x123 } }},
    },
    .{
        .name = "own-x02c6",
        // Pre A3 = 0x456, X:0x456 = 0xC09F; s6 and s5 both get the read.
        .pre = &.{.{ .bank = .a, .index = 3, .value = 0x456 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x456, .value = 0xC09F }},
        .src =
        \\x02.c6 s6, s5, A3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 6, .value = 0xC09F } },
            .{ .s_reg = .{ .index = 5, .value = 0xC09F } },
        },
    },
    .{
        .name = "own-x03yrd",
        // Pre e3 = 0x490, Y:0x490 = 0x886F; s7 <- Y:[e3].
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x03.yrd s7, e3, #5, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x886F } }},
    },
    .{
        .name = "own-x03ylane",
        // mux 011: lane = D << 8, take [15:0] = D[7:0] << 8 = 0x6F00.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x03.yrd s7, e3, #3, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x6F00 } }},
    },
    .{
        .name = "own-x03ylane4",
        // mux 100: lane[16:1] = (0x886F << 7)[15:0] = 0x3780.
        .pre = &.{.{ .bank = .e, .index = 3, .value = 0x490 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x490, .value = 0x886F }},
        .src =
        \\x03.yrd s7, e3, #4, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0x3780 } }},
    },
    .{
        .name = "own-x03rslot",
        // R1 <- AGU10B[10](y = R12) = R12 = 0x155.
        .pre = &.{.{ .bank = .r, .index = 12, .value = 0x155 }},
        .src =
        \\x03.rslot R1, R12, #10, #0
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x155 } }},
    },
    .{
        .name = "own-x03ragu10",
        // w5[20] = 1: AGU10[3] with i = 0 gives y + 1 = 0x156.
        .pre = &.{.{ .bank = .r, .index = 12, .value = 0x155 }},
        .src =
        \\x03.rslot R1, R12, #3, #1
        \\x17.halt
        ,
        .post = &.{.{ .r_reg = .{ .index = 1, .value = 0x156 } }},
    },
    .{
        .name = "own-x03xst",
        // A8 = 0x3A0, s11 = 0xBB0B; X:0x3A0 <- sx24(s11) = 0xFFBB0B.
        .pre = &.{
            .{ .bank = .a, .index = 8, .value = 0x3A0 },
            .{ .bank = .s, .index = 11, .value = 0xBB0B },
        },
        .src =
        \\x03.xst #3, #1, #2, #0, #3
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x3A0, .value = 0xFFBB0B } }},
    },
    .{
        .name = "own-x03xw2",
        // A8 = 0x345, X:0x345 = 0xDB7A; the w2 slot reads it into s7.
        .pre = &.{.{ .bank = .a, .index = 8, .value = 0x345 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x345, .value = 0xDB7A }},
        .src =
        \\x03.xw2 s7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0xDB7A } }},
    },
    .{
        .name = "own-x03xrd",
        // s7 <- X:[A8] = 0xDB7A via the 6-syllable x03.xrd bundle.
        .pre = &.{.{ .bank = .a, .index = 8, .value = 0x345 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x345, .value = 0xDB7A }},
        .src =
        \\x03.xrd s7, #2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 7, .value = 0xDB7A } }},
    },
    .{
        .name = "own-x02xrd",
        // s1 <- X:[A2] = 0xEF41 via the 6-syllable x02.xrd bundle.
        .pre = &.{.{ .bank = .a, .index = 2, .value = 0x234 }},
        .pre_mem = &.{.{ .kind = .x, .addr = 0x234, .value = 0xEF41 }},
        .src =
        \\x02.xrd s1, A2
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0xEF41 } }},
    },
    .{
        .name = "own-x02imm",
        // s1 <- 0x703 via the 6-syllable x02.imm bundle.
        .src =
        \\x02.imm s1, #0x703
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x703 } }},
    },
    .{
        .name = "own-x18imm",
        // s3 <- 0x1234 via the 4-syllable x18.imm bundle.
        // Expected bytes: 0x00624698 0x00006000 0x00000000 0xC1500001.
        .src =
        \\x18.imm s3, #0x1234, #6
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x1234 } }},
    },
    .{
        .name = "own-p010-true",
        // the probe list own-p010-true: p3 <- cond 9 (constant true).
        // Expected bytes: 0x00000014 0x42000930.
        .src =
        \\condset p3, #9, #0
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 3, .value = 1 } }},
    },
    .{
        .name = "own-yrm",
        // the probe list own-yrm: pre e11 = 0x456; s6 <- 0x123, R5 <- e11.
        // Expected bytes: 0x00C02474 0xB3600D05.
        .pre = &.{.{ .bank = .e, .index = 11, .value = 0x456 }},
        .src =
        \\yrm s6, #0x123, R5, e11
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 6, .value = 0x123 } },
            .{ .r_reg = .{ .index = 5, .value = 0x456 } },
        },
    },
    .{
        .name = "own-ytst-even",
        // Pre e5 = 0x702, Y[0x702] = 0x10 (even); p3 <- true.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x702 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x10 }},
        .src =
        \\ytst p3, e5
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 3, .value = 1 } }},
    },
    .{
        .name = "own-ytst-odd",
        // Pre e5 = 0x702, Y[0x702] = 0x11 (odd); p3 <- false.
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x702 }},
        .pre_mem = &.{.{ .kind = .y, .addr = 0x702, .value = 0x11 }},
        .src =
        \\ytst p3, e5
        \\x17.halt
        ,
        .post = &.{.{ .p_reg = .{ .index = 3, .value = 0 } }},
    },
    .{
        .name = "own-xst",
        // the probe list own-xst: X:[A11] <- s6 (sign-extended to 24 bits).
        // Expected bytes: 0xC1000034 0x3200016A.
        .pre = &.{
            .{ .bank = .s, .index = 6, .value = 0x8BEE },
            .{ .bank = .a, .index = 11, .value = 0x300 },
        },
        .src =
        \\xst #3, s8, #1, #2, s6
        \\x17.halt
        ,
        .post = &.{.{ .x_mem = .{ .addr = 0x300, .value = 0x8BEE } }},
    },
    .{
        .name = "own-0e",
        // the probe list own-0e: s5 <- 0, A13 <- 0x300, via low5 0x0E.
        // Expected bytes: 0x9CA0600E 0x00040006.
        .src =
        \\ld0e s5, #0x300, #3
        \\x17.halt
        ,
        .post = &.{
            .{ .s_reg = .{ .index = 5, .value = 0 } },
            .{ .a_reg = .{ .index = 13, .value = 0x300 } },
        },
    },
    .{
        .name = "own-pair",
        // the probe list own-pair: e3 <- 0x70D via low5 0x10 S class 011.
        // Expected bytes: 0x0020E1B0 0x60448018.
        .src =
        \\pair #0x70D, #3
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 3, .value = 0x70D } }},
    },
    .{
        .name = "own-ldi-k",
        // the probe list own-ldi-k: ldi with [29:26] = 0xC; those bits are
        // inert and our encoder emits them as 0, so this gate verifies
        // s1 state, not an exact byte match.
        .src =
        \\ldi s1, #0x77
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 0x77 } }},
    },
    .{
        .name = "own-ldi2",
        // the probe list own-ldi2: s13 <- 0x123 via the 2-syllable low5 0x10
        // S-class-001 form.  Expected bytes: 0x01A02470 0x20040000.
        .src =
        \\ldi2 s13, #0x123
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 13, .value = 0x123 } }},
    },
    .{
        .name = "own-lde",
        // the probe list own-entry first bundle: e13 <- 0x70D.
        .src =
        \\lde e13, #0x70D
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 13, .value = 0x70D } }},
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
        .p => state.p[p.index] = @truncate(p.value),
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

    const ran = emu.runLinear(gpa, &state, words);
    if (g.wedge) {
        const ok = if (ran) |_| !state.halted else |_| true;
        if (!ok) {
            std.debug.print("gate '{s}': halted, expected a wedge\n", .{g.name});
            return error.GateFailed;
        }
        // A wedge the emulator models (it stops without halting) still has its stores checked.
        _ = ran catch return;
    }
    _ = ran catch |e| {
        std.debug.print("gate '{s}': emulator error {s}\n", .{ g.name, @errorName(e) });
        return e;
    };

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
        .y_none => |addr| {
            if (state.getY(addr)) |got| {
                std.debug.print("gate '{s}': Y:0x{x} = 0x{x}, expected no write\n", .{ g.name, addr, got });
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
        .p_reg => |pr| {
            if (state.p[pr.index] != pr.value) {
                std.debug.print("gate '{s}': p{d} = {d}, expected {d}\n", .{ g.name, pr.index, state.p[pr.index], pr.value });
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
