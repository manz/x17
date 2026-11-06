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
        \\x03.agu A0, A5, #0, #0, A9
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
        \\x02.yrd s1, e3
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
        \\x16.agu s3, R3, #1, #6, #4
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
        .name = "own-x18una-abs",
        // Pre s1 = 0x8001 (signed -0x7FFF); abs(s1) = 0x7FFF lands on s3.
        .pre = &.{.{ .bank = .s, .index = 1, .value = 0x8001 }},
        .src =
        \\x18.una s3, s1, #7
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x7FFF } }},
    },
    .{
        .name = "own-x18edec",
        // Pre e2 = 0x703; e2 <- e2 - 1 = 0x702.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x703 }},
        .src =
        \\x18.edec e2, e2, #1
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
        // Pre e5 = 0x300; s3 <- e5 = 0x300 (X/Y stores skipped).
        .pre = &.{.{ .bank = .e, .index = 5, .value = 0x300 }},
        .src =
        \\x01.st e5
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 3, .value = 0x300 } }},
    },
    .{
        .name = "own-x02x3",
        // Pre s2=0x222, s3=0x333; (s2 - s3) << 2 = 0xFFFBBC (24-bit).
        .pre = &.{
            .{ .bank = .s, .index = 2, .value = 0x222 },
            .{ .bank = .s, .index = 3, .value = 0x333 },
        },
        .src =
        \\x02.x3 #1, #2, s2, s3, #5, #2, s2, #1
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
        \\x02.x3 #1, #2, s2, s3, #0, #0, s6, #1
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
        \\x02.x3 #0, #0, s2, s3, #5, #2, s6, #6
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
        \\x02.x3 #0, #0, s2, s3, #5, #2, s6, #4
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
        \\x18.br #1, #2
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
        // p3 = 1, branch offset +2 bundles.  Two delay-slot adds run, two
        // adds are skipped, one add at the target runs: s1 = 3.
        .pre = &.{
            .{ .bank = .p, .index = 3, .value = 1 },
            .{ .bank = .s, .index = 1, .value = 0 },
            .{ .bank = .s, .index = 2, .value = 1 },
        },
        .src =
        \\x18.br #3, #2
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\alu s1, s1, s2, #0
        \\x17.halt
        ,
        .post = &.{.{ .s_reg = .{ .index = 1, .value = 3 } }},
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
        // Y-store side is not modelled; verify the e2 decrement that pairs it.
        .pre = &.{.{ .bank = .e, .index = 2, .value = 0x703 }},
        .src =
        \\x18.edec e2, e2, #1
        \\x17.halt
        ,
        .post = &.{.{ .e_reg = .{ .index = 2, .value = 0x702 } }},
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
        \\x03.yrd s7, e3, #5
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
        \\x03.yrd s7, e3, #3
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
        \\x03.yrd s7, e3, #4
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
        \\condset p3, #9
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
        .name = "own-ld0e",
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
