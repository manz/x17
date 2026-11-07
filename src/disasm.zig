//! x17 disassembler: bytes -> .sm source line(s).
//!
//! Given a bundle (1..7 syllables), walk the form and operand tables and
//! write a human-readable mnemonic with operands.  Single-syllable forms
//! dispatch via `isa.classify`; x17 bundles pick a sub-form by matching
//! S1/S2 class bits against the entries in `isa.subforms`.
//!
//! This is the reverse of the assembler's data-driven encoding: every
//! field read here comes from the same `Operand` table the assembler uses
//! to pack bits, so round-trip (assemble -> disasm) is byte-exact for any
//! form whose operand rows are complete.

const std = @import("std");
const isa = @import("isa");

pub const DisasmError = error{
    UnknownForm,
    BundleTooShort,
    BufferTooSmall,
};

/// Disassemble one bundle into `out`; returns bytes written.  The written
/// text does NOT include a trailing newline so callers can decide.
pub fn disasmBundle(out: []u8, bundle: []const u32) DisasmError!usize {
    if (bundle.len == 0) return DisasmError.BundleTooShort;

    // Try a multi-syllable sub-form first: matchSubForm is strict enough
    // that a mismatch is not a false negative (it compares ALL non-operand
    // bits).  Fall through to the single-syllable form classifier only
    // when no sub-form claims the bundle AND the low5 is known to carry a
    // single-syllable form.
    if (bundle.len >= 2) {
        if (isa.matchSubForm(bundle)) |sf| {
            const ops = isa.operandsFor(sf.name);
            return writeFormLine(out, sf.name, ops, bundle);
        }
        // S1[22:20] = 010 on a continue-shape x17: the 000 form plus `.stop`.
        if (bundle.len >= 3 and bundle[0] & 0x1F == 0x17 and bundle[1] & (7 << 20) == isa.STOP_ROW) {
            var b3 = [3]u32{ bundle[0], bundle[1] & ~isa.STOP_ROW, bundle[2] };
            if (isa.matchSubForm(&b3)) |sf| if (isa.stopCapable(sf)) {
                var name_buf: [64]u8 = undefined;
                const name = std.fmt.bufPrint(&name_buf, "{s}.stop", .{sf.name}) catch return DisasmError.BufferTooSmall;
                return writeFormLine(out, name, isa.operandsFor(sf.name), &b3);
            };
        }
    }

    // Any low5 we know is multi-syllable must have matched a sub-form above;
    // if it did not, the bundle is unknown.  Keeps us from printing a 1-syll
    // line for a partial x17/x19/alu11 bundle.
    const low5: u8 = @intCast(bundle[0] & 0x1F);
    if (low5 == 0x17 or low5 == 0x19 or low5 == 0x11) return DisasmError.UnknownForm;

    const f = isa.classify(bundle[0]) orelse return DisasmError.UnknownForm;
    const ops = isa.operandsFor(f.name);
    return writeFormLine(out, f.name, ops, bundle);
}

fn disasmX17(out: []u8, bundle: []const u32) DisasmError!usize {
    if (bundle.len < 3) return DisasmError.BundleTooShort;
    const sf = isa.matchSubForm(bundle) orelse return DisasmError.UnknownForm;
    const ops = isa.operandsFor(sf.name);
    return writeFormLine(out, sf.name, ops, bundle);
}

fn writeFormLine(
    out: []u8,
    name: []const u8,
    ops: []const isa.Operand,
    bundle: []const u32,
) DisasmError!usize {
    var cur: usize = 0;
    cur += printTo(out[cur..], "{s}", .{name});
    for (ops, 0..) |op, i| {
        const sep: []const u8 = if (i == 0) " " else ", ";
        cur += printTo(out[cur..], "{s}", .{sep});
        if (op.syll >= bundle.len) return DisasmError.BundleTooShort;
        const word = bundle[op.syll];
        const field = (word >> @as(u5, @intCast(op.lo))) & ((@as(u32, 1) << @as(u5, @intCast(op.width()))) - 1);
        cur += formatOperand(out[cur..], op, field);
    }
    // e0 as a Y address is no register (fact 466, 468): say so where it is read.
    if (std.mem.eql(u8, name, "st.wr") and bundle.len >= 2 and (bundle[1] >> 17) & 0xF == 0)
        cur += printTo(out[cur..], "{s}", .{" ; e0 = the previous word's h[20:5], not a register"});
    return cur;
}

fn formatOperand(out: []u8, op: isa.Operand, field: u32) usize {
    return switch (op.kind) {
        .s => printTo(out, "s{d}", .{field}),
        .e => printTo(out, "e{d}", .{field}),
        .r_reg => printTo(out, "R{d}", .{field}),
        .a_reg => printTo(out, "A{d}", .{field}),
        .p => printTo(out, "p{d}", .{field}),
        .imm, .rel, .relu, .rel4 => printTo(out, "#0x{x}", .{field}),
        .dest_ldi => blk: {
            if (field >= 16 and field <= 31) {
                break :blk printTo(out, "s{d}", .{field - 16});
            }
            break :blk printTo(out, "#0x{x}", .{field}); // raw for OPEN slots
        },
        .y_addr => printTo(out, "Y:0x{x}", .{field}),
    };
}

fn printTo(out: []u8, comptime fmt: []const u8, args: anytype) usize {
    const w = std.fmt.bufPrint(out, fmt, args) catch return 0;
    return w.len;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;
const asm_mod = @import("asm");

fn roundTrip(src: []const u8) ![]const u8 {
    const b = try asm_mod.assembleOne(src);
    var buf: [128]u8 = undefined;
    const n = try disasmBundle(&buf, b.slice());
    // Persist into a static via dupeZ-like trick: leak ok for tests.
    const out = try testing.allocator.dupe(u8, buf[0..n]);
    return out;
}

test "disasm rli" {
    const s = try roundTrip("rli R3, s5, #0x1234");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("rli R3, s5, #0x1234", s);
}

test "disasm ldi" {
    const s = try roundTrip("ldi s0, #0x1234");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("ldi s0, #0x1234", s);
}

test "disasm st.y" {
    const s = try roundTrip("st.y Y:0x200");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("st.y Y:0x200", s);
}

test "disasm alu" {
    const s = try roundTrip("alu s7, s2, s9, #0x16");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("alu s7, s2, s9, #0x16", s);
}

test "disasm x17.halt" {
    const s = try roundTrip("x17.halt");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.halt", s);
}

test "disasm x17.imm" {
    const s = try roundTrip("x17.imm s5, #0xBEE");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.imm s5, #0xbee", s);
}

test "disasm x17.alu" {
    const s = try roundTrip("x17.alu s1, s8, s2, #0, #3");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.alu s1, s8, s2, #0x0, #0x3", s);
}

test "disasm x17.agu" {
    const s = try roundTrip("x17.agu R5, e3, R2, #2");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.agu R5, e3, R2, #0x2", s);
}

test "disasm x17.e" {
    const s = try roundTrip("x17.e e5, s5, e9, e5, #0x0704");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.e e5, s5, e9, e5, #0x704", s);
}

test "disasm x17.ld" {
    const s = try roundTrip("x17.ld s5, s4, #2, e8, #1");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.ld s5, s4, #0x2, e8, #0x1", s);
}

test "disasm x17.alu05" {
    const s = try roundTrip("x17.alu05 s5, s3, s5, #0");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.alu05 s5, s3, s5, #0x0", s);
}

test "disasm x17.mve" {
    const s = try roundTrip("x17.mve e5, s4");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.mve e5, s4", s);
}

test "disasm x17.rsub" {
    const s = try roundTrip("x17.rsub s5, s3, #0xA, #5, #0");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.rsub s5, s3, #0xa, #0x5, #0x0", s);
}

test "disasm x17.xpm" {
    const s = try roundTrip("x17.xpm s5, #1");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("x17.xpm s5, #0x1", s);
}

test "disasm unknown bundle" {
    const bytes = [_]u32{0x1234_5678}; // low5 = 0x18, not in any form
    var buf: [64]u8 = undefined;
    try testing.expectError(DisasmError.UnknownForm, disasmBundle(&buf, &bytes));
}

test "disasm st.wr through e0 says e0 is no register" {
    const s = try roundTrip("st.wr #0x40, e0, s1");
    defer testing.allocator.free(s);
    try testing.expectEqualStrings("st.wr #0x40, e0, s1 ; e0 = the previous word's h[20:5], not a register", s);
}

test "disasm pc14 and x18.pnot round-trip Upstream's gate words" {
    const a = try roundTrip("pc14 p13, s7, #0xb, #0xc, #0x2, s7, #0x0, #0x14a3");
    defer testing.allocator.free(a);
    try testing.expectEqualStrings("pc14 p13, s7, #0xb, #0xc, #0x2, s7, #0x0, #0x14a3", a);
    const b = try roundTrip("x18.pnot p1, p5");
    defer testing.allocator.free(b);
    try testing.expectEqualStrings("x18.pnot p1, p5", b);
}

test "disasm prints the generic .stop suffix" {
    const a = try roundTrip("x17.ms.stop s0, #0x0, s5, e3, #0x1, #0x3, #0x0, R2, #0x1");
    defer testing.allocator.free(a);
    try testing.expectEqualStrings("x17.ms.stop s0, #0x0, s5, e3, #0x1, #0x3, #0x0, R2, #0x1", a);
    const b = try roundTrip("x17.sty.stop e5, #0x3, #0x0");
    defer testing.allocator.free(b);
    try testing.expectEqualStrings("x17.sty.stop e5, #0x3, #0x0", b);
}

test "a form with an operand on S1[22:20] takes no .stop" {
    try testing.expectError(error.AssemblyFailed, asm_mod.assembleOne("x17.ld.stop s1, e2"));
}
