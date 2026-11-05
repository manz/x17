//! x17 assembler, v0: one line, one bundle, one syllable out.
//!
//! Scope of this spike:
//!   * Lex a single line into tokens.
//!   * Parse `mnemonic op (, op)*`.
//!   * Match operands by file order against `isa.operandsFor(form)`.
//!   * Emit the head syllable by OR'ing the form's `value` with each
//!     encoded operand.
//!
//! What waits:
//!   * Multi-line source, labels, directives, const-expression folding.
//!   * Multi-syllable bundles (x17's head+S1+S2).
//!   * Error locations beyond "something was wrong at offset N".
//!   * ldi's dest decoding beyond s<N> (0 = address entry 0, 1 = source 1,
//!     2..15 marked OPEN in isa.toml - the assembler refuses them today).
//!
//! The round-trip gate (reassemble a disassembled stock bundle byte-exact)
//! bolts onto this file as a test once labels land.

const std = @import("std");
const isa = @import("isa");

pub const AsmError = error{
    EmptyInput,
    UnknownMnemonic,
    UnknownOperand,
    WrongOperandCount,
    OperandOutOfRange,
    BadToken,
    Overflow,
};

// ---------------------------------------------------------------------------
// Lexer
// ---------------------------------------------------------------------------

pub const TokenKind = enum {
    ident, // mnemonics, reg prefixes before the digit, bare labels
    s_reg, // sN
    e_reg, // eN
    r_reg, // RN  (upper R)
    p_reg, // pN
    number, // decimal or 0x-hex
    comma,
    hash,
    colon,
    end,
};

pub const Token = struct {
    kind: TokenKind,
    text: []const u8, // slice into source
    value: u32 = 0, // for number / register index
};

const Lexer = struct {
    src: []const u8,
    i: usize = 0,

    fn peek(self: *Lexer) u8 {
        if (self.i >= self.src.len) return 0;
        return self.src[self.i];
    }

    fn bump(self: *Lexer) u8 {
        const c = self.peek();
        self.i += 1;
        return c;
    }

    fn skipWs(self: *Lexer) void {
        while (self.i < self.src.len) {
            const c = self.src[self.i];
            if (c == ' ' or c == '\t' or c == '\r') {
                self.i += 1;
            } else if (c == '#' and self.i + 1 < self.src.len and (self.src[self.i + 1] == ' ' or isDigit(self.src[self.i + 1]) or self.src[self.i + 1] == '0')) {
                // `#` here is the immediate marker, not whitespace.
                return;
            } else if (c == ';') {
                // line comment to end of line
                while (self.i < self.src.len and self.src[self.i] != '\n') : (self.i += 1) {}
            } else {
                return;
            }
        }
    }

    fn next(self: *Lexer) !Token {
        self.skipWs();
        if (self.i >= self.src.len) return .{ .kind = .end, .text = self.src[self.i..self.i] };

        const start = self.i;
        const c = self.peek();

        if (c == ',') {
            self.i += 1;
            return .{ .kind = .comma, .text = self.src[start..self.i] };
        }
        if (c == ':') {
            self.i += 1;
            return .{ .kind = .colon, .text = self.src[start..self.i] };
        }
        if (c == '#') {
            self.i += 1;
            return .{ .kind = .hash, .text = self.src[start..self.i] };
        }
        if (isDigit(c)) return try self.number();

        if (isAlpha(c)) return try self.wordOrReg();

        return AsmError.BadToken;
    }

    fn number(self: *Lexer) !Token {
        const start = self.i;
        var value: u32 = 0;
        if (self.i + 1 < self.src.len and self.src[self.i] == '0' and (self.src[self.i + 1] == 'x' or self.src[self.i + 1] == 'X')) {
            self.i += 2;
            const digit_start = self.i;
            while (self.i < self.src.len and isHexDigit(self.src[self.i])) : (self.i += 1) {}
            if (self.i == digit_start) return AsmError.BadToken;
            value = std.fmt.parseInt(u32, self.src[digit_start..self.i], 16) catch return AsmError.Overflow;
        } else {
            while (self.i < self.src.len and isDigit(self.src[self.i])) : (self.i += 1) {}
            value = std.fmt.parseInt(u32, self.src[start..self.i], 10) catch return AsmError.Overflow;
        }
        return .{ .kind = .number, .text = self.src[start..self.i], .value = value };
    }

    fn wordOrReg(self: *Lexer) !Token {
        const start = self.i;
        while (self.i < self.src.len and (isAlpha(self.src[self.i]) or isDigit(self.src[self.i]) or self.src[self.i] == '.' or self.src[self.i] == '_')) : (self.i += 1) {}
        const text = self.src[start..self.i];

        // Register? Single prefix letter + run of digits, no other letters.
        if (text.len >= 2 and isDigit(text[1])) {
            const prefix = text[0];
            // Reject if any later char isn't a digit - treats `s0.foo` as
            // ident, `s0` as reg.
            var all_digits = true;
            for (text[1..]) |ch| {
                if (!isDigit(ch)) {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits) {
                const idx = std.fmt.parseInt(u32, text[1..], 10) catch return AsmError.BadToken;
                switch (prefix) {
                    's' => return .{ .kind = .s_reg, .text = text, .value = idx },
                    'e' => return .{ .kind = .e_reg, .text = text, .value = idx },
                    'R' => return .{ .kind = .r_reg, .text = text, .value = idx },
                    'p' => return .{ .kind = .p_reg, .text = text, .value = idx },
                    else => {},
                }
            }
        }
        return .{ .kind = .ident, .text = text };
    }
};

fn isDigit(c: u8) bool {
    return c >= '0' and c <= '9';
}
fn isHexDigit(c: u8) bool {
    return isDigit(c) or (c >= 'a' and c <= 'f') or (c >= 'A' and c <= 'F');
}
fn isAlpha(c: u8) bool {
    return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or c == '_' or c == '.';
}

// ---------------------------------------------------------------------------
// Operand match + encode
// ---------------------------------------------------------------------------

fn matchOperand(op: isa.Operand, tok: Token) !u32 {
    switch (op.kind) {
        .s => {
            if (tok.kind != .s_reg) return AsmError.UnknownOperand;
            if (tok.value > 15) return AsmError.OperandOutOfRange;
            return tok.value;
        },
        .e => {
            if (tok.kind != .e_reg) return AsmError.UnknownOperand;
            if (tok.value > 15) return AsmError.OperandOutOfRange;
            return tok.value;
        },
        .r_reg => {
            if (tok.kind != .r_reg) return AsmError.UnknownOperand;
            if (tok.value > 15) return AsmError.OperandOutOfRange;
            return tok.value;
        },
        .p => {
            if (tok.kind != .p_reg) return AsmError.UnknownOperand;
            return tok.value;
        },
        .imm => {
            if (tok.kind != .number) return AsmError.UnknownOperand;
            const max: u32 = if (op.width() == 32) 0xFFFF_FFFF else (@as(u32, 1) << @as(u5, @intCast(op.width()))) - 1;
            if (tok.value > max) return AsmError.OperandOutOfRange;
            return tok.value;
        },
        .dest_ldi => {
            // v0 accepts only s<N>; maps to k = N + 16 per isa.toml.
            if (tok.kind != .s_reg) return AsmError.UnknownOperand;
            if (tok.value > 15) return AsmError.OperandOutOfRange;
            return tok.value + 16;
        },
    }
}

// ---------------------------------------------------------------------------
// Assemble one line
// ---------------------------------------------------------------------------

/// Assemble one source line into the head syllable of a bundle.
///
/// `src` is one logical line; `#` introduces an immediate, `;` or `#` after
/// whitespace a comment (the lexer already draws that line).  Returns the
/// 32-bit syllable to write at the bundle's head slot.
pub fn assembleOne(src: []const u8) !u32 {
    var lex = Lexer{ .src = src };
    const head = try lex.next();
    if (head.kind == .end) return AsmError.EmptyInput;
    if (head.kind != .ident) return AsmError.UnknownMnemonic;

    // Find the form.
    var form: ?*const isa.Form = null;
    for (isa.forms) |*f| {
        if (std.mem.eql(u8, f.name, head.text)) {
            form = f;
            break;
        }
    }
    const f = form orelse return AsmError.UnknownMnemonic;
    const ops = isa.operandsFor(f.name);
    if (ops.len == 0) return AsmError.UnknownMnemonic; // no operand table yet

    // Walk operands in file order, skipping one comma between each and an
    // optional `#` in front of each immediate.
    var word: u32 = f.value;
    for (ops, 0..) |op, i| {
        if (i > 0) {
            const comma = try lex.next();
            if (comma.kind != .comma) return AsmError.WrongOperandCount;
        }
        var tok = try lex.next();
        if (tok.kind == .hash) tok = try lex.next();
        const field = try matchOperand(op, tok);
        word |= op.encode(field);
    }

    const tail = try lex.next();
    if (tail.kind != .end) return AsmError.WrongOperandCount;
    return word;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "assemble rli R3, s5, #0x1234" {
    const word = try assembleOne("rli R3, s5, #0x1234");
    // dest_R=3 << 27 = XXXXXXXX
    // dest_s=5 << 21 = XXXXXXXX
    // imm 0x1234 << 5 = 0x00024680
    // base value for rli = 0x0200000C
    // OR: 0x1AA2468C
    try testing.expectEqual(@as(u32, 0x1AA2_468C), word);
}

test "assemble ldi s0, #0x1234 maps dest to k=16" {
    const word = try assembleOne("ldi s0, #0x1234");
    // dest = 16 (s0 -> k=16) at bits [25:21] = 0x02000000
    // imm 0x1234 at [20:5] = 0x00024680
    // base value for ldi = 0xC000000B
    // OR: 0xC202468B
    try testing.expectEqual(@as(u32, 0xC202_468B), word);
}

test "assemble ldi s15 maps dest to k=31" {
    const word = try assembleOne("ldi s15, #0");
    // dest = 31 at [25:21] = 31 << 21 = 0x03E00000
    try testing.expectEqual(@as(u32, 0xC000_000B) | (31 << 21), word);
}

test "unknown mnemonic errors" {
    try testing.expectError(AsmError.UnknownMnemonic, assembleOne("nope s0, #0"));
}

test "wrong operand kind errors" {
    try testing.expectError(AsmError.UnknownOperand, assembleOne("rli s3, s5, #0x1234"));
}

test "imm overflow errors" {
    // rli's imm16 is 16 bits; 0x1_0000 overflows.
    try testing.expectError(AsmError.OperandOutOfRange, assembleOne("rli R0, s0, #0x10000"));
}

test "s-reg index out of range errors" {
    try testing.expectError(AsmError.OperandOutOfRange, assembleOne("rli R0, s16, #0"));
}

test "comment to end of line is tolerated" {
    const word = try assembleOne("ldi s0, #0 ; load zero into source k=16");
    try testing.expectEqual(@as(u32, 0xC000_000B) | (16 << 21), word);
}

test "empty input errors" {
    try testing.expectError(AsmError.EmptyInput, assembleOne("   "));
}

test "missing operand errors" {
    try testing.expectError(AsmError.WrongOperandCount, assembleOne("rli R0, s0"));
}

test "extra trailing token errors" {
    try testing.expectError(AsmError.WrongOperandCount, assembleOne("ldi s0, #0 extra"));
}
