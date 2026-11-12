//! x17 assembler.
//!
//! One line, one bundle.  A file assembles in two passes, a816-style:
//!   1. size every bundle and bind labels (`name:`) and constants
//!      (`.const NAME = expr`) to a flat symbol table;
//!   2. encode with every symbol known, check `.assert expr`.
//! Symbols resolve from that one table at the end, never during emit, and
//! every diagnostic carries a span: a later link step can take the symbols
//! a file leaves unresolved without a rewrite (ARCHITECTURE.md).
//!
//! Operands match by file order against `isa.operandsFor(form)`.  Immediates
//! are constant expressions (`+ - * / << >> & | ~`, parentheses, symbols).
//! Branch offsets (operand kinds rel / relu / rel4) take a bare label, the
//! target, which the assembler turns into the form's offset; `#expr` stays
//! the raw field, so measured gate programs keep their exact bytes.
//!
//! Errors are `Diagnostic`s: code, span, expected/found message, hint,
//! printed as a rust-style caret block.
//!
//! `.include` and macros expand before pass 1 (expand.zig).
//!
//! Placement and images (`assembleProgram`, image.zig): `.pass init` /
//! `.pass decoder` open a program at row 0 / row 0x40, `.org row` moves
//! forward to a row, and `.xdata` / `.ydata` / `.param` carry a pass's host
//! data.  The flat entry (`assembleFile*`) takes `.org` but refuses the rest.
//!
//! What waits: `.assert cycles`.

const std = @import("std");
const isa = @import("isa");
const expand = @import("expand.zig");

pub const AsmError = error{
    AssemblyFailed,
};

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/// Byte range into the original source.  Inclusive `start`, exclusive `end`.
/// A zero-length span (`end == start`) anchors a caret under a single
/// column, which we use for "unexpected end-of-input" cases.
pub const Span = struct {
    start: usize,
    end: usize,

    pub fn len(self: Span) usize {
        return self.end - self.start;
    }
};

pub const ErrorCode = enum {
    e0001_unknown_mnemonic,
    e0002_empty_input,
    e0003_bad_token,
    e0004_number_overflow,
    e0005_missing_operand,
    e0006_missing_comma,
    e0007_extra_tokens,
    e0008_expected_scalar_reg,
    e0009_expected_element_reg,
    e0010_expected_r_reg,
    e0011_expected_predicate,
    e0012_expected_immediate,
    e0013_imm_out_of_range,
    e0014_reg_index_out_of_range,
    e0015_ldi_dest_not_s_reg,
    e0016_expected_y_prefix,
    e0017_expected_colon,
    e0018_undefined_symbol,
    e0019_duplicate_symbol,
    e0020_bad_expression,
    e0021_branch_misaligned,
    e0022_branch_out_of_range,
    e0023_bad_directive,
    e0024_assert_failed,
    e0025_include_failed,
    e0026_include_cycle,
    e0027_bad_macro,
    e0028_macro_call,
    e0029_form_collision,
    e0030_bad_placement,
    e0031_data_range,

    pub fn text(c: ErrorCode) []const u8 {
        return switch (c) {
            .e0001_unknown_mnemonic => "E0001",
            .e0002_empty_input => "E0002",
            .e0003_bad_token => "E0003",
            .e0004_number_overflow => "E0004",
            .e0005_missing_operand => "E0005",
            .e0006_missing_comma => "E0006",
            .e0007_extra_tokens => "E0007",
            .e0008_expected_scalar_reg => "E0008",
            .e0009_expected_element_reg => "E0009",
            .e0010_expected_r_reg => "E0010",
            .e0011_expected_predicate => "E0011",
            .e0012_expected_immediate => "E0012",
            .e0013_imm_out_of_range => "E0013",
            .e0014_reg_index_out_of_range => "E0014",
            .e0015_ldi_dest_not_s_reg => "E0015",
            .e0016_expected_y_prefix => "E0016",
            .e0017_expected_colon => "E0017",
            .e0018_undefined_symbol => "E0018",
            .e0019_duplicate_symbol => "E0019",
            .e0020_bad_expression => "E0020",
            .e0021_branch_misaligned => "E0021",
            .e0022_branch_out_of_range => "E0022",
            .e0023_bad_directive => "E0023",
            .e0024_assert_failed => "E0024",
            .e0025_include_failed => "E0025",
            .e0026_include_cycle => "E0026",
            .e0027_bad_macro => "E0027",
            .e0028_macro_call => "E0028",
            .e0029_form_collision => "E0029",
            .e0030_bad_placement => "E0030",
            .e0031_data_range => "E0031",
        };
    }
};

/// A single error with enough context to print a rust-style block:
///   error[E0008]: expected scalar register, found R register
///     --> input:1:5
///      |
///    1 | rli s3, s5, #0x1234
///      |     ^^ help: scalar registers are written s0..s15
pub const Diagnostic = struct {
    code: ErrorCode,
    span: Span,
    /// Short message after the code, e.g. "expected scalar register, found R register".
    msg_buf: [128]u8 = undefined,
    msg_len: u8 = 0,
    /// Optional caret-line hint, e.g. "scalar registers are written s0..s15".
    hint_buf: [96]u8 = undefined,
    hint_len: u8 = 0,

    pub fn msg(self: *const Diagnostic) []const u8 {
        return self.msg_buf[0..self.msg_len];
    }

    pub fn hint(self: *const Diagnostic) []const u8 {
        return self.hint_buf[0..self.hint_len];
    }

    pub fn setMsg(self: *Diagnostic, bytes: []const u8) void {
        const n = @min(bytes.len, self.msg_buf.len);
        @memcpy(self.msg_buf[0..n], bytes[0..n]);
        self.msg_len = @intCast(n);
    }

    pub fn setHint(self: *Diagnostic, bytes: []const u8) void {
        const n = @min(bytes.len, self.hint_buf.len);
        @memcpy(self.hint_buf[0..n], bytes[0..n]);
        self.hint_len = @intCast(n);
    }
};

pub const MAX_BUNDLE_SYLLABLES: usize = isa.MAX_BUNDLE_SYLLABLES;

/// A bundle: 1-7 syllables of 32 bits each, written to consecutive slots
/// starting at the bundle's head position.  `len` says how many of `words`
/// are valid; the rest are undefined.
pub const Bundle = struct {
    words: [MAX_BUNDLE_SYLLABLES]u32,
    len: u8,

    pub fn slice(self: *const Bundle) []const u32 {
        return self.words[0..self.len];
    }
};

pub const Result = union(enum) {
    ok: Bundle,
    err: Diagnostic,
};

fn oneWord(w: u32) Bundle {
    var b: Bundle = .{ .words = undefined, .len = 1 };
    b.words[0] = w;
    return b;
}

/// Column index (1-based) of `offset` within its line.  Lines are single
/// here (we only assemble one line), so this is `offset + 1`; the helper
/// exists so the multi-line parser gets the same shape later.
fn columnOf(source: []const u8, offset: usize) usize {
    _ = source;
    return offset + 1;
}

/// Render a Diagnostic into `out`, returning the number of bytes written.
/// Writes a rust-style block:
///
///   error[E0008]: expected scalar register, found R register
///     --> <filename>:1:5
///      |
///    1 | rli s3, s5, #0x1234
///      |     ^^ help: scalar registers are s0..s15
///
/// Truncates silently if `out` is too small (returns `out.len`); call with
/// a 2 KiB buffer to be safe.
pub fn renderDiag(out: []u8, filename: []const u8, source: []const u8, d: Diagnostic) usize {
    return renderDiagAt(out, filename, 1, source, d);
}

/// Like `renderDiag` but with a caller-supplied line number, for multi-line
/// source files where this `source` is the single offending line.
pub fn renderDiagAt(out: []u8, filename: []const u8, line_no: usize, source: []const u8, d: Diagnostic) usize {
    var cur: usize = 0;
    const col = columnOf(source, d.span.start);
    cur += (std.fmt.bufPrint(out[cur..], "error[{s}]: {s}\n", .{ d.code.text(), d.msg() }) catch return cur).len;
    cur += (std.fmt.bufPrint(out[cur..], "  --> {s}:{d}:{d}\n", .{ filename, line_no, col }) catch return cur).len;
    cur += (std.fmt.bufPrint(out[cur..], "   |\n", .{}) catch return cur).len;
    cur += (std.fmt.bufPrint(out[cur..], " {d} | {s}\n", .{ line_no, source }) catch return cur).len;
    cur += (std.fmt.bufPrint(out[cur..], "   | ", .{}) catch return cur).len;
    var k: usize = 0;
    while (k < col - 1 and cur < out.len) : (k += 1) {
        out[cur] = ' ';
        cur += 1;
    }
    const caret_count = @max(@as(usize, 1), d.span.len());
    var j: usize = 0;
    while (j < caret_count and cur < out.len) : (j += 1) {
        out[cur] = '^';
        cur += 1;
    }
    if (d.hint_len > 0) {
        cur += (std.fmt.bufPrint(out[cur..], " help: {s}\n", .{d.hint()}) catch return cur).len;
    } else if (cur < out.len) {
        out[cur] = '\n';
        cur += 1;
    }
    return cur;
}

// ---------------------------------------------------------------------------
// Lexer
// ---------------------------------------------------------------------------

pub const TokenKind = enum {
    ident,
    s_reg,
    e_reg,
    r_reg,
    a_reg,
    p_reg,
    number,
    comma,
    hash,
    colon,
    lparen,
    rparen,
    plus,
    minus,
    star,
    slash,
    shl,
    shr,
    amp,
    pipe,
    tilde,
    equals,
    eq,
    ne,
    lt,
    le,
    gt,
    ge,
    end,
};

pub const Token = struct {
    kind: TokenKind,
    text: []const u8,
    value: u32 = 0,
    start: usize, // byte offset into source
};

const LexError = struct {
    code: ErrorCode,
    span: Span,
    msg: []const u8,
    hint: []const u8 = "",
};

const LexResult = union(enum) {
    tok: Token,
    err: LexError,
};

const Lexer = struct {
    src: []const u8,
    i: usize = 0,

    fn skipWs(self: *Lexer) void {
        while (self.i < self.src.len) {
            const c = self.src[self.i];
            if (c == ' ' or c == '\t' or c == '\r' or c == '\n') {
                self.i += 1;
            } else if (c == ';') {
                while (self.i < self.src.len and self.src[self.i] != '\n') : (self.i += 1) {}
            } else {
                return;
            }
        }
    }

    fn next(self: *Lexer) LexResult {
        self.skipWs();
        if (self.i >= self.src.len) {
            return .{ .tok = .{ .kind = .end, .text = self.src[self.src.len..self.src.len], .start = self.src.len } };
        }

        const start = self.i;
        const c = self.src[start];

        if (c == ',') {
            self.i += 1;
            return .{ .tok = .{ .kind = .comma, .text = self.src[start..self.i], .start = start } };
        }
        if (c == ':') {
            self.i += 1;
            return .{ .tok = .{ .kind = .colon, .text = self.src[start..self.i], .start = start } };
        }
        if (c == '#') {
            self.i += 1;
            return .{ .tok = .{ .kind = .hash, .text = self.src[start..self.i], .start = start } };
        }
        if (isDigit(c)) return self.number(start);
        if (isAlpha(c)) return self.wordOrReg(start);

        const two = self.i + 1 < self.src.len;
        if (two and c == '<' and self.src[self.i + 1] == '<') return self.punct(start, 2, .shl);
        if (two and c == '>' and self.src[self.i + 1] == '>') return self.punct(start, 2, .shr);
        if (two and self.src[self.i + 1] == '=') {
            switch (c) {
                '=' => return self.punct(start, 2, .eq),
                '!' => return self.punct(start, 2, .ne),
                '<' => return self.punct(start, 2, .le),
                '>' => return self.punct(start, 2, .ge),
                else => {},
            }
        }
        if (c == '<') return self.punct(start, 1, .lt);
        if (c == '>') return self.punct(start, 1, .gt);
        const single: ?TokenKind = switch (c) {
            '(' => .lparen,
            ')' => .rparen,
            '+' => .plus,
            '-' => .minus,
            '*' => .star,
            '/' => .slash,
            '&' => .amp,
            '|' => .pipe,
            '~' => .tilde,
            '=' => .equals,
            else => null,
        };
        if (single) |k| return self.punct(start, 1, k);

        self.i += 1;
        return .{ .err = .{
            .code = .e0003_bad_token,
            .span = .{ .start = start, .end = self.i },
            .msg = "unexpected character",
            .hint = "x17 source accepts identifiers, numbers, operators and ',' ':' '#'",
        } };
    }

    fn punct(self: *Lexer, start: usize, n: usize, k: TokenKind) LexResult {
        self.i += n;
        return .{ .tok = .{ .kind = k, .text = self.src[start..self.i], .start = start } };
    }

    fn peek(self: *Lexer) LexResult {
        const save = self.i;
        defer self.i = save;
        return self.next();
    }

    fn number(self: *Lexer, start: usize) LexResult {
        var value: u32 = 0;
        if (self.i + 1 < self.src.len and self.src[self.i] == '0' and (self.src[self.i + 1] == 'x' or self.src[self.i + 1] == 'X')) {
            self.i += 2;
            const digit_start = self.i;
            while (self.i < self.src.len and isHexDigit(self.src[self.i])) : (self.i += 1) {}
            if (self.i == digit_start) {
                return .{ .err = .{
                    .code = .e0003_bad_token,
                    .span = .{ .start = start, .end = self.i },
                    .msg = "hex literal has no digits after 0x",
                } };
            }
            value = std.fmt.parseInt(u32, self.src[digit_start..self.i], 16) catch {
                return .{ .err = .{
                    .code = .e0004_number_overflow,
                    .span = .{ .start = start, .end = self.i },
                    .msg = "hex literal does not fit in 32 bits",
                } };
            };
        } else {
            while (self.i < self.src.len and isDigit(self.src[self.i])) : (self.i += 1) {}
            value = std.fmt.parseInt(u32, self.src[start..self.i], 10) catch {
                return .{ .err = .{
                    .code = .e0004_number_overflow,
                    .span = .{ .start = start, .end = self.i },
                    .msg = "decimal literal does not fit in 32 bits",
                } };
            };
        }
        return .{ .tok = .{ .kind = .number, .text = self.src[start..self.i], .value = value, .start = start } };
    }

    fn wordOrReg(self: *Lexer, start: usize) LexResult {
        while (self.i < self.src.len and (isAlpha(self.src[self.i]) or isDigit(self.src[self.i]) or self.src[self.i] == '.' or self.src[self.i] == '_')) : (self.i += 1) {}
        const text = self.src[start..self.i];

        if (text.len >= 2 and isDigit(text[1])) {
            const prefix = text[0];
            var all_digits = true;
            for (text[1..]) |ch| {
                if (!isDigit(ch)) {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits) {
                const idx = std.fmt.parseInt(u32, text[1..], 10) catch 0;
                switch (prefix) {
                    's' => return .{ .tok = .{ .kind = .s_reg, .text = text, .value = idx, .start = start } },
                    'e' => return .{ .tok = .{ .kind = .e_reg, .text = text, .value = idx, .start = start } },
                    'R' => return .{ .tok = .{ .kind = .r_reg, .text = text, .value = idx, .start = start } },
                    'A' => return .{ .tok = .{ .kind = .a_reg, .text = text, .value = idx, .start = start } },
                    'p' => return .{ .tok = .{ .kind = .p_reg, .text = text, .value = idx, .start = start } },
                    else => {},
                }
            }
        }
        return .{ .tok = .{ .kind = .ident, .text = text, .start = start } };
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

/// If `tok` fits `op`, return the 4..16-bit field value; otherwise set `d`
/// and return null.
fn matchOperand(op: isa.Operand, tok: Token, d: *Diagnostic) ?u32 {
    switch (op.kind) {
        .s => {
            if (tok.kind != .s_reg) {
                d.* = .{ .code = .e0008_expected_scalar_reg, .span = tokSpan(tok) };
                setExpected(d, "scalar register", tok);
                d.setHint("scalar registers are written s0..s15");
                return null;
            }
            if (tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("scalar register index must be 0..15");
                return null;
            }
            return tok.value;
        },
        .e => {
            if (tok.kind != .e_reg) {
                d.* = .{ .code = .e0009_expected_element_reg, .span = tokSpan(tok) };
                setExpected(d, "element register", tok);
                d.setHint("element registers are written e0..e15");
                return null;
            }
            if (tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("element register index must be 0..15");
                return null;
            }
            return tok.value;
        },
        .eh => {
            if (tok.kind != .e_reg) {
                d.* = .{ .code = .e0009_expected_element_reg, .span = tokSpan(tok) };
                setExpected(d, "element register", tok);
                d.setHint("the register this store post-increments, e1..e15");
                return null;
            }
            if (tok.value == 0 or tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("post-increment target must be e1..e15");
                d.setHint("e0 in this field means no write: use x17.sty");
                return null;
            }
            return tok.value;
        },
        .e8 => {
            if (tok.kind != .e_reg) {
                d.* = .{ .code = .e0009_expected_element_reg, .span = tokSpan(tok) };
                setExpected(d, "element register e8..e15", tok);
                d.setHint("this form's e field has its top bit fixed: only e8..e15 fit");
                return null;
            }
            if (tok.value < 8 or tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("this form takes e8..e15 only");
                d.setHint("the form's key fixes the e field's top bit at 1");
                return null;
            }
            return tok.value - 8;
        },
        .r_reg => {
            if (tok.kind != .r_reg) {
                d.* = .{ .code = .e0010_expected_r_reg, .span = tokSpan(tok) };
                setExpected(d, "R register", tok);
                d.setHint("R-file registers are written R0..R15 (capital R)");
                return null;
            }
            if (tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("R register index must be 0..15");
                return null;
            }
            return tok.value;
        },
        .a_reg => {
            if (tok.kind != .a_reg) {
                d.* = .{ .code = .e0010_expected_r_reg, .span = tokSpan(tok) };
                setExpected(d, "A register", tok);
                d.setHint("address registers are written A0..A31 (capital A)");
                return null;
            }
            // The A file has 32 registers; a 4-bit field reaches A0..A15 only.
            const a_max: u32 = if (op.width() >= 5) 31 else 15;
            if (tok.value > a_max) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg(if (a_max == 31) "A register index must be 0..31" else "this form's A field reaches A0..A15 only");
                return null;
            }
            return tok.value;
        },
        .p => {
            if (tok.kind != .p_reg) {
                d.* = .{ .code = .e0011_expected_predicate, .span = tokSpan(tok) };
                setExpected(d, "predicate", tok);
                d.setHint("predicates are written p0..p15 (p0 is always true)");
                return null;
            }
            return tok.value;
        },
        .imm => {
            if (tok.kind != .number) {
                d.* = .{ .code = .e0012_expected_immediate, .span = tokSpan(tok) };
                setExpected(d, "immediate", tok);
                d.setHint("prefix immediates with '#', e.g. #0x1234 or #42");
                return null;
            }
            const w = op.width();
            const max: u32 = if (w == 32) 0xFFFF_FFFF else (@as(u32, 1) << @as(u5, @intCast(w))) - 1;
            if (tok.value > max) {
                d.* = .{ .code = .e0013_imm_out_of_range, .span = tokSpan(tok) };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "immediate 0x{x} does not fit in {d} bits", .{ tok.value, w }) catch buf[0..0];
                d.setMsg(m);
                return null;
            }
            return tok.value;
        },
        .y_addr, .rel, .relu, .rel4, .a4 => unreachable, // handled inline in the operand loop
        .dest_ldi => {
            if (tok.kind != .s_reg) {
                d.* = .{ .code = .e0015_ldi_dest_not_s_reg, .span = tokSpan(tok) };
                d.setMsg("ldi destination must be a scalar register");
                d.setHint("sN maps to source index k = N + 16 in isa.toml");
                return null;
            }
            if (tok.value > 15) {
                d.* = .{ .code = .e0014_reg_index_out_of_range, .span = tokSpan(tok) };
                d.setMsg("scalar register index must be 0..15");
                return null;
            }
            return tok.value + 16;
        },
    }
}

fn tokSpan(t: Token) Span {
    return .{ .start = t.start, .end = t.start + t.text.len };
}

fn kindName(k: TokenKind) []const u8 {
    return switch (k) {
        .ident => "identifier",
        .s_reg => "scalar register",
        .e_reg => "element register",
        .r_reg => "R register",
        .a_reg => "A register",
        .p_reg => "predicate",
        .number => "number",
        .comma => "','",
        .hash => "'#'",
        .colon => "':'",
        .lparen => "'('",
        .rparen => "')'",
        .plus => "'+'",
        .minus => "'-'",
        .star => "'*'",
        .slash => "'/'",
        .shl => "'<<'",
        .shr => "'>>'",
        .amp => "'&'",
        .pipe => "'|'",
        .tilde => "'~'",
        .equals => "'='",
        .eq => "'=='",
        .ne => "'!='",
        .lt => "'<'",
        .le => "'<='",
        .gt => "'>'",
        .ge => "'>='",
        .end => "end of input",
    };
}

fn setExpected(d: *Diagnostic, want: []const u8, tok: Token) void {
    var buf: [128]u8 = undefined;
    const m = std.fmt.bufPrint(&buf, "expected {s}, found {s}", .{ want, kindName(tok.kind) }) catch buf[0..0];
    d.setMsg(m);
}

// ---------------------------------------------------------------------------
// Symbols and constant expressions
// ---------------------------------------------------------------------------

pub const SymbolTable = std.StringHashMapUnmanaged(i64);

/// What a line assembles against: the flat symbol table, the bundle's own
/// position (q, in syllables), and whether this is the sizing pass, where an
/// unknown symbol reads as 0 and no range is checked.
pub const Env = struct {
    symbols: *const SymbolTable,
    pc: u32 = 0,
    sizing: bool = false,
    /// An image build: `.pass`, `.xdata`, `.ydata` and `.param` are allowed.
    image: bool = false,
    /// `.mode N` when given: `.ydata` must sit in that mode's 16-bit Y range (fact 960).
    mode: ?u8 = null,
};

const Value = struct {
    v: i64,
    span: Span,
};

const ExprResult = union(enum) {
    ok: Value,
    err: Result,
};

fn diagAt(code: ErrorCode, span: Span, msg: []const u8, hint: []const u8) Result {
    var d = Diagnostic{ .code = code, .span = span };
    d.setMsg(msg);
    if (hint.len > 0) d.setHint(hint);
    return .{ .err = d };
}

fn nextTok(lex: *Lexer) union(enum) { tok: Token, err: Result } {
    return switch (lex.next()) {
        .tok => |t| .{ .tok = t },
        .err => |e| .{ .err = fromLexErr(e) },
    };
}

fn peekKind(lex: *Lexer) TokenKind {
    return switch (lex.peek()) {
        .tok => |t| t.kind,
        .err => .end,
    };
}

/// Binary levels, loosest first: comparisons (0 or 1), | & (<< >>) (+ -) (* /).
const levels = [_][]const TokenKind{
    &.{ .eq, .ne, .lt, .le, .gt, .ge },
    &.{.pipe},
    &.{.amp},
    &.{ .shl, .shr },
    &.{ .plus, .minus },
    &.{ .star, .slash },
};

/// expr := the binary levels above over unary := ('-' | '~') unary | primary,
/// primary := number | symbol | '(' expr ')'.  64-bit, wrapping.
fn parseExpr(lex: *Lexer, env: ?*const Env) ExprResult {
    return parseLevel(lex, env, 0);
}

fn parseLevel(lex: *Lexer, env: ?*const Env, level: usize) ExprResult {
    if (level == levels.len) return parseUnary(lex, env);
    var lhs = switch (parseLevel(lex, env, level + 1)) {
        .ok => |v| v,
        .err => |r| return .{ .err = r },
    };
    while (std.mem.indexOfScalar(TokenKind, levels[level], peekKind(lex)) != null) {
        const op = switch (nextTok(lex)) {
            .tok => |t| t,
            .err => |r| return .{ .err = r },
        };
        const rhs = switch (parseLevel(lex, env, level + 1)) {
            .ok => |v| v,
            .err => |r| return .{ .err = r },
        };
        const span: Span = .{ .start = lhs.span.start, .end = rhs.span.end };
        lhs = .{ .span = span, .v = switch (op.kind) {
            .eq => @intFromBool(lhs.v == rhs.v),
            .ne => @intFromBool(lhs.v != rhs.v),
            .lt => @intFromBool(lhs.v < rhs.v),
            .le => @intFromBool(lhs.v <= rhs.v),
            .gt => @intFromBool(lhs.v > rhs.v),
            .ge => @intFromBool(lhs.v >= rhs.v),
            .pipe => lhs.v | rhs.v,
            .amp => lhs.v & rhs.v,
            .plus => lhs.v +% rhs.v,
            .minus => lhs.v -% rhs.v,
            .star => lhs.v *% rhs.v,
            .shl, .shr => blk: {
                if (rhs.v < 0 or rhs.v > 63) return .{ .err = diagAt(.e0020_bad_expression, rhs.span, "shift amount must be 0..63", "") };
                const n: u6 = @intCast(rhs.v);
                break :blk if (op.kind == .shl) lhs.v << n else lhs.v >> n;
            },
            .slash => blk: {
                if (rhs.v == 0) return .{ .err = diagAt(.e0020_bad_expression, rhs.span, "division by zero", "") };
                break :blk @divTrunc(lhs.v, rhs.v);
            },
            else => unreachable,
        } };
    }
    return .{ .ok = lhs };
}

fn parseUnary(lex: *Lexer, env: ?*const Env) ExprResult {
    const t = switch (nextTok(lex)) {
        .tok => |tok| tok,
        .err => |r| return .{ .err = r },
    };
    switch (t.kind) {
        .minus, .tilde => {
            const inner = switch (parseUnary(lex, env)) {
                .ok => |v| v,
                .err => |r| return .{ .err = r },
            };
            const v = if (t.kind == .minus) -%inner.v else ~inner.v;
            return .{ .ok = .{ .v = v, .span = .{ .start = t.start, .end = inner.span.end } } };
        },
        .lparen => {
            const inner = switch (parseExpr(lex, env)) {
                .ok => |v| v,
                .err => |r| return .{ .err = r },
            };
            const close = switch (nextTok(lex)) {
                .tok => |tok| tok,
                .err => |r| return .{ .err = r },
            };
            if (close.kind != .rparen) {
                var d = Diagnostic{ .code = .e0020_bad_expression, .span = tokSpan(close) };
                setExpected(&d, "')'", close);
                d.setHint("every '(' needs its ')'");
                return .{ .err = .{ .err = d } };
            }
            return .{ .ok = .{ .v = inner.v, .span = .{ .start = t.start, .end = close.start + 1 } } };
        },
        .number => return .{ .ok = .{ .v = t.value, .span = tokSpan(t) } },
        .ident => {
            if (env) |e| {
                if (e.symbols.get(t.text)) |v| return .{ .ok = .{ .v = v, .span = tokSpan(t) } };
                if (e.sizing) return .{ .ok = .{ .v = 0, .span = tokSpan(t) } };
            }
            var buf: [128]u8 = undefined;
            const m = std.fmt.bufPrint(&buf, "undefined symbol '{s}'", .{t.text}) catch buf[0..0];
            return .{ .err = diagAt(.e0018_undefined_symbol, tokSpan(t), m, "define it with `name:` or `.const name = value`") };
        },
        else => {
            var d = Diagnostic{ .code = .e0012_expected_immediate, .span = tokSpan(t) };
            setExpected(&d, "a number, symbol or '('", t);
            d.setHint("immediates look like #0x1234, #42 or #(BASE + 4)");
            return .{ .err = .{ .err = d } };
        },
    }
}

const FieldResult = union(enum) {
    ok: u32,
    err: Result,
};

/// A value into a w-bit field: 0..2^w-1, or a negative value down to
/// -2^(w-1) stored as two's complement.
fn immField(val: Value, op: isa.Operand, env: ?*const Env) FieldResult {
    const w: u6 = @intCast(op.width());
    const mask: u64 = (@as(u64, 1) << w) - 1;
    const bits: u32 = @intCast(@as(u64, @bitCast(val.v)) & mask);
    if (env) |e| if (e.sizing) return .{ .ok = bits };
    const lo: i64 = -(@as(i64, 1) << (w - 1));
    if (val.v > @as(i64, @intCast(mask)) or val.v < lo) {
        var buf: [128]u8 = undefined;
        const m = if (val.v < 0)
            std.fmt.bufPrint(&buf, "immediate {d} does not fit in {d} bits", .{ val.v, w }) catch buf[0..0]
        else
            std.fmt.bufPrint(&buf, "immediate 0x{x} does not fit in {d} bits", .{ val.v, w }) catch buf[0..0];
        var hb: [96]u8 = undefined;
        const h = std.fmt.bufPrint(&hb, "{s} takes {d}..0x{x}", .{ op.name, lo, mask }) catch hb[0..0];
        return .{ .err = diagAt(.e0013_imm_out_of_range, val.span, m, h) };
    }
    return .{ .ok = bits };
}

/// A branch target into the form's offset field: (target - q) / scale.
fn relField(target: Value, op: isa.Operand, env: ?*const Env) FieldResult {
    const e = env orelse return .{ .err = diagAt(.e0018_undefined_symbol, target.span, "a branch target needs a file to resolve against", "write the raw offset with '#', or assemble a whole file") };
    if (e.sizing) return .{ .ok = 0 };
    const w: u6 = @intCast(op.width());
    const scale: i64 = if (op.kind == .rel4) 4 else 1;
    const d = target.v - @as(i64, e.pc);
    if (@rem(d, scale) != 0) {
        var buf: [128]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "target is {d} syllables away; this branch reaches multiples of 4", .{d}) catch buf[0..0];
        return .{ .err = diagAt(.e0021_branch_misaligned, target.span, m, "move the label or the branch by the remainder") };
    }
    const off = @divExact(d, scale);
    const lo: i64 = if (op.kind == .relu) 0 else -(@as(i64, 1) << (w - 1));
    const hi: i64 = if (op.kind == .relu) (@as(i64, 1) << w) - 1 else (@as(i64, 1) << (w - 1)) - 1;
    if (off < lo or off > hi) {
        var buf: [128]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "target is {d} syllables away; this branch reaches {d}..{d}", .{ d, lo * scale, hi * scale }) catch buf[0..0];
        const h = if (op.kind == .relu) "this branch only jumps forward" else "";
        return .{ .err = diagAt(.e0022_branch_out_of_range, target.span, m, h) };
    }
    const mask: u64 = (@as(u64, 1) << w) - 1;
    return .{ .ok = @intCast(@as(u64, @bitCast(off)) & mask) };
}

/// An immediate or branch operand whose first token `tok` is already read.
fn valueOperand(lex: *Lexer, tok: Token, op: isa.Operand, env: ?*const Env) FieldResult {
    const raw = tok.kind == .hash;
    if (!raw) lex.i = tok.start; // re-read the expression from its first token
    const val = switch (parseExpr(lex, env)) {
        .ok => |v| v,
        .err => |r| return .{ .err = r },
    };
    if (op.kind.isRel() and !raw) return relField(val, op, env);
    return immField(val, op, env);
}

// ---------------------------------------------------------------------------
// Assemble one line
// ---------------------------------------------------------------------------

/// Assemble one source line.  Carries diagnostics.  No symbols: branch
/// operands take the raw `#offset` form only.
pub fn assembleOneResult(src: []const u8) Result {
    return assembleLine(src, 0, null);
}

/// Assemble the instruction starting at byte `start` of the line `src`
/// (after any label), against `env`.  Spans stay line-relative.
pub fn assembleLine(src: []const u8, start: usize, env: ?*const Env) Result {
    var lex = Lexer{ .src = src, .i = start };
    const head_r = lex.next();
    const head = switch (head_r) {
        .tok => |t| t,
        .err => |e| return fromLexErr(e),
    };
    if (head.kind == .end) {
        var d = Diagnostic{ .code = .e0002_empty_input, .span = .{ .start = 0, .end = 0 } };
        d.setMsg("no instruction on this line");
        return .{ .err = d };
    }
    if (head.kind != .ident) {
        var d = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(head) };
        d.setMsg("expected a mnemonic at the start of the line");
        return .{ .err = d };
    }

    // Multi-syllable bundles all route through the same data-driven packer
    // in assembleX17.  The sub-form name (x17.e, x19.ld, alu11, ...) picks
    // the base words + operand table; the "x17/x19" prefix is not special.
    if (isa.subFormByName(head.text) != null or isa.stopBase(head.text) != null) {
        return assembleX17(head, &lex, src, env);
    }

    var form: ?*const isa.Form = null;
    for (isa.forms) |*f| {
        if (std.mem.eql(u8, f.name, head.text)) {
            form = f;
            break;
        }
    }
    const f = form orelse {
        var d = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(head) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "unknown mnemonic '{s}'", .{head.text}) catch buf[0..0];
        d.setMsg(m);
        return .{ .err = d };
    };
    const ops = isa.operandsFor(f.name);
    if (ops.len == 0) {
        var d = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(head) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "mnemonic '{s}' has no operand table yet", .{head.text}) catch buf[0..0];
        d.setMsg(m);
        return .{ .err = d };
    }

    var word: u32 = f.value;
    for (ops, 0..) |op, i| {
        if (i > 0) {
            const cr = lex.next();
            const ct = switch (cr) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
            if (ct.kind == .end) {
                var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "missing operand {d}/{d} ({s})", .{ i + 1, ops.len, op.name }) catch buf[0..0];
                d.setMsg(m);
                return .{ .err = d };
            }
            if (ct.kind != .comma) {
                var d = Diagnostic{ .code = .e0006_missing_comma, .span = tokSpan(ct) };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "expected ',' between operands, found '{s}'", .{ct.text}) catch buf[0..0];
                d.setMsg(m);
                return .{ .err = d };
            }
        }
        const tr = lex.next();
        var tok = switch (tr) {
            .tok => |t| t,
            .err => |e| return fromLexErr(e),
        };
        if (tok.kind == .end) {
            var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
            var buf: [96]u8 = undefined;
            const m = std.fmt.bufPrint(&buf, "missing operand {d}/{d} ({s})", .{ i + 1, ops.len, op.name }) catch buf[0..0];
            d.setMsg(m);
            return .{ .err = d };
        }

        // y_addr consumes three tokens: `Y` `:` `<number>` (optional `#`).
        if (op.kind == .y_addr) {
            switch (parseYAddr(&lex, tok, op)) {
                .ok => |v| word = op.place(word, v),
                .err => |r| return r,
            }
            continue;
        }

        if (op.kind == .imm or op.kind.isRel()) {
            if (tok.kind == .hash and peekKind(&lex) == .end) {
                var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
                d.setMsg("'#' is not followed by a number");
                return .{ .err = d };
            }
            switch (valueOperand(&lex, tok, op, env)) {
                .ok => |v| word = op.place(word, v),
                .err => |r| return r,
            }
            continue;
        }
        if (tok.kind == .hash) {
            tok = switch (lex.next()) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
        }
        var d: Diagnostic = undefined;
        const field = matchOperand(op, tok, &d) orelse return .{ .err = d };
        word = op.place(word, field);
    }

    const tail_r = lex.next();
    const tail = switch (tail_r) {
        .tok => |t| t,
        .err => |e| return fromLexErr(e),
    };
    if (tail.kind != .end) {
        var d = Diagnostic{ .code = .e0007_extra_tokens, .span = tokSpan(tail) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "unexpected token '{s}' after operands", .{tail.text}) catch buf[0..0];
        d.setMsg(m);
        return .{ .err = d };
    }
    // A one-word form whose low5 makes a longer bundle (st.y at 0x0F is 2)
    // gets zero syllables after it, the no-op second the engine reads.
    // st (0x0F class 001) stays one word: its second is only a paired wr
    // (fact 21: three back-to-back stores each run, one per lane).
    var b = oneWord(word);
    const is_st = word & 0x1F == 0x0F and word >> 29 == 1;
    const n = if (is_st) 1 else isa.bundleLen(word) orelse 1;
    if (n > 1 and n <= 2) {
        b.words[1] = 0;
        b.len = n;
    }
    return .{ .ok = b };
}

/// Assemble an x17.* bundle.  Head's text is `x17.<sub>`; the lexer is
/// positioned just after it.  Looks up the sub-form in `isa.subforms`,
/// seeds each syllable with the sub-form's base, then walks the operand
/// rows (which name their syllable via `syll`) and OR's the encoded
/// fields into the matching word.
/// `Y` `:` `<number>` (optional `#`), the y_addr operand; `tok` is the `Y`.
fn parseYAddr(lex: *Lexer, tok: Token, op: isa.Operand) union(enum) { ok: u32, err: Result } {
    if (tok.kind != .ident or !std.mem.eql(u8, tok.text, "Y")) {
        var d = Diagnostic{ .code = .e0016_expected_y_prefix, .span = tokSpan(tok) };
        setExpected(&d, "'Y'", tok);
        d.setHint("Y-space addresses look like Y:1 or Y:#0x1234");
        return .{ .err = .{ .err = d } };
    }
    const cr = lex.next();
    const colon = switch (cr) {
        .tok => |t| t,
        .err => |e| return .{ .err = fromLexErr(e) },
    };
    if (colon.kind != .colon) {
        var d = Diagnostic{ .code = .e0017_expected_colon, .span = tokSpan(colon) };
        setExpected(&d, "':' after 'Y'", colon);
        return .{ .err = .{ .err = d } };
    }
    var nr = lex.next();
    var ntok = switch (nr) {
        .tok => |t| t,
        .err => |e| return .{ .err = fromLexErr(e) },
    };
    if (ntok.kind == .hash) {
        nr = lex.next();
        ntok = switch (nr) {
            .tok => |t| t,
            .err => |e| return .{ .err = fromLexErr(e) },
        };
    }
    if (ntok.kind != .number) {
        var d = Diagnostic{ .code = .e0012_expected_immediate, .span = tokSpan(ntok) };
        setExpected(&d, "Y-space address immediate", ntok);
        d.setHint("Y:<N> or Y:#<N>; N is a 16-bit value");
        return .{ .err = .{ .err = d } };
    }
    const w = op.width();
    const max: u32 = (@as(u32, 1) << @as(u5, @intCast(w))) - 1;
    if (ntok.value > max) {
        var d = Diagnostic{ .code = .e0013_imm_out_of_range, .span = tokSpan(ntok) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "Y-space address 0x{x} does not fit in {d} bits", .{ ntok.value, w }) catch buf[0..0];
        d.setMsg(m);
        return .{ .err = .{ .err = d } };
    }
    return .{ .ok = ntok.value };
}

fn assembleX17(head: Token, lex: *Lexer, src: []const u8, env: ?*const Env) Result {
    // `<x17 form>.stop`: the form's fields with S1[22:20] = 010 (isa.stopBase).
    const stop_base = if (isa.subFormByName(head.text) == null) isa.stopBase(head.text) else null;
    const sf = stop_base orelse isa.subFormByName(head.text) orelse {
        var d = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(head) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "unknown sub-form '{s}'", .{head.text}) catch buf[0..0];
        d.setMsg(m);
        d.setHint("see subforms.txt for the sub-forms this assembler knows");
        return .{ .err = d };
    };

    var b: Bundle = .{ .words = undefined, .len = sf.syllables };
    var si: usize = 0;
    while (si < sf.syllables) : (si += 1) b.words[si] = sf.base[si];

    const ops = isa.operandsFor(sf.name);
    var given: usize = 0;
    var want: usize = 0;
    for (ops) |o| want += @intFromBool(!o.isTie() and !o.isOptional());
    for (ops) |op| {
        // Tie rows take no token; they are filled from their source after the loop.
        if (op.isTie()) continue;
        // An optional row (trailing) left out keeps its field 0.
        if (op.isOptional() and peekKind(lex) == .end) break;
        defer given += 1;
        if (given > 0) {
            const cr = lex.next();
            const ct = switch (cr) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
            if (ct.kind == .end) {
                var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "missing operand {d}/{d} ({s})", .{ given + 1, want, op.name }) catch buf[0..0];
                d.setMsg(m);
                return .{ .err = d };
            }
            if (ct.kind != .comma) {
                var d = Diagnostic{ .code = .e0006_missing_comma, .span = tokSpan(ct) };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "expected ',' between operands, found '{s}'", .{ct.text}) catch buf[0..0];
                d.setMsg(m);
                return .{ .err = d };
            }
        }
        var tok = switch (lex.next()) {
            .tok => |t| t,
            .err => |e| return fromLexErr(e),
        };
        if (tok.kind == .end) {
            var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
            var buf: [96]u8 = undefined;
            const m = std.fmt.bufPrint(&buf, "missing operand {d}/{d} ({s})", .{ given + 1, want, op.name }) catch buf[0..0];
            d.setMsg(m);
            return .{ .err = d };
        }
        if (op.kind == .y_addr) {
            switch (parseYAddr(lex, tok, op)) {
                .ok => |v| b.words[op.syll] = op.place(b.words[op.syll], v),
                .err => |r| return r,
            }
            continue;
        }
        if (op.syll >= sf.syllables) {
            var dx = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(tok) };
            dx.setMsg("operand table names a syllable past the bundle length");
            return .{ .err = dx };
        }
        if (op.kind == .a4) {
            if (tok.kind != .hash) lex.i = tok.start;
            switch (parseExpr(lex, env)) {
                .ok => |val| {
                    const sizing = if (env) |e| e.sizing else false;
                    const k: u32 = if (sizing) 1 else if (val.v < 0 or val.v > 0xFFFF) 0xFFFF else @intCast(val.v);
                    if (k == 0 or k >= 31) {
                        var dx = Diagnostic{ .code = .e0014_reg_index_out_of_range, .span = val.span };
                        var mb: [96]u8 = undefined;
                        dx.setMsg(std.fmt.bufPrint(&mb, "A index {d} cannot be read through here", .{k}) catch mb[0..0]);
                        dx.setHint("A1..A30: A0 reads a bus value and A31 junk (fact 649)");
                        return .{ .err = dx };
                    }
                    b.words[op.syll] = op.place(b.words[op.syll], k >> 2);
                    b.words[0] = (b.words[0] & 0x3FFF_FFFF) | ((k & 3) << 30);
                },
                .err => |r| return r,
            }
            continue;
        }
        if (op.kind == .imm or op.kind.isRel()) {
            switch (valueOperand(lex, tok, op, env)) {
                .ok => |v| {
                    if (collides(sf.name, op.name, v)) |d| return .{ .err = d.at(tokSpan(tok)) };
                    b.words[op.syll] = op.place(b.words[op.syll], v);
                },
                .err => |r| return r,
            }
            continue;
        }
        if (tok.kind == .hash) {
            tok = switch (lex.next()) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
        }
        var d: Diagnostic = undefined;
        const field = matchOperand(op, tok, &d) orelse return .{ .err = d };
        if (collides(sf.name, op.name, field)) |c| return .{ .err = c.at(tokSpan(tok)) };
        if (op.syll >= sf.syllables) {
            var dx = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(tok) };
            dx.setMsg("operand table names a syllable past the bundle length");
            return .{ .err = dx };
        }
        if (op.kind == .eh) {
            b.words[op.syll] = op.place(b.words[op.syll], field >> 1);
            b.words[0] = (b.words[0] & 0x7FFF_FFFF) | ((field & 1) << 31);
            continue;
        }
        b.words[op.syll] = op.place(b.words[op.syll], field);
    }
    isa.applyTies(ops, b.words[0..b.len]);
    if (wrapWithoutModulo(ops, b.words[0..b.len])) {
        var d = Diagnostic{ .code = .e0029_form_collision, .span = tokSpan(head) };
        d.setMsg("#k wraps only with fn 6..11; this fn never wraps (fact 963)");
        d.setHint("use the fn's wrapping twin (fn + 6, e.g. #8 for x+r) or drop #k");
        return .{ .err = d };
    }

    const tail = switch (lex.next()) {
        .tok => |t| t,
        .err => |e| return fromLexErr(e),
    };
    if (tail.kind != .end) {
        var d = Diagnostic{ .code = .e0007_extra_tokens, .span = tokSpan(tail) };
        var buf: [96]u8 = undefined;
        const m = std.fmt.bufPrint(&buf, "unexpected token '{s}' after operands", .{tail.text}) catch buf[0..0];
        d.setMsg(m);
        return .{ .err = d };
    }
    if (stop_base != null) b.words[1] |= isa.STOP_ROW;
    return .{ .ok = b };
}

const Collision = struct {
    msg: []const u8,
    hint: []const u8,
    fn at(c: Collision, span: Span) Diagnostic {
        var d = Diagnostic{ .code = .e0029_form_collision, .span = span };
        d.setMsg(c.msg);
        d.setHint(c.hint);
        return d;
    }
};

/// An operand range outside which a sub-form's bundle decodes as a different form: the
/// operand's field shares bits with a unit class that picks the form.
const ClassRule = struct {
    forms: []const []const u8,
    operand: []const u8,
    min: u32,
    max: u32,
    msg: []const u8,
    hint: []const u8,
};

const class_rules = [_]ClassRule{
    // x17.sty needs S1 class != 0 (fact 275); S1[31:29] is src_lo.  Silicon has run sources
    // s1, s9, s5 and s3 only; class 000 there is unmeasured.
    .{ .forms = &.{ "x17.sty", "x17.styi", "x17.stys" }, .operand = "src_lo", .min = 1, .max = 7, .msg = "x17.sty from s0 or s8 makes S1 class 000: x17 decodes that as x17.ld, and silicon has never run it", .hint = "copy to a register not a multiple of 8 first (s1, s3, s5, s9 are measured)" },
    // x16 picks its form from (S1 class, S2 class); these fields overlap S1[31:29].
    .{ .forms = &.{"x16.xld"}, .operand = "dest_s", .min = 6, .max = 7, .msg = "x16.xld's destination is also its S1 class: only s6 and s7 keep class 011, others decode as another x16 form", .hint = "load into s6 or s7, then copy" },
    .{ .forms = &.{"x16.st2"}, .operand = "x_src", .min = 6, .max = 7, .msg = "x16.st2's X source is also its S1 class: only s6 and s7 keep class 011, others decode as another x16 form", .hint = "store from s6 or s7" },
    .{ .forms = &.{"x16.b16"}, .operand = "st_src", .min = 2, .max = 15, .msg = "x16.b16 from s0 or s1 makes S1 class 000: x16 decodes that as its ALU or the stream read", .hint = "store from s2..s15" },
    .{ .forms = &.{"x16.agu2"}, .operand = "fn", .min = 0, .max = 7, .msg = "x16.agu2 fn 8-15 sets S1[29]: class 001 decodes as x16.agumul", .hint = "fn 0..7 only in this form" },
    .{ .forms = &.{"x16.agumul"}, .operand = "dest_lo", .min = 1, .max = 3, .msg = "x16.agumul with dest_lo 0 makes S1 class 000: x16 decodes that as x16.agu2", .hint = "pick a destination whose index is not a multiple of 4" },
    .{ .forms = &.{"x16.mac"}, .operand = "xhi_lo", .min = 1, .max = 1, .msg = "x16.mac's xhi_lo is its S1 class: only 1 runs the MAC, other values decode as another x16 form", .hint = "xhi_lo must be #1" },
};

comptime {
    // Diagnostic buffers: 128-byte message, 96-byte hint.
    for (class_rules) |r| if (r.msg.len > 128 or r.hint.len > 96) @compileError("class rule text too long: " ++ r.msg);
}

/// Operand values that turn a sub-form's bundle into a different form.
/// A nonzero `k` (the wrap register) on a form whose agu_xr fn never wraps: only fn 6..11 do (963).
fn wrapWithoutModulo(ops: []const isa.Operand, words: []const u32) bool {
    var k: u32 = 0;
    for (ops) |o| {
        if (std.mem.eql(u8, o.name, "?k") and o.syll < words.len) k = o.valueIn(words);
    }
    if (k == 0 or words.len < 3) return false;
    const f = (words[2] >> 22) & 0x1F;
    return f < 6 or f > 11;
}

fn collides(form: []const u8, operand: []const u8, v: u32) ?Collision {
    for (class_rules) |r| {
        if (!std.mem.eql(u8, r.operand, operand)) continue;
        for (r.forms) |f| if (std.mem.eql(u8, f, form)) {
            if (v < r.min or v > r.max) return .{ .msg = r.msg, .hint = r.hint };
        };
    }
    return null;
}

fn fromLexErr(e: LexError) Result {
    var d = Diagnostic{ .code = e.code, .span = e.span };
    d.setMsg(e.msg);
    if (e.hint.len > 0) d.setHint(e.hint);
    return .{ .err = d };
}

/// Fixed-capacity text a `FileError` owns, so it outlives the expansion arena.
pub fn Fixed(comptime n: usize) type {
    return struct {
        buf: [n]u8 = undefined,
        len: usize = 0,

        pub fn set(self: *@This(), bytes: []const u8) void {
            self.len = @min(bytes.len, n);
            @memcpy(self.buf[0..self.len], bytes[0..self.len]);
        }

        pub fn get(self: *const @This()) []const u8 {
            return self.buf[0..self.len];
        }
    };
}

/// Error returned by `assembleFile` carrying enough context for a stellar
/// error print: the file and line that failed, the line's text, the
/// diagnostic, and the macro call that produced the line, if any.
pub const FileError = struct {
    line: usize,
    diag: Diagnostic,
    text: Fixed(256) = .{},
    /// Empty for the top-level source when no path was given.
    file: Fixed(256) = .{},
    call_name: Fixed(64) = .{},
    call_file: Fixed(256) = .{},
    /// 0 when the line did not come from a macro.
    call_line: usize = 0,

    pub fn lineText(self: *const FileError) []const u8 {
        return self.text.get();
    }

    fn from(l: expand.Line, d: Diagnostic) FileError {
        var fe: FileError = .{ .line = l.site.line, .diag = d };
        fe.text.set(l.text);
        fe.diag.span.end = @min(fe.diag.span.end, fe.text.len);
        fe.diag.span.start = @min(fe.diag.span.start, fe.diag.span.end);
        fe.file.set(l.site.file);
        if (l.call) |c| {
            fe.call_name.set(c.name);
            fe.call_file.set(c.site.file);
            fe.call_line = c.site.line;
        }
        return fe;
    }
};

pub const Loader = expand.Loader;

pub const FileOptions = struct {
    /// The source's path: names it in diagnostics and anchors `.include`.
    path: []const u8 = "",
    /// Reads included files; null refuses `.include`.
    loader: ?Loader = null,
    /// When set, gets one entry per emitted bundle: where it came from (for profiles).
    sites: ?*std.ArrayList(Site) = null,
};

/// The source of one emitted bundle: its word offset, the line that produced it (inside a
/// macro body, that line), and the macro call it was expanded from, if any.
pub const Site = struct {
    pc: u32,
    file: Fixed(96) = .{},
    line: usize,
    text: Fixed(96) = .{},
    call_file: Fixed(96) = .{},
    call_line: usize = 0,
};

pub const FileResult = union(enum) { ok: []u32, err: FileError };

/// Assemble a source with no path and no includes (see `assembleFileWith`).
pub fn assembleFile(gpa: std.mem.Allocator, source: []const u8) !FileResult {
    return assembleFileWith(gpa, source, .{});
}

/// Assemble a multi-line source file to one flat program at q0: expand
/// `.include` and macros (expand.zig), then two passes (see the file header).
/// Blank lines and lines starting with `;` or `#` are comments.  A line is
/// `[label:] [instruction | directive]`.  Directives: `.const NAME = expr`,
/// `.assert expr` (checked in pass 2), `.org row`.  On success, returns a newly
/// allocated slice of syllables (caller frees).  On failure, returns a
/// `FileError` attached to the offending line.
pub fn assembleFileWith(gpa: std.mem.Allocator, source: []const u8, opts: FileOptions) !FileResult {
    var r = try assembleProgramWith(gpa, source, opts, false);
    switch (r) {
        .err => |fe| return .{ .err = fe },
        .ok => |*prog| {
            defer prog.deinit();
            return .{ .ok = try gpa.dupe(u32, prog.passes[0].syllables) };
        },
    }
}

// ---------------------------------------------------------------------------
// Programs: passes, placement and host data
// ---------------------------------------------------------------------------

/// The layout facts an image build checks (handbook §2, §11).
pub const layout = struct {
    /// Program memory rows (fact 771); a row holds 4 syllables, one per lane.
    pub const p_rows: u32 = 0x1000;
    pub const lanes: u32 = 4;
    /// Zero rows after a program's last row (fact 72).
    pub const tail_rows: u32 = 2;
    /// An init pass loads at row 0, the decoder at row 0x40 = q256 (facts 763, 770).
    pub const init_row: u32 = 0;
    pub const decoder_row: u32 = 0x40;
    /// Host-written X stays below 0x1C00 (fact 653); words are 24-bit.
    pub const x_words: u32 = 0x1C00;
    /// Host-written Y stays below 0x2E80 (fact 760); words are 16-bit (fact 758).
    pub const y_words: u32 = 0x2E80;
    /// The parameter block, Y:0xFD80 up to the status header at Y:0xFD94.
    pub const param_lo: u32 = 0xFD80;
    pub const param_hi: u32 = 0xFD94;

    /// The DSP's Y map per configure mode (fact 960): the descriptor table's 16-bit Y entry and
    /// its window entry, in 0x80-word blocks, plus the parameter block every mode maps.  Modes
    /// without a table are null.
    pub fn yMap(mode: u8) ?[]const [2]u32 {
        const param_block = [2]u32{ 0xFD80, 0xFDFF };
        const S = struct {
            const m0 = [_][2]u32{ .{ 0, 0x2E7F }, .{ 0x2E80, 0x2EFF }, param_block };
            const m1 = [_][2]u32{ .{ 0, 0x177F }, .{ 0x1780, 0x187F }, param_block };
            const m3 = [_][2]u32{ .{ 0, 0x227F }, .{ 0x2400, 0x267F }, param_block };
            const m5 = [_][2]u32{ .{ 0, 0x15FF }, .{ 0x2000, 0x20FF }, param_block };
        };
        return switch (mode) {
            0 => &S.m0,
            1 => &S.m1,
            3 => &S.m3,
            5 => &S.m5,
            else => null,
        };
    }

    /// The DSP's X map per configure mode (fact 961): the descriptor table's entries 2 and 3.
    pub fn xMap(mode: u8) ?[]const [2]u32 {
        const S = struct {
            const m0 = [_][2]u32{.{ 0, 0x1BFF }};
            const m1 = [_][2]u32{.{ 0, 0x11FF }};
            const m3 = [_][2]u32{ .{ 0, 0x1FFF }, .{ 0x2000, 0x20FF } };
            const m5 = [_][2]u32{.{ 0, 0x37F }};
        };
        return switch (mode) {
            0 => &S.m0,
            1 => &S.m1,
            3 => &S.m3,
            5 => &S.m5,
            else => null,
        };
    }

    pub fn yMapped(map: []const [2]u32, addr: u32) bool {
        for (map) |r| if (addr >= r[0] and addr <= r[1]) return true;
        return false;
    }

    /// q order runs column-major over row pairs (fact 45).
    pub fn qOf(row: u32, lane: u32) u32 {
        return 8 * (row / 2) + 2 * lane + row % 2;
    }
    pub fn rowOf(q: u32) u32 {
        return 2 * (q / 8) + q % 2;
    }
    pub fn laneOf(q: u32) u32 {
        return (q % 8) / 2;
    }
    /// The highest row any of q in [base, end) sits in; `end > base`.
    pub fn lastRow(base: u32, end: u32) u32 {
        var hi: u32 = 0;
        var q = base;
        while (q < end) : (q += 1) hi = @max(hi, rowOf(q));
        return hi;
    }
};

pub const PassKind = enum { init, decoder };
pub const Space = enum { x, y, param };

pub const DataBlock = struct {
    space: Space,
    addr: u32,
    /// X: 24-bit, Y and param: 16-bit, already masked.
    words: []const u32,
    file: []const u8,
    line: usize,
};

pub const Pass = struct {
    kind: PassKind,
    /// The pass's first row: 0 for init, 0x40 for the decoder.
    base_row: u32,
    /// Syllables from q = qOf(base_row, 0), in q order.
    syllables: []const u32,
    data: []const DataBlock,
    sites: []const BundleSite,

    pub fn baseQ(self: *const Pass) u32 {
        return layout.qOf(self.base_row, 0);
    }
    /// The last row the program writes, the two zero rows after it included.
    pub fn endRow(self: *const Pass) u32 {
        const b = self.baseQ();
        return layout.lastRow(b, b + @as(u32, @intCast(self.syllables.len))) + layout.tail_rows;
    }
};

pub const Label = struct { name: []const u8, q: u32, pass: usize };

/// Where one bundle of a pass came from (full strings, unlike the fixed-size `Site`).
pub const BundleSite = struct {
    q: u32,
    len: u32,
    file: []const u8,
    line: usize,
    text: []const u8,
    /// The macro call the line was expanded from, if any.
    call: ?struct { name: []const u8, file: []const u8, line: usize } = null,
};

/// An assembled source: init passes in load order, then at most one decoder.
pub const Program = struct {
    arena: std.heap.ArenaAllocator,
    passes: []const Pass = &.{},
    labels: []const Label = &.{},
    /// `.mode N`: the configure mode the image is built for (fact 960), if given.
    mode: ?u8 = null,

    pub fn deinit(self: *Program) void {
        self.arena.deinit();
    }
};

pub const ProgramResult = union(enum) { ok: Program, err: FileError };

/// Assemble a source as an image: `.pass` blocks, `.org`, and per-pass host data
/// (see the file header).  A source with no `.pass` is one init pass.
pub fn assembleProgram(gpa: std.mem.Allocator, source: []const u8, opts: FileOptions) !ProgramResult {
    return assembleProgramWith(gpa, source, opts, true);
}

const PassBuild = struct {
    kind: PassKind,
    base_row: u32,
    opened_at: ?usize,
    syllables: std.ArrayList(u32) = .empty,
    data: std.ArrayList(DataBlock) = .empty,
    sites: std.ArrayList(BundleSite) = .empty,
    last_line: ?usize = null,

    fn finish(self: *PassBuild) Pass {
        return .{
            .kind = self.kind,
            .base_row = self.base_row,
            .syllables = self.syllables.items,
            .data = self.data.items,
            .sites = self.sites.items,
        };
    }
};

fn passDiag(span: Span, msg: []const u8, hint: []const u8) Diagnostic {
    return diagAt(.e0030_bad_placement, span, msg, hint).err;
}

fn assembleProgramWith(gpa: std.mem.Allocator, source: []const u8, opts: FileOptions, image: bool) !ProgramResult {
    var arena_state = std.heap.ArenaAllocator.init(gpa);
    defer arena_state.deinit();
    var x: expand.Expander = .{ .arena = arena_state.allocator(), .gpa = gpa, .loader = opts.loader };
    x.file(opts.path, source) catch |e| switch (e) {
        error.Expand => return .{ .err = FileError.from(x.failure.?.line, x.failure.?.diag) },
        error.OutOfMemory => return error.OutOfMemory,
    };

    var symbols: SymbolTable = .empty;
    defer symbols.deinit(gpa);
    var prog: Program = .{ .arena = std.heap.ArenaAllocator.init(gpa) };
    errdefer prog.deinit();

    for ([_]bool{ true, false }) |sizing| {
        var b: Builder = .{ .gpa = gpa, .pa = prog.arena.allocator(), .sizing = sizing, .opts = opts, .lines = x.out.items };
        if (try b.run(&symbols, image)) |fail| {
            prog.deinit();
            return .{ .err = fail };
        }
        if (!sizing) {
            prog.passes = b.passes.items;
            prog.labels = b.labels.items;
            prog.mode = b.mode;
        }
    }
    return .{ .ok = prog };
}

/// One walk over the expanded lines (either pass): tracks the open pass, its q, and
/// collects syllables, data and labels on the second walk.
const Builder = struct {
    gpa: std.mem.Allocator,
    pa: std.mem.Allocator,
    sizing: bool,
    opts: FileOptions,
    lines: []const expand.Line,
    passes: std.ArrayList(Pass) = .empty,
    labels: std.ArrayList(Label) = .empty,
    cur: PassBuild = .{ .kind = .init, .base_row = 0, .opened_at = null },
    pc: u32 = 0,
    explicit: bool = false,
    has_decoder: bool = false,
    implicit_used: bool = false,
    /// The line of each init pass's last bundle once its end row reaches the decoder's.
    init_overlap: ?usize = null,
    /// `.mode N`, which must come before any pass, bundle or data.
    mode: ?u8 = null,

    fn fail(self: *Builder, at: usize, d: Diagnostic) FileError {
        return FileError.from(self.lines[at], d);
    }

    fn run(self: *Builder, symbols: *SymbolTable, image: bool) !?FileError {
        for (self.lines, 0..) |l, i| {
            const env: Env = .{ .symbols = symbols, .pc = self.pc, .sizing = self.sizing, .image = image, .mode = self.mode };
            var label: ?[]const u8 = null;
            const r = try fileLine(self.gpa, l.text, symbols, env, &label);
            if (label) |name| {
                if (!self.sizing) try self.labels.append(self.pa, .{ .name = try self.pa.dupe(u8, name), .q = self.pc, .pass = self.passes.items.len });
            }
            const fe: ?FileError = switch (r) {
                .err => |d| self.fail(i, d),
                .none => null,
                .bundle => |b| try self.bundle(l, i, b),
                .pass => |p| try self.open(i, p.kind, p.span),
                .org => |o| try self.org(i, o.row, o.span),
                .mode => |m| self.setMode(i, m.mode, m.span),
                .data => |blk| try self.addData(l, i, blk),
            };
            if (fe != null) return fe;
        }
        if (try self.close()) |fe| return fe;
        if (self.has_decoder) {
            if (self.init_overlap) |at| return self.fail(at, passDiag(.{ .start = 0, .end = 0 }, "an init pass reaches row 0x40 with its two zero rows", "init passes stay below the decoder's row 0x40"));
        }
        return null;
    }

    fn bundle(self: *Builder, l: expand.Line, i: usize, b: Bundle) !?FileError {
        if (self.cur.opened_at == null) self.implicit_used = true;
        if (!self.sizing) {
            try self.cur.syllables.appendSlice(self.pa, b.slice());
            const text = std.mem.trim(u8, l.text, " \t");
            var bs: BundleSite = .{ .q = self.pc, .len = @intCast(b.len), .file = try self.pa.dupe(u8, l.site.file), .line = l.site.line, .text = try self.pa.dupe(u8, text) };
            if (l.call) |c| bs.call = .{ .name = try self.pa.dupe(u8, c.name), .file = try self.pa.dupe(u8, c.site.file), .line = c.site.line };
            try self.cur.sites.append(self.pa, bs);
            if (self.opts.sites) |out| {
                var site: Site = .{ .pc = self.pc, .line = l.site.line };
                site.file.set(l.site.file);
                site.text.set(text);
                if (l.call) |c| {
                    site.call_file.set(c.site.file);
                    site.call_line = c.site.line;
                }
                try out.append(self.gpa, site);
            }
        }
        self.cur.last_line = i;
        self.pc += b.len;
        return null;
    }

    fn setMode(self: *Builder, i: usize, mode: u8, span: Span) ?FileError {
        if (self.mode != null or self.explicit or self.implicit_used)
            return self.fail(i, passDiag(span, "'.mode' after a pass, code, data or another '.mode'", "give '.mode' once, before the first '.pass'"));
        self.mode = mode;
        return null;
    }

    /// `.pass kind`: close the open pass, start the next at its base row.
    fn open(self: *Builder, i: usize, kind: PassKind, span: Span) !?FileError {
        if (self.implicit_used and !self.explicit)
            return self.fail(i, passDiag(span, "'.pass' after code or data outside any pass", "put every bundle and data line under a '.pass'"));
        if (self.has_decoder)
            return self.fail(i, passDiag(span, "a pass after the decoder", "init passes come first; the decoder is the last pass"));
        if (self.explicit) {
            if (try self.close()) |fe| return fe;
        }
        self.explicit = true;
        if (kind == .decoder) self.has_decoder = true;
        const row = if (kind == .decoder) layout.decoder_row else layout.init_row;
        self.cur = .{ .kind = kind, .base_row = row, .opened_at = i };
        self.pc = layout.qOf(row, 0);
        return null;
    }

    fn close(self: *Builder) !?FileError {
        const at = self.cur.opened_at orelse 0;
        if (self.cur.last_line == null) {
            if (self.cur.opened_at == null and self.passes.items.len == 0 and self.cur.data.items.len == 0) {
                // An empty flat source: one empty pass.
                if (!self.sizing) try self.passes.append(self.pa, self.cur.finish());
                return null;
            }
            return self.fail(at, passDiag(.{ .start = 0, .end = 0 }, "a pass with no bundles", "every pass is a program; add its code or drop the '.pass'"));
        }
        const last = self.cur.last_line.?;
        const end = layout.lastRow(layout.qOf(self.cur.base_row, 0), self.pc) + layout.tail_rows;
        if (end >= layout.p_rows)
            return self.fail(last, passDiag(.{ .start = 0, .end = 0 }, "the program runs past row 0xFFF with its two zero rows", "program memory is rows 0..0xFFF (fact 771)"));
        if (self.cur.kind == .init and end >= layout.decoder_row and self.init_overlap == null) self.init_overlap = last;
        if (!self.sizing) try self.passes.append(self.pa, self.cur.finish());
        return null;
    }

    fn org(self: *Builder, i: usize, row: u32, span: Span) !?FileError {
        if (row >= layout.p_rows) return self.fail(i, passDiag(span, "'.org' row past 0xFFF", "program memory is rows 0..0xFFF"));
        if (row % 2 != 0) return self.fail(i, passDiag(span, "'.org' to an odd row", "q order runs over row pairs; an odd row starts mid-pair"));
        const q = layout.qOf(row, 0);
        if (q < self.pc) return self.fail(i, passDiag(span, "'.org' moves backwards", "'.org' only moves forward from the current q"));
        if (!self.sizing) try self.cur.syllables.appendNTimes(self.pa, 0, q - self.pc);
        self.pc = q;
        return null;
    }

    fn addData(self: *Builder, l: expand.Line, i: usize, blk: DataLine) !?FileError {
        defer self.gpa.free(blk.words);
        if (self.cur.opened_at == null) self.implicit_used = true;
        if (blk.space == .param and self.cur.kind != .decoder)
            return self.fail(i, diagAt(.e0031_data_range, blk.span, "'.param' outside the decoder pass", "the parameter block is written before the decode arm").err);
        if (self.sizing) return null;
        for (self.cur.data.items) |o| {
            if (o.space == blk.space and blk.addr < o.addr + o.words.len and o.addr < blk.addr + blk.words.len) {
                var buf: [128]u8 = undefined;
                const msg = std.fmt.bufPrint(&buf, "overlaps the block at line {d}", .{o.line}) catch buf[0..0];
                return self.fail(i, diagAt(.e0031_data_range, blk.span, msg, "one pass writes each address once").err);
            }
        }
        try self.cur.data.append(self.pa, .{
            .space = blk.space,
            .addr = blk.addr,
            .words = try self.pa.dupe(u32, blk.words),
            .file = try self.pa.dupe(u8, l.site.file),
            .line = l.site.line,
        });
        return null;
    }
};

const DataLine = struct { space: Space, addr: u32, words: []u32, span: Span };

const LineResult = union(enum) {
    none,
    bundle: Bundle,
    err: Diagnostic,
    pass: struct { kind: PassKind, span: Span },
    org: struct { row: u32, span: Span },
    mode: struct { mode: u8, span: Span },
    /// `words` is gpa-allocated; the builder frees it.
    data: DataLine,
};

fn lineErr(r: Result) LineResult {
    return .{ .err = r.err };
}

/// One file line in either pass.  Pass 1 (env.sizing) binds labels and
/// constants; pass 2 re-evaluates constants with every symbol known and
/// checks asserts.
fn fileLine(gpa: std.mem.Allocator, raw: []const u8, symbols: *SymbolTable, env: Env, label: *?[]const u8) !LineResult {
    var lex = Lexer{ .src = raw };
    var start: usize = 0;
    // `name:` at the start of the line binds name to this bundle's q
    const first = switch (nextTok(&lex)) {
        .tok => |t| t,
        .err => |r| return lineErr(r),
    };
    if (first.kind == .ident and peekKind(&lex) == .colon) {
        _ = lex.next();
        start = lex.i;
        label.* = first.text;
        if (env.sizing) {
            if (try define(gpa, symbols, first, env.pc)) |d| return .{ .err = d };
        }
    } else {
        lex.i = 0;
    }
    const tok = switch (nextTok(&lex)) {
        .tok => |t| t,
        .err => |r| return lineErr(r),
    };
    if (tok.kind == .end) return .none;
    if (tok.kind == .ident and tok.text[0] == '.') return directive(gpa, &lex, tok, symbols, &env);
    return switch (assembleLine(raw, start, &env)) {
        .ok => |b| .{ .bundle = b },
        .err => |d| .{ .err = d },
    };
}

fn define(gpa: std.mem.Allocator, symbols: *SymbolTable, name: Token, v: i64) !?Diagnostic {
    const gop = try symbols.getOrPut(gpa, name.text);
    if (gop.found_existing) {
        var d = Diagnostic{ .code = .e0019_duplicate_symbol, .span = tokSpan(name) };
        var buf: [128]u8 = undefined;
        d.setMsg(std.fmt.bufPrint(&buf, "symbol '{s}' is already defined", .{name.text}) catch buf[0..0]);
        d.setHint("labels and constants share one namespace");
        return d;
    }
    gop.value_ptr.* = v;
    return null;
}

fn expectEnd(lex: *Lexer) ?Diagnostic {
    const t = switch (lex.next()) {
        .tok => |tok| tok,
        .err => |e| return fromLexErr(e).err,
    };
    if (t.kind == .end) return null;
    var d = Diagnostic{ .code = .e0007_extra_tokens, .span = tokSpan(t) };
    var buf: [96]u8 = undefined;
    d.setMsg(std.fmt.bufPrint(&buf, "unexpected token '{s}' after the directive", .{t.text}) catch buf[0..0]);
    return d;
}

fn directive(gpa: std.mem.Allocator, lex: *Lexer, dir: Token, symbols: *SymbolTable, env: *const Env) !LineResult {
    if (std.mem.eql(u8, dir.text, ".const")) {
        const name = switch (nextTok(lex)) {
            .tok => |t| t,
            .err => |r| return lineErr(r),
        };
        if (name.kind != .ident or name.text[0] == '.') {
            var d = Diagnostic{ .code = .e0023_bad_directive, .span = tokSpan(name) };
            setExpected(&d, "a constant name", name);
            d.setHint(".const NAME = expr");
            return .{ .err = d };
        }
        const eq = switch (nextTok(lex)) {
            .tok => |t| t,
            .err => |r| return lineErr(r),
        };
        if (eq.kind != .equals) {
            var d = Diagnostic{ .code = .e0023_bad_directive, .span = tokSpan(eq) };
            setExpected(&d, "'='", eq);
            d.setHint(".const NAME = expr");
            return .{ .err = d };
        }
        const val = switch (parseExpr(lex, env)) {
            .ok => |v| v,
            .err => |r| return lineErr(r),
        };
        if (expectEnd(lex)) |d| return .{ .err = d };
        if (env.sizing) {
            if (try define(gpa, symbols, name, val.v)) |d| return .{ .err = d };
        } else {
            symbols.getPtr(name.text).?.* = val.v;
        }
        return .none;
    }
    if (std.mem.eql(u8, dir.text, ".assert")) {
        const val = switch (parseExpr(lex, env)) {
            .ok => |v| v,
            .err => |r| return lineErr(r),
        };
        if (expectEnd(lex)) |d| return .{ .err = d };
        if (!env.sizing and val.v == 0) {
            return lineErr(diagAt(.e0024_assert_failed, val.span, "assertion failed", "the expression evaluated to 0"));
        }
        return .none;
    }
    if (std.mem.eql(u8, dir.text, ".org")) {
        const val = switch (parseExpr(lex, env)) {
            .ok => |v| v,
            .err => |r| return lineErr(r),
        };
        if (expectEnd(lex)) |d| return .{ .err = d };
        if (val.v < 0 or val.v > 0xFFFF_FFFF) return lineErr(diagAt(.e0030_bad_placement, val.span, "'.org' row out of range", "rows are 0..0xFFF"));
        return .{ .org = .{ .row = @intCast(val.v), .span = val.span } };
    }
    const image_only = [_][]const u8{ ".pass", ".xdata", ".ydata", ".param", ".mode" };
    for (image_only) |name| if (std.mem.eql(u8, dir.text, name) and !env.image) {
        var d = Diagnostic{ .code = .e0023_bad_directive, .span = tokSpan(dir) };
        var buf: [96]u8 = undefined;
        d.setMsg(std.fmt.bufPrint(&buf, "'{s}' needs an image build", .{dir.text}) catch buf[0..0]);
        d.setHint("build passes and host data with `x17 image`");
        return .{ .err = d };
    };
    if (std.mem.eql(u8, dir.text, ".pass")) return passDirective(lex, dir);
    if (std.mem.eql(u8, dir.text, ".mode")) {
        const val = switch (parseExpr(lex, env)) {
            .ok => |v| v,
            .err => |r| return lineErr(r),
        };
        if (expectEnd(lex)) |d| return .{ .err = d };
        if (val.v < 0 or val.v > 0xFF or layout.yMap(@intCast(val.v)) == null)
            return lineErr(diagAt(.e0023_bad_directive, val.span, "no Y map for this mode", "known modes are 0, 1, 3 and 5 (fact 960)"));
        return .{ .mode = .{ .mode = @intCast(val.v), .span = val.span } };
    }
    if (std.mem.eql(u8, dir.text, ".xdata")) return dataDirective(gpa, lex, .x, env);
    if (std.mem.eql(u8, dir.text, ".ydata")) return dataDirective(gpa, lex, .y, env);
    if (std.mem.eql(u8, dir.text, ".param")) return dataDirective(gpa, lex, .param, env);
    var d = Diagnostic{ .code = .e0023_bad_directive, .span = tokSpan(dir) };
    var buf: [96]u8 = undefined;
    d.setMsg(std.fmt.bufPrint(&buf, "unknown directive '{s}'", .{dir.text}) catch buf[0..0]);
    d.setHint("directives: .const, .assert, .org, .mode, .pass, .xdata, .ydata, .param");
    return .{ .err = d };
}

fn passDirective(lex: *Lexer, dir: Token) LineResult {
    const kind_tok = switch (nextTok(lex)) {
        .tok => |t| t,
        .err => |r| return lineErr(r),
    };
    const kind: ?PassKind = if (kind_tok.kind != .ident)
        null
    else if (std.mem.eql(u8, kind_tok.text, "init"))
        .init
    else if (std.mem.eql(u8, kind_tok.text, "decoder"))
        .decoder
    else
        null;
    if (kind == null) {
        var d = Diagnostic{ .code = .e0023_bad_directive, .span = tokSpan(kind_tok) };
        setExpected(&d, "'init' or 'decoder'", kind_tok);
        d.setHint(".pass init | .pass decoder");
        return .{ .err = d };
    }
    if (expectEnd(lex)) |d| return .{ .err = d };
    return .{ .pass = .{ .kind = kind.?, .span = .{ .start = dir.start, .end = kind_tok.start + kind_tok.text.len } } };
}

/// `.xdata addr, w...` / `.ydata addr, w...` / `.param addr, w...`: range-checked in pass 2.
fn dataDirective(gpa: std.mem.Allocator, lex: *Lexer, space: Space, env: *const Env) !LineResult {
    const addr = switch (parseExpr(lex, env)) {
        .ok => |v| v,
        .err => |r| return lineErr(r),
    };
    var words: std.ArrayList(u32) = .empty;
    errdefer words.deinit(gpa);
    var last_span = addr.span;
    while (true) {
        const t = switch (nextTok(lex)) {
            .tok => |tok| tok,
            .err => |r| {
                words.deinit(gpa);
                return lineErr(r);
            },
        };
        if (t.kind == .end) break;
        if (t.kind != .comma) {
            words.deinit(gpa);
            var d = Diagnostic{ .code = .e0006_missing_comma, .span = tokSpan(t) };
            setExpected(&d, "','", t);
            return .{ .err = d };
        }
        const v = switch (parseExpr(lex, env)) {
            .ok => |v| v,
            .err => |r| {
                words.deinit(gpa);
                return lineErr(r);
            },
        };
        last_span = v.span;
        if (!env.sizing) {
            if (dataValueDiag(space, v)) |d| {
                words.deinit(gpa);
                return .{ .err = d };
            }
        }
        const mask: i64 = if (space == .x) 0xFFFFFF else 0xFFFF;
        try words.append(gpa, @intCast(v.v & mask));
    }
    const span: Span = .{ .start = addr.span.start, .end = last_span.end };
    if (words.items.len == 0) {
        return lineErr(diagAt(.e0005_missing_operand, span, "no data words", "write the words after the address: .xdata addr, w0, w1"));
    }
    if (!env.sizing) {
        if (dataAddrDiag(space, addr.v, words.items.len, span, env.mode)) |d| {
            words.deinit(gpa);
            return .{ .err = d };
        }
    }
    return .{ .data = .{ .space = space, .addr = @intCast(std.math.clamp(addr.v, 0, 0xFFFF_FFFF)), .words = try words.toOwnedSlice(gpa), .span = span } };
}

fn dataValueDiag(space: Space, v: Value) ?Diagnostic {
    const lo: i64, const hi: i64 = if (space == .x) .{ -0x800000, 0xFFFFFF } else .{ -0x8000, 0xFFFF };
    if (v.v >= lo and v.v <= hi) return null;
    const hint = if (space == .x) "X words are 24-bit" else "Y words are 16-bit (fact 758)";
    return diagAt(.e0031_data_range, v.span, "data word out of range", hint).err;
}

fn dataAddrDiag(space: Space, addr: i64, n: usize, span: Span, mode: ?u8) ?Diagnostic {
    // With a mode, host-written X fills that mode's X map (fact 961).
    if (space == .x) if (mode) |m| {
        const map = layout.xMap(m).?;
        var k: i64 = 0;
        while (k < n) : (k += 1) {
            const a = addr + k;
            if (a < 0 or a > 0xFFFF or !layout.yMapped(map, @intCast(a)))
                return diagAt(.e0031_data_range, span, "X data outside the mode's X map", "outside its mode's map the DSP neither sees nor keeps X (fact 961)").err;
        }
        return null;
    };
    // With a mode, host-written Y fills the mode's 16-bit range (its map's first entry); the
    // window behind it is a 32-bit halfword mapping (653) and is not loaded as 16-bit words.
    if (space == .y) if (mode) |m| {
        const r = layout.yMap(m).?[0];
        if (addr >= r[0] and addr + @as(i64, @intCast(n)) <= @as(i64, r[1]) + 1) return null;
        return diagAt(.e0031_data_range, span, "Y data outside the mode's 16-bit Y range", "outside its mode's map the DSP neither sees nor keeps Y (fact 960)").err;
    };
    const lo: i64, const hi: i64, const hint: []const u8 = switch (space) {
        .x => .{ 0, layout.x_words, "host-written X stays below 0x1C00 (fact 653)" },
        .y => .{ 0, layout.y_words, "host-written Y stays below 0x2E80 (fact 760)" },
        .param => .{ layout.param_lo, layout.param_hi, "the parameter block is Y:0xFD80..0xFD93" },
    };
    if (addr >= lo and addr + @as(i64, @intCast(n)) <= hi) return null;
    return diagAt(.e0031_data_range, span, "data block outside its window", hint).err;
}

/// Thin wrapper that drops diagnostics and returns the bundle.
pub fn assembleOne(src: []const u8) AsmError!Bundle {
    return switch (assembleOneResult(src)) {
        .ok => |b| b,
        .err => AsmError.AssemblyFailed,
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

fn okWord(src: []const u8) !u32 {
    const b = try okBundle(src);
    if (b.len != 1) {
        std.debug.print("expected 1-syllable bundle, got {d}\n", .{b.len});
        return error.TestExpectedOneSyllable;
    }
    return b.words[0];
}

/// st.y and other 0x0E..0x10 one-word forms assemble as head + zero second.
fn okBundle(src: []const u8) !Bundle {
    return switch (assembleOneResult(src)) {
        .ok => |b| b,
        .err => |d| {
            std.debug.print("unexpected error[{s}]: {s}\n", .{ d.code.text(), d.msg() });
            return error.TestExpectedOk;
        },
    };
}

fn expectErr(src: []const u8, code: ErrorCode) !Diagnostic {
    switch (assembleOneResult(src)) {
        .ok => |b| {
            std.debug.print("expected error, got ok (len={d}, first=0x{x})\n", .{ b.len, b.words[0] });
            return error.TestExpectedErr;
        },
        .err => |d| {
            if (d.code != code) {
                std.debug.print("expected {s}, got {s}: {s}\n", .{ code.text(), d.code.text(), d.msg() });
                return error.TestWrongErrorCode;
            }
            return d;
        },
    }
}

test "assemble rli R3, s5, #0x1234" {
    try testing.expectEqual(@as(u32, 0x1AA2_468C), try okWord("rli R3, s5, #0x1234"));
}

test "assemble ldi s0, #0x1234 maps dest to k=16" {
    try testing.expectEqual(@as(u32, 0xCA02_468B), try okWord("ldi s0, #0x1234"));
}

test "assemble alu s7, s2, s9, #5" {
    // dest_s=7 <<15 = 0x00038000
    // a_s   =2 <<11 = 0x00001000
    // b_s   =9 <<26 = 0x24000000
    // op    =5 <<5  = 0x000000A0
    // base          = 0x40000005
    // OR            = 0x640390A5
    try testing.expectEqual(@as(u32, 0x6403_90A5), try okWord("alu s7, s2, s9, #5"));
}

test "assemble alu with b in high class (s12)" {
    // b_s=12 <<26 = 0x30000000; bit 29 = 1 picks class 011.
    const w = try okWord("alu s0, s0, s12, #0");
    try testing.expectEqual(@as(u32, 0x4000_0005) | (12 << 26), w);
}

test "diag: unknown mnemonic points at mnemonic" {
    const d = try expectErr("nope s0, #0", .e0001_unknown_mnemonic);
    try testing.expectEqual(@as(usize, 0), d.span.start);
    try testing.expectEqual(@as(usize, 4), d.span.end);
}

test "diag: expected scalar reg, caret at R3" {
    const d = try expectErr("alu R3, s2, s9, #5", .e0008_expected_scalar_reg);
    try testing.expectEqual(@as(usize, 4), d.span.start);
    try testing.expectEqual(@as(usize, 6), d.span.end);
    try testing.expectEqualStrings("scalar registers are written s0..s15", d.hint());
}

test "diag: imm out of range names the width" {
    const d = try expectErr("rli R0, s0, #0x10000", .e0013_imm_out_of_range);
    try testing.expect(std.mem.indexOf(u8, d.msg(), "16 bits") != null);
}

test "diag: missing operand anchors at end of input" {
    const src = "rli R0, s0";
    const d = try expectErr(src, .e0005_missing_operand);
    try testing.expectEqual(src.len, d.span.start);
    try testing.expectEqual(src.len, d.span.end);
}

test "diag: extra trailing token flagged" {
    _ = try expectErr("ldi s0, #0 extra", .e0007_extra_tokens);
}

test "diag: renderDiag prints rust-style block" {
    const src = "alu R3, s2, s9, #5";
    const d = try expectErr(src, .e0008_expected_scalar_reg);
    var buf: [1024]u8 = undefined;
    const n = renderDiag(&buf, "input", src, d);
    const out = buf[0..n];
    try testing.expect(std.mem.indexOf(u8, out, "error[E0008]:") != null);
    try testing.expect(std.mem.indexOf(u8, out, "--> input:1:5") != null);
    try testing.expect(std.mem.indexOf(u8, out, "\n 1 | alu R3") != null);
    try testing.expect(std.mem.indexOf(u8, out, "^^") != null);
    try testing.expect(std.mem.indexOf(u8, out, "help: scalar registers") != null);
}

test "assemble st.y Y:1" {
    // addr=1 << 5 = 0x20; base = 0x2000_000F; OR = 0x2000_002F
    try testing.expectEqual(@as(u32, 0x2000_002F), try okWord("st.y Y:1"));
}

test "assemble st.y Y:#0x1234" {
    // addr=0x1234 << 5 = 0x0002_4680; base = 0x2000_000F; OR = 0x2002_468F
    try testing.expectEqual(@as(u32, 0x2002_468F), try okWord("st.y Y:#0x1234"));
}

test "diag: y_addr missing Y prefix" {
    _ = try expectErr("st.y 1", .e0016_expected_y_prefix);
}

test "diag: y_addr missing colon" {
    _ = try expectErr("st.y Y 1", .e0017_expected_colon);
}

test "diag: y_addr overflow carries width" {
    const d = try expectErr("st.y Y:#0x10000", .e0013_imm_out_of_range);
    try testing.expect(std.mem.indexOf(u8, d.msg(), "16 bits") != null);
}

test "x17.halt emits the canonical 3-syllable bundle" {
    const b = try okBundle("x17.halt");
    try testing.expectEqual(@as(u8, 3), b.len);
    try testing.expectEqual(@as(u32, 0x0000_0017), b.words[0]);
    try testing.expectEqual(@as(u32, 0x3130_8000), b.words[1]);
    try testing.expectEqual(@as(u32, 0xA000_0000), b.words[2]);
}

test "diag: x17.halt takes no operands" {
    _ = try expectErr("x17.halt s0", .e0007_extra_tokens);
}

test "diag: unknown x17 sub-form" {
    const d = try expectErr("x17.wat", .e0001_unknown_mnemonic);
    try testing.expect(std.mem.indexOf(u8, d.msg(), "x17.wat") != null);
}

test "round-trip gate: own-x17alu-shl bundle" {
    // own-x17alu-shl, mid-program x17 at r3l0/r2l1/r3l1:
    //   head = 0x00000017  (op = head[10:7] = 0, add)
    //   S1   = 0x31328000  (dest=s1 @27:24, n=3 @23:20, b=s2 @19:16)
    //   S2   = 0xF0000000  (a=s8 @28:25, class 111)
    const b = try okBundle("x17.alu s1, s8, s2, #0, #3");
    try testing.expectEqual(@as(u8, 3), b.len);
    try testing.expectEqual(@as(u32, 0x0000_0017), b.words[0]);
    try testing.expectEqual(@as(u32, 0x3132_8000), b.words[1]);
    try testing.expectEqual(@as(u32, 0xF000_0000), b.words[2]);
}

test "round-trip gate: own-x17alu-rsub bundle (op=4, a=s2, b=s4)" {
    // own-x17alu-rsub, same base with head[10:7]=4 and b=s4;
    // bytes: head 0x00000217, S1 0x31348000, S2 0xF0000000.
    const b = try okBundle("x17.alu s1, s8, s4, #4, #3");
    try testing.expectEqual(@as(u32, 0x0000_0217), b.words[0]);
    try testing.expectEqual(@as(u32, 0x3134_8000), b.words[1]);
    try testing.expectEqual(@as(u32, 0xF000_0000), b.words[2]);
}

test "x17.imm encodes dest into S1 and imm into head" {
    // own-x17imm: [0x005F7017, 0x60004280, 0xE0000000].
    //   head: 0x005F7017 = base 0x00000017 | (imm 0x0BEE << 11 = 0x005F7000)
    //   S1:   0x60004280 = base 0x60004000 | (dest_s=5 << 7 = 0x280)
    //   S2:   0xE0000000 = base 0xE0000000 (no operand fields)
    const b = try okBundle("x17.imm s5, #0x0BEE");
    try testing.expectEqual(@as(u8, 3), b.len);
    try testing.expectEqual(@as(u32, 0x005F_7017), b.words[0]);
    try testing.expectEqual(@as(u32, 0x6000_4280), b.words[1]);
    try testing.expectEqual(@as(u32, 0xE000_0000), b.words[2]);
}

test "round-trip gate: own-x17ctl tail bundle (x17.halt)" {
    // own-x17ctl tail (r3l3, r4l0, r5l0) uses S1 = 0x11308001
    // (the 011 S1 class, S1[0] set).  own-x17alu-shl tail (r2l1-r3l1-r2l0
    // actually... many entries) uses the S1 = 0x31308000 canonical form.
    // The three stable halt syllables are:
    //   head = 0x00000017
    //   S1   = 0x31308000
    //   S2   = 0xA0000000
    const b = try okBundle("x17.halt");
    try testing.expectEqual(@as(u32, 0x0000_0017), b.words[0]);
    try testing.expectEqual(@as(u32, 0x3130_8000), b.words[1]);
    try testing.expectEqual(@as(u32, 0xA000_0000), b.words[2]);
}

test "round-trip gate: own-st3 reassembles byte-exact" {
    // own-st3: three of OUR stores at r5l1..l3.
    //   r5l1 = 0x2000200F  <- st.y Y:0x100
    //   r5l2 = 0x2000400F  <- st.y Y:0x200
    //   r5l3 = 0x2000600F  <- st.y Y:0x300
    // Byte-exact: no inert bits (unlike own-ldi-y2's [29:26] = 0xC quirk).
    try testing.expectEqual(@as(u32, 0x2000_200F), try okWord("st.y Y:0x100"));
    try testing.expectEqual(@as(u32, 0x2000_400F), try okWord("st.y Y:0x200"));
    try testing.expectEqual(@as(u32, 0x2000_600F), try okWord("st.y Y:0x300"));
}

test "comment to end of line is tolerated" {
    try testing.expectEqual(
        @as(u32, 0xC800_000B) | (16 << 21),
        try okWord("ldi s0, #0 ; load zero into source k=16"),
    );
}

// ---------------------------------------------------------------------------
// Two-pass file tests: labels, constants, expressions
// ---------------------------------------------------------------------------

fn fileOk(src: []const u8) ![]u32 {
    return switch (try assembleFile(testing.allocator, src)) {
        .ok => |w| w,
        .err => |fe| {
            var buf: [2048]u8 = undefined;
            const n = renderDiagAt(&buf, "test", fe.line, fe.lineText(), fe.diag);
            std.debug.print("{s}", .{buf[0..n]});
            return error.TestUnexpectedResult;
        },
    };
}

fn fileErr(src: []const u8) !FileError {
    return switch (try assembleFile(testing.allocator, src)) {
        .ok => |w| {
            testing.allocator.free(w);
            return error.TestUnexpectedResult;
        },
        .err => |fe| fe,
    };
}

fn expectSameBytes(a_src: []const u8, b_src: []const u8) !void {
    const a = try fileOk(a_src);
    defer testing.allocator.free(a);
    const b = try fileOk(b_src);
    defer testing.allocator.free(b);
    try testing.expectEqualSlices(u32, b, a);
}

test "file: a forward label on bcc encodes the measured own-bcc-fwd bytes" {
    try expectSameBytes(
        \\bcc #0, skip
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\skip: alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\x17.halt
    ,
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
    );
}

test "file: a backward label on bcc encodes the measured own-bcc-back bytes" {
    try expectSameBytes(
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\again:
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\alu s1, s1, s2, #14
        \\bcc #2, again
        \\plog p2, p2, p2, #7, #0x0E
        \\alu s0, s0, s0, #0
        \\x17.halt
    ,
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
    );
}

test "file: jmp and br take label targets in syllables" {
    try expectSameBytes(
        \\top: ldi s1, #1
        \\jmp #1, #0, top
        \\br #3, out
        \\ldi s2, #2
        \\out: x17.halt
    ,
        \\ldi s1, #1
        \\jmp #1, #0, #0xFFFF
        \\br #3, #2
        \\ldi s2, #2
        \\x17.halt
    );
}

test "file: constants, forward references and expressions fold" {
    try expectSameBytes(
        \\.const WORD = BASE << 4 | 3
        \\.const BASE = 0x40
        \\ldi s5, #WORD
        \\ldi s6, #-1
        \\ldi s7, #(end - start) * 2
        \\start: x17.imm s5, #~0 & 0xFF
        \\end: x17.halt
        \\.assert end - start == 3
    ,
        \\ldi s5, #0x403
        \\ldi s6, #0xFFFF
        \\ldi s7, #6
        \\x17.imm s5, #0xFF
        \\x17.halt
    );
}

test "file: undefined symbol points at the name" {
    const fe = try fileErr(
        \\ldi s1, #1
        \\ldi s2, #(MISSING + 1)
    );
    try testing.expectEqual(ErrorCode.e0018_undefined_symbol, fe.diag.code);
    try testing.expectEqual(@as(usize, 2), fe.line);
    try testing.expectEqualStrings("MISSING", fe.lineText()[fe.diag.span.start..fe.diag.span.end]);
}

test "file: a label defined twice is refused" {
    const fe = try fileErr(
        \\a: ldi s1, #1
        \\a: ldi s2, #2
    );
    try testing.expectEqual(ErrorCode.e0019_duplicate_symbol, fe.diag.code);
}

test "file: bcc to a target not a multiple of 4 away is refused" {
    const fe = try fileErr(
        \\bcc #0, t
        \\ldi s1, #1
        \\t: x17.halt
    );
    try testing.expectEqual(ErrorCode.e0021_branch_misaligned, fe.diag.code);
}

test "file: br cannot reach backwards" {
    const fe = try fileErr(
        \\t: ldi s1, #1
        \\br #3, t
    );
    try testing.expectEqual(ErrorCode.e0022_branch_out_of_range, fe.diag.code);
}

test "file: a false .assert fails in pass 2" {
    const fe = try fileErr(
        \\.const N = 3
        \\.assert N == 4
    );
    try testing.expectEqual(ErrorCode.e0024_assert_failed, fe.diag.code);
}

test "file: unknown directive" {
    const fe = try fileErr(".align 2");
    try testing.expectEqual(ErrorCode.e0023_bad_directive, fe.diag.code);
}

fn progOk(src: []const u8) !Program {
    return switch (try assembleProgram(testing.allocator, src, .{})) {
        .ok => |p| p,
        .err => |fe| {
            var buf: [2048]u8 = undefined;
            const n = renderDiagAt(&buf, "test", fe.line, fe.lineText(), fe.diag);
            std.debug.print("{s}", .{buf[0..n]});
            return error.TestUnexpectedResult;
        },
    };
}

fn progErr(src: []const u8) !FileError {
    switch (try assembleProgram(testing.allocator, src, .{})) {
        .ok => |p| {
            var q = p;
            q.deinit();
            return error.TestUnexpectedResult;
        },
        .err => |fe| return fe,
    }
}

test "layout: q order runs column-major over row pairs (fact 45)" {
    try testing.expectEqual(@as(u32, 256), layout.qOf(0x40, 0));
    try testing.expectEqual(@as(u32, 1), layout.qOf(1, 0));
    try testing.expectEqual(@as(u32, 2), layout.qOf(0, 1));
    for (0..64) |q| {
        const qq: u32 = @intCast(q);
        try testing.expectEqual(qq, layout.qOf(layout.rowOf(qq), layout.laneOf(qq)));
    }
}

test "file: .org pads with zero syllables to the row's q" {
    const w = try fileOk("x17.halt\n.org 2\nx17.halt");
    defer testing.allocator.free(w);
    try testing.expectEqual(@as(usize, 11), w.len);
    try testing.expectEqual(@as(u32, 0), w[3]);
    try testing.expectEqual(@as(u32, 0x00000017), w[8]);
}

test "file: .org refuses backwards, odd and past-the-end rows" {
    for ([_][]const u8{ "x17.halt\nx17.halt\nx17.halt\n.org 0", ".org 3", ".org 0x1000" }) |src| {
        const fe = try fileErr(src);
        try testing.expectEqual(ErrorCode.e0030_bad_placement, fe.diag.code);
    }
}

test "file: image directives need an image build" {
    for ([_][]const u8{ ".pass init", ".xdata 0, 1", ".ydata 0, 1", ".param 0xFD80, 1" }) |src| {
        const fe = try fileErr(src);
        try testing.expectEqual(ErrorCode.e0023_bad_directive, fe.diag.code);
    }
}

test "program: init passes at q0, the decoder at row 0x40 = q256" {
    var p = try progOk(
        \\.pass init
        \\.xdata 0x10, 1, -1
        \\a: x17.halt
        \\.pass init
        \\.ydata 0x2E7F, 0xFFFF
        \\x17.halt
        \\.pass decoder
        \\.param 0xFD83, 4
        \\top: x17.halt
    );
    defer p.deinit();
    try testing.expectEqual(@as(usize, 3), p.passes.len);
    try testing.expectEqual(PassKind.decoder, p.passes[2].kind);
    try testing.expectEqual(@as(u32, 256), p.passes[2].baseQ());
    try testing.expectEqual(@as(u32, 0xFFFFFF), p.passes[0].data[0].words[1]);
    try testing.expectEqual(Space.param, p.passes[2].data[0].space);
    try testing.expectEqual(@as(u32, 256), p.labels[1].q);
    try testing.expectEqual(@as(usize, 2), p.labels[1].pass);
    // 3 syllables at q256..258 sit in rows 0x40 and 0x41; two zero rows follow.
    try testing.expectEqual(@as(u32, 0x43), p.passes[2].endRow());
}

test "program: a source without .pass is one init pass" {
    var p = try progOk("x17.halt\n.xdata 0, 5");
    defer p.deinit();
    try testing.expectEqual(@as(usize, 1), p.passes.len);
    try testing.expectEqual(PassKind.init, p.passes[0].kind);
}

test "program: pass order and content rules" {
    for ([_][]const u8{
        "x17.halt\n.pass init\nx17.halt", // code before the first .pass
        ".pass decoder\nx17.halt\n.pass init\nx17.halt", // a pass after the decoder
        ".pass init\n.pass decoder\nx17.halt", // an empty pass
    }) |src| {
        const fe = try progErr(src);
        try testing.expectEqual(ErrorCode.e0030_bad_placement, fe.diag.code);
    }
}

test "program: data windows and word widths" {
    for ([_][]const u8{
        ".pass init\nx17.halt\n.xdata 0x1BFF, 1, 2", // X past 0x1C00 (fact 653)
        ".pass init\nx17.halt\n.ydata 0x2E80, 1", // Y past 0x2E80 (fact 760)
        ".pass init\nx17.halt\n.ydata 0, 0x10000", // Y is 16-bit (fact 758)
        ".pass init\nx17.halt\n.xdata 0, 0x1000000", // X is 24-bit
        ".pass init\nx17.halt\n.param 0xFD83, 4", // .param outside the decoder
        ".pass decoder\nx17.halt\n.param 0xFD94, 1", // the status header is the engine's
        ".pass init\nx17.halt\n.xdata 0, 1, 2\n.xdata 1, 3", // overlap in one pass
    }) |src| {
        const fe = try progErr(src);
        try testing.expectEqual(ErrorCode.e0031_data_range, fe.diag.code);
    }
}

test "program: .mode checks .ydata and .xdata against the mode's maps (960, 961)" {
    var ok = try progOk(".mode 3\n.pass init\nx17.halt\n.ydata 0x227F, 1\n.xdata 0x20FF, 1");
    try testing.expectEqual(@as(?u8, 3), ok.mode);
    ok.deinit();
    for ([_][]const u8{
        ".mode 3\n.pass init\nx17.halt\n.ydata 0x2D00, 1", // ALAC's old setup block
        ".mode 3\n.pass init\nx17.halt\n.ydata 0x227F, 1, 2", // runs off the end
        ".mode 3\n.pass init\nx17.halt\n.ydata 0x2400, 1", // the window
        ".mode 0\n.pass init\nx17.halt\n.ydata 0x2E80, 1", // the window
        ".mode 1\n.pass init\nx17.halt\n.xdata 0x1200, 1", // past mode 1's X (961)
    }) |src| {
        const fe = try progErr(src);
        try testing.expectEqual(ErrorCode.e0031_data_range, fe.diag.code);
    }
    for ([_][]const u8{
        ".pass init\n.mode 3\nx17.halt", // after a pass
        ".mode 3\n.mode 3\n.pass init\nx17.halt", // twice
    }) |src| {
        const fe = try progErr(src);
        try testing.expectEqual(ErrorCode.e0030_bad_placement, fe.diag.code);
    }
    const fe = try progErr(".mode 2\n.pass init\nx17.halt");
    try testing.expectEqual(ErrorCode.e0023_bad_directive, fe.diag.code);
}

test "program: init passes stay below the decoder's row 0x40" {
    const fe = try progErr(".pass init\n.org 0x3C\nx17.halt\nx17.halt\nx17.halt\n.pass decoder\nx17.halt");
    try testing.expectEqual(ErrorCode.e0030_bad_placement, fe.diag.code);
    var ok = try progOk(".pass init\n.org 0x3C\nx17.halt\n.pass decoder\nx17.halt");
    ok.deinit();
}

test "program: a program's two zero rows stay inside row 0xFFF" {
    const fe = try progErr(".pass decoder\n.org 0xFFE\nx17.halt");
    try testing.expectEqual(ErrorCode.e0030_bad_placement, fe.diag.code);
}

test "file: an immediate out of range reports the value and the field" {
    const fe = try fileErr(".const BIG = 0x10000\nldi s1, #BIG");
    try testing.expectEqual(ErrorCode.e0013_imm_out_of_range, fe.diag.code);
    try testing.expectEqualStrings("immediate 0x10000 does not fit in 16 bits", fe.diag.msg());
}

test {
    _ = expand;
}

test "file: a macro assembles to the same bytes as its expansion" {
    const with = try assembleFile(testing.allocator,
        \\.const ADD = 14
        \\.macro add2(d, a, b) {
        \\        alu d, a, b, #ADD
        \\        alu d, a, b, #ADD
        \\}
        \\        ldi s2, #1
        \\        add2(s1, s1, s2)
        \\        x17.halt
    );
    defer testing.allocator.free(with.ok);
    const by_hand = try assembleFile(testing.allocator,
        \\        ldi s2, #1
        \\        alu s1, s1, s2, #14
        \\        alu s1, s1, s2, #14
        \\        x17.halt
    );
    defer testing.allocator.free(by_hand.ok);
    try testing.expectEqualSlices(u32, by_hand.ok, with.ok);
}

test "file: an error inside a macro names the body line and the call" {
    const r = try assembleFile(testing.allocator,
        \\.macro bad(r) {
        \\        ldi r, #0x10000
        \\}
        \\        ldi s1, #1
        \\        bad(s3)
    );
    const fe = r.err;
    try testing.expectEqual(ErrorCode.e0013_imm_out_of_range, fe.diag.code);
    try testing.expectEqual(@as(usize, 2), fe.line);
    try testing.expectEqualStrings("        ldi s3, #0x10000", fe.lineText());
    try testing.expectEqualStrings("bad", fe.call_name.get());
    try testing.expectEqual(@as(usize, 5), fe.call_line);
}

test "x16 forms refuse operands that flip their S1 class" {
    const Case = struct { bad: []const u8, good: []const u8 };
    for ([_]Case{
        .{ .bad = "x16.xld s5, #10", .good = "x16.xld s6, #10" },
        .{ .bad = "x16.st2 s8, #6, e4, s6", .good = "x16.st2 s7, #6, e4, s6" },
        .{ .bad = "x16.b16 s10, #0x300, s1, e5", .good = "x16.b16 s10, #0x300, s2, e5" },
        .{ .bad = "x16.agu2 s3, e2, R3, #8, #1, #1", .good = "x16.agu2 s3, e2, R3, #7, #1, #1" },
        .{ .bad = "x16.agumul #0, #0, s3, s5", .good = "x16.agumul #1, #0, s3, s5" },
        .{ .bad = "x16.mac s7, s6, #0, #0, s2, s3, s5, #1", .good = "x16.mac s7, s6, #1, #0, s2, s3, s5, #1" },
    }) |c| {
        switch (assembleOneResult(c.bad)) {
            .err => |d| try std.testing.expectEqual(ErrorCode.e0029_form_collision, d.code),
            .ok => return error.TestUnexpectedResult,
        }
        switch (assembleOneResult(c.good)) {
            .err => return error.TestUnexpectedResult,
            .ok => {},
        }
    }
}

test "x17.sty refuses s0 and s8, whose bundle decodes as x17.ld" {
    switch (assembleOneResult("x17.sty e3, #0, #1")) {
        .err => |d| try std.testing.expectEqual(ErrorCode.e0029_form_collision, d.code),
        .ok => return error.TestUnexpectedResult,
    }
    switch (assembleOneResult("x17.sty e3, #5, #0")) {
        .err => return error.TestUnexpectedResult,
        .ok => {},
    }
}
