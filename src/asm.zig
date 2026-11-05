//! x17 assembler, v0: one line, one bundle, one syllable out.
//!
//! Scope of this spike:
//!   * Lex a single line into tokens carrying source-byte spans.
//!   * Parse `mnemonic op (, op)*`.
//!   * Match operands by file order against `isa.operandsFor(form)`.
//!   * Emit the head syllable by OR'ing the form's `value` with each
//!     encoded operand.
//!   * On error, build a `Diagnostic` carrying an error code, span, and
//!     "expected/found" message, printable with a rust-style caret block.
//!
//! What waits:
//!   * Multi-line source, labels, directives, const-expression folding.
//!   * Multi-syllable bundles (x17's head+S1+S2, bcc's S word).
//!   * ldi's dest decoding beyond s<N> (0 = address entry 0, 1 = source 1,
//!     2..15 marked OPEN in isa.toml - the assembler refuses them today).
//!   * `Y:<imm>` operand kind for st.y.
//!
//! The round-trip gate (reassemble a disassembled stock bundle byte-exact)
//! bolts onto this file as a test once labels land.

const std = @import("std");
const isa = @import("isa");

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

    fn setMsg(self: *Diagnostic, bytes: []const u8) void {
        const n = @min(bytes.len, self.msg_buf.len);
        @memcpy(self.msg_buf[0..n], bytes[0..n]);
        self.msg_len = @intCast(n);
    }

    fn setHint(self: *Diagnostic, bytes: []const u8) void {
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
    p_reg,
    number,
    comma,
    hash,
    colon,
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

        self.i += 1;
        return .{ .err = .{
            .code = .e0003_bad_token,
            .span = .{ .start = start, .end = self.i },
            .msg = "unexpected character",
            .hint = "x17 source accepts identifiers, numbers, ',' ':' '#'",
        } };
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
        .y_addr => unreachable, // handled inline in the operand loop
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
        .p_reg => "predicate",
        .number => "number",
        .comma => "','",
        .hash => "'#'",
        .colon => "':'",
        .end => "end of input",
    };
}

fn setExpected(d: *Diagnostic, want: []const u8, tok: Token) void {
    var buf: [128]u8 = undefined;
    const m = std.fmt.bufPrint(&buf, "expected {s}, found {s}", .{ want, kindName(tok.kind) }) catch buf[0..0];
    d.setMsg(m);
}

// ---------------------------------------------------------------------------
// Assemble one line
// ---------------------------------------------------------------------------

/// Assemble one source line.  Carries diagnostics.
pub fn assembleOneResult(src: []const u8) Result {
    var lex = Lexer{ .src = src };
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

    // x17.<sub> / x19.<sub> namespace: multi-syllable bundles, dispatched
    // out of band.  Both route through the same data-driven packer; which
    // sub-form is picked by name alone, not by the x17 vs x19 prefix.
    if (std.mem.startsWith(u8, head.text, "x17.") or std.mem.startsWith(u8, head.text, "x19.")) {
        return assembleX17(head, &lex, src);
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
            if (tok.kind != .ident or !std.mem.eql(u8, tok.text, "Y")) {
                var d = Diagnostic{ .code = .e0016_expected_y_prefix, .span = tokSpan(tok) };
                setExpected(&d, "'Y'", tok);
                d.setHint("Y-space addresses look like Y:1 or Y:#0x1234");
                return .{ .err = d };
            }
            const cr = lex.next();
            const colon = switch (cr) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
            if (colon.kind != .colon) {
                var d = Diagnostic{ .code = .e0017_expected_colon, .span = tokSpan(colon) };
                setExpected(&d, "':' after 'Y'", colon);
                return .{ .err = d };
            }
            var nr = lex.next();
            var ntok = switch (nr) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
            if (ntok.kind == .hash) {
                nr = lex.next();
                ntok = switch (nr) {
                    .tok => |t| t,
                    .err => |e| return fromLexErr(e),
                };
            }
            if (ntok.kind != .number) {
                var d = Diagnostic{ .code = .e0012_expected_immediate, .span = tokSpan(ntok) };
                setExpected(&d, "Y-space address immediate", ntok);
                d.setHint("Y:<N> or Y:#<N>; N is a 16-bit value");
                return .{ .err = d };
            }
            const w = op.width();
            const max: u32 = (@as(u32, 1) << @as(u5, @intCast(w))) - 1;
            if (ntok.value > max) {
                var d = Diagnostic{ .code = .e0013_imm_out_of_range, .span = tokSpan(ntok) };
                var buf: [96]u8 = undefined;
                const m = std.fmt.bufPrint(&buf, "Y-space address 0x{x} does not fit in {d} bits", .{ ntok.value, w }) catch buf[0..0];
                d.setMsg(m);
                return .{ .err = d };
            }
            word |= op.encode(ntok.value);
            continue;
        }

        if (tok.kind == .hash) {
            const nr = lex.next();
            tok = switch (nr) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
            if (tok.kind == .end) {
                var d = Diagnostic{ .code = .e0005_missing_operand, .span = .{ .start = src.len, .end = src.len } };
                d.setMsg("'#' is not followed by a number");
                return .{ .err = d };
            }
        }
        var d: Diagnostic = undefined;
        const field = matchOperand(op, tok, &d) orelse return .{ .err = d };
        word |= op.encode(field);
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
    return .{ .ok = oneWord(word) };
}

/// Assemble an x17.* bundle.  Head's text is `x17.<sub>`; the lexer is
/// positioned just after it.  Looks up the sub-form in `isa.subforms`,
/// seeds each syllable with the sub-form's base, then walks the operand
/// rows (which name their syllable via `syll`) and OR's the encoded
/// fields into the matching word.
fn assembleX17(head: Token, lex: *Lexer, src: []const u8) Result {
    const sf = isa.subFormByName(head.text) orelse {
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
        var tok = switch (lex.next()) {
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
        if (tok.kind == .hash) {
            tok = switch (lex.next()) {
                .tok => |t| t,
                .err => |e| return fromLexErr(e),
            };
        }
        var d: Diagnostic = undefined;
        const field = matchOperand(op, tok, &d) orelse return .{ .err = d };
        if (op.syll >= sf.syllables) {
            var dx = Diagnostic{ .code = .e0001_unknown_mnemonic, .span = tokSpan(tok) };
            dx.setMsg("operand table names a syllable past the bundle length");
            return .{ .err = dx };
        }
        b.words[op.syll] |= op.encode(field);
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
    return .{ .ok = b };
}

fn fromLexErr(e: LexError) Result {
    var d = Diagnostic{ .code = e.code, .span = e.span };
    d.setMsg(e.msg);
    if (e.hint.len > 0) d.setHint(e.hint);
    return .{ .err = d };
}

/// Error returned by `assembleFile` carrying enough context for a stellar
/// error print: which line failed, the line text (a slice into the input),
/// and the diagnostic.
pub const FileError = struct {
    line: usize,
    line_text: []const u8,
    diag: Diagnostic,
};

/// Assemble a multi-line source file.  Blank lines and lines starting with
/// `;` or `#` are skipped as comments.  On success, returns a newly
/// allocated slice of syllables (caller frees).  On failure, returns a
/// `FileError` attached to the offending line.
pub fn assembleFile(
    gpa: std.mem.Allocator,
    source: []const u8,
) !union(enum) { ok: []u32, err: FileError } {
    var list: std.ArrayList(u32) = .empty;
    errdefer list.deinit(gpa);

    var line_no: usize = 0;
    var it = std.mem.splitScalar(u8, source, '\n');
    while (it.next()) |raw| {
        line_no += 1;
        const trimmed = std.mem.trim(u8, raw, " \t\r");
        if (trimmed.len == 0) continue;
        if (trimmed[0] == ';' or trimmed[0] == '#') continue;
        switch (assembleOneResult(raw)) {
            .ok => |b| try list.appendSlice(gpa, b.slice()),
            .err => |d| return .{ .err = .{ .line = line_no, .line_text = raw, .diag = d } },
        }
    }
    const owned = try list.toOwnedSlice(gpa);
    return .{ .ok = owned };
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
    try testing.expectEqual(@as(u32, 0xC202_468B), try okWord("ldi s0, #0x1234"));
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
    // the probe list own-x17alu-shl, mid-program x17 at r3l0/r2l1/r3l1:
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
    // the probe list own-x17alu-rsub, same base with head[10:7]=4 and b=s4;
    // bytes: head 0x00000217, S1 0x31348000, S2 0xF0000000.
    const b = try okBundle("x17.alu s1, s8, s4, #4, #3");
    try testing.expectEqual(@as(u32, 0x0000_0217), b.words[0]);
    try testing.expectEqual(@as(u32, 0x3134_8000), b.words[1]);
    try testing.expectEqual(@as(u32, 0xF000_0000), b.words[2]);
}

test "x17.imm encodes dest into S1 and imm into head" {
    // own-x17imm (the probe list): [0x005F7017, 0x60004280, 0xE0000000].
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
    // the probe list own-x17ctl tail (r3l3, r4l0, r5l0) uses S1 = 0x11308001
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
    // the probe list own-st3: three of OUR stores at r5l1..l3.
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
        @as(u32, 0xC000_000B) | (16 << 21),
        try okWord("ldi s0, #0 ; load zero into source k=16"),
    );
}
