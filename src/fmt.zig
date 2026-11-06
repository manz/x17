//! `x17 fmt`: one canonical layout for .sm sources (a816 format / zig fmt).
//!
//!   label:  mnemonic op, op, op           ; comment
//!   ^0      ^8                            ^32
//!
//! - Instructions and macro calls start at column 8; a label sits at column
//!   0 and pads to 8 (one space when longer).
//! - Operands: one space after each top-level comma, runs of spaces folded.
//! - Trailing comments start at column 32, or one space after the code.
//! - Full-line comments stay at column 0 when they were there, else 8.
//! - Directives start at column 0; a run of `.const` lines aligns its `=`.
//! - `.macro name(a, b) {` and `}` at column 0.
//! - Tabs become spaces, trailing spaces go, blank runs fold to one, the
//!   file ends with one newline.
//!
//! The formatter only moves whitespace.  The CLI also assembles before and
//! after and refuses a result whose bytes differ.

const std = @import("std");

pub const code_col = 8;
pub const comment_col = 32;

const Kind = enum { blank, comment, code, constant };

const Out = struct {
    kind: Kind,
    /// For `.const`: the name, the expression; for code: the code text.
    a: []const u8 = "",
    b: []const u8 = "",
    comment: []const u8 = "",
    indent: usize = 0,
};

/// Formats `src`; the result is allocated with `gpa`.
pub fn format(gpa: std.mem.Allocator, src: []const u8) ![]u8 {
    var arena_state = std.heap.ArenaAllocator.init(gpa);
    defer arena_state.deinit();
    const arena = arena_state.allocator();

    var lines: std.ArrayList(Out) = .empty;
    var it = std.mem.splitScalar(u8, src, '\n');
    while (it.next()) |raw0| {
        const raw = try detab(arena, std.mem.trimEnd(u8, raw0, "\r"));
        try lines.append(arena, try classify(arena, raw));
    }

    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(gpa);
    var i: usize = 0;
    var pending_blank = false;
    var wrote = false;
    while (i < lines.items.len) {
        const l = lines.items[i];
        if (l.kind == .blank) {
            pending_blank = wrote;
            i += 1;
            continue;
        }
        if (pending_blank) try out.append(gpa, '\n');
        pending_blank = false;
        wrote = true;
        if (l.kind == .constant) {
            // A run of consecutive .const lines aligns its names.
            var j = i;
            var w: usize = 0;
            while (j < lines.items.len and lines.items[j].kind == .constant) : (j += 1) w = @max(w, lines.items[j].a.len);
            for (lines.items[i..j]) |c| {
                const start = out.items.len;
                try out.print(gpa, ".const {s}", .{c.a});
                try out.appendNTimes(gpa, ' ', w - c.a.len);
                try out.print(gpa, " = {s}", .{c.b});
                try endLine(gpa, &out, start, c.comment);
            }
            i = j;
            continue;
        }
        const start = out.items.len;
        switch (l.kind) {
            .comment => {
                try out.appendNTimes(gpa, ' ', l.indent);
                try out.appendSlice(gpa, l.comment);
                try out.append(gpa, '\n');
            },
            .code => {
                try out.appendSlice(gpa, l.a);
                try endLine(gpa, &out, start, l.comment);
            },
            else => unreachable,
        }
        i += 1;
    }
    return out.toOwnedSlice(gpa);
}

fn endLine(gpa: std.mem.Allocator, out: *std.ArrayList(u8), start: usize, comment: []const u8) !void {
    if (comment.len > 0) {
        const used = out.items.len - start;
        try out.appendNTimes(gpa, ' ', if (used < comment_col) comment_col - used else 1);
        try out.appendSlice(gpa, comment);
    }
    try out.append(gpa, '\n');
}

fn detab(arena: std.mem.Allocator, s: []const u8) ![]const u8 {
    if (std.mem.indexOfScalar(u8, s, '\t') == null) return std.mem.trimEnd(u8, s, " ");
    var o: std.ArrayList(u8) = .empty;
    for (s) |c| {
        if (c == '\t') {
            try o.append(arena, ' ');
            while (o.items.len % 8 != 0) try o.append(arena, ' ');
        } else try o.append(arena, c);
    }
    return std.mem.trimEnd(u8, o.items, " ");
}

/// Splits off a `;` comment that is not inside quotes.
fn splitComment(s: []const u8) struct { []const u8, []const u8 } {
    var quoted = false;
    for (s, 0..) |c, k| {
        if (c == '"') quoted = !quoted;
        if (c == ';' and !quoted) return .{ std.mem.trimEnd(u8, s[0..k], " "), std.mem.trimEnd(u8, s[k..], " ") };
    }
    return .{ s, "" };
}

fn classify(arena: std.mem.Allocator, raw: []const u8) !Out {
    const t = std.mem.trimStart(u8, raw, " ");
    if (t.len == 0) return .{ .kind = .blank };
    if (t[0] == ';' or t[0] == '#') return .{ .kind = .comment, .comment = t, .indent = if (raw.len == t.len) 0 else code_col };
    const code, const comment = splitComment(t);
    if (code.len == 0) return .{ .kind = .comment, .comment = comment, .indent = if (raw.len == t.len) 0 else code_col };

    if (startsWord(code, ".const")) {
        const rest = std.mem.trim(u8, code[".const".len..], " ");
        if (std.mem.indexOfScalar(u8, rest, '=')) |eq| {
            const name = std.mem.trim(u8, rest[0..eq], " ");
            const expr = try fold(arena, std.mem.trim(u8, rest[eq + 1 ..], " "));
            if (name.len > 0 and expr.len > 0) return .{ .kind = .constant, .a = name, .b = expr, .comment = comment };
        }
        return .{ .kind = .code, .a = try fold(arena, code), .comment = comment };
    }
    if (startsWord(code, ".macro")) return .{ .kind = .code, .a = try macroHeader(arena, code), .comment = comment };
    if (code[0] == '.' or code[0] == '}') return .{ .kind = .code, .a = try fold(arena, code), .comment = comment };

    // [label:] [instruction | macro call]
    var p: usize = 0;
    while (p < code.len and isIdentChar(code[p])) p += 1;
    var o: std.ArrayList(u8) = .empty;
    var body = code;
    if (p > 0 and p < code.len and code[p] == ':' and !isDigit(code[0])) {
        try o.appendSlice(arena, code[0 .. p + 1]);
        body = std.mem.trimStart(u8, code[p + 1 ..], " ");
        if (body.len == 0) return .{ .kind = .code, .a = o.items, .comment = comment };
        if (o.items.len < code_col) try o.appendNTimes(arena, ' ', code_col - o.items.len) else try o.append(arena, ' ');
    } else {
        try o.appendNTimes(arena, ' ', code_col);
    }
    try o.appendSlice(arena, try instruction(arena, body));
    return .{ .kind = .code, .a = o.items, .comment = comment };
}

/// `mnemonic op, op` or `name(arg, arg)`.
fn instruction(arena: std.mem.Allocator, s: []const u8) ![]const u8 {
    var p: usize = 0;
    while (p < s.len and isIdentChar(s[p])) p += 1;
    if (p > 0 and p < s.len and s[p] == '(') {
        if (matchParen(s, p)) |close| {
            if (std.mem.trim(u8, s[close + 1 ..], " ").len == 0) {
                return std.fmt.allocPrint(arena, "{s}({s})", .{ s[0..p], try joinTop(arena, s[p + 1 .. close]) });
            }
        }
        return fold(arena, s);
    }
    const rest = std.mem.trim(u8, s[p..], " ");
    if (p == 0 or rest.len == 0) return fold(arena, s);
    return std.fmt.allocPrint(arena, "{s} {s}", .{ s[0..p], try joinTop(arena, rest) });
}

fn macroHeader(arena: std.mem.Allocator, s: []const u8) ![]const u8 {
    var p: usize = ".macro".len;
    while (p < s.len and s[p] == ' ') p += 1;
    const ns = p;
    while (p < s.len and isIdentChar(s[p])) p += 1;
    const name = s[ns..p];
    while (p < s.len and s[p] == ' ') p += 1;
    if (p < s.len and s[p] == '(') {
        const close = matchParen(s, p) orelse return fold(arena, s);
        const tail = std.mem.trim(u8, s[close + 1 ..], " ");
        return std.fmt.allocPrint(arena, ".macro {s}({s}) {s}", .{ name, try joinTop(arena, s[p + 1 .. close]), tail });
    }
    return std.fmt.allocPrint(arena, ".macro {s} {s}", .{ name, std.mem.trim(u8, s[p..], " ") });
}

fn matchParen(s: []const u8, open: usize) ?usize {
    var depth: usize = 0;
    for (s[open..], open..) |c, k| {
        if (c == '(') depth += 1;
        if (c == ')') {
            depth -= 1;
            if (depth == 0) return k;
        }
    }
    return null;
}

/// Top-level comma list, one space after each comma, spaces folded.
fn joinTop(arena: std.mem.Allocator, s: []const u8) ![]const u8 {
    var o: std.ArrayList(u8) = .empty;
    var depth: usize = 0;
    var quoted = false;
    var start: usize = 0;
    var k: usize = 0;
    while (k <= s.len) : (k += 1) {
        const at_end = k == s.len;
        const c: u8 = if (at_end) ',' else s[k];
        if (c == '"') quoted = !quoted;
        if (!quoted and c == '(') depth += 1;
        if (!quoted and c == ')' and depth > 0) depth -= 1;
        if (at_end or (c == ',' and depth == 0 and !quoted)) {
            const part = std.mem.trim(u8, s[start..k], " ");
            if (o.items.len > 0 or !at_end or part.len > 0) {
                if (start > 0) try o.appendSlice(arena, ", ");
                try o.appendSlice(arena, try fold(arena, part));
            }
            start = k + 1;
        }
    }
    return o.items;
}

/// Folds runs of spaces outside quotes to one.
fn fold(arena: std.mem.Allocator, s: []const u8) ![]const u8 {
    var o: std.ArrayList(u8) = .empty;
    var quoted = false;
    for (s, 0..) |c, k| {
        if (c == '"') quoted = !quoted;
        if (c == ' ' and !quoted and k > 0 and s[k - 1] == ' ') continue;
        try o.append(arena, c);
    }
    return o.items;
}

fn startsWord(t: []const u8, w: []const u8) bool {
    return std.mem.startsWith(u8, t, w) and (t.len == w.len or !isIdentChar(t[w.len]));
}

fn isDigit(c: u8) bool {
    return c >= '0' and c <= '9';
}

fn isIdentChar(c: u8) bool {
    return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or isDigit(c) or c == '_' or c == '.';
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

fn expectFmt(src: []const u8, want: []const u8) !void {
    const got = try format(testing.allocator, src);
    defer testing.allocator.free(got);
    try testing.expectEqualStrings(want, got);
    const again = try format(testing.allocator, got);
    defer testing.allocator.free(again);
    try testing.expectEqualStrings(got, again); // idempotent
}

test "columns: label, code, comment" {
    try expectFmt("skip: alu s1,s1 ,  s2,#14 ;add\n", "skip:   alu s1, s1, s2, #14     ;add\n");
    try expectFmt("\tldi s2, #1\n", "        ldi s2, #1\n");
    try expectFmt("a_long_label: x17.halt\n", "a_long_label: x17.halt\n");
    try expectFmt("over:\n", "over:\n");
}

test "a long line keeps one space before its comment" {
    try expectFmt("ldi s2, #(1 << 2 | 1 << 3 | 1 << 4 | 1 << 5) ; bits\n", "        ldi s2, #(1 << 2 | 1 << 3 | 1 << 4 | 1 << 5) ; bits\n");
}

test "operands: commas inside parentheses are left alone, spaces fold" {
    try expectFmt("ldi s1, #(  A  +  B )\n", "        ldi s1, #( A + B )\n");
    try expectFmt("add( s1 ,s1,s2 )\n", "        add(s1, s1, s2)\n");
    try expectFmt("f((1,2), 3)\n", "        f((1,2), 3)\n");
}

test ".const runs align their names" {
    try expectFmt(".const A = 1\n.const LONG   =  2 ; two\n\n.const B = 3\n",
        \\.const A    = 1
        \\.const LONG = 2                 ; two
        \\
        \\.const B = 3
        \\
    );
}

test "macro headers, bodies and closers" {
    try expectFmt(".macro  add ( d,a , b )  {\n alu d, a, b, #ADD\n}\n",
        \\.macro add(d, a, b) {
        \\        alu d, a, b, #ADD
        \\}
        \\
    );
}

test "comments, blank runs, file edges" {
    try expectFmt("\n\n; top\n\n\n   ; inner\n#hash\nx17.halt\n\n\n",
        \\; top
        \\
        \\        ; inner
        \\#hash
        \\        x17.halt
        \\
    );
}

test "directives and include paths keep their quoted text" {
    try expectFmt("   .include   \"lib/a b.sm\"  ; x\n", ".include \"lib/a b.sm\"           ; x\n");
    try expectFmt(".assert  end - start  ==  3\n", ".assert end - start == 3\n");
}
