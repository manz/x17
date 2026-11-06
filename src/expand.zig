//! Source expansion, before the assembler's two passes (a816 syntax):
//!
//!   .include "path"              paste a file; the path is relative to the
//!                                including file.  Cycles are refused.
//!   .macro name(a, b) {          define; the body runs to a line holding
//!       ldi a, #b                only `}`.  Parameters substitute as whole
//!   }                            identifiers.
//!   name(s1, 4)                  expand; may follow a label.
//!
//! Labels a macro body defines are renamed per expansion, so a macro used
//! twice does not bind a label twice.  Every expanded line keeps where it came
//! from (file, line) and, inside a macro, the call that produced it.

const std = @import("std");
const asm_mod = @import("asm.zig");

const Diagnostic = asm_mod.Diagnostic;
const ErrorCode = asm_mod.ErrorCode;

pub const Loader = struct {
    ctx: ?*anyopaque = null,
    /// The file's bytes, allocated with `gpa`, or null when it is missing.
    load: *const fn (ctx: ?*anyopaque, gpa: std.mem.Allocator, path: []const u8) anyerror!?[]u8,
};

pub const Site = struct { file: []const u8, line: usize };
pub const Call = struct { name: []const u8, site: Site };

pub const Line = struct {
    text: []const u8,
    site: Site,
    call: ?Call = null,
};

pub const Failure = struct {
    line: Line,
    diag: Diagnostic,
};

const Macro = struct {
    params: []const []const u8,
    labels: []const []const u8,
    body: []const Line,
};

const max_include_depth = 16;
const max_macro_depth = 32;

pub const Expander = struct {
    /// Every line and text lives here; free it after assembling.
    arena: std.mem.Allocator,
    gpa: std.mem.Allocator,
    loader: ?Loader,
    macros: std.StringHashMapUnmanaged(Macro) = .empty,
    includes: std.ArrayList([]const u8) = .empty,
    out: std.ArrayList(Line) = .empty,
    expansions: usize = 0,
    failure: ?Failure = null,

    pub const Error = error{ Expand, OutOfMemory };

    pub fn file(self: *Expander, path: []const u8, source: []const u8) Error!void {
        try self.includes.append(self.arena, path);
        defer _ = self.includes.pop();
        var lines: std.ArrayList(Line) = .empty;
        var it = std.mem.splitScalar(u8, source, '\n');
        var n: usize = 0;
        while (it.next()) |raw| {
            n += 1;
            try lines.append(self.arena, .{ .text = std.mem.trimEnd(u8, raw, "\r"), .site = .{ .file = path, .line = n } });
        }
        try self.block(lines.items, null, 0);
    }

    fn block(self: *Expander, lines: []const Line, call: ?Call, depth: usize) Error!void {
        var i: usize = 0;
        while (i < lines.len) : (i += 1) {
            var l = lines[i];
            if (call) |c| l.call = c;
            const t = std.mem.trim(u8, l.text, " \t");
            if (t.len == 0 or t[0] == ';' or t[0] == '#') continue;
            if (startsWord(t, ".macro")) {
                i = try self.define(lines, i);
                continue;
            }
            if (startsWord(t, ".include")) {
                try self.include(l, t, depth);
                continue;
            }
            if (try self.expandCall(l, depth)) continue;
            try self.out.append(self.arena, l);
        }
    }

    fn fail(self: *Expander, l: Line, code: ErrorCode, start: usize, end: usize, msg: []const u8, hint: []const u8) Error {
        var d = Diagnostic{ .code = code, .span = .{ .start = start, .end = end } };
        d.setMsg(msg);
        d.setHint(hint);
        self.failure = .{ .line = l, .diag = d };
        return error.Expand;
    }

    fn include(self: *Expander, l: Line, t: []const u8, depth: usize) Error!void {
        const off = l.text.len - std.mem.trimStart(u8, l.text, " \t").len;
        const rest = std.mem.trim(u8, t[".include".len..], " \t");
        const q0 = std.mem.indexOfScalar(u8, rest, '"');
        const q1 = if (q0) |a| std.mem.indexOfScalarPos(u8, rest, a + 1, '"') else null;
        if (q0 != 0 or q1 == null) return self.fail(l, .e0025_include_failed, off, l.text.len, "expected a quoted path", ".include \"file.sm\"");
        const name = rest[1..q1.?];
        const span_start = off + (t.len - rest.len) + 1;
        const span_end = span_start + name.len;
        const tail = std.mem.trim(u8, rest[q1.? + 1 ..], " \t");
        if (tail.len != 0 and tail[0] != ';') return self.fail(l, .e0007_extra_tokens, span_end + 1, l.text.len, "unexpected text after the path", "one path per .include");
        const loader = self.loader orelse
            return self.fail(l, .e0025_include_failed, span_start, span_end, "this source cannot include files", "assemble from a file path");
        const dir = std.fs.path.dirname(l.site.file) orelse "";
        const path = if (dir.len == 0 or std.fs.path.isAbsolute(name)) name else try std.fs.path.join(self.arena, &.{ dir, name });
        for (self.includes.items) |p| {
            if (std.mem.eql(u8, p, path)) return self.fail(l, .e0026_include_cycle, span_start, span_end, "this file is already being included", "an include cycle never ends");
        }
        if (self.includes.items.len >= max_include_depth)
            return self.fail(l, .e0026_include_cycle, span_start, span_end, "includes nest too deep", "at most 16 levels");
        const bytes = (loader.load(loader.ctx, self.gpa, path) catch null) orelse {
            var buf: [128]u8 = undefined;
            const m = std.fmt.bufPrint(&buf, "cannot read '{s}'", .{path}) catch "cannot read the file";
            return self.fail(l, .e0025_include_failed, span_start, span_end, m, "paths are relative to the including file");
        };
        defer self.gpa.free(bytes);
        const owned = try self.arena.dupe(u8, bytes);
        _ = depth;
        try self.file(path, owned);
    }

    /// `.macro name(a, b) {` through a line holding only `}`.  Returns the
    /// index of the closing line.
    fn define(self: *Expander, lines: []const Line, at: usize) Error!usize {
        const l = lines[at];
        const off = l.text.len - std.mem.trimStart(u8, l.text, " \t").len;
        var p = off + ".macro".len;
        skipSpace(l.text, &p);
        const name_start = p;
        while (p < l.text.len and isIdentChar(l.text[p])) p += 1;
        const name = l.text[name_start..p];
        const hint = ".macro name(a, b) {";
        if (name.len == 0 or name[0] == '.' or isDigit(name[0]))
            return self.fail(l, .e0027_bad_macro, name_start, @max(p, name_start + 1), "expected a macro name", hint);
        skipSpace(l.text, &p);
        var params: std.ArrayList([]const u8) = .empty;
        if (p < l.text.len and l.text[p] == '(') {
            p += 1;
            while (true) {
                skipSpace(l.text, &p);
                if (p < l.text.len and l.text[p] == ')' and params.items.len == 0) break;
                const ps = p;
                while (p < l.text.len and isIdentChar(l.text[p])) p += 1;
                if (p == ps) return self.fail(l, .e0027_bad_macro, ps, ps + 1, "expected a parameter name", hint);
                try params.append(self.arena, l.text[ps..p]);
                skipSpace(l.text, &p);
                if (p < l.text.len and l.text[p] == ',') {
                    p += 1;
                    continue;
                }
                break;
            }
            if (p >= l.text.len or l.text[p] != ')') return self.fail(l, .e0027_bad_macro, p, p + 1, "expected ')'", hint);
            p += 1;
            skipSpace(l.text, &p);
        }
        if (p >= l.text.len or l.text[p] != '{') return self.fail(l, .e0027_bad_macro, p, p + 1, "expected '{' to open the body", hint);
        p += 1;
        const after = std.mem.trim(u8, l.text[p..], " \t");
        if (after.len != 0 and after[0] != ';') return self.fail(l, .e0027_bad_macro, p, l.text.len, "the body starts on the next line", hint);

        var body: std.ArrayList(Line) = .empty;
        var labels: std.ArrayList([]const u8) = .empty;
        var i = at + 1;
        while (i < lines.len) : (i += 1) {
            const t = std.mem.trim(u8, lines[i].text, " \t");
            if (std.mem.eql(u8, t, "}")) break;
            if (startsWord(t, ".macro")) {
                const o = lines[i].text.len - std.mem.trimStart(u8, lines[i].text, " \t").len;
                return self.fail(lines[i], .e0027_bad_macro, o, o + ".macro".len, "a macro cannot define a macro", "close the outer body with '}' first");
            }
            if (labelOf(t)) |lab| try labels.append(self.arena, lab);
            try body.append(self.arena, lines[i]);
        } else return self.fail(l, .e0027_bad_macro, name_start, name_start + name.len, "this macro body is never closed", "end the body with a line holding only '}'");

        const gop = try self.macros.getOrPut(self.arena, name);
        if (gop.found_existing) return self.fail(l, .e0019_duplicate_symbol, name_start, name_start + name.len, "this macro is already defined", "macro names are unique");
        gop.value_ptr.* = .{ .params = params.items, .labels = labels.items, .body = body.items };
        return i;
    }

    /// `[label:] name(args)`: expands when `name` is a defined macro.
    fn expandCall(self: *Expander, l: Line, depth: usize) Error!bool {
        var p: usize = 0;
        skipSpace(l.text, &p);
        var ns = p;
        while (p < l.text.len and isIdentChar(l.text[p])) p += 1;
        var label: ?[]const u8 = null;
        if (p < l.text.len and l.text[p] == ':' and p > ns) {
            label = l.text[0 .. p + 1];
            p += 1;
            skipSpace(l.text, &p);
            ns = p;
            while (p < l.text.len and isIdentChar(l.text[p])) p += 1;
        }
        const name = l.text[ns..p];
        const m = self.macros.get(name) orelse return false;
        if (p >= l.text.len or l.text[p] != '(')
            return self.fail(l, .e0028_macro_call, ns, p, "a macro is called with parentheses", "name(arg, ...)");
        p += 1;
        var args: std.ArrayList([]const u8) = .empty;
        var nest: usize = 0;
        var as = p;
        var closed = false;
        while (p < l.text.len) : (p += 1) {
            switch (l.text[p]) {
                '(' => nest += 1,
                ')' => if (nest == 0) {
                    closed = true;
                    break;
                } else {
                    nest -= 1;
                },
                ',' => if (nest == 0) {
                    try args.append(self.arena, std.mem.trim(u8, l.text[as..p], " \t"));
                    as = p + 1;
                },
                ';' => break,
                else => {},
            }
        }
        if (!closed) return self.fail(l, .e0028_macro_call, ns, p, "the argument list is never closed", "name(arg, ...)");
        const last = std.mem.trim(u8, l.text[as..p], " \t");
        if (last.len != 0 or args.items.len != 0) try args.append(self.arena, last);
        const tail = std.mem.trim(u8, l.text[p + 1 ..], " \t");
        if (tail.len != 0 and tail[0] != ';') return self.fail(l, .e0007_extra_tokens, p + 1, l.text.len, "unexpected text after the call", "one call per line");
        if (args.items.len != m.params.len) {
            var buf: [128]u8 = undefined;
            const msg = std.fmt.bufPrint(&buf, "'{s}' takes {d} argument(s), found {d}", .{ name, m.params.len, args.items.len }) catch "wrong argument count";
            return self.fail(l, .e0028_macro_call, ns, p + 1, msg, "match the .macro header");
        }
        if (depth >= max_macro_depth) return self.fail(l, .e0028_macro_call, ns, ns + name.len, "macros expand too deep", "a macro that calls itself never ends");

        if (label) |lab| try self.out.append(self.arena, .{ .text = lab, .site = l.site, .call = l.call });
        self.expansions += 1;
        var subs: std.StringHashMapUnmanaged([]const u8) = .empty;
        for (m.params, args.items) |k, v| try subs.put(self.arena, k, v);
        for (m.labels) |k| try subs.put(self.arena, k, try std.fmt.allocPrint(self.arena, "__m{d}_{s}", .{ self.expansions, k }));
        var body: std.ArrayList(Line) = .empty;
        for (m.body) |b| try body.append(self.arena, .{ .text = try substitute(self.arena, b.text, &subs), .site = b.site });
        try self.block(body.items, .{ .name = name, .site = l.site }, depth + 1);
        return true;
    }
};

fn startsWord(t: []const u8, w: []const u8) bool {
    return std.mem.startsWith(u8, t, w) and (t.len == w.len or !isIdentChar(t[w.len]));
}

fn skipSpace(s: []const u8, p: *usize) void {
    while (p.* < s.len and (s[p.*] == ' ' or s[p.*] == '\t')) p.* += 1;
}

fn isDigit(c: u8) bool {
    return c >= '0' and c <= '9';
}

fn isIdentChar(c: u8) bool {
    return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or isDigit(c) or c == '_' or c == '.';
}

/// `name:` at the start of a trimmed line.
fn labelOf(t: []const u8) ?[]const u8 {
    var p: usize = 0;
    while (p < t.len and isIdentChar(t[p])) p += 1;
    if (p == 0 or p >= t.len or t[p] != ':' or t[0] == '.' or isDigit(t[0])) return null;
    return t[0..p];
}

/// Replaces whole identifiers found in `subs`, up to a `;` comment.
fn substitute(arena: std.mem.Allocator, text: []const u8, subs: *const std.StringHashMapUnmanaged([]const u8)) ![]const u8 {
    var out: std.ArrayList(u8) = .empty;
    var p: usize = 0;
    while (p < text.len) {
        const c = text[p];
        if (c == ';') {
            try out.appendSlice(arena, text[p..]);
            break;
        }
        if (isIdentChar(c)) {
            const s = p;
            while (p < text.len and isIdentChar(text[p])) p += 1;
            const word = text[s..p];
            const rep = if (isDigit(word[0])) null else subs.get(word);
            try out.appendSlice(arena, rep orelse word);
            continue;
        }
        try out.append(arena, c);
        p += 1;
    }
    return out.items;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const Files = struct {
    entries: []const struct { []const u8, []const u8 },
    fn load(ctx: ?*anyopaque, gpa: std.mem.Allocator, path: []const u8) anyerror!?[]u8 {
        const self: *const Files = @ptrCast(@alignCast(ctx.?));
        for (self.entries) |e| if (std.mem.eql(u8, e[0], path)) return try gpa.dupe(u8, e[1]);
        return null;
    }
};

fn run(arena: std.mem.Allocator, files: ?*const Files, src: []const u8) !Expander {
    var x: Expander = .{
        .arena = arena,
        .gpa = testing.allocator,
        .loader = if (files) |f| .{ .ctx = @constCast(f), .load = Files.load } else null,
    };
    x.file("main.sm", src) catch |e| switch (e) {
        error.Expand => return x,
        else => return e,
    };
    return x;
}

fn expectLines(x: *const Expander, want: []const []const u8) !void {
    try testing.expect(x.failure == null);
    try testing.expectEqual(want.len, x.out.items.len);
    for (want, x.out.items) |w, l| try testing.expectEqualStrings(w, std.mem.trim(u8, l.text, " \t"));
}

test "a macro substitutes parameters as whole identifiers" {
    var a = std.heap.ArenaAllocator.init(testing.allocator);
    defer a.deinit();
    const x = try run(a.allocator(), null,
        \\.macro set(r, v) {
        \\    ldi r, #v     ; r stays in comments
        \\}
        \\set(s1, 4)
        \\set(s2, (1 << 3))
    );
    try expectLines(&x, &.{ "ldi s1, #4     ; r stays in comments", "ldi s2, #(1 << 3)     ; r stays in comments" });
    try testing.expectEqualStrings("set", x.out.items[0].call.?.name);
    try testing.expectEqual(@as(usize, 4), x.out.items[0].call.?.site.line);
    try testing.expectEqual(@as(usize, 2), x.out.items[0].site.line);
}

test "labels inside a macro are renamed per expansion; a call keeps its label" {
    var a = std.heap.ArenaAllocator.init(testing.allocator);
    defer a.deinit();
    const x = try run(a.allocator(), null,
        \\.macro spin() {
        \\loop:   bcc #0, loop
        \\}
        \\top: spin()
        \\spin()
    );
    try expectLines(&x, &.{ "top:", "__m1_loop:   bcc #0, __m1_loop", "__m2_loop:   bcc #0, __m2_loop" });
}

test "macros nest, and the identifier match leaves x17.halt alone" {
    var a = std.heap.ArenaAllocator.init(testing.allocator);
    defer a.deinit();
    const x = try run(a.allocator(), null,
        \\.macro halt() {
        \\    x17.halt
        \\}
        \\.macro both(r) {
        \\    ldi r, #1
        \\    halt()
        \\}
        \\both(s3)
    );
    try expectLines(&x, &.{ "ldi s3, #1", "x17.halt" });
}

test "include pastes a file relative to the includer" {
    var a = std.heap.ArenaAllocator.init(testing.allocator);
    defer a.deinit();
    const files: Files = .{ .entries = &.{
        .{ "lib/a.sm", ".include \"b.sm\"\nldi s1, #1" },
        .{ "lib/b.sm", "ldi s2, #2" },
    } };
    const x = try run(a.allocator(), &files, ".include \"lib/a.sm\"\nx17.halt");
    try expectLines(&x, &.{ "ldi s2, #2", "ldi s1, #1", "x17.halt" });
    try testing.expectEqualStrings("lib/b.sm", x.out.items[0].site.file);
}

fn expectFail(files: ?*const Files, src: []const u8, code: ErrorCode, caret: []const u8) !void {
    var a = std.heap.ArenaAllocator.init(testing.allocator);
    defer a.deinit();
    const x = try run(a.allocator(), files, src);
    const f = x.failure orelse return error.TestExpectedFailure;
    try testing.expectEqual(code, f.diag.code);
    try testing.expectEqualStrings(caret, f.line.text[f.diag.span.start..f.diag.span.end]);
}

test "include errors: missing file, cycle, no loader, unquoted" {
    const files: Files = .{ .entries = &.{
        .{ "a.sm", ".include \"b.sm\"" },
        .{ "b.sm", ".include \"a.sm\"" },
    } };
    try expectFail(&files, ".include \"nope.sm\"", .e0025_include_failed, "nope.sm");
    try expectFail(&files, ".include \"a.sm\"", .e0026_include_cycle, "a.sm");
    try expectFail(null, ".include \"a.sm\"", .e0025_include_failed, "a.sm");
    try expectFail(&files, ".include a.sm", .e0025_include_failed, ".include a.sm");
}

test "macro errors: unclosed, nested, duplicate, argument count, recursion" {
    try expectFail(null, ".macro m() {\nldi s1, #1", .e0027_bad_macro, "m");
    try expectFail(null, ".macro m() {\n.macro n() {\n}\n}", .e0027_bad_macro, ".macro");
    try expectFail(null, ".macro m() {\n}\n.macro m() {\n}", .e0019_duplicate_symbol, "m");
    try expectFail(null, ".macro m(a) {\n}\nm(1, 2)", .e0028_macro_call, "m(1, 2)");
    try expectFail(null, ".macro m() {\nm()\n}\nm()", .e0028_macro_call, "m");
    try expectFail(null, ".macro m(a {\n}", .e0027_bad_macro, "{");
}
