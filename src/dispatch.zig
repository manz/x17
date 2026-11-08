//! x17 CLI dispatcher.  Follows the shiro pattern: a Verb tree, each verb
//! parses its own flags with zig-clap, and `run(gpa, args, out)` walks the
//! tree and calls the matched handler.

const std = @import("std");
const clap = @import("clap");
const isa = @import("isa");
const asm_mod = @import("asm");
const emu = @import("emu");
const disasm = @import("disasm");
const fmt_mod = @import("fmt");
const doc = @import("doc");

const Writer = std.Io.Writer;
const Handler = *const fn (gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) anyerror!void;

pub const Verb = struct {
    name: []const u8,
    help: []const u8,
    run: ?Handler = null,
    sub: ?[]const Verb = null,
};

// ---------------------------------------------------------------------------
// Verb tree
// ---------------------------------------------------------------------------

pub const top = [_]Verb{
    .{ .name = "asm", .help = "assemble a .sm file; prints hex syllables", .run = &cmdAsm },
    .{ .name = "run", .help = "assemble + execute a .sm file; prints s/Y state", .run = &cmdRun },
    .{ .name = "fmt", .help = "format .sm files in place (--check: report only)", .run = &cmdFmt },
    .{ .name = "disasm", .help = "decode syllables (hex arg or .bin file) to .sm", .run = &cmdDisasm },
    .{ .name = "forms", .help = "list known forms", .run = &cmdForms },
    .{ .name = "doc", .help = "print the SM1 instruction reference (--check <file>: diff only)", .run = &cmdDoc },
};

// ---------------------------------------------------------------------------
// Dispatcher
// ---------------------------------------------------------------------------

pub fn run(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    if (args.len == 0) return printHelp(&top, "x17", out);
    return dispatchInto(gpa, &top, args, "x17", out);
}

fn dispatchInto(
    gpa: std.mem.Allocator,
    verbs: []const Verb,
    args: []const []const u8,
    path: []const u8,
    out: *Writer,
) !void {
    const head = args[0];
    if (std.mem.eql(u8, head, "-h") or std.mem.eql(u8, head, "--help")) {
        return printHelp(verbs, path, out);
    }
    for (verbs) |v| {
        if (!std.mem.eql(u8, v.name, head)) continue;
        if (v.sub) |sub| {
            if (args.len == 1) return printHelp(sub, v.name, out);
            return dispatchInto(gpa, sub, args[1..], v.name, out);
        }
        if (v.run) |h| return h(gpa, args[1..], out);
        return error.NoHandler;
    }
    try out.print("x17: unknown verb '{s}'\n", .{head});
    try printHelp(verbs, path, out);
    return error.UnknownVerb;
}

fn printHelp(verbs: []const Verb, path: []const u8, out: *Writer) !void {
    try out.print("usage: {s} <verb> [args]\n\nverbs:\n", .{path});
    for (verbs) |v| {
        const marker: []const u8 = if (v.sub != null) "+" else " ";
        try out.print("  {s} {s:<8}  {s}\n", .{ marker, v.name, v.help });
    }
}

// ---------------------------------------------------------------------------
// clap plumbing
// ---------------------------------------------------------------------------

pub const SliceIter = struct {
    items: []const []const u8 = &.{},
    idx: usize = 0,
    pub fn next(self: *@This()) ?[]const u8 {
        if (self.idx >= self.items.len) return null;
        defer self.idx += 1;
        return self.items[self.idx];
    }
};

fn parseVerb(
    comptime params: []const clap.Param(clap.Help),
    gpa: std.mem.Allocator,
    args: []const []const u8,
    verb_name: []const u8,
    out: *Writer,
    iter: *SliceIter,
) !clap.ResultEx(clap.Help, params, clap.parsers.default) {
    iter.* = .{ .items = args };
    var diag = clap.Diagnostic{};
    const parsed = clap.parseEx(
        clap.Help,
        params,
        clap.parsers.default,
        iter,
        .{ .diagnostic = &diag, .allocator = gpa },
    ) catch |err| {
        try out.print("{s}: ", .{verb_name});
        diag.report(out, err) catch {};
        try out.print("\ntry `{s} --help`\n", .{verb_name});
        return err;
    };
    return parsed;
}

// ---------------------------------------------------------------------------
// Verb: asm
// ---------------------------------------------------------------------------

fn readSource(gpa: std.mem.Allocator, path: []const u8) ![]u8 {
    const path_z = try gpa.allocSentinel(u8, path.len, 0);
    defer gpa.free(path_z);
    @memcpy(path_z[0..path.len], path);
    const fd = std.c.open(path_z, @bitCast(@as(i32, 0)), @as(std.c.mode_t, 0));
    if (fd < 0) return error.FileNotFound;
    defer _ = std.c.close(fd);
    var list: std.ArrayList(u8) = .empty;
    errdefer list.deinit(gpa);
    var buf: [4096]u8 = undefined;
    while (true) {
        const n = std.c.read(fd, &buf, buf.len);
        if (n < 0) return error.ReadFailed;
        if (n == 0) break;
        try list.appendSlice(gpa, buf[0..@intCast(n)]);
        if (list.items.len > 1 * 1024 * 1024) return error.FileTooLarge;
    }
    return try list.toOwnedSlice(gpa);
}

fn renderFileError(fe: asm_mod.FileError, path: []const u8, out: *Writer) !void {
    var buf: [1024]u8 = undefined;
    const file = if (fe.file.len > 0) fe.file.get() else path;
    const n = asm_mod.renderDiagAt(&buf, file, fe.line, fe.lineText(), fe.diag);
    try out.writeAll(buf[0..n]);
    if (fe.call_line > 0) {
        try out.print("   = note: in macro '{s}', called at {s}:{d}\n", .{ fe.call_name.get(), fe.call_file.get(), fe.call_line });
    }
}

fn loadInclude(ctx: ?*anyopaque, gpa: std.mem.Allocator, path: []const u8) anyerror!?[]u8 {
    _ = ctx;
    return readSource(gpa, path) catch |e| switch (e) {
        error.FileNotFound => null,
        else => e,
    };
}

fn assemblePath(gpa: std.mem.Allocator, src: []const u8, path: []const u8) !asm_mod.FileResult {
    return asm_mod.assembleFileWith(gpa, src, .{ .path = path, .loader = .{ .load = loadInclude } });
}

const asm_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 asm`
    \\-f, --format <str>       output format: hex (default), c, bin
    \\<str>                    source file path (.sm)
    \\
);

fn cmdAsm(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&asm_params, gpa, args, "asm", out, &iter);
    defer parsed.deinit();

    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 asm [--format hex|c|bin] <file.sm>\n");
        return;
    }
    if (parsed.positionals[0] == null) {
        try out.writeAll("asm: missing <file.sm>\n");
        return error.MissingFile;
    }
    const path = parsed.positionals[0].?;
    const format: []const u8 = parsed.args.format orelse "hex";

    const src = readSource(gpa, path) catch |e| {
        try out.print("asm: cannot read '{s}': {s}\n", .{ path, @errorName(e) });
        return e;
    };
    defer gpa.free(src);

    const result = try assemblePath(gpa, src, path);
    const words = switch (result) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            return error.AssemblyFailed;
        },
    };
    defer gpa.free(words);

    if (std.mem.eql(u8, format, "hex")) {
        for (words) |w| try out.print("{x:0>8}\n", .{w});
    } else if (std.mem.eql(u8, format, "c")) {
        try out.writeAll("static const uint32_t program[] = {\n");
        for (words) |w| try out.print("    0x{x:0>8},\n", .{w});
        try out.writeAll("};\n");
    } else if (std.mem.eql(u8, format, "bin")) {
        for (words) |w| {
            var le: [4]u8 = undefined;
            std.mem.writeInt(u32, &le, w, .little);
            try out.writeAll(&le);
        }
    } else {
        try out.print("asm: unknown --format '{s}' (hex|c|bin)\n", .{format});
        return error.BadFormat;
    }
}

// ---------------------------------------------------------------------------
// Verb: fmt
// ---------------------------------------------------------------------------

const fmt_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 fmt`
    \\    --check              list files that would change; change nothing
    \\<str>...                 source files (.sm)
    \\
);

fn cmdFmt(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&fmt_params, gpa, args, "fmt", out, &iter);
    defer parsed.deinit();

    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 fmt [--check] <file.sm>...\n");
        return;
    }
    const paths = parsed.positionals[0];
    if (paths.len == 0) {
        try out.writeAll("fmt: missing <file.sm>\n");
        return error.MissingFile;
    }
    const check = parsed.args.check != 0;
    var unformatted = false;
    for (paths) |path| {
        if (try fmtOne(gpa, path, check, out)) unformatted = true;
    }
    if (check and unformatted) return error.Unformatted;
}

/// Formats one file; returns whether it was (or would be) changed.  Refuses
/// to write a result that assembles to different bytes.
fn fmtOne(gpa: std.mem.Allocator, path: []const u8, check: bool, out: *Writer) !bool {
    const src = readSource(gpa, path) catch |e| {
        try out.print("fmt: cannot read '{s}': {s}\n", .{ path, @errorName(e) });
        return e;
    };
    defer gpa.free(src);
    const formatted = try fmt_mod.format(gpa, src);
    defer gpa.free(formatted);
    if (std.mem.eql(u8, src, formatted)) return false;

    const before = switch (try assemblePath(gpa, src, path)) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            try out.print("fmt: '{s}' does not assemble; left as it is\n", .{path});
            return error.AssemblyFailed;
        },
    };
    defer gpa.free(before);
    const after = switch (try assemblePath(gpa, formatted, path)) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            try out.print("fmt: bug: the formatted '{s}' no longer assembles; left as it is\n", .{path});
            return error.FormatChangedBytes;
        },
    };
    defer gpa.free(after);
    if (!std.mem.eql(u32, before, after)) {
        try out.print("fmt: bug: formatting '{s}' changes its bytes; left as it is\n", .{path});
        return error.FormatChangedBytes;
    }
    if (check) {
        try out.print("{s}\n", .{path});
        return true;
    }
    try writeFile(gpa, path, formatted);
    try out.print("formatted {s}\n", .{path});
    return true;
}

fn writeFile(gpa: std.mem.Allocator, path: []const u8, bytes: []const u8) !void {
    const path_z = try gpa.allocSentinel(u8, path.len, 0);
    @memcpy(path_z[0..path.len], path);
    defer gpa.free(path_z);
    const fd = std.c.open(path_z, .{ .ACCMODE = .WRONLY, .CREAT = true, .TRUNC = true }, @as(std.c.mode_t, 0o644));
    if (fd < 0) return error.WriteFailed;
    defer _ = std.c.close(fd);
    var done: usize = 0;
    while (done < bytes.len) {
        const n = std.c.write(fd, bytes[done..].ptr, bytes.len - done);
        if (n <= 0) return error.WriteFailed;
        done += @intCast(n);
    }
}

// ---------------------------------------------------------------------------
// Verb: run
// ---------------------------------------------------------------------------

const run_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 run`
    \\-l, --lane <usize>       which lane to run (0..3), default 0
    \\    --v <usize>          initial value for this lane's V register
    \\<str>                    source file path (.sm)
    \\
);

fn cmdRun(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&run_params, gpa, args, "run", out, &iter);
    defer parsed.deinit();

    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 run [--lane N] [--v K] <file.sm>\n");
        return;
    }
    if (parsed.positionals[0] == null) {
        try out.writeAll("run: missing <file.sm>\n");
        return error.MissingFile;
    }
    const path = parsed.positionals[0].?;
    const lane: u8 = @intCast(parsed.args.lane orelse 0);
    const v_init: u32 = @intCast(parsed.args.v orelse 0);

    const src = try readSource(gpa, path);
    defer gpa.free(src);

    const result = try assemblePath(gpa, src, path);
    const words = switch (result) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            return error.AssemblyFailed;
        },
    };
    defer gpa.free(words);

    var state = emu.State{};
    defer state.deinit(gpa);
    state.cur_lane = lane & 0x3;
    state.v[lane & 0x3] = v_init;

    // Codec programs loop: allow a long run (a runaway still stops at the cap).
    const bundles = try emu.runFrom(gpa, &state, words, 0, 1 << 24);

    try out.print("ran {d} bundles on lane {d} (V[{d}] init = 0x{x})\n\n", .{
        bundles, lane, lane, v_init,
    });

    try printBank(out, "s regs", "s", &state.s);
    try printBank(out, "e regs", "e", &state.e);
    try printBank(out, "R regs", "R", &state.r);
    try printBank(out, "A regs", "A", &state.a);
    try printMemory(gpa, out, "Y mem", "Y", &state.y_mem);
    try printMemory(gpa, out, "X mem", "X", &state.x_mem);
}

fn printBank(out: *Writer, label: []const u8, prefix: []const u8, regs: []const u32) !void {
    var any = false;
    for (regs, 0..) |val, idx| {
        if (val == 0) continue;
        if (!any) {
            try out.print("{s}:\n", .{label});
            any = true;
        }
        try out.print("  {s}{d:<2} = 0x{x:0>6}\n", .{ prefix, idx, val });
    }
    if (!any) try out.print("{s}: all zero\n", .{label});
}

fn printMemory(
    gpa: std.mem.Allocator,
    out: *Writer,
    label: []const u8,
    prefix: []const u8,
    mem: *const std.AutoHashMapUnmanaged(u32, u32),
) !void {
    if (mem.count() == 0) {
        try out.print("{s}: no writes\n", .{label});
        return;
    }
    try out.print("{s} ({d} writes):\n", .{ label, mem.count() });
    // Every write, sorted: a codec run writes thousands, and a partial dump hides them.
    const keys = try gpa.alloc(u32, mem.count());
    defer gpa.free(keys);
    var n: usize = 0;
    var kit = mem.keyIterator();
    while (kit.next()) |k| : (n += 1) keys[n] = k.*;
    std.mem.sort(u32, keys, {}, std.sort.asc(u32));
    for (keys) |k| {
        const v = mem.get(k).?;
        try out.print("  {s}:0x{x:<4} = 0x{x:0>6}\n", .{ prefix, k, v });
    }
}

// ---------------------------------------------------------------------------
// Verb: disasm
// ---------------------------------------------------------------------------

const disasm_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 disasm`
    \\<str>...                 one or more hex syllables, OR a single .bin path
    \\
);

fn parseHexU32(s: []const u8) ?u32 {
    const t = if (std.mem.startsWith(u8, s, "0x") or std.mem.startsWith(u8, s, "0X")) s[2..] else s;
    return std.fmt.parseInt(u32, t, 16) catch null;
}

fn cmdDisasm(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&disasm_params, gpa, args, "disasm", out, &iter);
    defer parsed.deinit();

    if (parsed.args.help != 0) {
        try out.writeAll(
            \\usage:
            \\  x17 disasm <hex> [<hex> ...]    one or more u32 syllables (hex)
            \\  x17 disasm <file.bin>           little-endian u32 syllable stream
            \\
        );
        return;
    }
    if (parsed.positionals[0].len == 0) {
        try out.writeAll("disasm: need at least one <hex> syllable or <file.bin>\n");
        return error.MissingFile;
    }

    // If exactly one arg and it parses as a path (exists on disk) and not
    // as a plain hex u32, treat it as a .bin file.  Otherwise each arg is a
    // hex syllable.
    const rest = parsed.positionals[0];
    var words: []u32 = undefined;
    var words_owned = false;

    if (rest.len == 1 and parseHexU32(rest[0]) == null) {
        const buf = readSource(gpa, rest[0]) catch |e| {
            try out.print("disasm: cannot read '{s}': {s}\n", .{ rest[0], @errorName(e) });
            return e;
        };
        defer gpa.free(buf);
        if (buf.len % 4 != 0) {
            try out.print("disasm: '{s}' is not a multiple of 4 bytes ({d} bytes)\n", .{ rest[0], buf.len });
            return error.BadFormat;
        }
        const n = buf.len / 4;
        const w = try gpa.alloc(u32, n);
        for (0..n) |i| w[i] = std.mem.readInt(u32, buf[i * 4 ..][0..4], .little);
        words = w;
        words_owned = true;
    } else {
        const w = try gpa.alloc(u32, rest.len);
        for (rest, 0..) |arg, i| {
            w[i] = parseHexU32(arg) orelse {
                gpa.free(w);
                try out.print("disasm: '{s}' is not a hex u32\n", .{arg});
                return error.BadFormat;
            };
        }
        words = w;
        words_owned = true;
    }
    defer if (words_owned) gpa.free(words);

    // Walk bundle by bundle using the same length rule the emu uses.
    var i: usize = 0;
    while (i < words.len) {
        const head = words[i];
        const len = emu.bundleLen(head) orelse 1;
        const end = @min(i + len, words.len);
        const slice = words[i..end];
        var line_buf: [256]u8 = undefined;
        const n = disasm.disasmBundle(&line_buf, slice) catch |e| {
            try out.print("; disasm failed at syllable {d}: {s}\n", .{ i, @errorName(e) });
            try out.print("0x{x:0>8}\n", .{head});
            i += 1;
            continue;
        };
        try out.writeAll(line_buf[0..n]);
        try out.writeByte('\n');
        i = end;
    }
}

// ---------------------------------------------------------------------------
// Verb: forms
// ---------------------------------------------------------------------------

fn cmdForms(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    _ = gpa;
    _ = args;
    try out.print("forms ({d}):\n", .{isa.forms.len});
    for (isa.forms) |f| {
        try out.print("  {s: <8}  mask=0x{x:0>8}  value=0x{x:0>8}\n", .{ f.name, f.mask, f.value });
    }
    try out.print("\nsub-forms ({d}):\n", .{isa.subforms.len});
    for (isa.subforms) |sf| {
        try out.print("  {s: <10}  {d}sy  base[0]=0x{x:0>8}\n", .{ sf.name, sf.syllables, sf.base[0] });
    }
}

const doc_params = clap.parseParamsComptime(
    \\-h, --help               show this help
    \\    --check <str>        compare with a checked-in reference; change nothing
    \\
);

fn cmdDoc(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&doc_params, gpa, args, "doc", out, &iter);
    defer parsed.deinit();
    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 doc [--check docs/sm1/reference.md]\n");
        return;
    }
    const path = parsed.args.check orelse return doc.render(out);
    var buf: Writer.Allocating = .init(gpa);
    defer buf.deinit();
    try doc.render(&buf.writer);
    const have = readSource(gpa, path) catch |e| {
        try out.print("doc: cannot read '{s}': {s}\n", .{ path, @errorName(e) });
        return error.MissingFile;
    };
    defer gpa.free(have);
    if (!std.mem.eql(u8, have, buf.written())) {
        try out.print("doc: {s} is stale; regenerate with `x17 doc > {s}`\n", .{ path, path });
        return error.Unformatted;
    }
}
