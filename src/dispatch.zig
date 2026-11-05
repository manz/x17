//! x17 CLI dispatcher.  Follows the shiro pattern: a Verb tree, each verb
//! parses its own flags with zig-clap, and `run(gpa, args, out)` walks the
//! tree and calls the matched handler.

const std = @import("std");
const clap = @import("clap");
const isa = @import("isa");
const asm_mod = @import("asm");
const emu = @import("emu");

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
    .{ .name = "forms", .help = "list known forms", .run = &cmdForms },
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
    const n = asm_mod.renderDiagAt(&buf, path, fe.line, fe.line_text, fe.diag);
    try out.writeAll(buf[0..n]);
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

    const result = try asm_mod.assembleFile(gpa, src);
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

    const result = try asm_mod.assembleFile(gpa, src);
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

    const bundles = try emu.runLinear(gpa, &state, words);

    try out.print("ran {d} bundles on lane {d} (V[{d}] init = 0x{x})\n\n", .{
        bundles, lane, lane, v_init,
    });

    // Non-zero scalar regs.
    var any_s = false;
    for (state.s, 0..) |val, idx| {
        if (val != 0) {
            if (!any_s) {
                try out.writeAll("s regs:\n");
                any_s = true;
            }
            try out.print("  s{d:<2} = 0x{x:0>6}\n", .{ idx, val });
        }
    }
    if (!any_s) try out.writeAll("s regs: all zero\n");

    // Y memory.
    if (state.y_mem.count() == 0) {
        try out.writeAll("Y mem: no writes\n");
    } else {
        try out.print("Y mem ({d} writes):\n", .{state.y_mem.count()});
        var kit = state.y_mem.keyIterator();
        // Collect + sort for stable output.
        var keys: [128]u32 = undefined;
        var n: usize = 0;
        while (kit.next()) |k| {
            if (n >= keys.len) break;
            keys[n] = k.*;
            n += 1;
        }
        std.mem.sort(u32, keys[0..n], {}, std.sort.asc(u32));
        for (keys[0..n]) |k| {
            const v = state.y_mem.get(k).?;
            try out.print("  Y:0x{x:<4} = 0x{x:0>6}\n", .{ k, v });
        }
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
