//! x17 CLI dispatcher.  Follows the shiro pattern: a Verb tree, each verb
//! parses its own flags with zig-clap, and `run(gpa, args, out)` walks the
//! tree and calls the matched handler.

const std = @import("std");
const clap = @import("clap");
const isa = @import("isa");
const asm_mod = @import("asm");
const emu = @import("emu");
const image_mod = @import("image");
const disasm = @import("disasm.zig");
const fmt_mod = @import("fmt.zig");
const doc = @import("doc.zig");
const play_mod = @import("play.zig");
const host_mod = @import("host");

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
    .{ .name = "image", .help = "assemble passes + host data into an x17 image (+ debug sidecar)", .run = &cmdImage },
    .{ .name = "run", .help = "assemble + execute a .sm file; prints s/Y state", .run = &cmdRun },
    .{ .name = "patch", .help = "copy an x17 image with a halt + 2 nops at a decoder q", .run = &cmdPatch },
    .{ .name = "play", .help = "run an x17 image over a packet file through the host model; writes PCM", .run = &cmdPlay },
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
    return readFileMax(gpa, path, 1 * 1024 * 1024);
}

fn readFileMax(gpa: std.mem.Allocator, path: []const u8, max: usize) ![]u8 {
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
        if (list.items.len > max) return error.FileTooLarge;
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
// Verb: image
// ---------------------------------------------------------------------------

const image_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 image`
    \\-o, --out <str>          image path (required)
    \\-g, --debug <str>        debug sidecar path (JSON)
    \\<str>                    source file path (.sm)
    \\
);

fn cmdImage(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&image_params, gpa, args, "image", out, &iter);
    defer parsed.deinit();
    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 image -o <out.x17i> [-g <out.json>] <file.sm>\n");
        return;
    }
    const path = parsed.positionals[0] orelse {
        try out.writeAll("image: missing <file.sm>\n");
        return error.MissingFile;
    };
    const out_path = parsed.args.out orelse {
        try out.writeAll("image: missing -o <out.x17i>\n");
        return error.MissingFile;
    };
    const src = readSource(gpa, path) catch |e| {
        try out.print("image: cannot read '{s}': {s}\n", .{ path, @errorName(e) });
        return e;
    };
    defer gpa.free(src);
    var prog = switch (try asm_mod.assembleProgram(gpa, src, .{ .path = path, .loader = .{ .load = loadInclude } })) {
        .ok => |p| p,
        .err => |fe| {
            try renderFileError(fe, path, out);
            return error.AssemblyFailed;
        },
    };
    defer prog.deinit();
    const bytes = try image_mod.write(gpa, &prog);
    defer gpa.free(bytes);
    try writeFile(gpa, out_path, bytes);
    if (parsed.args.debug) |dbg_path| {
        const j = try image_mod.debugJson(gpa, &prog);
        defer gpa.free(j);
        try writeFile(gpa, dbg_path, j);
    }
    for (prog.passes, 0..) |*pass, i| {
        try out.print("pass {d} {s}: rows 0x{x}..0x{x}, {d} syllables, {d} data blocks\n", .{ i, @tagName(pass.kind), pass.base_row, pass.endRow(), pass.syllables.len, pass.data.len });
    }
}

// ---------------------------------------------------------------------------
// Verb: patch
// ---------------------------------------------------------------------------

const patch_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 patch`
    \\    --halt-at <usize>    decoder q to overwrite with x17.halt + 2 nops (required)
    \\-o, --out <str>          patched image path (required)
    \\<str>                    source image (.x17i)
    \\
);

fn cmdPatch(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&patch_params, gpa, args, "patch", out, &iter);
    defer parsed.deinit();
    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 patch --halt-at Q -o <out.x17i> <in.x17i>\n");
        return;
    }
    const in_path = parsed.positionals[0] orelse return missing(out, "patch: missing <in.x17i>\n");
    const out_path = parsed.args.out orelse return missing(out, "patch: missing -o <out.x17i>\n");
    const q = parsed.args.@"halt-at" orelse return missing(out, "patch: missing --halt-at Q\n");
    const bytes = try readFileMax(gpa, in_path, 64 * 1024 * 1024);
    defer gpa.free(bytes);
    var img = try image_mod.parse(gpa, bytes);
    defer img.deinit(gpa);
    var old: [image_mod.halt_patch.len]u32 = undefined;
    image_mod.patchDecoder(&img, @intCast(q), &image_mod.halt_patch, &old) catch |e| {
        try out.print("patch: q 0x{x}..0x{x} is not all in the decoder pass\n", .{ q, q + old.len - 1 });
        return e;
    };
    const new = try image_mod.serialize(gpa, &img);
    defer gpa.free(new);
    try writeFile(gpa, out_path, new);
    try out.print("halt at q 0x{x}; replaced", .{q});
    for (old) |w| try out.print(" {x:0>8}", .{w});
    try out.writeAll("\n");
}

fn missing(out: *Writer, msg: []const u8) !void {
    try out.writeAll(msg);
    return error.MissingFile;
}

// ---------------------------------------------------------------------------
// Verb: play
// ---------------------------------------------------------------------------

const play_params = clap.parseParamsComptime(
    \\-h, --help               show help for `x17 play`
    \\-p, --packets <str>      packet file: u32 LE length + bytes, repeated (required)
    \\-o, --out <str>          PCM out: s16 LE as the host drained it (required)
    \\-r, --rate <usize>       sampling rate for the parameter block, default 44100
    \\-y, --y <str>...         ADDR=VALUE Y write after loading, before the first packet (hex ok)
    \\    --max-bundles <usize> bundle budget per packet, default 2^26
    \\    --trap-list          skip refused bundles as nops; rank them by hits (hints only)
    \\    --trace <usize>      print the first N bundles run: changes, X / Y reads and writes
    \\    --trace-from <usize> with --trace: start at this packet (init passes not traced)
    \\    --stop-at <usize>    stop before the first decoder bundle at this q
    \\    --stop-skip <usize>  with --stop-at: run through this many arrivals first
    \\    --stop-after <usize> stop once this many bundles have run (init passes included)
    \\    --dump <str>         on a stop or trap, write the state (registers, X, Y) as JSON
    \\    --p-entry <usize>    p1..p15 at the start, bit k = p<k> (default 0; 0xFFFC = 803's state)
    \\    --poison <usize>     start dirty: registers, p and unstored X / Y words read as junk from this seed
    \\    --configure-arm      945's arming: one run at configure, packet 1 only queued, then one run per packet
    \\    --profile <usize>    print the decoder's hot list, N rows per table: packets, P use, functions, blocks, bundles
    \\    --debug <str>        sidecar for --profile labels and lines (default: the image path with .json)
    \\<str>                    image path (.x17i)
    \\
);

fn parseYWrite(s: []const u8) ?host_mod.Param {
    const eq = std.mem.indexOfScalar(u8, s, '=') orelse return null;
    const a = std.fmt.parseInt(u32, s[0..eq], 0) catch return null;
    const v = std.fmt.parseInt(i32, s[eq + 1 ..], 0) catch return null;
    return .{ .addr = a, .value = @bitCast(v) };
}

fn cmdPlay(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&play_params, gpa, args, "play", out, &iter);
    defer parsed.deinit();
    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 play -p <packets> -o <out.pcm> [-r HZ] [-y ADDR=VAL]... [--max-bundles N] [--trap-list] [--trace N [--trace-from PKT]] [--stop-at Q [--stop-skip K] | --stop-after N] [--dump FILE] [--p-entry BITS] [--poison SEED] [--configure-arm] [--profile N [--debug FILE]] <image.x17i>\n");
        return;
    }
    const img_path = parsed.positionals[0] orelse {
        try out.writeAll("play: missing <image.x17i>\n");
        return error.MissingFile;
    };
    const pkt_path = parsed.args.packets orelse {
        try out.writeAll("play: missing -p <packets>\n");
        return error.MissingFile;
    };
    const out_path = parsed.args.out orelse {
        try out.writeAll("play: missing -o <out.pcm>\n");
        return error.MissingFile;
    };
    var writes: std.ArrayList(host_mod.Param) = .empty;
    defer writes.deinit(gpa);
    for (parsed.args.y) |s| try writes.append(gpa, parseYWrite(s) orelse {
        try out.print("play: bad -y '{s}', want ADDR=VALUE\n", .{s});
        return error.InvalidArgument;
    });

    const img_bytes = readFileMax(gpa, img_path, 64 * 1024 * 1024) catch |e| {
        try out.print("play: cannot read '{s}': {s}\n", .{ img_path, @errorName(e) });
        return e;
    };
    defer gpa.free(img_bytes);
    var img = image_mod.parse(gpa, img_bytes) catch |e| {
        try out.print("play: '{s}' is not an x17 image: {s}\n", .{ img_path, @errorName(e) });
        return e;
    };
    defer img.deinit(gpa);
    const pkt_bytes = readFileMax(gpa, pkt_path, 1024 * 1024 * 1024) catch |e| {
        try out.print("play: cannot read '{s}': {s}\n", .{ pkt_path, @errorName(e) });
        return e;
    };
    defer gpa.free(pkt_bytes);
    const pkts = play_mod.packets(gpa, pkt_bytes) catch |e| {
        try out.print("play: '{s}': {s}\n", .{ pkt_path, @errorName(e) });
        return e;
    };
    defer gpa.free(pkts);

    var tl: play_mod.TrapList = .{ .gpa = gpa };
    defer tl.deinit();
    var tracer: play_mod.Tracer = .{ .gpa = gpa, .out = out, .left = parsed.args.trace orelse 0, .from = parsed.args.@"trace-from" };
    defer tracer.deinit();
    var dump: std.Io.Writer.Allocating = .init(gpa);
    defer dump.deinit();
    var prof: play_mod.Profile = .{ .gpa = gpa };
    defer prof.deinit();
    var res = try play_mod.play(gpa, &img, pkts, .{
        .rate_hz = @intCast(parsed.args.rate orelse 44100),
        .y_writes = writes.items,
        .max_bundles_packet = parsed.args.@"max-bundles" orelse 1 << 26,
        .trap_list = if (parsed.args.@"trap-list" != 0) &tl else null,
        .tracer = if (parsed.args.trace != null) &tracer else null,
        .stop_at = parsed.args.@"stop-at",
        .stop_skip = parsed.args.@"stop-skip" orelse 0,
        .stop_after = parsed.args.@"stop-after",
        .dump = if (parsed.args.dump != null) &dump.writer else null,
        .p_entry = @truncate(parsed.args.@"p-entry" orelse 0),
        .poison = if (parsed.args.poison) |v| @intCast(v) else null,
        .configure_arm = parsed.args.@"configure-arm" != 0,
        .profile = if (parsed.args.profile != null) &prof else null,
    });
    defer res.deinit(gpa);
    try writeFile(gpa, out_path, std.mem.sliceAsBytes(res.pcm));
    try out.print("{d}/{d} packets, {d} bundles, {d} samples out, {d} stream waits\n", .{ res.packets, pkts.len, res.bundles, res.pcm.len, res.waits });
    if (parsed.args.@"trap-list" != 0) {
        const ranked = try tl.ranked(gpa);
        defer gpa.free(ranked);
        try out.print("{d} refused forms skipped{s}:\n", .{ ranked.len, if (tl.oom) " (list incomplete: out of memory)" else "" });
        for (ranked) |e| {
            try out.print("  {d:>10} hits  first q {d} (0x{x}):", .{ e.hits, e.first_pc, e.first_pc });
            for (e.syllables[0..e.n]) |w| try out.print(" {x:0>8}", .{w});
            try out.writeAll("\n");
        }
    }
    if (parsed.args.profile) |n| try printPlayProfile(gpa, out, &prof, &img, img_path, parsed.args.debug, n);
    if (parsed.args.dump) |path| if (res.trap != null) try writeFile(gpa, path, dump.written());
    if (res.trap) |t| if (t.err == error.Breakpoint) {
        if (t.packet) |i| try out.print("stopped in packet {d}", .{i}) else try out.print("stopped in init pass {d}", .{t.pass});
        try out.print(" before q {d} (0x{x})\n", .{ t.pc, t.pc });
        return;
    };
    if (res.trap) |t| {
        if (t.packet) |i| try out.print("trap: {s} in packet {d}", .{ @errorName(t.err), i }) else try out.print("trap: {s} in init pass {d}", .{ @errorName(t.err), t.pass });
        try out.print(" at q {d} (0x{x}):", .{ t.pc, t.pc });
        for (t.syllables[0..t.n_syllables]) |w| try out.print(" {x:0>8}", .{w});
        var buf: [256]u8 = undefined;
        const n = disasm.disasmBundle(&buf, t.syllables[0..t.n_syllables]) catch 0;
        try out.print("\n  {s}\n", .{if (n == 0) "?" else buf[0..n]});
        return error.Trapped;
    }
}

/// The profile report, with labels and lines from the sidecar when one can be read.  An
/// explicit --debug that cannot be read is an error; the default path is optional.
fn printPlayProfile(gpa: std.mem.Allocator, out: *Writer, prof: *const play_mod.Profile, img: *const image_mod.Image, img_path: []const u8, debug: ?[]const u8, n: usize) !void {
    const stem = if (std.mem.endsWith(u8, img_path, ".x17i")) img_path[0 .. img_path.len - 5] else img_path;
    const default_path = try std.fmt.allocPrint(gpa, "{s}.json", .{stem});
    defer gpa.free(default_path);
    const bytes: ?[]u8 = readFileMax(gpa, debug orelse default_path, 256 * 1024 * 1024) catch |e| blk: {
        if (debug != null) {
            try out.print("play: cannot read sidecar '{s}': {s}\n", .{ debug.?, @errorName(e) });
            return e;
        }
        break :blk null;
    };
    defer if (bytes) |b| gpa.free(b);
    var parsed: ?std.json.Parsed(std.json.Value) = null;
    defer if (parsed) |*p| p.deinit();
    if (bytes) |b| parsed = std.json.parseFromSlice(std.json.Value, gpa, b, .{}) catch |e| {
        try out.print("play: sidecar '{s}' is not JSON: {s}\n", .{ debug orelse default_path, @errorName(e) });
        return e;
    };
    try play_mod.writeProfile(gpa, out, prof, img, if (parsed) |p| p.value else null, n);
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

const ImageBytes = union(enum) { ok: []u8, err: asm_mod.FileError };

/// A source's image bytes (caller frees), or the error that stops it assembling.
fn imageBytes(gpa: std.mem.Allocator, src: []const u8, path: []const u8) !ImageBytes {
    var prog = switch (try asm_mod.assembleProgram(gpa, src, .{ .path = path, .loader = .{ .load = loadInclude } })) {
        .ok => |p| p,
        .err => |fe| return .{ .err = fe },
    };
    defer prog.deinit();
    return .{ .ok = try image_mod.write(gpa, &prog) };
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

    // Compare whole images, so passes and host data are checked as well as code.
    const before = switch (try imageBytes(gpa, src, path)) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            try out.print("fmt: '{s}' does not assemble; left as it is\n", .{path});
            return error.AssemblyFailed;
        },
    };
    defer gpa.free(before);
    const after = switch (try imageBytes(gpa, formatted, path)) {
        .ok => |w| w,
        .err => |fe| {
            try renderFileError(fe, path, out);
            try out.print("fmt: bug: the formatted '{s}' no longer assembles; left as it is\n", .{path});
            return error.FormatChangedBytes;
        },
    };
    defer gpa.free(after);
    if (!std.mem.eql(u8, before, after)) {
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
    \\    --steps <usize>      stop after N bundles and print the state there (word = pc)
    \\    --profile <usize>    print the N bundles run most often, with their words and text
    \\<str>                    source file path (.sm)
    \\
);

fn cmdRun(gpa: std.mem.Allocator, args: []const []const u8, out: *Writer) !void {
    var iter: SliceIter = .{};
    var parsed = try parseVerb(&run_params, gpa, args, "run", out, &iter);
    defer parsed.deinit();

    if (parsed.args.help != 0) {
        try out.writeAll("usage: x17 run [--lane N] [--v K] [--steps N] [--profile N] <file.sm>\n");
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

    var sites: std.ArrayList(asm_mod.Site) = .empty;
    defer sites.deinit(gpa);
    const result = try asm_mod.assembleFileWith(gpa, src, .{
        .path = path,
        .loader = .{ .load = loadInclude },
        .sites = if (parsed.args.profile != null) &sites else null,
    });
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
    const profile: ?[]u64 = if (parsed.args.profile != null) try gpa.alloc(u64, words.len) else null;
    defer if (profile) |p| gpa.free(p);
    if (profile) |p| @memset(p, 0);
    state.profile = profile;

    // Codec programs loop: allow a long run (a runaway still stops at the cap).  --steps stops
    // early on purpose, to compare against a probe at that point on silicon.
    const steps = parsed.args.steps;
    const bundles = emu.runFrom(gpa, &state, words, 0, steps orelse 1 << 24) catch |e| switch (e) {
        error.RunawayProgram => if (steps) |n| blk: {
            try out.print("stopped after {d} bundles; next bundle at word {d}\n", .{ n, state.pc });
            break :blk n;
        } else return e,
        else => return e,
    };

    try out.print("ran {d} bundles on lane {d} (V[{d}] init = 0x{x})\n\n", .{
        bundles, lane, lane, v_init,
    });

    if (profile) |p| try printProfile(gpa, out, words, sites.items, p, parsed.args.profile.?);

    try printBank(out, "s regs", "s", &state.s);
    try printBank(out, "e regs", "e", &state.e);
    try printBank(out, "R regs", "R", &state.r);
    try printBank(out, "A regs", "A", &state.a);
    try printMemory(gpa, out, "Y mem", "Y", &state.y_mem);
    try printMemory(gpa, out, "X mem", "X", &state.x_mem);
}

/// The `top` bundles run most often: count, share of the run, word index, disassembly. Nops
/// (`alu s0, s0, s0`) show as such, so unfilled delay slots stand out.
fn printProfile(gpa: std.mem.Allocator, out: *Writer, words: []const u32, sites: []const asm_mod.Site, profile: []const u64, n_top: usize) !void {
    var idx: std.ArrayList(usize) = .empty;
    defer idx.deinit(gpa);
    var total: u64 = 0;
    for (profile, 0..) |c, i| if (c != 0) {
        total += c;
        try idx.append(gpa, i);
    };
    const Ctx = struct {
        p: []const u64,
        fn more(ctx: @This(), a: usize, b: usize) bool {
            return ctx.p[a] > ctx.p[b] or (ctx.p[a] == ctx.p[b] and a < b);
        }
    };
    std.mem.sort(usize, idx.items, Ctx{ .p = profile }, Ctx.more);
    try out.print("profile: {d} bundles\n", .{total});
    for (idx.items[0..@min(n_top, idx.items.len)]) |i| {
        const len = emu.bundleLen(words[i]) orelse 1;
        var buf: [256]u8 = undefined;
        const n = disasm.disasmBundle(&buf, words[i..@min(words.len, i + len)]) catch 0;
        const pct = 100.0 * @as(f64, @floatFromInt(profile[i])) / @as(f64, @floatFromInt(@max(total, 1)));
        try out.print("  {d:>9} {d:>5.1}%  @{d:<5} {s}\n", .{ profile[i], pct, i, if (n == 0) "?" else buf[0..n] });
        if (siteAt(sites, i)) |st| {
            try out.print("{s:>26}{s}:{d}", .{ "", std.fs.path.basename(st.file.get()), st.line });
            if (st.call_line != 0) try out.print(" (from {s}:{d})", .{ std.fs.path.basename(st.call_file.get()), st.call_line });
            try out.print("  {s}\n", .{st.text.get()});
        }
    }
    try out.writeAll("\n");
}

fn siteAt(sites: []const asm_mod.Site, pc: usize) ?*const asm_mod.Site {
    var lo: usize = 0;
    var hi: usize = sites.len;
    while (lo < hi) {
        const mid = (lo + hi) / 2;
        if (sites[mid].pc < pc) lo = mid + 1 else hi = mid;
    }
    return if (lo < sites.len and sites[lo].pc == pc) &sites[lo] else null;
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
