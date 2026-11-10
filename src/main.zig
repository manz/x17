//! x17 dev rig entry point.  Thin: read argv, hand off to the dispatcher.

const std = @import("std");
const dispatch = @import("dispatch.zig");

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;

    var it = try init.minimal.args.iterateAllocator(gpa);
    defer it.deinit();

    var list: std.ArrayList([]const u8) = .empty;
    defer {
        for (list.items) |a| gpa.free(a);
        list.deinit(gpa);
    }
    while (it.next()) |arg| {
        try list.append(gpa, try gpa.dupe(u8, arg));
    }

    var stdout_buf: [4096]u8 = undefined;
    var fw: std.Io.File.Writer = .initStreaming(.stdout(), init.io, &stdout_buf);
    defer fw.interface.flush() catch {};
    const out: *std.Io.Writer = &fw.interface;

    const args: []const []const u8 = if (list.items.len == 0) &.{} else list.items[1..];
    dispatch.run(gpa, args, out) catch |e| switch (e) {
        error.UnknownVerb, error.AssemblyFailed, error.MissingFile, error.BadFormat, error.Unformatted, error.FormatChangedBytes => {
            out.flush() catch {};
            std.process.exit(2);
        },
        else => return e,
    };
}

test {
    // The CLI's files (dispatch, disasm, doc, fmt, play) and the test-only suites run in this binary
    // (see build.zig).
    _ = dispatch;
    _ = @import("disasm.zig");
    _ = @import("doc.zig");
    _ = @import("fmt.zig");
    _ = @import("play.zig");
    _ = @import("lib_test.zig");
    _ = @import("silicon_gates.zig");
}
