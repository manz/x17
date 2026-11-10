const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const clap_dep = b.dependency("clap", .{
        .target = target,
        .optimize = optimize,
    });
    const clap_mod = clap_dep.module("clap");

    // isa, asm, emu and host are public modules: a dependent package runs the assembler and
    // the emulator from its own tests (b.dependency("x17", ...).module("asm")).
    const isa_mod = b.addModule("isa", .{
        .root_source_file = b.path("src/isa.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .strip = b.option(bool, "strip", "Strip debug info from the x17 binary"),
    });
    exe_mod.addImport("isa", isa_mod);

    const asm_mod = b.addModule("asm", .{
        .root_source_file = b.path("src/asm.zig"),
        .target = target,
        .optimize = optimize,
    });
    asm_mod.addImport("isa", isa_mod);
    exe_mod.addImport("asm", asm_mod);

    // Zig 0.17's self-hosted x86_64 backend miscompiles parts of the emulator (Linux debug
    // segfaults through a bad State pointer), so the exe and the test binaries that run it
    // build with LLVM.  The rest keep the default backend, far quicker to compile.
    // -Dllvm=true puts every artifact on LLVM.
    const all_llvm = b.option(bool, "llvm", "Build every artifact with LLVM") orelse false;
    const use_llvm: ?bool = if (all_llvm) true else null;
    const needs_llvm: ?bool = true;
    const exe = b.addExecutable(.{
        .name = "x17",
        .root_module = exe_mod,
        .use_llvm = needs_llvm,
    });
    b.installArtifact(exe);

    const run = b.addRunArtifact(exe);
    run.step.dependOn(b.getInstallStep());
    b.step("run", "Run x17").dependOn(&run.step);

    const emu_mod = b.addModule("emu", .{
        .root_source_file = b.path("src/emu.zig"),
        .target = target,
        .optimize = optimize,
    });
    emu_mod.addImport("isa", isa_mod);
    emu_mod.addImport("asm", asm_mod);

    const image_mod = b.addModule("image", .{
        .root_source_file = b.path("src/image.zig"),
        .target = target,
        .optimize = optimize,
    });
    image_mod.addImport("asm", asm_mod);

    const host_mod = b.addModule("host", .{
        .root_source_file = b.path("src/host.zig"),
        .target = target,
        .optimize = optimize,
    });
    host_mod.addImport("emu", emu_mod);
    host_mod.addImport("asm", asm_mod);

    exe_mod.addImport("clap", clap_mod);
    exe_mod.addImport("image", image_mod);
    exe_mod.addImport("emu", emu_mod);
    exe_mod.addImport("host", host_mod);

    // Zig runs only the tests of a binary's root module.  Each public module is a root of its
    // own; the CLI root (main.zig) gathers dispatch, disasm, doc, fmt and the test-only suites.
    const test_step = b.step("test", "Run unit tests");
    for ([_]*std.Build.Module{ isa_mod, asm_mod, image_mod }) |m|
        test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = m, .use_llvm = use_llvm })).step);
    for ([_]*std.Build.Module{ emu_mod, host_mod }) |m|
        test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = m, .use_llvm = needs_llvm })).step);

    const cli_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    for ([_][]const u8{ "clap", "isa", "asm", "emu", "image", "host" }, [_]*std.Build.Module{ clap_mod, isa_mod, asm_mod, emu_mod, image_mod, host_mod }) |name, m|
        cli_mod.addImport(name, m);
    for ([_][]const u8{ "docs/sm1/handbook.md", "README.md", "ARCHITECTURE.md", "ASSEMBLY.md", "PROBES.md", "docs/sm1/reference.md" }) |doc|
        cli_mod.addAnonymousImport(std.fs.path.basename(doc), .{ .root_source_file = b.path(doc) });
    // -Dtest-filter=<name>: run only the CLI-root tests whose name contains it (mutation checks).
    const test_filter = b.option([]const u8, "test-filter", "run only CLI-root tests matching this");
    const cli_filters: []const []const u8 = if (test_filter) |f| b.dupeStrings(&.{f}) else &.{};
    const cli_t = b.addTest(.{ .root_module = cli_mod, .filters = cli_filters, .use_llvm = needs_llvm });
    const cli_run = b.addRunArtifact(cli_t);
    // X17_COMPAT_PROG names an external program the compat test reads; rerun rather than cache.
    if (b.graph.environ_map.get("X17_COMPAT_PROG") != null or b.graph.environ_map.get("X17_COMPAT_IMAGE") != null)
        cli_run.has_side_effects = true;
    test_step.dependOn(&cli_run.step);
    b.step("test-lib", "Only the CLI-root tests (pair with -Dtest-filter)").dependOn(&cli_run.step);
}
