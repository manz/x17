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
    });
    exe_mod.addImport("isa", isa_mod);

    const asm_mod = b.addModule("asm", .{
        .root_source_file = b.path("src/asm.zig"),
        .target = target,
        .optimize = optimize,
    });
    asm_mod.addImport("isa", isa_mod);
    exe_mod.addImport("asm", asm_mod);

    const exe = b.addExecutable(.{
        .name = "x17",
        .root_module = exe_mod,
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

    const dispatch_mod = b.createModule(.{
        .root_source_file = b.path("src/dispatch.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    dispatch_mod.addImport("clap", clap_mod);
    dispatch_mod.addImport("isa", isa_mod);
    dispatch_mod.addImport("asm", asm_mod);
    dispatch_mod.addImport("emu", emu_mod);
    exe_mod.addImport("dispatch", dispatch_mod);
    exe_mod.addImport("emu", emu_mod);

    const disasm_mod = b.createModule(.{
        .root_source_file = b.path("src/disasm.zig"),
        .target = target,
        .optimize = optimize,
    });
    disasm_mod.addImport("isa", isa_mod);
    disasm_mod.addImport("asm", asm_mod);
    dispatch_mod.addImport("disasm", disasm_mod);

    const fmt_mod = b.createModule(.{
        .root_source_file = b.path("src/fmt.zig"),
        .target = target,
        .optimize = optimize,
    });
    dispatch_mod.addImport("fmt", fmt_mod);

    const doc_mod = b.createModule(.{
        .root_source_file = b.path("src/doc.zig"),
        .target = target,
        .optimize = optimize,
    });
    doc_mod.addImport("isa", isa_mod);
    dispatch_mod.addImport("doc", doc_mod);

    const gates_mod = b.createModule(.{
        .root_source_file = b.path("src/silicon_gates.zig"),
        .target = target,
        .optimize = optimize,
    });
    gates_mod.addImport("isa", isa_mod);
    gates_mod.addImport("asm", asm_mod);
    gates_mod.addImport("emu", emu_mod);

    const host_mod = b.addModule("host", .{
        .root_source_file = b.path("src/host.zig"),
        .target = target,
        .optimize = optimize,
    });
    host_mod.addImport("emu", emu_mod);
    host_mod.addImport("asm", asm_mod);

    const test_step = b.step("test", "Run unit tests");
    const fmt_t = b.addTest(.{ .root_module = fmt_mod });
    test_step.dependOn(&b.addRunArtifact(fmt_t).step);
    const doc_t = b.addTest(.{ .root_module = doc_mod });
    test_step.dependOn(&b.addRunArtifact(doc_t).step);
    const host_t = b.addTest(.{ .root_module = host_mod });
    test_step.dependOn(&b.addRunArtifact(host_t).step);
    const isa_t = b.addTest(.{ .root_module = isa_mod });
    test_step.dependOn(&b.addRunArtifact(isa_t).step);
    const asm_t = b.addTest(.{ .root_module = asm_mod });
    test_step.dependOn(&b.addRunArtifact(asm_t).step);
    const emu_t = b.addTest(.{ .root_module = emu_mod });
    test_step.dependOn(&b.addRunArtifact(emu_t).step);
    const gates_t = b.addTest(.{ .root_module = gates_mod });
    test_step.dependOn(&b.addRunArtifact(gates_t).step);
    const lib_mod = b.createModule(.{
        .root_source_file = b.path("src/lib_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    lib_mod.addImport("asm", asm_mod);
    lib_mod.addImport("emu", emu_mod);
    lib_mod.addAnonymousImport("handbook.md", .{ .root_source_file = b.path("docs/sm1/handbook.md") });
    lib_mod.addAnonymousImport("README.md", .{ .root_source_file = b.path("README.md") });
    lib_mod.addAnonymousImport("ARCHITECTURE.md", .{ .root_source_file = b.path("ARCHITECTURE.md") });
    lib_mod.addAnonymousImport("ASSEMBLY.md", .{ .root_source_file = b.path("ASSEMBLY.md") });
    lib_mod.addAnonymousImport("PROBES.md", .{ .root_source_file = b.path("PROBES.md") });
    lib_mod.addAnonymousImport("reference.md", .{ .root_source_file = b.path("docs/sm1/reference.md") });
    lib_mod.addImport("host", host_mod);
    // -Dtest-filter=<name>: run only the ISA tests whose name contains it (mutation checks).
    const test_filter = b.option([]const u8, "test-filter", "run only lib_test tests matching this");
    const lib_filters: []const []const u8 = if (test_filter) |f| b.dupeStrings(&.{f}) else &.{};
    const lib_t = b.addTest(.{ .root_module = lib_mod, .filters = lib_filters });
    const lib_run = b.addRunArtifact(lib_t);
    // X17_COMPAT_PROG names an external program the compat test reads; rerun rather than cache.
    if (b.graph.environ_map.get("X17_COMPAT_PROG") != null or b.graph.environ_map.get("X17_COMPAT_IMAGE") != null)
        lib_run.has_side_effects = true;
    test_step.dependOn(&lib_run.step);
    b.step("test-lib", "Only lib_test.zig (pair with -Dtest-filter)").dependOn(&lib_run.step);
    const disasm_t = b.addTest(.{ .root_module = disasm_mod });
    test_step.dependOn(&b.addRunArtifact(disasm_t).step);
}
