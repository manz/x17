const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const clap_dep = b.dependency("clap", .{
        .target = target,
        .optimize = optimize,
    });
    const clap_mod = clap_dep.module("clap");

    const isa_mod = b.createModule(.{
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

    const asm_mod = b.createModule(.{
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

    const emu_mod = b.createModule(.{
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

    const gates_mod = b.createModule(.{
        .root_source_file = b.path("src/silicon_gates.zig"),
        .target = target,
        .optimize = optimize,
    });
    gates_mod.addImport("isa", isa_mod);
    gates_mod.addImport("asm", asm_mod);
    gates_mod.addImport("emu", emu_mod);

    const host_mod = b.createModule(.{
        .root_source_file = b.path("src/host.zig"),
        .target = target,
        .optimize = optimize,
    });
    host_mod.addImport("emu", emu_mod);
    host_mod.addImport("asm", asm_mod);

    const test_step = b.step("test", "Run unit tests");
    const fmt_t = b.addTest(.{ .root_module = fmt_mod });
    test_step.dependOn(&b.addRunArtifact(fmt_t).step);
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
    lib_mod.addAnonymousImport("bits.sm", .{ .root_source_file = b.path("examples/lib/bits.sm") });
    lib_mod.addAnonymousImport("rice.sm", .{ .root_source_file = b.path("examples/lib/rice.sm") });
    lib_mod.addAnonymousImport("predictor.sm", .{ .root_source_file = b.path("examples/lib/predictor.sm") });
    lib_mod.addAnonymousImport("unmix.sm", .{ .root_source_file = b.path("examples/lib/unmix.sm") });
    lib_mod.addAnonymousImport("mem.sm", .{ .root_source_file = b.path("examples/lib/mem.sm") });
    lib_mod.addAnonymousImport("alac.sm", .{ .root_source_file = b.path("examples/lib/alac.sm") });
    lib_mod.addAnonymousImport("huffman.sm", .{ .root_source_file = b.path("examples/lib/huffman.sm") });
    lib_mod.addAnonymousImport("sections.sm", .{ .root_source_file = b.path("examples/lib/sections.sm") });
    lib_mod.addAnonymousImport("dequant.sm", .{ .root_source_file = b.path("examples/lib/dequant.sm") });
    for ([_][]const u8{ "mono.at", "tone.at", "white24.at", "clicks.c0.ff", "clicks.o30.ff", "mono.at.int", "tone.at.int" }) |f| {
        lib_mod.addAnonymousImport(b.fmt("{s}.pkts", .{f}), .{ .root_source_file = b.path(b.fmt("testdata/alac/{s}.pkts", .{f})) });
    }
    const lib_t = b.addTest(.{ .root_module = lib_mod });

    const bench_mod = b.createModule(.{
        .root_source_file = b.path("src/bench.zig"),
        .target = target,
        .optimize = .ReleaseFast,
    });
    bench_mod.addImport("asm", asm_mod);
    bench_mod.addImport("emu", emu_mod);
    for ([_][]const u8{ "bits", "rice", "predictor", "unmix", "mem", "alac" }) |l| {
        bench_mod.addAnonymousImport(b.fmt("{s}.sm", .{l}), .{ .root_source_file = b.path(b.fmt("examples/lib/{s}.sm", .{l})) });
    }
    for ([_][]const u8{ "mono.at", "tone.at", "white24.at", "clicks.c0.ff", "clicks.o30.ff", "mono.at.int", "tone.at.int" }) |f| {
        bench_mod.addAnonymousImport(b.fmt("{s}.pkts", .{f}), .{ .root_source_file = b.path(b.fmt("testdata/alac/{s}.pkts", .{f})) });
    }
    const bench = b.addExecutable(.{ .name = "bench", .root_module = bench_mod });
    b.step("bench", "Bundles per ALAC packet, by stage").dependOn(&b.addRunArtifact(bench).step);
    test_step.dependOn(&b.addRunArtifact(lib_t).step);
    const disasm_t = b.addTest(.{ .root_module = disasm_mod });
    test_step.dependOn(&b.addRunArtifact(disasm_t).step);
}
