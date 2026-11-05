const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const isa_mod = b.createModule(.{
        .root_source_file = b.path("src/isa.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe_mod.addImport("isa", isa_mod);

    const exe = b.addExecutable(.{
        .name = "x17",
        .root_module = exe_mod,
    });
    b.installArtifact(exe);

    const run = b.addRunArtifact(exe);
    run.step.dependOn(b.getInstallStep());
    b.step("run", "Run x17").dependOn(&run.step);

    const test_step = b.step("test", "Run unit tests");
    const isa_t = b.addTest(.{ .root_module = isa_mod });
    test_step.dependOn(&b.addRunArtifact(isa_t).step);
}
