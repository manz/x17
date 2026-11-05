//! Spike: load the sm1 ISA table from data/isa.toml at comptime.
//!
//! At this point we only parse enough to prove the pipeline: the `[layout]`
//! section's `row_syllables`, `main_row`, and the `length` lookup table that
//! maps a head syllable's low5 to its instruction length in syllables.
//! Everything else in isa.toml stays unparsed; later passes will add opcode
//! forms (x17 bundles, head + S1 + S2 class fields) and the AGU / ALU
//! sub-classes.
//!
//! Comptime parse is the architectural bet of the x17 dev rig: if the
//! encoding table lives in a comptime-typed Zig struct then both the
//! assembler and the (future) Zig emulator share one source of truth, and a
//! stale copy on disk can never desync them at runtime.

const std = @import("std");

/// Low5 (bits [4:0] of a syllable's fmt field) -> instruction length in
/// syllables.  From isa.toml's `length = [[0x01, 7], ...]` table.
pub const LengthEntry = struct {
    low5: u8,
    syllables: u8,
};

pub const Layout = struct {
    row_syllables: u8,
    main_row: u16,
    length_table: []const LengthEntry,
};

/// Raw text of the ISA table, embedded at build time.
pub const isa_toml = @embedFile("isa.toml");

/// Comptime-parsed layout.  First real win of the Zig path: if isa.toml is
/// malformed the compiler fails the build, not some codec's first run.
pub const layout: Layout = parseLayout(isa_toml);

fn parseLayout(src: []const u8) Layout {
    @setEvalBranchQuota(100_000);

    // Walk the file line by line, picking only the three keys we care about
    // inside the [layout] section.  This is deliberately dumb: no general
    // TOML parser yet, just the minimum to prove comptime consumption works.
    var in_layout = false;
    var row_syllables: u8 = 0;
    var main_row: u16 = 0;
    comptime var length_buf: [32]LengthEntry = undefined;
    comptime var length_len: usize = 0;

    var it = std.mem.splitScalar(u8, src, '\n');
    while (it.next()) |raw| {
        const line = stripComment(raw);
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len == 0) continue;
        if (trimmed[0] == '[') {
            in_layout = std.mem.eql(u8, trimmed, "[layout]");
            continue;
        }
        if (!in_layout) continue;

        if (keyFor(trimmed, "row_syllables")) |v| {
            row_syllables = parseU8(v);
        } else if (keyFor(trimmed, "main_row")) |v| {
            main_row = parseU16(v);
        } else if (keyFor(trimmed, "length")) |v| {
            length_len = parseLengthArray(v, &length_buf);
        }
    }

    const slice: [length_len]LengthEntry = length_buf[0..length_len].*;
    return .{
        .row_syllables = row_syllables,
        .main_row = main_row,
        .length_table = &slice,
    };
}

fn stripComment(s: []const u8) []const u8 {
    if (std.mem.indexOfScalar(u8, s, '#')) |i| return s[0..i];
    return s;
}

fn keyFor(line: []const u8, key: []const u8) ?[]const u8 {
    if (!std.mem.startsWith(u8, line, key)) return null;
    const rest = line[key.len..];
    const eq = std.mem.indexOfScalar(u8, rest, '=') orelse return null;
    return std.mem.trim(u8, rest[eq + 1 ..], " \t\r");
}

fn parseU8(s: []const u8) u8 {
    const trimmed = std.mem.trim(u8, s, " \t\r");
    if (std.mem.startsWith(u8, trimmed, "0x")) {
        return std.fmt.parseInt(u8, trimmed[2..], 16) catch 0;
    }
    return std.fmt.parseInt(u8, trimmed, 10) catch 0;
}

fn parseU16(s: []const u8) u16 {
    const trimmed = std.mem.trim(u8, s, " \t\r");
    if (std.mem.startsWith(u8, trimmed, "0x")) {
        return std.fmt.parseInt(u16, trimmed[2..], 16) catch 0;
    }
    return std.fmt.parseInt(u16, trimmed, 10) catch 0;
}

/// Parse `[[0x01, 7], [0x04, 6], ...]` into LengthEntry pairs.  Dumb state
/// machine, intentionally permissive: whitespace OK, trailing comments
/// already stripped.
fn parseLengthArray(src: []const u8, out: []LengthEntry) usize {
    var count: usize = 0;
    var i: usize = 0;
    while (i < src.len) {
        while (i < src.len and src[i] != '[') : (i += 1) {}
        if (i >= src.len) break;
        // Skip the outer '[' once.
        if (count == 0 and i + 1 < src.len and src[i + 1] == '[') {
            i += 1;
            continue;
        }
        i += 1; // past inner '['
        const comma_rel = std.mem.indexOfScalar(u8, src[i..], ',') orelse break;
        const close_rel = std.mem.indexOfScalar(u8, src[i..], ']') orelse break;
        const low5 = parseU8(std.mem.trim(u8, src[i .. i + comma_rel], " \t"));
        const syllables = parseU8(std.mem.trim(u8, src[i + comma_rel + 1 .. i + close_rel], " \t"));
        out[count] = .{ .low5 = low5, .syllables = syllables };
        count += 1;
        i += close_rel + 1;
        if (count >= out.len) break;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

test "layout parses row_syllables and main_row" {
    try testing.expectEqual(@as(u8, 4), layout.row_syllables);
    try testing.expectEqual(@as(u16, 0x40), layout.main_row);
}

test "length table carries the six known head widths" {
    try testing.expectEqual(@as(usize, 6), layout.length_table.len);
    // From isa.toml: [[0x01, 7], [0x04, 6], [0x0D, 1], [0x15, 2], [0x17, 3], [0x19, 4]]
    const expect: [6]LengthEntry = .{
        .{ .low5 = 0x01, .syllables = 7 },
        .{ .low5 = 0x04, .syllables = 6 },
        .{ .low5 = 0x0D, .syllables = 1 },
        .{ .low5 = 0x15, .syllables = 2 },
        .{ .low5 = 0x17, .syllables = 3 },
        .{ .low5 = 0x19, .syllables = 4 },
    };
    for (expect, 0..) |e, i| {
        try testing.expectEqual(e.low5, layout.length_table[i].low5);
        try testing.expectEqual(e.syllables, layout.length_table[i].syllables);
    }
}

test "x17 is a 3-syllable bundle" {
    for (layout.length_table) |e| {
        if (e.low5 == 0x17) {
            try testing.expectEqual(@as(u8, 3), e.syllables);
            return;
        }
    }
    try testing.expect(false); // x17 must be present
}
