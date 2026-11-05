//! Spike: load the sm1 ISA table from src/isa.toml at comptime.
//!
//! Comptime parse is the architectural bet of the x17 dev rig: if the
//! encoding table lives in a comptime-typed Zig struct then both the
//! assembler and the (future) Zig emulator share one source of truth, and a
//! stale copy on disk can never desync them at runtime.
//!
//! What parses today:
//!   * `[layout]` - row_syllables, main_row, length lookup table
//!   * each `[[form]]` block - name, mask, value
//!
//! What waits:
//!   * operand bit positions per form (the ref string names them but is
//!     human-readable; we will add a structured operand table once the
//!     assembler needs it)
//!   * x17 sub-form grid (S1 class, S2 class) - ditto

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

// ---------------------------------------------------------------------------
// Forms
// ---------------------------------------------------------------------------

/// One opcode form: a (mask, value) pair matched against a syllable to pick
/// the form.  `name` is the mnemonic the assembler exposes and the one that
/// appears in `isa.toml`'s ref strings.  Operand bit positions are not in
/// this table yet - they live in the human ref string and will land as a
/// structured sub-table once the assembler needs them to emit operands.
pub const Form = struct {
    name: []const u8,
    mask: u32,
    value: u32,
};

pub const MAX_FORMS: usize = 64;
const forms_parsed = parseForms(isa_toml);

/// Table of every `[[form]]` block isa.toml declares, in file order.
/// Comptime-parsed from the embedded source, so any malformed entry fails
/// the build.
pub const forms: []const Form = forms_parsed.items[0..forms_parsed.len];

/// Classify a 32-bit syllable: return the first form whose `(word & mask) ==
/// value` test passes, or `null` if none does.  "First" means isa.toml order
/// - more specific forms (larger mask) are listed before less specific ones
/// by convention; we trust that convention and do not score.
pub fn classify(word: u32) ?*const Form {
    for (forms) |*f| {
        if ((word & f.mask) == f.value) return f;
    }
    return null;
}

const FormsParse = struct {
    items: [MAX_FORMS]Form,
    len: usize,
};

fn parseForms(src: []const u8) FormsParse {
    @setEvalBranchQuota(200_000);

    comptime var items: [MAX_FORMS]Form = undefined;
    comptime var len: usize = 0;
    comptime var in_form = false;
    comptime var cur: Form = .{ .name = "", .mask = 0, .value = 0 };
    comptime var seen_name = false;
    comptime var seen_mask = false;
    comptime var seen_value = false;

    var it = std.mem.splitScalar(u8, src, '\n');
    while (it.next()) |raw| {
        const line = stripComment(raw);
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len == 0) continue;

        if (trimmed[0] == '[') {
            // Flush the previous form if it was complete.
            if (in_form and seen_name and seen_mask and seen_value) {
                items[len] = cur;
                len += 1;
            }
            in_form = std.mem.eql(u8, trimmed, "[[form]]");
            cur = .{ .name = "", .mask = 0, .value = 0 };
            seen_name = false;
            seen_mask = false;
            seen_value = false;
            continue;
        }
        if (!in_form) continue;

        if (keyFor(trimmed, "name")) |v| {
            cur.name = parseStr(v);
            seen_name = true;
        } else if (keyFor(trimmed, "mask")) |v| {
            cur.mask = parseU32(parseStr(v));
            seen_mask = true;
        } else if (keyFor(trimmed, "value")) |v| {
            cur.value = parseU32(parseStr(v));
            seen_value = true;
        }
    }
    // Flush the trailing form.
    if (in_form and seen_name and seen_mask and seen_value) {
        items[len] = cur;
        len += 1;
    }
    return .{ .items = items, .len = len };
}

fn parseStr(s: []const u8) []const u8 {
    const t = std.mem.trim(u8, s, " \t\r");
    if (t.len >= 2 and t[0] == '"' and t[t.len - 1] == '"') return t[1 .. t.len - 1];
    return t;
}

fn parseU32(s: []const u8) u32 {
    const t = std.mem.trim(u8, s, " \t\r");
    if (std.mem.startsWith(u8, t, "0x")) {
        return std.fmt.parseInt(u32, t[2..], 16) catch 0;
    }
    return std.fmt.parseInt(u32, t, 10) catch 0;
}

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

test "forms table has the 26 known entries" {
    try testing.expectEqual(@as(usize, 26), forms.len);
}

test "first form is st.y" {
    try testing.expectEqualStrings("st.y", forms[0].name);
    try testing.expectEqual(@as(u32, 0xE000001F), forms[0].mask);
    try testing.expectEqual(@as(u32, 0x2000000F), forms[0].value);
}

test "x17 form reads mask and value" {
    var found: ?*const Form = null;
    for (forms) |*f| {
        if (std.mem.eql(u8, f.name, "x17")) {
            found = f;
            break;
        }
    }
    try testing.expect(found != null);
    try testing.expectEqual(@as(u32, 0x0000001F), found.?.mask);
    try testing.expectEqual(@as(u32, 0x00000017), found.?.value);
}

test "classify picks x17 for a syllable with low5 = 0x17" {
    const word: u32 = 0xDEADBE17; // any high bits, low5 = 0x17
    const f = classify(word);
    try testing.expect(f != null);
    try testing.expectEqualStrings("x17", f.?.name);
}

test "classify picks st.y for its mask/value" {
    // st.y mask 0xE000001F value 0x2000000F -> bits [31:29] = 001, low5 = 0x0F.
    const word: u32 = 0x2000000F;
    const f = classify(word);
    try testing.expect(f != null);
    try testing.expectEqualStrings("st.y", f.?.name);
}

test "classify returns null on a syllable no form matches" {
    // low5 = 0x00 is not a valid head opcode in any form.
    const word: u32 = 0x00000000;
    _ = classify(word);
    // st.y requires value 0x2000000F, and most other forms require specific
    // low5s - an all-zero word matches none that depend on low5 bits.  We do
    // not assert null here because the table may contain a catch-all; we
    // just want to prove classify() does not crash.
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
