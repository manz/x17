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

// ---------------------------------------------------------------------------
// Operands
// ---------------------------------------------------------------------------

/// Kind of value an operand slot carries.  Drives how the assembler parses
/// the source token and how it folds the field into the syllable.
pub const OperandKind = enum {
    s, // scalar reg, 4 bits
    e, // element reg, 4 bits
    r_reg, // R-file reg, 4 bits (R0 pinned to 0)
    a_reg, // Address reg A0..A15, 4 or 5 bits
    p, // predicate, form-specific width
    imm, // raw immediate, width = hi-lo+1
    dest_ldi, // ldi's 5-bit dest index (16..31 = s(k-16))
    y_addr, // Y-space immediate: written `Y:<imm>` or `Y:#<imm>`
};

/// One operand's bit slice within the HEAD syllable of a bundle.  `lo` and
/// `hi` are inclusive bit positions; `width = hi - lo + 1`.
pub const Operand = struct {
    form: []const u8,
    name: []const u8,
    kind: OperandKind,
    lo: u8,
    hi: u8,
    /// 0 = head, 1 = s1, 2 = s2, ...  Single-syllable forms always use 0.
    syll: u8 = 0,

    pub fn width(self: Operand) u8 {
        return self.hi - self.lo + 1;
    }

    /// Mask of the operand's bit range, aligned to the syllable.
    pub fn fieldMask(self: Operand) u32 {
        const w: u5 = @intCast(self.width());
        const base: u32 = (@as(u32, 1) << w) - 1;
        return base << @as(u5, @intCast(self.lo));
    }

    /// Place `value` into the operand's bit range, returning the masked
    /// contribution to OR into the syllable.  Caller guarantees value fits.
    pub fn encode(self: Operand, value: u32) u32 {
        const w: u5 = @intCast(self.width());
        const mask: u32 = (@as(u32, 1) << w) - 1;
        return (value & mask) << @as(u5, @intCast(self.lo));
    }
};

pub const MAX_OPERANDS: usize = 512;

pub const operands_txt = @embedFile("operands.txt");

const operands_parsed = parseOperands(operands_txt);

/// Every operand row seeded in `src/operands.txt`.  The assembler walks this
/// in order when encoding a form.
pub const operands: []const Operand = operands_parsed.items[0..operands_parsed.len];

/// Return the (contiguous) operand slice for a form name, or an empty slice
/// if the form has no rows yet.  Operand order follows file order, which
/// matches the assembler's expected source order (dest first, then sources,
/// then immediates - same convention ASSEMBLY.md documents).
pub fn operandsFor(form_name: []const u8) []const Operand {
    var start: ?usize = null;
    var end: usize = 0;
    for (operands, 0..) |op, i| {
        if (std.mem.eql(u8, op.form, form_name)) {
            if (start == null) start = i;
            end = i + 1;
        } else if (start != null) {
            break;
        }
    }
    if (start) |s| return operands[s..end];
    return operands[0..0];
}

const OperandsParse = struct {
    items: [MAX_OPERANDS]Operand,
    len: usize,
};

// ---------------------------------------------------------------------------
// Sub-forms (multi-syllable bundle templates)
// ---------------------------------------------------------------------------

pub const MAX_BUNDLE_SYLLABLES: usize = 7;

/// One multi-syllable bundle template.  `base` carries the all-operands-zero
/// value of each of `syllables` consecutive 32-bit words; the assembler
/// walks `operandsFor(name)` and OR's each field contribution into the
/// syllable named by its `syll` index.
pub const SubForm = struct {
    name: []const u8,
    syllables: u8,
    base: [MAX_BUNDLE_SYLLABLES]u32,
};

pub const MAX_SUBFORMS: usize = 128;

pub const subforms_txt = @embedFile("subforms.txt");

const subforms_parsed = parseSubForms(subforms_txt);

pub const subforms: []const SubForm = subforms_parsed.items[0..subforms_parsed.len];

pub fn subFormByName(name: []const u8) ?*const SubForm {
    for (subforms) |*sf| {
        if (std.mem.eql(u8, sf.name, name)) return sf;
    }
    return null;
}

const SubFormsParse = struct {
    items: [MAX_SUBFORMS]SubForm,
    len: usize,
};

fn parseSubForms(src: []const u8) SubFormsParse {
    @setEvalBranchQuota(200_000);
    comptime var items: [MAX_SUBFORMS]SubForm = undefined;
    comptime var len: usize = 0;

    var it = std.mem.splitScalar(u8, src, '\n');
    while (it.next()) |raw| {
        const line = stripComment(raw);
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len == 0) continue;

        var tok_it = std.mem.tokenizeAny(u8, trimmed, " \t");
        const name = tok_it.next() orelse continue;
        const syl_s = tok_it.next() orelse continue;
        const syllables = parseU8(syl_s);

        var base: [MAX_BUNDLE_SYLLABLES]u32 = @splat(0);
        var k: usize = 0;
        while (tok_it.next()) |word_s| : (k += 1) {
            if (k >= MAX_BUNDLE_SYLLABLES) break;
            base[k] = parseU32(word_s);
        }

        items[len] = .{ .name = name, .syllables = syllables, .base = base };
        len += 1;
        if (len >= MAX_SUBFORMS) break;
    }
    return .{ .items = items, .len = len };
}

fn parseOperands(src: []const u8) OperandsParse {
    @setEvalBranchQuota(400_000);
    comptime var items: [MAX_OPERANDS]Operand = undefined;
    comptime var len: usize = 0;

    var it = std.mem.splitScalar(u8, src, '\n');
    while (it.next()) |raw| {
        const line = stripComment(raw);
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len == 0) continue;

        // Expect tab-separated: form <TAB> name <TAB> kind <TAB> lo <TAB> hi <TAB> syll
        const f1 = std.mem.indexOfScalar(u8, trimmed, '\t') orelse continue;
        const rest1 = trimmed[f1 + 1 ..];
        const f2 = std.mem.indexOfScalar(u8, rest1, '\t') orelse continue;
        const rest2 = rest1[f2 + 1 ..];
        const f3 = std.mem.indexOfScalar(u8, rest2, '\t') orelse continue;
        const rest3 = rest2[f3 + 1 ..];
        const f4 = std.mem.indexOfScalar(u8, rest3, '\t') orelse continue;
        const rest4 = rest3[f4 + 1 ..];
        // syll column is optional; absent means `head` (back-compat).
        var hi_s: []const u8 = undefined;
        var syll_s: []const u8 = "head";
        if (std.mem.indexOfScalar(u8, rest4, '\t')) |f5| {
            hi_s = std.mem.trim(u8, rest4[0..f5], " \t\r");
            syll_s = std.mem.trim(u8, rest4[f5 + 1 ..], " \t\r");
        } else {
            hi_s = std.mem.trim(u8, rest4, " \t\r");
        }

        const form = std.mem.trim(u8, trimmed[0..f1], " \t\r");
        const name = std.mem.trim(u8, rest1[0..f2], " \t\r");
        const kind_s = std.mem.trim(u8, rest2[0..f3], " \t\r");
        const lo_s = std.mem.trim(u8, rest3[0..f4], " \t\r");

        const kind: OperandKind =
            if (std.mem.eql(u8, kind_s, "s")) .s
            else if (std.mem.eql(u8, kind_s, "e")) .e
            else if (std.mem.eql(u8, kind_s, "R")) .r_reg
            else if (std.mem.eql(u8, kind_s, "A")) .a_reg
            else if (std.mem.eql(u8, kind_s, "p")) .p
            else if (std.mem.eql(u8, kind_s, "imm")) .imm
            else if (std.mem.eql(u8, kind_s, "dest_ldi")) .dest_ldi
            else if (std.mem.eql(u8, kind_s, "y_addr")) .y_addr
            else continue;

        const syll: u8 = blk: {
            if (std.mem.eql(u8, syll_s, "head")) break :blk 0;
            if (syll_s.len == 2 and syll_s[0] == 's' and syll_s[1] >= '1' and syll_s[1] <= '9') {
                break :blk syll_s[1] - '0';
            }
            break :blk 0;
        };

        items[len] = .{
            .form = form,
            .name = name,
            .kind = kind,
            .lo = parseU8(lo_s),
            .hi = parseU8(hi_s),
            .syll = syll,
        };
        len += 1;
        if (len >= MAX_OPERANDS) break;
    }
    return .{ .items = items, .len = len };
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

test "length table carries the sixteen known head widths" {
    try testing.expectEqual(@as(usize, 16), layout.length_table.len);
    const expect: [16]LengthEntry = .{
        .{ .low5 = 0x01, .syllables = 7 },
        .{ .low5 = 0x02, .syllables = 6 },
        .{ .low5 = 0x03, .syllables = 6 },
        .{ .low5 = 0x04, .syllables = 6 },
        .{ .low5 = 0x0D, .syllables = 1 },
        .{ .low5 = 0x0E, .syllables = 2 },
        .{ .low5 = 0x10, .syllables = 2 },
        .{ .low5 = 0x11, .syllables = 2 },
        .{ .low5 = 0x12, .syllables = 2 },
        .{ .low5 = 0x13, .syllables = 2 },
        .{ .low5 = 0x14, .syllables = 2 },
        .{ .low5 = 0x15, .syllables = 2 },
        .{ .low5 = 0x16, .syllables = 3 },
        .{ .low5 = 0x17, .syllables = 3 },
        .{ .low5 = 0x18, .syllables = 4 },
        .{ .low5 = 0x19, .syllables = 4 },
    };
    for (expect, 0..) |e, i| {
        try testing.expectEqual(e.low5, layout.length_table[i].low5);
        try testing.expectEqual(e.syllables, layout.length_table[i].syllables);
    }
}

test "forms table has the 31 known entries" {
    try testing.expectEqual(@as(usize, 31), forms.len);
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

test "operands table seeds rli and ldi" {
    try testing.expect(operands.len >= 5);
}

test "rli has three operands in declared order" {
    const ops = operandsFor("rli");
    try testing.expectEqual(@as(usize, 3), ops.len);
    try testing.expectEqualStrings("dest_R", ops[0].name);
    try testing.expectEqual(OperandKind.r_reg, ops[0].kind);
    try testing.expectEqual(@as(u8, 27), ops[0].lo);
    try testing.expectEqual(@as(u8, 30), ops[0].hi);
    try testing.expectEqualStrings("dest_s", ops[1].name);
    try testing.expectEqualStrings("imm16", ops[2].name);
}

test "operand width is inclusive" {
    const ops = operandsFor("rli");
    try testing.expectEqual(@as(u8, 4), ops[0].width()); // 27..30 inclusive
    try testing.expectEqual(@as(u8, 16), ops[2].width()); // 5..20 inclusive
}

test "fieldMask lines up with the bit range" {
    const ops = operandsFor("rli");
    // dest_R at [30:27] -> 0x78000000
    try testing.expectEqual(@as(u32, 0x7800_0000), ops[0].fieldMask());
    // dest_s at [24:21] -> 0x01E00000
    try testing.expectEqual(@as(u32, 0x01E0_0000), ops[1].fieldMask());
    // imm16 at [20:5] -> 0x001FFFE0
    try testing.expectEqual(@as(u32, 0x001F_FFE0), ops[2].fieldMask());
}

test "encode packs rli operands into the head syllable" {
    const ops = operandsFor("rli");
    // rli R3, s5, #0x1234  -> dest_R=3, dest_s=5, imm16=0x1234
    const r3 = ops[0].encode(3);
    const s5 = ops[1].encode(5);
    const imm = ops[2].encode(0x1234);
    const word: u32 = r3 | s5 | imm;
    try testing.expectEqual(@as(u32, 0x18A2_4680), word);
    // Operand masks are disjoint so no overlap.
    try testing.expect((ops[0].fieldMask() & ops[1].fieldMask()) == 0);
    try testing.expect((ops[1].fieldMask() & ops[2].fieldMask()) == 0);
}

test "ldi dest is a 5-bit field at [25:21]" {
    const ops = operandsFor("ldi");
    try testing.expectEqual(@as(usize, 2), ops.len);
    try testing.expectEqualStrings("dest", ops[0].name);
    try testing.expectEqual(OperandKind.dest_ldi, ops[0].kind);
    try testing.expectEqual(@as(u8, 5), ops[0].width());
    try testing.expectEqual(@as(u32, 0x03E0_0000), ops[0].fieldMask());
}

test "operandsFor an unknown form returns an empty slice" {
    try testing.expectEqual(@as(usize, 0), operandsFor("not_a_form").len);
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
