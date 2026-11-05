//! x17: dev rig for Samsung sm1's DSP.  CLI stub; verbs land as the spike
//! grows.  Today it prints the ISA layout the comptime parse produced, so a
//! human can eyeball that `data/isa.toml` and the typed table agree.

const std = @import("std");
const isa = @import("isa");

pub fn main() !void {
    const p = std.debug.print;
    p("x17 dev rig - ISA layout (from src/isa.toml at comptime)\n", .{});
    p("  row_syllables: {d}\n", .{isa.layout.row_syllables});
    p("  main_row:      0x{x}\n", .{isa.layout.main_row});
    p("  length table ({d} entries):\n", .{isa.layout.length_table.len});
    for (isa.layout.length_table) |e| {
        p("    low5 0x{x:0>2} -> {d} syllables{s}\n", .{
            e.low5,
            e.syllables,
            if (e.low5 == 0x17) "  (x17: main compute bundle)" else "",
        });
    }
    p("  forms ({d}):\n", .{isa.forms.len});
    for (isa.forms) |f| {
        p("    {s: <7}  mask=0x{x:0>8}  value=0x{x:0>8}\n", .{ f.name, f.mask, f.value });
    }
    p("  operands ({d} rows):\n", .{isa.operands.len});
    for (isa.operands) |op| {
        p("    {s: <6} {s: <8} {s}  [{d}:{d}]  w={d}\n", .{
            op.form, op.name, @tagName(op.kind), op.hi, op.lo, op.width(),
        });
    }
}
