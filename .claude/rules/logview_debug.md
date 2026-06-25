# logview.html — message log debug notes

## What is known to work
- DataFlash .bin parser: correctly finds 194,439 records from a 3.4-min flight
- DF_SHOW whitelist filters to ~14,389 messages (7% of total)
- 3D trail is populated from XKF1 PN/PE/PD fields (2,051 position samples)
- Jump-to-start (⬆) and jump-to-end (⬇) buttons work
- Format auto-detection (DataFlash vs MAVLink) works

## The unsolved problem
The user says "i can only see a few messages" regardless of scroll position.
After jump-to-end they see the last few; at start they see the first few.
The message log never shows more than a handful of rows at once.

## Root cause hypothesis
The virtual scroll (`#mavlog`, `overflow-y:auto`, `position:relative`) renders
only the visible window (~ROW_H=34px rows). The `#mavlog` div gets almost no
height because the panel content above it (logo SVG + drop-zone + stats +
position readout) fills the 100vh flex column with `flex-shrink:0` items,
leaving `#mavlog-wrap { flex:1 }` almost nothing.

Attempts to fix:
1. `min-height: 200px` on `#mavlog-wrap` → not enough
2. `min-height: 300px` on `#mavlog-wrap` → still not enough on small screens
3. Capped SVG logo at `max-height:55px` → minor gain
4. Collapsed stats to 2 rows, compact drop-zone after load → minor gain
5. `overflow-y:auto` on `#panel` → unknown if it helped

## What to try next
**Option A — Ditch virtual scroll entirely.**
With 14,389 rows × 34px = 489 kpx of DOM, directly rendering all rows
will be heavy but may be the simplest correct solution.
Alternatively render all rows but use CSS `content-visibility:auto` for
paint-skipping without a manual virtual scroll.

**Option B — Separate the log into its own panel / full-width section.**
Move `#mavlog-wrap` OUT of `#panel` and give it its own full-width section
below the 3D view, or give it a fixed pixel height like `height:40vh`.

**Option C — Give `#mavlog` an explicit `height` instead of relying on flex.**
`#mavlog { height: calc(100vh - 420px); min-height: 250px; }` where 420px
covers the known fixed-height panel content above it.

**Option D — Make the whole page a two-column layout where the right column
is the 3D view and the left column is a full-height flex column that
allocates AT LEAST 50% to the log.**

## Files involved
- `docs/logview.html` — single-file, all CSS + JS inline
- No build step; edit directly

## Key constants
- `ROW_H = 34` px per message row (2 lines × 17px)
- `DF_SHOW` whitelist: MSG EV ERR MODE ARM CMD ORGN VER XKF1 NKF1 POS GPS GPA BARO ATT BAT POWR RCIN RCOU
- DataFlash magic: 0xA3 0x95; FMT type byte: 0x80; FMT record size: 89 bytes
