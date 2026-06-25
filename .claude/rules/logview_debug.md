# logview.html — message log debug notes

## What is known to work
- DataFlash .bin parser: correctly finds 194,439 records from a 3.4-min flight
- DF_SHOW whitelist filters to ~14,389 messages (7% of total)
- 3D trail is populated from XKF1 PN/PE/PD fields (2,051 position samples)
- Jump-to-start (⬆) and jump-to-end (⬇) buttons work
- Format auto-detection (DataFlash vs MAVLink) works

## Solved (2026-06-25)
Root cause: virtual scroll `renderLog()` read `#mavlog.clientHeight` ≈ 0
because the flex layout in the scrollable `#panel` doesn't correctly size
`flex:1` children when the fixed items above sum close to 100vh. With
`clientHeight=0`, only 4 rows were ever rendered regardless of scroll position.

Fix applied (Option A): ditched the virtual scroll entirely. `#mavlog` now
holds all rows as direct children rendered via `innerHTML`. CSS
`content-visibility:auto; contain-intrinsic-size:0 34px` on `.ml` lets the
browser skip painting off-screen rows without manual windowing. For 14k rows
this is fast enough (~30–50 ms to build the HTML string).

## Files involved
- `docs/logview.html` — single-file, all CSS + JS inline
- No build step; edit directly

## Key constants
- `ROW_H = 34` px per message row (2 lines × 17px)
- `DF_SHOW` whitelist: MSG EV ERR MODE ARM CMD ORGN VER XKF1 NKF1 POS GPS GPA BARO ATT BAT POWR RCIN RCOU
- DataFlash magic: 0xA3 0x95; FMT type byte: 0x80; FMT record size: 89 bytes
