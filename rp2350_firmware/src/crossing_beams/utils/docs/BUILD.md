# Building `crossing_beams`

A from-zero guide to compiling and flashing this firmware onto a Raspberry Pi
Pico 2 (RP2350). Covers the terminal, the VS Code Pico extension, flashing, and
how to edit `CMakeLists.txt` when you add files or targets.

---

## The mental model

Building firmware is **two separate steps** done by **two separate tools**:

```
   your .c files                    one .uf2 file
        │                                 ▲
        ▼                                 │
   ┌─────────┐   creates build      ┌──────────┐
   │  CMake  │ ───── recipe ──────► │  Ninja   │
   │(planner)│   (build.ninja)      │(builder) │
   └─────────┘                      └──────────┘
```

1. **CMake** — the *planner*. Reads `CMakeLists.txt`, locates the SDK, picks the
   board/compiler, and writes a recipe (`build.ninja`) into `build/`.
   Re-run it **only when you change `CMakeLists.txt`**.
2. **Ninja** — the *builder*. Reads the recipe and compiles `.c` → `.uf2`.
   Run it **every time you edit code**.

> The classic error `loading 'build.ninja': No such file or directory` just
> means you ran Ninja before CMake ever wrote the recipe. Configure once first.

---

## Method A — Terminal (3 commands)

```bash
cd ~/Repositories/lh2_on_rp2040/rp2350_firmware/src/crossing_beams
```

**Step 1 — Configure** (first time, or after editing `CMakeLists.txt`):
```bash
~/.pico-sdk/cmake/v3.31.5/bin/cmake -B build -G Ninja -DPICO_BOARD=pico2 .
```

**Step 2 — Build** (every code change):
```bash
# all targets
~/.pico-sdk/ninja/v1.12.1/ninja -C build

# or just one
~/.pico-sdk/ninja/v1.12.1/ninja -C build crossing_beams_mavlink_test
```

Output lands in `build/<target>.uf2`.

---

## Method B — VS Code Pico extension

1. Open the **`crossing_beams` folder** (the one with `CMakeLists.txt`) — not a
   parent folder.
2. Press **`Ctrl+Shift+B`**. The `tasks.json` here runs *CMake Configure* then
   *Compile Project* in sequence (via `dependsOn`), so it can't hit the
   "no build.ninja" error.
3. Watch the bottom terminal panel. Green = done.

Nuclear reset if the extension gets confused (stale cache / wrong compiler):
```bash
rm -rf ~/Repositories/lh2_on_rp2040/rp2350_firmware/src/crossing_beams/build
```
Then `Ctrl+Shift+B` again. `build/` is 100% regenerated output — safe to delete.

---

## Flashing onto the Pico 2

**Drag & drop:**
1. Hold **BOOTSEL**, plug in USB, release → a drive named `RPI-RP2` appears.
2. Drag the `.uf2` onto it. The Pico reboots into your code.

**picotool** (works while firmware is already running):
```bash
~/.pico-sdk/picotool/2.2.0-a4/picotool/picotool load build/crossing_beams.uf2 -fx
```
`-fx` reboots into the program after loading.

---

## Targets

| Target | Source | Purpose |
|---|---|---|
| `crossing_beams` | `main_real.c` | Real dual-core firmware: 4 sensors → solve3d → MAVLink |
| `crossing_beams_mavlink_test` | `main.c` | Synthetic square-path VPE test (no sensors) |
| `crossing_beams_test` | `main_test.c` | solve3d math self-test (no hardware) |

---

## Understanding `CMakeLists.txt`

The top block (lines marked *DO NOT EDIT*) is boilerplate the Pico VS Code
extension needs — it finds the SDK and toolchain. Leave it alone. The
interesting parts:

```cmake
include(pico_sdk_import.cmake)   # pull in the Pico SDK (must be before project())
project(crossing_beams C CXX ASM)
pico_sdk_init()                  # initialise SDK after project()
```

> Use `include(pico_sdk_import.cmake)`, **not**
> `include($ENV{PICO_SDK_PATH}/...)`. The local `pico_sdk_import.cmake` finds the
> SDK reliably even when `PICO_SDK_PATH` isn't exported in your shell — that was
> the cause of an earlier "PICO_SDK_PATH not set" failure.

### Anatomy of one target

```cmake
# 1. Declare the executable and list EVERY .c file it needs
add_executable(crossing_beams
    main_real.c
    lh2/lh2.c
    angle_decoder/angle_decoder.c
    cv/cv.c
    solve3d/solve3d.c
    mavlink/mavlink.c
)

# 2. Generate a C header from a PIO assembly file (LH2 capture state machine)
pico_generate_pio_header(crossing_beams
    ${CMAKE_CURRENT_LIST_DIR}/lh2/ts4231_capture.pio
)

# 3. Where to find #include headers (lets you write "lh2/lh2.h")
target_include_directories(crossing_beams PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# 4. SDK libraries this target uses — each unlocks a set of APIs
target_link_libraries(crossing_beams
    pico_stdlib        # core: GPIO, timers, printf
    hardware_pio       # PIO (needed for the .pio capture)
    hardware_dma       # DMA (PIO → RAM transfers)
    hardware_uart      # UART0 (MAVLink to the Pixhawk)
    pico_multicore     # multicore_launch_core1()
)

# 5. Produce .uf2/.hex/.bin/.map alongside the .elf
pico_add_extra_outputs(crossing_beams)

# 6. Route stdio: printf over USB, not the UART (UART is busy with MAVLink)
pico_enable_stdio_usb(crossing_beams  1)
pico_enable_stdio_uart(crossing_beams 0)

# 7. Compiler flags for this target only
target_compile_options(crossing_beams PRIVATE
    -Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized -O2
)
```

### Common edits

**Added a new `.c` file?** Add its path to the matching `add_executable(...)`
list, then **re-configure** (CMake must see it) and build.

**Need a hardware API the compiler can't find** (e.g. `hardware/i2c.h`)? Add the
matching library to `target_link_libraries` (here `hardware_i2c`), then
re-configure. Linker errors like *"undefined reference to `i2c_init`"* almost
always mean a missing library here.

**New `#include "foo/bar.h"` not found?** Make sure the directory is reachable
from a `target_include_directories` path. Everything under the project root is,
because of `${CMAKE_CURRENT_LIST_DIR}`.

**Want a whole new program?** Copy an entire target block (all 7 numbered parts),
rename `crossing_beams` to your new name everywhere in that block, and swap the
source files. Re-configure and it shows up as a new Ninja target.

### Useful CMake variables / flags

| Thing | Meaning |
|---|---|
| `-DPICO_BOARD=pico2` | Target board. `pico2` = RP2350. Use `pico` for RP2040. |
| `-DCMAKE_BUILD_TYPE=Debug` | Unoptimised + debug symbols (default here is Release/`-O2`). |
| `${CMAKE_CURRENT_LIST_DIR}` | Absolute path of the folder holding this `CMakeLists.txt`. |
| `PRIVATE` | These settings apply to this target only (not propagated). |
| `pico_enable_stdio_usb(t 1)` | `printf` goes out the USB CDC serial. |
| `pico_enable_stdio_uart(t 1)` | `printf` goes out UART0 — **off here** so MAVLink owns UART0. |

---

## The one rule that saves you

> **Changed `CMakeLists.txt`?** → run *both* steps (configure, then build).
> **Changed only `.c`/`.h`?** → run *only* Ninja (or `Ctrl+Shift+B`, always safe).
