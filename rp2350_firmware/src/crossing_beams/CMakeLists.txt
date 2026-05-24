cmake_minimum_required(VERSION 3.12)

# ============================================================================
# crossing_beams — LH2 3D solver (standalone, self-contained)
#
# All sources (including the lh2 hardware layer) live inside this directory.
# No dependency on the parent src/lh2 folder or any other firmware target.
# ============================================================================

add_executable(crossing_beams
    main.c
    lh2/lh2.c
    lh2/lh2_decoder.c
    angle_decoder/angle_decoder.c
    cv/cv.c
    solve3d/solve3d.c
)

# Generate PIO header from the local copy of ts4231_capture.pio
pico_generate_pio_header(crossing_beams
    ${CMAKE_CURRENT_LIST_DIR}/lh2/ts4231_capture.pio
)

# Include paths:
#   "lh2/lh2.h"              → rp2350_firmware/src/crossing_beams/lh2/lh2.h
#   "angle_decoder/..."      → rp2350_firmware/src/crossing_beams/angle_decoder/...
#   "cv/cv.h"                → rp2350_firmware/src/crossing_beams/cv/cv.h
#   "solve3d/solve3d.h"      → rp2350_firmware/src/crossing_beams/solve3d/solve3d.h
target_include_directories(crossing_beams PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(crossing_beams
    pico_stdlib
    hardware_pio
    hardware_dma
    pico_multicore
)

# Build UF2, BIN, HEX, MAP
pico_add_extra_outputs(crossing_beams)

# USB serial; no UART
pico_enable_stdio_usb(crossing_beams  1)
pico_enable_stdio_uart(crossing_beams 0)

target_compile_options(crossing_beams PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
    -Wno-maybe-uninitialized
    -O2
)
