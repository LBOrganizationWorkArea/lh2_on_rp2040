# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(crossing_beams C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

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

# ============================================================================
# crossing_beams_test — solve3d math self-test (no hardware required)
#
# Feeds a synthetic virtual scene through angles_to_pixels + solve3d_run,
# prints 3D points as CSV over USB serial.
#
# Only cv.c and solve3d.c are compiled — no lh2, no angle_decoder, no PIO.
# Flash crossing_beams_test.uf2, open USB serial, capture to pico_out.csv,
# then compare with:  python validate_solve3d.py > python_out.csv
#                     python compare_results.py pico_out.csv python_out.csv
# ============================================================================

add_executable(crossing_beams_test
    main_test.c
    cv/cv.c
    solve3d/solve3d.c
)

target_include_directories(crossing_beams_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(crossing_beams_test
    pico_stdlib
)

pico_add_extra_outputs(crossing_beams_test)

pico_enable_stdio_usb(crossing_beams_test  1)
pico_enable_stdio_uart(crossing_beams_test 0)

target_compile_options(crossing_beams_test PRIVATE
    -Wall
    -Wno-format
    -Wno-unused-function
    -Wno-maybe-uninitialized
    -O2
)
