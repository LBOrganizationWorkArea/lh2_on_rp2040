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
include(pico_sdk_import.cmake)

project(crossing_beams C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ============================================================================
# crossing_beams — LH2 3D solver (full hardware, all 4 TS4231 sensors)
# ============================================================================

add_executable(crossing_beams
    main_real.c
    lh2/lh2.c
    lh2/lh2_decoder.c
    angle_decoder/angle_decoder.c
    cv/cv.c
    solve3d/solve3d.c
    mavlink/mavlink.c
)

pico_generate_pio_header(crossing_beams
    ${CMAKE_CURRENT_LIST_DIR}/lh2/ts4231_capture.pio
)

target_include_directories(crossing_beams PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(crossing_beams
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_uart
    pico_multicore
)

pico_add_extra_outputs(crossing_beams)
pico_enable_stdio_usb(crossing_beams  1)
pico_enable_stdio_uart(crossing_beams 0)

target_compile_options(crossing_beams PRIVATE
    -Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized -O2
)

# ============================================================================
# crossing_beams_synthetic — full dual-core pipeline, no sensors
#
# Builds main_real.c with -DSYNTHETIC_CAPTURE: core 1 fabricates the LFSR
# counts a real sensor would emit for a body tracing a 1 m × 1 m square, then
# core 0 runs the REAL angle_decoder → solve3d → MAVLink path unchanged.
# Exercises the cross-core architecture on-chip without any hardware.
# The lh2/ PIO+DMA layer is intentionally omitted (unused in synthetic mode).
# See SYNTHETIC_CAPTURE.md.
# ============================================================================

add_executable(crossing_beams_synthetic
    main_real.c
    angle_decoder/angle_decoder.c
    cv/cv.c
    solve3d/solve3d.c
    mavlink/mavlink.c
)

target_compile_definitions(crossing_beams_synthetic PRIVATE SYNTHETIC_CAPTURE)

target_include_directories(crossing_beams_synthetic PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(crossing_beams_synthetic
    pico_stdlib
    hardware_uart
    pico_multicore
)

pico_add_extra_outputs(crossing_beams_synthetic)
pico_enable_stdio_usb(crossing_beams_synthetic  1)
pico_enable_stdio_uart(crossing_beams_synthetic 0)

target_compile_options(crossing_beams_synthetic PRIVATE
    -Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized -O2
)
