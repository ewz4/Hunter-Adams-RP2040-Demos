add_executable(boids)

add_compile_options(-Ofast)

# must match with pio filename and executable name from above
pico_generate_pio_header(boids ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(boids ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(boids ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
# REPLACE C FILE HERE
target_sources(boids PRIVATE boids5.c vga_graphics.c)

# must match with executable name
target_link_libraries(boids PRIVATE pico_stdlib pico_divider pico_multicore pico_bootsel_via_double_reset hardware_pio hardware_dma hardware_adc hardware_irq hardware_clocks hardware_pll)

# must match with executable name
pico_add_extra_outputs(boids)
