# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(final-project)

# initialize the sdk
pico_sdk_init()

#######

# name anything you want
add_executable(music_animation)

# must match with pio filename and executable name from above
pico_generate_pio_header(music_animation ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(music_animation ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(music_animation ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
pico_generate_pio_header(music_animation ${CMAKE_CURRENT_LIST_DIR}/rgb2.pio)

# must match with executable name and source file names
target_sources(music_animation PRIVATE 
	vga256_graphics_image.c 
	vga256_graphics.c
	)

# must match with executable name
target_link_libraries(music_animation PRIVATE 
	pico_stdlib 
	pico_bootsel_via_double_reset 
	hardware_pio 
	hardware_dma 
	hardware_adc 
	hardware_sync
	hardware_irq
	pico_multicore)

# must match with executable name
pico_add_extra_outputs(music_animation)

add_compile_options(-Ofast)
