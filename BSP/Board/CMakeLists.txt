# BSP/Board — DJI Type-C LED / KEY / CS GPIO helpers
add_library(bsp_board STATIC
  board_io.c
  board_led.c
  board_key.c
)

target_include_directories(bsp_board PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bsp_board PUBLIC stm32cubemx)
