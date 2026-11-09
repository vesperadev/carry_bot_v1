# Applications — FreeRTOS tasks (board IO + IMU)
add_library(applications STATIC
  Src/board_task.c
  Src/imu_task.c
)

target_include_directories(applications PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# bt_public.h only — BT_Send resolved at final link (exe already links bt).
# Do NOT target_link_libraries(bt): bt is INTERFACE and would duplicate sources.
target_include_directories(applications PRIVATE
  ${CMAKE_SOURCE_DIR}/app/BT
)

target_link_libraries(applications PUBLIC
  stm32cubemx
  bsp_board
  userlib_bmi088
  userlib_ist8310
)