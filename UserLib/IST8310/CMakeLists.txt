# UserLib/IST8310 — polling I2C3 magnetometer (Type-C onboard)
add_library(userlib_ist8310 STATIC
  ist8310.c
)

target_include_directories(userlib_ist8310 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(userlib_ist8310 PUBLIC stm32cubemx)
