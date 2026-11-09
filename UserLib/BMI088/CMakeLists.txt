# UserLib/BMI088 — polling SPI driver (QGimbal-adapted)
add_library(userlib_bmi088 STATIC
  bmi088.c
)

target_include_directories(userlib_bmi088 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(userlib_bmi088 PUBLIC stm32cubemx)
