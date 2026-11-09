# UserLib/AHRS — Mahony + wheel chassis attitude
add_library(userlib_ahrs STATIC
  mahony_ahrs.c
  wheel_ahrs.c
)

target_include_directories(userlib_ahrs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(userlib_ahrs PUBLIC
  userlib_bmi088
  userlib_ist8310
)
