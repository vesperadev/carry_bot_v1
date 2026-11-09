cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME carry_bot_v1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# --- fw_version (git branch/hash/desc for 串口1 ver) ---
set(FW_VERSION_H "${CMAKE_BINARY_DIR}/generated/fw_version.h")
add_custom_command(
    OUTPUT "${FW_VERSION_H}"
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUT_H=${FW_VERSION_H}
            -P ${CMAKE_SOURCE_DIR}/cmake/gen_fw_version.cmake
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating fw_version.h from git"
    VERBATIM
)
add_custom_target(gen_fw_version DEPENDS "${FW_VERSION_H}")
# Force regenerate each build (git HEAD may move without CMake reconfigure)
add_custom_target(gen_fw_version_always ALL
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DOUT_H=${FW_VERSION_H}
            -P ${CMAKE_SOURCE_DIR}/cmake/gen_fw_version.cmake
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Refreshing fw_version.h from git"
    VERBATIM
)
add_dependencies(${CMAKE_PROJECT_NAME} gen_fw_version_always)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR}/generated)


# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add user libraries
add_subdirectory(app/Link)
add_subdirectory(app/ZDT)
add_subdirectory(app/Servo)
add_subdirectory(app/Bridge)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Bridge test1m wheel geometry (optional overrides at configure time)
set(BRIDGE_WHEEL_DIAMETER_MM "100" CACHE STRING "Wheel diameter in mm (test1m)")
set(BRIDGE_WHEEL_GEAR_X100 "100" CACHE STRING "Motor:wheel ratio times 100 (100 = 1:1)")
set(BRIDGE_TEST1M_DIST_MM "1000" CACHE STRING "test1m forward distance in mm")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    BRIDGE_WHEEL_DIAMETER_MM=${BRIDGE_WHEEL_DIAMETER_MM}U
    BRIDGE_WHEEL_GEAR_X100=${BRIDGE_WHEEL_GEAR_X100}U
    BRIDGE_TEST1M_DIST_MM=${BRIDGE_TEST1M_DIST_MM}U
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    link
    zdt
    servo
    bridge

    # Add user defined libraries
)