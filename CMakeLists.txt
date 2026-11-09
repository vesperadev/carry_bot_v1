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

# Add user libraries (QGimbal-style: BSP / UserLib / Applications + app/BT)
add_subdirectory(BSP/Board)
add_subdirectory(UserLib/BMI088)
add_subdirectory(UserLib/IST8310)
add_subdirectory(UserLib/AHRS)
add_subdirectory(Applications)
add_subdirectory(app/BT)

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

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    bt
    bsp_board
    userlib_bmi088
    userlib_ist8310
    userlib_ahrs
    applications

    # Add user defined libraries
)
