# Link peer: DEBUG (BT/AT) or RPI
set(BT_LINK_MODE "DEBUG" CACHE STRING "BT link peer: DEBUG or RPI")
set_property(CACHE BT_LINK_MODE PROPERTY STRINGS DEBUG RPI)

add_library(bt INTERFACE)

target_sources(bt INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/bt.cpp
)

if(BT_LINK_MODE STREQUAL "RPI")
    target_sources(bt INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/bt_proto_rpi.cpp
    )
    target_compile_definitions(bt INTERFACE BT_LINK_RPI=1)
else()
    target_sources(bt INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/bt_proto_debug.cpp
    )
    target_compile_definitions(bt INTERFACE BT_LINK_DEBUG=1)
endif()

target_include_directories(bt INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/generated
    ${CMAKE_SOURCE_DIR}/Applications/Inc
)
