# app/Bridge: BT text commands (M*/S*) as LinkProtocol when LINK_PEER=BT
# LINK_PEER is defined by app/Link (add_subdirectory order: Link before Bridge).
add_library(bridge INTERFACE)

target_include_directories(bridge INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/generated
    ${CMAKE_SOURCE_DIR}/app/Link
    ${CMAKE_SOURCE_DIR}/app/ZDT
    ${CMAKE_SOURCE_DIR}/app/Servo
)

if(NOT DEFINED LINK_PEER)
    set(LINK_PEER "BT")
endif()

if(NOT LINK_PEER STREQUAL "RPI")
    target_sources(bridge INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge_protocol.cpp
    )
endif()