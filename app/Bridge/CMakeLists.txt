# app/Bridge: on this branch = USART1 sniff forward protocol (Link_GetProtocol).
# LINK_PEER is defined by app/Link (add_subdirectory order: Link before Bridge).
add_library(bridge INTERFACE)

target_include_directories(bridge INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/app/Link
    ${CMAKE_SOURCE_DIR}/app/ZDT
)

if(NOT DEFINED LINK_PEER)
    set(LINK_PEER "BT")
endif()

if(NOT LINK_PEER STREQUAL "RPI")
    target_sources(bridge INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/bridge_protocol.cpp
    )
endif()