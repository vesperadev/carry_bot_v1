# Link peer: BT (Bridge M*/S*) or RPI
# Switch: cmake -DLINK_PEER=RPI ...
set(LINK_PEER "BT" CACHE STRING "USART6 link peer: BT or RPI")
set_property(CACHE LINK_PEER PROPERTY STRINGS BT RPI)

add_library(link INTERFACE)

target_sources(link INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/link.cpp
)

if(LINK_PEER STREQUAL "RPI")
    target_sources(link INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/link_proto_rpi.cpp
    )
    target_compile_definitions(link INTERFACE LINK_PEER_RPI=1)
else()
    # BT: Link_GetProtocol() comes from app/Bridge (A-tier M*/S*)
    target_compile_definitions(link INTERFACE LINK_PEER_BT=1)
endif()

target_include_directories(link INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
