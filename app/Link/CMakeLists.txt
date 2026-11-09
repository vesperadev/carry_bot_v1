# 链路对端：BT(蓝牙/AT 调试) 或 RPI(树莓派)
# 切换方式：cmake -DLINK_PEER=RPI ... 重新配置
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
    target_sources(link INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/link_proto_debug.cpp
    )
    target_compile_definitions(link INTERFACE LINK_PEER_BT=1)
endif()

target_include_directories(link INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
