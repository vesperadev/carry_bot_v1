# app/ZDT: Emm V5 motor bus on USART1 (DMA + address slots)
add_library(zdt INTERFACE)

target_sources(zdt INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/zdt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/zdt_protocol.cpp
)

target_include_directories(zdt INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
