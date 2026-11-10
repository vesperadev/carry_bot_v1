# app/Servo: PCA9685 on I2C2 (DMA + mutex), CH0 clamp / CH1 turntable
add_library(servo INTERFACE)

target_sources(servo INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/pca9685.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/servo.cpp
)

target_include_directories(servo INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)