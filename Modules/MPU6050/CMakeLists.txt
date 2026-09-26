# CMakeLists.txt for MPU6050

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Auto-include source files (C for the ported DMP library, C++ for the port layer)
file(GLOB MODULE_MPU6050_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(xr PRIVATE ${MODULE_MPU6050_SRC})
