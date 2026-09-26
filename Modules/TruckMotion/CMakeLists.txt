# CMakeLists.txt for TruckMotion

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Header-only module: the feature chain and state machine run inside
# MPU6050's publish callbacks (no timer task, no translation units of its
# own today). The GLOB is kept so future .cpp sources are picked up
# automatically, mirroring Modules/MPU6050/CMakeLists.txt.
file(GLOB MODULE_TRUCKMOTION_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

if(MODULE_TRUCKMOTION_SRC)
    target_sources(xr PRIVATE ${MODULE_TRUCKMOTION_SRC})
endif()
