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
set(CMAKE_PROJECT_NAME Clock)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# ---------------------------------------------------------------------------
# XRobot: 依据 User/xrobot.yaml 自动生成 xrobot_main.hpp
#
# 生成到构建目录（${CMAKE_BINARY_DIR}/generated），不污染源码树；当 xrobot.yaml、
# Modules/modules.yaml 或任一模块 MANIFEST 变化时自动重跑。接线与 PLDX_Template
# 保持一致，这样直接执行 `cmake --build` 也能拿到最新的入口头文件，无需手工运行
# xrobot_gen_main。
#
# 注意：本工程不使用 -Werror（Modules/MPU6050 内的上游 DMP 源码带有告警），
# 因此这里刻意不照搬模板的 add_compile_options(-Werror)。
# ---------------------------------------------------------------------------
set(XROBOT_CONFIG "${CMAKE_SOURCE_DIR}/User/xrobot.yaml" CACHE FILEPATH
    "Absolute path to the xrobot YAML configuration")
get_filename_component(XROBOT_CONFIG "${XROBOT_CONFIG}" ABSOLUTE
                       BASE_DIR "${CMAKE_SOURCE_DIR}")
find_program(XROBOT_GEN_MAIN xrobot_gen_main REQUIRED)
set(XROBOT_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
set(XROBOT_GENERATED_HEADER "${XROBOT_GENERATED_DIR}/xrobot_main.hpp")
file(GLOB_RECURSE XROBOT_MANIFEST_INPUTS CONFIGURE_DEPENDS
     "${CMAKE_SOURCE_DIR}/Modules/*.hpp")
set(XROBOT_GENERATION_INPUTS "${XROBOT_CONFIG}"
    "${CMAKE_SOURCE_DIR}/Modules/modules.yaml" ${XROBOT_MANIFEST_INPUTS})
if(EXISTS "${CMAKE_SOURCE_DIR}/Modules/module-lock.yaml")
  list(APPEND XROBOT_GENERATION_INPUTS
       "${CMAKE_SOURCE_DIR}/Modules/module-lock.yaml")
endif()
add_custom_command(
  OUTPUT "${XROBOT_GENERATED_HEADER}"
  BYPRODUCTS "${XROBOT_GENERATED_DIR}/xrobot_constexpr.hpp"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${XROBOT_GENERATED_DIR}"
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${XROBOT_GENERATED_DIR}/xrobot_constexpr.hpp"
  COMMAND "${XROBOT_GEN_MAIN}" --config "${XROBOT_CONFIG}"
          --output "${XROBOT_GENERATED_HEADER}"
  COMMAND "${CMAKE_COMMAND}" -E touch
          "${XROBOT_GENERATED_DIR}/xrobot_constexpr.hpp"
  DEPENDS ${XROBOT_GENERATION_INPUTS} "${XROBOT_GEN_MAIN}"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  VERBATIM)
add_custom_target(xrobot_generated_header DEPENDS "${XROBOT_GENERATED_HEADER}")
add_dependencies(${CMAKE_PROJECT_NAME} xrobot_generated_header)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE "${XROBOT_GENERATED_HEADER}")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

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

    # Add user defined libraries
)

# Add LibXR
include(${CMAKE_CURRENT_LIST_DIR}/cmake/LibXR.CMake)
