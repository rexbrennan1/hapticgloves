cmake_minimum_required(VERSION 3.10)
project(haptic_glove_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Visual Studio 2022 compiler settings
if(MSVC)
    add_compile_options(
        /W3
        /wd4996    # Disable deprecated warnings
        /wd4244    # Disable conversion warnings  
        /wd4267    # Disable size_t warnings
        /DNOMINMAX # Prevent min/max macro conflicts
        /D_CRT_SECURE_NO_WARNINGS
        /utf-8     # UTF-8 encoding
    )
endif()

# Find OpenVR headers (headers only, no linking required)
find_path(OPENVR_INCLUDE_DIRS
    NAMES openvr_driver.h
    PATHS
        "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/headers"
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers"
        "C:/openvr/headers"
    PATH_SUFFIXES openvr
    REQUIRED
)

message(STATUS "OpenVR Headers: ${OPENVR_INCLUDE_DIRS}")

# Include OpenVR headers
include_directories(${OPENVR_INCLUDE_DIRS})

# Source files
add_library(driver_haptic_glove_driver SHARED
    src/driver_haptic_glove.cpp
)

# Output settings
set_target_properties(driver_haptic_glove_driver PROPERTIES
    OUTPUT_NAME "driver_haptic_glove_driver"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
)

# Copy resources
file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/)
file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/resources/)

# Post-build: Copy to SteamVR and ensure openvr_api.dll is present
if(WIN32)
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        # Copy OpenVR DLL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/bin/win64/openvr_api.dll"
            ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64/
        # Copy entire driver to SteamVR
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/haptic_glove_driver
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver"
        COMMENT "Installing haptic glove driver to SteamVR"
    )
endif()