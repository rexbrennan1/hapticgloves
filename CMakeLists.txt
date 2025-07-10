cmake_minimum_required(VERSION 3.10)
project(haptic_glove_driver)

set(CMAKE_CXX_STANDARD 17)

# Set compiler flags for Visual Studio 2022
if(MSVC)
    # Disable specific warnings that cause issues in VS2022
    add_compile_options(
        /W3                     # Warning level 3
        /wd4996                 # Disable deprecated function warnings
        /wd4244                 # Disable conversion warnings
        /wd4267                 # Disable size_t conversion warnings
        /DNOMINMAX              # Prevent Windows.h from defining min/max macros
        /D_CRT_SECURE_NO_WARNINGS
    )
    
    # Enable UTF-8 source and execution character sets
    add_compile_options(/utf-8)
endif()

# Find OpenVR
find_path(OPENVR_INCLUDE_DIRS
    NAMES
        openvr_driver.h
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers
        ${CMAKE_CURRENT_SOURCE_DIR}/../openvr/headers
        "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/headers"
        "C:/openvr/headers"
    PATH_SUFFIXES
        openvr
)

find_library(OPENVR_LIBRARIES
    NAMES
        openvr_api
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/win64
        ${CMAKE_CURRENT_SOURCE_DIR}/../openvr/lib/win64
        "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/bin/win64"
        "C:/openvr/lib/win64"
)

if(NOT OPENVR_INCLUDE_DIRS OR NOT OPENVR_LIBRARIES)
    message(FATAL_ERROR "OpenVR not found. Please install OpenVR SDK")
endif()

# Include directories
include_directories(${OPENVR_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/driver_haptic_glove.cpp
)

# Create the driver DLL
add_library(driver_haptic_glove SHARED ${SOURCES})

# Link libraries
target_link_libraries(driver_haptic_glove ${OPENVR_LIBRARIES})

# Set output directory
set_target_properties(driver_haptic_glove PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
)

# Copy driver files to output directory
file(COPY ${CMAKE_SOURCE_DIR}/driver.vrdrivermanifest 
     DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/)

file(COPY ${CMAKE_SOURCE_DIR}/resources/
     DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/resources/)

# Post-build step to copy to SteamVR drivers folder (optional)
if(WIN32)
    add_custom_command(TARGET driver_haptic_glove POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_BINARY_DIR}/haptic_glove_driver
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver"
        COMMENT "Copying driver to SteamVR drivers folder"
    )
endif()