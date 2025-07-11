cmake_minimum_required(VERSION 3.10)
project(haptic_glove_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/headers"
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
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/bin/win64"
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

# Create the driver DLL with the EXACT name SteamVR expects
add_library(driver_haptic_glove_driver SHARED ${SOURCES})

# Link libraries
target_link_libraries(driver_haptic_glove_driver ${OPENVR_LIBRARIES})

# Set output directory and ensure correct naming
set_target_properties(driver_haptic_glove_driver PROPERTIES
    OUTPUT_NAME "driver_haptic_glove_driver"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64
)

# Copy driver files to output directory
file(COPY ${CMAKE_SOURCE_DIR}/driver.vrdrivermanifest 
     DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/)

file(COPY ${CMAKE_SOURCE_DIR}/resources/
     DESTINATION ${CMAKE_BINARY_DIR}/haptic_glove_driver/resources/)

# Post-build step to copy DLL to correct location and then to SteamVR
if(WIN32)
    # Copy DLL from Release folder to win64 folder
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:driver_haptic_glove_driver>
        ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64/driver_haptic_glove_driver.dll
        COMMENT "Copying DLL to correct location"
    )
    
    # Copy OpenVR API DLL if it exists
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENVR_LIBRARIES}"
        ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64/
        COMMENT "Copying OpenVR API DLL"
        VERBATIM
    )
    
    # Try to find and copy openvr_api.dll from SteamVR
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/bin/win64/openvr_api.dll"
        ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64/
        COMMENT "Copying OpenVR API DLL from SteamVR"
        VERBATIM
    )
    
    # Copy entire driver folder to SteamVR (optional)
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_BINARY_DIR}/haptic_glove_driver
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver"
        COMMENT "Copying driver to SteamVR drivers folder"
    )
endif()

# Debug information
message(STATUS "OpenVR Include Dir: ${OPENVR_INCLUDE_DIRS}")
message(STATUS "OpenVR Library: ${OPENVR_LIBRARIES}")
message(STATUS "Output Directory: ${CMAKE_BINARY_DIR}/haptic_glove_driver/bin/win64")