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

# Find OpenVR headers
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
include_directories(${OPENVR_INCLUDE_DIRS})

# Source files
add_library(driver_haptic_glove_driver SHARED
    src/driver_haptic_glove.cpp
)

# Output settings
set_target_properties(driver_haptic_glove_driver PROPERTIES
    OUTPUT_NAME "driver_haptic_glove_driver"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_BINARY_DIR}/bin
)

# Copy resources to build directory
file(COPY driver.vrdrivermanifest DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY resources/ DESTINATION ${CMAKE_BINARY_DIR}/resources/)

# Post-build: Copy everything to SteamVR (Updated for Index controller emulation)
if(WIN32)
    # Create the target directory structure
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory 
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver/bin/win64"
        COMMENT "Creating driver directory structure"
    )
    
    # Copy the DLL
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:driver_haptic_glove_driver>
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver/bin/win64/driver_haptic_glove_driver.dll"
        COMMENT "Copying driver DLL"
    )
    
    # Copy OpenVR DLL
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/bin/win64/openvr_api.dll"
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver/bin/win64/"
        COMMENT "Copying OpenVR API DLL"
    )
    
    # Copy manifest
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/driver.vrdrivermanifest
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver/"
        COMMENT "Copying driver manifest"
    )
    
    # Copy resources
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/resources
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/drivers/haptic_glove_driver/resources"
        COMMENT "Copying driver resources"
    )
    
    # Print installation success message
    add_custom_command(TARGET driver_haptic_glove_driver POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "=== Driver installed to SteamVR ==="
        COMMAND ${CMAKE_COMMAND} -E echo "Please restart SteamVR to load the driver"
        COMMENT "Installation complete"
    )
endif()