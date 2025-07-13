# CMakeLists.txt for Skeletal Hand Tracking Driver
# This build system handles compilation and automatic SteamVR installation
# Key improvements: Automatic OpenVR detection, proper linking, and seamless deployment

cmake_minimum_required(VERSION 3.10)
project(haptic_glove_skeletal_driver VERSION 1.0.0)

# Modern C++ standard for better performance and safety
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project configuration
set(DRIVER_NAME "haptic_glove_driver")
set(TARGET_NAME "driver_${DRIVER_NAME}")

#=============================================================================
# Compiler Configuration
#=============================================================================

# Visual Studio configuration for real-time VR performance
if(MSVC)
    # Common flags for all configurations
    add_compile_options(
        /W3                           # Warning level 3
        /wd4996                       # Disable deprecated function warnings
        /wd4244 /wd4267              # Disable conversion warnings (common with OpenVR)
        /DNOMINMAX                    # Prevent min/max macro conflicts
        /D_CRT_SECURE_NO_WARNINGS    # Suppress secure CRT warnings
        /utf-8                        # UTF-8 source encoding
    )
    
    # Configuration-specific flags - this prevents conflicts
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /Zi /RTC1")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /DNDEBUG /fp:fast")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /O2 /Zi /DNDEBUG")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /O1 /DNDEBUG")
endif()

# GCC/Clang configuration for cross-platform support
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(
        -Wall -Wextra                 # Enable useful warnings
        -O3                           # Maximum optimization for VR performance
        -ffast-math                   # Fast math for real-time calculations
        -march=native                 # Optimize for target CPU
    )
endif()

#=============================================================================
# Platform Detection and Architecture Setup
#=============================================================================

# Determine target architecture for OpenVR library selection
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH_TARGET "64")
    set(PROCESSOR_ARCH "64")
else()
    set(ARCH_TARGET "32") 
    set(PROCESSOR_ARCH "32")
endif()

# Platform-specific settings
if(WIN32)
    set(PLATFORM_NAME "win")
    set(LIBRARY_PREFIX "")
    set(LIBRARY_SUFFIX ".dll")
    add_definitions(-DWIN32 -D_WINDOWS)
elseif(UNIX)
    set(PLATFORM_NAME "linux")
    set(LIBRARY_PREFIX "lib")
    set(LIBRARY_SUFFIX ".so")
    add_definitions(-DLINUX -DPOSIX)
elseif(APPLE)
    set(PLATFORM_NAME "osx")
    set(LIBRARY_PREFIX "lib")
    set(LIBRARY_SUFFIX ".dylib")
    add_definitions(-DOSX)
endif()

# Complete platform string for OpenVR paths
set(PLATFORM_TARGET "${PLATFORM_NAME}${ARCH_TARGET}")

message(STATUS "Building for platform: ${PLATFORM_TARGET}")

#=============================================================================
# OpenVR SDK Detection and Integration
#=============================================================================

# Multiple search strategies for finding OpenVR - this is crucial for reliability!

# Strategy 1: Look for OpenVR in common installation locations
find_path(OPENVR_ROOT_DIR
    NAMES headers/openvr_driver.h
    PATHS
        # SteamVR installation directories
        "$ENV{PROGRAMFILES}/Steam/steamapps/common/SteamVR"
        "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR" 
        "$ENV{ProgramFiles}/Steam/steamapps/common/SteamVR"
        "$ENV{ProgramFiles\(x86\)}/Steam/steamapps/common/SteamVR"
        
        # Alternative Steam locations
        "C:/Steam/steamapps/common/SteamVR"
        "D:/Steam/steamapps/common/SteamVR"
        "E:/Steam/steamapps/common/SteamVR"
        
        # Development SDK locations
        "${CMAKE_CURRENT_SOURCE_DIR}/openvr"
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr"
        "${CMAKE_CURRENT_SOURCE_DIR}/../openvr"
        "C:/openvr"
        "/usr/local/openvr"
        "/opt/openvr"
    DOC "OpenVR SDK root directory"
)

# Strategy 2: Direct header search if root not found
if(NOT OPENVR_ROOT_DIR)
    find_path(OPENVR_INCLUDE_DIRS
        NAMES openvr_driver.h
        PATHS
            "$ENV{PROGRAMFILES}/Steam/steamapps/common/SteamVR/headers"
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/headers"
            "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers"
            "C:/openvr/headers"
        PATH_SUFFIXES openvr
        DOC "OpenVR headers directory"
    )
else()
    set(OPENVR_INCLUDE_DIRS "${OPENVR_ROOT_DIR}/headers")
endif()

# Strategy 3: Find OpenVR library files
if(OPENVR_ROOT_DIR)
    set(OPENVR_LIB_DIR "${OPENVR_ROOT_DIR}/lib/${PLATFORM_TARGET}")
    set(OPENVR_BIN_DIR "${OPENVR_ROOT_DIR}/bin/${PLATFORM_TARGET}")
else()
    # Fallback search
    find_path(OPENVR_LIB_DIR
        NAMES "openvr_api.lib" "libopenvr_api.so" "libopenvr_api.dylib"
        PATHS
            "$ENV{PROGRAMFILES}/Steam/steamapps/common/SteamVR/lib/${PLATFORM_TARGET}"
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR/lib/${PLATFORM_TARGET}"
            "${CMAKE_CURRENT_SOURCE_DIR}/openvr/lib/${PLATFORM_TARGET}"
        DOC "OpenVR library directory"
    )
endif()

# Find the actual library file
find_library(OPENVR_LIBRARIES
    NAMES openvr_api
    PATHS "${OPENVR_LIB_DIR}" "${OPENVR_BIN_DIR}"
    NO_DEFAULT_PATH
    DOC "OpenVR API library"
)

# Validation and error handling
if(NOT OPENVR_INCLUDE_DIRS)
    message(FATAL_ERROR 
        "OpenVR headers not found! Please install SteamVR or download OpenVR SDK.\n"
        "Searched in:\n"
        "  - Steam installation directories\n" 
        "  - ${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers\n"
        "  - C:/openvr/headers\n"
        "Download OpenVR SDK from: https://github.com/ValveSoftware/openvr")
endif()

if(NOT OPENVR_LIBRARIES)
    message(FATAL_ERROR
        "OpenVR library not found! Please install SteamVR or download OpenVR SDK.\n"
        "Expected library in: ${OPENVR_LIB_DIR}")
endif()

# Success messages
message(STATUS "OpenVR Headers: ${OPENVR_INCLUDE_DIRS}")
message(STATUS "OpenVR Library: ${OPENVR_LIBRARIES}")
if(OPENVR_ROOT_DIR)
    message(STATUS "OpenVR Root: ${OPENVR_ROOT_DIR}")
endif()

#=============================================================================
# Build Target Configuration
#=============================================================================

# Source files for our skeletal driver
set(DRIVER_SOURCES
    src/driver_haptic_glove.cpp
)

# Create the shared library that SteamVR will load
add_library(${TARGET_NAME} SHARED ${DRIVER_SOURCES})

# Include directories
target_include_directories(${TARGET_NAME} PRIVATE 
    ${OPENVR_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link OpenVR library
target_link_libraries(${TARGET_NAME} PRIVATE ${OPENVR_LIBRARIES})

# Platform-specific linking
if(WIN32)
    # Windows system libraries needed for serial communication
    target_link_libraries(${TARGET_NAME} PRIVATE
        kernel32 user32 gdi32 winspool shell32 ole32 oleaut32 uuid comdlg32 advapi32
    )
elseif(UNIX)
    # Linux system libraries
    target_link_libraries(${TARGET_NAME} PRIVATE
        pthread dl
    )
endif()

#=============================================================================
# Output Configuration
#=============================================================================

# Set the output name to match OpenVR driver conventions
set_target_properties(${TARGET_NAME} PROPERTIES
    OUTPUT_NAME "driver_${DRIVER_NAME}"
    PREFIX ""  # No 'lib' prefix on any platform
    
    # Organize build outputs
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    
    # Ensure consistent output across build types
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/bin/${PLATFORM_TARGET}"
)

#=============================================================================
# Resource File Management
#=============================================================================

# Copy driver manifest - this tells SteamVR about our driver
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver.vrdrivermanifest"
    "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/driver.vrdrivermanifest"
    COPYONLY
)

# Copy settings file
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/settings/default.vrsettings"
    "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/resources/settings/default.vrsettings"
    COPYONLY
)

# Copy input profile - crucial for skeletal input to work
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/input/hand_tracking_profile.json"
    "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/resources/input/hand_tracking_profile.json"
    COPYONLY
)

# Copy any icon files
file(GLOB ICON_FILES "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/*")
if(ICON_FILES)
    foreach(ICON_FILE ${ICON_FILES})
        get_filename_component(ICON_NAME ${ICON_FILE} NAME)
        configure_file(
            ${ICON_FILE}
            "${CMAKE_BINARY_DIR}/${DRIVER_NAME}/resources/icons/${ICON_NAME}"
            COPYONLY
        )
    endforeach()
endif()

#=============================================================================
# SteamVR Installation and Registration
#=============================================================================

# Find SteamVR installation for automatic deployment
if(WIN32)
    # Multiple strategies for finding Steam on Windows
    find_path(STEAMVR_ROOT_DIR
        NAMES "bin/${PLATFORM_TARGET}/vrpathreg.exe"
        PATHS
            "$ENV{PROGRAMFILES}/Steam/steamapps/common/SteamVR"
            "$ENV{PROGRAMFILES\(X86\)}/Steam/steamapps/common/SteamVR"
            "$ENV{ProgramFiles}/Steam/steamapps/common/SteamVR"
            "$ENV{ProgramFiles\(x86\)}/Steam/steamapps/common/SteamVR"
            "C:/Steam/steamapps/common/SteamVR"
            "D:/Steam/steamapps/common/SteamVR"
            "E:/Steam/steamapps/common/SteamVR"
        DOC "SteamVR installation directory"
    )
elseif(UNIX)
    find_path(STEAMVR_ROOT_DIR
        NAMES "bin/linux64/vrpathreg"
        PATHS
            "$ENV{HOME}/.steam/steam/steamapps/common/SteamVR"
            "$ENV{HOME}/.local/share/Steam/steamapps/common/SteamVR"
            "/usr/local/SteamVR"
        DOC "SteamVR installation directory"
    )
endif()

if(STEAMVR_ROOT_DIR)
    message(STATUS "SteamVR found: ${STEAMVR_ROOT_DIR}")
    
    # Target for installing driver to SteamVR
    add_custom_target(install_to_steamvr
        COMMENT "Installing driver to SteamVR..."
        
        # Create target directory
        COMMAND ${CMAKE_COMMAND} -E make_directory 
            "${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
        
        # Copy entire driver directory
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_BINARY_DIR}/${DRIVER_NAME}"
            "${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
            
        # Success message
        COMMAND ${CMAKE_COMMAND} -E echo "Driver installed to: ${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
        COMMAND ${CMAKE_COMMAND} -E echo "Please restart SteamVR to load the driver"
        
        DEPENDS ${TARGET_NAME}
    )
    
    # Automatic registration with SteamVR's path registry
    if(WIN32)
        set(VRPATHREG_EXECUTABLE "${STEAMVR_ROOT_DIR}/bin/${PLATFORM_TARGET}/vrpathreg.exe")
    else()
        set(VRPATHREG_EXECUTABLE "${STEAMVR_ROOT_DIR}/bin/${PLATFORM_TARGET}/vrpathreg")
    endif()
    
    if(EXISTS "${VRPATHREG_EXECUTABLE}")
        add_custom_target(register_driver
            COMMENT "Registering driver with SteamVR..."
            
            # Add driver path to SteamVR registry
            COMMAND "${VRPATHREG_EXECUTABLE}" adddriver 
                "${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
                
            COMMAND ${CMAKE_COMMAND} -E echo "Driver registered with SteamVR"
            COMMAND ${CMAKE_COMMAND} -E echo "Run 'make unregister_driver' to remove registration"
            
            DEPENDS install_to_steamvr
        )
        
        # Target for unregistering driver
        add_custom_target(unregister_driver
            COMMENT "Unregistering driver from SteamVR..."
            
            COMMAND "${VRPATHREG_EXECUTABLE}" removedriver 
                "${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
                
            COMMAND ${CMAKE_COMMAND} -E echo "Driver unregistered from SteamVR"
        )
    endif()
    
    # All-in-one target for complete installation
    add_custom_target(deploy
        COMMENT "Complete driver deployment to SteamVR"
        DEPENDS register_driver
    )
    
else()
    message(WARNING 
        "SteamVR installation not found. Manual installation required:\n"
        "1. Copy ${CMAKE_BINARY_DIR}/${DRIVER_NAME} to SteamVR/drivers/\n"
        "2. Restart SteamVR")
        
    # Manual installation target
    add_custom_target(install_to_steamvr
        COMMENT "SteamVR not found - manual installation required"
        COMMAND ${CMAKE_COMMAND} -E echo "Built driver is ready in: ${CMAKE_BINARY_DIR}/${DRIVER_NAME}"
        COMMAND ${CMAKE_COMMAND} -E echo "Copy this directory to your SteamVR/drivers/ folder"
        DEPENDS ${TARGET_NAME}
    )
endif()

#=============================================================================
# Development and Testing Targets
#=============================================================================

# Development build target with debugging info
add_custom_target(debug_build
    COMMENT "Building debug version with verbose logging"
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config Debug
)

# Clean installation target
add_custom_target(clean_install
    COMMENT "Removing driver from SteamVR and rebuilding"
    COMMAND ${CMAKE_COMMAND} -E remove_directory 
        "${STEAMVR_ROOT_DIR}/drivers/${DRIVER_NAME}"
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target deploy
)

# Show configuration summary
message(STATUS "")
message(STATUS "=== Haptic Glove Driver Configuration ===")
message(STATUS "Driver Name: ${DRIVER_NAME}")
message(STATUS "Target Platform: ${PLATFORM_TARGET}")
message(STATUS "Build Directory: ${CMAKE_BINARY_DIR}/${DRIVER_NAME}")
if(STEAMVR_ROOT_DIR)
    message(STATUS "SteamVR Directory: ${STEAMVR_ROOT_DIR}")
    message(STATUS "")
    message(STATUS "Build commands:")
    message(STATUS "  make                 - Build driver")
    message(STATUS "  make deploy          - Build and install to SteamVR")
    message(STATUS "  make unregister_driver - Remove from SteamVR")
else()
    message(STATUS "SteamVR: Not found (manual installation required)")
endif()
message(STATUS "==========================================")
message(STATUS "")