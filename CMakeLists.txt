cmake_minimum_required(VERSION 3.20)
project(HapticGloveDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Visual Studio 2022 specific settings
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    add_compile_options(/W3)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# Find OpenVR
find_path(OPENVR_INCLUDE_DIR
    NAMES openvr_driver.h
    PATHS
    ${CMAKE_SOURCE_DIR}/third_party/openvr/headers
    ${CMAKE_SOURCE_DIR}/openvr/headers
    ENV OPENVR_ROOT_DIR
    PATH_SUFFIXES headers
)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OPENVR_PLATFORM "win64")
    else()
        set(OPENVR_PLATFORM "win32")
    endif()
elseif(APPLE)
    set(OPENVR_PLATFORM "osx32")
elseif(UNIX)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OPENVR_PLATFORM "linux64")
    else()
        set(OPENVR_PLATFORM "linux32")
    endif()
endif()

find_library(OPENVR_LIBRARY
    NAMES openvr_api
    PATHS
    ${CMAKE_SOURCE_DIR}/third_party/openvr/lib/${OPENVR_PLATFORM}
    ${CMAKE_SOURCE_DIR}/openvr/lib/${OPENVR_PLATFORM}
    ENV OPENVR_ROOT_DIR
    PATH_SUFFIXES lib/${OPENVR_PLATFORM}
)

if(NOT OPENVR_INCLUDE_DIR OR NOT OPENVR_LIBRARY)
    message(FATAL_ERROR "OpenVR not found. Please set OPENVR_ROOT_DIR or place OpenVR in third_party/openvr/")
endif()

# Source files
set(SOURCES
    src/driver_haptic_glove.cpp
    src/haptic_glove_device.cpp
    src/driverlog.cpp
)

set(HEADERS
    src/haptic_glove_device.h
    src/driverlog.h
)

# Create the driver library
add_library(driver_haptic_glove SHARED ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(driver_haptic_glove PRIVATE
    ${OPENVR_INCLUDE_DIR}
    src/
)

# Link libraries
target_link_libraries(driver_haptic_glove ${OPENVR_LIBRARY})

# Platform-specific settings
if(WIN32)
    target_compile_definitions(driver_haptic_glove PRIVATE _WIN32_WINNT=0x0A00)  # Windows 10
    target_compile_definitions(driver_haptic_glove PRIVATE NOMINMAX)
    target_compile_definitions(driver_haptic_glove PRIVATE WIN32_LEAN_AND_MEAN)
    
    # Visual Studio 2022 specific settings
    if(MSVC)
        target_compile_options(driver_haptic_glove PRIVATE /permissive-)
        target_compile_options(driver_haptic_glove PRIVATE /Zc:__cplusplus)
    endif()
    
    # Windows-specific libraries
    target_link_libraries(driver_haptic_glove ws2_32)
    
    # Set output directory
    set_target_properties(driver_haptic_glove PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/win64
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/win64
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/win64
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/win64
        LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/win64
        LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/win64
    )
elseif(UNIX)
    # Linux-specific libraries
    target_link_libraries(driver_haptic_glove pthread)
    
    # Set output directory
    set_target_properties(driver_haptic_glove PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/linux64
    )
endif()

# Copy resources
configure_file(
    ${CMAKE_SOURCE_DIR}/resources/driver.vrdrivermanifest
    ${CMAKE_BINARY_DIR}/driver.vrdrivermanifest
    COPYONLY
)

# Install rules
install(TARGETS driver_haptic_glove 
    DESTINATION bin/${OPENVR_PLATFORM})

install(FILES resources/driver.vrdrivermanifest 
    DESTINATION .)