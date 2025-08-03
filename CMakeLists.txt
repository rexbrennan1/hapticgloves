cmake_minimum_required(VERSION 3.16)
project(driver_hapticgloves)

set(CMAKE_CXX_STANDARD 17)

# Find OpenVR
find_path(OPENVR_ROOT_DIR
    NAMES
    headers/openvr.h
    PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr
    ${CMAKE_CURRENT_SOURCE_DIR}/openvr
)

if(WIN32)
    set(OPENVR_LIBRARIES ${OPENVR_ROOT_DIR}/lib/win64/openvr_api.lib)
elseif(UNIX AND NOT APPLE)
    set(OPENVR_LIBRARIES ${OPENVR_ROOT_DIR}/lib/linux64/libopenvr_api.so)
endif()

set(OPENVR_INCLUDE_DIRS ${OPENVR_ROOT_DIR}/headers)

# Create the driver DLL
add_library(driver_hapticgloves SHARED
    src/main.cpp
)

target_include_directories(driver_hapticgloves PRIVATE 
    ${OPENVR_INCLUDE_DIRS}
)

target_link_libraries(driver_hapticgloves 
    ${OPENVR_LIBRARIES}
)

# Set output directory
set_target_properties(driver_hapticgloves PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64"
)

# Copy OpenVR binaries if needed
if(WIN32)
    add_custom_command(TARGET driver_hapticgloves POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENVR_ROOT_DIR}/bin/win64/openvr_api.dll"
        $<TARGET_FILE_DIR:driver_hapticgloves>)
endif()