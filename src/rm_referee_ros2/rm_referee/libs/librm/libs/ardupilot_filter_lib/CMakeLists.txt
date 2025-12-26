cmake_minimum_required(VERSION 3.15)
project(ardupilot_filter VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Core filter source files (excluding AP_Filter framework files)
set(FILTER_SOURCES
    LowPassFilter.cpp
    LowPassFilter2p.cpp
    NotchFilter.cpp
    HarmonicNotchFilter.cpp
    DerivativeFilter.cpp
    ModeFilter.cpp
    SlewLimiter.cpp
    # Note: AP_Filter.cpp, AP_Filter_params.cpp, AP_NotchFilter_params.cpp are excluded
    # They require the AP_Param framework which is not included in standalone build
)

# Core filter header files (now in ap_filters directory)
set(FILTER_HEADERS
    ap_filters/Filter.h
    ap_filters/FilterClass.h
    ap_filters/FilterWithBuffer.h
    ap_filters/AverageFilter.h
    ap_filters/Butter.h
    ap_filters/LowPassFilter.h
    ap_filters/LowPassFilter2p.h
    ap_filters/NotchFilter.h
    ap_filters/HarmonicNotchFilter.h
    ap_filters/DerivativeFilter.h
    ap_filters/ModeFilter.h
    ap_filters/SlewLimiter.h
    ap_filters/filter_config.h
    ap_filters/filter_math.h
    ap_filters/AP_Filter.h
    ap_filters/AP_Filter_config.h
)

# Create static library
add_library(ardupilot_filter STATIC ${FILTER_SOURCES})

# Set include directories
target_include_directories(ardupilot_filter
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(ardupilot_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ap_filters)

# Compiler options
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ardupilot_filter PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
    )
endif()

# Installation
install(TARGETS ardupilot_filter
    EXPORT ardupilot_filter-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ap_filters
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT ardupilot_filter-targets
    FILE ardupilot_filter-targets.cmake
    NAMESPACE ardupilot_filter::
    DESTINATION lib/cmake/ardupilot_filter
)

