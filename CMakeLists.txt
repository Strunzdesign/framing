cmake_minimum_required(VERSION 2.6)

project(framing)

# Automatically set the version number
set(FRAMING_VERSION_MAJOR  \"1\")
set(FRAMING_VERSION_MINOR  \"0pre\")
configure_file (
    "${PROJECT_SOURCE_DIR}/src/FramingConfig.h.in"
    "${PROJECT_SOURCE_DIR}/src/FramingConfig.h"
)

add_subdirectory(src)
