cmake_minimum_required(VERSION 3.16)
# Set the project name and version
project(SuperDrone)

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# specifiy include directories for project
set(SOURCES
    src/preHalDrone.c
)

# Add the include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src
)

# register project components
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS src
    REQUIRES driver freertos arduino
)