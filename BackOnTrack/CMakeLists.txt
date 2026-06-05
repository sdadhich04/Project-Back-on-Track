cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/sensor_hal"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/data_types"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/posture"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/telemetry"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/ble"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(BackOnTrack)
