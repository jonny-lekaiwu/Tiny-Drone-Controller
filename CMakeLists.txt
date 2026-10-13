cmake_minimum_required(VERSION 3.16)
file(STRINGS "${CMAKE_SOURCE_DIR}/version.txt" PROJECT_VER LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/version.txt")
if(NOT PROJECT_VER MATCHES "^[0-9][0-9]\\.[0-9][0-9]\\.[0-9][0-9]$")
    message(FATAL_ERROR "version.txt must use XX.XX.XX format")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake) 
project(drone_control)
if(NOT IDF_TARGET STREQUAL "esp32c3")
    message(FATAL_ERROR "Tiny-Drone-Controller requires esp32c3")
endif()

add_custom_target(flash_all ALL
    COMMAND "${PYTHON}" "${CMAKE_SOURCE_DIR}/scripts/merge_firmware.py"
        --build-dir "${CMAKE_BINARY_DIR}" --output "${CMAKE_SOURCE_DIR}/flash_all.bin"
    DEPENDS gen_project_binary bootloader partition_table_bin blank_ota_data
    COMMENT "Merging Controller flash_all.bin"
    VERBATIM
    USES_TERMINAL
)
