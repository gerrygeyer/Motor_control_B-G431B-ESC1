cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DRONE_B-G431B-ESC1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Core sources are collected below via file(GLOB_RECURSE)
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Collect all source files from Core/Src and set include path for Core/Inc
file(GLOB_RECURSE CORE_SRC_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.S
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.s
)

# Remove system_stm32g4xx.c as it's already in STM32_Drivers
list(FILTER CORE_SRC_FILES EXCLUDE REGEX "system_stm32g4xx\\.c$")

if(CORE_SRC_FILES)
    message(STATUS "Found ${CORE_SRC_FILES}")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CORE_SRC_FILES})
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Force Thumb mode
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mthumb
    -mcpu=cortex-m4
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mthumb
    -mcpu=cortex-m4
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -lm
    # Add user defined libraries
)

# Debug & Flash Targets
add_custom_target(flash
    COMMAND bash -c "openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c init -c 'reset init' -c 'program ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf verify reset exit'"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing STM32 with OpenOCD..."
)
