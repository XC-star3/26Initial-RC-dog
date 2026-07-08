set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings. Prefer PATH, but also look in the common
# STM32Cube bundle locations so the project survives being moved.
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(_TOOLCHAIN_HINT_DIRS)
if(DEFINED ENV{CUBE_BUNDLE_PATH})
    file(GLOB _CUBE_ENV_TOOL_DIRS "$ENV{CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*/bin")
    list(APPEND _TOOLCHAIN_HINT_DIRS ${_CUBE_ENV_TOOL_DIRS})
endif()

if(DEFINED ENV{USERPROFILE})
    file(GLOB _CUBE_USER_TOOL_DIRS "$ENV{USERPROFILE}/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/*/bin")
    list(APPEND _TOOLCHAIN_HINT_DIRS ${_CUBE_USER_TOOL_DIRS})
endif()

file(GLOB _CUBE_IDE_TOOL_DIRS
    "C:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin"
    "D:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin"
)
list(APPEND _TOOLCHAIN_HINT_DIRS ${_CUBE_IDE_TOOL_DIRS})

find_program(TOOLCHAIN_GCC NAMES arm-none-eabi-gcc arm-none-eabi-gcc.exe HINTS ${_TOOLCHAIN_HINT_DIRS})
if(NOT TOOLCHAIN_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc was not found. Install STM32CubeCLT/STM32CubeIDE or add the GNU Arm toolchain bin directory to PATH.")
endif()

get_filename_component(_TOOLCHAIN_BIN_DIR "${TOOLCHAIN_GCC}" DIRECTORY)
find_program(TOOLCHAIN_GXX NAMES arm-none-eabi-g++ arm-none-eabi-g++.exe HINTS "${_TOOLCHAIN_BIN_DIR}" REQUIRED)
find_program(TOOLCHAIN_OBJCOPY NAMES arm-none-eabi-objcopy arm-none-eabi-objcopy.exe HINTS "${_TOOLCHAIN_BIN_DIR}" REQUIRED)
find_program(TOOLCHAIN_SIZE NAMES arm-none-eabi-size arm-none-eabi-size.exe HINTS "${_TOOLCHAIN_BIN_DIR}" REQUIRED)

set(CMAKE_C_COMPILER                "${TOOLCHAIN_GCC}" CACHE FILEPATH "GNU Arm C compiler")
set(CMAKE_ASM_COMPILER              "${TOOLCHAIN_GCC}" CACHE FILEPATH "GNU Arm ASM compiler")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_GXX}" CACHE FILEPATH "GNU Arm CXX compiler")
set(CMAKE_LINKER                    "${TOOLCHAIN_GXX}" CACHE FILEPATH "GNU Arm linker")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_OBJCOPY}" CACHE FILEPATH "GNU Arm objcopy")
set(CMAKE_SIZE                      "${TOOLCHAIN_SIZE}" CACHE FILEPATH "GNU Arm size")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
