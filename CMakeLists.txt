# Duet3Expansion: the firmware for the Duet 3 expansion and tool boards, built as part of
# DuetFirmware against the shared libraries in lib/. A build tree builds one board, DUET_BOARD, as
# an executable target of the same name; Boards.cmake describes the boards. Its image goes to the
# top of the build tree: the .elf, the .bin with its CRC appended, and the .map.

cmake_minimum_required(VERSION 3.25)
project(Duet3Expansion LANGUAGES C CXX)

# The source directories, in link order; each holds the sources directly in it
set(_dirs
    src
    src/CAN
    src/ClosedLoop
    src/ClosedLoop/Encoders
    src/CommandProcessing
    src/Fans
    src/FilamentMonitors
    src/GPIO
    src/Hardware)
if(D3E_DRIVERS)
    list(APPEND _dirs src/Hardware/Drivers)
endif()
if(DUET_MCU STREQUAL "SAME5x")
    list(APPEND _dirs src/Hardware/SAME5x_C21 src/Hardware/SAME5x_C21/SAME5x)
elseif(DUET_MCU STREQUAL "SAMC21")
    list(APPEND _dirs src/Hardware/SAME5x_C21 src/Hardware/SAME5x_C21/SAMC21)
else()
    list(APPEND _dirs src/Hardware/STM32 src/Hardware/STM32/STM32H5)
endif()
list(APPEND _dirs
    src/Heating
    src/Heating/Sensors
    src/InputMonitors
    src/LedStrips
    src/Movement
    src/Movement/Kinematics
    src/Movement/StepperDrivers
    src/Platform)

# The C sources come before the C++ ones
set(_c)
set(_cxx)
foreach(_dir IN LISTS _dirs)
    file(GLOB _dir_c   CONFIGURE_DEPENDS ${_dir}/*.c)
    file(GLOB _dir_cxx CONFIGURE_DEPENDS ${_dir}/*.cpp)
    list(APPEND _c ${_dir_c})
    list(APPEND _cxx ${_dir_cxx})
endforeach()

duet_firmware_executable(${DUET_BOARD}
    OUTPUT_NAME Duet3Firmware_${DUET_BOARD}
    LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/${D3E_LINKER_SCRIPT}"
    SOURCES ${_c} ${_cxx})
# Version.cpp holds the build date
duet_build_date(${DUET_BOARD} src/Version.cpp)

target_include_directories(${DUET_BOARD} PRIVATE src)

target_compile_definitions(${DUET_BOARD} PRIVATE
    ${DUET_PART_DEFINE}
    ${D3E_DEFINE}
    $<$<COMPILE_LANGUAGE:C>:noexcept=>)

target_compile_options(${DUET_BOARD} PRIVATE
    $<$<CONFIG:Release>:-O3>
    -fno-math-errno
    -fsingle-precision-constant
    -Wall
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    -Werror
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fno-exceptions;-Wfloat-conversion;-Wshadow;-Wsign-promo>)

target_link_options(${DUET_BOARD} PRIVATE
    --specs=nano.specs
    LINKER:--no-warn-rwx-segment)

# The libraries in the order the linker scans them, which decides the layout of the image. As a
# group, so that each archive appears once on the command line although they call each other.
set(_libs Duet::CoreN2G Duet::CANlib Duet::RRFLibraries Duet::FreeRTOS)
if(DUET_MCU STREQUAL "SAMC21")
    # Route the compiler's single-precision soft-float calls to Qfplib
    foreach(_fn fadd fsub fmul fdiv i2f ui2f f2iz f2uiz)
        target_link_options(${DUET_BOARD} PRIVATE LINKER:-wrap,__aeabi_${_fn})
    endforeach()
    list(PREPEND _libs Duet::Qfplib)
endif()
list(JOIN _libs "," _group)
target_link_libraries(${DUET_BOARD} PRIVATE "$<LINK_GROUP:RESCAN,${_group}>")

duet_firmware_images(${DUET_BOARD} CRC)
