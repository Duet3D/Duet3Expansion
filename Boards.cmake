# The Duet3Expansion boards: everything about DUET_BOARD apart from its MCU's toolchain file, which
# its preset selects. Included by the top-level CMakeLists.txt before the libraries are added,
# because it also sets the options they are built with.
#
# Library configuration:
#   DUET_LIBRARIES    the libraries to build
#   DUET_PART_DEFINE  macro naming the MCU part
#   DUET_RTOS, COREN2G_CAN
#
# Firmware:
#   D3E_MCU            the DUET_MCU the board is built for
#   D3E_DEFINE         board macro, which selects the board's configuration headers
#   D3E_LINKER_SCRIPT  linker script, under src/
#   D3E_DRIVERS        build the device drivers in src/Hardware/Drivers

# Every expansion board runs FreeRTOS and talks to the main board over CAN
set(DUET_LIBRARIES FreeRTOS RRFLibraries CANlib CoreN2G)
set(DUET_RTOS ON)
set(COREN2G_CAN ON)

set(D3E_DEFINE ${DUET_BOARD})
set(D3E_DRIVERS OFF)

if(DUET_BOARD MATCHES "^(EXP3HC|EXP1HCL|F3PTB|M23CL|TOOL1RR|TOOLINDX)$")
    set(D3E_MCU SAME5x)
    set(D3E_LINKER_SCRIPT Hardware/SAME5x_C21/SAME5x/same51n19a_flash_with_bootloader.ld)
    if(DUET_BOARD STREQUAL "EXP3HC")
        set(DUET_PART_DEFINE __SAME51N19A__)
    elseif(DUET_BOARD STREQUAL "TOOLINDX")
        set(DUET_PART_DEFINE __SAME51J19A__)
        set(D3E_LINKER_SCRIPT Hardware/SAME5x_C21/SAME5x/same51n19a_flash_with_16k_bootloader.ld)
    else()
        set(DUET_PART_DEFINE __SAME51G19A__)
    endif()
    if(DUET_BOARD MATCHES "^(F3PTB|TOOL1RR|TOOLINDX)$")
        set(D3E_DRIVERS ON)
    endif()
elseif(DUET_BOARD MATCHES "^(EXP1XD|TOOL1LC|SAMMYC21|SZP)$")
    set(D3E_MCU SAMC21)
    set(DUET_PART_DEFINE __SAMC21G18A__)
    set(D3E_LINKER_SCRIPT Hardware/SAME5x_C21/SAMC21/samc21g18a_flash_with_bootloader.ld)
    # The Cortex-M0+ has no FPU: Qfplib does its single-precision arithmetic
    list(APPEND DUET_LIBRARIES Qfplib)
    if(DUET_BOARD MATCHES "^(SAMMYC21|SZP)$")
        set(D3E_DRIVERS ON)
    endif()
elseif(DUET_BOARD STREQUAL "NodeTrix")
    set(D3E_MCU STM32H5)
    set(DUET_PART_DEFINE STM32H523xx)
    set(D3E_DEFINE NODETRIX)
    set(D3E_LINKER_SCRIPT Hardware/STM32/STM32H5/stm32h523xx_flash.ld)
    set(D3E_DRIVERS ON)
else()
    message(FATAL_ERROR "Unknown DUET_BOARD '${DUET_BOARD}': configure with a board's preset (cmake --list-presets)")
endif()

if(NOT DUET_MCU STREQUAL D3E_MCU)
    message(FATAL_ERROR "${DUET_BOARD} is a ${D3E_MCU} board, but this build tree is for ${DUET_MCU}: configure it with its preset")
endif()
