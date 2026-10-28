# The Duet3Bootloader boards - the bootloaders for the expansion and tool boards, and the CAN IAPs
# that update a main board from its expansion boards - everything about DUET_BOARD apart from its
# MCU's toolchain file, which its preset selects. Included by the top-level CMakeLists.txt before
# the libraries are added, because it also sets the options they are built with. The bootloaders
# for the expansion and tool boards are named by the MCU they are built for, as each serves every
# board with that MCU; SAMMYC21 is the one board with a bootloader of its own.
#
# Variants of a board, which its presets set:
#   BL_COMPOSITE      the SAME5x bootloader for composite boards, which is linked above the first 4K
#
# Library configuration:
#   DUET_LIBRARIES    the libraries to build
#   DUET_PART_DEFINE  macro naming the MCU part
#   COREN2G_CAN, RRFLIBRARIES_PRINTF_FLOAT, and the libraries' optimisation
#
# Firmware:
#   BL_MCU            the DUET_MCU the board is built for
#   BL_OUTPUT         image file name, without extension
#   BL_DEFINES        macros that select the board's variant
#   BL_HARDWARE       directory under src/Hardware with the board's MCU support
#   BL_LINKER_SCRIPT  linker script, under src/Hardware/<BL_HARDWARE>
#   BL_HEX            also produce an Intel hex image
#   BL_COMPILE_OPTIONS extra options for compiling the board's sources
#   BL_LINK_OPTIONS   extra options for the link

# The bootloaders run without an RTOS, need only CAN from CoreN2G, and are built for size
set(DUET_LIBRARIES RRFLibraries CANlib CoreN2G)
set(DUET_RTOS OFF)
set(COREN2G_CAN ON)
set(RRFLIBRARIES_PRINTF_FLOAT OFF)
set(RRFLIBRARIES_OPTIMISATION -Os)
set(COREN2G_OPTIMISATION -Os)

option(BL_COMPOSITE "Build the SAME5x bootloader for composite boards" OFF)

set(BL_OUTPUT Duet3Bootloader-${DUET_BOARD})
set(BL_DEFINES)
set(BL_HEX ON)
set(BL_COMPILE_OPTIONS)
set(BL_LINK_OPTIONS)

if(DUET_BOARD STREQUAL "SAME5x")
    set(BL_MCU SAME5x)
    set(DUET_PART_DEFINE __SAME51N19A__)
    set(BL_HARDWARE SAME51)
    if(BL_COMPOSITE)
        string(APPEND BL_OUTPUT _composite)
        set(BL_DEFINES COMPOSITE)
        set(BL_LINKER_SCRIPT same51n19a_flash_0x1000.ld)
    else()
        set(BL_LINKER_SCRIPT same51n19a_flash.ld)
    endif()
elseif(DUET_BOARD MATCHES "^(SAMC21|SAMMYC21)$")
    set(BL_MCU SAMC21)
    set(DUET_PART_DEFINE __SAMC21G18A__)
    set(BL_HARDWARE SAMC21)
    set(BL_LINKER_SCRIPT samc21g18a_flash.ld)
    if(DUET_BOARD STREQUAL "SAMMYC21")
        set(BL_DEFINES SAMMYC21)
    endif()
elseif(DUET_BOARD STREQUAL "STM32H5")
    set(BL_MCU STM32H5)
    set(DUET_PART_DEFINE STM32H523xx)
    set(BL_HARDWARE STM32H5)
    set(BL_LINKER_SCRIPT stm32h523xx_flash.ld)
    set(BL_COMPILE_OPTIONS -fno-math-errno)
    set(CANLIB_OPTIMISATION -Os)
elseif(DUET_BOARD MATCHES "^(Duet3_MB6HC|Duet3_MB6XD)$")
    set(BL_MCU SAME70)
    set(DUET_PART_DEFINE __SAME70Q20B__)
    string(REPLACE "Duet3_" "" _board ${DUET_BOARD})
    set(BL_OUTPUT Duet3_CANiap32_${_board})
    set(BL_DEFINES ${_board} CAN_IAP)
    set(BL_HARDWARE SAME70)
    set(BL_LINKER_SCRIPT same70_iap_ram.ld)
    set(BL_HEX OFF)
    set(COREN2G_OPTIMISATION -O3)
    # Linked against the Cortex-M4 runtime libraries, as the Eclipse project does - possibly a typo
    set(BL_LINK_OPTIONS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
elseif(DUET_BOARD STREQUAL "Duet3Mini5plus")
    set(BL_MCU SAME5x)
    set(DUET_PART_DEFINE __SAME54P20A__)
    set(BL_OUTPUT Duet3_CANiap32_Mini5plus)
    set(BL_DEFINES CAN_IAP)
    set(BL_HARDWARE SAME51)
    set(BL_LINKER_SCRIPT same54p20a_iap_ram.ld)
    set(BL_HEX OFF)
else()
    message(FATAL_ERROR "Unknown DUET_BOARD '${DUET_BOARD}': configure with a board's preset (cmake --list-presets)")
endif()

if(BL_COMPOSITE AND NOT DUET_BOARD STREQUAL "SAME5x")
    message(FATAL_ERROR "BL_COMPOSITE is a variant of the SAME5x bootloader, not of ${DUET_BOARD}")
endif()

if(NOT DUET_MCU STREQUAL BL_MCU)
    message(FATAL_ERROR "${DUET_BOARD} is a ${BL_MCU} board, but this build tree is for ${DUET_MCU}: configure it with its preset")
endif()
