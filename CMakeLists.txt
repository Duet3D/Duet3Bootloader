# Duet3Bootloader: the bootloaders for the Duet 3 expansion and tool boards, and the CAN IAPs for
# the main boards, built as part of DuetFirmware against the shared libraries in lib/. A build tree
# builds one board, DUET_BOARD, as an executable target of the same name; Boards.cmake describes
# the boards. Its image goes to the top of the build tree: the .elf, the .bin with its CRC appended,
# the .map, and a .hex for the bootloaders.

cmake_minimum_required(VERSION 3.25)
project(Duet3Bootloader LANGUAGES C CXX)

# The source directories, in link order; each holds the sources directly in it. The C sources come
# before the C++ ones.
set(_c)
set(_cxx)
foreach(_dir src src/CAN src/Hardware/${BL_HARDWARE})
    file(GLOB _dir_c   CONFIGURE_DEPENDS ${_dir}/*.c)
    file(GLOB _dir_cxx CONFIGURE_DEPENDS ${_dir}/*.cpp)
    list(APPEND _c ${_dir_c})
    list(APPEND _cxx ${_dir_cxx})
endforeach()

duet_firmware_executable(${DUET_BOARD}
    OUTPUT_NAME ${BL_OUTPUT}
    LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/Hardware/${BL_HARDWARE}/${BL_LINKER_SCRIPT}"
    SOURCES ${_c} ${_cxx})

target_include_directories(${DUET_BOARD} PRIVATE src src/Hardware/${BL_HARDWARE})

target_compile_definitions(${DUET_BOARD} PRIVATE
    ${DUET_PART_DEFINE}
    ${BL_DEFINES}
    $<$<COMPILE_LANGUAGE:C>:noexcept=>)

target_compile_options(${DUET_BOARD} PRIVATE
    $<$<CONFIG:Release>:-Os>
    -fsingle-precision-constant
    -Wundef
    -Wdouble-promotion
    -Werror=return-type
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics;-fno-rtti;-fno-exceptions>)

target_link_options(${DUET_BOARD} PRIVATE
    --specs=nano.specs
    LINKER:--no-warn-rwx-segment
    ${BL_LINK_OPTIONS})

# The libraries in the order the linker scans them, which decides the layout of the image. As a
# group, so that each archive appears once on the command line although they call each other.
target_link_libraries(${DUET_BOARD} PRIVATE
    "$<LINK_GROUP:RESCAN,Duet::CoreN2G,Duet::RRFLibraries,Duet::CANlib>")

if(BL_HEX)
    duet_firmware_images(${DUET_BOARD} CRC HEX)
else()
    duet_firmware_images(${DUET_BOARD} CRC)
endif()
