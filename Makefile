# Duet3Bootloader Master Makefile
# Builds the bootloader and CAN in-application programmer for the Duet 3 boards

# Cross-compiler toolchain.
# RepRapFirmware exports CROSS_COMPILE when building this as a submodule.
# When building CoreN2G standalone, fall back to a toolchain on PATH.
ARM_GNU_TOOLCHAIN_VERSION ?= 15.2.rel1
ifeq ($(OS),Windows_NT)
HOST_ARCH_RAW := $(subst AMD64,x86_64,$(subst ARM64,aarch64,$(PROCESSOR_ARCHITECTURE)))
else
HOST_ARCH_RAW := $(shell uname -m)
HOST_OS_RAW := $(shell uname -s)
endif

ifeq ($(HOST_ARCH_RAW),aarch64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),arm64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := aarch64
else ifeq ($(HOST_ARCH_RAW),x86_64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else ifeq ($(HOST_ARCH_RAW),amd64)
ARM_GNU_TOOLCHAIN_HOST_ARCH := x86_64
else
ARM_GNU_TOOLCHAIN_HOST_ARCH := $(HOST_ARCH_RAW)
endif

ifeq ($(OS),Windows_NT)
ARM_GNU_TOOLCHAIN_HOST := mingw-w64-$(ARM_GNU_TOOLCHAIN_HOST_ARCH)
else ifeq ($(HOST_OS_RAW),Darwin)
ARM_GNU_TOOLCHAIN_HOST := darwin-$(subst aarch64,arm64,$(ARM_GNU_TOOLCHAIN_HOST_ARCH))
else
ARM_GNU_TOOLCHAIN_HOST := $(ARM_GNU_TOOLCHAIN_HOST_ARCH)
endif

CROSS_COMPILE ?= $(abspath ../arm-gnu-toolchain-$(ARM_GNU_TOOLCHAIN_VERSION)-$(ARM_GNU_TOOLCHAIN_HOST)-arm-none-eabi/bin/arm-none-eabi-)
export CROSS_COMPILE

# Toolchain programs
CC  := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
AS  := $(CROSS_COMPILE)gcc
AR  := $(CROSS_COMPILE)ar
LD  := $(CROSS_COMPILE)g++
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size
export CC CXX AS AR LD OBJCOPY SIZE

# Workspace root
WORKSPACE := ..
export WORKSPACE

# Quiet build support (Linux kernel style)
# Use V=1 for verbose output
ifeq ($(V),1)
	Q :=
	VERBOSE :=
else
	Q := @
	VERBOSE := -s
endif
export Q VERBOSE

# Debug build support
# Use DEBUG=1 to build with debug symbols and reduced optimization
ifeq ($(DEBUG),1)
DEBUG_FLAGS := -g3 -Og -DDEBUG
else
DEBUG_FLAGS :=
endif
export DEBUG_FLAGS

# Default target
.DEFAULT_GOAL := help

# Available build configurations
CONFIGS := SAME5x SAME5x_composite SAMC21 SAMMYC21 STM32H5 Duet3_MB6HC Duet3_MB6XD Duet3_Mini5plus

# Declare all board targets as phony
.PHONY: $(CONFIGS)

# Print available targets
.PHONY: help
help:
	$(Q)echo ""
	$(Q)echo "Duet3Bootloader Build System"
	$(Q)echo "============================="
	$(Q)echo ""
	$(Q)echo "Build targets:"
	$(Q)echo "  SAME5x              - Bootloader for SAME5x expansion and tool boards"
	$(Q)echo "  SAME5x_composite    - Composite SAME5x bootloader, linked at offset 0x1000"
	$(Q)echo "  SAMC21              - Bootloader for SAMC21 expansion and tool boards"
	$(Q)echo "  SAMMYC21            - Bootloader for the Sammy C21 board"
	$(Q)echo "  STM32H5             - Bootloader for the STM32H5 expansion and tool boards"
	$(Q)echo "  Duet3_MB6HC         - CAN in-application programmer for the MB6HC main board"
	$(Q)echo "  Duet3_MB6XD         - CAN in-application programmer for the MB6XD main board"
	$(Q)echo "  Duet3_Mini5plus     - CAN in-application programmer for the Mini 5+ main board"
	$(Q)echo ""
	$(Q)echo "Other targets:"
	$(Q)echo "  all                 - Build all configurations"
	$(Q)echo "  clean               - Clean all build outputs and libraries"
	$(Q)echo "  test-toolchain      - Verify toolchain is accessible"
	$(Q)echo ""
	$(Q)echo "Environment variables:"
	$(Q)echo "  CROSS_COMPILE       - Toolchain prefix (default: $(CROSS_COMPILE))"
	$(Q)echo "  V=1                 - Enable verbose build output"
	$(Q)echo "  DEBUG=1             - Build with debug symbols (-g3 -Og)"
	$(Q)echo ""
	$(Q)echo "Examples:"
	$(Q)echo "  make SAME5x                                # Build the SAME5x bootloader"
	$(Q)echo "  make SAMC21 V=1                            # Build with verbose output"

# Build all configurations
.PHONY: all
all:
	$(Q)for config in $(CONFIGS); do \
		$(MAKE) "$$config" || exit 1; \
	done

# Verify toolchain
.PHONY: test-toolchain
test-toolchain:
	$(Q)echo "Testing toolchain..."
	$(Q)if [ ! -f "$(CC)" ]; then \
		echo "ERROR: Toolchain not found at: $(CC)"; \
		echo "Please install the ARM GCC toolchain and set CROSS_COMPILE"; \
		exit 1; \
	fi
	$(Q)echo "Toolchain: $(CROSS_COMPILE)"
	$(Q)$(CC) --version | head -n 1
	$(Q)echo "Toolchain OK"

# Common library build rules (to avoid duplicate recipes in board makefiles)
# Phony so the sub-make always gets to decide: an archive left behind by a failed library build is otherwise never retried
.PHONY: $(WORKSPACE)/CoreN2G/SAME5x_CAN/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAMC21_CAN/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/SAME70_CAN/libCoreN2G.a \
        $(WORKSPACE)/CoreN2G/STM32H5_CAN/libCoreN2G.a \
        $(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAMC21/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a \
        $(WORKSPACE)/RRFLibraries/STM32H5/libRRFLibraries.a \
        $(WORKSPACE)/CANlib/SAME51_nonRTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/SAMC21_nonRTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/SAME70_nonRTOS/libCANlib.a \
        $(WORKSPACE)/CANlib/STM32H5_nonRTOS/libCANlib.a

$(WORKSPACE)/CoreN2G/SAME5x_CAN/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_CAN"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_CAN

$(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME51"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME51

$(WORKSPACE)/CANlib/SAME51_nonRTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME51_nonRTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAME51_nonRTOS

$(WORKSPACE)/CoreN2G/SAMC21_CAN/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAMC21_CAN"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAMC21_CAN

$(WORKSPACE)/RRFLibraries/SAMC21/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAMC21"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAMC21

$(WORKSPACE)/CANlib/SAMC21_nonRTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAMC21_nonRTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAMC21_nonRTOS

$(WORKSPACE)/CoreN2G/SAME70_CAN/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME70_CAN"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME70_CAN

$(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME70"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME70

$(WORKSPACE)/CANlib/SAME70_nonRTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME70_nonRTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAME70_nonRTOS

$(WORKSPACE)/CoreN2G/STM32H5_CAN/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/STM32H5_CAN"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G STM32H5_CAN

$(WORKSPACE)/RRFLibraries/STM32H5/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/STM32H5"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries STM32H5

$(WORKSPACE)/CANlib/STM32H5_nonRTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/STM32H5_nonRTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib STM32H5_nonRTOS

# Include the specific makefile based on the target
# Only include one at a time to avoid conflicts
ifneq ($(MAKECMDGOALS),)
ifneq ($(MAKECMDGOALS),all)
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),help)
ifneq ($(MAKECMDGOALS),test-toolchain)
ifeq ($(findstring clean-,$(MAKECMDGOALS)),)
# Include only the makefile for the requested target
ifeq ($(MAKECMDGOALS),SAME5x)
-include Makefiles/SAME5x.mk
endif
ifeq ($(MAKECMDGOALS),SAME5x_composite)
-include Makefiles/SAME5x_composite.mk
endif
ifeq ($(MAKECMDGOALS),SAMC21)
-include Makefiles/SAMC21.mk
endif
ifeq ($(MAKECMDGOALS),SAMMYC21)
-include Makefiles/SAMMYC21.mk
endif
ifeq ($(MAKECMDGOALS),STM32H5)
-include Makefiles/STM32H5.mk
endif
ifeq ($(MAKECMDGOALS),Duet3_MB6HC)
-include Makefiles/Duet3_MB6HC.mk
endif
ifeq ($(MAKECMDGOALS),Duet3_MB6XD)
-include Makefiles/Duet3_MB6XD.mk
endif
ifeq ($(MAKECMDGOALS),Duet3_Mini5plus)
-include Makefiles/Duet3_Mini5plus.mk
endif
endif
endif
endif
endif
endif
endif

# Generic clean target
# The objects do not depend on DEBUG_FLAGS, so switching DEBUG on or off only takes effect after the libraries have been cleaned too
.PHONY: clean
clean:
	$(Q)echo "Cleaning all build outputs..."
	$(Q)for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  RM      $$config"; \
			rm -rf "$$config"; \
		fi; \
	done
	$(Q)echo "Cleaning library dependencies..."
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib clean
	$(Q)echo "Clean complete"

.PHONY: clean-all
clean-all: clean
