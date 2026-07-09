# Duet3Bootloader Master Makefile
# Builds the bootloader and CAN in-application programmer for the Duet 3 boards

# Cross-compiler toolchain (relative to project root)
#CROSS_COMPILE ?= ../arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
CROSS_COMPILE ?= ../arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
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
	DEBUG_FLAGS := -g3 -Og
	$(info Building with debug symbols enabled)
else
	DEBUG_FLAGS :=
endif
export DEBUG_FLAGS

# Default target
.DEFAULT_GOAL := help

# Available build configurations
CONFIGS := SAME5x SAME5x_composite SAMC21 SAMMYC21 Duet3_MB6HC Duet3_MB6XD Duet3_Mini5plus

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
	$(Q)echo "  Duet3_MB6HC         - CAN in-application programmer for the MB6HC main board"
	$(Q)echo "  Duet3_MB6XD         - CAN in-application programmer for the MB6XD main board"
	$(Q)echo "  Duet3_Mini5plus     - CAN in-application programmer for the Mini 5+ main board"
	$(Q)echo ""
	$(Q)echo "Other targets:"
	$(Q)echo "  all                 - Build all configurations"
	$(Q)echo "  clean               - Clean all build outputs"
	$(Q)echo "  clean-all           - Clean all build outputs and libraries"
	$(Q)echo "  clean-<config>      - Clean specific configuration"
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
.PHONY: clean
clean:
	$(Q)echo "Cleaning all build outputs..."
	$(Q)for config in $(CONFIGS); do \
		if [ -d "$$config" ]; then \
			echo "  RM      $$config"; \
			rm -rf "$$config"; \
		fi; \
	done
	$(Q)echo "Clean complete"

# Clean all including libraries
.PHONY: clean-all
clean-all: clean
	$(Q)echo "Cleaning library dependencies..."
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib clean
	$(Q)echo "Clean all complete"
