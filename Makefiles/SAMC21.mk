# Makefile for the SAMC21 bootloader configuration
# Bootloader for SAMC21 expansion and tool boards

# Board name
BOARD := SAMC21

# Output binary name
BINARY := Duet3Bootloader-SAMC21

# MCU configuration
MCU := SAMC21G18A
MCU_ARCH := cortex-m0plus
LINK_ARCH := cortex-m0plus

# Compiler defines
# C files also get the noexcept define so shared headers compile as C
C_DEFINES := -D__SAMC21G18A__ -Dnoexcept=
CXX_DEFINES := -D__SAMC21G18A__

# Optimization and debug
OPT := -Os
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/SAMC21/samc21g18a_flash.ld

# Source directories (relative to project root)
SRC_DIRS := \
	src \
	src/CAN \
	src/Hardware/SAMC21

# Include paths for C files (minimal set)
C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAMC21 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAMC21_DFP/1.2.176/samc21/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAMC21 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21 \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAMC21/hal/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAMC21/hal/utils/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAMC21/hri \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAMC21_DFP/1.2.176/samc21/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/RRFLibraries/src

# Libraries
LIBS := \
	-L$(WORKSPACE)/CoreN2G/SAMC21_CAN \
	-L$(WORKSPACE)/RRFLibraries/SAMC21 \
	-L$(WORKSPACE)/CANlib/SAMC21_nonRTOS \
	-lCoreN2G -lRRFLibraries -lCANlib

# Library dependencies
LIB_DEPS := \
	$(WORKSPACE)/CoreN2G/SAMC21_CAN/libCoreN2G.a \
	$(WORKSPACE)/RRFLibraries/SAMC21/libRRFLibraries.a \
	$(WORKSPACE)/CANlib/SAMC21_nonRTOS/libCANlib.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib -Wundef -Wdouble-promotion -Werror=return-type \
	-fsingle-precision-constant

# C flags
CFLAGS := $(COMMON_FLAGS) $(OPT) $(C_DEFINES) $(C_INCLUDES) -std=gnu99 $(CFLAGS_EXTRA)

# Build directory
BUILD_DIR := $(BOARD)

# C++ flags
CXXFLAGS := $(COMMON_FLAGS) $(OPT) $(CXX_DEFINES) $(CXX_INCLUDES) -std=c++20 \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions $(CXXFLAGS_EXTRA)

# Linker flags
LDFLAGS := $(LDOPT) --specs=nano.specs -Wl,--gc-sections -Wl,--entry=Reset_Handler \
	-Wl,--fatal-warnings -Wl,--no-warn-rwx-segment -mcpu=$(LINK_ARCH) \
	-T$(LINKER_SCRIPT) -Wl,-Map,$(CURDIR)/$(BUILD_DIR)/$(BINARY).map,--cref

# Find all source files
C_SRC := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))
CXX_SRC := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.cpp))

# Object files
C_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SRC))
CXX_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CXX_SRC))
OBJS := $(C_OBJS) $(CXX_OBJS)

# Dependency files
DEPS := $(OBJS:.o=.d)

# Output files
ELF := $(BUILD_DIR)/$(BINARY).elf
BIN := $(BUILD_DIR)/$(BINARY).bin
HEX := $(BUILD_DIR)/$(BINARY).hex

# Default target
.PHONY: $(BOARD)
$(BOARD): $(BIN)
	$(Q)echo ""
	$(Q)echo "Build complete for $(BOARD):"
	$(Q)$(SIZE) $(ELF)
	$(Q)echo ""
	$(Q)echo "Binary: $(BIN)"

# Link
$(ELF): $(OBJS) $(LIB_DEPS)
	$(Q)echo "  LD      $(notdir $@)"
	$(Q)$(LD) $(OBJS) $(LIBS) $(LDFLAGS) -o $@

# Generate binary and append CRC
$(BIN): $(ELF)
	$(Q)echo "  OBJCOPY $(notdir $@)"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)echo "  OBJCOPY $(notdir $(HEX))"
	$(Q)$(OBJCOPY) -O ihex $(ELF) $(HEX)
	$(Q)echo "  CRC     $(notdir $@)"
	$(Q)if command -v CrcAppender > /dev/null 2>&1; then \
		CrcAppender $@; \
	else \
		echo "  WARNING CrcAppender not found, skipping CRC"; \
	fi

# Compile C files
$(BUILD_DIR)/%.o: %.c
	$(Q)echo "  CC      $<"
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Compile C++ files
$(BUILD_DIR)/%.o: %.cpp
	$(Q)echo "  CXX     $<"
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

# Clean target
.PHONY: clean-$(BOARD)
clean-$(BOARD):
	$(Q)echo "  RM      $(BUILD_DIR)"
	$(Q)rm -rf $(BUILD_DIR)

# Include dependencies
-include $(DEPS)
