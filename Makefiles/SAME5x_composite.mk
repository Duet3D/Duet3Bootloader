# Makefile for the SAME5x_composite bootloader configuration
# Composite bootloader for SAME5x boards, linked at offset 0x1000

# Board name
BOARD := SAME5x_composite

# Output binary name
BINARY := Duet3Bootloader-SAME5x_composite

# MCU configuration
MCU := SAME51N19A
MCU_ARCH := cortex-m4
LINK_ARCH := cortex-m4

# Compiler defines
# C files also get the noexcept define so shared headers compile as C
C_DEFINES := -D__SAME51N19A__ -Dnoexcept=
CXX_DEFINES := -D__SAME51N19A__ -DCOMPOSITE

# Optimization and debug
OPT := -Os
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/SAME51/same51n19a_flash_0x1000.ld

# Source directories (relative to project root)
SRC_DIRS := \
	src \
	src/CAN \
	src/Hardware/SAME51

# Include paths for C files (minimal set)
C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAME51 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAME51 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21 \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(WORKSPACE)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(WORKSPACE)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/RRFLibraries/src

# Libraries
LIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME5x_CAN \
	-L$(WORKSPACE)/RRFLibraries/SAME51 \
	-L$(WORKSPACE)/CANlib/SAME51_nonRTOS \
	-lCoreN2G -lRRFLibraries -lCANlib

# Library dependencies
LIB_DEPS := \
	$(WORKSPACE)/CoreN2G/SAME5x_CAN/libCoreN2G.a \
	$(WORKSPACE)/RRFLibraries/SAME51/libRRFLibraries.a \
	$(WORKSPACE)/CANlib/SAME51_nonRTOS/libCANlib.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mfp16-format=ieee \
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
	-Wl,--fatal-warnings -Wl,--no-warn-rwx-segment -mcpu=$(LINK_ARCH) -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
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
