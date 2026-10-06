# Makefile for the Duet3_MB6XD bootloader configuration
# CAN in-application programmer for the Duet 3 MB6XD main board

# Board name
BOARD := Duet3_MB6XD

# Output binary name
BINARY := Duet3_CANiap32_MB6XD

# MCU configuration
MCU := SAME70Q20B
MCU_ARCH := cortex-m7
LINK_ARCH := cortex-m4

# Compiler defines
# C files also get the noexcept define so shared headers compile as C
C_DEFINES := -D__SAME70Q20B__ -DMB6XD -DCAN_IAP -Dnoexcept=
CXX_DEFINES := -D__SAME70Q20B__ -DMB6XD -DCAN_IAP

# Optimization and debug
OPT := -Os
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/SAME70/same70_iap_ram.ld

# Source directories (relative to project root)
SRC_DIRS := \
	src \
	src/CAN \
	src/Hardware/SAME70

# Include paths for C files (minimal set)
C_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CANlib \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(WORKSPACE)/CoreN2G \
	-I$(WORKSPACE)/CANlib \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/SAME70 \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/common/utils \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/drivers \
	-I$(WORKSPACE)/CoreN2G/src/SAM4S_4E_E70/asf/sam/utils/cmsis/same70/include \
	-I$(WORKSPACE)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/RRFLibraries/src

# Libraries
LIBS := \
	-L$(WORKSPACE)/CoreN2G/SAME70_CAN \
	-L$(WORKSPACE)/RRFLibraries/SAME70 \
	-L$(WORKSPACE)/CANlib/SAME70_nonRTOS \
	-lCoreN2G -lRRFLibraries -lCANlib

# Library dependencies
LIB_DEPS := \
	$(WORKSPACE)/CoreN2G/SAME70_CAN/libCoreN2G.a \
	$(WORKSPACE)/RRFLibraries/SAME70/libRRFLibraries.a \
	$(WORKSPACE)/CANlib/SAME70_nonRTOS/libCANlib.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -mfp16-format=ieee -mno-unaligned-access \
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
