# Makefile for the STM32H5 bootloader configuration
# Bootloader for STM32H5 expansion and tool boards

# Board name
BOARD := STM32H5

# Output binary name
BINARY := Duet3Bootloader-STM32H5

# MCU configuration
MCU := STM32H5
MCU_ARCH := cortex-m33
LINK_ARCH := cortex-m33

# Compiler defines
# C files also get the noexcept define so shared headers compile as C
C_DEFINES := -DSTM32H523xx -Dnoexcept=
CXX_DEFINES := -DSTM32H523xx

# Optimization and debug
OPT := -Os
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/STM32H5/stm32h523xx_flash.ld

# Source directories (relative to project root)
SRC_DIRS := \
	src \
	src/CAN \
	src/Hardware/STM32H5

# Include paths for C files (minimal set)
C_INCLUDES := \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/STM32H5 \
	-I$(LIBRARIES_DIR)/CoreN2G/src \

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(CURDIR)/src \
	-I$(CURDIR)/src/Hardware/STM32H5 \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STMCubeMX/Core/Inc \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STMCubeMX/Drivers/STM32H5xx_HAL_Driver/Inc \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Device/ST/STM32H5xx/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STM32 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/STM32/STM32H5 \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/RRFLibraries/src

# Libraries
LIBS := \
	-L$(LIBRARIES_DIR)/CoreN2G/STM32H5_CAN \
	-L$(LIBRARIES_DIR)/RRFLibraries/STM32H5 \
	-L$(LIBRARIES_DIR)/CANlib/STM32H5_nonRTOS \
	-lCANlib -lCoreN2G -lRRFLibraries

# Library dependencies
LIB_DEPS := \
	$(LIBRARIES_DIR)/CoreN2G/STM32H5_CAN/libCoreN2G.a \
	$(LIBRARIES_DIR)/RRFLibraries/STM32H5/libRRFLibraries.a \
	$(LIBRARIES_DIR)/CANlib/STM32H5_nonRTOS/libCANlib.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mfp16-format=ieee \
	-fno-math-errno -ffunction-sections -fdata-sections -nostdlib -Wundef -Wdouble-promotion -Werror=return-type \
	-fsingle-precision-constant -Wall -g3

# C flags
CFLAGS := $(COMMON_FLAGS) $(OPT) $(C_DEFINES) $(C_INCLUDES) -std=gnu99 $(CFLAGS_EXTRA)

# Build directory
BUILD_DIR := $(BOARD)

# C++ flags
CXXFLAGS := $(COMMON_FLAGS) $(OPT) $(CXX_DEFINES) $(CXX_INCLUDES) -std=c++20 \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -Werror -Wnoexcept -Wshadow -Wsign-promo $(CXXFLAGS_EXTRA)

# Linker flags
LDFLAGS := $(LDOPT) --specs=nano.specs -Wl,--gc-sections -Wl,--entry=Reset_Handler \
	-Wl,--fatal-warnings -mcpu=$(LINK_ARCH) -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
	-T$(LINKER_SCRIPT) -Wl,-Map,$(CURDIR)/$(BUILD_DIR)/$(BINARY).map,--cref

# Find all source files
C_SRC := $(foreach dir,$(SRC_DIRS),$(sort $(wildcard $(dir)/*.c)))
CXX_SRC := $(foreach dir,$(SRC_DIRS),$(sort $(wildcard $(dir)/*.cpp)))

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
	$(Q)$(LD) $(OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group $(LDFLAGS) -o $@

# Generate binary and append CRC
$(BIN): $(ELF)
	$(Q)echo "  OBJCOPY $(notdir $@)"
	$(Q)$(OBJCOPY) -O binary $< $@
	$(Q)echo "  OBJCOPY $(notdir $(HEX))"
	$(Q)$(OBJCOPY) -O ihex $(ELF) $(HEX)
	$(Q)echo "  CRC     $(notdir $@)"
	$(Q)$(CRC_APPENDER) $@

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
