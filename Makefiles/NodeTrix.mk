# Makefile for NodeTrix (STM32H523)
# This board uses an STM32H523 MCU with Cortex-M33 + FPU

# Board name
BOARD := NodeTrix

# Output binary name
BINARY := Duet3Firmware_NodeTrix

# MCU configuration
MCU := STM32H523
MCU_ARCH := cortex-m33
FPU_FLAGS := -mfpu=fpv5-sp-d16 -mfloat-abi=hard

# Compiler defines
# C files only get noexcept define
C_DEFINES := -DSTM32H523xx -D__ARM_ARCH_8EM__=1 -Dnoexcept=

# C++ files get board-specific defines
CXX_DEFINES := -DSTM32H523xx -D__ARM_ARCH_8EM__=1 -DNODETRIX -DRTOS

# Optimization and debug
OPT := -O3
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/STM32/STM32H5/STM32H523xx_FLASH.ld

# Source directories (relative to project root)
SRC_DIRS := \
	src \
	src/CAN \
	src/ClosedLoop \
	src/ClosedLoop/Encoders \
	src/CommandProcessing \
	src/Fans \
	src/FilamentMonitors \
	src/GPIO \
	src/Hardware \
	src/Hardware/Drivers \
	src/Hardware/STM32 \
	src/Hardware/STM32/STM32H5 \
	src/Heating \
	src/Heating/Sensors \
	src/InputMonitors \
	src/LedStrips \
	src/Movement \
	src/Movement/Kinematics \
	src/Movement/StepperDrivers \
	src/Platform

# Include paths for C files (minimal set)
C_INCLUDES := \
	-I$(CURDIR)/src \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Include \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Device/ST/STM32H5xx/Include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(CURDIR)/src \
	-I$(WORKSPACE)/CoreN2G/src \
	-I$(WORKSPACE)/CoreN2G/src/STM32 \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Core/Inc \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Include \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Drivers/CMSIS/Device/ST/STM32H5xx/Include \
	-I$(WORKSPACE)/CoreN2G/src/STMCubeMX/Drivers/STM32H5xx_HAL_Driver/Inc \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/FreeRTOS/src/include \
	-I$(WORKSPACE)/FreeRTOS/src/portable/GCC/ARM_CM33_NTZ/non_secure

# Libraries
LIBS := \
	-L$(WORKSPACE)/CoreN2G/STM32H5_CAN_RTOS \
	-L$(WORKSPACE)/CANlib/STM32H5_RTOS \
	-L$(WORKSPACE)/RRFLibraries/STM32H5_RTOS \
	-L$(WORKSPACE)/FreeRTOS/STM32H5 \
	-lCoreN2G -lCANlib -lRRFLibraries -lFreeRTOS

# Library dependencies
LIB_DEPS := \
	$(WORKSPACE)/CoreN2G/STM32H5_CAN_RTOS/libCoreN2G.a \
	$(WORKSPACE)/CANlib/STM32H5_RTOS/libCANlib.a \
	$(WORKSPACE)/RRFLibraries/STM32H5_RTOS/libRRFLibraries.a \
	$(WORKSPACE)/FreeRTOS/STM32H5/libFreeRTOS.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb $(FPU_FLAGS) -fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib -Wundef -Wdouble-promotion -Werror=return-type \
	-fsingle-precision-constant -Wall -Werror

# C flags (matching Eclipse, no extra warnings)
CFLAGS := $(COMMON_FLAGS) $(OPT) $(C_DEFINES) $(C_INCLUDES) -std=gnu99 $(CFLAGS_EXTRA)

# Build directory
BUILD_DIR := $(BOARD)

# C++ flags (matching Eclipse, no extra warnings)
CXXFLAGS := $(COMMON_FLAGS) $(OPT) $(CXX_DEFINES) $(CXX_INCLUDES) -std=c++20 \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -Wfloat-conversion -Wshadow -Wsign-promo $(CXXFLAGS_EXTRA)

# Linker flags
LDFLAGS := $(LDOPT) --specs=nano.specs -Wl,--gc-sections -Wl,--entry=Reset_Handler \
	-Wl,--fatal-warnings -Wl,--no-warn-rwx-segment -mcpu=$(MCU_ARCH) $(FPU_FLAGS) \
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

# Pre-build step (touch Version.cpp like Eclipse does)
.PHONY: pre-build-$(BOARD)
pre-build-$(BOARD):
	$(Q)touch -c $(CURDIR)/src/Version.cpp

# Default target
.PHONY: $(BOARD)
$(BOARD): pre-build-$(BOARD) $(BIN)
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
