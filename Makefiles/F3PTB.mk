# Makefile for F3PTB (SAME51)
# This board uses SAME51 MCU with Cortex-M4F + FPU

# Board name
BOARD := F3PTB

# Output binary name
BINARY := Duet3Firmware_F3PTB

# MCU configuration
MCU := SAME51N19A
MCU_ARCH := cortex-m4
FPU_FLAGS := -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Compiler defines
# C files only get noexcept define
C_DEFINES := -D__SAME51N19A__ -D__ARM_ARCH_7EM__=1 -Dnoexcept=

# C++ files get board-specific defines
CXX_DEFINES := -D__SAME51N19A__ -D__ARM_ARCH_7EM__=1 -DF3PTB -DRTOS

# Optimization and debug
OPT := -O3
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/SAME5x_C21/SAME5x/same51n19a_flash_with_bootloader.ld

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
	src/Hardware/SAME5x_C21 \
	src/Hardware/SAME5x_C21/SAME5x \
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
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(CURDIR)/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(CURDIR)/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hal/utils/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAME5x/hri \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAME51_DFP/1.1.139/include \
	-I$(LIBRARIES_DIR)/RRFLibraries/src \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/include \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/portable/GCC/ARM_CM4F

# Libraries
LIBS := \
	-L$(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_RTOS \
	-L$(LIBRARIES_DIR)/CANlib/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS \
	-L$(LIBRARIES_DIR)/FreeRTOS/SAME51 \
	-lCoreN2G -lCANlib -lRRFLibraries -lFreeRTOS

# Library dependencies
LIB_DEPS := \
	$(LIBRARIES_DIR)/CoreN2G/SAME5x_CAN_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/CANlib/SAME51_RTOS/libCANlib.a \
	$(LIBRARIES_DIR)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a \
	$(LIBRARIES_DIR)/FreeRTOS/SAME51/libFreeRTOS.a

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb $(FPU_FLAGS) -fno-math-errno -mfp16-format=ieee \
	-ffunction-sections -fdata-sections -nostdlib -Wundef -Wdouble-promotion -Werror=return-type \
	-fsingle-precision-constant -Wall -Werror

# C flags (matching Eclipse, no extra warnings)
CFLAGS := $(COMMON_FLAGS) $(OPT) $(C_DEFINES) $(C_INCLUDES) -std=gnu99 $(CFLAGS_EXTRA)

# Build directory
BUILD_DIR := $(BOARD)

# C++ flags (matching Eclipse, no extra warnings)
CXXFLAGS := $(COMMON_FLAGS) $(OPT) $(CXX_DEFINES) $(CXX_INCLUDES) -std=gnu++17 \
	-fno-threadsafe-statics -fno-rtti -fno-exceptions -Wfloat-conversion $(CXXFLAGS_EXTRA)

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

# Bind board-specific values to their targets so this file can coexist with
# other board makefiles in the same make invocation.
$(BOARD): BOARD := $(BOARD)
$(BOARD): ELF := $(ELF)
$(BOARD): BIN := $(BIN)

$(ELF): OBJS := $(OBJS)
$(ELF): LIBS := $(LIBS)
$(ELF): LDFLAGS := $(LDFLAGS)

$(BUILD_DIR)/%.o: BUILD_DIR := $(BUILD_DIR)
$(BUILD_DIR)/%.o: CFLAGS := $(CFLAGS)
$(BUILD_DIR)/%.o: CXXFLAGS := $(CXXFLAGS)

clean-$(BOARD): BUILD_DIR := $(BUILD_DIR)

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
ifneq ($(filter $(BOARD) all,$(MAKECMDGOALS)),)
-include $(DEPS)
endif
