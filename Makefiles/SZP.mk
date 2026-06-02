# Makefile for SZP (SAMC21)
# This board uses SAMC21G18A MCU with Cortex-M0+ (no FPU)

# Board name
BOARD := SZP

# Output binary name
BINARY := Duet3Firmware_SZP

# MCU configuration
MCU := SAMC21G18A
MCU_ARCH := cortex-m0plus
FPU_FLAGS :=

# Compiler defines
# C files only get noexcept define
C_DEFINES := -D__SAMC21G18A__ -Dnoexcept=

# C++ files get board-specific defines
CXX_DEFINES := -D__SAMC21G18A__ -DSZP -DRTOS

# Optimization and debug
OPT := -O3
LDOPT := -Os
DEBUG_FLAGS ?=
CFLAGS_EXTRA := $(DEBUG_FLAGS)
CXXFLAGS_EXTRA := $(DEBUG_FLAGS)

# Linker script
LINKER_SCRIPT := $(CURDIR)/src/Hardware/SAME5x_C21/SAMC21/samc21g18a_flash_with_bootloader.ld

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
	src/Hardware/SAME5x_C21/SAMC21 \
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
	-I$(LIBRARIES_DIR)/Qfplib-M0-full \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(CURDIR)/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAMC21_DFP/1.2.176/samc21/include

# Include paths for C++ files (full set)
CXX_INCLUDES := \
	-I$(LIBRARIES_DIR)/Qfplib-M0-full \
	-I$(LIBRARIES_DIR)/CoreN2G \
	-I$(LIBRARIES_DIR)/FreeRTOS \
	-I$(CURDIR)/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21 \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAMC21/hal/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAMC21/hal/utils/include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/SAME5x_C21/SAMC21/hri \
	-I$(LIBRARIES_DIR)/CoreN2G/src/arm/CMSIS/5.4.0/CMSIS/Core/Include \
	-I$(LIBRARIES_DIR)/CoreN2G/src/atmel/SAMC21_DFP/1.2.176/samc21/include \
	-I$(LIBRARIES_DIR)/RRFLibraries/src \
	-I$(LIBRARIES_DIR)/CANlib/src \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/include \
	-I$(LIBRARIES_DIR)/FreeRTOS/src/portable/GCC/ARM_CM0

# Libraries (Qfplib-M0-full for software floating point)
LIBS := \
	-L$(LIBRARIES_DIR)/Qfplib-M0-full/SAMC21 \
	-L$(LIBRARIES_DIR)/CoreN2G/SAMC21_CAN_RTOS \
	-L$(LIBRARIES_DIR)/CANlib/SAMC21_RTOS \
	-L$(LIBRARIES_DIR)/RRFLibraries/SAMC21_RTOS \
	-L$(LIBRARIES_DIR)/FreeRTOS/SAMC21 \
	-lQfplib-M0-full -lCoreN2G -lCANlib -lRRFLibraries -lFreeRTOS

# Library dependencies
LIB_DEPS := \
	$(LIBRARIES_DIR)/Qfplib-M0-full/SAMC21/libQfplib-M0-full.a \
	$(LIBRARIES_DIR)/CoreN2G/SAMC21_CAN_RTOS/libCoreN2G.a \
	$(LIBRARIES_DIR)/CANlib/SAMC21_RTOS/libCANlib.a \
	$(LIBRARIES_DIR)/RRFLibraries/SAMC21_RTOS/libRRFLibraries.a \
	$(LIBRARIES_DIR)/FreeRTOS/SAMC21/libFreeRTOS.a

# Qfplib function wrapping for software floating point on M0+
FP_WRAP_FLAGS := \
	-Wl,-wrap,__aeabi_fadd \
	-Wl,-wrap,__aeabi_fsub \
	-Wl,-wrap,__aeabi_fmul \
	-Wl,-wrap,__aeabi_fdiv \
	-Wl,-wrap,__aeabi_i2f \
	-Wl,-wrap,__aeabi_ui2f \
	-Wl,-wrap,__aeabi_f2iz \
	-Wl,-wrap,__aeabi_f2uiz

# Common flags
COMMON_FLAGS := -c -mcpu=$(MCU_ARCH) -mthumb -fno-math-errno -mfp16-format=ieee \
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
	-Wl,--fatal-warnings -Wl,--no-warn-rwx-segment -mcpu=$(MCU_ARCH) \
	-T$(LINKER_SCRIPT) -Wl,-Map,$(CURDIR)/$(BUILD_DIR)/$(BINARY).map,--cref $(FP_WRAP_FLAGS)

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
	$(Q)if ! command -v CrcAppender > /dev/null 2>&1; then \
		echo "  ERROR   CrcAppender not found"; \
		exit 1; \
	fi
	$(Q)CrcAppender $@

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
