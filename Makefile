# Duet3Expansion Master Makefile
# Builds firmware for various Duet 3 expansion boards

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
CONFIGS := EXP3HC EXP1XD EXP1HCL TOOL1LC SAMMYC21 SZP M23CL F3PTB TOOL1RR TOOLINDX

# Declare all board targets as phony
.PHONY: $(CONFIGS)

# Print available targets
.PHONY: help
help:
	$(Q)echo ""
	$(Q)echo "Duet3Expansion Build System"
	$(Q)echo "============================"
	$(Q)echo ""
	$(Q)echo "Build targets:"
	$(Q)echo "  EXP1HCL             - Duet 3 Expansion 1HCL (SAME51)"
	$(Q)echo "  EXP1XD              - Duet 3 Expansion 1XD (SAMC21)"
	$(Q)echo "  EXP3HC              - Duet 3 Expansion 3HC (SAME51)"
	$(Q)echo "  F3PTB               - F3PTB (SAME51)"
	$(Q)echo "  M23CL               - M23CL (SAME51)"
	$(Q)echo "  SAMMYC21            - Sammy C21 (SAMC21)"
	$(Q)echo "  SZP                 - SZP (SAMC21)"
	$(Q)echo "  TOOL1LC             - Duet 3 Tool 1LC (SAMC21)"
	$(Q)echo "  TOOL1RR             - Tool 1RR (SAME51)"
	$(Q)echo "  TOOLINDX            - Tool Index (SAME51)"
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
	$(Q)echo "  make EXP3HC                                # Build EXP3HC firmware"
	$(Q)echo "  make EXP1XD V=1                            # Build with verbose output"
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
$(WORKSPACE)/CoreN2G/SAME5x_CAN_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAME5x_CAN_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAME5x_CAN_RTOS

$(WORKSPACE)/RRFLibraries/SAME51_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAME51_RTOS

$(WORKSPACE)/FreeRTOS/SAME51/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAME51"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAME51 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAME51_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAME51_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAME51_RTOS

$(WORKSPACE)/CoreN2G/SAMC21_CAN_RTOS/libCoreN2G.a:
	$(Q)echo "  BUILD   CoreN2G/SAMC21_CAN_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G SAMC21_CAN_RTOS

$(WORKSPACE)/RRFLibraries/SAMC21_RTOS/libRRFLibraries.a:
	$(Q)echo "  BUILD   RRFLibraries/SAMC21_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries SAMC21_RTOS

$(WORKSPACE)/FreeRTOS/SAMC21/libFreeRTOS.a:
	$(Q)echo "  BUILD   FreeRTOS/SAMC21"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS SAMC21 FREERTOS_CONFIG_DIR="$(CURDIR)/src"

$(WORKSPACE)/CANlib/SAMC21_RTOS/libCANlib.a:
	$(Q)echo "  BUILD   CANlib/SAMC21_RTOS"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib SAMC21_RTOS

$(WORKSPACE)/Qfplib-M0-full/SAMC21/libQfplib-M0-full.a:
	$(Q)echo "  BUILD   Qfplib-M0-full/SAMC21"
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/Qfplib-M0-full SAMC21

# Include the specific makefile based on the target
# Only include one at a time to avoid conflicts
ifneq ($(MAKECMDGOALS),)
ifneq ($(MAKECMDGOALS),all)
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),help)
ifneq ($(MAKECMDGOALS),test-toolchain)
ifeq ($(findstring clean-,$(MAKECMDGOALS)),)
# Include only the makefile for the requested target
ifeq ($(MAKECMDGOALS),EXP1HCL)
-include Makefiles/EXP1HCL.mk
endif
ifeq ($(MAKECMDGOALS),EXP1XD)
-include Makefiles/EXP1XD.mk
endif
ifeq ($(MAKECMDGOALS),EXP3HC)
-include Makefiles/EXP3HC.mk
endif
ifeq ($(MAKECMDGOALS),F3PTB)
-include Makefiles/F3PTB.mk
endif
ifeq ($(MAKECMDGOALS),M23CL)
-include Makefiles/M23CL.mk
endif
ifeq ($(MAKECMDGOALS),SAMMYC21)
-include Makefiles/SAMMYC21.mk
endif
ifeq ($(MAKECMDGOALS),SZP)
-include Makefiles/SZP.mk
endif
ifeq ($(MAKECMDGOALS),TOOL1LC)
-include Makefiles/TOOL1LC.mk
endif
ifeq ($(MAKECMDGOALS),TOOL1RR)
-include Makefiles/TOOL1RR.mk
endif
ifeq ($(MAKECMDGOALS),TOOLINDX)
-include Makefiles/TOOLINDX.mk
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
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/FreeRTOS clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CoreN2G clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/RRFLibraries clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/CANlib clean
	$(Q)$(MAKE) $(VERBOSE) -C $(WORKSPACE)/Qfplib-M0-full clean
	$(Q)echo "Clean all complete"
