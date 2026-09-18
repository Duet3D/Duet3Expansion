/*
 * SoftwareReset.cpp
 *
 *  Created on: 15 Nov 2019
 *      Author: David
 */

#include "SoftwareReset.h"
#include <Platform/Tasks.h>
#include <Platform/Platform.h>
#include <General/Portability.h>
#include <ctime>

extern uint32_t _estack;			// defined in the linker script

// The following must be kept in line with enum class SoftwareResetReason
const char *const SoftwareResetData::ReasonText[] =
{
	"User",
	"Erase",
	"NMI",
	"HardFault",
	"StuckInSpinLoop",
	"WatchdogTimeout",
	"UsageFault",
	"OtherFault",
	"StackOverflow",
	"AssertionFailed",
	"HeatTaskStuck",
	"MemoryProtectionFault",
	"TerminateCalled",
	"PureOrDeletedVirtualFunctionCalled",
	"OutOfMemory",
	"Unknown"
};

uint8_t SoftwareResetData::extraDebugInfo;			// extra info for debugging

// Return true if this struct can be written without erasing it first
bool SoftwareResetData::IsVacant() const noexcept
{
	const uint32_t *p = reinterpret_cast<const uint32_t*>(this);
	for (size_t i = 0; i < sizeof(*this)/sizeof(uint32_t); ++i)
	{
		if (*p != 0xFFFFFFFF)
		{
			return false;
		}
		++p;
	}
	return true;
}

void SoftwareResetData::Clear() noexcept
{
	memset(this, 0xFF, sizeof(*this));
}

// Populate this reset data from the parameters passed and the CPU state
void SoftwareResetData::Populate(uint16_t reason, const uint32_t *stk) noexcept
{
	magic = magicValue;
	resetReason = reason | ((extraDebugInfo & 0x07) << 5);
	when = (uint32_t)Platform::GetDateTime();
	neverUsedRam = Tasks::GetNeverUsedRam();
	icsr = SCB->ICSR;
#if SAMC21 || RP2040
	// ARM Cortex M0+ doesn't have these registers
	hfsr = cfsr = bfar = 0;
# else
	hfsr = SCB->HFSR;
	cfsr = SCB->CFSR;
# if USE_MPU
	if ((reason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::memFault)
	{
		bfar = SCB->MMFAR;				// on a memory fault we store the MMFAR instead of the BFAR
	}
	else
	{
		bfar = SCB->BFAR;
	}
# else
	bfar = SCB->BFAR;
# endif
#endif

	// Get the task name if we can. There may be no task executing, so we must allow for this.
	const TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
	taskName = (currentTask == nullptr) ? 0x656e6f6e : LoadLEU32(pcTaskGetName(currentTask));

	sp = reinterpret_cast<uint32_t>(stk);
	if (stk == nullptr)
	{
		stackOffset = 0;
		stackMarkerValid = 0;
		spare = 0;
	}
	else
	{
		const char *stackLimit = (currentTask == nullptr) ? reinterpret_cast<const char *>(sysStackLimit) : (const char*)currentTask + sizeof(TaskBase);
		stackOffset = ((const char*)stk - stackLimit) >> 2;
		stackMarkerValid = stackLimit[0] == 0xA5 && stackLimit[3] == 0xA5;
		spare = 0;
		for (uint32_t& stval : stack)
		{
#if __FPU_USED
			if (&stval - stack == 8 && ResetReasonHasExceptionFrame(reason))
			{
				stk += 18;				// skip the FP registers
			}
#endif
			stval = (stk < &_estack) ? *stk++ : 0xFFFFFFFF;
		}
	}
}

void SoftwareResetData::PrintPart1(unsigned int slot, const StringRef& reply) const noexcept
{
	reply.copy("Last software reset ");
	if (when != 0)
	{
		const time_t whenTime = (time_t)when;
		tm timeInfo;
		gmtime_r(&whenTime, &timeInfo);
		reply.catf("at %04u-%02u-%02u %02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
	}
	else
	{
		reply.cat("time unknown");
	}
	reply.cat(", reason: ");
	if (resetReason & (uint32_t)SoftwareResetReason::deliberate)
	{
		reply.cat("deliberate ");
	}
	reply.cat(ReasonText[(resetReason >> 5) & 0x0F]);

#if !SAMC21
	// If it's a forced hard fault or a memory access fault, provide some more information
	if ((resetReason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::hardFault && (hfsr & 1u << 30) != 0)
	{
		if (cfsr & (1u << 25)) { reply.cat(" zeroDiv"); }
		if (cfsr & (1u << 24)) { reply.cat(" unaligned"); }
		if (cfsr & (1u << 18)) { reply.cat(" invPC"); }
		if (cfsr & (1u << 17)) { reply.cat(" invState"); }
		if (cfsr & (1u << 16)) { reply.cat(" undefInstr"); }
		if (cfsr & (1u << 15)) { reply.cat(" bfarValid"); }
		if (cfsr & (1u << 12)) { reply.cat(" stkErr"); }
		if (cfsr & (1u << 11)) { reply.cat(" unstkErr"); }
		if (cfsr & (1u << 10)) { reply.cat(" imprec"); }
		if (cfsr & (1u << 9)) { reply.cat(" precise"); }
		if (cfsr & (1u << 8)) { reply.cat(" ibus"); }
	}
# if USE_MPU
	else if ((resetReason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::memFault)
	{
		if (cfsr & (1u << 7)) { reply.cat(" mmarValid"); }
		if (cfsr & (1u << 4)) { reply.cat(" mstkErr"); }
		if (cfsr & (1u << 3)) { reply.cat(" munstkErr"); }
		if (cfsr & (1u << 1)) { reply.cat(" daccViol"); }
		if (cfsr & (1u << 0)) { reply.cat(" iaccViol"); }
	}
# endif
#endif

	reply.catf(", %s spinning, available RAM %" PRIi32 ", slot %u",
			Module(resetReason & 0x1F).ToString(),
			neverUsedRam,
			slot);

	// Our format buffer is only 256 characters long, so the next 2 lines must be written separately
	// The task name may include nulls at the end, so print it as a string
	const uint32_t taskNameWords[2] = { taskName, 0u };
#if SAMC21
	reply.lcatf("Software reset code 0x%04x ICSR 0x%08" PRIx32 " SP 0x%08" PRIx32 " Task %s Freestk %u %s",
				resetReason, icsr, sp, (const char *)taskNameWords, (unsigned int)stackOffset, (sp == 0) ? "n/a" : (stackMarkerValid) ? "ok" : "bad marker");
#else
	reply.lcatf("Software reset code 0x%04x HFSR 0x%08" PRIx32 " CFSR 0x%08" PRIx32 " ICSR 0x%08" PRIx32 " BFAR 0x%08" PRIx32 " SP 0x%08" PRIx32 " Task %s Freestk %u %s",
				resetReason, hfsr, cfsr, icsr, bfar, sp, (const char *)taskNameWords, (unsigned int)stackOffset, (sp == 0) ? "n/a" : (stackMarkerValid) ? "ok" : "bad marker");
#endif
}

void SoftwareResetData::PrintPart2(const StringRef& reply) const noexcept
{
	if (sp != 0)
	{
		// We saved a stack dump, so print it
		reply.copy("Stack:");
		for (uint32_t stval : stack)
		{
			reply.catf(" %08" PRIx32, stval);
		}
	}
}

// End
