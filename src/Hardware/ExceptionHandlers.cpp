/*
 * ExceptionHandlers.cpp
 *
 *  Created on: 12 Sep 2020
 *      Author: David
 */

#include "ExceptionHandlers.h"
#include <Platform/Platform.h>
#include <Platform/Tasks.h>
#include <Hardware/NonVolatileMemory.h>
#include <Cache.h>

// Perform a software reset. 'stk' points to the exception stack (r0 r1 r2 r3 r12 lr pc xPSR) if the cause is an exception, otherwise it is nullptr.
void SoftwareReset(SoftwareResetReason initialReason, const uint32_t *_ecv_array null stk) noexcept
{
	uint16_t fullReason = (uint16_t)initialReason;
	IrqDisable();								// disable interrupts before we call any flash functions. We don't enable them again.
	WatchdogReset();							// kick the watchdog

	Cache::Disable();

	if (Platform::WasDeliberateError())
	{
		fullReason |= (uint16_t)SoftwareResetReason::deliberate;
	}

	// Record the reason for the software reset
	NonVolatileMemory * const mem = new(Tasks::GetNVMBuffer(stk)) NonVolatileMemory(NvmPage::common);
	SoftwareResetData * const srd = mem->AllocateResetDataSlot();
	srd->Populate(fullReason, stk);
	mem->EnsureWritten();

	Platform::ResetProcessor();
	for(;;) {}
}

[[noreturn]] void OutOfMemoryHandler() noexcept
{
	SoftwareReset(SoftwareResetReason::outOfMemory, GetStackPointer());
}

// Exception handlers

// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
extern "C" [[noreturn]] void hardFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset(SoftwareResetReason::hardFault, pulFaultStackAddress);
}

// The fault handler implementation calls a function called hardFaultDispatcher()
#if RP2040
extern "C" void isr_hardfault() noexcept __attribute__((naked));
void isr_hardfault() noexcept
#else
extern "C" void HardFault_Handler() noexcept __attribute__((naked));
void HardFault_Handler() noexcept
#endif
{
	__asm volatile
	(
#if SAMC21 || RP2040
		" mrs r0, msp												\n"
		" mov r1, lr												\n"
		" movs r2, #4												\n"
		" tst r1, r2												\n"
		" beq skip_hfh												\n"
		" mrs r0, psp												\n"
		"skip_hfh:													\n"
#else
		" tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
		" ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
		" mrseq r0, msp                                             \n"
		" mrsne r0, psp                                             \n"
#endif
		" ldr r2, handler_hf_address_const                          \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_hf_address_const: .word hardFaultDispatcher       \n"
	);
}

#if !RP2040

extern "C" [[noreturn]] void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress)
{
	SoftwareReset(SoftwareResetReason::wdtFault, pulFaultStackAddress);
}

extern "C" void WDT_Handler() noexcept __attribute__((naked));
void WDT_Handler() noexcept
{
	__asm volatile
	(
#if SAMC21
		" mrs r0, msp												\n"
		" mov r1, lr												\n"
		" movs r2, #4												\n"
		" tst r1, r2												\n"
		" beq skip_wdt												\n"
		" mrs r0, psp												\n"
		"skip_wdt:													\n"
#else
		" tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
		" ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
		" mrseq r0, msp                                             \n"
		" mrsne r0, psp                                             \n"
#endif
		" ldr r2, handler_wdt_address_const                         \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_wdt_address_const: .word wdtFaultDispatcher       \n"
	);
}

#endif

extern "C" [[noreturn]] void otherFaultDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset(SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
}

// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
// The fault handler implementation calls a function called otherFaultDispatcher()
extern "C" [[noreturn]] void OtherFault_Handler() noexcept __attribute__((naked));
void OtherFault_Handler() noexcept
{
	__asm volatile
	(
#if SAMC21 || RP2040
		" mrs r0, msp												\n"
		" mov r1, lr												\n"
		" movs r2, #4												\n"
		" tst r1, r2												\n"
		" beq skip_ofh												\n"
		" mrs r0, psp												\n"
		"skip_ofh:													\n"
#else
		" tst lr, #4                                                \n"		/* test bit 2 of the EXC_RETURN in LR to determine which stack was in use */
		" ite eq                                                    \n"		/* load the appropriate stack pointer into R0 */
		" mrseq r0, msp                                             \n"
		" mrsne r0, psp                                             \n"
#endif
		" ldr r2, handler_oflt_address_const                        \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_oflt_address_const: .word otherFaultDispatcher    \n"
	);
}

// We could set up the following fault handlers to retrieve the program counter in the same way as for a Hard Fault,
// however these exceptions are unlikely to occur, so for now we just report the exception type.
#if RP2040
extern "C" [[noreturn]] void isr_nmi() noexcept
#else
extern "C" [[noreturn]] void NMI_Handler() noexcept
#endif
{
	SoftwareReset(SoftwareResetReason::NMI);
}

extern "C" void UsageFault_Handler() noexcept
{
	SoftwareReset(SoftwareResetReason::usageFault);
}

extern "C" [[noreturn]] void DebugMon_Handler() noexcept __attribute__ ((alias("OtherFault_Handler")));

// FreeRTOS hooks that we need to provide
extern "C" [[noreturn]] void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName) noexcept
{
	SoftwareReset(SoftwareResetReason::stackOverflow, pulFaultStackAddress);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept __attribute((naked, noreturn));
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) noexcept
{
	// r0 = pxTask, r1 = pxTaskName
	__asm volatile
	(
		" push {r0, r1, lr}											\n"		/* save parameters and call address on the stack */
		" mov r0, sp												\n"
		" ldr r2, handler_sovf_address_const                        \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_sovf_address_const: .word stackOverflowDispatcher \n"
	);
}

extern "C" [[noreturn]] void assertCalledDispatcher(const uint32_t *pulFaultStackAddress) noexcept
{
	SoftwareReset(SoftwareResetReason::assertCalled, pulFaultStackAddress);
}

extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute((naked));
void vAssertCalled(uint32_t line, const char *file) noexcept
{
	__asm volatile
	(
		" push {r0, r1, lr}											\n"		/* save parameters and call address */
		" mov r0, sp												\n"
		" ldr r2, handler_asrt_address_const                        \n"
		" bx r2                                                     \n"
		" .align 2                                                  \n"		/* make the 2 LSBs zero at the next instruction */
		" handler_asrt_address_const: .word assertCalledDispatcher  \n"
	);
}

// End
