/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include <hpl_user_area.h>
#include "Tasks.h"
#include "Platform.h"
#include "SoftwareReset.h"
#include <Hardware/Devices.h>
#include <Cache.h>
#include <malloc.h>

#include "FreeRTOS.h"
#include "task.h"

#if SAME5x
# include <hri_wdt_e54.h>
#elif SAMC21
# include <hri_wdt_c21.h>
#endif

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);

constexpr unsigned int MainTaskStackWords = 800;

static Task<MainTaskStackWords> mainTask;
static Mutex mallocMutex;

extern "C" void MainTask(void * pvParameters);

// We need to make malloc/free thread safe, else sprintf and related I/O functions are liable to crash.
// We must use a recursive mutex for it.
extern "C" void __malloc_lock ( struct _reent *_r )
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void __malloc_unlock ( struct _reent *_r )
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

// Application entry point
void AppMain()
{
#ifndef DEBUG

	// Check that the bootloader is protected and EEPROM is configured
# if SAME5x
	uint64_t nvmUserRow0 = *reinterpret_cast<const uint64_t*>(NVMCTRL_USER);						// we only need values in the first 64 bits of the user area
	constexpr uint64_t mask =     ((uint64_t)0x0F << 32) | ((uint64_t)0x07 << 36) | (0x0F << 26);	// we just want NVM_BOOT (bits 26-29), SEE.SBLK (bits 32-35) and SEE.PSZ (bits 36:38)
	constexpr uint64_t reqValue = ((uint64_t)0x01 << 32) | ((uint64_t)0x03 << 36) | (0x07 << 26);	// 4K SMART EEPROM and 64K bootloader (SBLK=1 PSZ=3)
# elif SAMC21
	uint32_t nvmUserRow0 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER);						// we only need values in the first 32 bits of the user area
	constexpr uint32_t mask =     NVMCTRL_FUSES_EEPROM_SIZE_Msk | NVMCTRL_FUSES_BOOTPROT_Msk;		// we just want BOOTPROT (bits 0-2) and EEPROM (bits 4-6)
	constexpr uint32_t reqValue = (0x02 << NVMCTRL_FUSES_EEPROM_SIZE_Pos) | (0x01 << NVMCTRL_FUSES_BOOTPROT_Pos);	// 4K EEPROM and 16K bootloader
# endif

	if ((nvmUserRow0 & mask) != reqValue)
	{
		nvmUserRow0 = (nvmUserRow0 & ~mask) | reqValue;												// set up the required value
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), 0, reinterpret_cast<const uint8_t*>(&nvmUserRow0), sizeof(nvmUserRow0));

		// If we reset immediately then the user area write doesn't complete and the bits get set to all 1s.
		delayMicroseconds(10000);
		Platform::ResetProcessor();
	}
#endif

	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

	CoreInit();
	DeviceInit();

#if !SAMC21		// SAMC21 has a DIVAS unit, but that does not have an interrupt
	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
#endif

#if USE_CACHE
	Cache::Init();
	Cache::Enable();
#endif

	// Create the startup task
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);

	// Initialise watchdog clock
	WatchdogInit();
	NVIC_EnableIRQ(WDT_IRQn);		// enable the watchdog early warning interrupt

	vTaskStartScheduler();			// doesn't return
	while (true) { }
}

extern "C" void MainTask(void *pvParameters)
{
	mallocMutex.Create("Malloc");

	RepRap::Init();
	for (;;)
	{
		RepRap::Spin();
	}
}

extern "C" uint32_t _estack;		// this is defined in the linker script

namespace Tasks
{
	static void GetHandlerStackUsage(uint32_t* maxStack, uint32_t* neverUsed)
	{
		const char * const ramend = (const char *)&_estack;
		const char * const heapend = sbrk(0);
		const char * stack_lwm = heapend;
		while (stack_lwm < ramend && *stack_lwm == memPattern)
		{
			++stack_lwm;
		}
		if (maxStack != nullptr) { *maxStack = ramend - stack_lwm; }
		if (neverUsed != nullptr) { *neverUsed = stack_lwm - heapend; }
	}
}

uint32_t Tasks::GetNeverUsedRam()
{
	uint32_t maxStack, neverUsedRam;
	GetHandlerStackUsage(&maxStack, &neverUsedRam);
	return neverUsedRam;
}

void Tasks::Diagnostics(const StringRef& reply)
{
	// Append a memory report to a string
	uint32_t maxStack, neverUsedRam;
	GetHandlerStackUsage(&maxStack, &neverUsedRam);
	reply.lcatf("Never used RAM %.1fKb, max stack %" PRIu32 "b\n", (double)neverUsedRam/1024, maxStack);

	// Now the per-task memory report
	bool printed = false;
	for (const TaskBase *t = TaskBase::GetTaskList(); t != nullptr; t = t->GetNext())
	{
		TaskStatus_t taskDetails;
		vTaskGetInfo(t->GetHandle(), &taskDetails, pdTRUE, eInvalid);
		if (printed)
		{
			reply.cat(' ');
		}
		reply.catf("%s %u", taskDetails.pcTaskName, (unsigned int)(taskDetails.usStackHighWaterMark * sizeof(StackType_t)));
		printed = true;
	}

	// Show the up time and reason for the last reset
	const uint32_t now = (uint32_t)(millis64()/1000u);		// get up time in seconds
	reply.lcatf("Last reset %02d:%02d:%02d ago, cause: ", (unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60));

	const uint8_t resetCause = RSTC->RCAUSE.reg;
	switch (resetCause)
	{
	case RSTC_RCAUSE_POR:		reply.cat("power up"); break;
	case RSTC_RCAUSE_BODCORE:	reply.cat("core brownout"); break;
	case RSTC_RCAUSE_BODVDD:	reply.cat("VDD brownout"); break;
	case RSTC_RCAUSE_EXT:		reply.cat("reset button"); break;
	case RSTC_RCAUSE_WDT:		reply.cat("watchdog"); break;
	case RSTC_RCAUSE_SYST:		reply.cat("software"); break;
#if SAME5x
	case RSTC_RCAUSE_NVM:		reply.cat("nvm"); break;
	case RSTC_RCAUSE_BACKUP:	reply.cat("backup/hibernate"); break;
#endif
	default:					reply.catf("%u", resetCause); break;
	}
}

static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

extern "C" void vApplicationTickHook(void)
{
	CoreSysTick();
	WatchdogReset();							// kick the watchdog
	RepRap::Tick();
}

extern "C" void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer. */
    *pulIdleTaskStackSize = ARRAY_SIZE(uxIdleTaskStack);
}

static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

extern "C" void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer. */
    *pulTimerTaskStackSize = ARRAY_SIZE(uxTimerTaskStack);
}

// Helper function to cause a divide by zero error without the compiler noticing we are doing that
uint32_t Tasks::DoDivide(uint32_t a, uint32_t b)
{
	return a/b;
}

// Exception handlers
extern "C"
{
	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	void hardFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    Platform::SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
	void HardFault_Handler() __attribute__((naked));
	void HardFault_Handler()
	{
	    __asm volatile
	    (
#if SAMC21
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
	        " handler_hf_address_const: .word hardFaultDispatcher       \n"
	    );
	}

	void wdtFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
	}

	void WDT_Handler() __attribute__((naked));
	void WDT_Handler()
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
	        " handler_wdt_address_const: .word wdtFaultDispatcher       \n"
	    );
	}

	void otherFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
	}

	// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
	// The fault handler implementation calls a function called otherFaultDispatcher()
	void OtherFault_Handler() __attribute__((naked));
	void OtherFault_Handler()
	{
	    __asm volatile
	    (
#if SAMC21
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
	        " handler_oflt_address_const: .word otherFaultDispatcher    \n"
	    );
	}

	// We could set up the following fault handlers to retrieve the program counter in the same way as for a Hard Fault,
	// however these exceptions are unlikely to occur, so for now we just report the exception type.
	void NMI_Handler        ()
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::NMI);
	}

	void UsageFault_Handler ()
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::usageFault);
	}

	void DebugMon_Handler   () __attribute__ ((alias("OtherFault_Handler"), nothrow));

	// FreeRTOS hooks that we need to provide
	void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName)
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress);
	}

	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) __attribute((naked));
	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
	{
		// r0 = pxTask, r1 = pxTaskName
	    __asm volatile
	    (
	    	" push {r0, r1, lr}											\n"		/* save parameters and call address on the stack */
	    	" mov r0, sp												\n"
	        " ldr r2, handler_sovf_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_sovf_address_const: .word stackOverflowDispatcher \n"
	    );
	}

	void assertCalledDispatcher(const uint32_t *pulFaultStackAddress)
	{
		Platform::SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
	}

	void vAssertCalled(uint32_t line, const char *file) __attribute((naked));
	void vAssertCalled(uint32_t line, const char *file)
	{
	    __asm volatile
	    (
	    	" push {r0, r1, lr}											\n"		/* save parameters and call address */
	    	" mov r0, sp												\n"
	        " ldr r2, handler_asrt_address_const                        \n"
	        " bx r2                                                     \n"
	        " handler_asrt_address_const: .word assertCalledDispatcher  \n"
	    );
	}

}

// End
