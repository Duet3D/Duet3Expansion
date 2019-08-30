/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include <atmel_start.h>
#include "Tasks.h"
#include <malloc.h>

#include "FreeRTOS.h"
#include "task.h"

const uint8_t memPattern = 0xA5;

extern "C" char *sbrk(int i);
//extern char _end;


// The main task currently runs GCodes, so it needs to be large enough to hold the matrices used for auto calibration.
constexpr unsigned int MainTaskStackWords = 2040;

static Task<MainTaskStackWords> mainTask;
static Mutex spiMutex;
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
	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
#ifndef SAMC21		// SAMC21 has no divide unit, so there is no divide-by-zero exception
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
#endif

#if USE_CACHE
	// Enable the cache
	struct cmcc_config g_cmcc_cfg;
	cmcc_get_config_defaults(&g_cmcc_cfg);
	cmcc_init(CMCC, &g_cmcc_cfg);
	EnableCache();
#endif

	// Create the startup task
	mainTask.Create(MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);
	vTaskStartScheduler();			// doesn't return
}

void delay(uint32_t ms)
{
	vTaskDelay(ms);
}

extern void Init();
extern void Spin();

extern "C" void MainTask(void *pvParameters)
{
	mallocMutex.Create("Malloc");
	spiMutex.Create("SPI");

	Init();
	for (;;)
	{
		Spin();
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

#if 0
	// Write data about the current task (if RTOS) or the system
	void Diagnostics(MessageType mtype)
	{
		Platform& p = reprap.GetPlatform();
		p.Message(mtype, "=== RTOS ===\n");
		// Print memory stats
		{
			const char * const ramstart =
#if SAME70
				(char *) 0x20400000;
#elif SAM4E || SAM4S
				(char *) 0x20000000;
#elif SAM3XA
				(char *) 0x20070000;
#elif defined(_SAME51_)
				(char *) 0x20000000;
#else
# error Unsupported processor
#endif
			p.MessageF(mtype, "Static ram: %d\n", &_end - ramstart);

			const struct mallinfo mi = mallinfo();
			p.MessageF(mtype, "Dynamic ram: %d of which %d recycled\n", mi.uordblks, mi.fordblks);

			uint32_t maxStack, neverUsed;
			GetHandlerStackUsage(&maxStack, &neverUsed);
			p.MessageF(mtype, "Exception stack ram used: %" PRIu32 "\n", maxStack);
			p.MessageF(mtype, "Never used ram: %" PRIu32 "\n", neverUsed);
		}

		p.Message(mtype, "Tasks:");
		for (const TaskBase *t = TaskBase::GetTaskList(); t != nullptr; t = t->GetNext())
		{
			TaskStatus_t taskDetails;
			vTaskGetInfo(t->GetHandle(), &taskDetails, pdTRUE, eInvalid);
			const char* const stateText = (taskDetails.eCurrentState == eRunning) ? "running"
											: (taskDetails.eCurrentState == eReady) ? "ready"
												: (taskDetails.eCurrentState == eBlocked) ? "blocked"
													: (taskDetails.eCurrentState == eSuspended) ? "suspended"
														: "invalid";
			p.MessageF(mtype, " %s(%s,%u)",
				taskDetails.pcTaskName, stateText, (unsigned int)(taskDetails.usStackHighWaterMark * sizeof(StackType_t)));
		}
		p.Message(mtype, "\nOwned mutexes:");

		for (const Mutex *m = Mutex::GetMutexList(); m != nullptr; m = m->GetNext())
		{
			const TaskHandle holder = m->GetHolder();
			if (holder != nullptr)
			{
				p.MessageF(mtype, " %s(%s)", m->GetName(), pcTaskGetName(holder));
			}
		}
		p.MessageF(mtype, "\n");
	}
#endif

}

uint32_t Tasks::GetNeverUsedRam()
{
	uint32_t neverUsedRam;
	GetHandlerStackUsage(nullptr, &neverUsedRam);
	return neverUsedRam;
}

Mutex *Tasks::GetSpiMutex()
{
	return &spiMutex;
}

static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

static volatile uint64_t g_ms_ticks = 0;		// Count of 1ms time ticks. */

extern "C" void vApplicationTickHook(void)
{
	g_ms_ticks++;
	RepRap::Tick();
//	wdt_restart(WDT);							// kick the watchdog
}

uint32_t millis()
{
    return (uint32_t)g_ms_ticks;
}

uint64_t millis64()
{
	hal_atomic_t flags;
	atomic_enter_critical(&flags);
	const uint64_t ret = g_ms_ticks;			// take a copy with interrupts disabled to guard against rollover while we read it
	atomic_leave_critical(&flags);
	return ret;
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

// Exception handlers
extern "C"
{
	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	void hardFaultDispatcher(const uint32_t *pulFaultStackAddress)
	{
	    //reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
		while (1) {}
	}

	// The fault handler implementation calls a function called hardFaultDispatcher()
	void HardFault_Handler() __attribute__((naked));
	void HardFault_Handler()
	{
	    __asm volatile
	    (
#ifdef SAMC21
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
	    //reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::wdtFault, pulFaultStackAddress + 5);
		while (1) {}
	}

	void WDT_Handler() __attribute__((naked));
	void WDT_Handler()
	{
	    __asm volatile
	    (
#ifdef SAMC21
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
	    //reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::otherFault, pulFaultStackAddress + 5);
		while (1) {}
	}

	// 2017-05-25: A user is getting 'otherFault' reports, so now we do a stack dump for those too.
	// The fault handler implementation calls a function called otherFaultDispatcher()
	void OtherFault_Handler() __attribute__((naked));
	void OtherFault_Handler()
	{
	    __asm volatile
	    (
#ifdef SAMC21
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
		//reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::NMI);
		while (1) {}
	}

	void UsageFault_Handler ()
	{
		//reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::usageFault);
		while (1) {}
	}

	void DebugMon_Handler   () __attribute__ ((alias("OtherFault_Handler")));

	// FreeRTOS hooks that we need to provide
	void stackOverflowDispatcher(const uint32_t *pulFaultStackAddress, char* pcTaskName)
	{
	    //reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::stackOverflow, pulFaultStackAddress);
		while (1) {}
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
	    //reprap.GetPlatform().SoftwareReset((uint16_t)SoftwareResetReason::assertCalled, pulFaultStackAddress);
		while (1) {}
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
