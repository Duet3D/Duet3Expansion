/*
 * Startup.cpp
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#include "Tasks.h"
#include <Platform.h>
#include <TaskPriorities.h>
#include <Movement/Move.h>
#include <Heating/Heat.h>
#include <InputMonitors/InputMonitor.h>
#include <CommandProcessing/CommandProcessor.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include <Hardware/Devices.h>
#include <Hardware/NonVolatileMemory.h>
#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include <Duet3Common.h>
#include <CAN/CanInterface.h>
#include <Cache.h>
#include <Flash.h>

#include <malloc.h>

#include <FreeRTOS.h>
#include <task.h>
#include <freertos_task_additions.h>

#include <hpl_user_area.h>

#if SAME5x
# include <hri_wdt_e54.h>
constexpr uint32_t FlashBlockSize = 0x00010000;					// the erase size we assume for flash, and the bootloader size (64K)
#elif SAMC21
# include <hri_wdt_c21.h>
constexpr uint32_t FlashBlockSize = 0x00004000;					// the erase size we assume for flash, and the bootloader size (16K)
#endif

// Define replacement standard library functions

#if SAMC21
// Reduce the size of the system stack below the default 1024 to save memory. When we set it to 512, M122 reported just 12 words unused, so try a higher value.
# define SystemStackSize	(600)
#endif

#include <syscalls.h>

constexpr uint32_t BlockReceiveTimeout = 2000;					// bootloader block receive timeout milliseconds

constexpr uint8_t memPattern = 0xA5;

constexpr unsigned int MainTaskStackWords = 850;				// this seems very large; but a user had a stack overflow when it was set to 800

static Task<MainTaskStackWords> mainTask;
static Mutex mallocMutex;
static unsigned int heatTaskIdleTicks = 0;

// Idle task data
constexpr unsigned int IdleTaskStackWords = 50;					// currently we don't use the idle talk for anything, so this can be quite small
static Task<IdleTaskStackWords> idleTask;

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) noexcept
{
	*ppxIdleTaskTCBBuffer = idleTask.GetTaskMemory();
	*ppxIdleTaskStackBuffer = idleTask.GetStackBase();
	*pulIdleTaskStackSize = idleTask.GetStackSize();
}

#if configUSE_TIMERS

// Timer task data
constexpr unsigned int TimerTaskStackWords = 60;
static Task<TimerTaskStackWords> timerTask;

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) noexcept
{
    *ppxTimerTaskTCBBuffer = timerTask.GetTaskMemory();
    *ppxTimerTaskStackBuffer = timerTask.GetStackBase();
    *pulTimerTaskStackSize = timerTask.GetStackSize();
}

#endif

extern "C" [[noreturn]] void MainTask(void * pvParameters) noexcept;
extern "C" [[noreturn]] void UpdateBootloaderTask(void * pvParameters) noexcept;

// Make malloc/free thread safe. We must use a recursive mutex for it.
extern "C" void __malloc_lock (struct _reent *_r) noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't take mutex if scheduler not started or suspended
	{
		mallocMutex.Take();
	}
}

extern "C" void __malloc_unlock (struct _reent *_r) noexcept
{
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)		// don't release mutex if scheduler not started or suspended
	{
		mallocMutex.Release();
	}
}

// Get a 4-byte aligned NonVolatileMemory buffer suitable for the crash handler to use for reading/writing flash memory.
// We don't want to use a static buffer because that is wasteful of RAM, given that only the crash handler uses it, we have interrupts disabled while we use it,
// and we reset immediately afterwards.
// Instead we use either the bottom or top of the main task stack.
// Parameter 'stk' is the stack we are interested in, which we must not overwrite. The caller is either using the same stack a little lower, or the exception stack.
void *Tasks::GetNVMBuffer(const uint32_t *stk) noexcept
{
	constexpr size_t stackAllowance = 128;
	static_assert((sizeof(NonVolatileMemory) & 3) == 0);
	static_assert(MainTaskStackWords * 4 >= 2 * sizeof(NonVolatileMemory) + stackAllowance + 4);
	const char * const cStack = reinterpret_cast<const char*>(stk);

	// See if we can use the bottom of the main task stack
	char *ret = (char *)&mainTask + sizeof(TaskBase);
	if (cStack > ret + (sizeof(NonVolatileMemory) + stackAllowance + 4))	// allow space for the buffer + 128b in case we are on that stack
	{
		ret += 4;															// the +4 is so that we leave the stack marker alone in case the main task raised the exception
	}
	else
	{
		ret += (MainTaskStackWords * 4) - sizeof(NonVolatileMemory);		// use the top area instead
	}
	return ret;
}

// Application entry point
[[noreturn]] void AppMain() noexcept
{
	bool updateBootloader = false;

#ifndef DEBUG

	// Check that the bootloader is protected and EEPROM is configured
	union
	{
		uint64_t b64[5];
		uint32_t b32[10];
	} nvmUserRow;

	memcpy(&nvmUserRow, reinterpret_cast<const void*>(NVMCTRL_USER), sizeof(nvmUserRow));

# if SAME5x
	uint64_t& nvmUserRow0 = nvmUserRow.b64[0];
	constexpr uint64_t mask =     ((uint64_t)0x0F << 32) | ((uint64_t)0x07 << 36) | (0x0F << 26);	// we just want NVM_BOOT (bits 26-29), SEE.SBLK (bits 32-35) and SEE.PSZ (bits 36:38)
	constexpr uint64_t reqValue = ((uint64_t)0x01 << 32) | ((uint64_t)0x03 << 36) | (0x07 << 26);	// 4K SMART EEPROM and 64K bootloader (SBLK=1 PSZ=3 NVM_BOOT=0x07)
	constexpr uint64_t bootprotMask = (0x0F << 26);													// mask for bootloader protection only
	constexpr uint64_t reqValueNoBootprot = (0x0F << 26);											// value for no bootloader protection
# elif SAMC21
	uint32_t& nvmUserRow0 = nvmUserRow.b32[0];
	constexpr uint32_t mask =     NVMCTRL_FUSES_EEPROM_SIZE_Msk | NVMCTRL_FUSES_BOOTPROT_Msk;		// we just want BOOTPROT (bits 0-2) and EEPROM (bits 4-6)
	constexpr uint32_t reqValue = (0x02 << NVMCTRL_FUSES_EEPROM_SIZE_Pos) | (0x01 << NVMCTRL_FUSES_BOOTPROT_Pos);	// 4K EEPROM and 16K bootloader
	constexpr uint32_t bootprotMask = NVMCTRL_FUSES_BOOTPROT_Msk;									// mask for bootloader protection only
	constexpr uint32_t reqValueNoBootprot = (0x07 << NVMCTRL_FUSES_BOOTPROT_Pos);					// value for no bootloader protection
# endif

	if (nvmUserRow.b32[UpdateBootloaderMagicWordIndex] == UpdateBootloaderMagicValue && (nvmUserRow0 & bootprotMask) == reqValueNoBootprot)
	{
		// Update the bootloader
		nvmUserRow.b32[UpdateBootloaderMagicWordIndex] = 0xFFFFFFFF;								// clear the bootloader update flag
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), 0, reinterpret_cast<const uint8_t*>(&nvmUserRow), sizeof(nvmUserRow));
		delayMicroseconds(10000);																	// in case we reset early due to low voltage etc.
		updateBootloader = true;																	// we can't update it until we have started RTOS
	}
	else if ((nvmUserRow0 & mask) != reqValue)
	{
		nvmUserRow0 = (nvmUserRow0 & ~mask) | reqValue;												// set up the required value
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), 0, reinterpret_cast<const uint8_t*>(&nvmUserRow), sizeof(nvmUserRow));

		// If we reset immediately then the user area write doesn't complete and the bits get set to all 1s.
		delayMicroseconds(10000);
		Platform::ResetProcessor();
	}
#endif

	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* p = heapTop;
	const char * stack_ptr = (const char *_ecv_array)GetStackPointer();
	while (p + 16 < stack_ptr)
	{
		*p++ = memPattern;
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

#if 0	// If we get an imprecise hard fault reported immediately after startup, temporarily enable this to make all hard faults precise, with some loss of performance
	SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;		// disable write buffer
#endif

	idleTask.AddToList();			// add the FreeRTOS internal tasks to the task list

#if configUSE_TIMERS
	timerTask.AddToList();
#endif

	// Create the startup task and memory allocation mutex
	mainTask.Create((updateBootloader) ? UpdateBootloaderTask : MainTask, "MAIN", nullptr, TaskPriority::SpinPriority);
	mallocMutex.Create("Malloc");

	// Initialise watchdog clock
	WatchdogInit();
	NVIC_EnableIRQ(WDT_IRQn);		// enable the watchdog early warning interrupt

	StepTimer::Init();				// initialise the step pulse timer now because we use it for measuring task CPU usage
	vTaskStartScheduler();			// doesn't return
	while (true) { }
}

// The main task loop that runs during normal operation
extern "C" [[noreturn]] void MainTask(void *pvParameters) noexcept
{
	Platform::Init();
	Heat::Init();
	InputMonitor::Init();

#if SUPPORT_DRIVERS
	moveInstance = new Move();
	moveInstance->Init();
#endif

	for (;;)
	{
		Platform::Spin();
		CommandProcessor::Spin();
#if SUPPORT_DRIVERS
		FilamentMonitor::Spin();
#endif
	}
}

// Request a block of the bootloader, returning true if successful
static FirmwareFlashErrorCode RequestBootloaderBlock(uint32_t fileOffset, uint32_t numBytes, CanMessageBuffer& buf)
{
	CanMessageFirmwareUpdateRequest * const msg = buf.SetupRequestMessage<CanMessageFirmwareUpdateRequest>(0, CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
	SafeStrncpy(msg->boardType, BOOTLOADER_NAME, sizeof(msg->boardType));
	msg->boardVersion = 0;
	msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
	msg->fileWanted = 3;
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
	Platform::OnProcessingCanMessage();								// turn the green LED on
	return FirmwareFlashErrorCode::ok;
}

// Get a buffer of data from the host, returning true if successful
static FirmwareFlashErrorCode GetBootloaderBlock(uint8_t *blockBuffer)
{
	CanMessageBuffer buf(nullptr);
	const FirmwareFlashErrorCode err = RequestBootloaderBlock(0, FlashBlockSize, buf);	// ask for 16K or 64K as a single block
	if (err != FirmwareFlashErrorCode::ok)
	{
		return err;
	}

	uint32_t whenStartedWaiting = millis();
	uint32_t bytesReceived = 0;
	bool done = false;
	do
	{
		Platform::SpinMinimal();									// check if it's time to turn the LED off
		const bool ok = CanInterface::GetCanMessage(&buf);
		if (ok)
		{
			if (buf.id.MsgType() == CanMessageType::firmwareBlockResponse)
			{
				const CanMessageFirmwareUpdateResponse& response = buf.msg.firmwareUpdateResponse;
				switch (response.err)
				{
				case CanMessageFirmwareUpdateResponse::ErrNoFile:
					return FirmwareFlashErrorCode::noFile;

				case CanMessageFirmwareUpdateResponse::ErrBadOffset:
					return FirmwareFlashErrorCode::badOffset;

				case CanMessageFirmwareUpdateResponse::ErrOther:
					return FirmwareFlashErrorCode::hostOther;

				case CanMessageFirmwareUpdateResponse::ErrNone:
					if (response.fileOffset <= bytesReceived)
					{
						const uint32_t bufferOffset = response.fileOffset;
						const uint32_t bytesToCopy = min<uint32_t>(FlashBlockSize - bufferOffset, response.dataLength);
						memcpy(blockBuffer + bufferOffset, response.data, bytesToCopy);
						if (response.fileOffset + bytesToCopy > bytesReceived)
						{
							bytesReceived = response.fileOffset + bytesToCopy;
						}
						if (bytesReceived == FlashBlockSize || bytesReceived >= response.fileLength)
						{
							// Reached the end of the file
							memset(blockBuffer + bytesReceived, 0xFF, FlashBlockSize - bytesReceived);
							done = true;
						}
					}
					whenStartedWaiting = millis();
				}
			}
		}
		else if (millis() - whenStartedWaiting > BlockReceiveTimeout)
		{
			if (bytesReceived == 0)
			{
				return FirmwareFlashErrorCode::blockReceiveTimeout;
			}
			RequestBootloaderBlock(bytesReceived, FlashBlockSize - bytesReceived, buf);			// ask for a block from the starting offset
			whenStartedWaiting = millis();
		}
	} while (!done);

	return FirmwareFlashErrorCode::ok;
}

static void ReportFlashError(FirmwareFlashErrorCode err)
{
	for (unsigned int i = 0; i < (unsigned int)err; ++i)
	{
		Platform::WriteLed(0, true);
		delay(200);
		Platform::WriteLed(0, false);
		delay(200);
	}

	delay(1000);
}

// Compute the CRC32 of a dword-aligned block of memory
// This assumes the caller has exclusive use of the DMAC
uint32_t ComputeCRC32(const uint32_t *start, const uint32_t *end)
{
#if SAME5x
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
#elif SAMC21
	DMAC->CTRL.bit.CRCENABLE = 0;
#else
# error Unsupported processor
#endif
	DMAC->CRCCHKSUM.reg = 0xFFFFFFFF;
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
#if SAMC21
	DMAC->CTRL.bit.CRCENABLE = 1;
#endif
	while (start < end)
	{
		DMAC->CRCDATAIN.reg = *start++;
		asm volatile("nop");
		asm volatile("nop");
	}

	DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
	asm volatile("nop");
	return DMAC->CRCCHKSUM.reg;
}

// Check that the bootloader we have been passed has a valid CRC
bool CheckCRC(uint32_t *blockBuffer) noexcept
{
	const uint32_t crcOffset = blockBuffer[7];						// vector M9 gives the offset of the CRC from start of file, in bytes
	if (crcOffset < 0x2000 || crcOffset > FlashBlockSize - 4)		// if the file is shorter than 8K or longer than the buffer, error
	{
		return false;
	}
	const uint32_t expectedCRC = blockBuffer[crcOffset/4];
	return ComputeCRC32(blockBuffer, blockBuffer + crcOffset/4) == expectedCRC;
}

// The task that runs to update the bootloader
extern "C" [[noreturn]] void UpdateBootloaderTask(void *pvParameters) noexcept
{
	// Allocate a buffer large enough to contain the entire bootloader
	uint32_t * blockBuffer = nullptr;
	Platform::InitMinimal();
	for (;;)
	{
		if (blockBuffer == nullptr)
		{
			blockBuffer = new uint32_t[FlashBlockSize/4];			// if this fails then an OutOfMemory reset will occur
		}

		const FirmwareFlashErrorCode err = GetBootloaderBlock(reinterpret_cast<uint8_t*>(blockBuffer));
		const uint32_t start = millis();
		do
		{
			Platform::SpinMinimal();								// make sure the currentVin is up to date and the green LED gets turned off
		} while (millis() - start < 300);

		if (err != FirmwareFlashErrorCode::ok)
		{
			ReportFlashError(err);
		}
		else if (!CheckCRC(blockBuffer))
		{
			ReportFlashError(FirmwareFlashErrorCode::badCRC);
		}
#if HAS_VOLTAGE_MONITOR
		else if (Platform::GetCurrentVinVoltage() < 10.5)
		{
			ReportFlashError(FirmwareFlashErrorCode::vinTooLow);
		}
#endif
		else if (!Flash::Init())
		{
			ReportFlashError(FirmwareFlashErrorCode::flashInitFailed);
		}
		else if (!Flash::Unlock(FLASH_ADDR, FlashBlockSize))
		{
			ReportFlashError(FirmwareFlashErrorCode::unlockFailed);
		}
		else if (!Flash::Erase(FLASH_ADDR, FlashBlockSize))
		{
			ReportFlashError(FirmwareFlashErrorCode::eraseFailed);
		}
		else if (!Flash::Write(FLASH_ADDR, FlashBlockSize, blockBuffer))
		{
			ReportFlashError(FirmwareFlashErrorCode::writeFailed);
		}
		else if (!Flash::Lock(FLASH_ADDR, FlashBlockSize))
		{
			ReportFlashError(FirmwareFlashErrorCode::lockFailed);
		}
		else
		{
			break;		// success!
		}
	}

	CanInterface::Shutdown();

	// If we reset immediately then the user area write doesn't complete and the bits get set to all 1s.
	delayMicroseconds(10000);
	Platform::ResetProcessor();
}

// Return the amount of free handler stack space
static ptrdiff_t GetHandlerFreeStack() noexcept
{
	const char * const ramend = (const char*)&_estack;
	const char * stack_lwm = sysStackLimit;
	while (stack_lwm < ramend && *stack_lwm == memPattern)
	{
		++stack_lwm;
	}
	return stack_lwm - sysStackLimit;
}

ptrdiff_t Tasks::GetNeverUsedRam() noexcept
{
	return heapLimit - heapTop;
}

// Function called by FreeRTOS and internally to reset the run-time counter and return the number of timer ticks since it was last reset
extern "C" uint32_t TaskResetRunTimeCounter() noexcept
{
	static uint32_t whenLastReset = 0;
	const uint32_t now = StepTimer::GetTimerTicks();
	const uint32_t ret = now - whenLastReset;
	whenLastReset = now;
	return ret;
}

void Tasks::Diagnostics(const StringRef& reply) noexcept
{
	// Append a memory report to a string
	reply.lcatf("Never used RAM %d, free system stack %d words\nTasks:", GetNeverUsedRam(), GetHandlerFreeStack()/4);

	// Now the per-task memory report
	const uint32_t timeSinceLastCall = TaskResetRunTimeCounter();
	float totalCpuPercent = 0.0;
	for (TaskBase *t = TaskBase::GetTaskList(); t != nullptr; t = t->GetNext())
	{
		ExtendedTaskStatus_t taskDetails;
		vTaskGetExtendedInfo(t->GetFreeRTOSHandle(), &taskDetails);

		const char* stateText;
		switch (taskDetails.eCurrentState)
		{
		case esRunning:
			stateText = "running";
			break;
		case esReady:
			stateText = "ready";
			break;
		case esNotifyWaiting:
			stateText = "notifyWait";
			break;
		case esResourceWaiting:
			stateText = "resourceWait:";
			break;
		case esDelaying:
			stateText = "delaying";
			break;
		case esSuspended:
			stateText = "suspended";
			break;
		case esBlocked:
			stateText = "blocked";
			break;
		default:
			stateText = "invalid";
			break;
		}

		const char *mutexName = nullptr;
		if (taskDetails.eCurrentState == esResourceWaiting)
		{
			const Mutex *m = Mutex::GetMutexList();
			while (m != nullptr)
			{
				if ((const void *)m == taskDetails.pvResource)
				{
					mutexName = m->GetName();
					break;
				}
			}
		}

		const float cpuPercent = (100 * (float)taskDetails.ulRunTimeCounter)/(float)timeSinceLastCall;
		totalCpuPercent += cpuPercent;
		reply.catf(" %s(%s%s,%.1f%%,%u)", taskDetails.pcTaskName, stateText, mutexName, (double)cpuPercent, (unsigned int)taskDetails.usStackHighWaterMark);
	}
	reply.catf(", total %.1f%%", (double)totalCpuPercent);

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

// Allocate memory permanently. Using this saves about 8 bytes per object. You must not call free() on the returned object.
// It doesn't try to allocate from the free list maintained by malloc, only from virgin memory.
void *Tasks::AllocPermanent(size_t sz, std::align_val_t align) noexcept
{
	__malloc_lock(nullptr);
	void * const ret = CoreAllocPermanent(sz, align);
	__malloc_unlock(nullptr);
	return ret;
}

extern "C" void vApplicationTickHook(void) noexcept
{
	CoreSysTick();
	WatchdogReset();							// kick the watchdog
	Platform::Tick();
	++heatTaskIdleTicks;
#if 0
	const bool heatTaskStuck = (heatTaskIdleTicks >= MaxTicksInSpinState);
	if (heatTaskStuck || ticksInSpinState >= MaxTicksInSpinState)		// if we stall for 20 seconds, save diagnostic data and reset
	{
		resetting = true;
		for (size_t i = 0; i < MaxHeaters; i++)
		{
			Platform::SetHeater(i, 0.0);
		}
		Platform::DisableAllDrives();

		// We now save the stack when we get stuck in a spin loop
		__asm volatile("mrs r2, psp");
		register const uint32_t * stackPtr asm ("r2");					// we want the PSP not the MSP
		Platform::SoftwareReset(
			(heatTaskStuck) ? (uint16_t)SoftwareResetReason::heaterWatchdog : (uint16_t)SoftwareResetReason::stuckInSpin,
			stackPtr + 5);												// discard uninteresting registers, keep LR PC PSR
	}
#endif
}

static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

extern "C" void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize) noexcept
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer. */
    *pulTimerTaskStackSize = ARRAY_SIZE(uxTimerTaskStack);
}

// Helper function to cause a divide by zero error without the compiler noticing we are doing that
uint32_t Tasks::DoDivide(uint32_t a, uint32_t b) noexcept
{
	return a/b;
}

uint32_t Tasks::DoMemoryRead(const uint32_t* addr) noexcept
{
	return *addr;
}

// Functions called by CanMessageBuffer in CANlib
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept
{
	return Tasks::AllocPermanent(sz, align);
}

void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept { }

// End
