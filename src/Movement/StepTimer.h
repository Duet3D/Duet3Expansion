/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

// Class to implement a software timer with a few microseconds resolution
class StepTimer
{
public:
	// The callback function returns true if it wants another callback, after setting the requested time via the second parameter
	typedef uint32_t Ticks;
	typedef void (*TimerCallbackFunction)(CallbackParameter);

	StepTimer();

	// Set up the callback function and parameter
	void SetCallback(TimerCallbackFunction cb, CallbackParameter param);

	// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent
	bool ScheduleCallback(Ticks when);

	// As ScheduleCallback but base priority >= NvicPriorityStep when called
	bool ScheduleCallbackFromIsr(Ticks when);

	// Cancel any scheduled callbacks
	void CancelCallback();

	// As CancelCallback but base priority >= NvicPriorityStep when called
	void CancelCallbackFromIsr();

	// Initialise the timer system
	static void Init();

	// Disable the timer interrupt. Called when we shut down the system.
	static void DisableTimerInterrupt();

	// Get the current tick count
	static Ticks GetTimerTicks() __attribute__ ((hot));

	// Get the tick rate (can also access it directly as StepClockRate)
	static uint32_t GetTickRate() { return StepClockRate; }

	// Convert a number of step timer ticks to microseconds
	// Our tick rate is a multiple of 1000 so instead of multiplying n by 1000000 and risking overflow, we multiply by 1000 and divide by StepClockRate/1000
	static uint32_t TicksToMicroseconds(uint32_t n) { return (n * 1000)/(StepClockRate/1000); }

	// ISR called from StepTimer. May sometimes get called prematurely.
	static void Interrupt();

	static uint32_t GetLocalTimeOffset() { return localTimeOffset; }
	static void SetLocalTimeOffset(uint32_t offset) { localTimeOffset = offset; synced = true; whenLastSynced = millis(); }
	static uint32_t ConvertToLocalTime(uint32_t masterTime) { return masterTime + localTimeOffset; }
	static uint32_t ConvertToMasterTime(uint32_t localTime) { return localTime - localTimeOffset; }
	static uint32_t GetMasterTime() { return ConvertToMasterTime(GetTimerTicks()); }

	static bool IsSynced();

	static void Diagnostics(const StringRef& reply);

	static constexpr uint32_t StepClockRate = 48000000/64;						// 48MHz divided by 64
	static constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	static constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	static constexpr uint32_t MinInterruptInterval = 6;							// about 6us
	static constexpr uint32_t MinSyncInterval = 1000;							// maximum interval in milliseconds between sync messages for us to remain synced

private:
	static bool ScheduleTimerInterrupt(uint32_t tim);							// Schedule an interrupt at the specified clock count, or return true if it has passed already

	StepTimer *next;
	Ticks whenDue;
	TimerCallbackFunction callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer * volatile pendingList;									// list of pending callbacks, soonest first
	static volatile uint32_t localTimeOffset;									// local time minus master time
	static volatile uint32_t whenLastSynced;									// the millis tick count when we last synced
	static volatile bool synced;
};

inline __attribute__((always_inline)) StepTimer::Ticks StepTimer::GetTimerTicks()
{
	StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
	// On the EXP3HC board it isn't enough just to wait for SYNCBUSY.COUNT here
	while (StepTc->CTRLBSET.bit.CMD != 0) { }
	while (StepTc->SYNCBUSY.bit.COUNT) { }
	return StepTc->COUNT.reg;
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
