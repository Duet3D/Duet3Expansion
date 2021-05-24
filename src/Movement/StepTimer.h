/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

class CanMessageTimeSync;

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
	static Ticks GetTimerTicks() SPEED_CRITICAL;

	// Get the tick rate (can also access it directly as StepClockRate)
	static uint32_t GetTickRate() { return StepClockRate; }

	// Convert a number of step timer ticks to microseconds
	// Our tick rate is a multiple of 1000 so instead of multiplying n by 1000000 and risking overflow, we multiply by 1000 and divide by StepClockRate/1000
	static uint32_t TicksToIntegerMicroseconds(uint32_t n) { return (n * 1000)/(StepClockRate/1000); }
	static float TicksToFloatMicroseconds(uint32_t n) { return (float)n * (1000000.0f/StepClockRate); }

	// ISR called from StepTimer. May sometimes get called prematurely.
	static void Interrupt() SPEED_CRITICAL;

	static uint32_t GetLocalTimeOffset() { return localTimeOffset; }
	static void ProcessTimeSyncMessage(const CanMessageTimeSync& msg, size_t msgLen, uint16_t timeStamp) noexcept;
	static uint32_t ConvertToLocalTime(uint32_t masterTime) { return masterTime + localTimeOffset; }
	static uint32_t ConvertToMasterTime(uint32_t localTime) { return localTime - localTimeOffset; }
	static uint32_t GetMasterTime() { return ConvertToMasterTime(GetTimerTicks()); }

	static bool IsSynced();

	static void Diagnostics(const StringRef& reply);

	static constexpr uint32_t StepClockRate = 48000000/64;						// 48MHz divided by 64
	static constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	static constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	static constexpr uint32_t MinInterruptInterval = 6;							// about 8us. Needs to be long enough for StepTimer::ScheduleTimerInterrupt to work during DMA.
	static constexpr uint32_t MinSyncInterval = 1000;							// maximum interval in milliseconds between sync messages for us to remain synced

private:
	static bool ScheduleTimerInterrupt(uint32_t tim) SPEED_CRITICAL;			// schedule an interrupt at the specified clock count, or return true if it has passed already

	StepTimer *next;
	Ticks whenDue;
	TimerCallbackFunction callback;
	CallbackParameter cbParam;
	volatile bool active;

	static StepTimer * volatile pendingList;									// list of pending callbacks, soonest first
	static volatile uint32_t localTimeOffset;									// local time minus master time
	static volatile uint32_t whenLastSynced;									// the millis tick count when we last synced
	static uint32_t prevMasterTime;												// the previous master time received
	static uint32_t prevLocalTime;												// the previous local time when the master time was received, corrected for receive processing delay
	static int32_t peakPosJitter, peakNegJitter;								// the max and min corrections we made to local time offset while synced
	static uint32_t peakReceiveDelay;											// the maximum receive delay we measured by using the receive time stamp
	static volatile unsigned int syncCount;										// the number of messages we have received since starting sync
	static unsigned int numResyncs;

	static constexpr uint32_t MaxSyncJitter = StepClockRate/100;				// 10ms
	static constexpr unsigned int MaxSyncCount = 10;
};

inline __attribute__((always_inline)) StepTimer::Ticks StepTimer::GetTimerTicks()
{
	StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
#if SAMC21
	// Tony's tests suggest that the following nop is not needed, but including it makes it faster
	asm volatile("nop");														// allow time for the peripheral to react to the command (faster than DMB instruction)
#else
	// On the SAME5x it isn't enough just to wait for SYNCBUSY.COUNT here, nor is it enough just to use a DSB instruction
	while (StepTc->CTRLBSET.bit.CMD != 0) { }
#endif
	while (StepTc->SYNCBUSY.bit.COUNT) { }
	return StepTc->COUNT.reg;
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
