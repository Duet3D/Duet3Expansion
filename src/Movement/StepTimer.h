/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

// This is a static class instead of a namespace, so that we can have private data and inline functions
class StepTimer
{
public:
	static constexpr uint32_t StepClockRate = 120000000/128;					// we don't have a divisor of 128 available so we use GCLK1 which is half GCLK0
	static constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	static constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	static constexpr uint32_t MinInterruptInterval = 12;						// 12 clocks is about 6us

	static void Init();

	static inline uint32_t GetInterruptClocks()
	{
		StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
		while (StepTc->SYNCBUSY.bit.COUNT) { }
		return StepTc->COUNT.reg;
	}

	static bool ScheduleStepInterrupt(uint32_t tim) __attribute__ ((hot));		// Schedule an interrupt at the specified clock count, or return true if it has passed already
	static void DisableStepInterrupt();											// Make sure we get no step interrupts
	static bool ScheduleSoftTimerInterrupt(uint32_t tim);						// Schedule an interrupt at the specified clock count, or return true if it has passed already
	static void DisableSoftTimerInterrupt();									// Make sure we get no software timer interrupts

	static uint32_t GetLocalTimeOffset() { return localTimeOffset; }
	static void SetLocalTimeOffset(uint32_t offset) { localTimeOffset = offset; synced = true; }
	static uint32_t ConvertToLocalTime(uint32_t masterTime) { return masterTime + localTimeOffset; }
	static uint32_t ConvertToMasterTime(uint32_t localTime) { return localTime - localTimeOffset; }
	static uint32_t GetMasterTime() { return ConvertToMasterTime(GetInterruptClocks()); }

	static void Reset() { localTimeOffset = 0; synced = false; }
	static bool IsSynced() { return synced; }

private:
	static uint32_t localTimeOffset;											// local time minus master time
	static bool synced;
};

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
