/*
 * StepTimer.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPTIMER_H_
#define SRC_MOVEMENT_STEPTIMER_H_

#include "RepRapFirmware.h"

namespace StepTimer
{
	constexpr uint32_t StepClockRate = 120000000/128;					// we don't have a divisor of 128 available so we use GCLK1 which is half GCLK0
	constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
	constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;
	constexpr uint32_t MinInterruptInterval = 12;						// 12 clocks is about 6us

	void Init();

	inline uint32_t GetInterruptClocks()
	{
		StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
		while (StepTc->SYNCBUSY.bit.COUNT) { }
		return StepTc->COUNT.reg;
	}

	bool ScheduleStepInterrupt(uint32_t tim) __attribute__ ((hot));		// Schedule an interrupt at the specified clock count, or return true if it has passed already
	void DisableStepInterrupt();										// Make sure we get no step interrupts
	bool ScheduleSoftTimerInterrupt(uint32_t tim);						// Schedule an interrupt at the specified clock count, or return true if it has passed already
	void DisableSoftTimerInterrupt();									// Make sure we get no software timer interrupts
}

#endif /* SRC_MOVEMENT_STEPTIMER_H_ */
