/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include "RTOSIface/RTOSIface.h"
#include "SoftTimer.h"
#include "Move.h"

uint32_t StepTimer::localTimeOffset = 0;
bool StepTimer::synced = false;

void StepTimer::Init()
{
#if defined(SAME51)
	hri_mclk_set_APBDMASK_TC6_bit(MCLK);			// TODO this is currently hard coded to TC6
	hri_gclk_write_PCHCTRL_reg(GCLK, TC6_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

	// We will be using TC7 as a slave, so we must clock that too
	hri_mclk_set_APBDMASK_TC7_bit(MCLK);			// TODO this is currently hard coded to TC7
	hri_gclk_write_PCHCTRL_reg(GCLK, TC7_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));
#elif defined(SAMC21)
	hri_mclk_set_APBCMASK_TC2_bit(MCLK);			// TODO this is currently hard coded to TC2
	hri_gclk_write_PCHCTRL_reg(GCLK, TC2_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));

	// We will be using TC3 as a slave, so we must clock that too
	hri_mclk_set_APBCMASK_TC3_bit(MCLK);			// TODO this is currently hard coded to TC3
	hri_gclk_write_PCHCTRL_reg(GCLK, TC3_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));
#else
# error Unsupported processor
#endif

	if (!hri_tc_is_syncing(StepTc, TC_SYNCBUSY_SWRST))
	{
		if (hri_tc_get_CTRLA_reg(StepTc, TC_CTRLA_ENABLE))
		{
			hri_tc_clear_CTRLA_ENABLE_bit(StepTc);
			hri_tc_wait_for_sync(StepTc, TC_SYNCBUSY_ENABLE);
		}
		hri_tc_write_CTRLA_reg(StepTc, TC_CTRLA_SWRST);
	}
	hri_tc_wait_for_sync(StepTc, TC_SYNCBUSY_SWRST);

	hri_tc_write_CTRLA_reg(StepTc, TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV64);
	hri_tc_write_DBGCTRL_reg(StepTc, 0);
	hri_tc_write_EVCTRL_reg(StepTc, 0);
	hri_tc_write_WAVE_reg(StepTc, TC_WAVE_WAVEGEN_NFRQ);

	hri_tc_set_CTRLA_ENABLE_bit(StepTc);

	NVIC_DisableIRQ(StepTcIRQn);
	NVIC_ClearPendingIRQ(StepTcIRQn);
	NVIC_EnableIRQ(StepTcIRQn);
}

// Schedule an interrupt at the specified clock count, or return true if it has passed already
bool StepTimer::ScheduleStepInterrupt(uint32_t tim)
{
	AtomicCriticalSectionLocker lock;

	const int32_t diff = (int32_t)(tim - GetInterruptClocks());		// see how long we have to go
	if (diff < (int32_t)MinInterruptInterval)						// if less than about 6us or already passed
	{
		return true;												// tell the caller to simulate an interrupt instead
	}

	hri_tccount32_write_CC_reg(StepTc, 0, tim);
	hri_tc_set_INTEN_MC0_bit(StepTc);
	return false;
}

// Make sure we get no step interrupts
void StepTimer::DisableStepInterrupt()
{
	hri_tc_clear_INTEN_MC0_bit(StepTc);
}

// Schedule an interrupt at the specified clock count, or return true if it has passed already
bool StepTimer::ScheduleSoftTimerInterrupt(uint32_t tim)
{
	AtomicCriticalSectionLocker lock;
	const int32_t diff = (int32_t)(tim - GetInterruptClocks());		// see how long we have to go
	if (diff < (int32_t)MinInterruptInterval)						// if less than about 6us or already passed
	{
		return true;												// tell the caller to simulate an interrupt instead
	}

	hri_tccount32_write_CC_reg(StepTc, 1, tim);
	hri_tc_set_INTEN_MC1_bit(StepTc);
	return false;
}

// Make sure we get no software timer interrupts
void StepTimer::DisableSoftTimerInterrupt()
{
	hri_tc_clear_INTEN_MC1_bit(StepTc);
}

extern "C" void STEP_TC_HANDLER()
{
	uint8_t tcsr = StepTc->INTFLAG.reg;									// read the status register, which clears the status bits
	tcsr &= StepTc->INTENSET.reg;										// select only enabled interrupts

	if ((tcsr & TC_INTFLAG_MC0) != 0)									// the step interrupt uses MC0 compare
	{
		StepTc->INTENCLR.reg = TC_INTFLAG_MC0;							// disable the interrupt
		StepTc->INTFLAG.reg = TC_INTFLAG_MC0;							// clear the interrupt
#ifdef MOVE_DEBUG
		++numInterruptsExecuted;
		lastInterruptTime = GetInterruptClocks();
#endif
		moveInstance->Interrupt();										// execute the step interrupt
	}

	if ((tcsr & TC_INTFLAG_MC1) != 0)									// soft timer uses MC1 compare
	{
		StepTc->INTENCLR.reg = TC_INTFLAG_MC1;							// disable the interrupt
		StepTc->INTFLAG.reg = TC_INTFLAG_MC1;							// clear the interrupt
#ifdef SOFT_TIMER_DEBUG
		++numSoftTimerInterruptsExecuted;
#endif
		SoftTimer::Interrupt();
	}
}

// End
