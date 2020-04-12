/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include <RTOSIface/RTOSIface.h>
#include "Move.h"

StepTimer * volatile StepTimer::pendingList = nullptr;
uint32_t StepTimer::localTimeOffset = 0;
uint32_t StepTimer::whenLastSynced;
bool StepTimer::synced = false;

void StepTimer::Init()
{
	// We use StepTcNumber+1 as the slave for 32-bit mode so we need to clock that one too
#if defined(SAME51)
	EnableTcClock(StepTcNumber, GCLK_PCHCTRL_GEN_GCLK2_Val);
	EnableTcClock(StepTcNumber + 1, GCLK_PCHCTRL_GEN_GCLK2_Val);
#elif defined(SAMC21)
	EnableTcClock(StepTcNumber, GCLK_PCHCTRL_GEN_GCLK0_Val);
	EnableTcClock(StepTcNumber + 1, GCLK_PCHCTRL_GEN_GCLK0_Val);
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

/*static*/ bool StepTimer::IsSynced()
{
	if (synced && millis() - whenLastSynced > MinSyncInterval)
	{
		synced = false;
	}
	return synced;
}

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
bool StepTimer::ScheduleTimerInterrupt(uint32_t tim)
{
	// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
	AtomicCriticalSectionLocker lock;

	const int32_t diff = (int32_t)(tim - GetTimerTicks());			// see how long we have to go
	if (diff < (int32_t)MinInterruptInterval)						// if less than about 6us or already passed
	{
		return true;												// tell the caller to simulate an interrupt instead
	}

	StepTc->CC[0].reg = tim;
	while (StepTc->SYNCBUSY.reg & TC_SYNCBUSY_CC0) { }
	StepTc->INTFLAG.reg = TC_INTFLAG_MC0;							// clear any existing compare match
	StepTc->INTENSET.reg = TC_INTFLAG_MC0;
	return false;
}

// Make sure we get no timer interrupts
void StepTimer::DisableTimerInterrupt()
{
	StepTc->INTENCLR.reg = TC_INTFLAG_MC0;
}

// The guts of the ISR
/*static*/ inline void StepTimer::Interrupt()
{
	StepTimer * tmr = pendingList;
	if (tmr != nullptr)
	{
		for (;;)
		{
			StepTimer * const nextTimer = tmr->next;
			pendingList = nextTimer;								// remove it from the pending list

			tmr->active = false;
			tmr->callback(tmr->cbParam);							// execute its callback. This may schedule another callback and hence change the pending list.

			tmr = pendingList;
			if (tmr == nullptr || tmr != nextTimer)
			{
				break;												// no more timers, or another timer has been inserted and an interrupt scheduled
			}

			if (!ScheduleTimerInterrupt(tmr->whenDue))
			{
				break;
			}
		}
	}
}

// Step pulse timer interrupt
extern "C" void STEP_TC_HANDLER() __attribute__ ((hot));

void STEP_TC_HANDLER()
{
	uint8_t tcsr = StepTc->INTFLAG.reg;								// read the status register, which clears the status bits
	tcsr &= StepTc->INTENSET.reg;									// select only enabled interrupts

	if ((tcsr & TC_INTFLAG_MC0) != 0)								// the step interrupt uses MC0 compare
	{
		StepTc->INTENCLR.reg = TC_INTFLAG_MC0;						// disable the interrupt (no need to clear it, we do that before we re-enable it)
#ifdef TIMER_DEBUG
		++numInterruptsExecuted;
		lastInterruptTime = GetInterruptClocks();
#endif
		StepTimer::Interrupt();										// this will re-enable the interrupt if necessary
	}
}

StepTimer::StepTimer() : next(nullptr), callback(nullptr), active(false)
{
}

// Set up the callback function and parameter
void StepTimer::SetCallback(TimerCallbackFunction cb, CallbackParameter param)
{
	callback = cb;
	cbParam = param;
}

// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent.
bool StepTimer::ScheduleCallbackFromIsr(Ticks when)
{
	if (active)
	{
		CancelCallbackFromIsr();
	}

	whenDue = when;

	StepTimer* pst = pendingList;

	// Optimise the common case of no other timer in the list
	if (pst == nullptr)
	{
		// No other callbacks are scheduled
		if (ScheduleTimerInterrupt(when))
		{
			return true;
		}
		next = nullptr;
		pendingList = this;
	}
	else
	{
		const Ticks now = GetTimerTicks();
		const int32_t howSoon = (int32_t)(when - now);
		if (howSoon < (int32_t)(pst->whenDue - now))
		{
			// This callback is due earlier than the first existing one
			if (ScheduleTimerInterrupt(when))
			{
				return true;
			}
			next = pst;
			pendingList = this;
		}
		else
		{
			StepTimer *prev;
			do
			{
				prev = pst;
				pst = pst->next;
			}
			while (pst != nullptr && (int32_t)(pst->whenDue - now) <= howSoon);
			next = pst;
			prev->next = this;
		}
	}

	active = true;
	return false;
}

bool StepTimer::ScheduleCallback(Ticks when)
{
	AtomicCriticalSectionLocker lock;
	return ScheduleCallbackFromIsr(when);
}

// Cancel any scheduled callback for this timer. Harmless if there is no callback scheduled.
void StepTimer::CancelCallbackFromIsr()
{
	for (StepTimer** ppst = const_cast<StepTimer**>(&pendingList); *ppst != nullptr; ppst = &((*ppst)->next))
	{
		if (*ppst == this)
		{
			*ppst = this->next;		// unlink this from the pending list
			active = false;
			break;
		}
	}
}

void StepTimer::CancelCallback()
{
	AtomicCriticalSectionLocker lock;
	CancelCallbackFromIsr();
}

// End
