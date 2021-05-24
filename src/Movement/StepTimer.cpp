/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include <Platform.h>
#include <RTOSIface/RTOSIface.h>
#include <CanMessageFormats.h>
#include <CAN/CanInterface.h>

#if SAME5x
# include <hri_tc_e54.h>
#elif SAMC21
# include <hri_tc_c21.h>
#endif

StepTimer * volatile StepTimer::pendingList = nullptr;
volatile uint32_t StepTimer::localTimeOffset = 0;
volatile uint32_t StepTimer::whenLastSynced;
uint32_t StepTimer::prevMasterTime;												// the previous master time received
uint32_t StepTimer::prevLocalTime;												// the previous local time when the master time was received, corrected for receive processing delay
int32_t StepTimer::peakPosJitter = 0;
int32_t StepTimer::peakNegJitter = 0;
uint32_t StepTimer::peakReceiveDelay = 0;
volatile unsigned int StepTimer::syncCount = 0;
unsigned int StepTimer::numResyncs = 0;

void StepTimer::Init()
{
	// We use StepTcNumber+1 as the slave for 32-bit mode so we need to clock that one too
	EnableTcClock(StepTcNumber, GclkNum48MHz);
	EnableTcClock(StepTcNumber + 1, GclkNum48MHz);

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
	if (syncCount == MaxSyncCount)
	{
		// Check that we received a sync message recently
		const uint32_t wls = whenLastSynced;						// capture whenLastSynced before we call millis in case we get interrupted
		if (millis() - wls > MinSyncInterval)
		{
			syncCount = 0;
			++numResyncs;
		}
	}
	return syncCount == MaxSyncCount;
}

/*static*/ void StepTimer::ProcessTimeSyncMessage(const CanMessageTimeSync& msg, size_t msgLen, uint16_t timeStamp) noexcept
{
	uint32_t localTimeNow;
	uint16_t timeStampNow;
	{
		AtomicCriticalSectionLocker lock;							// there must be no delay between calling GetTimerTicks and GetTimeStampCounter
		localTimeNow = StepTimer::GetTimerTicks();
		timeStampNow = CanInterface::GetTimeStampCounter();
	}

	// The time stamp counter runs at the CAN normal bit rate, but the step clock runs at 48MHz/64. Calculate the delay to in step clocks.
	// Datasheet suggests that on the SAMC21 only 15 bits of timestamp counter are readable, but Microchip confirmed this is a documentation error (case 00625843)
	const uint32_t timeStampDelay = ((uint32_t)((timeStampNow - timeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;	// timestamp counter is 16 bits

	// Save the peak timestamp delay for diagnostic purposes
	if (timeStampDelay > peakReceiveDelay)
	{
		peakReceiveDelay = timeStampDelay;
	}

	const uint32_t oldLocalTime = prevLocalTime;					// save the previous values
	const uint32_t oldMasterTime = prevMasterTime;

	prevLocalTime = localTimeNow - timeStampDelay;
	prevMasterTime = msg.timeSent;

	const unsigned int locSyncCount = syncCount;					// capture volatile variable
	if (locSyncCount == 0)											// we can't sync until we have previous message details
	{
		syncCount = 1;
	}
	else if (msg.lastTimeSent == oldMasterTime && msg.lastTimeAcknowledgeDelay != 0)
	{
		// We have the previous message details and now we have the transmit delay for that message
		const uint32_t correctedMasterTime = oldMasterTime + msg.lastTimeAcknowledgeDelay;
		const uint32_t newOffset = oldLocalTime - correctedMasterTime;

		//TODO convert this to a PLL, but note that there could be a constant offset if the clocks run at slightly different speeds
		const uint32_t oldOffset = localTimeOffset;
		localTimeOffset = newOffset;
		const int32_t diff = (int32_t)(newOffset - oldOffset);
		if ((uint32_t)labs(diff) > MaxSyncJitter && locSyncCount > 1)
		{
			syncCount = 0;
			++numResyncs;
		}
		else
		{
			whenLastSynced = millis();
			if (locSyncCount == MaxSyncCount)
			{
				if (diff > peakPosJitter)
				{
					peakPosJitter = diff;
				}
				else if (diff < peakNegJitter)
				{
					peakNegJitter = diff;
				}
				Platform::SetPrinting(msg.isPrinting);
				if (msgLen >= CanMessageTimeSync::SizeWithRealTime)	// if real time is included
				{
					Platform::SetDateTime(msg.realTime);
				}
			}
			else
			{
				syncCount = locSyncCount + 1;
			}
		}
	}
	else
	{
		// Looks like we missed a time sync message. Ignore it.
	}
}

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
inline bool StepTimer::ScheduleTimerInterrupt(uint32_t tim)
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
			if (tmr != nextTimer || tmr == nullptr)					// tmr != nextTimer is the common case of a new step interrupt scheduled, so test that first
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
extern "C" void STEP_TC_HANDLER() SPEED_CRITICAL;

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
	whenDue = when;
	StepTimer* pst;
	for (;;)				// this loop executes at most twice
	{
		pst = pendingList;

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
			active = true;
			return false;
		}

		if (!active)
		{
			break;
		}
		CancelCallbackFromIsr();
	}

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

/*static*/ void StepTimer::Diagnostics(const StringRef& reply)
{
	reply.lcatf("Peak sync jitter %" PRIi32 "/%" PRIi32 ", peak Rx sync delay %" PRIu32 ", resyncs %u, ", peakNegJitter, peakPosJitter, peakReceiveDelay, numResyncs);
	peakNegJitter = peakPosJitter = 0;
	numResyncs = 0;
	peakReceiveDelay = 0;

	StepTimer *pst = pendingList;
	if (pst == nullptr)
	{
		reply.cat("no step interrupt scheduled");
	}
	else
	{
		reply.catf("next step interrupt due in %" PRIu32 " ticks, %s",
					pst->whenDue - GetTimerTicks(),
					((StepTc->INTENSET.reg & TC_INTFLAG_MC0) == 0) ? "disabled" : "enabled");
		if (StepTc->CC[0].reg != pst->whenDue)
		{
			reply.cat(", CC0 mismatch!!");
		}
	}
}

// Function called by FreeRTOS to read the timer
extern "C" uint32_t StepTimerGetTimerTicks() noexcept
{
	return StepTimer::GetTimerTicks();
}

// End
