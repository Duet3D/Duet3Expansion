/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include <Platform/Platform.h>
#include <RTOSIface/RTOSIface.h>
#include <CanMessageFormats.h>
#include <CAN/CanInterface.h>
#include "MoveTiming.h"

#if DEDICATED_STEP_TIMER
# include <Movement/Move.h>				// for Move::StepInterrupt
#endif

#if SAME5x
# include <hri_tc_e54.h>
#elif SAMC21
# include <hri_tc_c21.h>
#elif RP2040
# include <hardware/watchdog.h>
# include <hardware/timer.h>
#elif STM32H5
# include <stm32h5xx_hal_conf.h>
# include <stm32h5xx_hal_tim.h>
#elif STM32H7
# include <stm32h7xx_hal_conf.h>
# include <stm32h7xx_hal_tim.h>
#endif

StepTimer * volatile StepTimer::pendingList = nullptr;
uint32_t StepTimer::movementDelay = 0;											// how many timer ticks the move timer is behind the raw timer
uint32_t StepTimer::ownMovementDelay = 0;										// how many timer ticks the move timer is behind the raw timer
bool StepTimer::ownMovementDelayIncreased = false;								// true if we added movement delay and we haven't broadcast it or requested it

volatile uint32_t StepTimer::localTimeOffset = 0;
volatile uint32_t StepTimer::whenLastSynced;
uint32_t StepTimer::prevMasterTime;												// the previous master time received
uint32_t StepTimer::prevLocalTime;												// the previous local time when the master time was received, corrected for receive processing delay
int32_t StepTimer::peakPosJitter = 0;
int32_t StepTimer::peakNegJitter = 0;
int32_t StepTimer::errorAccumulator = 0;
bool StepTimer::gotJitter = false;
uint32_t StepTimer::peakReceiveDelay = 0;
volatile unsigned int StepTimer::syncCount = 0;
unsigned int StepTimer::numJitterResyncs = 0;
unsigned int StepTimer::numTimeoutResyncs = 0;

extern "C" void STEP_TC_HANDLER() noexcept SPEED_CRITICAL;

void StepTimer::Init() noexcept
{
#if RP2040
	// Reprogram the tick generator to run at 750kHz instead of 1MHz. We use a 12MHz crystal, so we need a divisor of 16.
	watchdog_start_tick((XOSC_MHZ * 4)/3);
	// Claim timer 0 because we use it as the step timer
	hardware_alarm_claim(StepTimerAlarmNumber);
	// Claim the associated interrupt
	irq_set_exclusive_handler(StepTcIRQn, STEP_TC_HANDLER);
	// Clear and enable the interrupt
	NVIC_DisableIRQ((IRQn_Type)StepTcIRQn);
	NVIC_ClearPendingIRQ((IRQn_Type)StepTcIRQn);
	NVIC_EnableIRQ((IRQn_Type)StepTcIRQn);
#elif STM32
	// The CAN external time stamp counter is timer 3.
	// As that is only 16-bit and we need a 32-bit step timer, we use timer 5 for the step timer and clock it at the same rate as timer 3.
	EnableTimerClock(StepTimerNumber);
	EnableTimerClock(TimeStampTimerNumber);
	StepTimerHw->PSC = GetTimerClockFrequency(StepTimerNumber)/StepClockRate;
	TimeStampTimerHw->PSC = GetTimerClockFrequency(TimeStampTimerNumber)/StepClockRate;
	StepTimerHw->DIER &= ~(TIM_DIER_CC1IE);							// disable the interrupt
	StepTimerHw->ARR = 0xFFFFFFFF;
	TimeStampTimerHw->ARR = 0x0000FFFF;
	NVIC_SetPriority(StepTimerIRQn, NvicPriorityStep);			    // set the priority for this IRQ ->ARR
	NVIC_ClearPendingIRQ(StepTimerIRQn);
	NVIC_EnableIRQ(StepTimerIRQn);
	{
		// Start the two timers in sync
		AtomicCriticalSectionLocker lock;
		StepTimerHw->CNT = 0;
		TimeStampTimerHw->CNT = 0;
		StepTimerHw->CR1 |= TIM_CR1_CEN;
		TimeStampTimerHw->CR1 |= TIM_CR1_CEN;
	}
#elif SAMC21 || SAME5x
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
#else
# error Unsupported hardware
#endif
}

#if !RP2040 && !STM32

// Get the step timer clock count
/*static*/ StepTimer::Ticks StepTimer::GetTimerTicks() noexcept
{
	AtomicCriticalSectionLocker lock;
	StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
# if SAMC21
	// Tony's tests suggest that the following nop is not needed, but including it makes it faster
	asm volatile("nop");														// allow time for the peripheral to react to the command (faster than DMB instruction)
# else
	// On the SAME5x it isn't enough just to wait for SYNCBUSY.COUNT here, nor is it enough just to use a DSB instruction
	while (StepTc->CTRLBSET.bit.CMD != 0) { }
# endif
	while (StepTc->SYNCBUSY.bit.COUNT) { }
	return StepTc->COUNT.reg;
}

// Get the step timer clock count
/*static*/ StepTimer::Ticks StepTimer::GetTimerTicksWhenInterruptsDisabled() noexcept
{
	StepTc->CTRLBSET.reg = TC_CTRLBSET_CMD_READSYNC;
# if SAMC21
	// Tony's tests suggest that the following nop is not needed, but including it makes it faster
	asm volatile("nop");														// allow time for the peripheral to react to the command (faster than DMB instruction)
# else
	// On the SAME5x it isn't enough just to wait for SYNCBUSY.COUNT here, nor is it enough just to use a DSB instruction
	while (StepTc->CTRLBSET.bit.CMD != 0) { }
# endif
	while (StepTc->SYNCBUSY.bit.COUNT) { }
	return StepTc->COUNT.reg;
}

#endif

// Check whether we have synced and received a clock sync message recently
/*static*/ bool StepTimer::CheckSynced() noexcept
{
	if (syncCount == MaxSyncCount)
	{
		// Check that we received a sync message recently
		const uint32_t wls = whenLastSynced;						// capture whenLastSynced before we call millis in case we get interrupted
		if (millis() - wls > MinSyncInterval)
		{
			syncCount = 0;
			++numTimeoutResyncs;
		}
	}
	return syncCount == MaxSyncCount;
}

// Check whether we have synced
/*static*/ bool StepTimer::IsSynced() noexcept
{
	return syncCount == MaxSyncCount;
}

/*static*/ void StepTimer::ProcessTimeSyncMessage(const CanMessageTimeSync& msg, size_t msgLen, uint16_t timeStamp) noexcept
{
	static uint32_t originalOffset = 0;

#if SAME70 || STM32 || (RP2040 && !USE_SPICAN)
	// On these processors the timestamp counter is the same as the step counter
	const uint32_t localTimeNow = StepTimer::GetTimerTicks();
	const uint32_t timeStampDelay = (localTimeNow - timeStamp) & 0xFFFF;
#else
	uint32_t localTimeNow;
	uint16_t timeStampNow;
	{
		AtomicCriticalSectionLocker lock;							// there must be no delay between calling GetTimerTicks and GetTimeStampCounter
		localTimeNow = StepTimer::GetTimerTicksWhenInterruptsDisabled();
		timeStampNow = CanInterface::GetTimeStampCounter();
	}

	// The time stamp counter runs at the CAN normal bit rate, but the step clock runs at 48MHz/64. Calculate the delay in step clocks.
	// Datasheet suggests that on the SAMC21 only 15 bits of timestamp counter are readable, but Microchip confirmed this is a documentation error (case 00625843)
	const uint32_t timeStampDelay = ((uint32_t)((timeStampNow - timeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;	// timestamp counter is 16 bits
#endif

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
		errorAccumulator = 0;
	}
	else if (msg.lastTimeSent == oldMasterTime && msg.lastTimeAcknowledgeDelay != 0)
	{
		// We have the previous message details and now we have the transmit delay for that message
		const uint32_t correctedMasterTime = oldMasterTime + msg.lastTimeAcknowledgeDelay;
		const uint32_t newOffset = oldLocalTime - correctedMasterTime;

		const int32_t diff = (int32_t)(newOffset - localTimeOffset);
		if ((uint32_t)labs(diff) > MaxSyncJitter && locSyncCount > 1)
		{
			localTimeOffset = newOffset;
			syncCount = 0;
			++numJitterResyncs;
		}
		else
		{
			whenLastSynced = millis();
			if (locSyncCount < MaxSyncCount)
			{
				localTimeOffset = newOffset;
				originalOffset = newOffset;
				syncCount = locSyncCount + 1;
			}
			else
			{
				errorAccumulator += diff;
				localTimeOffset += (uint32_t)(errorAccumulator/64 + diff/4);		// this is effectively a PI controller with P=1/4, I=freq/64
				if (!gotJitter)
				{
					peakPosJitter = peakNegJitter = diff;
					gotJitter = true;
				}
				else if (diff > peakPosJitter)
				{
					peakPosJitter = diff;
				}
				else if (diff < peakNegJitter)
				{
					peakNegJitter = diff;
				}
				Platform::SetPrinting(msg.isPrinting);
				if (msgLen >= CanMessageTimeSync::SizeWithRealTime)					// if real time is included
				{
					Platform::SetDateTime(msg.realTime);
					if (msgLen >= CanMessageTimeSync::SizeWithRealTimeAndMovementDelay)
					{
						AtomicCriticalSectionLocker lock;
						if (msg.movementDelay >= movementDelay)
						{
							movementDelay = msg.movementDelay;
							ownMovementDelayIncreased = false;
						}
					}
				}
				if (Platform::Debug(Module::CAN))
				{
					debugPrintf("TS RxD,TxD,diff,errac,offs %" PRIu32 ",%u,%" PRIi32 ",%" PRIi32 ",%" PRIi32 "\n",
								timeStampDelay, (unsigned int)msg.lastTimeAcknowledgeDelay, diff, errorAccumulator, (int32_t)(newOffset - originalOffset));
				}
			}
		}
	}
	else
	{
		// Looks like we missed a time sync message, or the message didn't specify the acknowledge delay. Ignore it.
	}
}

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
inline /*static*/ bool StepTimer::ScheduleTimerInterrupt(Ticks tim) noexcept
{
	// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
	AtomicCriticalSectionLocker lock;

	const int32_t diff = (int32_t)(tim - GetTimerTicksWhenInterruptsDisabled());	// see how long we have to go
	if (diff < (int32_t)MoveTiming::MinInterruptInterval)			// if less than about 6us or already passed
	{
		return true;												// tell the caller to simulate an interrupt instead
	}

#if STM32
	StepTimerHw->SR = ~(TIM_IT_CC1);								// clear any pending interrupt
	StepTimerHw->CCR1 = tim;										// set the time when we want the interrupt
	StepTimerHw->DIER |= TIM_DIER_CC1IE;							// enable the interrupt
#elif RP2040
	hw_set_bits(&timer_hw->inte, 1u << StepTimerAlarmNumber);		// enable the interrupt
	timer_hw->alarm[StepTimerAlarmNumber] = tim;					// writing the value arms the timer
#else
	StepTc->CC[0].reg = tim;
	while (StepTc->SYNCBUSY.reg & TC_SYNCBUSY_CC0) { }
	StepTc->INTFLAG.reg = TC_INTFLAG_MC0;							// clear any existing compare match
	StepTc->INTENSET.reg = TC_INTFLAG_MC0;
#endif
	return false;
}

#if DEDICATED_STEP_TIMER

// Schedule a step interrupt, returning true if it was not scheduled because it is already due or imminent.
// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
/*static*/ bool StepTimer::ScheduleMovementCallbackFromIsr(Ticks when) noexcept
{
	when += movementDelay + localTimeOffset;

	// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
	AtomicCriticalSectionLocker lock;

	const int32_t diff = (int32_t)(when - GetTimerTicksWhenInterruptsDisabled());	// see how long we have to go
	if (diff < (int32_t)MoveTiming::MinInterruptInterval)			// if less than about 6us or already passed
	{
		return true;												// tell the caller to simulate an interrupt instead
	}

#if STM32
	StepTimerHw->CCR1 = when;
	StepTimerHw->SR = ~(TIM_IT_CC1);								// clear any pending compare match
	StepTimerHw->DIER |= TIM_DIER_CC1IE;							// enable the interrupt
#else
	StepTc->CC[1].reg = when;
	while (StepTc->SYNCBUSY.reg & TC_SYNCBUSY_CC1) { }
	StepTc->INTFLAG.reg = TC_INTFLAG_MC1;							// clear any existing compare match
	StepTc->INTENSET.reg = TC_INTFLAG_MC1;
#endif
	return false;
}

#endif

// Make sure we get no timer interrupts
void StepTimer::DisableTimerInterrupt() noexcept
{
#if STM32
	StepTimerHw->DIER &= ~(TIM_DIER_CC1IE);							// disable the interrupt
#elif RP2040
	hw_clear_bits(&timer_hw->inte, 1u << StepTimerAlarmNumber);		// disable the interrupt
#else
	StepTc->INTENCLR.reg = TC_INTFLAG_MC0 | TC_INTFLAG_MC1;
#endif
}

// The guts of the ISR for the generic timer
/*static*/ inline void StepTimer::Interrupt() noexcept
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
			if (likely(tmr != nextTimer) || tmr == nullptr)	// tmr != nextTimer is the common case of a new step interrupt scheduled, so test that first
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

// Step pulse timer ISR
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void STEP_TC_HANDLER() noexcept
{
#if STM32
	StepTimerHw->SR = ~(TIM_IT_CC1);								// clear any pending compare match
	StepTimerHw->DIER &= ~(TIM_DIER_CC1IE);							// disable the interrupt
	StepTimer::Interrupt();											// this will re-enable the interrupt if necessary
#elif RP2040
	hw_clear_bits(&timer_hw->intr, 1u << StepTimerAlarmNumber);		// clear the alarm interrupt
	StepTimer::Interrupt();											// this will re-enable the interrupt if necessary
#else
	uint8_t tcsr = StepTc->INTFLAG.reg;								// read the status register, which clears the status bits
	tcsr &= StepTc->INTENSET.reg;									// select only enabled interrupts

# if DEDICATED_STEP_TIMER
	if (likely((tcsr & TC_INTFLAG_MC1) != 0))						// the dedicated step interrupt interrupt uses MC1 compare
	{
		StepTc->INTENCLR.reg = TC_INTFLAG_MC1;						// disable the interrupt (no need to clear it, we do that before we re-enable it)
		moveInstance->Interrupt();
	}

	if (unlikely((tcsr & TC_INTFLAG_MC0) != 0))						// the generic timer interrupt uses MC0 compare
# else
	if (likely((tcsr & TC_INTFLAG_MC0) != 0))						// the generic timer interrupt uses MC0 compare
# endif
	{
		StepTc->INTENCLR.reg = TC_INTFLAG_MC0;						// disable the interrupt (no need to clear it, we do that before we re-enable it)
# ifdef TIMER_DEBUG
		++numInterruptsExecuted;
		lastInterruptTime = GetInterruptClocks();
# endif
		StepTimer::Interrupt();										// this will re-enable the interrupt if necessary
	}
#endif
}

StepTimer::StepTimer() noexcept : next(nullptr), callback(nullptr), active(false)
{
}

// Set up the callback function and parameter
void StepTimer::SetCallback(TimerCallbackFunction cb, CallbackParameter param) noexcept
{
	callback = cb;
	cbParam = param;
}

#if !DEDICATED_STEP_TIMER

// As ScheduleCallback but base priority >= NvicPriorityStep when called. Can be called from within a callback.
bool StepTimer::ScheduleMovementCallbackFromIsr(Ticks when) noexcept
{
	return ScheduleCallbackFromIsr(when + movementDelay + localTimeOffset);
}

#endif

// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent.
bool StepTimer::ScheduleCallbackFromIsr(Ticks when) noexcept
{
	whenDue = when;
	StepTimer* pst;
	for (;;)				// this loop executes at most twice
	{
		pst = pendingList;

		// Optimise the common case of no other timer in the list
		if (likely(pst == nullptr))
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

bool StepTimer::ScheduleCallback(Ticks when) noexcept
{
	AtomicCriticalSectionLocker lock;
	return ScheduleCallbackFromIsr(when);
}

// Cancel any scheduled callback for this timer. Harmless if there is no callback scheduled.
void StepTimer::CancelCallbackFromIsr() noexcept
{
	for (StepTimer** ppst = const_cast<StepTimer**>(&pendingList); *ppst != nullptr; ppst = &((*ppst)->next))
	{
		if (*ppst == this)
		{
			*ppst = this->next;		// unlink this from the pending list
			this->next = nullptr;
			break;
		}
	}
	active = false;
}

void StepTimer::CancelCallback() noexcept
{
	AtomicCriticalSectionLocker lock;
	CancelCallbackFromIsr();
}

/*static*/ void StepTimer::Diagnostics(const StringRef& reply)
{
	reply.lcatf("Sync err accum %" PRIi32 ", peak jitter %" PRIi32 "/%" PRIi32 ", peak Rx delay %" PRIu32 ", resyncs %u/%u, ",
					errorAccumulator, peakNegJitter, peakPosJitter, peakReceiveDelay, numTimeoutResyncs, numJitterResyncs);
	gotJitter = false;
	numTimeoutResyncs = numJitterResyncs = 0;
	peakReceiveDelay = 0;

	const StepTimer *const pst = pendingList;
	if (pst == nullptr)
	{
		reply.cat("no timer interrupt scheduled");
	}
	else
	{
#if STM32
		reply.lcatf("Next step interrupt due in %" PRIu32 " ticks, %s",
					pst->whenDue - GetTimerTicks(),
					(StepTimerHw->DIER & TIM_IT_CC1) == 0 ? "disabled" : "enabled");
		if (StepTimerHw->CCR1 != pst->whenDue)
		{
			reply.cat(", CC0 mismatch!!");
		}
#elif RP2040
		reply.catf("next timer interrupt due in %" PRIu32 " ticks, %s",
					timer_hw->alarm[StepTimerAlarmNumber] - GetTimerTicks(),
					(timer_hw->inte & (1u << StepTimerAlarmNumber)) ? "enabled" : "disabled");
#else
		reply.catf("next timer interrupt due in %" PRIu32 " ticks, %s",
					pst->whenDue - GetTimerTicks(),
					((StepTc->INTENSET.reg & TC_INTFLAG_MC0) == 0) ? "disabled" : "enabled");
		if (StepTc->CC[0].reg != pst->whenDue)
		{
			reply.cat(", CC0 mismatch!!");
		}
#endif
	}

#if DEDICATED_STEP_TIMER && (SAME5x || SAMC21)
	reply.catf(", next step interrupt due in %" PRIu32 " ticks, %s",
				StepTc->CC[1].reg - GetTimerTicks(),
				((StepTc->INTENSET.reg & TC_INTFLAG_MC1) == 0) ? "disabled" : "enabled");
#endif
}

// End
