/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David

 A note on bed levelling:

 As at version 1.21 we support two types of bed compensation:
 1. The old 3, 4 and 5-point compensation using a RandomProbePointSet. We will probably discontinue this soon.
 2. Mesh bed levelling

 There is an interaction between using G30 to home Z or set a precise Z=0 height just before a print, and bed compensation.
 Consider the following sequence:
 1. Home Z, using either G30 or an endstop.
 2. Run G29 to generate a height map. If the Z=0 point has drifted off, the height map may have a Z offset.
 3. Use G30 to get an accurate Z=0 point. We want to keep the shape of the height map, but get rid of the offset.
 4. Run G29 to generate a height map. This should generate a height map with on offset at the point we just probed.
 5. Cancel bed compensation. The height at the point we just probed should be zero.

 So as well as maintaining a height map, we maintain a Z offset from it. The procedure is:
 1. Whenever bed compensation is not being used, the Z offset should be zero.
 2. Whenever we run G29 to probe the bed, we have a choice:
 (a) accept that the map may have a height offset; and set the Z offset to zero. This is what we do currently.
 (b) normalise the height map to zero, adjust the Z=0 origin, and set the Z offset to zero.
 3. When we run G30 to reset the Z=0 height, and we have a height map loaded, we adjust the Z offset to be the negative of the
    height map indication of that point.
 4. If we now cancel the height map, we also clear the Z offset, and the height at the point we probed remains correct.
 5. If we now run G29 to probe again, the height map should have near zero offset at the point we probed, if there has been no drift.

 Before we introduced the Z offset, at step 4 we would have a potentially large Z error as if the G30 hadn't been run,
 and at step 5 the new height map would have an offset again.

 */

#include "Move.h"

#if SUPPORT_DRIVERS

#include "StepTimer.h"
#include "MoveTiming.h"
#include <Platform/Platform.h>
#include <CAN/CanInterface.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <Platform/TaskPriorities.h>
#include <AppNotifyIndices.h>

#if HAS_SMART_DRIVERS
# include "StepperDrivers/TMC51xx.h"
# include "StepperDrivers/TMC22xx.h"
#endif

#if 1	//debug
unsigned int moveCompleteTimeoutErrs;
unsigned int getCanMoveTimeoutErrs;
#endif

#if SAMC21 || RP2040
constexpr size_t MoveTaskStackWords = 180;
#else
constexpr size_t MoveTaskStackWords = 220;
#endif

static Task<MoveTaskStackWords> *moveTask;

extern "C" [[noreturn]] void MoveLoop(void * param) noexcept
{
	static_cast<Move*>(param)->TaskLoop();
}

Move::Move() noexcept
	: scheduledMoves(0), completedMoves(0), taskWaitingForMoveToComplete(nullptr),
#if USE_TC_FOR_STEP
	  lastStepHighTime(0),
#else
	  lastStepLowTime(0),
#endif
	  lastDirChangeTime(0),
	  numHiccups(0), stepErrorState(StepErrorState::noError)
{
	// Build the DDA ring
	DDA *dda = new DDA(nullptr);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for (size_t i = 1; i < DdaRingLength; i++)
	{
		DDA * const oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	ddaRingAddPointer->SetNext(dda);
	dda->SetPrevious(ddaRingAddPointer);

#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		movementAccumulators[i] = 0;
#if SUPPORT_CLOSED_LOOP
		netMicrostepsTaken[i] = 0.0;
#endif
	}
}

void Move::Init() noexcept
{
	// Empty the ring
	ddaRingGetPointer = ddaRingAddPointer;
	maxPrepareTime = 0;

	moveTask = new Task<MoveTaskStackWords>;
	moveTask->Create(MoveLoop, "Move", this, TaskPriority::MovePriority);

# if HAS_SMART_DRIVERS
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		SetMicrostepping(i, 16, true);
	}
# endif
}

void Move::Exit() noexcept
{
	StepTimer::DisableTimerInterrupt();
	moveTask->TerminateAndUnlink();

	// Clear the DDA ring so that we don't report any moves as pending
	while (ddaRingGetPointer != ddaRingAddPointer)
	{
		ddaRingGetPointer->Free();
		ddaRingGetPointer = ddaRingGetPointer->GetNext();
	}
}

[[noreturn]] void Move::TaskLoop() noexcept
{
#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif
	while (true)
	{
		for (;;)
		{
			// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
			while (ddaRingCheckPointer->GetState() == DDA::completed)
			{
				// Check for step errors and record/print them if we have any, before we lose the DMs
				if (ddaRingCheckPointer->HasStepError())
				{
#ifdef DEBUG
					if (Platform::Debug(moduleMove))
					{
						ddaRingCheckPointer->DebugPrintAll();
					}
#endif
					Platform::LogError(ErrorCode::BadMove);
				}

				// Now release the DMs and check for underrun
				ddaRingCheckPointer->Free();
				ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
			}

			// If we have a free slot for a new move, quit this loop
			if (ddaRingAddPointer->GetState() == DDA::empty)
			{
				break;
			}

			// Wait for a move to complete
			{
				AtomicCriticalSectionLocker lock;

				if (ddaRingCheckPointer->GetState() == DDA::completed)
				{
					continue;
				}
				taskWaitingForMoveToComplete = TaskBase::GetCallerTaskHandle();
			}
#if 1	//debug
			if (!TaskBase::TakeIndexed(NotifyIndices::Move, 2000) && ddaRingCheckPointer->GetState() == DDA::completed)
			{
				++moveCompleteTimeoutErrs;
			}
#else
			TaskBase::Take();
#endif
		};

		// Get another move and add it to the ring
#if 1	//debug
		CanMessageBuffer *buf;
		for (;;)
		{
			buf = CanInterface::GetCanMove(2000);
			if (buf != nullptr)
			{
				break;
			}
			buf = CanInterface::GetCanMove(0);
			if (buf != nullptr)
			{
				++getCanMoveTimeoutErrs;
				break;
			}
		}
#else
		CanMessageBuffer *buf = CanInterface::GetCanMove(TaskBase::TimeoutUnlimited);
#endif
		MicrosecondsTimer prepareTimer;
		const CanMessageType msgType = buf->id.MsgType();
		switch (msgType)
		{
		case CanMessageType::movementLinearShaped:
			{
				const bool moveAdded = ddaRingAddPointer->Init(buf->msg.moveLinearShaped);
				if (moveAdded)
				{
					ddaRingAddPointer = ddaRingAddPointer->GetNext();
					scheduledMoves++;
				}
				const uint32_t elapsedTime = prepareTimer.Read();
				if (elapsedTime > Move::maxPrepareTime)
				{
					Move::maxPrepareTime = elapsedTime;
				}
			}
			break;

		default:				// should not happen
			break;
		}

		CanMessageBuffer::Free(buf);

		// See whether we need to kick off a move
		if (currentDda == nullptr)
		{
			// No DDA is executing, so start executing a new one if possible
			DDA * const cdda = ddaRingGetPointer;										// capture volatile variable
			if (cdda->GetState() == DDA::frozen)
			{
				IrqDisable();
				StartNextMove(cdda, StepTimer::GetTimerTicks());
#if DEDICATED_STEP_TIMER
				if (cdda->ScheduleNextStepInterrupt())
#else
				if (cdda->ScheduleNextStepInterrupt(timer))
#endif
				{
					Interrupt();
				}
				IrqEnable();
			}
		}
	}
}

void Move::Diagnostics(const StringRef& reply) noexcept
{
	reply.catf("Moves scheduled %" PRIu32 ", completed %" PRIu32 ", in progress %d, hiccups %" PRIu32 ", segs %u, step errors %u, maxLate %" PRIi32 " maxPrep %" PRIu32 ", maxOverdue %" PRIu32 ", maxInc %" PRIu32,
					scheduledMoves, completedMoves, (int)(currentDda != nullptr), numHiccups, MoveSegment::NumCreated(),
					GetAndClearStepErrors(), DriveMovement::GetAndClearMaxStepsLate(), maxPrepareTime, GetAndClearMaxTicksOverdue(), GetAndClearMaxOverdueIncrement());
	numHiccups = 0;
	maxPrepareTime = 0;
#if 1	//debug
	reply.catf(", mcErrs %u, gcmErrs %u", moveCompleteTimeoutErrs, getCanMoveTimeoutErrs);
#endif
#if 1	//debug
	reply.catf(", ebfmin %.2f max %.2f", (double)minExtrusionPending, (double)maxExtrusionPending);
	minExtrusionPending = maxExtrusionPending = 0.0;
#endif
}

int32_t Move::GetPosition(size_t driver) const noexcept
{
	return ddaRingAddPointer->GetPrevious()->GetPosition(driver);
}

// Stop some or all of the moving drivers
void Move::StopDrivers(uint16_t whichDrives) noexcept
{
#if SAME5x
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);
#elif SAMC21 || RP2040
	const irqflags_t flags = IrqSave();
#else
# error Unsupported processor
#endif
	DDA *const cdda = currentDda;					// capture volatile
	if (cdda != nullptr)
	{
		cdda->StopDrivers(whichDrives);
		if (cdda->GetState() == DDA::completed)
		{
			CurrentMoveCompleted();					// tell the DDA ring that the current move is complete
		}
	}
#if SAME5x
	RestoreBasePriority(oldPrio);
#elif SAMC21 || RP2040
	IrqRestore(flags);
#else
# error Unsupported processor
#endif
}

// Filament monitor support
// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and isPrinting is true unless we are currently executing an extruding but non-printing move
int32_t Move::GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept
{
	DriveMovement& dm = dms[driver];
	AtomicCriticalSectionLocker lock;							// we don't want a move to complete and the ISR update the movement accumulators while we are doing this
	const int32_t ret = dm.movementAccumulator;
	const int32_t adjustment = dm.GetNetStepsTaken();
	dm.movementAccumulator = -adjustment;
	isPrinting = dms[driver].extruderPrinting;
	return ret + adjustment;
}

// This is the function that is called by the timer interrupt to step the motors. It is also called form Move::Spin() if the first step for a move is due immediately.
// This may occasionally get called prematurely.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void Move::Interrupt() noexcept
{
#if SINGLE_DRIVER
	if (dms[0].state >= firstMotionState)
#else
	if (activeDMs != nullptr)
	{
#endif
	uint32_t now = StepTimer::GetMovementTimerTicks();
	const uint32_t isrStartTime = now;
#if 0	// TEMP DEBUG - see later
		for (unsigned int iterationCount = 0; ; )
#else
		for (;;)
#endif
		{
			// Generate steps for the current move segments
			StepDrivers(now);									// check endstops if necessary and step the drivers

			if (activeDMs == nullptr || stepErrorState != StepErrorState::noError )
			{
				WakeMoveTaskFromISR();							// we may have just completed a special move, so wake up the Move task so that it can notice that
				break;
			}

			// Schedule a callback at the time when the next step is due, and quit unless it is due immediately
			if (!ScheduleNextStepInterrupt())
			{
				break;
			}

			// The next step is due immediately. Check whether we have been in this ISR for too long already and need to take a break
			now = StepTimer::GetMovementTimerTicks();
			const int32_t clocksTaken = (int32_t)(now - isrStartTime);
			if (clocksTaken >= (int32_t)MoveTiming::MaxStepInterruptTime)
			{
				// Force a break by updating the move start time.
				++numHiccups;
				uint32_t hiccupTimeInserted = 0;
				for (uint32_t hiccupTime = MoveTiming::HiccupTime; ; hiccupTime += MoveTiming::HiccupIncrement)
				{
					hiccupTimeInserted += hiccupTime;
					StepTimer::IncreaseMovementDelay(hiccupTime);

					// Reschedule the next step interrupt. This time it should succeed if the hiccup time was long enough.
					if (!ScheduleNextStepInterrupt())
					{
						//TODO tell the main board we are behind schedule
						(void)hiccupTimeInserted;
					}
					// The hiccup wasn't long enough, so go round the loop again
				}
			}
		}
	}
}

#if HAS_SMART_DRIVERS

bool Move::SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept
{
	return SmartDrivers::SetMicrostepping(driver, microsteps, interpolate);
}

#endif

#endif	//SUPPORT_DRIVERS

// End
