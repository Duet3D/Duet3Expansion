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
	: scheduledMoves(0), taskWaitingForMoveToComplete(nullptr),
#if SUPPORT_SLOW_DRIVERS
# if USE_TC_FOR_STEP
	  lastStepHighTime(0),
# else
	  lastStepLowTime(0),
# endif
	  lastDirChangeTime(0),
#endif
	  numHiccups(0), stepErrorState(StepErrorState::noError)
{
#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		lastMoveStepsTaken[i] = 0;
		stepsPerMm[i] = DefaultStepsPerMm;
		directions[i] = true;
#if SUPPORT_CLOSED_LOOP
		netMicrostepsTaken[i] = 0.0;
#endif
	}
}

void Move::Init() noexcept
{
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
}

[[noreturn]] void Move::TaskLoop() noexcept
{
#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif
	while (true)
	{
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
				const bool moveAdded = AddMove(buf->msg.moveLinearShaped);
				if (moveAdded)
				{
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
	}
}

void Move::Diagnostics(const StringRef& reply) noexcept
{
	reply.catf("Moves scheduled %" PRIu32 ", hiccups %" PRIu32 ", segs %u, step errors %u, maxLate %" PRIi32 " maxPrep %" PRIu32 ", maxOverdue %" PRIu32 ", maxInc %" PRIu32,
					scheduledMoves, numHiccups, MoveSegment::NumCreated(),
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
	return dms[driver].currentMotorPosition;
}

// Set up a remote move. Return true if it represents real movement, else false.
// All values have already been converted to step clocks and the total distance has been normalised to 1.0.
// The whenToExecute field of the movement message has already bee converted to local time
// This version handles the new movement message that includes the input shaping plan and passes extruder movement as distance, not steps
bool Move::AddMove(const CanMessageMovementLinearShaped& msg) noexcept
{
	// Prepare for movement
	PrepParams params;

	// Normalise the move to unit distance
	params.acceleration = msg.acceleration;
	params.deceleration = msg.deceleration;
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;

	// We occasionally receive a message with zero clocks needed. This messes up the calculations, so add one steady clock in this case.
	if (msg.accelerationClocks + msg.steadyClocks + msg.decelClocks == 0)
	{
		params.steadyClocks = 1;
	}

	MovementFlags segFlags;
	segFlags.Clear();
	segFlags.nonPrintingMove = !msg.usePressureAdvance;
	segFlags.noShaping = !msg.useLateInputShaping;

	for (size_t drive = 0; drive < msg.numDrivers; drive++)
	{
		if ((msg.extruderDrives & (1u << drive)) != 0)
		{
			// It's an extruder
			const float extrusionRequested = msg.perDrive[drive].extrusion;
			if (extrusionRequested != 0.0)
			{
				AddLinearSegments(drive, msg.whenToExecute, params, extrusionRequested, segFlags);
				//TODO will Move do the following?
				EnableDrivers(drive, false);
			}
		}
		else
		{
			const float delta = (float)msg.perDrive[drive].steps;
			lastMoveStepsTaken[drive] = delta;
			if (delta != 0.0)
			{
				AddLinearSegments(drive, msg.whenToExecute, params, delta, segFlags);
				//TODO will Move do the following?
				EnableDrivers(drive, false);
			}
		}
	}
	return true;
}

#if SINGLE_DRIVER

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void Move::StepDrivers(uint32_t now) noexcept
{
# if SUPPORT_CLOSED_LOOP
	if (ClosedLoop::GetClosedLoopInstance(0)->IsClosedLoopEnabled())
	{
		return;
	}
# endif

	// Determine whether the driver is due for stepping, overdue, or will be due very shortly
	if (dms[0].state >= DMState::firstMotionState && (int32_t)(dms[0].nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)	// if the next step is due
	{
		// Step the driver

# if SUPPORT_SLOW_DRIVERS
		if (IsSlowDriver())									// if using a slow driver
		{
#  if USE_TC_FOR_STEP
			const uint32_t lastStepPulseTime = lastStepHighTime;
			while (now - lastStepPulseTime < GetSlowDriverStepPeriodClocks() || now - lastDirChangeTime < GetSlowDriverDirSetupClocks())
			{
				now = StepTimer::GetTimerTicks();
			}
			StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
			lastStepHighTime = StepTimer::GetTimerTicks();
			(void)dms[0].CalcNextStepTime(now);
#  else
			uint32_t lastStepPulseTime = lastStepLowTime;
			while (now - lastStepPulseTime < GetSlowDriverStepLowClocks() || now - lastDirChangeTime < GetSlowDriverDirSetupClocks())
			{
				now = StepTimer::GetTimerTicks();
			}
			StepDriverHigh();									// generate the step
			lastStepPulseTime = StepTimer::GetTimerTicks();
			(void)dms[0].CalcNextStepTime(*this);

			// 3a. Reset the step pin low
			while (StepTimer::GetTimerTicks() - lastStepPulseTime < GetSlowDriverStepHighClocks()) {}
			StepDriverLow();									// set all step pins low
			lastStepLowTime = StepTimer::GetTimerTicks();
#  endif
		}
		else
# endif
		{
# if USE_TC_FOR_STEP
			StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
			(void)dms[0].CalcNextStepTime(now);
# else
			StepDriverHigh();									// generate the step
			(void)dms[0].CalcNextStepTime(now);
			StepDriverLow();									// set the step pin low
# endif
		}

		++stepsDone[0];
		if (dms[0].directionChanged)
		{
			dms[0].directionChanged = false;
			SetDirection(dms[0].direction);
		}
	}
}

#else

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
void Move::StepDrivers(uint32_t now) noexcept
{
	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	while (dm != nullptr && (int32_t)(dm->nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)		// if the next step is due
	{
		driversStepping |= driveDriverBits[dm->drive];
		dm = dm->nextDM;
	}

# if SUPPORT_SLOW_DRIVERS
	if ((driversStepping & slowDriversBitmap != 0)					// if using any slow drivers
	{
		uint32_t lastStepPulseTime = lastStepLowTime;
		uint32_t rawNow;
		do
		{
			rawNow = StepTimer::GetTimerTicks();
		} while (rawNow - lastStepPulseTime < GetSlowDriverStepLowClocks() || rawNow - lastDirChangeTime < GetSlowDriverDirSetupClocks());
		StepDriversHigh(driversStepping);							// set the step pins high
		lastStepPulseTime = StepTimer::GetTimerTicks();

		PrepareForNextSteps(dm, now);

		while (StepTimer::GetTimerTicks() - lastStepPulseTime < GetSlowDriverStepHighClocks()) {}
		StepDriversLow();											// set all step pins low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}
	else
# endif
	{
		StepDriversHigh(driversStepping);							// set the step pins high
# if SAME70
		__DSB();													// without this the step pulse can be far too short
# endif
		PrepareForNextSteps(dm, now);
		StepDriversLow();											// set all step pins low
	}

	// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (dmToInsert->state >= DMState::firstMotionState)
		{
			if (dmToInsert->directionChanged)
			{
				dmToInsert->directionChanged = false;
				SetDirection(dmToInsert->drive, dmToInsert->direction);
			}
			InsertDM(dmToInsert);
		}
		dmToInsert = nextToInsert;
	}
}

#endif

// Prepare each DM that we generated a step for for the next step
void Move::PrepareForNextSteps(DriveMovement *stopDm, uint32_t now) noexcept
{
#if SINGLE_DRIVER
	if (unlikely(dms[0].state == DMState::starting))
	{
		if (dms[0].NewSegment(now) != nullptr && dms[0].state != DMState::starting)
		{
			dms[0].driversCurrentlyUsed = dms[0].driversNormallyUsed;	// we previously set driversCurrentlyUsed to 0 to avoid generating a step, so restore it now
			(void)dms[0].CalcNextStepTimeFull(now);					// calculate next step time
			dms[0].directionChanged = true;							// force the direction to be set up
		}
	}
	else
	{
		(void)dms[0].CalcNextStepTime(now);							// calculate next step time, which may change the required direction
	}
#else
	for (DriveMovement *dm2 = activeDMs; dm2 != stopDm; dm2 = dm2->nextDM)
	{
		if (unlikely(dm2->state == DMState::starting))
		{
			if (dm2->NewSegment(now) != nullptr && dm2->state != DMState::starting)
			{
				dm2->driversCurrentlyUsed = dm2->driversNormallyUsed;	// we previously set driversCurrentlyUsed to 0 to avoid generating a step, so restore it now
				(void)dm2->CalcNextStepTimeFull(now);					// calculate next step time
				dm2->directionChanged = true;							// force the direction to be set up
			}
		}
		else
		{
			(void)dm2->CalcNextStepTime(now);							// calculate next step time, which may change the required direction
		}
	}
#endif
}

// Stop some drivers and update the corresponding motor positions
void Move::StopDrivers(uint16_t whichDrives) noexcept
{
	DriversBitmap dr(whichDrives);
	dr.Iterate([this](size_t drive, unsigned int)
				{
					StopDriveFromRemote(drive);
				}
			  );
}

// Add some linear segments to be executed by a driver, taking account of possible input shaping. This is used by linear axes and by extruders.
// We never add a segment that starts earlier than any existing segments, but we may add segments when there are none already.
void Move::AddLinearSegments(size_t logicalDrive, uint32_t startTime, const PrepParams& params, motioncalc_t steps, MovementFlags moveFlags) noexcept
{
	DriveMovement* const dmp = &dms[logicalDrive];
	const motioncalc_t stepsPerMm = steps/(motioncalc_t)1.0;

	const uint32_t steadyStartTime = startTime + params.accelClocks;
	const uint32_t decelStartTime = steadyStartTime + params.steadyClocks;

	// Phases with zero duration will not get executed and may lead to infinities in the calculations. Avoid introducing them. Keep the total distance correct.
	const motioncalc_t accelDistance = (params.accelClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)params.accelDistance;
	const motioncalc_t decelDistance = (params.decelClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)(1.0 - params.decelStartDistance);
	const motioncalc_t steadyDistance = (params.steadyClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)1.0 - accelDistance - decelDistance;

	if (moveFlags.noShaping)
	{
		if (params.accelClocks != 0)
		{
			dmp->AddSegment(startTime, params.accelClocks, accelDistance * stepsPerMm, (motioncalc_t)params.acceleration * stepsPerMm, moveFlags);
		}
		if (params.steadyClocks != 0)
		{
			dmp->AddSegment(steadyStartTime, params.steadyClocks, steadyDistance * stepsPerMm, (motioncalc_t)0.0, moveFlags);
		}
		if (params.decelClocks != 0)
		{
			dmp->AddSegment(decelStartTime, params.decelClocks, decelDistance * stepsPerMm, -((motioncalc_t)params.deceleration * stepsPerMm), moveFlags);
		}
	}
	else
	{
		for (size_t index = 0; index < axisShaper.GetNumImpulses(); ++index)
		{
			const motioncalc_t factor = axisShaper.GetImpulseSize(index) * stepsPerMm;
			const uint32_t delay = axisShaper.GetImpulseDelay(index);
			if (params.accelClocks != 0)
			{
				dmp->AddSegment(startTime + delay, params.accelClocks, accelDistance * factor, (motioncalc_t)params.acceleration * factor, moveFlags);
			}
			if (params.steadyClocks != 0)
			{
				dmp->AddSegment(steadyStartTime + delay, params.steadyClocks, steadyDistance * factor, (motioncalc_t)0.0, moveFlags);
			}
			if (params.decelClocks != 0)
			{
				dmp->AddSegment(decelStartTime + delay, params.decelClocks, decelDistance * factor, -((motioncalc_t)params.deceleration * factor), moveFlags);
			}
		}
	}

	// If there were no segments attached to this DM initially, we need to schedule the interrupt for the new segment at the start of the list.
	// Don't do this until we have added all the segments for this move, because the first segment we added may have been modified and/or split when we added further segments to implement input shaping
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);					// shut out the step interrupt
	if (dmp->state == DMState::idle)
	{
		if (dmp->ScheduleFirstSegment())
		{
#if !SINGLE_DRIVER
			InsertDM(dmp);
			if (activeDMs == dmp)													// if this is now the first DM in the active list
#endif
			{
				if (ScheduleNextStepInterrupt())
				{
					Interrupt();
				}
			}
		}
	}
	RestoreBasePriority(oldPrio);
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
	if (dms[0].state >= DMState::firstMotionState)
#else
	if (activeDMs != nullptr)
#endif
	{
		uint32_t now = StepTimer::GetMovementTimerTicks();
		const uint32_t isrStartTime = now;
		for (;;)
		{
			// Generate steps for the current move segments
			StepDrivers(now);									// check endstops if necessary and step the drivers

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

float Move::DriveStepsPerUnit(size_t drive) const noexcept { return stepsPerMm[drive]; }

void Move::SetDriveStepsPerUnit(size_t drive, float val)
{
	if (drive < NumDrivers)
	{
		stepsPerMm[drive] = val;
	}
}

#if SUPPORT_SLOW_DRIVERS

static inline void UpdateTiming(uint32_t& timing, uint32_t clocks) noexcept
{
# if SINGLE_DRIVER
		timing = clocks;
# else
		if (clocks > timing)
		{
			timing = clocks;
		}
# endif
}

void Move::SetDriverStepTiming(size_t drive, const float timings[4]) noexcept
{
	bool isSlow = false;

# if USE_TC_FOR_STEP

	// Step high time - must do this one first because it affects the conversion of some of the others
	if (timings[0] > MinimumStepHighMicroseconds)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[0], MicrosecondsToStepTCClocks(timings[0]));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[0] = MicrosecondsToStepTCClocks(MinimumStepHighMicroseconds);
	}
#  endif

	// To get the new width to be applied to the step pulse, we need to update CCBUF[0] and then push it to CC[0]. Writing CC[0] directly doesn't work.
	StepGenTc->CCBUF[0].reg = (uint16_t)slowDriverStepTimingClocks[0];
	StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_UPDATE;

	// Step low time - must convert this to minimum period
	const float minimumPeriod = timings[1] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (minimumPeriod > 0.4)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[1], MicrosecondsToStepClocks(minimumPeriod));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[1] = 1;
	}
#  endif

	// Direction setup time - we can just convert this one
	if (timings[2] > 0.2)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[2], MicrosecondsToStepClocks(timings[2]));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[2] = 0;
	}
#  endif

	// Direction hold time - we need to convert hold time from trailing edge to hold time from leading edge
	const float holdTimeFromLeadingEdge = timings[3] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (holdTimeFromLeadingEdge > 0.4)
	{
		isSlow = true;
		const uint32_t clocks = MicrosecondsToStepClocks(holdTimeFromLeadingEdge);
		UpdateTiming(slowDriverStepTimingClocks[3], clocks);
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[3] = 1;
	}
#  endif

# else

	// Not using TC to generate step pulses
	for (size_t i = 0; i < 4; ++i)
	{
		if (timings[i] > 0.2)
		{
			isSlow = true;
			const uint32_t clocks = MicrosecondsToStepClocks(timings[i]);
			UpdateTiming(slowDriverStepTimingClocks[i], clocks);
		}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
		else
		{
			slowDriverStepTimingClocks[i] = 0;
		}
#  endif
	}

# endif	// USE_TC_FOR_STEP

# if SINGLE_DRIVER
	isSlowDriver = isSlow;
# else
	slowDriversBitmap.SetOrClearBit(drive, isSlow);
# endif
}

float Move::GetSlowDriverStepHighMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	return (float)slowDriverStepTimingClocks[0]/StepPulseClocksPerMicrosecond;
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[0]);
# endif
}

float Move::GetSlowDriverStepLowMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	const float period = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
	return period - GetSlowDriverStepHighMicroseconds();
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
# endif
}

float Move::GetSlowDriverDirSetupMicroseconds() const noexcept
{
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[2]);
}

float Move::GetSlowDriverDirHoldMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	const float dirHoldFromLeadingEdge = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
	return dirHoldFromLeadingEdge - GetSlowDriverStepHighMicroseconds();
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
# endif
}

#endif		// SUPPORT_SLOW_DRIVERS

#if HAS_SMART_DRIVERS

bool Move::SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept
{
	return SmartDrivers::SetMicrostepping(driver, microsteps, interpolate);
}

#endif

#endif	//SUPPORT_DRIVERS

// End
