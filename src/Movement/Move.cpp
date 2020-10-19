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
#include "StepTimer.h"
#include "Platform.h"
#include <CAN/CanInterface.h>
#include "CanMessageFormats.h"

Move::Move()
#if SUPPORT_DRIVERS
	: currentDda(nullptr), extrudersPrinting(false), scheduledMoves(0), completedMoves(0), numHiccups(0), active(false)
#endif
{
#if SUPPORT_DRIVERS
	kinematics = Kinematics::Create(KinematicsType::cartesian);			// default to Cartesian

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

	DriveMovement::InitialAllocate(NumDms);
	timer.SetCallback(Move::TimerCallback, static_cast<void*>(this));

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		extrusionAccumulators[i] = 0;
	}
#endif
}

void Move::Init()
{
#if SUPPORT_DRIVERS
	// Empty the ring
	ddaRingGetPointer = ddaRingCheckPointer = ddaRingAddPointer;
	DDA *dda = ddaRingAddPointer;
	do
	{
		dda->Init();
		dda = dda->GetNext();
	} while (dda != ddaRingAddPointer);

	currentDda = nullptr;
	stepErrors = 0;

	idleCount = 0;

	active = true;
#endif
}

void Move::Exit()
{
	StepTimer::DisableTimerInterrupt();

#if SUPPORT_DRIVERS
	// Clear the DDA ring so that we don't report any moves as pending
	currentDda = nullptr;
	while (ddaRingGetPointer != ddaRingAddPointer)
	{
		ddaRingGetPointer->Complete();
		ddaRingGetPointer = ddaRingGetPointer->GetNext();
	}

	while (ddaRingCheckPointer->GetState() == DDA::completed)
	{
		(void)ddaRingCheckPointer->Free();
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}
	active = false;												// don't accept any more moves
#endif
}

// Start the next move. Return true if laser or IO bits need to be active
// Must be called with base priority greater than or equal to the step interrupt, to avoid a race with the step ISR.
inline void Move::StartNextMove(DDA *cdda, uint32_t startTime)
pre(ddaRingGetPointer->GetState() == DDA::frozen)
{
	if (!cdda->IsPrintingMove())
	{
		extrudersPrinting = false;
	}
	else if (!extrudersPrinting)
	{
		extrudersPrinting = true;
		extrudersPrintingSince = millis();
	}
	currentDda = cdda;
	cdda->Start(startTime);
}

void Move::Spin()
{
#if SUPPORT_DRIVERS
	if (!active)
	{
		return;
	}

	if (idleCount < 1000)
	{
		++idleCount;
	}

	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	while (ddaRingCheckPointer->GetState() == DDA::completed)
	{
		// Check for step errors and record/print them if we have any, before we lose the DMs
		if (ddaRingCheckPointer->HasStepError())
		{
			if (Platform::Debug(moduleMove))
			{
				ddaRingCheckPointer->DebugPrintAll();
			}
			++stepErrors;
			Platform::LogError(ErrorCode::BadMove);
		}

		// Now release the DMs and check for underrun
		(void)ddaRingCheckPointer->Free();
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}

	// See if we can add another move to the ring
	bool canAddMove = (   ddaRingAddPointer->GetState() == DDA::empty
					   && ddaRingAddPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
					   && DriveMovement::NumFree() >= (int)NumDrivers						// check that we won't run out of DMs
					  );
	if (canAddMove)
	{
		// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
		// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is less than 0.5 seconds.
		const DDA *dda = ddaRingAddPointer;
		uint32_t unPreparedTime = 0;
		uint32_t prevMoveTime = 0;
		for (;;)
		{
			dda = dda->GetPrevious();
			if (dda->GetState() != DDA::provisional)
			{
				break;
			}
			unPreparedTime += prevMoveTime;
			prevMoveTime = dda->GetClocksNeeded();
		}

		canAddMove = (unPreparedTime < StepTimer::StepClockRate/2 || unPreparedTime + prevMoveTime < 2 * StepTimer::StepClockRate);
	}

	if (canAddMove)
	{
		// OK to add another move
		CanMessageMovement move;
		if (CanInterface::GetCanMove(move))
		{
			if (ddaRingAddPointer->Init(move))
			{
				ddaRingAddPointer = ddaRingAddPointer->GetNext();
				idleCount = 0;
				scheduledMoves++;
			}
		}
	}

	// See whether we need to kick off a move
	if (currentDda == nullptr)
	{
		// No DDA is executing, so start executing a new one if possible
		if (!canAddMove || idleCount > 10)							// better to have a few moves in the queue so that we can do lookahead
		{
			// Prepare one move and execute it. We assume that we will enter the next if-block before it completes, giving us time to prepare more moves.
			DDA * const cdda = ddaRingGetPointer;					// capture volatile variable
			if (cdda->GetState() == DDA::frozen)
			{
				AtomicCriticalSectionLocker();
				StartNextMove(cdda, StepTimer::GetTimerTicks());
				if (cdda->ScheduleNextStepInterrupt(timer))
				{
					Interrupt();
				}
			}
		}
	}
#endif
}

void Move::Diagnostics(const StringRef& reply)
{
#if SUPPORT_DRIVERS
	reply.catf("Moves scheduled %" PRIu32 ", completed %" PRIu32 ", in progress %d, hiccups %" PRIu32 "\n",
					scheduledMoves, completedMoves, (int)(currentDda != nullptr), numHiccups);
	numHiccups = 0;
	StepTimer::Diagnostics(reply);
#endif
}

#if SUPPORT_DRIVERS

# if 0
// Try to push some babystepping through the lookahead queue
float Move::PushBabyStepping(float amount)
{
	return ddaRingAddPointer->AdvanceBabyStepping(amount);
}
# endif

// Change the kinematics to the specified type if it isn't already
// If it is already correct leave its parameters alone.
// This violates our rule on no dynamic memory allocation after the initialisation phase,
// however this function is normally called only when M665, M667 and M669 commands in config.g are processed.
bool Move::SetKinematics(KinematicsType k)
{
	if (kinematics->GetKinematicsType() != k)
	{
		Kinematics * const nk = Kinematics::Create(k);
		if (nk == nullptr)
		{
			return false;
		}
		delete kinematics;
		kinematics = nk;
	}
	return true;
}

// Return true if this is a raw motor move
bool Move::IsRawMotorMove(uint8_t moveType) const
{
	return moveType == 2 /* || ((moveType == 1 || moveType == 3) && kinematics->GetHomingMode() != Kinematics::HomingMode::homeCartesianAxes)*/ ;
}

#if 0
// Pause the print as soon as we can, returning true if we are able to skip any moves and updating 'rp' to the first move we skipped.
bool Move::PausePrint(RestorePoint& rp)
{
	// Find a move we can pause after.
	// Ideally, we would adjust a move if necessary and possible so that we can pause after it, but for now we don't do that.
	// There are a few possibilities:
	// 1. There is no currently executing move and no moves in the queue, and GCodes does not have a move for us.
	//    Pause immediately. Resume from the current file position.
	// 2. There is no currently executing move and no moves in the queue, and GCodes has a move for us but that move has not been started.
	//    Pause immediately. Discard the move that GCodes gas for us, and resume from the start file position of that move.
	// 3. There is no currently executing move and no moves in the queue, and GCodes has a move for that has not been started.
	//    We must complete that move and then pause
	// 5. There is no currently-executing move but there are moves in the queue. Unlikely, but possible.
	//    If the first move in the queue is the first segment in its move, pause immediately, resume from its start address. Otherwise proceed as in case 5.
	// 4. There is a currently-executing move, possibly some moves in the queue, and GCodes may have a whole or partial move for us.
	//    See if we can pause after any of them and before the next. If we can, resume from the start position of the following move.
	//    If we can't, then the last move in the queue must be part of a multi-segment move and GCodes has the rest. We must finish that move and then pause.
	//
	// So on return we need to signal one of the following to GCodes:
	// 1. We have skipped some moves in the queue. Pass back the file address of the first move we have skipped, the feed rate at the start of that move
	//    and the iobits at the start of that move, and return true.
	// 2. All moves in the queue need to be executed. Also any move held by GCodes needs to be completed it is it not the first segment.
	//    Update the restore point with the coordinates and iobits as at the end of the previous move and return false.
	//    The extruder position, file position and feed rate are not filled in.
	//
	// In general, we can pause after a move if it is the last segment and its end speed is slow enough.
	// We can pause before a move if it is the first segment in that move.
	// The caller should set up rp.feedrate to the default feed rate for the file gcode source before calling this.

	const DDA * const savedDdaRingAddPointer = ddaRingAddPointer;
	bool pauseOkHere;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda == nullptr)
	{
		pauseOkHere = true;								// no move was executing, so we have already paused here whether it was a good idea or not.
		dda = ddaRingGetPointer;
	}
	else
	{
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	while (dda != savedDdaRingAddPointer)
	{
		if (pauseOkHere)
		{
			// We can pause before executing this move
			ddaRingAddPointer = dda;
			break;
		}
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	cpu_irq_enable();

	// We may be going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = ddaRingAddPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes hasn't changed between the moves

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	if (ddaRingAddPointer == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	dda = ddaRingAddPointer;
	rp.proportionDone = dda->GetProportionDone(false);	// get the proportion of the current multi-segment move that has been completed
	if (dda->UsingStandardFeedrate())
	{
		rp.feedRate = dda->GetRequestedSpeed();
	}
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();

	// Free the DDAs for the moves we are going to skip
	do
	{
		(void)dda->Free();
		dda = dda->GetNext();
		scheduledMoves--;
	}
	while (dda != savedDdaRingAddPointer);

	return true;
}
#endif

#if 0	//HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Pause the print immediately, returning true if we were able to skip or abort any moves and setting up to the move we aborted
bool Move::LowPowerOrStallPause(RestorePoint& rp)
{
	const DDA * const savedDdaRingAddPointer = ddaRingAddPointer;
	bool abortedMove = false;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda != nullptr && dda->GetFilePosition() != noFilePosition)
	{
		// We are executing a move that has a file address, so we can interrupt it
		Platform::DisableStepInterrupt();
		dda->MoveAborted();
		CurrentMoveCompleted();							// updates live endpoints, extrusion, ddaRingGetPointer, currentDda etc.
		--completedMoves;								// this move wasn't really completed
		abortedMove = true;
	}
	else
	{
		if (dda == nullptr)
		{
			// No move is being executed
			dda = ddaRingGetPointer;
		}
		while (dda != savedDdaRingAddPointer)
		{
			if (dda->GetFilePosition() != noFilePosition)
			{
				break;									// we can pause before executing this move
			}
			dda = dda->GetNext();
		}
	}

	cpu_irq_enable();

	if (dda == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	// We are going to skip some moves, or part of a move.
	// Store the parameters of the first move we are going to execute when we resume
	rp.feedRate = dda->GetRequestedSpeed();
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();
	rp.proportionDone = dda->GetProportionDone(abortedMove);	// store how much of the complete multi-segment move's extrusion has been done

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	ddaRingAddPointer = (abortedMove) ? dda->GetNext() : dda;

	// Get the end coordinates of the last move that was or will be completed, or the coordinates of the current move when we aborted it.
	DDA * const prevDda = ddaRingAddPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes and yAxes have't changed between the moves

	// Free the DDAs for the moves we are going to skip
	for (dda = ddaRingAddPointer; dda != savedDdaRingAddPointer; dda = dda->GetNext())
	{
		(void)dda->Free();
		scheduledMoves--;
	}

	return true;
}

#endif

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		extrusionAccumulators[driver] += currentDda->GetStepsTaken(driver);
	}
	currentDda = nullptr;
	ddaRingGetPointer = ddaRingGetPointer->GetNext();
	completedMoves++;
}

int32_t Move::GetPosition(size_t driver) const
{
	return ddaRingAddPointer->GetPrevious()->GetPosition(driver);
}

void Move::StopDrivers(uint16_t whichDrivers)
{
#if SAME5x
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);
#elif SAMC21
	const irqflags_t flags = cpu_irq_save();
#else
# error Unsupported processor
#endif
	DDA *cdda = currentDda;				// capture volatile
	if (cdda != nullptr)
	{
		cdda->StopDrivers(whichDrivers);
		if (cdda->GetState() == DDA::completed)
		{
			CurrentMoveCompleted();					// tell the DDA ring that the current move is complete
		}
	}
#if SAME5x
	RestoreBasePriority(oldPrio);
#elif SAMC21
	cpu_irq_restore(flags);
#else
# error Unsupported processor
#endif
}

// Filament monitor support
// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and isPrinting is true unless we are currently executing an extruding but non-printing move
int32_t Move::GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept
{
	AtomicCriticalSectionLocker lock;
	const int32_t ret = extrusionAccumulators[driver];
	const DDA * const cdda = currentDda;						// capture volatile variable
	const int32_t adjustment = (cdda == nullptr) ? 0 : cdda->GetStepsTaken(driver);
	extrusionAccumulators[driver] = -adjustment;
	isPrinting = extrudersPrinting;
	return ret + adjustment;
}

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrintAll();
	}
}

// This is the function that is called by the timer interrupt to step the motors.
// This may occasionally get called prematurely.
void Move::Interrupt()
{
	const uint32_t isrStartTime = StepTimer::GetTimerTicks();
	uint32_t now = isrStartTime;
	for (;;)
	{
		// Generate a step for the current move
		DDA* cdda = currentDda;										// capture volatile variable
		if (cdda == nullptr)
		{
			return;													// no current  move, so no steps needed
		}

		cdda->StepDrivers(now);
		if (cdda->GetState() == DDA::completed)
		{
			const uint32_t finishTime = cdda->GetMoveFinishTime();	// calculate when this move should finish
			CurrentMoveCompleted();									// tell the DDA ring that the current move is complete and set currentDda to nullptr

			// Start the next move, if one is ready
			cdda = ddaRingGetPointer;
			if (cdda->GetState() != DDA::frozen)
			{
				return;
			}

			StartNextMove(cdda, finishTime);
		}

		// Schedule a callback at the time when the next step is due, and quit unless it is due immediately
		if (!cdda->ScheduleNextStepInterrupt(timer))
		{
			return;
		}

		// The next step is due immediately. Check whether we have been in this ISR for too long already and need to take a break
		now = StepTimer::GetTimerTicks();
		if (now - isrStartTime >= DDA::MaxStepInterruptTime)
		{
			// Force a break by updating the move start time.
			// If the inserted hiccup is too short then it won't help. So we double the hiccup time on each iteration.
			++numHiccups;
			cdda->InsertHiccup(now);

			// Reschedule the next step interrupt. This time it should succeed if the hiccup time was long enough.
			if (!cdda->ScheduleNextStepInterrupt(timer))
			{
				return;
			}
		}
	}
}

#endif	//SUPPORT_DRIVERS

// End
