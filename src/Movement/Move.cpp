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
#include "CAN/CanSlaveInterface.h"
#include "Hardware/Interrupts.h"
#include "CanMessageFormats.h"

Move::Move() : currentDda(nullptr), active(false), scheduledMoves(0), completedMoves(0)
{
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
}

void Move::Init()
{
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
	numLookaheadUnderruns = numPrepareUnderruns = 0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	lastStateChangeTime = millis();
	idleCount = 0;

	longestGcodeWaitInterval = 0;
//	numHiccups = 0;

	active = true;
}

void Move::Exit()
{
	StepTimer::DisableStepInterrupt();

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
}

void Move::Spin()
{
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
		if (ddaRingCheckPointer->Free())
		{
			++numLookaheadUnderruns;
		}
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}

	// See if we can add another move to the ring
	bool canAddMove = (   ddaRingAddPointer->GetState() == DDA::empty
					   && ddaRingAddPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
					   && DriveMovement::NumFree() >= (int)NumDrivers							// check that we won't run out of DMs
					  );
	if (canAddMove)
	{
		// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
		// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is less than 0.5 seconds.
		const DDA *dda = ddaRingAddPointer;
		uint32_t unPreparedTime = 0;
		uint32_t prevMoveTime = 0;
		for(;;)
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
		// OK to add another move. First check if a special move is available.
		CanMessageMovement move;
		if (CanSlaveInterface::GetCanMove(move))
		{
			ddaRingAddPointer->Init(move);
			ddaRingAddPointer = ddaRingAddPointer->GetNext();
			idleCount = 0;
			scheduledMoves++;
			if (moveState == MoveState::idle || moveState == MoveState::timing)
			{
				moveState = MoveState::collecting;
				const uint32_t now = millis();
				const uint32_t timeWaiting = now - lastStateChangeTime;
				if (timeWaiting > longestGcodeWaitInterval)
				{
					longestGcodeWaitInterval = timeWaiting;
				}
				lastStateChangeTime = now;
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
			StepTimer::DisableStepInterrupt();						// should be disabled already because we weren't executing a move, but make sure
			DDA * const dda = ddaRingGetPointer;					// capture volatile variable
			if (dda->GetState() == DDA::frozen)
			{
				if (StartNextMove(StepTimer::GetInterruptClocks()))	// start the next move
				{
					Interrupt();
				}
				moveState = MoveState::executing;
			}
			else
			{
				if (moveState == MoveState::executing && GCodes::IsPaused())
				{
					lastStateChangeTime = millis();					// record when we first noticed that the machine was idle
					moveState = MoveState::timing;
				}
				else if (moveState == MoveState::timing && millis() - lastStateChangeTime >= idleTimeout)
				{
					Platform::SetDriversIdle();						// put all drives in idle hold
					moveState = MoveState::idle;
				}
			}
		}
	}
}

#if 0
// Try to push some babystepping through the lookahead queue
float Move::PushBabyStepping(float amount)
{
	return ddaRingAddPointer->AdvanceBabyStepping(amount);
}
#endif

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

void Move::Diagnostics(MessageType mtype)
{
#if 0
	Platform::Message(mtype, "=== Move ===\n");
	Platform::MessageF(mtype, "Hiccups: %u, StepErrors: %u, LaErrors: %u, FreeDm: %d, MinFreeDm: %d, MaxWait: %" PRIu32 "ms, Underruns: %u, %u\n",
						DDA::numHiccups, stepErrors, numLookaheadErrors, DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	DDA::numHiccups = 0;
	stepErrors = 0;
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();
#endif
	Platform::MessageF(mtype, "Scheduled moves: %" PRIu32 ", completed moves: %" PRIu32 /*"\n"*/, scheduledMoves, completedMoves);
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered and DDA::SetPositions
void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	if (CartesianToMotorSteps(coords, ep, true))
	{
		const size_t numAxes = GCodes::GetTotalAxes();
		for (size_t drive = numAxes; drive < numDrives; ++drive)
		{
			ep[drive] = MotorMovementToSteps(drive, coords[drive]);
		}
	}
}

// Convert distance to steps for a particular drive
int32_t Move::MotorMovementToSteps(size_t drive, float coord)
{
	return lrintf(coord * Platform::DriveStepsPerUnit(drive));
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// This is computationally expensive on a delta or SCARA machine, so only call it when necessary, and never from the step ISR.
void Move::MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	kinematics->MotorStepsToCartesian(motorPos, Platform::GetDriveStepsPerUnit(), numVisibleAxes, numTotalAxes, machinePos);
}

// Convert Cartesian coordinates to motor steps, axes only, returning true if successful.
// Used to perform movement and G92 commands.
// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered, DDA::SetPositions and Move::EndPointToMachine
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, Platform::GetDriveStepsPerUnit(),
														GCodes::GetVisibleAxes(), GCodes::GetTotalAxes(), motorPos, isCoordinated);
	if (!inInterrupt() && Platform::Debug(moduleMove) && Platform::Debug(moduleDda))
	{
		if (b)
		{
			debugPrintf("Transformed %.2f %.2f %.2f to %" PRIu32 " %" PRIu32 " %" PRIu32 "\n", (double)machinePos[0], (double)machinePos[1], (double)machinePos[2], motorPos[0], motorPos[1], motorPos[2]);
		}
		else
		{
			debugPrintf("Unable to transform %.2f %.2f %.2f\n", (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
		}
	}
	return b;
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	currentDda = nullptr;
	ddaRingGetPointer = ddaRingGetPointer->GetNext();
	completedMoves++;
}

// Try to start another move. Must be called with interrupts disabled, to avoid a race condition.
bool Move::TryStartNextMove(uint32_t startTime)
{
	const DDA::DDAState st = ddaRingGetPointer->GetState();
	if (st == DDA::frozen)
	{
		return StartNextMove(startTime);
	}
	else
	{
		if (st == DDA::provisional)
		{
			// There are more moves available, but they are not prepared yet. Signal an underrun.
			++numPrepareUnderruns;
		}
#if 0
		reprap.GetPlatform().ExtrudeOff();			// turn off ancillary PWM
#endif
		return false;
	}
}

/*static*/ float Move::MotorStepsToMovement(size_t drive, int32_t endpoint)
{
	return ((float)(endpoint))/Platform::DriveStepsPerUnit(drive);
}

// Return the idle timeout in seconds
float Move::IdleTimeout() const
{
	return (float)idleTimeout * 0.001;
}

// Set the idle timeout in seconds
void Move::SetIdleTimeout(float timeout)
{
	idleTimeout = (uint32_t)lrintf(timeout * 1000.0);
}

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrintAll();
	}
}

// End
