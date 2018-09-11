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
#include "GCodes/GCodes.h"
//#include "Tools/Tool.h"

constexpr uint32_t UsualMinimumPreparedTime = StepTimer::StepClockRate/10;			// 100ms
constexpr uint32_t AbsoluteMinimumPreparedTime = StepTimer::StepClockRate/20;		// 50ms

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
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;
	maxPrintingAcceleration = maxTravelAcceleration = 10000.0;
	drcEnabled = false;											// disable dynamic ringing cancellation
	drcMinimumAcceleration = 10.0;

	// Put the origin on the lookahead ring with default velocity in the previous position to the first one that will be used.
	// Do this by calling SetLiveCoordinates and SetPositions, so that the motor coordinates will be correct too even on a delta.
	{
		float move[DRIVES];
		for (size_t i = 0; i < DRIVES; i++)
		{
			move[i] = 0.0;
			liveEndPoints[i] = 0;								// not actually right for a delta, but better than printing random values in response to M114
		}
		SetLiveCoordinates(move);
		SetPositions(move);
	}

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionAccumulators[i] = 0;
		extruderNonPrinting[i] = false;
		extrusionPending[i] = 0.0;
	}

	usingMesh = false;
	useTaper = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	lastStateChangeTime = millis();
	idleCount = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	longestGcodeWaitInterval = 0;
	specialMoveAvailable = false;

	StepTimer::Init();

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
		GCodes::RawMove nextMove;
		(void) GCodes::ReadMove(nextMove);			// throw away any move that GCodes tries to pass us
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
					   && DriveMovement::NumFree() >= (int)DRIVES							// check that we won't run out of DMs
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
		if (specialMoveAvailable)
		{
			if (simulationMode < 2)
			{
				if (ddaRingAddPointer->Init(specialMoveCoords))
				{
					ddaRingAddPointer = ddaRingAddPointer->GetNext();
					if (moveState == MoveState::idle || moveState == MoveState::timing)
					{
						// We were previously idle, so we have a state change
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
			specialMoveAvailable = false;
		}
		else
		{
			// If there's a G Code move available, add it to the DDA ring for processing.
			GCodes::RawMove nextMove;
			if (GCodes::ReadMove(nextMove))		// if we have a new move
			{
				if (simulationMode < 2)		// in simulation mode 2 and higher, we don't process incoming moves beyond this point
				{
#if 0	// disabled this because it causes jerky movements on the SCARA printer
					// Add on the extrusion left over from last time.
					const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
					for (size_t drive = numAxes; drive < DRIVES; ++drive)
					{
						nextMove.coords[drive] += extrusionPending[drive - numAxes];
					}
#endif
#if 0
					if (nextMove.moveType == 0)
					{
						AxisAndBedTransform(nextMove.coords, nextMove.xAxes, nextMove.yAxes, true);
					}
#endif
					if (ddaRingAddPointer->Init(nextMove, true))
					{
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

#if 0	// see above
					// Save the amount of extrusion not done
					for (size_t drive = numAxes; drive < DRIVES; ++drive)
					{
						extrusionPending[drive - numAxes] = nextMove.coords[drive];
					}
#endif
				}
			}
		}
	}

	// If we are simulating, simulate completion of the current move.
	// Do this here rather than at the end, so that when simulating, currentDda is non-null for most of the time and IsExtruding() returns the correct value
	{
		DDA *cdda;													// currentDda is declared volatile, so copy it in the next line
		if (simulationMode != 0 && (cdda = currentDda) != nullptr)
		{
			simulationTime += (float)cdda->GetClocksNeeded()/StepTimer::StepClockRate;
			cdda->Complete();
			CurrentMoveCompleted();
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
			if (dda->GetState() == DDA::provisional)
			{
				dda->Prepare(simulationMode);
			}
			if (dda->GetState() == DDA::frozen)
			{
				if (simulationMode != 0)
				{
					currentDda = dda;								// pretend we are executing this move
				}
				else
				{
					if (StartNextMove(StepTimer::GetInterruptClocks()))	// start the next move
					{
						Interrupt();
					}
				}
				moveState = MoveState::executing;
			}
			else if (simulationMode == 0)
			{
				if (moveState == MoveState::executing && GCodes::IsPaused())
				{
					lastStateChangeTime = millis();					// record when we first noticed that the machine was idle
					moveState = MoveState::timing;
				}
				else if (moveState == MoveState::timing && millis() - lastStateChangeTime >= idleTimeout)
				{
					Platform::SetDriversIdle();			// put all drives in idle hold
					moveState = MoveState::idle;
				}
			}
		}
	}

	DDA *cdda = currentDda;											// currentDda is volatile, so copy it
	if (cdda != nullptr)
	{
		// See whether we need to prepare any moves. First count how many prepared or executing moves we have and how long they will take.
		int32_t preparedTime = 0;
		uint32_t preparedCount = 0;
		DDA::DDAState st;
		while ((st = cdda->GetState()) == DDA::completed || st == DDA::executing || st == DDA::frozen)
		{
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
			if (cdda == ddaRingAddPointer)
			{
				break;
			}
		}

		// If the number of prepared moves will execute in less than the minimum time, prepare another move.
		// Try to avoid preparing deceleration-only moves
		while (st == DDA::provisional
				&& preparedTime < (int32_t)UsualMinimumPreparedTime		// prepare moves one eighth of a second ahead of when they will be needed
				&& preparedCount < DdaRingLength/2 - 1					// but don't prepare as much as half the ring
			  )
		{
			if (cdda->IsGoodToPrepare() || preparedTime < (int32_t)AbsoluteMinimumPreparedTime)
			{
				cdda->Prepare(simulationMode);
			}
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
			st = cdda->GetState();
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
		Kinematics *nk = Kinematics::Create(k);
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
	Platform::Message(mtype, "=== Move ===\n");
	Platform::MessageF(mtype, "Hiccups: %u, StepErrors: %u, LaErrors: %u, FreeDm: %d, MinFreeDm: %d, MaxWait: %" PRIu32 "ms, Underruns: %u, %u\n",
						DDA::numHiccups, stepErrors, numLookaheadErrors, DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	DDA::numHiccups = 0;
	stepErrors = 0;
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();

	Platform::MessageF(mtype, "Scheduled moves: %" PRIu32 ", completed moves: %" PRIu32 "\n", scheduledMoves, completedMoves);
}

// Set the current position to be this
void Move::SetNewPosition(const float positionNow[DRIVES], bool doBedCompensation)
{
	float newPos[DRIVES];
	memcpy(newPos, positionNow, sizeof(newPos));			// copy to local storage because Transform modifies it
	SetLiveCoordinates(newPos);
	SetPositions(newPos);
}

// These are the actual numbers we want in the positions, so don't transform them.
void Move::SetPositions(const float move[DRIVES])
{
	if (DDARingEmpty())
	{
		ddaRingAddPointer->GetPrevious()->SetPositions(move, DRIVES);
	}
	else
	{
		Platform::Message(ErrorMessage, "SetPositions called when DDA ring not empty\n");
	}
}

void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	if (CartesianToMotorSteps(coords, ep, true))
	{
		const size_t numAxes = GCodes::GetTotalAxes();
		for (size_t drive = numAxes; drive < numDrives; ++drive)
		{
			ep[drive] = MotorEndPointToMachine(drive, coords[drive]);
		}
	}
}

// Convert distance to steps for a particular drive
int32_t Move::MotorEndPointToMachine(size_t drive, float coord)
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
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, Platform::GetDriveStepsPerUnit(),
														GCodes::GetVisibleAxes(), GCodes::GetTotalAxes(), motorPos, isCoordinated);
	if (Platform::Debug(moduleMove) && Platform::Debug(moduleDda))
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

float Move::GetTopSpeed() const
{
	const DDA * const currDda = currentDda;
	return (currDda == nullptr) ? 0.0 : currDda->GetTopSpeed();
}

float Move::GetRequestedSpeed() const
{
	const DDA * const currDda = currentDda;
	return (currDda == nullptr) ? 0.0 : currDda->GetRequestedSpeed();
}

// Perform motor endpoint adjustment
void Move::AdjustMotorPositions(const float_t adjustment[], size_t numMotors)
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const int32_t * const endCoordinates = lastQueuedMove->DriveCoordinates();
	const float * const driveStepsPerUnit = Platform::GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < numMotors; ++drive)
	{
		const int32_t ep = endCoordinates[drive] + lrintf(adjustment[drive] * driveStepsPerUnit[drive]);
		lastQueuedMove->SetDriveCoordinate(ep, drive);
		liveEndPoints[drive] = ep;
	}

	liveCoordinatesValid = false;		// force the live XYZ position to be recalculated
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));
	const size_t numAxes = GCodes::GetTotalAxes();
	for (size_t drive = numAxes; drive < DRIVES; ++drive)
	{
		extrusionAccumulators[drive - numAxes] += currentDda->GetStepsTaken(drive);
		if (currentDda->IsNonPrintingExtruderMove(drive))
		{
			extruderNonPrinting[drive - numAxes] = true;
		}
	}
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

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const size_t numAxes = GCodes::GetVisibleAxes();
	for (size_t i = 0; i < MaxAxes; i++)
	{
		if (i < numAxes)
		{
			m[i] = lastQueuedMove->GetEndCoordinate(i, disableMotorMapping);
		}
		else
		{
			m[i] = 0.0;
		}
	}
}

/*static*/ float Move::MotorEndpointToPosition(int32_t endpoint, size_t drive)
{
	return ((float)(endpoint))/Platform::DriveStepsPerUnit(drive);
}

// Is filament being extruded?
bool Move::IsExtruding() const
{
	AtomicCriticalSectionLocker lock;

	return currentDda != nullptr && currentDda->IsPrintingMove();
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, AxesBitmap xAxes, AxesBitmap yAxes) const
{
	GetCurrentMachinePosition(m, IsRawMotorMove(moveType));
}

#if 0
// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry
void Move::LiveCoordinates(float m[DRIVES], AxesBitmap xAxes, AxesBitmap yAxes)
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();		// do this before we disable interrupts
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();			// do this before we disable interrupts
	cpu_irq_disable();
	if (liveCoordinatesValid)
	{
		// All coordinates are valid, so copy them across
		memcpy(m, const_cast<const float *>(liveCoordinates), sizeof(m[0]) * DRIVES);
		cpu_irq_enable();
	}
	else
	{
		// Only the extruder coordinates are valid, so we need to convert the motor endpoints to coordinates
		memcpy(m + numTotalAxes, const_cast<const float *>(liveCoordinates + numTotalAxes), sizeof(m[0]) * (DRIVES - numTotalAxes));
		int32_t tempEndPoints[MaxAxes];
		memcpy(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints));
		cpu_irq_enable();

		MotorStepsToCartesian(tempEndPoints, numVisibleAxes, numTotalAxes, m);		// this is slow, so do it with interrupts enabled

		// If the ISR has not updated the endpoints, store the live coordinates back so that we don't need to do it again
		cpu_irq_disable();
		if (memcmp(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints)) == 0)
		{
			memcpy(const_cast<float *>(liveCoordinates), m, sizeof(m[0]) * numVisibleAxes);
			liveCoordinatesValid = true;
		}
		cpu_irq_enable();
	}
}
#endif

// These are the actual numbers that we want to be the coordinates, so don't transform them.
// The caller must make sure that no moves are in progress or pending when calling this
void Move::SetLiveCoordinates(const float coords[DRIVES])
{
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		liveCoordinates[drive] = coords[drive];
	}
	liveCoordinatesValid = true;
	EndPointToMachine(coords, const_cast<int32_t *>(liveEndPoints), GCodes::GetVisibleAxes());
}

void Move::ResetExtruderPositions()
{
	AtomicCriticalSectionLocker lock;

	const size_t totalAxes = GCodes::GetTotalAxes();
	for (size_t eDrive = totalAxes; eDrive < DRIVES; eDrive++)
	{
		liveCoordinates[eDrive] = 0.0;
	}
}

#if 0
// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and nonPrinting is true if we are currently executing an extruding but non-printing move
// or we completed one since the last call.
int32_t Move::GetAccumulatedExtrusion(size_t extruder, bool& nonPrinting)
{
	const size_t drive = extruder + reprap.GetGCodes().GetTotalAxes();
	if (drive < DRIVES)
	{
		const irqflags_t flags = cpu_irq_save();
		const int32_t ret = extrusionAccumulators[extruder];
		const DDA * const cdda = currentDda;						// capture volatile variable
		const int32_t adjustment = (cdda == nullptr) ? 0 : cdda->GetStepsTaken(drive);
		if (adjustment == 0)
		{
			// Either there is no current move, or we are at the very start of this move, or it doesn't involve this extruder e.g. a travel move
			nonPrinting = extruderNonPrinting[extruder];
		}
		else if (cdda->IsPrintingMove())
		{
			nonPrinting = extruderNonPrinting[extruder];
			extruderNonPrinting[extruder] = false;
		}
		else
		{
			nonPrinting = true;
		}
		extrusionAccumulators[extruder] = -adjustment;
		cpu_irq_restore(flags);
		return ret + adjustment;
	}

	nonPrinting = true;
	return 0.0;
}
#endif

// Enter or leave simulation mode
void Move::Simulate(uint8_t simMode)
{
	simulationMode = simMode;
	if (simMode != 0)
	{
		simulationTime = 0.0;
	}
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

#if 0
// Process M204
GCodeResult Move::ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('S'))
	{
		// For backwards compatibility with old versions of Marlin (e.g. for Cura and the Prusa fork of slic3r), set both accelerations
		seen = true;
		maxTravelAcceleration = maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('P'))
	{
		seen = true;
		maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('T'))
	{
		seen = true;
		maxTravelAcceleration = gb.GetFValue();
	}
	if (!seen)
	{
		reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f", (double)maxPrintingAcceleration, (double)maxTravelAcceleration);
	}
	return GCodeResult::ok;
}

// Process M593
GCodeResult Move::ConfigureDynamicAcceleration(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('F'))
	{
		seen = true;
		const float f = gb.GetFValue();
		if (f >= 4.0 && f <= 10000.0)
		{
			drcPeriod = 1.0/f;
			drcEnabled = true;
		}
		else
		{
			drcEnabled = false;
		}
	}
	if (gb.Seen('L'))
	{
		seen = true;
		drcMinimumAcceleration = max<float>(gb.GetFValue(), 1.0);		// very low accelerations cause problems with the maths
	}

	if (!seen)
	{
		if (reprap.GetMove().IsDRCenabled())
		{
			reply.printf("Dynamic ringing cancellation at %.1fHz, min. acceleration %.1f", (double)(1.0/drcPeriod), (double)drcMinimumAcceleration);
		}
		else
		{
			reply.copy("Dynamic ringing cancellation is disabled");
		}
	}
	return GCodeResult::ok;
}
#endif

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrintAll();
	}
}

// End
