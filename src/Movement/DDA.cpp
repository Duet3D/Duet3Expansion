/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"

#if SUPPORT_DRIVERS

#include <Platform/Platform.h>
#include "Move.h"
#include <CanMessageFormats.h>
#include <CAN/CanInterface.h>
#include <limits>

DDA::DDA(DDA* n) noexcept : next(n), prev(nullptr), state(empty)
{
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		endPoint[i] = 0;
	}
	flags.all = 0;						// in particular we need to set endCoordinatesValid and usePressureAdvance to false
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
pre(state == provisional || state == committed)
{
	switch (state)
	{
	case provisional:
		return clocksNeeded;
	case committed:
		{
			const int32_t timeExecuting = (int32_t)(StepTimer::GetMovementTimerTicks() - afterPrepare.moveStartTime);
			return (timeExecuting <= 0) ? clocksNeeded							// move has not started yet
					: ((uint32_t)timeExecuting > clocksNeeded) ? 0				// move has completed
						: clocksNeeded - (uint32_t)timeExecuting;				// move is part way through
		}
	default:
		return 0;
	}
}

#if !SINGLE_DRIVER

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr && (*dmp)->nextStepTime < dm->nextStepTime)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

// Remove this drive from the list of drives with steps due
// Called from the step ISR only.
void DDA::RemoveDM(size_t drive) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr)
	{
		DriveMovement * const dm = *dmp;
		if (dm->drive == drive)
		{
			(*dmp) = dm->nextDM;
			break;
		}
		dmp = &(dm->nextDM);
	}
}

#endif

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const noexcept
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), (double)vec[i]);
	}
	debugPrintf("]");
}

// Print the text followed by the DDA only
void DDA::DebugPrint() const noexcept
{
	debugPrintf("DDA: a=%e d=%e startv=%e topv=%e endv=%e cks=%" PRIu32 " fl=%u\n",
				(double)acceleration, (double)deceleration, (double)startSpeed, (double)topSpeed, (double)endSpeed, clocksNeeded, flags.all);
}

// This is called by Move to initialize all DDAs
void DDA::Init() noexcept
{
	state = empty;
	for (DriveMovement& ddm : ddms)
	{
		ddm.state = DMState::idle;
	}
}

// Set up a remote move. Return true if it represents real movement, else false.
// All values have already been converted to step clocks and the total distance has been normalised to 1.0.
// The whenToExecute field of the movement message has already bee converted to local time
// This version handles the new movement message that includes the input shaping plan and passes extruder movement as distance, not steps
bool DDA::Init(const CanMessageMovementLinearShaped& msg) noexcept
{
	afterPrepare.moveStartTime = msg.whenToExecute;
	flags.all = 0;
	flags.isPrintingMove = flags.usePressureAdvance = msg.usePressureAdvance;

	// Prepare for movement
	PrepParams params;

	// Normalise the move to unit distance
	params.acceleration = acceleration = msg.acceleration;
	params.deceleration = deceleration = msg.deceleration;
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;

	// We occasionally receive a message with zero clocks needed. This messes up the calculations, so add one steady clock in this case.
	if (clocksNeeded == 0)
	{
		clocksNeeded = params.steadyClocks = 1;
	}

	MovementFlags segFlags;
	segFlags.Clear();
	segFlags.nonPrintingMove = !msg.usePressureAdvance;
	segFlags.noShaping = !msg.useLateInputShaping;

	afterPrepare.drivesMoving.Clear();

	for (size_t drive = 0; drive < NumDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];						// the steps for this move will be added later
		if (drive >= msg.numDrivers)
		{
			directionVector[drive] = 0.0;
		}
		else if ((msg.extruderDrives & (1u << drive)) != 0)
		{
			// It's an extruder
			const float extrusionRequested = msg.perDrive[drive].extrusion;
			directionVector[drive] = extrusionRequested;
			if (extrusionRequested != 0.0)
			{
				move.AddLinearSegments(*this, drive, msg.whenToExecute, params, extrusionRequested, segFlags);
				//TODO will Move do the following?
				reprap.GetMove().EnableDrivers(drive, false);
			}
		}
		else
		{
			const float delta = (float)msg.perDrive[drive].steps;
			directionVector[drive] = delta;
			if (delta != 0.0)
			{
				move.AddLinearSegments(*this, drive, msg.whenToExecute, params, delta, segFlags);
				afterPrepare.drivesMoving.SetBit(drive);
				//TODO will Move do the following?
				reprap.GetMove().EnableDrivers(drive, false);
			}
		}
	}

	state = committed;												// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
}

#if USE_TC_FOR_STEP
uint32_t DDA::lastStepHighTime = 0;
#else
uint32_t DDA::lastStepLowTime = 0;
#endif
uint32_t DDA::lastDirChangeTime = 0;

#if SINGLE_DRIVER

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void DDA::StepDrivers(uint32_t now) noexcept
{
# if SUPPORT_CLOSED_LOOP
	if (ClosedLoop::GetClosedLoopInstance(0)->IsClosedLoopEnabled())
	{
		return;
	}
# endif

	// Determine whether the driver is due for stepping, overdue, or will be due very shortly
	if (ddms[0].state >= DMState::firstMotionState && (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval >= ddms[0].nextStepTime)	// if the next step is due
	{
		// Step the driver

# if SUPPORT_SLOW_DRIVERS
		if (Platform::IsSlowDriver())									// if using a slow driver
		{
#  if USE_TC_FOR_STEP
			const uint32_t lastStepPulseTime = lastStepHighTime;
			while (now - lastStepPulseTime < Platform::GetSlowDriverStepPeriodClocks() || now - lastDirChangeTime < Platform::GetSlowDriverDirSetupClocks())
			{
				now = StepTimer::GetTimerTicks();
			}
			StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
			lastStepHighTime = StepTimer::GetTimerTicks();
			(void)ddms[0].CalcNextStepTime(*this);
#  else
			uint32_t lastStepPulseTime = lastStepLowTime;
			while (now - lastStepPulseTime < Platform::GetSlowDriverStepLowClocks() || now - lastDirChangeTime < Platform::GetSlowDriverDirSetupClocks())
			{
				now = StepTimer::GetTimerTicks();
			}
			Platform::StepDriverHigh();									// generate the step
			lastStepPulseTime = StepTimer::GetTimerTicks();
			(void)ddms[0].CalcNextStepTime(*this);

			// 3a. Reset the step pin low
			while (StepTimer::GetTimerTicks() - lastStepPulseTime < Platform::GetSlowDriverStepHighClocks()) {}
			Platform::StepDriverLow();									// set all step pins low
			lastStepLowTime = StepTimer::GetTimerTicks();
#  endif
		}
		else
# endif
		{
# if USE_TC_FOR_STEP
			StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
			(void)ddms[0].CalcNextStepTime(*this);
# else
			Platform::StepDriverHigh();									// generate the step
			(void)ddms[0].CalcNextStepTime(*this);
			Platform::StepDriverLow();									// set the step pin low
# endif
		}

		++stepsDone[0];
		if (ddms[0].directionChanged)
		{
			ddms[0].directionChanged = false;
			Platform::SetDirection(ddms[0].direction);
		}
	}

	// If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (ddms[0].state < DMState::firstMotionState && StepTimer::GetTimerTicks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

#else

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
void DDA::StepDrivers(uint32_t now) noexcept
{
	// 1. There is no step 1.
	// 2. Determine which drivers are due for stepping, overdue, or will be due very shortly
	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	const uint32_t elapsedTime = (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval;
	while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
	{
		driversStepping |= Platform::GetDriversBitmap(dm->drive);
		++stepsDone[dm->drive];
		dm = dm->nextDM;
	}

# if SUPPORT_SLOW_DRIVERS
	if ((driversStepping & Platform::GetSlowDriversBitmap().GetRaw()) != 0)	// if using any slow drivers
	{
		uint32_t lastStepPulseTime = lastStepLowTime;
		while (now - lastStepPulseTime < Platform::GetSlowDriverStepLowClocks() || now - lastDirChangeTime < Platform::GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetTimerTicks();
		}
		Platform::StepDriversHigh(driversStepping);					// set the step pins high
		lastStepPulseTime = StepTimer::GetTimerTicks();

		for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			(void)dm2->CalcNextStepTime(*this);						// calculate next step times
		}

		while (StepTimer::GetTimerTicks() - lastStepPulseTime < Platform::GetSlowDriverStepHighClocks()) {}
		Platform::StepDriversLow();									// set all step pins low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}
	else
# endif
	{
		Platform::StepDriversHigh(driversStepping);					// set the step pins high
		for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			(void)dm2->CalcNextStepTime(*this);						// calculate next step times
		}
		Platform::StepDriversLow();									// set all step pins low
	}

	// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (dmToInsert->state >= DMState::firstMotionState)
		{
			InsertDM(dmToInsert);
			if (dmToInsert->directionChanged)
			{
				dmToInsert->directionChanged = false;
				Platform::SetDirection(dmToInsert->drive, dmToInsert->direction);
			}
		}
		dmToInsert = nextToInsert;
	}

	// 6. If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr && StepTimer::GetTimerTicks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

#endif

// Stop some drivers. Caller sets base priority is high enough to shut out step interrupts
void DDA::StopDrivers(uint16_t whichDrives) noexcept
{
	if (state == executing)
	{
#if SUPPORT_CLOSED_LOOP
		const uint32_t ticksSinceStart = StepTimer::GetTimerTicks() - afterPrepare.moveStartTime;
#endif
		for (size_t drive = 0; drive < NumDrivers; ++drive)
		{
			if (whichDrives & (1u << drive))
			{
				DriveMovement& dm = ddms[drive];
				if (dm.state >= DMState::firstMotionState)
				{
#if SUPPORT_CLOSED_LOOP
					MotionParameters mp;
					GetCurrentMotion(drive, ticksSinceStart, mp);
					directionVector[drive] = mp.position;				// adjust directionVector to be the amount actually moved so that it will be picked up when the move completes
#endif
					dm.state = DMState::idle;

#if SINGLE_DRIVER
					state = completed;
#else
					RemoveDM(drive);
					if (activeDMs == nullptr)
					{
						state = completed;
					}
#endif
				}
			}
		}
	}
}

uint32_t DDA::GetAndClearMaxTicksOverdue() noexcept
{
	const uint32_t ret = maxTicksOverdue;
	maxTicksOverdue =  0;
	return ret;
}

uint32_t DDA::GetAndClearMaxOverdueIncrement() noexcept
{
	const uint32_t ret = maxOverdueIncrement;
	maxOverdueIncrement =  0;
	return ret;
}

#endif	// SUPPORT_DRIVERS

// End
