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
#include "Kinematics/LinearDeltaKinematics.h"		// for DELTA_AXES
#include <CanMessageFormats.h>
#include <CAN/CanInterface.h>
#include <limits>

#ifdef DUET_NG
# define DDA_MOVE_DEBUG	(0)
#else
// On the wired Duets we don't have enough RAM to support this
# define DDA_MOVE_DEBUG	(0)
#endif

#if DDA_MOVE_DEBUG

// Structure to hold the essential parameters of a move, for debugging
struct MoveParameters
{
	float accelDistance;
	float steadyDistance;
	float decelDistance;
	float requestedSpeed;
	float startSpeed;
	float topSpeed;
	float endSpeed;
	float targetNextSpeed;
	uint32_t endstopChecks;
	uint16_t flags;

	MoveParameters() noexcept
	{
		accelDistance = steadyDistance = decelDistance = requestedSpeed = startSpeed = topSpeed = endSpeed = targetNextSpeed = 0.0;
		endstopChecks = 0;
		flags = 0;
	}

	void DebugPrint() const noexcept
	{
		Platform::MessageF(DebugMessage, "%f,%f,%f,%f,%f,%f,%f,%f,%08" PRIX32 ",%04x\n",
								(double)accelDistance, (double)steadyDistance, (double)decelDistance, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed,
								(double)targetNextSpeed, endstopChecks, flags);
	}

	static void PrintHeading() noexcept
	{
		Platform::Message(DebugMessage,
									"accelDistance,steadyDistance,decelDistance,requestedSpeed,startSpeed,topSpeed,endSpeed,"
									"targetNextSpeed,endstopChecks,flags\n");
	}
};

const size_t NumSavedMoves = 128;

static MoveParameters savedMoves[NumSavedMoves];
static size_t savedMovePointer = 0;

// Print the saved moves in CSV format for analysis
/*static*/ void DDA::PrintMoves() noexcept
{
	// Print the saved moved in CSV format
	MoveParameters::PrintHeading();
	for (size_t i = 0; i < NumSavedMoves; ++i)
	{
		savedMoves[savedMovePointer].DebugPrint();
		savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
	}
}

#else

/*static*/ void DDA::PrintMoves() noexcept { }

#endif

unsigned int DDA::stepErrors = 0;
uint32_t DDA::maxTicksOverdue = 0;
uint32_t DDA::maxOverdueIncrement = 0;

uint32_t DDA::stepsRequested[NumDrivers];
uint32_t DDA::stepsDone[NumDrivers];

DDA::DDA(DDA* n) noexcept : next(n), prev(nullptr), state(empty)
{
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		endPoint[i] = 0;
		ddms[i].state = DMState::idle;
		ddms[i].drive = i;
	}
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
pre(state == executing || state == frozen || state == completed) noexcept
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(afterPrepare.moveStartTime + clocksNeeded - StepTimer::GetTimerTicks())
			: (int32_t)clocksNeeded;
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

// Print the DDA and active DMs
void DDA::DebugPrintAll() const noexcept
{
	DebugPrint();
	for (size_t axis = 0; axis < NumDrivers; ++axis)
	{
		ddms[axis].DebugPrint();
	}
}

// Set up the segments (without input shaping) if we haven't done so already
void DDA::EnsureSegments(const PrepParams& params) noexcept
{
	if (segments == nullptr)
	{
		segments = AxisShaper::GetUnshapedSegments(*this, params);
	}
}

void DDA::ReleaseSegments() noexcept
{
	for (MoveSegment* seg = segments; seg != nullptr; )
	{
		MoveSegment* const nextSeg = seg->GetNext();
		MoveSegment::Release(seg);
		seg = nextSeg;
	}
	segments = nullptr;
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
// This one handles the old format movement message, used by older versions of RRF and the ATE
// The whenToExecute field of the movement message has already bee converted to local time
bool DDA::Init(const CanMessageMovementLinear& msg) noexcept
{
	afterPrepare.moveStartTime = msg.whenToExecute;
	flags.all = 0;
	flags.isPrintingMove = flags.usePressureAdvance = (msg.pressureAdvanceDrives != 0);
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;

	// Calculate the speeds and accelerations assuming unit movement length
	topSpeed = 2.0/(2 * msg.steadyClocks + (msg.initialSpeedFraction + 1.0) * msg.accelerationClocks + (msg.finalSpeedFraction + 1.0) * msg.decelClocks);
	startSpeed = topSpeed * msg.initialSpeedFraction;
	endSpeed = topSpeed * msg.finalSpeedFraction;

	// Prepare for movement
	PrepParams params;											// the default constructor clears params.plan to 'no shaping'

	// Set up the move parameters
	// Calculate the distances as a fraction of the total movement length
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;
	params.acceleration = acceleration = (msg.accelerationClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.initialSpeedFraction))/msg.accelerationClocks;
	params.deceleration = deceleration = (msg.decelClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.finalSpeedFraction))/msg.decelClocks;
	params.accelDistance = (msg.accelerationClocks == 0) ? 0.0
						: (msg.accelerationClocks == clocksNeeded) ? 1.0
							: topSpeed * (1.0 + msg.initialSpeedFraction) * msg.accelerationClocks * 0.5;
	const float decelDistance = (msg.decelClocks == 0) ? 0.0
						: (msg.decelClocks == clocksNeeded) ? 1.0
							: topSpeed * (1.0 + msg.finalSpeedFraction) * msg.decelClocks * 0.5;
	params.decelStartDistance = 1.0 - decelDistance;

	segments = nullptr;
#if !SINGLE_DRIVER
	activeDMs = nullptr;
#endif

	bool realMove = false;
	for (size_t drive = 0; drive < NumDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];		// the steps for this move will be added later
		DriveMovement& dm = ddms[drive];

#if !SINGLE_DRIVER
		dm.nextDM = nullptr;
#endif
		const int32_t delta = (drive < msg.numDrivers) ? msg.perDrive[drive].steps : 0;
		directionVector[drive] = (float)delta;
		bool stepsToDo = false;
		if (delta != 0)
		{
			realMove = true;
			EnsureSegments(params);						// we are probably going to need segments
			dm.totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			dm.direction = (delta >= 0);				// for now this is the direction of net movement, but it gets adjusted later if it is a delta movement
			stepsRequested[drive] += labs(delta);
			Platform::EnableDrive(drive, 0);
			if ((msg.pressureAdvanceDrives & (1u << drive)) != 0)
			{
				dm.PrepareExtruder(*this, (float)delta);
				realMove = true;
				stepsToDo = true;
			}
			else
			{
				stepsToDo = dm.PrepareCartesianAxis(*this);
				if (stepsToDo)
				{
					realMove = true;
#if !SINGLE_DRIVER
					InsertDM(&dm);
#endif
					const int32_t netSteps = (dm.reverseStartStep < dm.totalSteps) ? (2 * dm.reverseStartStep) - dm.totalSteps : dm.totalSteps;
					if (dm.direction)
					{
						endPoint[drive] += netSteps;
					}
					else
					{
						endPoint[drive] -= netSteps;
					}
				}
			}
		}
		if (!stepsToDo)
		{
			dm.state = DMState::idle;
			dm.currentSegment = nullptr;
			// Set up the steps so that GetStepsTaken will return zero
			dm.totalSteps = 0;
			dm.nextStep = 0;
			dm.reverseStartStep = 1;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		ReleaseSegments();			// we may have set up the segments, in which case we must recycle them
		return false;
	}

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
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
	params.shapingPlan.condensedPlan = msg.shapingPlan;

	// Normalise the move to unit distance
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;

	// We occasionally receive a message with zero clocks needed. This messes up the calculations, so add one steady clock in this case.
	if (clocksNeeded == 0)
	{
		clocksNeeded = params.steadyClocks = 1;
	}

	params.acceleration = acceleration = msg.acceleration;
	params.deceleration = deceleration = msg.deceleration;

	// Set up the plan
	segments = nullptr;
	moveInstance->GetAxisShaper().GetRemoteSegments(*this, params);

#if !SINGLE_DRIVER
	activeDMs = nullptr;
#endif
	bool realMove = false;
	for (size_t drive = 0; drive < NumDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];					// the steps for this move will be added later
		DriveMovement& dm = ddms[drive];
#if !SINGLE_DRIVER
		dm.nextDM = nullptr;
#endif
		bool stepsToDo = false;
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
				dm.totalSteps = 0;
				dm.direction = (extrusionRequested > 0.0);			// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
				Platform::EnableDrive(drive, 0);
				dm.PrepareExtruder(*this, extrusionRequested);
				realMove = true;
				stepsToDo = true;
			}
		}
		else
		{
			const int32_t delta = msg.perDrive[drive].steps;
			directionVector[drive] = (float)delta;
			if (delta != 0)
			{
				dm.totalSteps = labs(delta);						// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
				dm.direction = (delta >= 0);						// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
				Platform::EnableDrive(drive, 0);
				stepsToDo = dm.PrepareCartesianAxis(*this);
				if (stepsToDo)
				{
					realMove = true;
#if !SINGLE_DRIVER
					InsertDM(&dm);
#endif
					const int32_t netSteps = (dm.reverseStartStep < dm.totalSteps) ? (2 * dm.reverseStartStep) - dm.totalSteps : dm.totalSteps;
					if (dm.direction)
					{
						endPoint[drive] += netSteps;
					}
					else
					{
						endPoint[drive] -= netSteps;
					}
				}
			}
		}
		if (!stepsToDo)
		{
			dm.state = DMState::idle;								// no steps to do
			dm.currentSegment = nullptr;
			// No steps to do, so set up the steps so that GetStepsTaken will return zero
			dm.totalSteps = 0;
			dm.nextStep = 0;
			dm.reverseStartStep = 1;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		ReleaseSegments();			// we may have set up the segments, in which case we must recycle them
		return false;
	}

	state = frozen;												// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
}

// Start executing this move. Must be called with interrupts disabled, to avoid a race condition.
// startTime is the earliest that we can start the move, but we must not start it before its planned time
// After calling this, the first interrupt must be scheduled
void DDA::Start(uint32_t tim) noexcept
{
	const int32_t ticksOverdue = (int32_t)(tim - afterPrepare.moveStartTime);
	if (ticksOverdue > 0)
	{
		// Record the maximum overdue time
		if ((uint32_t)ticksOverdue > maxTicksOverdue)
		{
			const uint32_t increment = (uint32_t)ticksOverdue - maxTicksOverdue;
			maxTicksOverdue = (uint32_t)ticksOverdue;
			if (increment > maxOverdueIncrement)
			{
				maxOverdueIncrement = increment;
			}
		}

		// This move is starting late. See if we can recover some of the lost time by bringing the first step forward if it isn't already overdue.
		// Otherwise, if our clock runs slightly slower than the master, we will keep getting behind until there is a break in the moves
		const uint32_t bringFowardBy = min<uint32_t>((uint32_t)ticksOverdue, WhenNextInterruptDue()/2);
		afterPrepare.moveStartTime = tim - bringFowardBy;
	}
	state = executing;

#if SINGLE_DRIVER
	if (ddms[0].state == DMState::extruderPendingPreparation)
	{
		ddms[0].LatePrepareExtruder(*this);
	}
	if (ddms[0].state >= DMState::firstMotionState)
	{
		Platform::SetDirection(ddms[0].direction);
	}
#else
	for (DriveMovement& dm : ddms)
	{
		if (dm.state == DMState::extruderPendingPreparation)
		{
			if (dm.LatePrepareExtruder(*this))
			{
				InsertDM(&dm);
			}
		}
		if (dm.state >= DMState::firstMotionState)
		{
			Platform::SetDirection(dm.drive, dm.direction);
		}
	}
#endif
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

bool DDA::HasStepError() const noexcept
{
#if 0	//debug
	if (hadHiccup)
	{
		return true;			// temporary for debugging DAA
	}
#endif

	for (size_t drive = 0; drive < NumDrivers; ++drive)
	{
		const DMState st = ddms[drive].state;
		if (st >= DMState::stepError1 && st < DMState::firstMotionState)
		{
			return true;
		}
	}
	return false;
}

unsigned int DDA::GetAndClearStepErrors() noexcept
{
	const unsigned int ret = stepErrors;
	stepErrors =  0;
	return ret;
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
