/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"

#if SUPPORT_DRIVERS

#include "Platform.h"
#include "Move.h"
#include "Kinematics/LinearDeltaKinematics.h"		// for DELTA_AXES
#include "CanMessageFormats.h"
#include <CAN/CanInterface.h>

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

	MoveParameters()
	{
		accelDistance = steadyDistance = decelDistance = requestedSpeed = startSpeed = topSpeed = endSpeed = targetNextSpeed = 0.0;
		endstopChecks = 0;
		flags = 0;
	}

	void DebugPrint() const
	{
		Platform::MessageF(DebugMessage, "%f,%f,%f,%f,%f,%f,%f,%f,%08" PRIX32 ",%04x\n",
								(double)accelDistance, (double)steadyDistance, (double)decelDistance, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed,
								(double)targetNextSpeed, endstopChecks, flags);
	}

	static void PrintHeading()
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
/*static*/ void DDA::PrintMoves()
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

/*static*/ void DDA::PrintMoves() { }

#endif

unsigned int DDA::stepErrors = 0;

uint32_t DDA::stepsRequested[NumDrivers];
uint32_t DDA::stepsDone[NumDrivers];

DDA::DDA(DDA* n) : next(n), prev(nullptr), state(empty)
{
	for (DriveMovement*& p : pddm)
	{
		p = nullptr;
	}

	for (int32_t& ep : endPoint)
	{
		ep = 0;
	}
}

void DDA::ReleaseDMs()
{
	for (DriveMovement*& p : pddm)
	{
		if (p != nullptr)
		{
			DriveMovement::Release(p);
			p = nullptr;
		}
	}
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(afterPrepare.moveStartTime + clocksNeeded - StepTimer::GetTimerTicks())
			: (int32_t)clocksNeeded;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm)
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
void DDA::RemoveDM(size_t drive)
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

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), (double)vec[i]);
	}
	debugPrintf("]");
}

// Print the text followed by the DDA only
void DDA::DebugPrint() const
{
	debugPrintf("DDA: a=%e d=%e startv=%e topv=%e endv=%e sa=%f sd=%f\n"
				"cks=%" PRIu32 " sstcda=%" PRIu32 " tstcddpdsc=%" PRIu32 " exac=%" PRIi32 "\n",
				(double)acceleration, (double)deceleration, (double)startSpeed, (double)topSpeed, (double)endSpeed, (double)accelDistance, (double)decelDistance,
				clocksNeeded, afterPrepare.startSpeedTimesCdivA, afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks, afterPrepare.extraAccelerationClocks);
}

// Print the DDA and active DMs
void DDA::DebugPrintAll() const
{
	DebugPrint();
	for (size_t axis = 0; axis < NumDrivers; ++axis)
	{
		if (pddm[axis] != nullptr)
		{
			pddm[axis]->DebugPrint("ABCDEF"[axis]);
		}
	}
}

// This is called by Move to initialize all DDAs
void DDA::Init()
{
	state = empty;
}

// Set up a real move. Return true if it represents real movement, else false.
// Return true if it is a real move
bool DDA::Init(const CanMessageMovementLinear& msg)
{
	// 0. Initialise the endpoints, which are used for diagnostic purposes, and set up the DriveMovement objects
	bool realMove = false;

	const size_t numDrivers = min<size_t>(msg.numDrivers, NumDrivers);
	for (size_t drive = 0; drive < numDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];		// the steps for this move will be added later
		const int32_t delta = msg.perDrive[drive].steps;

		if (delta != 0)
		{
			realMove = true;
			DriveMovement*& pdm = pddm[drive];
			pdm = DriveMovement::Allocate(drive, DMState::moving);
			pdm->totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			pdm->direction = (delta >= 0);				// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
			stepsRequested[drive] += labs(delta);
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	afterPrepare.moveStartTime = msg.whenToExecute;
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;
	flags.isPrintingMove = (msg.pressureAdvanceDrives != 0);
	flags.hadHiccup = false;
	flags.goingSlow = false;

	topSpeed = 2.0/(2 * msg.steadyClocks + (msg.initialSpeedFraction + 1.0) * msg.accelerationClocks + (msg.finalSpeedFraction + 1.0) * msg.decelClocks);
	startSpeed = topSpeed * msg.initialSpeedFraction;
	endSpeed = topSpeed * msg.finalSpeedFraction;

	acceleration = (msg.accelerationClocks == 0) ? 1.0 : (topSpeed * (1.0 - msg.initialSpeedFraction))/msg.accelerationClocks;
	deceleration = (msg.decelClocks == 0) ? 1.0 : (topSpeed * (1.0 - msg.finalSpeedFraction))/msg.decelClocks;

	accelDistance = (msg.accelerationClocks == 0) ? 0.0
						: (msg.accelerationClocks == clocksNeeded) ? 1.0
							: topSpeed * (1.0 + msg.initialSpeedFraction) * msg.accelerationClocks * 0.5;
	decelDistance = (msg.decelClocks == 0) ? 0.0
						: (msg.decelClocks == clocksNeeded) ? 1.0
							: topSpeed * (1.0 + msg.finalSpeedFraction) * msg.decelClocks * 0.5;

	PrepParams params;
	// We must avoid getting negative distance in the following calculation because it messes up the calculation of twoDistanceToStopTimesCsquaredDivD in DriveMovement:Prepare
	// The conditional code in calculating decelDistance should achieve that
	params.decelStartDistance = 1.0 - decelDistance;
	afterPrepare.startSpeedTimesCdivA = (uint32_t)roundU32(startSpeed/acceleration);
	params.topSpeedTimesCdivD = (uint32_t)roundU32(topSpeed/deceleration);
	afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks = params.topSpeedTimesCdivD + msg.accelerationClocks + msg.steadyClocks;
	afterPrepare.extraAccelerationClocks = msg.accelerationClocks - roundS32(accelDistance/topSpeed);
	params.compFactor = (topSpeed - startSpeed)/topSpeed;

	activeDMs = nullptr;

	for (size_t drive = 0; drive < numDrivers; ++drive)
	{
		DriveMovement* const pdm = FindDM(drive);
		if (pdm != nullptr && pdm->state == DMState::moving)
		{
			Platform::EnableDrive(drive);
			if ((msg.pressureAdvanceDrives & (1u << drive)) != 0)
			{
				// If there is any extruder jerk in this move, in theory that means we need to instantly extrude or retract some amount of filament.
				// Pass the speed change to PrepareExtruder
				// But PrepareExtruder doesn't use it currently, so don't bother
				pdm->PrepareExtruder(*this, params, 0.0);

				// Check for sensible values, print them if they look dubious
				if (Platform::Debug(moduleDda)
					&& (   pdm->totalSteps > 1000000
						|| pdm->reverseStartStep < pdm->mp.cart.decelStartStep
						|| (pdm->reverseStartStep <= pdm->totalSteps
							&& pdm->mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD > (int64_t)(pdm->mp.cart.twoCsquaredTimesMmPerStepDivD * pdm->reverseStartStep)
						   )
					   )
				   )
				{
					DebugPrintAll();
				}
			}
			else
			{
				pdm->PrepareCartesianAxis(*this, params);

				// Check for sensible values, print them if they look dubious
				if (Platform::Debug(moduleDda) && pdm->totalSteps > 1000000)
				{
					DebugPrintAll();
				}
			}

			const uint32_t netSteps = (pdm->reverseStartStep < pdm->totalSteps) ? (2 * pdm->reverseStartStep) - pdm->totalSteps : pdm->totalSteps;
			if (pdm->direction)
			{
				endPoint[drive] += netSteps;
			}
			else
			{
				endPoint[drive] -= netSteps;
			}

			// Prepare for the first step
			pdm->nextStep = 0;
			pdm->nextStepTime = 0;
			pdm->stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
			pdm->stepsTillRecalc = 0;							// so that we don't skip the calculation

			const bool stepsToDo = pdm->CalcNextStepTime(*this);
			if (stepsToDo)
			{
				pdm->directionChanged = false;
				InsertDM(pdm);
			}
			else
			{
				if (pdm->state == DMState::stepError)
				{
					DebugPrint();
					pdm->DebugPrint(drive + '0');
					//msg.DebugPrint();
				}
				pdm->state = DMState::idle;
			}
		}
	}

	if (Platform::Debug(moduleDda) && Platform::Debug(moduleMove))		// temp show the prepared DDA if debug enabled for both modules
	{
		DebugPrintAll();
	}

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
}

// Start executing this move. Must be called with interrupts disabled, to avoid a race condition.
// startTime is the earliest that we can start the move, but we must not start it before its planned time
void DDA::Start(uint32_t tim)
{
	if ((int32_t)(afterPrepare.moveStartTime - tim) < 0)
	{
		afterPrepare.moveStartTime = tim;			// this move is late starting, so record the actual start time
	}
	state = executing;

	if (activeDMs != nullptr)
	{
		for (size_t i = 0; i < NumDrivers; ++i)
		{
			DriveMovement* const pdm = FindDM(i);
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
#if SINGLE_DRIVER
				Platform::SetDirection(pdm->direction);
#else
				Platform::SetDirection(pdm->drive, pdm->direction);
#endif
			}
		}
	}
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
void DDA::StepDrivers(uint32_t now)
{
	// Determine whether the driver is due for stepping, overdue, or will be due very shortly
	DriveMovement* const dm = activeDMs;
	if (dm != nullptr && (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval >= dm->nextStepTime)	// if the next step is due
	{
		// Step the driver
		bool hasMoreSteps;

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
			lastStepHighTime = StepTimer::GetTimerTicks();				//TODO adjust lastStepLowTime to allow for the pulse length
			hasMoreSteps = dm->CalcNextStepTime(*this);
#  else
			uint32_t lastStepPulseTime = lastStepLowTime;
			while (now - lastStepPulseTime < Platform::GetSlowDriverStepLowClocks() || now - lastDirChangeTime < Platform::GetSlowDriverDirSetupClocks())
			{
				now = StepTimer::GetTimerTicks();
			}
			Platform::StepDriverHigh();									// generate the step
			lastStepPulseTime = StepTimer::GetTimerTicks();
			hasMoreSteps = dm->CalcNextStepTime(*this);

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
			hasMoreSteps = dm->CalcNextStepTime(*this);
# else
			Platform::StepDriverHigh();									// generate the step
			hasMoreSteps = dm->CalcNextStepTime(*this);
			Platform::StepDriverLow();									// set the step pin low
# endif
		}

		++stepsDone[0];
		if (!hasMoreSteps)
		{
			activeDMs = nullptr;
		}
		else if (dm->directionChanged)
		{
			dm->directionChanged = false;
			Platform::SetDirection(dm->direction);
		}
	}

	// If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr && StepTimer::GetTimerTicks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

#else

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
void DDA::StepDrivers(uint32_t now)
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
		if (dmToInsert->state == DMState::moving)
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

// Stop a drive and re-calculate the corresponding endpoint.
// For extruder drivers, we need to be able to calculate how much of the extrusion was completed after calling this.
void DDA::StopDrive(size_t drive)
{
	DriveMovement* const pdm = FindDM(drive);
	if (pdm != nullptr && pdm->state == DMState::moving)
	{
		pdm->state = DMState::idle;
		RemoveDM(drive);
		if (activeDMs == nullptr)
		{
			state = completed;
		}
	}
}

// This is called when we abort a move because we have hit an endstop or we are doing an emergency pause.
// It stop all drives and adjusts the end points of the current move to account for how far through the move we got.
// The caller must call MoveCompleted at some point after calling this.
void DDA::MoveAborted()
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < NumDrivers; ++drive)
		{
			StopDrive(drive);
		}
	}
	state = completed;
}

void DDA::StopDrivers(uint16_t whichDrivers)
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < NumDrivers; ++drive)
		{
			if (whichDrivers & (1 << drive))
			{
				StopDrive(drive);
			}
		}
	}
}

bool DDA::HasStepError() const
{
#if 0	//debug
	if (hadHiccup)
	{
		return true;			// temporary for debugging DAA
	}
#endif

	for (size_t drive = 0; drive < NumDrivers; ++drive)
	{
		const DriveMovement* const pdm = FindDM(drive);
		if (pdm != nullptr && pdm->state == DMState::stepError)
		{
			return true;
		}
	}
	return false;
}

// Free up this DDA, returning true if the lookahead underrun flag was set
void DDA::Free()
{
	ReleaseDMs();
	state = empty;
}

// Return the number of net steps already taken in this move by a particular drive
int32_t DDA::GetStepsTaken(size_t drive) const
{
	const DriveMovement * const dmp = FindDM(drive);
	return (dmp != nullptr) ? dmp->GetNetStepsTaken() : 0;
}

unsigned int DDA::GetAndClearStepErrors() noexcept
{
	const unsigned int ret = stepErrors;
	stepErrors =  0;
	return ret;
}

#endif	// SUPPORT_DRIVERS

// End
