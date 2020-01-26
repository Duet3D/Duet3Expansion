/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"
#include "Platform.h"
#include "Move.h"
#include "Endstops/EndstopsManager.h"
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

DDA::DDA(DDA* n) : next(n), prev(nullptr), state(empty)
{
	for (DriveMovement*& p : pddm)
	{
		p = nullptr;
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
	debugPrintf("DDA:");
	debugPrintf("\n"
				"a=%f d=%f startv=%f topv=%f endv=%f sa=%f sd=%f\n"
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

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
}

// Set up a real move. Return true if it represents real movement, else false.
// Return true if it is a real move
bool DDA::Init(const CanMessageMovement& msg)
{
	// 0. Update the endpoints (do we even need them?)
	bool realMove = false;

	for (size_t drive = 0; drive < NumDrivers; drive++)
	{
		int32_t delta = msg.perDrive[drive].steps;

		if (delta != 0)
		{
			realMove = true;
			DriveMovement*& pdm = pddm[drive];
			pdm = DriveMovement::Allocate(drive, DMState::moving);
			pdm->totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			pdm->direction = (delta >= 0);				// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
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
	flags.stopAllDrivesOnEndstopHit = msg.stopAllDrivesOnEndstopHit;

	flags.hadHiccup = false;
	flags.goingSlow = false;

#if SUPPORT_IOBITS
	laserPwmOrIoBits = qq;
#endif

	topSpeed = 2.0/(2 * msg.steadyClocks + (msg.initialSpeedFraction + 1.0) * msg.accelerationClocks + (msg.finalSpeedFraction + 1.0) * msg.decelClocks);
	startSpeed = topSpeed * msg.initialSpeedFraction;
	endSpeed = topSpeed * msg.finalSpeedFraction;

	acceleration = (msg.accelerationClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.initialSpeedFraction))/msg.accelerationClocks;
	deceleration = (msg.decelClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.finalSpeedFraction))/msg.decelClocks;

	accelDistance = topSpeed * (1.0 + msg.initialSpeedFraction) * msg.accelerationClocks * 0.5;
	decelDistance = topSpeed * (1.0 + msg.finalSpeedFraction) * msg.decelClocks * 0.5;

	state = provisional;
	Prepare(msg);
	return true;
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(const CanMessageMovement& msg)
{
	PrepParams params;
	params.decelStartDistance = 1.0 - decelDistance;

	if (msg.deltaDrives != 0)
	{
		afterPrepare.cKc = roundS32(msg.zMovement * DriveMovement::Kc);
		params.dvecX = msg.finalX - msg.initialX;
		params.dvecY = msg.finalY - msg.initialY;
		params.dvecZ = msg.zMovement;
		params.a2plusb2 = fsquare(params.dvecX) + fsquare(params.dvecY);
		params.initialX = msg.initialX;
		params.initialY = msg.initialY;
		params.dparams = static_cast<const LinearDeltaKinematics*>(&(moveInstance->GetKinematics()));
	}

	afterPrepare.startSpeedTimesCdivA = (uint32_t)roundU32(startSpeed/acceleration);
	params.topSpeedTimesCdivD = (uint32_t)roundU32(topSpeed/deceleration);
	afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks = params.topSpeedTimesCdivD + msg.accelerationClocks + msg.steadyClocks;
	afterPrepare.extraAccelerationClocks = msg.accelerationClocks - roundS32(accelDistance/topSpeed);
	params.compFactor = (topSpeed - startSpeed)/topSpeed;

	activeDMs = nullptr;

	for (size_t drive = 0; drive < NumDrivers; ++drive)
	{
		DriveMovement* const pdm = FindDM(drive);
		if (pdm != nullptr && pdm->state == DMState::moving)
		{
			Platform::EnableDrive(drive);
			if ((msg.deltaDrives & (1u << drive)) != 0)			// for now, additional axes are assumed to be not part of the delta mechanism
			{
				pdm->PrepareDeltaAxis(*this, params);

				// Check for sensible values, print them if they look dubious
				if (Platform::Debug(moduleDda) && pdm->totalSteps > 1000000)
				{
					DebugPrintAll();
				}
			}
			else if ((msg.pressureAdvanceDrives & (1u << drive)) != 0)
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

			// Prepare for the first step
			pdm->nextStep = 0;
			pdm->nextStepTime = 0;
			pdm->stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
			pdm->stepsTillRecalc = 0;							// so that we don't skip the calculation
			const bool stepsToDo = (pdm->IsDeltaMovement())
									? pdm->CalcNextStepTimeDelta(*this, false)
									: pdm->CalcNextStepTimeCartesian(*this, false);
			if (stepsToDo)
			{
				InsertDM(pdm);
			}
			else
			{
				pdm->state = DMState::idle;
			}
		}
	}

	if (Platform::Debug(moduleDda) && Platform::Debug(moduleMove))		// temp show the prepared DDA if debug enabled for both modules
	{
		DebugPrintAll();
	}

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// The remaining functions are speed-critical, so use full optimisation
// The GCC optimize pragma appears to be broken, if we try to force O3 optimisation here then functions are never inlined

// Start executing this move. Must be called with interrupts disabled, to avoid a race condition.
void DDA::Start(uint32_t tim)
pre(state == frozen)
{
	if ((int32_t)(afterPrepare.moveStartTime - tim) < 0)
	{
		afterPrepare.moveStartTime = tim;			// this move is late starting, so record the actual start time
	}
	state = executing;

	if (activeDMs != nullptr)
	{

#if SUPPORT_LASER
		// Deal with laser power
		if (GCodes::GetMachineType() == MachineType::laser)
		{
			// Ideally we should ramp up the laser power as the machine accelerates, but for now we don't.
			Platform::SetLaserPwm(laserPwmOrIoBits.laserPwm);
		}
#endif

		for (size_t i = 0; i < NumDrivers; ++i)
		{
			DriveMovement* const pdm = FindDM(i);
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
				const size_t drive = pdm->drive;
				Platform::SetDirection(drive, pdm->direction);
			}
		}
	}
}

uint32_t DDA::lastStepLowTime = 0;
uint32_t DDA::lastDirChangeTime = 0;

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
void DDA::StepDrivers()
{
	// 1. There is no step 1.
	// 2. Determine which drivers are due for stepping, overdue, or will be due very shortly
	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	uint32_t now = StepTimer::GetTimerTicks();
	const uint32_t elapsedTime = (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval;
	while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
	{
		driversStepping |= Platform::GetDriversBitmap(dm->drive);
		dm = dm->nextDM;
	}

	if ((driversStepping & Platform::GetSlowDriversBitmap().GetRaw()) == 0)	// if not using any external drivers
	{
		// 3. Step the drivers
		Platform::StepDriversHigh(driversStepping);					// generate the steps
	}
	else
	{
		// 3. Step the drivers
		uint32_t lastStepPulseTime = lastStepLowTime;
		while (now - lastStepPulseTime < Platform::GetSlowDriverStepLowClocks() || now - lastDirChangeTime < Platform::GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetTimerTicks();
		}
		Platform::StepDriversHigh(driversStepping);					// generate the steps
		lastStepPulseTime = StepTimer::GetTimerTicks();

		// 3a. Reset all step pins low. Do this now because some external drivers don't like the direction pins being changed before the end of the step pulse.
		while (StepTimer::GetTimerTicks() - lastStepPulseTime < Platform::GetSlowDriverStepHighClocks()) {}
		Platform::StepDriversLow();									// set all step pins low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}

	// 4. Remove those drives from the list, calculate the next step times, update the direction pins where necessary,
	//    and re-insert them so as to keep the list in step-time order.
	//    Note that the call to CalcNextStepTime may change the state of Direction pin.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		const bool hasMoreSteps = (dm->IsDeltaMovement())
				? dmToInsert->CalcNextStepTimeDelta(*this, true)
				: dmToInsert->CalcNextStepTimeCartesian(*this, true);
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (hasMoreSteps)
		{
			InsertDM(dmToInsert);
		}
		dmToInsert = nextToInsert;
	}

	// 5. Reset all step pins low. We already did this if we are using any external drivers, but doing it again does no harm.
	Platform::StepDriversLow();										// set all step pins low

	// 6. If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr && StepTimer::GetTimerTicks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

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

// Reduce the speed of this move to the indicated speed.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed()
{
	if (!flags.goingSlow)
	{
		flags.goingSlow = true;

		topSpeed *= (1.0/ProbingSpeedReductionFactor);

		// Adjust extraAccelerationClocks so that step timing will be correct in the steady speed phase at the new speed
		const uint32_t clocksSoFar = StepTimer::GetTimerTicks() - afterPrepare.moveStartTime;
		afterPrepare.extraAccelerationClocks = (afterPrepare.extraAccelerationClocks * (int32_t)ProbingSpeedReductionFactor) - ((int32_t)clocksSoFar * (int32_t)(ProbingSpeedReductionFactor - 1));

		// We also need to adjust the total clocks needed, to prevent step errors being recorded
		if (clocksSoFar < clocksNeeded)
		{
			clocksNeeded += (clocksNeeded - clocksSoFar) * (ProbingSpeedReductionFactor - 1u);
		}

		// Adjust the speed in the DMs
		for (size_t drive = 0; drive < NumDrivers; ++drive)
		{
			DriveMovement* const pdm = FindDM(drive);
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
				pdm->ReduceSpeed(*this, ProbingSpeedReductionFactor);
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
bool DDA::Free()
{
	ReleaseDMs();
	state = empty;
	return false;
}

// Return the number of net steps already taken in this move by a particular drive
int32_t DDA::GetStepsTaken(size_t drive) const
{
	const DriveMovement * const dmp = FindDM(drive);
	return (dmp != nullptr) ? dmp->GetNetStepsTaken() : 0;
}

// End
