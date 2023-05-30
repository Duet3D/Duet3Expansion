/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#include "DDA.h"								// needed because of our inline functions
#include "Kinematics/Kinematics.h"
#include "AxisShaper.h"
#include "ExtruderShaper.h"

#if SUPPORT_CLOSED_LOOP
# include "StepperDrivers/TMC51xx.h"			// for SmartDrivers::GetMicrostepShift
# include <Platform/Platform.h>					// for GetDirectionValueNoCheck
#endif

// Define the number of DDAs
const unsigned int DdaRingLength = 50;

struct CanMessageStopMovement;
struct CanMessageSetInputShaping;

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
public:
	Move() noexcept;
	void Init() noexcept;															// Start me up
	void Exit() noexcept;															// Shut down
	void Diagnostics(const StringRef& reply) noexcept;								// Report useful stuff

	void Interrupt() noexcept SPEED_CRITICAL;										// Timer callback for step generation
	void StopDrivers(uint16_t whichDrives) noexcept;
	void CurrentMoveCompleted() noexcept SPEED_CRITICAL;							// Signal that the current move has just been completed

#if SUPPORT_DELTA_MOVEMENT
	// Kinematics and related functions
	Kinematics& GetKinematics() const noexcept { return *kinematics; }
	bool SetKinematics(KinematicsType k) noexcept;									// Set kinematics, return true if successful
#endif

#if !DEDICATED_STEP_TIMER
	static void TimerCallback(CallbackParameter cb) noexcept
	{
		static_cast<Move*>(cb.vp)->Interrupt();
	}
#endif

	void PrintCurrentDda() const noexcept;											// For debugging

	void ResetMoveCounters() noexcept { scheduledMoves = completedMoves = 0; }
	void UpdateExtrusionPendingLimits(float extrusionPending) noexcept;

	int32_t GetPosition(size_t driver) const noexcept;

	// Filament monitor support
	int32_t GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount
	uint32_t ExtruderPrintingSince() const noexcept { return extrudersPrintingSince; }	// When we started doing normal moves after the most recent extruder-only move

	// Input shaping support
	AxisShaper& GetAxisShaper() noexcept { return axisShaper; }
	GCodeResult HandleInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept
	{
		return axisShaper.EutSetInputShaping(msg, dataLength, reply);
	}

	// Pressure advance
	ExtruderShaper& GetExtruderShaper(size_t drive) noexcept { return extruderShapers[drive]; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;	// Get the current step interval for this axis or extruder
	bool SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept;
#endif

	void DebugPrintCdda() const noexcept;											// for debugging

	[[noreturn]] void TaskLoop() noexcept;

#if SUPPORT_CLOSED_LOOP
	bool GetCurrentMotion(size_t driver, uint32_t when, bool closedLoopEnabled, MotionParameters& mParams) noexcept;
																					// get the net full steps taken, including in the current move so far, also speed and acceleration; return true if moving
	void SetCurrentMotorSteps(size_t driver, float fullSteps) noexcept;
	void InvertCurrentMotorSteps(size_t driver) noexcept;
#endif

	const volatile int32_t *GetLastMoveStepsTaken() const noexcept { return lastMoveStepsTaken; }

private:
	bool DDARingAdd() noexcept;														// Add a processed look-ahead entry to the DDA ring
	DDA* DDARingGet() noexcept;														// Get the next DDA ring entry to be run
	void StartNextMove(DDA *cdda, uint32_t startTime) noexcept;						// Start a move

	// Variables that are in the DDARing class in RepRapFirmware (we have only one DDARing so they are here)
	DDA* volatile currentDda;
	DDA* ddaRingAddPointer;
	DDA* volatile ddaRingGetPointer;
	DDA* ddaRingCheckPointer;

#if !DEDICATED_STEP_TIMER
	StepTimer timer;
#endif

	volatile int32_t lastMoveStepsTaken[NumDrivers];								// how many steps were taken in the last move we did
	volatile int32_t movementAccumulators[NumDrivers]; 								// Accumulated motor steps
#if SUPPORT_CLOSED_LOOP
	float netMicrostepsTaken[NumDrivers];											// the net microsteps taken not counting any move that is in progress
#endif
	volatile uint32_t extrudersPrintingSince;										// The milliseconds clock time when extrudersPrinting was set to true
	volatile bool extrudersPrinting;												// Set whenever an extruder starts a printing move, cleared by a non-printing extruder move
	TaskBase * volatile taskWaitingForMoveToComplete;
	// End DDARing variables

	Kinematics *kinematics;															// What kinematics we are using

	AxisShaper axisShaper;
	ExtruderShaper extruderShapers[NumDrivers];
	uint32_t scheduledMoves;														// Move counters for the code queue
	volatile uint32_t completedMoves;												// This one is modified by an ISR, hence volatile
	uint32_t numHiccups;															// How many times we delayed an interrupt to avoid using too much CPU time in interrupts
	uint32_t maxPrepareTime;
	float minExtrusionPending = 0.0, maxExtrusionPending = 0.0;
};

//******************************************************************************************************

// Update the min and max extrusion pending values. These are reported by M122 to assist with debugging print quality issues.
// Inlined because this is only called from one place.
inline void Move::UpdateExtrusionPendingLimits(float extrusionPending) noexcept
{
	if (extrusionPending > maxExtrusionPending) { maxExtrusionPending = extrusionPending; }
	else if (extrusionPending < minExtrusionPending) { minExtrusionPending = extrusionPending; }
}

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline uint32_t Move::GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept
{
	const DDA * const cdda = currentDda;		// capture volatile variable
	return (cdda != nullptr) ? cdda->GetStepInterval(axis, microstepShift) : 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

// Get the motor position in the current move so far, also speed and acceleration. Units are full steps and step clocks.
// Inlined because it is only called from one place
inline bool Move::GetCurrentMotion(size_t driver, uint32_t when, bool closedLoopEnabled, MotionParameters& mParams) noexcept
{
	const float multiplier = ldexpf((Platform::GetDirectionValueNoCheck(driver)) ? -1.0 : 1.0, -(int)SmartDrivers::GetMicrostepShift(driver));
	AtomicCriticalSectionLocker lock;				// we don't want an interrupt changing currentDda or netMicrostepsTaken while we execute this
	for (;;)
	{
		DDA * cdda = currentDda;					// capture volatile variable
		if (cdda == nullptr)
		{
			break;
		}

		const uint32_t clocksSinceMoveStart = when - cdda->GetStartTime();
		if (clocksSinceMoveStart <= cdda->GetClocksNeeded())
		{
			// This move is executing
			cdda->GetCurrentMotion(driver, clocksSinceMoveStart, mParams);

			// Convert microsteps to full steps
			mParams.position = (mParams.position + netMicrostepsTaken[driver]) * multiplier;
			mParams.speed *= multiplier;
			mParams.acceleration *= multiplier;
			return true;
		}

		// If the machine has been idle, a move is made current a little ahead of when it is due, so check whether the move hasn't started yet
		if ((int32_t)clocksSinceMoveStart < 0)
		{
			break;
		}

		// This move has finished. If we are running in closed loop mode, mark it as completed because there will be no interrupt to do that.
		if (!closedLoopEnabled)
		{
			break;
		}

		const uint32_t finishTime = cdda->GetMoveFinishTime();	// calculate when this move should finish
		cdda->SetCompleted();
		CurrentMoveCompleted();

		// Start the next move if one is ready
		cdda = ddaRingGetPointer;
		if (cdda->GetState() != DDA::frozen)
		{
			break;
		}

		StartNextMove(cdda, finishTime);
	}

	// Here when there is no current move
	mParams.position = netMicrostepsTaken[driver] * multiplier;
	mParams.speed = mParams.acceleration = 0.0;
	return false;
}

inline void Move::SetCurrentMotorSteps(size_t driver, float fullSteps) noexcept
{
	const float multiplier = ldexpf((Platform::GetDirectionValueNoCheck(driver)) ? -1.0 : 1.0, (int)SmartDrivers::GetMicrostepShift(driver));
	netMicrostepsTaken[driver] = fullSteps * multiplier;
}

// Invert the current number of microsteps taken. Called when the driver direction control is changed.
inline void Move::InvertCurrentMotorSteps(size_t driver) noexcept
{
	netMicrostepsTaken[driver] = -netMicrostepsTaken[driver];
}

#endif	// SUPPORT_CLOSED_LOOP

#endif	// SUPPORT_DRIVERS

#endif /* MOVE_H_ */
