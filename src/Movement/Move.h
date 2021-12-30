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

// Define the number of DDAs
const unsigned int DdaRingLength = 50;

struct CanMessageStopMovement;

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

	static void TimerCallback(CallbackParameter cb) noexcept
	{
		static_cast<Move*>(cb.vp)->Interrupt();
	}

	void PrintCurrentDda() const noexcept;											// For debugging

	void ResetMoveCounters() noexcept { scheduledMoves = completedMoves = 0; }

	int32_t GetPosition(size_t driver) const noexcept;

	// Filament monitor support
	int32_t GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount
	uint32_t ExtruderPrintingSince() const noexcept { return extrudersPrintingSince; }	// When we started doing normal moves after the most recent extruder-only move

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;	// Get the current step interval for this axis or extruder
	bool SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept;
#endif

	void DebugPrintCdda() const noexcept;											// for debugging

	[[noreturn]] void TaskLoop() noexcept;

#if SUPPORT_CLOSED_LOOP
	void GetCurrentMotion(MotionParameters& mParams) const noexcept;				// get the net full steps taken, including in the current move so far, also speed and acceleration
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

	StepTimer timer;
	volatile int32_t lastMoveStepsTaken[NumDrivers];								// how many steps were taken in the last move we did
	volatile int32_t movementAccumulators[NumDrivers]; 								// Accumulated motor steps
	volatile uint32_t extrudersPrintingSince;										// The milliseconds clock time when extrudersPrinting was set to true
	volatile bool extrudersPrinting;												// Set whenever an extruder starts a printing move, cleared by a non-printing extruder move
	TaskBase * volatile taskWaitingForMoveToComplete;
	// End DDARing variables

	Kinematics *kinematics;															// What kinematics we are using

	uint32_t scheduledMoves;														// Move counters for the code queue
	volatile uint32_t completedMoves;												// This one is modified by an ISR, hence volatile
	uint32_t numHiccups;															// How many times we delayed an interrupt to avoid using too much CPU time in interrupts
	uint32_t maxPrepareTime;

#if SUPPORT_CLOSED_LOOP
# if SINGLE_DRIVER
	int32_t netMicrostepsTaken;														// the net microsteps taken not counting any move that is in progress
	int driver0MicrostepShift;														// the microstepping set for driver 0 as a negative shift factor
# else
#  error Only one closed loop driver supported by this code
# endif
#endif
};

//******************************************************************************************************

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

// Get the net full steps taken, including in the current move so far, also speed and acceleration
inline void Move::GetCurrentMotion(MotionParameters& mParams) const noexcept
{
	AtomicCriticalSectionLocker lock;
	const DDA * const cdda = currentDda;			// capture volatile variable
	if (cdda != nullptr)
	{
		cdda->GetCurrentMotion(mParams, netMicrostepsTaken, driver0MicrostepShift);
	}
	else
	{
		mParams.position = ldexp((float)netMicrostepsTaken, driver0MicrostepShift);
		mParams.speed = mParams.acceleration = 0.0;
	}
}

#endif

#endif	// SUPPORT_DRIVERS

#endif /* MOVE_H_ */
