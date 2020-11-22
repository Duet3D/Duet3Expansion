/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include "RepRapFirmware.h"
#include "MessageType.h"

#include "DDA.h"								// needed because of our inline functions
#include "Kinematics/Kinematics.h"

#if SUPPORT_DRIVERS

// Define the number of DDAs and DMs.
// A DDA represents a move in the queue.
// Each DDA needs one DM per drive that it moves.
// However, DM's are large, so we provide fewer than DRIVES * DdaRingLength of them. The planner checks that enough DMs are available before filling in a new DDA.

const unsigned int DdaRingLength = 50;
const unsigned int NumDms = DdaRingLength * NumDrivers;

#endif	// SUPPORT_DRIVERS

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
public:
	Move();
	void Init();																	// Start me up
	void Spin();																	// Called in a tight loop to keep the class going
	void Exit();																	// Shut down
	void Diagnostics(const StringRef& reply);										// Report useful stuff

#if SUPPORT_DRIVERS
	void Interrupt() __attribute__ ((hot));											// Timer callback for step generation
	void StopDrivers(uint16_t whichDrivers);
	void CurrentMoveCompleted() __attribute__ ((hot));								// Signal that the current move has just been completed

	// Kinematics and related functions
	Kinematics& GetKinematics() const { return *kinematics; }
	bool SetKinematics(KinematicsType k);											// Set kinematics, return true if successful
																					// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	static void TimerCallback(CallbackParameter cb)
	{
		static_cast<Move*>(cb.vp)->Interrupt();
	}

	void PrintCurrentDda() const;													// For debugging

	void ResetMoveCounters() { scheduledMoves = completedMoves = 0; }

	int32_t GetPosition(size_t driver) const;

	// Filament monitor support
	int32_t GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount
	uint32_t ExtruderPrintingSince() const noexcept { return extrudersPrintingSince; }	// When we started doing normal moves after the most recent extruder-only move

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;			// Get the current step interval for this axis or extruder
#endif

private:
	bool DDARingAdd();																// Add a processed look-ahead entry to the DDA ring
	DDA* DDARingGet();																// Get the next DDA ring entry to be run
	void StartNextMove(DDA *cdda, uint32_t startTime);								// Start a move

	// Variables that are in the DDARing class in RepRapFirmware (we have only one DDARing so they are here)
	DDA* volatile currentDda;
	DDA* ddaRingAddPointer;
	DDA* volatile ddaRingGetPointer;
	DDA* ddaRingCheckPointer;

	StepTimer timer;
	volatile int32_t extrusionAccumulators[NumDrivers]; 							// Accumulated extruder motor steps
	volatile uint32_t extrudersPrintingSince;										// The milliseconds clock time when extrudersPrinting was set to true
	volatile bool extrudersPrinting;												// Set whenever an extruder starts a printing move, cleared by a non-printing extruder move
	// End DDARing variables

	Kinematics *kinematics;								// What kinematics we are using

	unsigned int stepErrors;							// count of step errors, for diagnostics
	uint32_t scheduledMoves;							// Move counters for the code queue
	volatile uint32_t completedMoves;					// This one is modified by an ISR, hence volatile
	uint32_t numHiccups;								// How many times we delayed an interrupt to avoid using too much CPU time in interrupts
	bool active;										// Are we live and running?

#endif	//SUPPORT_DRIVERS

};

//******************************************************************************************************

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline uint32_t Move::GetStepInterval(size_t axis, uint32_t microstepShift) const
{
	const DDA * const cdda = currentDda;		// capture volatile variable
	return (cdda != nullptr) ? cdda->GetStepInterval(axis, microstepShift) : 0;
}

#endif

#endif /* MOVE_H_ */
