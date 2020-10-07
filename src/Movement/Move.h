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

	// Kinematics and related functions
	Kinematics& GetKinematics() const { return *kinematics; }
	bool SetKinematics(KinematicsType k);											// Set kinematics, return true if successful
																					// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	// Temporary kinematics functions
	bool IsDeltaMode() const { return kinematics->GetKinematicsType() == KinematicsType::linearDelta; }
	// End temporary functions

	bool IsRawMotorMove(uint8_t moveType) const;									// Return true if this is a raw motor move

	static void TimerCallback(CallbackParameter cb)
	{
		static_cast<Move*>(cb.vp)->Interrupt();
	}

	void CurrentMoveCompleted() __attribute__ ((hot));								// Signal that the current move has just been completed

	void PrintCurrentDda() const;													// For debugging

	void ResetMoveCounters() { scheduledMoves = completedMoves = 0; }

	int32_t GetPosition(size_t driver) const;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;			// Get the current step interval for this axis or extruder
#endif

private:
	bool DDARingAdd();									// Add a processed look-ahead entry to the DDA ring
	DDA* DDARingGet();									// Get the next DDA ring entry to be run

	// Variables that are in the DDARing class in RepRapFirmware (we have only one DDARing so they are here)
	DDA* volatile currentDda;
	DDA* ddaRingAddPointer;
	DDA* volatile ddaRingGetPointer;
	DDA* ddaRingCheckPointer;

	StepTimer timer;
	// End DDARing variables

	unsigned int idleCount;								// The number of times Spin was called and had no new moves to process

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
