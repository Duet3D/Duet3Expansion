/*
 * DDARing.h
 *
 *  Created on: 28 Feb 2019
 *      Author: David
 *
 *  This class represents a queue of moves, where for each move the movement is synchronised between all the motors involved.
 */

#ifndef SRC_MOVEMENT_DDARING_H_
#define SRC_MOVEMENT_DDARING_H_

#include "DDA.h"

class MovementState;

class DDARing
{
public:
	DDARing() noexcept;

	void Init(unsigned int numDdas) noexcept;
	void Exit() noexcept;

	bool CanAddMove() const noexcept;
	void AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;			// add a move from the ATE to the movement queue

	const volatile int32_t *GetLastMoveStepsTaken() const noexcept { return lastMoveStepsTaken; }

	uint32_t Spin(bool signalMoveCompletion, bool shouldStartMove) noexcept SPEED_CRITICAL;	// Try to process moves in the ring
	bool IsIdle() const noexcept;														// Return true if this DDA ring is idle

	DDA *GetCurrentDDA() const noexcept;												// If a move from this ring should be executing now, fetch its DDA

	uint32_t GetScheduledMoves() const noexcept { return scheduledMoves; }				// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return completedMoves; }				// How many moves have been completed?
	void ResetMoveCounters() noexcept { scheduledMoves = completedMoves = 0; }

//	void SetPositions(Move& move, const float positions[NumDrivers], AxesBitmap axes) noexcept;	// Force the machine coordinates to be these

	void Diagnostics(unsigned int ringNumber, const StringRef& reply) noexcept;

private:
	uint32_t PrepareMoves(DDA *firstUnpreparedMove, uint32_t moveTimeLeft, unsigned int alreadyPrepared) noexcept;

	DDA* addPointer;															// Pointer to the next DDA that we can use to add a new move, if this DDA is free
	DDA* volatile getPointer;													// Pointer to the oldest committed or provisional move, if not equal to addPointer

	unsigned int numDdasInRing;													// The number of DDAs that this ring contains

	uint32_t scheduledMoves;													// Number of moves scheduled in this ring
	uint32_t completedMoves;													// Number of moves completed in this ring

	unsigned int numPrepareUnderruns;											// How many times we wanted a new move but there were only un-prepared moves in the queue
	unsigned int numNoMoveUnderruns;											// How many times we wanted a new move but there were none

	volatile int32_t lastMoveStepsTaken[NumDrivers];							// how many steps were taken in the last move we did
};

#endif /* SRC_MOVEMENT_DDARING_H_ */
