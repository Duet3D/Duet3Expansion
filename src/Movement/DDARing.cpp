/*
 * DDARing.cpp
 *
 *  Created on: 28 Feb 2019
 *      Author: David
 */

#include "DDARing.h"
#include "Move.h"
#include <Platform/Platform.h>
#include <Platform/Tasks.h>

/* Note on how the DDA ring works, using the new step-generation code that implements late input shaping:
 * A DDA represents a straight-line move with at least one of an acceleration segment, a steady speed segment, and a deceleration segment.
 * A single G0 or G1 command may be represented by a single DDA, or by multiple DDAs when the move has been segmented.
 *
 * DDAs are added to a ring in response to G0, G1, G2 and G3 commands and when RRF generates movement automatically (e.g. probing moves).
 * A newly-added DDA is in state 'provisional' and has its end speed set to zero. In this state its speed, acceleration and deceleration can be modified.
 * These modifications happen as other DDAs are added to the ring and the DDAs are adjusted to give a smooth transition between them.
 *
 * Shortly before a move is due to be executed, DDA::Prepare is called. This causes the move parameters to be frozen.
 * Move segments are generated, and/or the move details are sent to CAN-connected expansion boards. The DDA state is set to "scheduled".
 *
 * The scheduled DDA remains in the ring until the time for it to finish executing has passed, in order that we can report on
 * the parameters of the currently-executing move, e.g. requested and top speeds, extrusion rate, and extrusion amount for the filament monitor.
 *
 * When a move requires that endstops and/or Z probes are active, all other moves are completed before starting it, and no new moves are allowed
 * to be added to the ring until it completes. So it is the only move in the ring with state 'scheduled'.
 */

constexpr uint32_t MoveStartPollInterval = 10;					// delay in milliseconds between checking whether we should start moves

DDARing::DDARing() noexcept : scheduledMoves(0), completedMoves(0)
{
}

// This can be called in the constructor for class Move
void DDARing::Init(unsigned int numDdas) noexcept
{
	numDdasInRing = numDdas;

	// Build the DDA ring
	DDA *dda = new DDA(nullptr);
	addPointer = dda;
	for (size_t i = 1; i < numDdas; i++)
	{
		DDA * const oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	addPointer->SetNext(dda);
	dda->SetPrevious(addPointer);

	getPointer = addPointer;
	numPrepareUnderruns = numNoMoveUnderruns = 0;
}

void DDARing::Exit() noexcept
{
	// Clear the DDA ring so that we don't report any moves as pending
	DDA *gp;										// use a local variable to avoid loading volatile variable getPointer too often
	while ((gp = getPointer) != addPointer)
	{
		gp->Free();
		getPointer = gp = gp->GetNext();
	}
}

bool DDARing::CanAddMove() const noexcept
{
	 if (   addPointer->GetState() == DDA::empty
		 && addPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
		)
	 {
			// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
			// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is less than 0.5 seconds.
			const DDA *dda = addPointer;
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

			return (unPreparedTime < StepClockRate/2 || unPreparedTime + prevMoveTime < 2 * StepClockRate);
	 }
	 return false;
}

// Try to process moves in the ring. Called by the Move task.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or MoveTiming::StandardMoveWakeupInterval if there are no unprepared moves left.
uint32_t DDARing::Spin(bool signalMoveCompletion, bool shouldStartMove) noexcept
{
	DDA *cdda = getPointer;											// capture volatile variable

	// See if we can retire any completed moves
	while (cdda->GetState() == DDA::committed && cdda->HasExpired())
	{
		++completedMoves;
		//debugPrintf("Retiring move: now=%" PRIu32 " start=%" PRIu32 " dur=%" PRIu32 "\n", StepTimer::GetMovementTimerTicks(), cdda->GetMoveStartTime(), cdda->GetClocksNeeded());
		if (cdda->Free())
		{
			++numLookaheadUnderruns;
		}
		getPointer = cdda = cdda->GetNext();
	}

	// If we are already moving, see whether we need to prepare any more moves
	if (cdda->GetState() == DDA::committed)							// if we have started executing moves
	{
		const DDA* const currentMove = cdda;						// save for later

		// Count how many prepared or executing moves we have and how long they will take
		uint32_t preparedTime = 0;
		unsigned int preparedCount = 0;
		while (cdda->GetState() == DDA::committed)
		{
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
		}

		const uint32_t ret = (cdda->GetState() == DDA::provisional)
						? PrepareMoves(cdda, preparedTime, preparedCount)
							: MoveTiming::StandardMoveWakeupInterval;

		if (signalMoveCompletion)
		{
			// Wake up the Move task shortly after we expect the current move to finish
			const int32_t moveTicksLeft = currentMove->GetMoveFinishTime() - StepTimer::GetMovementTimerTicks();
			if (moveTicksLeft < 0)
			{
				return 0;
			}

			const uint32_t moveTime = moveTicksLeft/(StepClockRate/1000) + 1;	// 1ms ticks until the move finishes plus 1ms
			if (moveTime < ret)
			{
				return moveTime;
			}
		}

		return ret;
	}

	// No DDA is committed, so commit a new one if possible
	if (shouldStartMove)
	{
		const uint32_t ret = PrepareMoves(cdda, 0, 0);
		if (cdda->GetState() == DDA::committed)
		{
			if (signalMoveCompletion)
			{
				// Wake up the Move task shortly after we expect the current move to finish
				const int32_t moveTicksLeft = cdda->GetMoveFinishTime() - StepTimer::GetMovementTimerTicks();
				if (moveTicksLeft < 0)
				{
					return 0;
				}

				const uint32_t moveTime = moveTicksLeft/(StepTimer::StepClockRate/1000) + 1;	// 1ms ticks until the move finishes plus 1ms
				if (moveTime < ret)
				{
					return moveTime;
				}
			}
		}
		return ret;
	}

	return (cdda->GetState() == DDA::provisional)
			? MoveStartPollInterval									// there are moves in the queue but it is not time to prepare them yet
				: MoveTiming::StandardMoveWakeupInterval;			// the queue is empty, nothing to do until new moves arrive
}

// Prepare some moves. moveTimeLeft is the total length remaining of moves that are already executing or prepared.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or MoveTiming::StandardMoveWakeupInterval if there are no unprepared moves left.
uint32_t DDARing::PrepareMoves(DDA *firstUnpreparedMove, uint32_t moveTimeLeft, unsigned int alreadyPrepared) noexcept
{
	// If the number of prepared moves will execute in less than the minimum time, prepare another move.
	// Try to avoid preparing deceleration-only moves too early
	while (	  firstUnpreparedMove->GetState() == DDA::provisional
		   && moveTimeLeft < (int32_t)MoveTiming::UsualMinimumPreparedTime	// prepare moves one tenth of a second ahead of when they will be needed
		   && alreadyPrepared * 2 < numDdasInRing						// but don't prepare more than half the ring, to handle accelerate/decelerate moves in small segments
		   && (firstUnpreparedMove->IsGoodToPrepare() || moveTimeLeft < MoveTiming::AbsoluteMinimumPreparedTime)
		  )
	{
		firstUnpreparedMove->Prepare(*this);
		moveTimeLeft += firstUnpreparedMove->GetTimeLeft();
		++alreadyPrepared;
		firstUnpreparedMove = firstUnpreparedMove->GetNext();
	}

	// Decide how soon we want to be called again to prepare further moves
	if (firstUnpreparedMove->GetState() == DDA::provisional)
	{
		// There are more moves waiting to be prepared, so ask to be woken up early
		const int32_t clocksTillWakeup = (int32_t)(moveTimeLeft - MoveTiming::UsualMinimumPreparedTime);			// calculate how long before we run out of prepared moves, less the usual advance prepare time
		return (clocksTillWakeup <= 0) ? 2 : max<uint32_t>((uint32_t)clocksTillWakeup/(StepClockRate/1000), 2);		// wake up at that time, but delay for at least 2 ticks
	}

	// There are no moves waiting to be prepared
	return MoveTiming::StandardMoveWakeupInterval;
}

// Return true if this DDA ring is idle
bool DDARing::IsIdle() const noexcept
{
	return getPointer->GetState() == DDA::empty;
}

void DDARing::Diagnostics(unsigned int ringNumber, const StringRef& reply) noexcept
{
	reply.catf("=== DDARing %u ===\nScheduled moves %" PRIu32 ", completed %" PRIu32 ", Underruns [%u, %u]\n",
					ringNumber, scheduledMoves, completedMoves, numPrepareUnderruns, numNoMoveUnderruns);
	numPrepareUnderruns = numNoMoveUnderruns = 0;
}

// Add a move from the ATE to the movement queue
void DDARing::AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept
{
	if (addPointer->GetState() == DDA::empty)
	{
		if (addPointer->InitFromRemote(msg))
		{
			addPointer = addPointer->GetNext();
			scheduledMoves++;
		}
	}
}

// End
