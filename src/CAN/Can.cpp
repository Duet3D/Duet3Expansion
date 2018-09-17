/*
 * Can.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include "Can.h"
#include "CanMessageFormats.h"
#include "Movement/StepTimer.h"

namespace CanManager
{
	const CanMovementMessage& GetCanMove()
	{
		static bool running = false;
		static bool forwards;
		static CanMovementMessage msg;

		static const int32_t steps1 = 6 * 200 * 16;

		if (!running)
		{
			forwards = true;
			msg.accelerationClocks = StepTimer::StepClockRate/2;
			msg.steadyClocks = StepTimer::StepClockRate * 2;
			msg.decelClocks = StepTimer::StepClockRate/2;
			msg.initialSpeedFraction = 0.0;
			msg.finalSpeedFraction = 0.0;

			msg.flags.deltaDrives = 0;
			msg.flags.endStopsToCheck = 0;
			msg.flags.pressureAdvanceDrives = 0;
			msg.flags.stopAllDrivesOnEndstopHit = false;

			msg.initialX = 0.0;			// only relevant for delta moves
			msg.initialY = 0.0;			// only relevant for delta moves
			msg.finalX = 1.0;			// only relevant for delta moves
			msg.finalY = 1.0;			// only relevant for delta moves
			msg.zMovement = 0.0;		// only relevant for delta moves

			msg.moveStartTime = StepTimer::GetInterruptClocks() + StepTimer::StepClockRate/100;		// start the move 10ms from now
			running = true;
		}
		else
		{
			if (msg.steadyClocks >= StepTimer::StepClockRate * 4)
			{
				msg.steadyClocks = 0;
			}
			else
			{
				msg.steadyClocks += StepTimer::StepClockRate;
			}
			// We can leave all the other parameters the same
			msg.moveStartTime += msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;
		}

		msg.perDrive[0].steps = (forwards) ? steps1 : -steps1;
		msg.perDrive[1].steps = msg.perDrive[0].steps/2;
		msg.perDrive[2].steps = msg.perDrive[0].steps/4;
		msg.timeNow = StepTimer::GetInterruptClocks();
		forwards = !forwards;

		return msg;
	}

	// This is called from the step ISR when the move is stopped by the Z probe
	void MoveStoppedByZProbe()
	{
		//TODO
	}
}

// End
