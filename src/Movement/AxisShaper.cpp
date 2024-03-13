/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "AxisShaper.h"

#if SUPPORT_DRIVERS

#include "StepTimer.h"
#include "DDA.h"
#include "MoveSegment.h"
#include <CanMessageFormats.h>

AxisShaper::AxisShaper() noexcept
	  : numExtraImpulses(0)
{
}

// Handle a request from the master board to set input shaping parameters
GCodeResult AxisShaper::EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept
{
	if (msg.numExtraImpulses <= MaxExtraImpulses && dataLength >= msg.GetActualDataLength())
	{
		numExtraImpulses = msg.numExtraImpulses;
		for (size_t i = 0; i < numExtraImpulses; ++i)
		{
			coefficients[i] = msg.impulses[i].coefficient;
			durations[i] = msg.impulses[i].duration;
		}
		CalculateDerivedParameters();
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

// Calculate the input shaping parameters that we can derive from the primary ones
void AxisShaper::CalculateDerivedParameters() noexcept
{
	// Calculate the total extra duration of input shaping
	totalShapingClocks = 0.0;
	extraClocksAtStart = 0.0;
	extraClocksAtEnd = 0.0;
	extraDistanceAtStart = 0.0;
	extraDistanceAtEnd = 0.0;

	{
		float u = 0.0;
		for (unsigned int i = 0; i < numExtraImpulses; ++i)
		{
			const float segTime = durations[i];
			totalShapingClocks += segTime;
			extraClocksAtStart += (1.0 - coefficients[i]) * segTime;
			extraClocksAtEnd += coefficients[i] * segTime;
			const float speedChange = coefficients[i] * segTime;
			extraDistanceAtStart += (1.0 - coefficients[i]) * (u + 0.5 * speedChange) * segTime;
			u += speedChange;
		}
	}

	minimumShapingStartOriginalClocks = totalShapingClocks - extraClocksAtStart + (MinimumMiddleSegmentTime * StepTimer::StepClockRate);
	minimumShapingEndOriginalClocks = totalShapingClocks - extraClocksAtEnd + (MinimumMiddleSegmentTime * StepTimer::StepClockRate);
	minimumNonOverlappedOriginalClocks = (totalShapingClocks * 2) - extraClocksAtStart - extraClocksAtEnd + (MinimumMiddleSegmentTime * StepTimer::StepClockRate);

	{
		float v = 0.0;
		for (int i = numExtraImpulses - 1; i >= 0; --i)
		{
			const float segTime = durations[i];
			const float speedChange = (1.0 - coefficients[i]) * segTime;
			extraDistanceAtEnd += coefficients[i] * (v - 0.5 * speedChange) * segTime;
			v -= speedChange;
		}
	}

	if (numExtraImpulses != 0)
	{
		overlappedShapingClocks = 2 * totalShapingClocks;
		// Calculate the clocks and coefficients needed when we shape the start of acceleration/deceleration and then immediately shape the end
		float maxVal = 0.0;
		for (unsigned int i = 0; i < numExtraImpulses; ++i)
		{
			overlappedDurations[i] = overlappedDurations[i + numExtraImpulses] = durations[i];
			float val = coefficients[i];
			overlappedCoefficients[i] = val;
			if (val > maxVal)
			{
				maxVal = val;
			}
			val = 1.0 - val;
			overlappedCoefficients[i + numExtraImpulses] = val;
			if (val > maxVal)
			{
				maxVal = val;
			}
		}

		// Now scale the values by maxVal so that the highest coefficient is 1.0, and calculate the total distance per unit acceleration
		overlappedDistancePerA = 0.0;
		float u = 0.0;
		for (unsigned int i = 0; i < 2 * numExtraImpulses; ++i)
		{
			overlappedCoefficients[i] /= maxVal;
			const float speedChange = overlappedCoefficients[i] * overlappedDurations[i];
			overlappedDistancePerA += (u + 0.5 * speedChange) * overlappedDurations[i];
			u += speedChange;
		}
		overlappedDeltaVPerA = u;
	}
}

// Calculate up the shaped segments for a move. The only field in PrepParams that this ever modifies is the debugPrint flag.
void AxisShaper::GetRemoteSegments(DDA& dda, PrepParams& params) const noexcept
{
	// Do the acceleration phase
	float accelDistanceExTopSpeed;
	float effectiveAccelTime;
	if (params.accelClocks == 0)
	{
		accelDistanceExTopSpeed = 0.0;
		effectiveAccelTime = 0.0;
	}
	else
	{
		float accelDistanceExTopSpeedPerA;									// the distance needed for acceleration minus the contribution from the top speed, per unit acceleration, in stepClocks^2
		if (params.shapingPlan.shapeAccelOverlapped)
		{
			effectiveAccelTime = overlappedDeltaVPerA;
			accelDistanceExTopSpeedPerA = overlappedDistancePerA - effectiveAccelTime * params.accelClocks;
		}
		else
		{
			effectiveAccelTime = params.accelClocks;
			accelDistanceExTopSpeedPerA = 0.0;
			if (params.shapingPlan.shapeAccelEnd)
			{
				effectiveAccelTime -= extraClocksAtEnd;
				accelDistanceExTopSpeedPerA += extraDistanceAtEnd;
			}
			if (params.shapingPlan.shapeAccelStart)
			{
				effectiveAccelTime -= extraClocksAtStart;
				accelDistanceExTopSpeedPerA += extraDistanceAtStart - effectiveAccelTime * extraClocksAtStart;
			}
			accelDistanceExTopSpeedPerA -= 0.5 * fsquare(effectiveAccelTime);
		}
		accelDistanceExTopSpeed = accelDistanceExTopSpeedPerA * params.acceleration;
	}

	// Do the deceleration phase
	float decelDistanceExTopSpeed;
	float effectiveDecelTime;
	if (params.decelClocks == 0)
	{
		decelDistanceExTopSpeed = 0.0;
		effectiveDecelTime = 0.0;
	}
	else
	{
		float decelDistanceExTopSpeedPerA;									// the distance needed for deceleration minus the contribution from the top speed
		if (params.shapingPlan.shapeDecelOverlapped)
		{
			effectiveDecelTime = overlappedDeltaVPerA;
			decelDistanceExTopSpeedPerA = -overlappedDistancePerA;
		}
		else
		{
			effectiveDecelTime = params.decelClocks;
			decelDistanceExTopSpeedPerA = 0.0;
			if (params.shapingPlan.shapeDecelStart)
			{
				effectiveDecelTime -= extraClocksAtStart;
				decelDistanceExTopSpeedPerA -= extraDistanceAtStart;
			}
			if (params.shapingPlan.shapeDecelEnd)
			{
				effectiveDecelTime -= extraClocksAtEnd;
				decelDistanceExTopSpeedPerA -= extraDistanceAtEnd + effectiveDecelTime * extraClocksAtEnd;
			}
			decelDistanceExTopSpeedPerA -= 0.5 * fsquare(effectiveDecelTime);
		}
		decelDistanceExTopSpeed = decelDistanceExTopSpeedPerA * params.deceleration;
	}

	dda.topSpeed = (1.0 - accelDistanceExTopSpeed - decelDistanceExTopSpeed)/dda.clocksNeeded;
	dda.startSpeed = dda.topSpeed - params.acceleration * effectiveAccelTime;
	dda.endSpeed = dda.topSpeed - params.deceleration * effectiveDecelTime;
	params.accelDistance =      accelDistanceExTopSpeed + dda.topSpeed * params.accelClocks;
	const float decelDistance = decelDistanceExTopSpeed + dda.topSpeed * params.decelClocks;
	params.decelStartDistance =  1.0 - decelDistance;

#ifdef DEBUG
	debugPrintf("plan=%u ad=%.4e dd=%.4e\n", (unsigned int)params.shapingPlan.condensedPlan, (double)params.accelDistance, (double)decelDistance);
#endif

	MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
	MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
	dda.segments = FinishShapedSegments(dda, params, accelSegs, decelSegs);
}

// If there is an acceleration phase, generate the acceleration segments according to the plan, and set the number of acceleration segments in the plan
MoveSegment *AxisShaper::GetAccelerationSegments(const DDA& dda, PrepParams& params) const noexcept
{
	if (params.accelDistance > 0.0)
	{
		if (params.shapingPlan.shapeAccelOverlapped)
		{
			MoveSegment *accelSegs = nullptr;
			float segStartSpeed = dda.topSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				accelSegs = MoveSegment::Allocate(accelSegs);
				const float acceleration = params.acceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				const float speedIncrease = acceleration * segTime;
				segStartSpeed -= speedIncrease;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * speedIncrease)) * segTime;
				accelSegs->SetNonLinear(segLen, segTime, b, c, acceleration);
			}
			return accelSegs;
		}

		float accumulatedSegTime = 0.0;
		float endDistance = params.accelDistance;
		MoveSegment *endAccelSegs = nullptr;
		if (params.shapingPlan.shapeAccelEnd)
		{
			// Shape the end of the acceleration
			float segStartSpeed = dda.topSpeed;
			for (unsigned int i = numExtraImpulses; i != 0; )
			{
				--i;
				endAccelSegs = MoveSegment::Allocate(endAccelSegs);
				const float acceleration = params.acceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				const float speedIncrease = acceleration * segTime;
				segStartSpeed -= speedIncrease;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * speedIncrease)) * segTime;
				endDistance -= segLen;
				endAccelSegs->SetNonLinear(segLen, segTime, b, c, acceleration);
			}
			accumulatedSegTime += totalShapingClocks;
		}

		float startDistance = 0.0;
		float startSpeed = dda.startSpeed;
		MoveSegment *startAccelSegs = nullptr;
		if (params.shapingPlan.shapeAccelStart)
		{
			// Shape the start of the acceleration
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float acceleration = params.acceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float speedIncrease = acceleration * segTime;
				const float segLen = (startSpeed + (0.5 * speedIncrease)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c, acceleration);
				if (startAccelSegs == nullptr)
				{
					startAccelSegs = seg;
				}
				else
				{
					startAccelSegs->AddToTail(seg);
				}
				startSpeed += acceleration * segTime;
			}
			accumulatedSegTime += totalShapingClocks;
		}

		// Do the constant acceleration part
		if (endDistance > startDistance)
		{
			endAccelSegs = MoveSegment::Allocate(endAccelSegs);
			const float b = startSpeed/(-params.acceleration);
			const float c = 2.0/params.acceleration;
			const float segTime = params.accelClocks - accumulatedSegTime;
			endAccelSegs->SetNonLinear(endDistance - startDistance, segTime, b, c, params.acceleration);
		}
		else
		{
#ifdef DEBUG
			debugPrintf("Missing steady accel segment\n");
#endif
			params.shapingPlan.debugPrint = true;
		}

		if (startAccelSegs == nullptr)
		{
			return endAccelSegs;
		}

		if (endAccelSegs != nullptr)
		{
			startAccelSegs->AddToTail(endAccelSegs);
		}
		return startAccelSegs;
	}

	return nullptr;
}

// If there is a deceleration phase, generate the deceleration segments according to the plan, and set the number of deceleration segments in the plan
MoveSegment *AxisShaper::GetDecelerationSegments(const DDA& dda, PrepParams& params) const noexcept
{
	if (params.decelStartDistance < dda.totalDistance)
	{
		if (params.shapingPlan.shapeDecelOverlapped)
		{
			MoveSegment *decelSegs = nullptr;
			float segStartSpeed = dda.endSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				decelSegs = MoveSegment::Allocate(decelSegs);
				const float deceleration = params.deceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				const float speedDecrease = deceleration * segTime;
				segStartSpeed += speedDecrease;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * speedDecrease)) * segTime;
				decelSegs->SetNonLinear(segLen, segTime, b, c, -deceleration);
			}
			return decelSegs;
		}

		float accumulatedSegTime = 0.0;
		float endDistance = dda.totalDistance;
		MoveSegment *endDecelSegs = nullptr;
		if (params.shapingPlan.shapeDecelEnd)
		{
			// Shape the end of the deceleration
			float segStartSpeed = dda.endSpeed;
			for (unsigned int i = numExtraImpulses; i != 0; )
			{
				--i;
				endDecelSegs = MoveSegment::Allocate(endDecelSegs);
				const float deceleration = params.deceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				const float speedDecreasee = deceleration * segTime;
				segStartSpeed += speedDecreasee;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * speedDecreasee)) * segTime;
				endDecelSegs->SetNonLinear(segLen, segTime, b, c, -deceleration);
				endDistance -= segLen;
			}
			accumulatedSegTime += totalShapingClocks;
		}

		float startDistance = params.decelStartDistance;
		float startSpeed = dda.topSpeed;
		MoveSegment *startDecelSegs = nullptr;
		if (params.shapingPlan.shapeDecelStart)
		{
			// Shape the start of the deceleration
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float deceleration = params.deceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float speedDecrease = deceleration * segTime;
				const float segLen = (startSpeed + (-0.5 * speedDecrease)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c, -deceleration);
				if (startDecelSegs == nullptr)
				{
					startDecelSegs = seg;
				}
				else
				{
					startDecelSegs->AddToTail(seg);
				}
				startSpeed -= deceleration * segTime;
			}
			accumulatedSegTime += totalShapingClocks;
		}

		// Do the constant deceleration part
		if (endDistance > startDistance)
		{
			endDecelSegs = MoveSegment::Allocate(endDecelSegs);
			const float b = startSpeed/params.deceleration;
			const float c = -2.0/params.deceleration;
			const float segTime = params.decelClocks - accumulatedSegTime;
			endDecelSegs->SetNonLinear(endDistance - startDistance, segTime, b, c, -params.deceleration);
		}
		else
		{
#ifdef DEBUG
			debugPrintf("Missing steady decel segment\n");
#endif
			params.shapingPlan.debugPrint = true;
		}

		if (startDecelSegs == nullptr)
		{
			return endDecelSegs;
		}

		if (endDecelSegs != nullptr)
		{
			startDecelSegs->AddToTail(endDecelSegs);
		}
		return startDecelSegs;
	}

	return nullptr;
}

// Generate the steady speed segment (if any), tack all the segments together, and return them
// Must set up params.steadyClocks before calling this
MoveSegment *AxisShaper::FinishShapedSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
{
	if (params.steadyClocks > 0.0)
	{
		// Insert a steady speed segment before the deceleration segments
		decelSegs = MoveSegment::Allocate(decelSegs);
		const float c = 1.0/dda.topSpeed;
		decelSegs->SetLinear(params.decelStartDistance - params.accelDistance, params.steadyClocks, c);
	}

	if (accelSegs != nullptr)
	{
		if (decelSegs != nullptr)
		{
			accelSegs->AddToTail(decelSegs);
		}
		return accelSegs;
	}

	return decelSegs;
}

// Calculate the move segments when input shaping is not in use
/*static*/ MoveSegment *AxisShaper::GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept
{
	// Deceleration phase
	MoveSegment * tempSegments;
	if (params.decelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(nullptr);
		const float b = dda.topSpeed/params.deceleration;
		const float c = -2.0/params.deceleration;
		tempSegments->SetNonLinear(dda.totalDistance - params.decelStartDistance, params.decelClocks, b, c, -params.deceleration);
	}
	else
	{
		tempSegments = nullptr;
	}

	// Steady speed phase
	if (params.steadyClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float c = 1.0/dda.topSpeed;
		tempSegments->SetLinear(params.decelStartDistance - params.accelDistance, params.steadyClocks, c);
	}

	// Acceleration phase
	if (params.accelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float b = dda.startSpeed/(-params.acceleration);
		const float c = 2.0/params.acceleration;
		tempSegments->SetNonLinear(params.accelDistance, params.accelClocks, b, c, params.acceleration);
	}

	return tempSegments;
}

#endif

// End
