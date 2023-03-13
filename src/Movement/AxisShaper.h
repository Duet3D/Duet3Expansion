/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#include <InputShaperPlan.h>

class DDA;
class PrepParams;
class MoveSegment;

struct CanMessageSetInputShaping;
struct CanMessageMovementLinear;

class AxisShaper
{
public:
	AxisShaper() noexcept;

	// Handle a request from the master board to set input shaping parameters
	GCodeResult EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept;

	// Calculate the shaped segments for a move
	void GetRemoteSegments(DDA& dda, PrepParams& paramsg) const noexcept;

	// Calculate the move segments when input shaping is not used
	static MoveSegment *GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept;

private:
	void CalculateDerivedParameters() noexcept;
	MoveSegment *GetAccelerationSegments(const DDA& dda, PrepParams& params) const noexcept;
	MoveSegment *GetDecelerationSegments(const DDA& dda, PrepParams& params) const noexcept;
	MoveSegment *FinishShapedSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept;

	static constexpr unsigned int MaxExtraImpulses = 4;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.1;
	static constexpr float MinimumMiddleSegmentTime = 5.0/1000.0;	// minimum length of the segment between shaped start and shaped end of an acceleration or deceleration

	// Parameters that fully define the shaping
	unsigned int numExtraImpulses;						// the number of extra impulses
	float coefficients[MaxExtraImpulses];				// the coefficients of all the impulses
	float durations[MaxExtraImpulses];					// the duration in step clocks of each impulse

	// Secondary parameters, calculated from the primary ones
	float totalShapingClocks;							// the total input shaping time in step clocks
	float minimumShapingStartOriginalClocks;			// the minimum acceleration/deceleration time for which we can shape the start, without changing the acceleration/deceleration
	float minimumShapingEndOriginalClocks;				// the minimum acceleration/deceleration time for which we can shape the start, without changing the acceleration/deceleration
	float minimumNonOverlappedOriginalClocks;			// the minimum original acceleration or deceleration time using non-overlapped start and end shaping
	float extraClocksAtStart;							// the extra time needed to shape the start of acceleration or deceleration
	float extraClocksAtEnd;								// the extra time needed to shape the end of acceleration or deceleration
	float extraDistanceAtStart;							// the extra distance per unit acceleration to shape the start of acceleration or deceleration, less the initial velocity contribution
	float extraDistanceAtEnd;							// the extra distance per unit acceleration to shape the end of acceleration or deceleration, less the final velocity contribution
	float overlappedDurations[2 * MaxExtraImpulses];	// the duration in step clocks of each impulse of an overlapped acceleration or deceleration
	float overlappedCoefficients[2 * MaxExtraImpulses];	// the coefficients if we use a shaped start immediately followed by a shaped end
	float overlappedShapingClocks;						// the acceleration or deceleration duration when we use overlapping, in step clocks
	float overlappedDeltaVPerA;							// the effective acceleration time (velocity change per unit acceleration) when we use overlapping, in step clocks
	float overlappedDistancePerA;						// the distance needed by an overlapped acceleration or deceleration, less the initial velocity contribution
};

#endif

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
