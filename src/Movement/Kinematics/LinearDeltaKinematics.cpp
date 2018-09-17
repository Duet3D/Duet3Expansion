/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "LinearDeltaKinematics.h"

#include "Movement/Move.h"
#include "Platform.h"

LinearDeltaKinematics::LinearDeltaKinematics() : Kinematics(KinematicsType::linearDelta, -1.0, 0.0, true)
{
	Init();
}

// Return the name of the current kinematics
const char *LinearDeltaKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "delta" : "Linear delta";
}

void LinearDeltaKinematics::Init()
{
	diagonal = DefaultDiagonal;
	radius = DefaultDeltaRadius;
	xTilt = yTilt = 0.0;
	printRadius = DefaultPrintRadius;
	homedHeight = DefaultDeltaHomedHeight;
    doneAutoCalibration = false;

	for (size_t axis = 0; axis < DELTA_AXES; ++axis)
	{
		angleCorrections[axis] = 0.0;
		endstopAdjustments[axis] = 0.0;
		towerX[axis] = towerY[axis] = 0.0;
	}

	Recalc();
}

void LinearDeltaKinematics::Recalc()
{
	towerX[DELTA_A_AXIS] = -(radius * cosf((30 + angleCorrections[DELTA_A_AXIS]) * DegreesToRadians));
	towerY[DELTA_A_AXIS] = -(radius * sinf((30 + angleCorrections[DELTA_A_AXIS]) * DegreesToRadians));
	towerX[DELTA_B_AXIS] = +(radius * cosf((30 - angleCorrections[DELTA_B_AXIS]) * DegreesToRadians));
	towerY[DELTA_B_AXIS] = -(radius * sinf((30 - angleCorrections[DELTA_B_AXIS]) * DegreesToRadians));
	towerX[DELTA_C_AXIS] = -(radius * sinf(angleCorrections[DELTA_C_AXIS] * DegreesToRadians));
	towerY[DELTA_C_AXIS] = +(radius * cosf(angleCorrections[DELTA_C_AXIS] * DegreesToRadians));

	Xbc = towerX[DELTA_C_AXIS] - towerX[DELTA_B_AXIS];
	Xca = towerX[DELTA_A_AXIS] - towerX[DELTA_C_AXIS];
	Xab = towerX[DELTA_B_AXIS] - towerX[DELTA_A_AXIS];
	Ybc = towerY[DELTA_C_AXIS] - towerY[DELTA_B_AXIS];
	Yca = towerY[DELTA_A_AXIS] - towerY[DELTA_C_AXIS];
	Yab = towerY[DELTA_B_AXIS] - towerY[DELTA_A_AXIS];
	coreFa = fsquare(towerX[DELTA_A_AXIS]) + fsquare(towerY[DELTA_A_AXIS]);
	coreFb = fsquare(towerX[DELTA_B_AXIS]) + fsquare(towerY[DELTA_B_AXIS]);
	coreFc = fsquare(towerX[DELTA_C_AXIS]) + fsquare(towerY[DELTA_C_AXIS]);
	Q = (Xca * Yab - Xab * Yca) * 2;
	Q2 = fsquare(Q);
	D2 = fsquare(diagonal);

	// Calculate the base carriage height when the printer is homed, i.e. the carriages are at the endstops less the corrections
	const float tempHeight = diagonal;		// any sensible height will do here
	float machinePos[DELTA_AXES];
	ForwardTransform(tempHeight, tempHeight, tempHeight, machinePos);
	homedCarriageHeight = homedHeight + tempHeight - machinePos[Z_AXIS];
	printRadiusSquared = fsquare(printRadius);
}

// Make the average of the endstop adjustments zero, without changing the individual homed carriage heights
void LinearDeltaKinematics::NormaliseEndstopAdjustments()
{
	const float eav = (endstopAdjustments[DELTA_A_AXIS] + endstopAdjustments[DELTA_B_AXIS] + endstopAdjustments[DELTA_C_AXIS])/3.0;
	endstopAdjustments[DELTA_A_AXIS] -= eav;
	endstopAdjustments[DELTA_B_AXIS] -= eav;
	endstopAdjustments[DELTA_C_AXIS] -= eav;
	homedHeight += eav;
	homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float LinearDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	if (axis < DELTA_AXES)
	{
		return sqrtf(D2 - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
			 + machinePos[Z_AXIS]
			 + (machinePos[X_AXIS] * xTilt)
			 + (machinePos[Y_AXIS] * yTilt);
	}
	else
	{
		return machinePos[axis];
	}
}

// Calculate the Cartesian coordinates from the motor coordinates
void LinearDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const
{
	const float Fa = coreFa + fsquare(Ha);
	const float Fb = coreFb + fsquare(Hb);
	const float Fc = coreFc + fsquare(Hc);

	// Calculate PQRSU such that x = -(S - Uz)/Q, y = (P - Rz)/Q
	const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
	const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);
	const float R = ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc)) * 2;
	const float U = ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc)) * 2;

	const float R2 = fsquare(R), U2 = fsquare(U);

	const float A = U2 + R2 + Q2;
	const float minusHalfB = S * U + P * R + Ha * Q2 + towerX[DELTA_A_AXIS] * U * Q - towerY[DELTA_A_AXIS] * R * Q;
	const float C = fsquare(S + towerX[DELTA_A_AXIS] * Q) + fsquare(P - towerY[DELTA_A_AXIS] * Q) + (fsquare(Ha) - D2) * Q2;

	const float z = (minusHalfB - sqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z - ((machinePos[X_AXIS] * xTilt) + (machinePos[Y_AXIS] * yTilt));
}

// Convert Cartesian coordinates to motor steps
bool LinearDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	bool ok = true;
	for (size_t axis = 0; axis < min<size_t>(numVisibleAxes, DELTA_AXES); ++axis)
	{
		const float pos = Transform(machinePos, axis);
		if (std::isnan(pos) || std::isinf(pos))
		{
			ok = false;
		}
		else
		{
			motorPos[axis] = lrintf(pos * stepsPerMm[axis]);
		}
	}

	// Transform any additional axes linearly
	for (size_t axis = DELTA_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return ok;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void LinearDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	ForwardTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = DELTA_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool LinearDeltaKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
bool LinearDeltaKinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated) const
{
	const AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
	bool limited = false;
	if ((axesHomed & allAxes) == allAxes)
	{
		// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(coords[X_AXIS]) + fsquare(coords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
			coords[X_AXIS] *= factor;
			coords[Y_AXIS] *= factor;
			limited = true;
		}

		if (coords[Z_AXIS] < Platform::AxisMinimum(Z_AXIS))
		{
			coords[Z_AXIS] = Platform::AxisMinimum(Z_AXIS);
			limited = true;
		}
		else
		{
			// Determine the maximum reachable height at this radius, in the worst case when the head is on a radius to a tower
			const float maxHeight = homedCarriageHeight - sqrtf(D2 - fsquare(radius - sqrtf(diagonalSquared)));
			if (coords[Z_AXIS] > maxHeight)
			{
				coords[Z_AXIS] = maxHeight;
				limited = true;
			}
		}
	}

	// Limit any additional axes according to the M208 limits
	if (LimitPositionFromAxis(coords, Z_AXIS + 1, numVisibleAxes, axesHomed))
	{
		limited = true;
	}

	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void LinearDeltaKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
	positions[Z_AXIS] = homedHeight;
}

// Return the type of motion computation needed by an axis
MotionType LinearDeltaKinematics::GetMotionType(size_t axis) const
{
	return (axis < DELTA_AXES) ? MotionType::segmentFreeDelta : MotionType::linear;
}

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower endstop adjustments
// 3 = delta radius
// 4 = X tower correction
// 5 = Y tower correction
// 6 = diagonal rod length
// 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
floatc_t LinearDeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const
{
	const float perturb = 0.2;			// perturbation amount in mm or degrees
	LinearDeltaKinematics hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		// Endstop corrections
		break;

	case 3:
		hiParams.radius += perturb;
		loParams.radius -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 4:
		hiParams.angleCorrections[DELTA_A_AXIS] += perturb;
		loParams.angleCorrections[DELTA_A_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.angleCorrections[DELTA_B_AXIS] += perturb;
		loParams.angleCorrections[DELTA_B_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 6:
		hiParams.diagonal += perturb;
		loParams.diagonal -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 7:
	case 8:
		// X and Y tilt
		break;
	}

	float newPos[DELTA_AXES];
	hiParams.ForwardTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	if (deriv == 7)
	{
		return -newPos[X_AXIS]/printRadius;
	}
	if (deriv == 8)
	{
		return -newPos[Y_AXIS]/printRadius;
	}

	const float zHi = newPos[Z_AXIS];
	loParams.ForwardTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
	const float zLo = newPos[Z_AXIS];

	return ((floatc_t)zHi - (floatc_t)zLo)/(floatc_t)(2 * perturb);
}

// Perform 3, 4, 6, 7, 8 or 9-factor adjustment.
// The input vector contains the following parameters in this order:
//  X, Y and Z endstop adjustments
//  Delta radius
//  X tower position adjustment
//  Y tower position adjustment
//  Diagonal rod length adjustment - omitted if doing 8-factor calibration (remainder are moved down)
//  X tilt adjustment
//  Y tilt adjustment
void LinearDeltaKinematics::Adjust(size_t numFactors, const floatc_t v[])
{
	const float oldCarriageHeightA = GetHomedCarriageHeight(DELTA_A_AXIS);	// save for later

	// Update endstop adjustments
	endstopAdjustments[DELTA_A_AXIS] += (float)v[0];
	endstopAdjustments[DELTA_B_AXIS] += (float)v[1];
	endstopAdjustments[DELTA_C_AXIS] += (float)v[2];
	NormaliseEndstopAdjustments();

	if (numFactors >= 4)
	{
		radius += (float)v[3];

		if (numFactors >= 6)
		{
			angleCorrections[DELTA_A_AXIS] += (float)v[4];
			angleCorrections[DELTA_B_AXIS] += (float)v[5];

			if (numFactors == 7 || numFactors == 9)
			{
				diagonal += (float)v[6];
			}

			if (numFactors == 8)
			{
				xTilt += (float)v[6]/printRadius;
				yTilt += (float)v[7]/printRadius;
			}
			else if (numFactors == 9)
			{
				xTilt += (float)v[7]/printRadius;
				yTilt += (float)v[8]/printRadius;
			}
		}

		Recalc();
	}

	// Adjusting the diagonal and the tower positions affects the homed carriage height.
	// We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
	const float heightError = GetHomedCarriageHeight(DELTA_A_AXIS) - oldCarriageHeightA - (float)v[0];
	homedHeight -= heightError;
	homedCarriageHeight -= heightError;

	// Note: if we adjusted the X and Y tilts, and there are any endstop adjustments, then the homed position won't be exactly in the centre
	// and changing the tilt will therefore affect the homed height. We ignore this for now. If it is ever significant, a second autocalibration
	// run will correct it.
}

// Print all the parameters for debugging
void LinearDeltaKinematics::PrintParameters(const StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonal %.3f radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
		(double)endstopAdjustments[DELTA_A_AXIS], (double)endstopAdjustments[DELTA_B_AXIS], (double)endstopAdjustments[DELTA_C_AXIS], (double)homedHeight, (double)diagonal, (double)radius,
		(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS], (double)(xTilt * 100.0), (double)(yTilt * 100.0));
}

// End
