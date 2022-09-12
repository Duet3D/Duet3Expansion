/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "LinearDeltaKinematics.h"

#include <Movement/Move.h>
#include <Platform/Platform.h>

LinearDeltaKinematics::LinearDeltaKinematics() : Kinematics(KinematicsType::linearDelta, -1.0, 0.0, true), numTowers(UsualNumTowers)
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
	radius = DefaultDeltaRadius;
	xTilt = yTilt = 0.0;
	printRadius = DefaultPrintRadius;
	homedHeight = DefaultDeltaHomedHeight;
    doneAutoCalibration = false;

	for (size_t axis = 0; axis < UsualNumTowers; ++axis)
	{
		angleCorrections[axis] = 0.0;
	}

	for (size_t axis = 0; axis < MaxTowers; ++axis)
	{
		diagonals[axis] = DefaultDiagonal;
		towerX[axis] = towerY[axis] = 0.0;
		endstopAdjustments[axis] = 0.0;
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

	// Calculate the base carriage heights when the printer is homed, i.e. the carriages are at the endstops, and the always-reachable height
	for (size_t axis = 0; axis < numTowers; ++axis)
	{
		D2[axis] = fsquare(diagonals[axis]);
		homedCarriageHeights[axis] = homedHeight
									+ fastSqrtf(D2[axis] - ((axis < UsualNumTowers) ? fsquare(radius) : fsquare(towerX[axis]) + fsquare(towerY[axis])))
									+ endstopAdjustments[axis];
	}

	printRadiusSquared = fsquare(printRadius);
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float LinearDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	if (axis < numTowers)
	{
		return fastSqrtf(D2[axis] - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
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
void LinearDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[XYZ_AXES]) const
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
	const float C = fsquare(S + towerX[DELTA_A_AXIS] * Q) + fsquare(P - towerY[DELTA_A_AXIS] * Q) + (fsquare(Ha) - D2[DELTA_A_AXIS]) * Q2;

	const float z = (minusHalfB - fastSqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z - ((machinePos[X_AXIS] * xTilt) + (machinePos[Y_AXIS] * yTilt));
}

// Convert Cartesian coordinates to motor steps
bool LinearDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	bool ok = true;
	for (size_t axis = 0; axis < numTowers; ++axis)
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
	for (size_t axis = numTowers; axis < numVisibleAxes; ++axis)
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
	for (size_t drive = numTowers; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return the type of motion computation needed by an axis
MotionType LinearDeltaKinematics::GetMotionType(size_t axis) const
{
	return (axis < numTowers) ? MotionType::segmentFreeDelta : MotionType::linear;
}

// Print all the parameters for debugging
void LinearDeltaKinematics::PrintParameters(const StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonals",
		(double)endstopAdjustments[DELTA_A_AXIS], (double)endstopAdjustments[DELTA_B_AXIS], (double)endstopAdjustments[DELTA_C_AXIS], (double)homedHeight);
	for (size_t tower = 0; tower < numTowers; ++tower)
	{
		reply.catf("%c%.3f", (tower == 0) ? ' ' : ':', (double)diagonals[tower]);
	}
	reply.catf(" radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
		(double)radius,
		(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS],
		(double)(xTilt * 100.0), (double)(yTilt * 100.0));
}

// End
