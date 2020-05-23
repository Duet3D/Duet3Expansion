/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef LINEARDELTAKINEMATICS_H_
#define LINEARDELTAKINEMATICS_H_

#include "RepRapFirmware.h"
#include "Kinematics.h"

// Class to hold the parameter for a delta machine.
class LinearDeltaKinematics : public Kinematics
{
public:
	// Constructors
	LinearDeltaKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	MotionType GetMotionType(size_t axis) const override;

    // Public functions specific to this class
	float GetDiagonalSquared(size_t tower) const { return D2[tower]; }
    float GetTowerX(size_t axis) const { return towerX[axis]; }
    float GetTowerY(size_t axis) const { return towerY[axis]; }
	float GetHomedHeight() const { return homedHeight; }

private:
	void Init();
	void Recalc();
    float Transform(const float headPos[], size_t axis) const;						// Calculate the motor position for a single tower from a Cartesian coordinate
    void ForwardTransform(float Ha, float Hb, float Hc, float headPos[XYZ_AXES]) const;	// Calculate the Cartesian position from the motor positions

	void PrintParameters(const StringRef& reply) const;								// Print all the parameters for debugging

	static constexpr size_t MaxTowers = 6;				// maximum number of delta towers
	static constexpr size_t UsualNumTowers = 3;			// the usual number of towers, which are the ones we use for forward kinematics and the ones we calibrate

	// Axis names used internally
	static constexpr size_t DELTA_A_AXIS = 0;
	static constexpr size_t DELTA_B_AXIS = 1;
	static constexpr size_t DELTA_C_AXIS = 2;

	// Delta parameter defaults in mm
	static constexpr float DefaultDiagonal = 215.0;
	static constexpr float DefaultDeltaRadius = 105.6;
	static constexpr float DefaultPrintRadius = 80.0;
	static constexpr float DefaultDeltaHomedHeight = 240.0;

	// Core parameters
	size_t numTowers;
	float diagonals[MaxTowers];							// The diagonal rod lengths
	float radius;										// The nominal delta radius, before any fine tuning of tower positions
	float angleCorrections[MaxTowers];					// Tower position corrections
	float endstopAdjustments[MaxTowers];				// How much above or below the ideal position each endstop is
	float printRadius;
	float homedHeight;
	float xTilt, yTilt;									// How much we need to raise Z for each unit of movement in the +X and +Y directions

	// Derived values
	float towerX[MaxTowers];							// The X coordinate of each tower
	float towerY[MaxTowers];							// The Y coordinate of each tower
	float printRadiusSquared;
	float homedCarriageHeights[MaxTowers];
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreFa, coreFb, coreFc;
	float Q, Q2;
	float D2[MaxTowers];

	bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif /* LINEARDELTAKINEMATICS_H_ */
