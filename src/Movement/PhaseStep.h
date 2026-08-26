/*
 * PhaseStep.h
 *
 *  Created on: 25 Aug 2026
 *      Author: Christian
 */

#ifndef SRC_MOVEMENT_PHASESTEP_H_
#define SRC_MOVEMENT_PHASESTEP_H_

#include <RepRapFirmware.h>

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP

// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position = 0.0;
	float speed = 0.0;
	float acceleration = 0.0;
};

#endif

#if SUPPORT_PHASE_STEPPING

# include <Movement/StepperDrivers/DriverMode.h>

// One term of the phase correction that is added to the electrical angle before the coil currents are computed, see M970.3
struct PhaseCorrectionHarmonic
{
	uint8_t harmonic;			// harmonic of the electrical cycle, 0 = unused entry
	float magnitude;			// magnitude in phase units, where 4096 is a full electrical cycle
	uint16_t phase;				// phase offset in phase units
};

constexpr size_t MaxPhaseCorrectionHarmonics = 4;
constexpr unsigned int MaxPhaseCorrectionHarmonic = 16;

class PhaseStep
{
public:
	friend class Move;
	friend class DriveMovement;

	void SetStandstillCurrent(float percent) noexcept;

	// Phase correction, shared by all instances because it is a property of the driver
	static GCodeResult ConfigureCorrection(size_t driver, unsigned int harmonic, bool seenMagnitude, float magnitudeDegrees, bool seenPhase, float phaseDegrees, const StringRef& reply) noexcept;
	static void AppendCorrections(size_t driver, const StringRef& reply) noexcept;
	static int32_t GetCorrection(size_t driver, uint32_t phase) noexcept;

	// Methods called by the motion system
	void InstanceControlLoop(size_t driver) noexcept;
	void SetEnabled(bool enable) noexcept { enabled = enable; }
	bool IsEnabled() const noexcept { return enabled; }
	void UpdatePhaseOffset(size_t driver) noexcept;
	void SetPhaseOffset(size_t driver, uint16_t offset) noexcept;
	uint16_t GetPhaseOffset(size_t driver) noexcept;
	float CalculateCurrentFraction() noexcept;

	// Configuration methods
	void SetKv(float newKv) noexcept { Kv = newKv; }
	void SetKa(float newKa) noexcept { Ka = newKa; }
	float GetKv() const noexcept { return Kv; }
	float GetKa() const noexcept { return Ka; }

private:
	static constexpr float DefaultHoldCurrentFraction = 0.71;	// the minimum fraction of the requested current that we apply when holding position

	void SetMotorPhase(size_t driver, uint16_t phase, float magnitude) noexcept;
	uint16_t CalculateStepPhase(size_t driver) noexcept;

	bool enabled = false;
	DriverMode modeBeforeEnabled = DriverMode::spreadCycle;	// the driver mode to restore when phase stepping is disabled

	// Holding current, and variables derived from it
	float holdCurrentFraction = DefaultHoldCurrentFraction;	// The minimum holding current when stationary
	float Kv = 1000.0;										// The velocity feedforward constant
	float Ka = 50000.0;										// The acceleration feedforward constant

	// Working variables
	// These variables are all used to calculate the required motor currents
	MotionParameters mParams;								// the target position, speed and acceleration

	float PIDVTerm;											// Velocity feedforward term
	float PIDATerm;											// Acceleration feedforward term
	float PIDControlSignal;									// The overall signal from the PID controller

	float currentFraction;
	int16_t coilA;											// The current to run through coil A
	int16_t coilB;											// The current to run through coil B
};

#endif

#endif /* SRC_MOVEMENT_PHASESTEP_H_ */
