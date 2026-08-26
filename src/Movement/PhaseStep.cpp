/*
 * PhaseStep.cpp
 *
 *  Created on: 25 Aug 2026
 *      Author: Christian
 *
 * Open loop commutation of stepper drivers in direct mode, ported from RepRapFirmware. The commanded phase is derived
 * from the motion parameters of the driver's DriveMovement, with velocity and acceleration feedforward setting the
 * motor current. The control loop is run from the TMC task before each SPI transfer.
 */

#include "PhaseStep.h"

#if SUPPORT_PHASE_STEPPING

# include <RTOSIface/RTOSIface.h>
# include <Movement/Move.h>
# include <Movement/StepperDrivers/SmartDrivers.h>
# include <ClosedLoop/Trigonometry.h>
# include <cmath>

static uint16_t currentPhase[NumDrivers] = { 0 };
static uint16_t phaseOffset[NumDrivers] = { 0 };			// The amount by which the phase should be offset for each driver
static PhaseCorrectionHarmonic phaseCorrections[NumDrivers][MaxPhaseCorrectionHarmonics] = { };

constexpr float PhaseUnitsPerDegree = 4096.0/360.0;

// Configure the phase correction of a driver via M970.3: harmonic of the electrical cycle, magnitude in degrees (0 removes the harmonic), phase offset in degrees
GCodeResult PhaseStep::ConfigureCorrection(size_t driver, unsigned int harmonic, bool seenMagnitude, float magnitudeDegrees, bool seenPhase, float phaseDegrees, const StringRef& reply) noexcept
{
	PhaseCorrectionHarmonic *_ecv_array const corrections = phaseCorrections[driver];
	PhaseCorrectionHarmonic *_ecv_null entry = nullptr;
	for (size_t i = 0; i < MaxPhaseCorrectionHarmonics; i++)
	{
		if (corrections[i].harmonic == harmonic)
		{
			entry = &corrections[i];
			break;
		}
	}

	if (seenMagnitude)
	{
		if (magnitudeDegrees == 0.0)
		{
			if (entry != nullptr)
			{
				entry->harmonic = 0;
			}
			return GCodeResult::ok;
		}
		if (entry == nullptr)
		{
			for (size_t i = 0; i < MaxPhaseCorrectionHarmonics; i++)
			{
				if (corrections[i].harmonic == 0)
				{
					entry = &corrections[i];
					entry->phase = 0;
					break;
				}
			}
			if (entry == nullptr)
			{
				reply.printf("Driver %u already has %u correction harmonics", driver, MaxPhaseCorrectionHarmonics);
				return GCodeResult::error;
			}
		}
		entry->harmonic = (uint8_t)harmonic;
		entry->magnitude = magnitudeDegrees * PhaseUnitsPerDegree;
	}
	else if (entry == nullptr)
	{
		reply.printf("Driver %u has no correction for harmonic %u", driver, harmonic);
		return GCodeResult::error;
	}

	if (seenPhase)
	{
		entry->phase = (uint16_t)lrintf(phaseDegrees * PhaseUnitsPerDegree) % 4096u;
	}
	return GCodeResult::ok;
}

// Append the configured phase corrections of a driver to the reply
void PhaseStep::AppendCorrections(size_t driver, const StringRef& reply) noexcept
{
	bool any = false;
	for (const PhaseCorrectionHarmonic& entry : phaseCorrections[driver])
	{
		if (entry.harmonic != 0)
		{
			reply.catf("%s S%u J%.3f O%.1f", (any) ? "," : "", entry.harmonic, (double)(entry.magnitude / PhaseUnitsPerDegree), (double)(entry.phase / PhaseUnitsPerDegree));
			any = true;
		}
	}
	if (!any)
	{
		reply.cat(" none");
	}
}

// Get the correction to add to the electrical angle of a driver, in phase units
int32_t PhaseStep::GetCorrection(size_t driver, uint32_t phase) noexcept
{
	float correction = 0.0;
	for (const PhaseCorrectionHarmonic& entry : phaseCorrections[driver])
	{
		if (entry.harmonic != 0)
		{
			float sine, cosine;
			Trigonometry::FastSinCos((uint16_t)((entry.harmonic * phase + entry.phase) % 4096u), sine, cosine);
			correction += entry.magnitude * sine * (1.0/248.0);
		}
	}
	return lrintf(correction);
}

// Set the motor currents and update the current phase.
// The phase must be taken modulo 4096 when computing the currents. Function Trigonometry::FastSinCos does that.
// 'magnitude' must be in range 0.0..1.0
void PhaseStep::SetMotorPhase(size_t driver, uint16_t phase, float magnitude) noexcept
{
	currentPhase[driver] = phase;
	float sine, cosine;
	Trigonometry::FastSinCos((uint16_t)((int32_t)phase + GetCorrection(driver, phase)), sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);
	SmartDrivers::SetMotorPhases(driver, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
}

// Set the standstill current fraction for this drive
void PhaseStep::SetStandstillCurrent(float percent) noexcept
{
	holdCurrentFraction = percent * 0.01;
}

void PhaseStep::InstanceControlLoop(size_t driver) noexcept
{
	if (unlikely(!enabled))
	{
		return;
	}

	const uint16_t commandedStepPhase = CalculateStepPhase(driver);
	SetMotorPhase(driver, commandedStepPhase, currentFraction);
}

void PhaseStep::UpdatePhaseOffset(size_t driver) noexcept
{
	AtomicCriticalSectionLocker lock;
	const uint16_t calculatedStepPhase = CalculateStepPhase(driver);
	phaseOffset[driver] = (currentPhase[driver] - (calculatedStepPhase - phaseOffset[driver])) % 4096u;
}

void PhaseStep::SetPhaseOffset(size_t driver, uint16_t offset) noexcept
{
	AtomicCriticalSectionLocker lock;
	phaseOffset[driver] = (offset) % 4096u;
}

uint16_t PhaseStep::GetPhaseOffset(size_t driver) noexcept
{
	return phaseOffset[driver];
}

// Calculate the phase from the current motion parameters. Move::GetCurrentMotion has already converted the position to full steps and applied the direction
uint16_t PhaseStep::CalculateStepPhase(size_t driver) noexcept
{
	const uint16_t calculatedStepPhase = (uint16_t)llrintf(mParams.position * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
	return (calculatedStepPhase + phaseOffset[driver]) % 4096u;
}

// Control the motor phase currents, returning the fraction of maximum current that we commanded
float PhaseStep::CalculateCurrentFraction() noexcept
{
	// In this mode the PID terms are not used and the A and V terms are independent of the loop time
	constexpr float scalingFactor = 100.0;
	constexpr float scalingFactorSqr = scalingFactor * scalingFactor;
	PIDVTerm = mParams.speed * Kv * scalingFactor;
	PIDATerm = mParams.acceleration * Ka * scalingFactorSqr;
	PIDControlSignal = min<float>(fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

	currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
	return currentFraction;
}

#endif

// End
