/*
 * AbsoluteEncoder.cpp
 *
 *  Created on: 15 Oct 2022
 *      Author: David
 */

#include "AbsoluteEncoder.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/NonVolatileMemory.h>

AbsoluteEncoder::AbsoluteEncoder(float p_stepAngle, unsigned int p_resolutionBits) noexcept
	: Encoder((1u << p_resolutionBits) * p_stepAngle/360.0),
	  stepAngle(p_stepAngle),
	  resolutionBits(p_resolutionBits),
	  resolutionToLutShiftFactor((p_resolutionBits < LutResolutionBits) ? 0 : p_resolutionBits - LutResolutionBits)
{}

int32_t AbsoluteEncoder::GetReading(bool& err) noexcept
{
	uint32_t currentAngle = GetRawReading(err);
	if (err)
	{
		return fullRotations * GetMaxValue() + lastAngle;
	}

	// Apply LUT correction (if the LUT is loaded)
	if (LUTLoaded)
	{
		const size_t windowStartIndex = currentAngle >> resolutionToLutShiftFactor;
		const float windowStart = correctionLUT[windowStartIndex];
		const uint32_t windowOffset = currentAngle & (1u << (resolutionToLutShiftFactor - 1));
		//TODO use linear interpolation between bottom and top of window
		currentAngle = lrintf(windowStart + windowOffset);
	}

	// Accumulate the full rotations if one has occurred
	const int32_t difference = (int32_t)currentAngle - (int32_t)lastAngle;
	if (abs(difference) > (int32_t)(GetMaxValue()/2))
	{
		fullRotations += (difference < 0) - (difference > 0);	// Add -1 if diff > 0, +1 if diff < 0
	}
	lastAngle = currentAngle;

	// Return the position plus the accumulated rotations
	return (fullRotations * GetMaxValue() + lastAngle);
}

// Get the raw reading accounting for reverse polarity but not he correction table or the offset
uint32_t AbsoluteEncoder::GetRawReading(bool& err) noexcept
{
	const uint32_t reading = GetAbsolutePosition(err);
	return (IsBackwards()) ? GetMaxValue() - 1 - reading : reading;
}

bool AbsoluteEncoder::LoadLUT() noexcept
{
	NonVolatileMemory mem(NvmPage::closedLoop);
	if (!mem.GetClosedLoopDataWritten()) { return false; }

	PopulateLUT(mem);
	return true;
}

// Populate the LUT when we already have the nonvolatile data
void AbsoluteEncoder::PopulateLUT(NonVolatileMemory& mem) noexcept
{
	// Read back the table of harmonics from NVRAM and construct the lookup table.
	// The table maps actual position to encoder reading using the following mapping:
	//  angleRead = angleExpected + sum[i = 0 to (NumHarmonics - 1)](Si * sin(angleExpected * i) + Ci * cos(angleExpected * i))
	// which we can represent as:
	//  angleRead = f(angleExpected)
	// We want the inverse mapping, i.e. angleExpected = g(angleRead) where g is the inverse of f
	// Start by using the approximation: angleExpected = angleRead - sum[i = 0 to (NumHarmonics - 1)](Si * sin(angleExpected * i) + Ci * cos(angleExpected * i))
	// Then iterate until the correction converges or the maximum number of iterations is reached. In tests
	const size_t LUTLength = GetNumLUTEntries();
	constexpr unsigned int MaxIterations = 5;
	unsigned int actualMaxIterationNumber = 0;
	const float *harmonicData = mem.GetClosedLoopHarmonicValues();
	minLUTCorrection = std::numeric_limits<float>::infinity();
	maxLUTCorrection = -std::numeric_limits<float>::infinity();
	for (size_t index = 0; index < LUTLength; index++)
	{
		const float basicAngle = TwoPi * index / LUTLength;
		float lastCorrection = 0.0;
		for (unsigned int iterations = 0; iterations < MaxIterations; ++iterations)
		{
			if (iterations > actualMaxIterationNumber)
			{
				actualMaxIterationNumber = iterations;
			}
			const float correctionAngle = lastCorrection * TwoPi/GetMaxValue();
			float correction = 0.0;
			for (size_t harmonic = 0; harmonic<NumHarmonics; harmonic++)
			{
				const float sineCoefficient = harmonicData[2 * harmonic];
				const float cosineCoefficient = harmonicData[2 * harmonic + 1];
				const float angle = harmonic * basicAngle;
				correction -= sineCoefficient * sinf(angle + correctionAngle) + cosineCoefficient * cosf(angle + correctionAngle);
			}
			const float diff = fabsf(correction - lastCorrection);
			lastCorrection = correction;
			if (diff < 1.0)
			{
				break;
			}
		}
		const int32_t temp = lrintf(lastCorrection + (float)(index << resolutionToLutShiftFactor));
		correctionLUT[index] = (uint16_t)temp & ((1u << resolutionBits) - 1);
		if (index != 0)
		{
			if (lastCorrection < minLUTCorrection) { minLUTCorrection = lastCorrection; }
			if (lastCorrection > maxLUTCorrection) { maxLUTCorrection = lastCorrection; }
		}
	}

#ifdef DEBUG
	debugPrintf("Actual max iterations %u, minCorrection %.1f, maxCorrection %.1f\n", actualMaxIterationNumber + 1, (double)minLUTCorrection, (double)maxLUTCorrection);
#endif

	// Mark the LUT as loaded
	LUTLoaded = true;
}

void AbsoluteEncoder::StoreLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept
{
	// Update the sine coefficient for the zero'th harmonic (i.e. constant term) to account for where we started
	//TODO

#ifdef DEBUG
	debugPrintf("Min/max errors [%.1f %.1f]\nSin/cos coefficients:", (double)minCalibrationError, (double)maxCalibrationError);
#endif

	// Store the table of harmonics to nonvolatile memory
	NonVolatileMemory mem(NvmPage::closedLoop);
	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		const float sineCoefficient = sines[harmonic]/numReadingsTaken;
		const float cosineCoefficient = cosines[harmonic]/numReadingsTaken;
		mem.SetClosedLoopHarmonicValue(harmonic * 2, sineCoefficient);
		mem.SetClosedLoopHarmonicValue(harmonic * 2 + 1, cosineCoefficient);
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sineCoefficient, (double)cosineCoefficient);
#endif
	}
#ifdef DEBUG
	debugPrintf("\n");
#endif
	mem.EnsureWritten();

	// Populate the LUT from the coefficients
	PopulateLUT(mem);
}

// Clear the LUT. We may be about to calibrate the encoder, so clear the calibration values too.
void AbsoluteEncoder::ClearLUT() noexcept
{
	LUTLoaded = false;
	minCalibrationError = std::numeric_limits<float>::infinity();
	maxCalibrationError = -std::numeric_limits<float>::infinity();
	for (size_t i = 0; i < NumHarmonics; ++i)
	{
		sines[i] = cosines[i] = 0.0;
	}
}

void AbsoluteEncoder::ScrubLUT() noexcept
{
	ClearLUT();
	StoreLUT(0, 1);
}

void AbsoluteEncoder::RecordDataPoint(float angle, float error) noexcept
{
	if (error < minCalibrationError) { minCalibrationError = error; }
	if (error > maxCalibrationError) { maxCalibrationError = error; }

	// Update the harmonic series coefficients
	for (size_t i = 0; i < NumHarmonics; ++i)
	{
		sines[i] += error * sinf(angle * i);
		cosines[i] += error * cosf(angle * i);
	}
}

#endif

// End
