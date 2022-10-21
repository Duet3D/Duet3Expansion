/*
 * AbsoluteEncoder.cpp
 *
 *  Created on: 15 Oct 2022
 *      Author: David
 */

#include "AbsoluteEncoder.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/NonVolatileMemory.h>

AbsoluteEncoder::AbsoluteEncoder(uint32_t p_stepsPerRev, unsigned int p_resolutionBits) noexcept
	: Encoder(p_stepsPerRev, (1u << p_resolutionBits) / (float)p_stepsPerRev),
	  resolutionBits(p_resolutionBits),
	  resolutionToLutShiftFactor((p_resolutionBits < LutResolutionBits) ? 0 : p_resolutionBits - LutResolutionBits)
{}

// Take a reading and store currentCount, currentPhasePosition, rawAngle and currentAngle. Return true if success.
bool AbsoluteEncoder::TakeReading() noexcept
{
	bool err = GetRawReading();
	if (!err)
	{
		rawAngle = (IsBackwards()) ? GetMaxValue() - 1 - rawReading : rawReading;
		uint32_t newAngle = rawAngle;

		// Apply LUT correction (if the LUT is loaded)
		if (LUTLoaded)
		{
			const size_t windowStartIndex = newAngle >> resolutionToLutShiftFactor;
			if (resolutionToLutShiftFactor == 0)
			{
				newAngle = correctionLUT[windowStartIndex];
			}
			else
			{
				const uint32_t windowOffset = newAngle & (1u << (resolutionToLutShiftFactor - 1));
				if (windowOffset <= (1u << (resolutionToLutShiftFactor - 1)))
				{
					newAngle = correctionLUT[windowStartIndex] + windowOffset;
				}
				else
				{
					newAngle = correctionLUT[windowStartIndex + 1] - (1u << resolutionToLutShiftFactor) + windowOffset;
				}
				newAngle &= ((1u << resolutionBits) - 1);
			}
		}

		currentPhasePosition = (((newAngle * stepsPerRev * 1024u) >> resolutionBits) + zeroCountPhasePosition) & 4095u;

		// Accumulate the full rotations if one has occurred
		const int32_t difference = (int32_t)newAngle - (int32_t)currentAngle;
		if (difference > (int32_t)(GetMaxValue()/2))
		{
			// Gone from a low value to a high value, so going down and wrapped round
			--fullRotations;
		}
		else if (difference < -(int32_t)(GetMaxValue()/2))
		{
			// Gone from a high value to a low value, to going up and wrapped round
			++fullRotations;
		}

		currentAngle = newAngle;
		currentCount = (fullRotations * (int32_t)GetMaxValue()) + (int32_t)currentAngle;
	}

	return err;
}

// Clear the accumulated full rotations so as to get the count back to a smaller number
void AbsoluteEncoder::ClearFullRevs() noexcept
{
	fullRotations = 0;
	currentCount = (int32_t)currentAngle;
}

// Tell the encoder what the step phase is at a particular count
void AbsoluteEncoder::SetKnownPhaseAtCount(uint32_t phase, int32_t count) noexcept
{
	count %= (int32_t)GetCountsPerRev();
	if (count < 0) { count += (int32_t)GetCountsPerRev(); }
	const uint32_t relativePhasePosition = ((uint32_t)count * stepsPerRev * 1024u) >> resolutionBits;
	zeroCountPhasePosition = (phase - relativePhasePosition) & 4095u;
}

bool AbsoluteEncoder::LoadLUT() noexcept
{
	NonVolatileMemory mem(NvmPage::closedLoop);
	if (!mem.GetClosedLoopDataValid()) { return false; }

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
	const NonVolatileMemory::HarmonicDataElement *harmonicData = mem.GetClosedLoopHarmonicValues();
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
			for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
			{
				const float sineCoefficient = harmonicData[2 * harmonic].f;
				const float cosineCoefficient = harmonicData[2 * harmonic + 1].f;
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
		if (lastCorrection < minLUTCorrection) { minLUTCorrection = lastCorrection; }
		if (lastCorrection > maxLUTCorrection) { maxLUTCorrection = lastCorrection; }
		rmsCorrection += fsquare(lastCorrection);
	}
	rmsCorrection = sqrtf(rmsCorrection/LUTLength);

#ifdef DEBUG
	debugPrintf("Actual max iterations %u, minCorrection %.1f, maxCorrection %.1f, RMS correction %.1f\n", actualMaxIterationNumber + 1, (double)minLUTCorrection, (double)maxLUTCorrection, (double)rmsCorrection);
#endif

	// Mark the LUT as loaded
	LUTLoaded = true;
}

void AbsoluteEncoder::StoreLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept
{
#ifdef DEBUG
	debugPrintf("Calibration min/max errors [%.1f %.1f]\nSin/cos coefficients:", (double)minCalibrationError, (double)maxCalibrationError);
#endif

	// Store the table of harmonics to nonvolatile memory
	NonVolatileMemory mem(NvmPage::closedLoop);
	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		const float sineCoefficient = 2.0 * sines[harmonic]/numReadingsTaken;
		const float cosineCoefficient = (harmonic == 0) ? cosines[harmonic]/numReadingsTaken : 2.0 * cosines[harmonic]/numReadingsTaken;
		mem.SetClosedLoopHarmonicValue(harmonic * 2, sineCoefficient);
		mem.SetClosedLoopHarmonicValue(harmonic * 2 + 1, cosineCoefficient);
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sineCoefficient, (double)cosineCoefficient);
#endif
	}
	mem.SetClosedLoopDataValid(true);
#ifdef DEBUG
	debugPrintf("\n");
#endif
	mem.EnsureWritten();

	// Populate the LUT from the coefficients
	PopulateLUT(mem);
}

void AbsoluteEncoder::CheckLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept
{
#ifdef DEBUG
	debugPrintf("Calibration check min/max errors [%.1f %.1f]\nSin/cos coefficients:", (double)minCalibrationError, (double)maxCalibrationError);
#endif
	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		sines[harmonic] = 2.0 * sines[harmonic]/numReadingsTaken;
		cosines[harmonic] = (harmonic == 0) ? cosines[harmonic]/numReadingsTaken : 2.0 * cosines[harmonic]/numReadingsTaken;
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sines[harmonic], (double)cosines[harmonic]);
#endif
	}
#ifdef DEBUG
	debugPrintf("\n");
#endif

	const size_t LUTLength = GetNumLUTEntries();
	minLUTError = std::numeric_limits<float>::infinity();
	maxLUTError = -std::numeric_limits<float>::infinity();
	rmsError = 0.0;
	for (size_t index = 0; index < LUTLength; index++)
	{
		const float basicAngle = TwoPi * index / LUTLength;
		float correction = 0.0;
		for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
		{
			const float sineCoefficient = sines[harmonic];
			const float cosineCoefficient = cosines[harmonic];
			const float angle = harmonic * basicAngle;
			correction -= sineCoefficient * sinf(angle) + cosineCoefficient * cosf(angle);
		}
		if (correction < minLUTError) { minLUTError = correction; }
		if (correction > maxLUTError) { maxLUTError = correction; }
		rmsError += fsquare(correction);
	}
	rmsError = sqrtf(rmsError/LUTLength);
}

// Clear the LUT. We may be about to calibrate the encoder, so clear the calibration values too.
void AbsoluteEncoder::ClearLUT() noexcept
{
	LUTLoaded = false;
}

void AbsoluteEncoder::ClearHarmonics() noexcept
{
	for (size_t i = 0; i < NumHarmonics; ++i)
	{
		sines[i] = cosines[i] = 0.0;
	}
	minCalibrationError = std::numeric_limits<float>::infinity();
	maxCalibrationError = -std::numeric_limits<float>::infinity();
}

void AbsoluteEncoder::ScrubLUT() noexcept
{
	ClearLUT();
	NonVolatileMemory mem(NvmPage::closedLoop);
	mem.SetClosedLoopDataValid(false);
	mem.EnsureWritten();
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

void AbsoluteEncoder::ReportCalibrationCheckResult(const StringRef& reply) const noexcept
{
	reply.lcatf("Calibration error: min %.1f, max %.1f, rms %.1f\n", (double)minLUTError, (double)maxLUTError, (double)rmsError);
}

#endif

// End
