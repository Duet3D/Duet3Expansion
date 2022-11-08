/*
 * AbsoluteEncoder.cpp
 *
 *  Created on: 15 Oct 2022
 *      Author: David
 */

#include "AbsoluteRotaryEncoder.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/NonVolatileMemory.h>

AbsoluteRotaryEncoder::AbsoluteRotaryEncoder(uint32_t p_stepsPerRev, unsigned int p_resolutionBits) noexcept
	: Encoder((1u << p_resolutionBits) / (float)p_stepsPerRev, p_stepsPerRev),
	  resolutionBits(p_resolutionBits),
	  resolutionToLutShiftFactor((p_resolutionBits < LutResolutionBits) ? 0 : p_resolutionBits - LutResolutionBits)
{}

// Take a reading and store at least currentCount and currentPhasePosition. Return true if error, false if success.
bool AbsoluteRotaryEncoder::TakeReading() noexcept
{
	bool err = GetRawReading();
	if (!err)
	{
		uint32_t newAngle = rawReading;

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
				const uint32_t windowOffset = newAngle & ((1u << resolutionToLutShiftFactor) - 1);
				if (windowOffset <= (1u << (resolutionToLutShiftFactor - 1)))
				{
					newAngle = correctionLUT[windowStartIndex] + windowOffset;
				}
				else
				{
					// Note that we store an duplicate of correctionLUT[0] at the end to avoid having to wrap when we add 1 to windowStartIndex
					newAngle = correctionLUT[windowStartIndex + 1] - (1u << resolutionToLutShiftFactor) + windowOffset;
				}
				newAngle &= (GetMaxValue() - 1);
			}
		}

		if (isBackwards)
		{
			newAngle = (GetMaxValue() - newAngle) & (GetMaxValue() - 1);
			rawAngle = (GetMaxValue() - rawReading) & (GetMaxValue() - 1);
		}
		else
		{
			rawAngle = rawReading;
		}
		currentPhasePosition = (((newAngle * GetPhasePositionsPerRev()) >> resolutionBits) + zeroCountPhasePosition) & 4095u;

		// Accumulate the full rotations if one has occurred
		const int32_t difference = (int32_t)newAngle - (int32_t)currentAngle;
		if (difference > (int32_t)(GetMaxValue()/2))
		{
			// Gone from a low value to a high value, so going down and wrapped round
			--fullRotations;
		}
		else if (difference < -(int32_t)(GetMaxValue()/2))
		{
			// Gone from a high value to a low value, so going up and wrapped round
			++fullRotations;
		}

		currentAngle = newAngle;
		currentCount = (fullRotations * (int32_t)GetMaxValue()) + (int32_t)currentAngle;
	}

	return err;
}

// Clear the accumulated full rotations so as to get the count back to a smaller number
void AbsoluteRotaryEncoder::ClearFullRevs() noexcept
{
	fullRotations = 0;
	currentCount = (int32_t)currentAngle;
}

// Encoder polarity. Changing this will change the encoder reading.
void AbsoluteRotaryEncoder::SetCalibrationBackwards(bool backwards) noexcept
{
	if (isBackwards != backwards)
	{
		ClearFullRevs();
		rawAngle = (GetMaxValue() - rawAngle) & (GetMaxValue() - 1);
		currentAngle = (GetMaxValue() - currentAngle) & (GetMaxValue() - 1);
		ClearFullRevs();
		isBackwards = backwards;
	}
}

// Load the calibration lookup table and clear bits TuningError:NeedsBasicTuning and/or TuningError::NotCalibrated in tuningNeeded as appropriate.
void AbsoluteRotaryEncoder::LoadLUT(TuningErrors& tuningNeeded) noexcept
{
	NonVolatileMemory mem(NvmPage::closedLoop);
	if (mem.GetClosedLoopCalibrationDataValid())
	{
		PopulateLUT(mem);
		tuningNeeded &= ~TuningError::NotCalibrated;
	}
}

// Populate the LUT when we already have the nonvolatile data and we have checked that it is valid
void AbsoluteRotaryEncoder::PopulateLUT(NonVolatileMemory& mem) noexcept
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
	rmsCorrection = 0.0;
#ifdef DEBUG
	int32_t totalCorrection = 0;
#endif
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
			for (size_t harmonic = 1; harmonic < NumHarmonics; harmonic++)
			{
				const float sineCoefficient = harmonicData[2 * harmonic - 2].f;
				const float cosineCoefficient = harmonicData[2 * harmonic - 1].f;
				const float angle = harmonic * basicAngle;
				correction += sineCoefficient * sinf(angle + correctionAngle) + cosineCoefficient * cosf(angle + correctionAngle);
			}
			const float diff = fabsf(correction - lastCorrection);
			lastCorrection = correction;
			if (diff < 1.0)
			{
				break;
			}
		}
		const int32_t temp = lrintf(lastCorrection) + (int32_t)(index << resolutionToLutShiftFactor);
		correctionLUT[index] = (uint16_t)temp & ((1u << resolutionBits) - 1);
		if (lastCorrection < minLUTCorrection) { minLUTCorrection = lastCorrection; }
		if (lastCorrection > maxLUTCorrection) { maxLUTCorrection = lastCorrection; }
		rmsCorrection += fsquare(lastCorrection);
#ifdef DEBUG
		totalCorrection += lrintf(lastCorrection);
#endif
	}
	correctionLUT[LUTLength] = correctionLUT[0];			// extra duplicate entry at end
	rmsCorrection = sqrtf(rmsCorrection/LUTLength);

	bool backwards;
	mem.GetClosedLoopZeroCountPhaseAndDirection(zeroCountPhasePosition, backwards);
	SetCalibrationBackwards(backwards);

#ifdef DEBUG
	debugPrintf("Actual max iterations %u, minCorrection %.1f, maxCorrection %.1f, RMS correction %.1f, zrp %" PRIu32 " totalCorr %" PRIi32 "\n",
					actualMaxIterationNumber + 1, (double)minLUTCorrection, (double)maxLUTCorrection, (double)rmsCorrection, zeroCountPhasePosition, totalCorrection);
#endif

	// Mark the LUT as loaded
	LUTLoaded = true;
}

// Clear the LUT. We may be about to calibrate the encoder, so clear the calibration values too.
void AbsoluteRotaryEncoder::ClearLUT() noexcept
{
	LUTLoaded = false;
}

void AbsoluteRotaryEncoder::ClearDataCollection(size_t p_numDataPoints) noexcept
{
	numDataPoints = p_numDataPoints;
	hysteresisSum = dataSum = dataBias = 0;
	minCalibrationError = std::numeric_limits<float>::infinity();
	maxCalibrationError = -std::numeric_limits<float>::infinity();
	calibrationPhase = 0;
}

void AbsoluteRotaryEncoder::ScrubLUT() noexcept
{
	ClearLUT();
	NonVolatileMemory mem(NvmPage::closedLoop);
	mem.SetClosedLoopCalibrationDataNotValid();
	mem.EnsureWritten();
}

// Record a data point. The first forwards data point must be index zero, and the first backwards data point must be index (numDataPoints - 1).
void AbsoluteRotaryEncoder::RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept
{
	if (backwards)
	{
		if (index + 1 == numDataPoints)
		{
			// This is the first data point collected during reverse motion.
			// It sometimes happens that the encoder appears to travel slightly more than a full rotation.
			// When using a 14-bit encoder and storing signed 16-bit values with two readings per point, this causes the data to overflow.
			// To avoid this, after collecting the first set of data points we bias the data by subtracting the average value.
			dataBias = dataSum/(int32_t)numDataPoints;
		}
		data -= initialCount;
		hysteresisSum += data - calibrationData[index];
		calibrationData[index] = (int16_t)((int32_t)calibrationData[index] + data - dataBias);
	}
	else
	{
		if (index == 0)
		{
			initialCount = data;
		}
		data -= initialCount;
		calibrationData[index] = (int16_t)data;
	}
	dataSum += data;
}

// Analyse the calibration data and optionally store it. We have the specified number of data points but we read each point twice, once while rotating forwards and once backwards.
// This takes a long time so it must be called by a low priority task
TuningErrors AbsoluteRotaryEncoder::Calibrate(bool store) noexcept
{
	// dataSum is the sum of all the readings. If it is negative then the encoder is running backwards.
	const float twiceExpectedMidPointDifference = (float)dataSum/(float)numDataPoints;

	// expectedMidPointDifference should be close to half the encoder counts/rev
	const float ratio = twiceExpectedMidPointDifference/GetMaxValue();
	const float rotationDirection = (ratio < 0.0) ? -1.0 : 1.0;
	measuredCountsPerStep = (twiceExpectedMidPointDifference * rotationDirection)/stepsPerRev;
	measuredHysteresis = ((float)hysteresisSum * rotationDirection)/((float)numDataPoints * countsPerStep);

	if (fabsf(ratio) > 1.05)
	{
		return TuningError::TooMuchMotion;
	}
	if (fabsf(ratio) < 0.95)
	{
		return TuningError::TooLittleMotion;
	}

	// Normalise initialCount to be within -GetMaxValue()..GetMaxValue()
	initialCount %= (int32_t)GetMaxValue();

	// Further normalise initialCount to keep the angles small, preferably we want the count to cross zero and back.
	if (dataSum > 0 && initialCount > 0) { initialCount -= (int32_t)GetMaxValue(); }
	else if (dataSum < 0 && initialCount < 0) { initialCount += (int32_t)GetMaxValue(); }

	const float expectedMidPointReading = twiceExpectedMidPointDifference * 0.5 + (float)initialCount;
	const float revFractionAtMidPoint = (float)(numDataPoints - 1)/(float)(2 * numDataPoints);
	const float correctionRevFraction = (expectedMidPointReading * rotationDirection)/(float)GetMaxValue() - revFractionAtMidPoint;
	const float phaseCorrection = correctionRevFraction * (float)GetPhasePositionsPerRev();
	int32_t expectedZeroReadingPhase = -(lrintf(phaseCorrection)) % 4096;
	if (expectedZeroReadingPhase < 0) { expectedZeroReadingPhase += 4096; }

#ifdef DEBUG
	debugPrintf("dataSum %" PRIi32 ", empr %.5f, init count %" PRIi32 ", crf %.5f, phase corr %.1f, bias %" PRIi32 ", zrp %" PRIu32 "\n",
					dataSum, (double)expectedMidPointReading, initialCount, (double)correctionRevFraction, (double)phaseCorrection, dataBias, expectedZeroReadingPhase);
#endif

	// Now Fourier analyse the data, using the expected zero reading phase to set the angle origin
	float minError = std::numeric_limits<float>::infinity();
	float maxError = -std::numeric_limits<float>::infinity();

	// Do a Fourier analysis of the data
	float sines[NumHarmonics], cosines[NumHarmonics];
	for (size_t i = 0; i < NumHarmonics; ++i)
	{
		sines[i] = cosines[i] = 0.0;
	}
	float rmsErrorAcc = 0.0;
	float sinSteps = 0.0;
	float cosSteps = 0.0;
	for (size_t i = 0; i < numDataPoints; ++i)
	{
		const float revFraction = ((float)i/(float)numDataPoints + correctionRevFraction) * rotationDirection;
		const float angle = TwoPi * revFraction;
		const float expectedValue = revFraction * (float)(GetMaxValue() * 2);		// *2 because we stored 2 values for each data point
		const int32_t actualValue = (int32_t)calibrationData[i] + dataBias + (2 * initialCount);
		const float error = expectedValue - (float)actualValue;

#if 0
		if ((i & 127) == 0 /*|| fabsf(error) > (float)GetMaxValue()*/)
		{
			debugPrintf("exp %.1f calib %" PRIi32 " err %.1f\n", (double)expectedValue, actualValue, (double)error);
		}
#endif
		if (error < minError) { minError = error; }
		if (error > maxError) { maxError = error; }
		rmsErrorAcc += fsquare(error);

		for (size_t j = 0; j < NumHarmonics; ++j)
		{
			sines[j] += error * sinf(angle * j);
			cosines[j] += error * cosf(angle * j);
		}

		sinSteps += error * sinf(TwoPi * (float)(i * stepsPerRev)/(float)numDataPoints);
		cosSteps += error * cosf(TwoPi * (float)(i * stepsPerRev)/(float)numDataPoints);
	}

	// We took 2 samples per point, so halve the errors
	minCalibrationError = 0.5 * minError;
	maxCalibrationError = 0.5 * maxError;

	// Calculate the RMS error
	rmsCalibrationError = 0.5 * sqrtf(rmsErrorAcc/numDataPoints);

#ifdef DEBUG
	debugPrintf("min/max/rms errors [%.1f %.1f %.1f] sin/cos steps [%.2f %.2f]\nSin/cos coefficients:",
				(double)minCalibrationError, (double)maxCalibrationError, (double)rmsCalibrationError, (double)(sinSteps/numDataPoints), (double)(cosSteps/numDataPoints));
#endif

	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		sines[harmonic] /= numDataPoints;
		if (harmonic == 0)
		{
			cosines[harmonic] = 0.5 * cosines[harmonic]/numDataPoints;
		}
		else
		{
			cosines[harmonic] /= numDataPoints;
		}
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sines[harmonic], (double)cosines[harmonic]);
#endif
	}
#ifdef DEBUG
	debugPrintf("\n");
#endif

	if (store)
	{
		SetCalibrationBackwards(rotationDirection < 0.0);

		// Store the table of harmonics to nonvolatile memory
		NonVolatileMemory mem(NvmPage::closedLoop);
		for (size_t harmonic = 1; harmonic < NumHarmonics; harmonic++)
		{
			mem.SetClosedLoopHarmonicValue(harmonic * 2 - 2, sines[harmonic]);
			mem.SetClosedLoopHarmonicValue(harmonic * 2 - 1, cosines[harmonic]);
		}
		mem.SetClosedLoopZeroCountPhaseAndDirection((uint32_t)expectedZeroReadingPhase, (rotationDirection < 0.0));		// this also flags the NVM data as valid
		mem.EnsureWritten();

		// Populate the LUT from the coefficients
		PopulateLUT(mem);
	}
	return 0;
}

void AbsoluteRotaryEncoder::AppendLUTCorrections(const StringRef& reply) const noexcept
{
	reply.catf("min %.1f, max %.1f, rms %.1f", (double)minLUTCorrection, (double)maxLUTCorrection, (double)rmsCorrection);
}

void AbsoluteRotaryEncoder::AppendCalibrationErrors(const StringRef& reply) const noexcept
{
	reply.catf("min %.1f, max %.1f, rms %.1f", (double)minCalibrationError, (double)maxCalibrationError, (double)rmsCalibrationError);
}

#endif

// End
