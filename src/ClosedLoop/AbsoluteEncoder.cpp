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

		if (IsBackwards())
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
	const uint32_t relativePhasePosition = ((uint32_t)count * GetPhasePositionsPerRev()) >> resolutionBits;
	zeroCountPhasePosition = (phase - relativePhasePosition) & 4095u;
}

// Encoder polarity. Changing this will change the encoder reading.
void AbsoluteEncoder::SetBackwards(bool backwards) noexcept
{
	if (isBackwards != backwards)
	{
		rawAngle = (GetMaxValue() - rawAngle) & (GetMaxValue() - 1);
		const uint32_t oldCurrentAngle = currentAngle;
		currentAngle = (GetMaxValue() - currentAngle) & (GetMaxValue() - 1);
		const int32_t angleChange = (int32_t)currentAngle - (int32_t)oldCurrentAngle;
		currentCount += angleChange;
		isBackwards = backwards;
	}
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
				const float sineCoefficient = (harmonic == 0) ? 0.0 : harmonicData[2 * harmonic].f;
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
		rmsCorrection += fsquare(lastCorrection + harmonicData[1].f);				// ignore the mean when computing the RMC
	}
	rmsCorrection = sqrtf(rmsCorrection/LUTLength);
	meanCorrection = -harmonicData[1].f;

#ifdef DEBUG
	debugPrintf("Actual max iterations %u, minCorrection %.1f, maxCorrection %.1f, RMS correction %.1f\n", actualMaxIterationNumber + 1, (double)minLUTCorrection, (double)maxLUTCorrection, (double)rmsCorrection);
#endif

	// Mark the LUT as loaded
	LUTLoaded = true;
}

void AbsoluteEncoder::StoreLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept
{
#if 0
#ifdef DEBUG
	debugPrintf("Calibration min/max errors [%.1f %.1f]\nSin/cos coefficients:", (double)minCalibrationError, (double)maxCalibrationError);
#endif

	// Store the table of harmonics to nonvolatile memory
	NonVolatileMemory mem(NvmPage::closedLoop);
	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		const float sineCoefficient = 2.0 * sines[harmonic]/numReadingsTaken;
		if (harmonic != 0)				// the zero-order sine coefficient is always zero, so we store the zero count phase there instead
		{
			mem.SetClosedLoopHarmonicValue(harmonic * 2, sineCoefficient);
		}
		const float cosineCoefficient = (harmonic == 0) ? cosines[harmonic]/numReadingsTaken : 2.0 * cosines[harmonic]/numReadingsTaken;
		mem.SetClosedLoopHarmonicValue(harmonic * 2 + 1, cosineCoefficient);
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sineCoefficient, (double)cosineCoefficient);
#endif
	}
	mem.SetClosedLoopZeroCountPhase(0xFFFFFFFF);
	mem.SetClosedLoopDataValid(true);
#ifdef DEBUG
	debugPrintf("\n");
#endif
	mem.EnsureWritten();

	// Populate the LUT from the coefficients
	PopulateLUT(mem);
#endif
}

void AbsoluteEncoder::CheckLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept
{
#if 0
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
		rmsError += fsquare(correction + cosines[0]);					// ignore the mean when computing the RMS error
	}
	rmsError = sqrtf(rmsError/LUTLength);
	meanError = -cosines[0];
#endif
}

// Clear the LUT. We may be about to calibrate the encoder, so clear the calibration values too.
void AbsoluteEncoder::ClearLUT() noexcept
{
	LUTLoaded = false;
}

void AbsoluteEncoder::ClearDataCollection(size_t p_numDataPoints) noexcept
{
	numDataPoints = p_numDataPoints;
	hysteresisSum = dataSum = dataBias = 0;
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

void AbsoluteEncoder::RecordDataPoint(size_t index, int16_t data, bool backwards) noexcept
{
	dataSum += data;
	if (backwards)
	{
		if (index + 1 == numDataPoints)
		{
			// This is the first data point collected during reverse motion.
			// It sometimes happens that the encoder appears to travel slightly more than a full rotation.
			// When using a 14-bit encoder and storing signed 16-bit values with two readings per point, this causes the data to overflow.
			// To avoid this, after collecting the first set of data points we bias the data by subtracting the average value.
			dataBias = dataSum/numDataPoints;
		}
		hysteresisSum += calibrationData[index] - data;
		calibrationData[index] = (int32_t)calibrationData[index] + (int32_t)data - dataBias;
	}
	else
	{
		calibrationData[index] = data;
	}
}

// Analyse the calibration data and optionally store it. We have the specified number of data points but we read each point twice, once while rotating forwards and once backwards.
void AbsoluteEncoder::Calibrate(int32_t initialCount, bool store) noexcept
{
	// Normalise initialCount to be within -GetMaxValue()..GetMaxValue()
	const int32_t normalisedInitialCount = initialCount % (int32_t)GetMaxValue();

	// dataSum is the sum of all the readings. If it is negative then the encoder is running backwards.
	const float twiceExpectedMidPointDifference = (float)dataSum/(float)numDataPoints;

	// expectedMidPointDifference should be close to half the encoder counts/rev
	const float ratio = twiceExpectedMidPointDifference/GetMaxValue();
	bool runningBackwards = false;
	if (ratio >= 0.95 && ratio <= 1.05)
	{
		// running forwards
		debugPrintf("forwards\n");
	}
	else if (ratio <= -0.95 && ratio >= -1.05)
	{
		debugPrintf("backwards\n");
		runningBackwards = true;
	}
	else
	{
		debugPrintf("bad encoder, ratio = %.2f\n", (double)ratio);
		//TODO report bad encoder and quit
	}

	const float expectedMidPointReading = twiceExpectedMidPointDifference/2.0 + (float)normalisedInitialCount;
	float phaseCorrection = (expectedMidPointReading * (float)GetPhasePositionsPerRev()/(float)GetMaxValue()) - (float)(GetPhasePositionsPerRev()/2);
	if (runningBackwards)
	{
		phaseCorrection = -phaseCorrection;
	}
	debugPrintf("exp mid pt rdg %.1f, init count %" PRIi32 ", norm init count %" PRIu32 ", phase corr %.1f\n", (double)expectedMidPointReading, initialCount, normalisedInitialCount, (double)phaseCorrection);

	int32_t expectedZeroReadingPhase = -(int32_t)phaseCorrection % 4096;
	if (expectedZeroReadingPhase < 0) { expectedZeroReadingPhase += 4096; }

	// Now Fourier analyse the data, using the expected zero reading phase to set the angle origin
	minCalibrationError = std::numeric_limits<float>::infinity();
	maxCalibrationError = -std::numeric_limits<float>::infinity();

	// Do a Fourier analysis of the data
	float sines[NumHarmonics], cosines[NumHarmonics];
	for (size_t i = 0; i < NumHarmonics; ++i)
	{
		sines[i] = cosines[i] = 0.0;
	}
	const float correctionRevFraction = phaseCorrection/GetPhasePositionsPerRev();
	debugPrintf("crf=%.3f hyst=%.2f\n", (double)correctionRevFraction, (double)((float)hysteresisSum/(float)numDataPoints));
	float rmsErrorAcc = 0.0;
	float sinSteps = 0.0;
	float cosSteps = 0.0;
	for (size_t i = 0; i < numDataPoints; ++i)
	{
		const float revFraction = (float)i/(float)numDataPoints + correctionRevFraction;
		const float angle = TwoPi * revFraction;
		float expectedValue = revFraction * GetMaxValue() * 2;		// *2 because we stored 2 values for each data point
		if (runningBackwards)
		{
			expectedValue = -expectedValue;
		}
		const int32_t actualValue = (int32_t)calibrationData[i] + dataBias + (2 * initialCount);
		float error = expectedValue - (float)actualValue;
		if (error >  (float)GetMaxValue()) { error -= (float)(2 * GetMaxValue()); }
		else if (error < -(float)GetMaxValue()) { error += (float)(2 * GetMaxValue()); }

		if ((i & 127) == 0 || fabsf(error) > (float)GetMaxValue())
		{
			debugPrintf("exp %.1f calib %" PRIi32 " err %.1f\n", (double)expectedValue, actualValue, (double)error);
		}
		if (error < minCalibrationError) { minCalibrationError = error; }
		if (error > maxCalibrationError) { maxCalibrationError = error; }
		rmsErrorAcc += fsquare(error);

		for (size_t j = 0; j < NumHarmonics; ++j)
		{
			sines[j] += error * sinf(angle * j);
			cosines[j] += error * cosf(angle * j);
		}

		sinSteps += error * sinf(TwoPi * (float)(i * stepsPerRev)/(float)numDataPoints);
		cosSteps += error * cosf(TwoPi * (float)(i * stepsPerRev)/(float)numDataPoints);
	}

	minCalibrationError *= 0.5;
	maxCalibrationError *= 0.5;
	rmsErrorAcc = 0.5 * sqrtf(rmsErrorAcc/numDataPoints);

#ifdef DEBUG
	debugPrintf("Calibration zrp %" PRIi32 " min/max/rms errors [%.1f %.1f %.1f] sin/cos steps [%.2f %.2f]\nSin/cos coefficients:",
				expectedZeroReadingPhase, (double)minCalibrationError, (double)maxCalibrationError, (double)rmsErrorAcc, (double)(sinSteps/numDataPoints), (double)(cosSteps/numDataPoints));
#endif
	for (size_t harmonic = 0; harmonic < NumHarmonics; harmonic++)
	{
		sines[harmonic] = sines[harmonic]/numDataPoints;
		cosines[harmonic] = (harmonic == 0) ? 0.5 * cosines[harmonic]/numDataPoints : cosines[harmonic]/numDataPoints;
#ifdef DEBUG
		debugPrintf(" [%.3f %.3f]", (double)sines[harmonic], (double)cosines[harmonic]);
#endif
	}
#ifdef DEBUG
	debugPrintf("\n");
#endif
}

void AbsoluteEncoder::ReportCalibrationResult(const StringRef& reply) const noexcept
{
	reply.lcatf("Calibration corrections: min %.1f, max %.1f, mean %.1f, deviation %.1f", (double)minLUTCorrection, (double)maxLUTCorrection, (double)meanCorrection, (double)rmsCorrection);
}

void AbsoluteEncoder::ReportCalibrationCheckResult(const StringRef& reply) const noexcept
{
	reply.lcatf("Calibration error: min %.1f, max %.1f, mean %.1f, deviation %.1f", (double)minLUTError, (double)maxLUTError, (double)meanError, (double)rmsError);
}

#endif

// End
