/*
 * AbsoluteEncoder.h
 *
 *  Created on: 6 Sep 2021
 *      Author: Louis
 */

/*
 * To use the AbsoluteEncoder class, define:
 *
 *  - MAX = The value given by the encoder when at it's maximum (e.g. when at 359.99 degrees)
 * 		    (When at minimum, the encoder must output 0)
 *
 *  - LUT_RESOLUTION = The resolution of the lookup table. Must be a power of 2 e.g. 16
 *
 *  - uint32_t GetAbsolutePosition() to return a value between 0 and MAX
 *
 *  - Plus all the virtual functions required by the Encoder class
 */

#ifndef SRC_CLOSEDLOOP_ABSOLUTEENCODER_H_
# define SRC_CLOSEDLOOP_ABSOLUTEENCODER_H_

# include "Encoder.h"

# if SUPPORT_CLOSED_LOOP

constexpr unsigned int NUM_HARMONICS = 17;	// Store harmonics 0-16

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
class AbsoluteEncoder : public Encoder
{
public:
	AbsoluteEncoder() noexcept : lastAngle(0), fullRotations(0), LUTLoaded(false) {};

	// Get the current reading
	int32_t GetReading() noexcept;

	// Lookup table (LUT) management
	bool LoadLUT() noexcept;
	void StoreLUT() noexcept;
	void ClearLUT() noexcept;
	void ScrubLUT() noexcept;
	void StoreLUTValueForPosition(int16_t encoder_reading, float real_world_position) noexcept;

	// Constants
	static constexpr unsigned int GetMaxValue() noexcept { return MAX; }
	static constexpr unsigned int GetLUTResolution() noexcept { return LUT_RESOLUTION; }
	EncoderPositioningType GetPositioningType() const noexcept override { return EncoderPositioningType::absolute; }

protected:
	// Must be defined to return a value between 0 and MAX
	virtual uint32_t GetAbsolutePosition(bool& error) noexcept = 0;

	// For calculating the relative position
	int32_t lastAngle;
	int32_t fullRotations;

	// LUT vars
	bool LUTLoaded;
	int zeroCrossingIndex;
	int zeroCrossingOffset;
	float minCorrection = 0.0, maxCorrection = 0.0;
	float correctionLUT[MAX / LUT_RESOLUTION];
};

# include <Hardware/NonVolatileMemory.h>

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
int32_t AbsoluteEncoder<MAX, LUT_RESOLUTION>::GetReading() noexcept {

	bool error;
	int32_t currentAngle = GetAbsolutePosition(error);

	if (error) {
		//TODO how to report an error?
		return fullRotations * MAX + lastAngle;
	}

	// Apply LUT correction (if the LUT is loaded)
	// (These divisions should be efficient because LUT_RESOLUTION is a power of 2)
	if (LUTLoaded) {
		int windowStartIndex = currentAngle / LUT_RESOLUTION;
		float windowStart = correctionLUT[windowStartIndex];
		int windowOffset = currentAngle % LUT_RESOLUTION;

		// Handle the zero-crossing
		if (windowStartIndex == zeroCrossingIndex && zeroCrossingOffset >= windowOffset) {
			windowStart = 0;
			windowOffset -= zeroCrossingOffset;
		}

		currentAngle = windowStart + windowOffset;
	}

	// Accumulate the full rotations if one has occurred
	int32_t difference = currentAngle - lastAngle;
	if (abs(difference) > (int32_t)MAX/2) {
		fullRotations += (difference < 0) - (difference > 0);	// Add -1 if diff > 0, +1 if diff < 0
	}
	lastAngle = currentAngle;

	// Return the position plus the accumulated rotations
	return fullRotations * MAX + lastAngle;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
bool AbsoluteEncoder<MAX, LUT_RESOLUTION>::LoadLUT() noexcept {

	NonVolatileMemory mem(NvmPage::closedLoop);
	if (!mem.GetClosedLoopDataWritten()) {return false;}
	const float* const fourierAngles = mem.GetClosedLoopLUTHarmonicAngles();
	const float* const fourierMagnitudes = mem.GetClosedLoopLUTHarmonicMagnitudes();

	// Read back LUT from NVRAM (Fourier transform -> array)
	minCorrection = maxCorrection = 0.0;
	const size_t LUTLength = MAX / LUT_RESOLUTION;
	for (size_t index = 0; index < LUTLength; index++) {
		float correction = 0.0;
		for (size_t harmonic=0; harmonic<NUM_HARMONICS; harmonic++) {
			correction += fourierMagnitudes[harmonic] * sinf(harmonic * TwoPi * index / LUTLength + fourierAngles[harmonic]);
		}
		correctionLUT[index] = (float)(index * LUT_RESOLUTION) - correction;
		if (correction < minCorrection) { minCorrection = correction; }
		else if (correction > maxCorrection) { maxCorrection = correction; }
	}

	// Find the zero-crossing index and offset
	float prevVal = correctionLUT[0];
	for (unsigned int i = 1; i<(MAX/LUT_RESOLUTION); i++) {
		float curVal = correctionLUT[i];
		if (abs(prevVal - curVal) > MAX/2) {
			zeroCrossingIndex = i-1;
			zeroCrossingOffset = round(MAX - prevVal);
			break;
		}
		prevVal = curVal;
	}

	// Mark the LUT as loaded (and return true)
	return LUTLoaded = true;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::StoreLUT() noexcept {
	// TODO: Verify all LUT values are present

	// Store LUT to NVRAM (as Fourier transform)
	NonVolatileMemory mem(NvmPage::closedLoop);
	for (size_t harmonic = 0; harmonic < NUM_HARMONICS; harmonic++) {
		float sum1 = 0.0, sum2 = 0.0;
		const size_t LUTLength = MAX / LUT_RESOLUTION;
		for (size_t i = 0; i < LUTLength; ++i)
		{
			const float offset = i * LUT_RESOLUTION - correctionLUT[i];
			sum1 += offset * sinf(harmonic * i * TwoPi/LUTLength);
			sum2 += offset * cosf(harmonic * i * TwoPi/LUTLength);
		}
		mem.SetClosedLoopLUTHarmonicAngle(harmonic, atan2f(sum2, sum1));
		mem.SetClosedLoopLUTHarmonicMagnitude(harmonic, sqrtf(fsquare(sum1) + fsquare(sum2)) / (harmonic == 0 ? LUTLength : (LUTLength/2)));
	}
	mem.EnsureWritten();

	// Read back the LUT (Ensures that the Fourier transformed version is used)
	LoadLUT();
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::ClearLUT() noexcept {
	LUTLoaded = false;
	minCorrection = maxCorrection = 0.0;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::ScrubLUT() noexcept {
	// TODO: Remove LUT from NVRAM
	ClearLUT();
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::StoreLUTValueForPosition(int16_t encoder_reading, float real_world_position) noexcept {
	correctionLUT[encoder_reading / LUT_RESOLUTION] = real_world_position;
}

# endif

#endif /* SRC_CLOSEDLOOP_ABSOLUTEENCODER_H_ */
