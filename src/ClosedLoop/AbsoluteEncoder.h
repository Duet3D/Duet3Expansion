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
	AbsoluteEncoder() noexcept : lastAngle(0), fullRotations(0), LUTLoaded(false) {}

	// Get the current reading
	int32_t GetReading() noexcept;

	// Lookup table (LUT) management
	void LoadLUT() noexcept;
	void StoreLUT() noexcept;
	void ClearLUT() noexcept;
	void ScrubLUT() noexcept;
	void StoreLUTValueForPosition(int16_t encoder_reading, float real_world_position) noexcept;

	// Constants
	unsigned int GetMaxValue() const noexcept { return MAX; }
	unsigned int GetLUTResolution() const noexcept { return LUT_RESOLUTION; }
	EncoderPositioningType GetPositioningType() const noexcept override { return EncoderPositioningType::absolute; }

protected:
	// Must be defined to return a value between 0 and MAX
	virtual uint32_t GetAbsolutePosition(bool& error) noexcept;

	// For calculating the relative position
	int32_t lastAngle;
	int32_t fullRotations;

	// LUT vars
	bool LUTLoaded;
	int zeroCrossingIndex;
	int zeroCrossingOffset;
	float correctionLUT[MAX / LUT_RESOLUTION];
	float fourierMagnitudes[NUM_HARMONICS];
	float fourierAngles[NUM_HARMONICS];

};

# include <ClosedLoop/AbsoluteEncoder.cpp>		// Required since template definitions/implementations cannot be separated

# endif

#endif /* SRC_CLOSEDLOOP_ABSOLUTEENCODER_H_ */
