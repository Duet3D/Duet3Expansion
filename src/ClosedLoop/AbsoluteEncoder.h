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

class NonVolatileMemory;

// Base class for absolute encoders. The encoder resolution(counts/revolution) must be a power of two.
class AbsoluteEncoder : public Encoder
{
public:
	// Constructors
	AbsoluteEncoder(unsigned int p_resolutionBits) noexcept
		: resolutionBits(p_resolutionBits), resolutionToLutShiftFactor((p_resolutionBits < LutResolutionBits) ? 0 : p_resolutionBits - LutResolutionBits)
	{}

	// Overridden virtual functions
	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return true; }

	// Get the current reading
	int32_t GetReading() noexcept override;

	// This ust be defined to return a value between 0 and one below GetMaxValue()
	virtual uint32_t GetAbsolutePosition(bool& error) noexcept = 0;

	// Lookup table (LUT) management
	void RecordDataPoint(float angle, float error) noexcept;
	bool LoadLUT() noexcept;
	void StoreLUT(uint32_t virtualStartPosition, uint32_t numReadingsTaken) noexcept;
	void ClearLUT() noexcept;
	void ScrubLUT() noexcept;

	unsigned int GetMaxValue() const noexcept { return 1ul << resolutionBits; }
	unsigned int GetNumLUTEntries() const noexcept { return min<unsigned int>(NumLutEntries, GetMaxValue()); }
	unsigned int GetResolutionBits() const noexcept { return resolutionBits; }
	unsigned int GetResolutionToLutShiftFactor() const noexcept { return resolutionToLutShiftFactor; }

protected:
	static constexpr unsigned int NumHarmonics = 17;				// store harmonics 0-16
	static constexpr unsigned int LutResolutionBits = 10;
	static constexpr size_t NumLutEntries = 1ul << LutResolutionBits;

	// Populate the LUT when we already have the nonvolatile data
	void PopulateLUT(NonVolatileMemory& mem) noexcept;

	// For calculating the relative position
	int32_t lastAngle = 0;
	int32_t fullRotations = 0;

	// LUT vars
	bool LUTLoaded = false;
	uint32_t zeroCrossingIndex = 0;
	uint32_t zeroCrossingOffset = 0;
	float minCalibrationError = 0.0, maxCalibrationError = 0.0;		// min and max corrections, for reporting in diagnostics
	float minLUTCorrection = 0.0, maxLUTCorrection = 0.0;			// min and max corrections, for reporting in diagnostics
	float correctionLUT[NumLutEntries];								// table of corrections from raw encoder reading to corrected reading
	float sines[NumHarmonics], cosines[NumHarmonics];

private:
	const unsigned int resolutionBits;								// encoder has (2 ** resolutionShiftFactor) counts/revolution
	const unsigned int resolutionToLutShiftFactor;					// shift the resolution right by this number of bits to get the LUT resolution
};

# endif

#endif /* SRC_CLOSEDLOOP_ABSOLUTEENCODER_H_ */
