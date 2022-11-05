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

#ifndef SRC_CLOSEDLOOP_ABSOLUTEROTARYENCODER_H_
# define SRC_CLOSEDLOOP_ABSOLUTEROTARYENCODER_H_

# include "Encoder.h"

# if SUPPORT_CLOSED_LOOP

#include "TuningErrors.h"

class NonVolatileMemory;

// Base class for absolute encoders. The encoder resolution(counts/revolution) must be a power of two.
class AbsoluteRotaryEncoder : public Encoder
{
public:
	// Constructors
	AbsoluteRotaryEncoder(uint32_t p_stepsPerRev, unsigned int p_resolutionBits) noexcept;

	// Overridden virtual functions
	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return true; }

	// Get the current reading
	bool TakeReading() noexcept override;

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	void ClearFullRevs() noexcept override;

	// Encoder polarity. Changing this will change the encoder reading.
	void SetBackwards(bool backwards) noexcept override;

	// Return the encoder polarity
	bool IsBackwards() const noexcept override { return isBackwards; }

	// Return true if rotary absolute encoder calibration is applicable to this encoder
	bool UsesCalibration() const noexcept override { return true; }

	// Return true if basic tuning is applicable to this encoder
	bool UsesBasicTuning() const noexcept override { return false; }

	// Get the angle within a rotation from the most recent reading, corrected for direction only
	uint32_t GetRawAngle() const noexcept { return rawAngle; }

	// Get the corrected angle within a rotation from the most recent reading
	uint32_t GetCurrentAngle() const noexcept { return currentAngle; }

	// Get the counts per revolution
	uint32_t GetCountsPerRev() const noexcept { return 1u << resolutionBits; }

	// Lookup table (LUT) management
	void ClearDataCollection(size_t p_numDataPoints) noexcept;
	void RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept;
	TuningErrors Calibrate(bool store) noexcept;

	bool LoadLUT() noexcept;
	void ClearLUT() noexcept;
	void ScrubLUT() noexcept;

	unsigned int GetMaxValue() const noexcept { return 1ul << resolutionBits; }
	unsigned int GetNumLUTEntries() const noexcept { return 1u << (resolutionBits - resolutionToLutShiftFactor); }
	unsigned int GetResolutionBits() const noexcept { return resolutionBits; }
	unsigned int GetResolutionToLutShiftFactor() const noexcept { return resolutionToLutShiftFactor; }

	void AppendLUTCorrections(const StringRef& reply) const noexcept;
	void AppendCalibrationErrors(const StringRef& reply) const noexcept;

	static constexpr size_t MaxDataPoints = 200 * 64;				// support up to 64 data points per full step, uses about 25K RAM

protected:
	// This must be defined to set rawReading to a value between 0 and one below GetMaxValue()
	virtual bool GetRawReading() noexcept = 0;

	uint32_t rawReading = 0;				// the value read from the encoder
	uint32_t rawAngle = 0;					// the value after correcting for direction
	uint32_t currentAngle = 0;				// the value after correcting for eccentricity
	int32_t fullRotations = 0;				// the number of full rotations counted
	float stepAngle;

	// For diagnostics
	float minLUTCorrection = 0.0, maxLUTCorrection = 0.0;			// min and max corrections, for reporting in diagnostics
	float rmsCorrection = 0.0;

private:
	static constexpr unsigned int NumHarmonics = 10;				// store harmonics 0-9
	static constexpr unsigned int LutResolutionBits = 10;
	static constexpr size_t NumLutEntries = 1ul << LutResolutionBits;

	// Populate the LUT when we already have the nonvolatile data
	void PopulateLUT(NonVolatileMemory& mem) noexcept;

	// General variables
	const unsigned int resolutionBits;								// encoder has (2 ** resolutionShiftFactor) counts/revolution
	const unsigned int resolutionToLutShiftFactor;					// shift the resolution right by this number of bits to get the LUT resolution

	// LUT variables
	uint16_t correctionLUT[NumLutEntries + 1];						// mapping from raw encoder reading to corrected reading. The extra entry is a duplicate of the first entry.
	bool LUTLoaded = false;
	bool isBackwards = false;

	// Calibration variables
	float minCalibrationError = 0.0, maxCalibrationError = 0.0, rmsCalibrationError = 0.0;	// min, max and RMS corrections, for reporting in diagnostics
	int32_t dataSum;
	int32_t hysteresisSum;
	int32_t initialCount;
	int32_t dataBias;
	size_t numDataPoints;
	unsigned int calibrationPhase;
	int16_t calibrationData[MaxDataPoints];
};

# endif

#endif /* SRC_CLOSEDLOOP_ABSOLUTEROTARYENCODER_H_ */
