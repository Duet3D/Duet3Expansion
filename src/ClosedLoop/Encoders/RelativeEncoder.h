/*
 * RelativeEncoder.h
 *
 *  Created on: 6 Sep 2021
 *      Author: Louis
 */

/*
 * To use the RelativeEncoder class, define:
 *
 *  - int32_t GetRelativePosition() to return a value representative of the distance moved relative to the start position
 *
 *  - Plus all the virtual functions required by the Encoder class
 */

#ifndef SRC_CLOSEDLOOP_RELATIVEENCODER_H_
# define SRC_CLOSEDLOOP_RELATIVEENCODER_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP

class RelativeEncoder : public Encoder
{
public:
	// Constructors
	RelativeEncoder(float p_countsPerRev, uint32_t p_stepsPerRev) noexcept
		: Encoder(p_countsPerRev/(float)p_stepsPerRev, p_stepsPerRev), countsPerRev(lrintf(p_countsPerRev))
	{
		phasesPerCount = 1024.0 * stepsPerCount;
	}

	// Overridden virtual functions

	// Take a reading and store at least currentCount and currentPhasePosition. Return true if error, false if success.
	bool TakeReading() noexcept override;

	// Tell the encoder what the step phase is at the current count. Only applicable to relative encoders.
	void SetKnownPhaseAtCurrentCount(uint32_t phase) noexcept override;

	// Encoder polarity for basic tuning purposes. Changing this will change the encoder reading.
	void SetTuningBackwards(bool backwards) noexcept override;

	// Encoder polarity for calibration purposes. Changing this will change the encoder reading.
	void SetCalibrationBackwards(bool backwards) noexcept override { }

	// Return true if rotary absolute encoder calibration is applicable to this encoder
	bool UsesCalibration() const noexcept override { return false; }

	// Return true if basic tuning is applicable to this encoder
	bool UsesBasicTuning() const noexcept override { return true; }

	// Set the forward tuning results
	void SetForwardTuningResults(float slope, float xMean, float yMean) noexcept override { forwardSlope = slope; forwardXmean = xMean; forwardYmean = yMean; }

	// Set the reverse tuning results
	void SetReverseTuningResults(float slope, float xMean, float yMean) noexcept override { reverseSlope = slope; reverseXmean = xMean; reverseYmean = yMean; }

	// Process the tuning data
	TuningErrors ProcessTuningData() noexcept override;

	// Clear the encoder data collection. Only applicable if the encoder supports calibration.
	void ClearDataCollection(size_t p_numDataPoints) noexcept override { }

	// Record a calibration data point. Only applicable if the encoder supports calibration.
	void RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept override { }

	// Load the calibration lookup table and clear bits TuningError:NeedsBasicTuning and/or TuningError::NotCalibrated in tuningNeeded as appropriate.
	void LoadLUT(TuningErrors& tuningNeeded) noexcept override { }

	// Clear the calibration lookup table. Only applicable if the encoder supports calibration.
	void ClearLUT() noexcept override { }

	// Clear the calibration lookup table and delete it from NVRAM. Only applicable if the encoder supports calibration.
	void ScrubLUT() noexcept override { }

	// Calibrate the encoder using the recorded data points. Only applicable if the encoder supports calibration.
	TuningErrors Calibrate(bool store) noexcept override { return TuningError::SystemError; }

	// Append a summary of calibration lookup table corrections to a string. Only applicable if the encoder supports calibration.
	void AppendLUTCorrections(const StringRef& reply) const noexcept override { }

	// Append a summary of measured calibration errors to a string. Only applicable if the encoder supports calibration.
	void AppendCalibrationErrors(const StringRef& reply) const noexcept override { }

	// Report whether we reverse the encoder direction
	bool IsBackwards() const noexcept { return reversePolarityMultiplier < 0.0; }

protected:
	// Get the relative position since the start
	virtual int32_t GetRelativePosition(bool& error) noexcept = 0;

	uint32_t countsPerRev;
	int32_t reversePolarityMultiplier = 1;
	uint32_t zeroCountPhasePosition = 0;

private:
	float phasesPerCount;

	// Tuning data
	float forwardSlope;
	float reverseSlope;
	float forwardXmean;
	float reverseXmean;
	float forwardYmean;
	float reverseYmean;
};

#endif

#endif /* SRC_CLOSEDLOOP_RELATIVEENCODER_H_ */
