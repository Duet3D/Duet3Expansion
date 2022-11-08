/*
 * Encoder.h
 *
 *  Created on: 10 Aug 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_ENCODER_H_
#define SRC_CLOSEDLOOP_ENCODER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include <GCodeResult.h>
#include "../TuningErrors.h"

class Encoder
{
public:
	Encoder(float p_countsPerStep, uint32_t p_stepsPerRev) noexcept;

	virtual ~Encoder() { }

	// Get the type of this encoder
	virtual EncoderType GetType() const noexcept = 0;

	// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
	virtual GCodeResult Init(const StringRef& reply) noexcept = 0;

	// Enable the encoder
	virtual void Enable() noexcept = 0;

	// Disable the encoder
	virtual void Disable() noexcept = 0;

	// Take a reading and store at least currentCount and currentPhasePosition. Return true if error, false if success.
	virtual bool TakeReading() noexcept = 0;

	// Tell the encoder what the step phase is at the current count. Only applicable to relative encoders.
	virtual void SetKnownPhaseAtCurrentCount(uint32_t phase) noexcept = 0;

	// Get diagnostic information and append it to a string
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	// Append brief encoder status as a string
	virtual void AppendStatus(const StringRef& reply) noexcept = 0;

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	virtual void ClearFullRevs() noexcept = 0;

	// Encoder polarity for basic tuning purposes. Changing this will change the encoder reading.
	virtual void SetTuningBackwards(bool backwards) noexcept = 0;

	// Encoder polarity for calibration purposes. Changing this will change the encoder reading.
	virtual void SetCalibrationBackwards(bool backwards) noexcept = 0;

	// Return true if rotary absolute encoder calibration is applicable to this encoder
	virtual bool UsesCalibration() const noexcept = 0;

	// Return true if basic tuning is applicable to this encoder
	virtual bool UsesBasicTuning() const noexcept = 0;

	// Set the forward tuning results. Only applicable if the encoder supports basic tuning.
	virtual void SetForwardTuningResults(float slope, float xMean, float yMean) noexcept = 0;

	// Set the reverse tuning results. Only applicable if the encoder supports basic tuning.
	virtual void SetReverseTuningResults(float slope, float xMean, float yMean) noexcept = 0;

	// Process the tuning data. Only applicable if the encoder supports basic tuning.
	virtual TuningErrors ProcessTuningData() noexcept = 0;

	// Clear the encoder data collection. Only applicable if the encoder supports calibration.
	virtual void ClearDataCollection(size_t p_numDataPoints) noexcept = 0;

	// Record a calibration data point. Only applicable if the encoder supports calibration.
	virtual void RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept = 0;

	// Load the calibration lookup table and clear bits TuningError:NeedsBasicTuning and/or TuningError::NotCalibrated in tuningNeeded as appropriate.
	virtual void LoadLUT(TuningErrors& tuningNeeded) noexcept = 0;

	// Clear the calibration lookup table. Only applicable if the encoder supports calibration.
	virtual void ClearLUT() noexcept = 0;

	// Clear the calibration lookup table and delete it from NVRAM. Only applicable if the encoder supports calibration.
	virtual void ScrubLUT() noexcept = 0;

	// Calibrate the encoder using the recorded data points. Only applicable if the encoder supports calibration.
	virtual TuningErrors Calibrate(bool store) noexcept = 0;

	// Append a summary of calibration lookup table corrections to a string. Only applicable if the encoder supports calibration.
	virtual void AppendLUTCorrections(const StringRef& reply) const noexcept = 0;

	// Append a summary of measured calibration errors to a string. Only applicable if the encoder supports calibration.
	virtual void AppendCalibrationErrors(const StringRef& reply) const noexcept = 0;

	// Get the count if the shaft encoder. Only used by calibration. Normally the same as GetCurrentCount except for combination encoders.
	virtual int32_t GetCurrentShaftCount() const noexcept { return currentCount; }

	// Get the initial set of tuning errors for this encoder
	TuningErrors MinimalTuningNeeded() const noexcept;

	// Get the accumulated count
	int32_t GetCurrentCount() const noexcept { return currentCount; }

	// Return the number of encoder counts per step
	float GetCountsPerStep() const noexcept { return countsPerStep; }

	// Return the number of encoder steps per count
	float GetStepsPerCount() const noexcept { return stepsPerCount; }

	// Return the number of steps per revolution
	uint32_t GetStepsPerRev() const noexcept { return stepsPerRev; }

	// Return the number of phase positions per revolution
	uint32_t GetPhasePositionsPerRev() const noexcept { return stepsPerRev * 1024u; }

	// Get the current phase position from the last reading - this may be more accurate than the fractional part of GetCurrentMotorSteps()
	uint32_t GetCurrentPhasePosition() const noexcept { return currentPhasePosition; }

	// Return the measured counts per step after tuning or calibration
	float GetMeasuredCountsPerStep() const noexcept { return measuredCountsPerStep; }

	// Return the measured hysteresis after tuning or calibration
	float GetMeasuredHysteresis() const noexcept { return measuredHysteresis; }

	// Define the maximum number of calibration data points we can store. Currently this is the same for all encoders that support calibration.
	static constexpr size_t MaxCalibrationDataPoints = 200 * 64;				// support up to 64 data points per full step, uses about 25K RAM

protected:
	uint32_t stepsPerRev;
	uint32_t currentPhasePosition = 0;
	int32_t currentCount = 0;
	float countsPerStep;
	float stepsPerCount;
	float measuredCountsPerStep;
	float measuredHysteresis;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
