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

class Encoder
{
public:
	Encoder(uint32_t p_stepsPerRev, float p_countsPerStep) noexcept : stepsPerRev(p_stepsPerRev), countsPerStep(p_countsPerStep), stepsPerCount(1.0/p_countsPerStep) { }

	virtual ~Encoder() { }

	// Get the type of this encoder
	virtual EncoderType GetType() const noexcept = 0;

	// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
	virtual GCodeResult Init(const StringRef& reply) noexcept = 0;

	// Enable the encoder
	virtual void Enable() noexcept = 0;

	// Disable the encoder
	virtual void Disable() noexcept = 0;

	// Get the current reading after correcting it and adding the offset
	virtual bool TakeReading() noexcept = 0;

	// Get diagnostic information and append it to a string
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	// Append brief encoder status as a string
	virtual void AppendStatus(const StringRef& reply) noexcept = 0;

	// Return true if this is an absolute encoder, false if it is relative
	virtual bool IsAbsolute() const noexcept = 0;

	// Tell the encoder what the step phase is at a particular count
	virtual void SetKnownPhaseAtCount(uint32_t phase, int32_t count) noexcept = 0;

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	virtual void ClearFullRevs() noexcept = 0;

	// Get the accumulated count
	int32_t GetCurrentCount() const noexcept { return currentCount; }

	// Encoder polarity. Changing this will change the encoder reading.
	virtual void SetBackwards(bool backwards) noexcept = 0;

	// Return the encoder polarity
	virtual bool IsBackwards() const noexcept = 0;

	// Return the number of encoder counts per step
	float GetCountsPerStep() const noexcept { return countsPerStep; }

	// Return the number of encoder steps per count
	float GetStepsPerCount() const noexcept { return stepsPerCount; }

	// Return the number of steps per revolution
	uint32_t GetStepsPerRev() const noexcept { return stepsPerRev; }

	// Get the current phase position from the last reading - this may be more accurate that the fractional part of GetCurrentMotorSteps()
	uint32_t GetCurrentPhasePosition() const noexcept { return currentPhasePosition; }

protected:
	uint32_t stepsPerRev;
	uint32_t currentPhasePosition = 0;
	int32_t currentCount = 0;
	uint32_t zeroCountPhasePosition = 0;
	float countsPerStep;
	float stepsPerCount;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
