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
	Encoder(float p_countsPerStep) noexcept : countsPerStep(p_countsPerStep), recipCountsPerStep(1.0/p_countsPerStep) { }
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
	virtual int32_t GetReading(bool& err) noexcept = 0;

	// Get diagnostic information and append it to a string
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	// Append brief encoder status as a string
	virtual void AppendStatus(const StringRef& reply) noexcept = 0;

	// Return true if this is an absolute encoder, false if it is relative
	virtual bool IsAbsolute() const noexcept = 0;

	// Offset management
	void AdjustOffset(int32_t newOffset) noexcept { offset += newOffset; }

	// Encoder polarity. Changing this will change subsequent encoder readings, so call AdjustOffset afterwards.
	void SetBackwards(bool backwards) noexcept { reversePolarityMultiplier = (backwards) ? -1 : 1; }

	// Return the encoder polarity
	bool IsBackwards() const noexcept { return reversePolarityMultiplier < 0; }

	// Return the number of encoder counts per step
	float GetCountsPerStep() const noexcept { return countsPerStep; }

	// Return the reciprocal of the number of encoder counts per step
	float GetRecipCountsPerStep() const noexcept { return recipCountsPerStep; }

protected:
	// For adjusting the reading
	int32_t offset = 0;
	int32_t reversePolarityMultiplier = 1;
	float countsPerStep;
	float recipCountsPerStep;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
