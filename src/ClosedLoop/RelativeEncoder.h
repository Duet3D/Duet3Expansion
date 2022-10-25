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

#include "TuningErrors.h"

class RelativeEncoder : public Encoder
{
public:
	// Constructors
	RelativeEncoder(uint32_t p_stepsPerRev, uint32_t p_countsPerRev) noexcept : Encoder(p_stepsPerRev, (float)p_countsPerRev/(float)p_stepsPerRev) {}

	// Overridden virtual functions

	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return false; }

	// Get the current reading
	bool TakeReading() noexcept override;

	// Tell the encoder what the step phase is at a particular count
	void SetKnownPhaseAtCount(uint32_t phase, int32_t count) noexcept;

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	void ClearFullRevs() noexcept override { currentCount %= (int32_t)countsPerRev; }

	// Encoder polarity. Changing this will change the encoder reading.
	void SetBackwards(bool backwards) noexcept override;

	// Return the encoder polarity
	bool IsBackwards() const noexcept override { return reversePolarityMultiplier < 0; }

	// Set the forward tuning results
	void SetForwardTuningResults(float slope, float origin, float xMean) noexcept { forwardSlope = slope; forwardOrigin = origin; forwardXmean = xMean; }

	// Set the reverse tuning results
	void SetReverseTuningResults(float slope, float origin, float xMean) noexcept { reverseSlope = slope; reverseOrigin = origin; reverseXmean = xMean; }

	// Process the tuning data
	TuningErrors ProcessTuningData() noexcept;

protected:
	// Get the relative position since the start
	virtual int32_t GetRelativePosition(bool& error) noexcept = 0;

private:
	uint32_t countsPerRev;
	int32_t reversePolarityMultiplier = 1;

	// Tuning data
	float forwardSlope;
	float reverseSlope;
	float forwardOrigin;
	float reverseOrigin;
	float forwardXmean;
	float reverseXmean;
};

#endif

#endif /* SRC_CLOSEDLOOP_RELATIVEENCODER_H_ */
