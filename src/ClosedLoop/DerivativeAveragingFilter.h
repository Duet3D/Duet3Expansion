/*
 * DerivativeAveragingFilter.h
 *
 *  Created on: 25 Aug 2021
 *      Author: Louis
 */

#ifndef SRC_DERIVATIVEAVERAGINGFILTER_H_
#define SRC_DERIVATIVEAVERAGINGFILTER_H_

#include "RepRapFirmware.h"

// Class that takes in readings and timestamps and outputs the current derivative
// N should be a power of 2 for best efficiency
template<size_t N> class DerivativeAveragingFilter
{
public:
	DerivativeAveragingFilter() noexcept { Reset(); }

	void Reset() noexcept { valid = false; index = 0; derivative = 0.0; }

	// Call this to put a new reading into the filter
	void ProcessReading(float reading, uint32_t timestamp) noexcept
	{
		const float prevReading = readings[index];
		const uint32_t prevTimestamp = timestamps[index];

		readings[index] = reading;
		timestamps[index] = timestamp;

		if (valid)
		{
			const float readingDelta = reading - prevReading;
			const uint32_t timestampDelta = timestamp - prevTimestamp;
			derivative = readingDelta / (float)timestampDelta;
		}

		index = (index + 1) % N;
		if (index == 0)
		{
			valid = true;
		}
	}

	bool IsValid() const volatile noexcept { return valid; }
	static constexpr size_t NumValues() noexcept { return N; }
	float GetDerivative() const noexcept { return derivative; }

private:
	bool valid;
	size_t index;
	float derivative;
	float readings[N];
	uint32_t timestamps[N];
};

#endif /* SRC_DERIVATIVEAVERAGINGFILTER_H_ */
