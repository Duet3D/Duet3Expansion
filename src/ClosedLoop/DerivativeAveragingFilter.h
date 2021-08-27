/*
 * DerivativeAveragingFilter.h
 *
 *  Created on: 25 Aug 2021
 *      Author: Louis
 */

#ifndef SRC_DERIVATIVEAVERAGINGFILTER_H_
#define SRC_DERIVATIVEAVERAGINGFILTER_H_

#include "RepRapFirmware.h"

// Class that takes in readings and timestamps
// and outputs the current derivative
// n should be a power of 2 for best efficiency
template<size_t n> class DerivativeAveragingFilter
{
public:
	DerivativeAveragingFilter() noexcept : init(false), valid(false) {}

	void Init(float reading, float timestamp) volatile noexcept
	{
		TaskCriticalSectionLocker lock;

		index = 0;
		for (size_t i = 0; i < n; ++i)
		{
			readings[i] = reading;
			timestamps[i] = timestamp;
		}
		init = true;
	}

	// Call this to put a new reading into the filter
	void ProcessReading(float reading, float timestamp) noexcept
	{
		TaskCriticalSectionLocker lock;

		float prevReading = readings[index];
		float prevTimestamp = timestamps[index];

		readings[index] = reading;
		timestamps[index] = timestamp;

		float readingDelta = reading - prevReading;
		float timestampDelta = timestamp - prevTimestamp;

		if (timestampDelta > 0) {
			derivative = readingDelta / timestampDelta;
		} else {
			derivative = 0;
		}

		++index;
		if (index == n)
		{
			index = 0;
			valid = true;
		}
	}

	bool IsInit() volatile noexcept { return init; }
	bool IsValid() volatile noexcept { return valid; }
	static constexpr size_t N() noexcept { return n; }
	float GetDerivative() noexcept { return derivative; }

private:
	bool init;
	bool valid;
	size_t index;
	float derivative;
	float readings[n];
	float timestamps[n];
};

#endif /* SRC_DERIVATIVEAVERAGINGFILTER_H_ */
