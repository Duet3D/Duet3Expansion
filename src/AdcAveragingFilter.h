/*
 * AdcAveragingFilter.h
 *
 *  Created on: 7 Sep 2018
 *      Author: David
 */

#ifndef SRC_ADCAVERAGINGFILTER_H_
#define SRC_ADCAVERAGINGFILTER_H_

#include "RepRapFirmware.h"
#include "RTOSIface/RTOSIface.h"

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template<size_t numAveraged> class AdcAveragingFilter
{
public:
	AdcAveragingFilter()
	{
		Init(0);
	}

	void Init(uint16_t val) volatile
	{
		TaskCriticalSectionLocker lock;

		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for (size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
	}

	// Call this to put a new reading into the filter
	void ProcessReading(uint16_t r)
	{
		TaskCriticalSectionLocker lock;

		sum = sum - readings[index] + r;
		readings[index] = r;
		++index;
		if (index == numAveraged)
		{
			index = 0;
			isValid = true;
		}
	}

	// Return the raw sum
	uint32_t GetSum() const volatile
	{
		return sum;
	}

	// Return the last reading
	uint32_t GetLastReading() const volatile
	{
		return readings[(index - 1) % numAveraged];
	}

	// Return true if we have a valid average
	bool IsValid() const volatile
	{
		return isValid;
	}

	static constexpr size_t NumAveraged() { return numAveraged; }

	// Function used as an ADC callback to feed a result into an averaging filter
	static void CallbackFeedIntoFilter(CallbackParameter cp, uint16_t val);

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

template<size_t numAveraged> void AdcAveragingFilter<numAveraged>::CallbackFeedIntoFilter(CallbackParameter cp, uint16_t val)
{
	static_cast<AdcAveragingFilter<numAveraged>*>(cp.vp)->ProcessReading(val);
}

#endif /* SRC_ADCAVERAGINGFILTER_H_ */
