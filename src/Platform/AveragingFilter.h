/*
 * AveragingFilter.h
 *
 *  Created on: 7 Sep 2018
 *      Author: David
 */

#ifndef SRC_PLATFORM_AVERAGINGFILTER_H_
#define SRC_PLATFORM_AVERAGINGFILTER_H_

#include "RepRapFirmware.h"
#include "RTOSIface/RTOSIface.h"

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template<size_t numAveraged> class AveragingFilter
{
public:
	AveragingFilter() noexcept
	{
		Init(0);
	}

	void Init(uint16_t val) noexcept
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
	void ProcessReading(uint16_t r) noexcept
	{
		TaskCriticalSectionLocker lock;

		sum = sum - readings[index] + r;
		readings[index] = r;
		if (index == numAveraged - 1)
		{
			index = 0;
			isValid = true;
		}
		else
		{
			++index;
		}
	}

	// Return the raw sum
	uint32_t GetSum() const volatile noexcept
	{
		return sum;
	}

	// Return true if we have a valid average
	bool IsValid() const volatile noexcept
	{
		return isValid;
	}

	static constexpr size_t NumAveraged() noexcept { return numAveraged; }

	// Function used as an ADC callback to feed a result into an averaging filter
	static void CallbackFeedIntoFilter(CallbackParameter cp, int32_t val) noexcept;

	bool CheckIntegrity() const noexcept;

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

// This is called from an ISR or high priority task to add a new reading to the filter.
template<size_t numAveraged> void AveragingFilter<numAveraged>::CallbackFeedIntoFilter(CallbackParameter cp, int32_t val) noexcept
{
	static_cast<AveragingFilter<numAveraged>*>(cp.vp)->ProcessReading((uint16_t)val);
}

template<size_t numAveraged> bool AveragingFilter<numAveraged>::CheckIntegrity() const noexcept
{
	AtomicCriticalSectionLocker lock;

	uint32_t locSum = 0;
	for (size_t i = 0; i < numAveraged; ++i)
	{
		locSum += readings[i];
	}
	return locSum == sum;
}

#endif /* SRC_PLATFORM_AVERAGINGFILTER_H_ */
