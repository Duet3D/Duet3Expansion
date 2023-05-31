/*
 * SampleBuffer.cpp
 *
 *  Created on: 22 May 2023
 *      Author: David
 */

#include "SampleBuffer.h"
#include <General/Portability.h>
#include <cstring>

void SampleBuffer::Init(size_t p_bytesPerSample) noexcept
{
	numBytesPerSample = p_bytesPerSample;
	roundedUpBytesPerSample = RoundUpToDword(p_bytesPerSample);		// bytes per sample rounded up to next multiple of 4

	// Calculate how many samples will fit in the buffer
	const size_t samplesPerBuffer = sizeof(data)/p_bytesPerSample;
	limit = samplesPerBuffer * roundedUpBytesPerSample;	// wrap the read/write pointers round when they reach this value
	writePointer = 0;
	readPointer = 0;
	tempWritePointer = 0;
	full = false;
}

size_t SampleBuffer::GetSample(uint8_t *dest) noexcept
{
	memcpy(dest, data + readPointer, numBytesPerSample);
	readPointer += roundedUpBytesPerSample;
	if (readPointer == limit)
	{
		readPointer = 0;
	}
	badSample = full = false;
	return numBytesPerSample;
}

// Call this when all the data for a sample has been put to the buffer
bool SampleBuffer::FinishSample() noexcept
{
	if (tempWritePointer == writePointer + numBytesPerSample)
	{
		tempWritePointer = writePointer + roundedUpBytesPerSample;
		if (tempWritePointer == limit)
		{
			tempWritePointer = 0;
		}
		writePointer = tempWritePointer;
		if (tempWritePointer == readPointer)
		{
			full = true;
		}
		return true;
	}
	badSample = true;
	return false;
}

// End

