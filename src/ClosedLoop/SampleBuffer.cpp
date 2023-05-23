/*
 * SampleBuffer.cpp
 *
 *  Created on: 22 May 2023
 *      Author: David
 */

#include "SampleBuffer.h"
#include <General/Portability.h>
#include <cstring>

typedef __fp16 float16_t;			///< A 16-bit floating point type

void SampleBuffer::Init(size_t p_bytesPerSample) noexcept
{
	numBytesPerSample = p_bytesPerSample;
	// Calculate how many samples will fit in the buffer
	const size_t samplesPerBuffer = sizeof(data)/p_bytesPerSample;
	limit = samplesPerBuffer * p_bytesPerSample;	// wrap the read/write pointers round when they reach this value
	writePointer = 0;
	readPointer = 0;
	tempWritePointer = 0;
	full = false;
}

size_t SampleBuffer::GetSample(uint8_t *dest) noexcept
{
	memcpy(dest, data + readPointer, numBytesPerSample);
	readPointer += numBytesPerSample;
	if (readPointer == limit)
	{
		readPointer = 0;
	}
	badSample = full = false;
	return numBytesPerSample;
}

// All values we put are multiples of 2 bytes long, so it's safe to store 16-bit values directly
void SampleBuffer::PutU16(uint16_t val) noexcept
{
	if (tempWritePointer - writePointer + sizeof(uint16_t) < limit)
	{
		*(uint16_t*)(data + tempWritePointer) = val;
		tempWritePointer += sizeof(uint16_t);
	}
	else
	{
		badSample = true;
	}
}

void SampleBuffer::PutI16(int16_t val) noexcept
{
	PutU16((uint16_t)val);
}

void SampleBuffer::PutU32(uint32_t val) noexcept
{
	if (tempWritePointer - writePointer + sizeof(uint32_t) < limit)
	{
		*((uint16_t*)(data + tempWritePointer)) = (uint16_t)val;
		*((uint16_t*)(data + tempWritePointer + sizeof(uint16_t))) = (uint16_t)(val >> 16);
		tempWritePointer += sizeof(uint32_t);
	}
	else
	{
		badSample = true;
	}
}

void SampleBuffer::PutI32(int32_t val) noexcept
{
	PutU32((uint32_t)val);
}

void SampleBuffer::PutF16(float val) noexcept
{
	if (tempWritePointer - writePointer + sizeof(float16_t) < limit)
	{
		*(float16_t*)(data + tempWritePointer) = (float16_t)val;
		tempWritePointer += sizeof(float16_t);
	}
	else
	{
		badSample = true;
	}
}

void SampleBuffer::PutF32(float val) noexcept
{
	if (tempWritePointer - writePointer + sizeof(float) < limit)
	{
		StoreLEF32(data + tempWritePointer, val);
		tempWritePointer += sizeof(float);
	}
	else
	{
		badSample = true;
	}
}

// Call this when all the data for a sample has been put to the buffer
bool SampleBuffer::FinishSample() noexcept
{
	if (tempWritePointer == writePointer + numBytesPerSample)
	{
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

