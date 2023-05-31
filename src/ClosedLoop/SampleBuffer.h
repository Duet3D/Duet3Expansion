/*
 * SampleBuffer.h
 *
 *  Created on: 22 May 2023
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_SAMPLEBUFFER_H_
#define SRC_CLOSEDLOOP_SAMPLEBUFFER_H_

#include <RepRapFirmware.h>

constexpr size_t RoundUpToDword(size_t val) noexcept { return (val + 3) & (~3); }

// Data collection buffer and related variables
class SampleBuffer
{
public:
	SampleBuffer() noexcept { };

	void PutI16(int16_t val) noexcept;
	void PutU16(uint16_t val) noexcept;
	void PutI32(int32_t val) noexcept;
	void PutU32(uint32_t val) noexcept;
	void PutF16(float val) noexcept;
	void PutF32(float val) noexcept;

	void Init(size_t p_bytesPerSample) noexcept;
	bool HasSample() const noexcept { return readPointer != writePointer || full; }
	size_t GetSample(uint8_t *dest) noexcept;
	bool IsFull() const noexcept { return full; }
	bool HadBadSample() const noexcept { return badSample; }
	size_t GetBytesPerSample() const noexcept { return numBytesPerSample; }
	bool FinishSample() noexcept;

private:
	static constexpr size_t DataBufferSize = 2000 * RoundUpToDword(MaxClosedLoopSampleLength);	// When collecting data we can accommodate 2000 samples with up to 38 bytes per sample

	alignas(4) uint8_t data[DataBufferSize];	// Ring buffer to store the samples in
	size_t numBytesPerSample;
	size_t roundedUpBytesPerSample;
	volatile size_t readPointer = 0;			// Send this sample next to the main board
	size_t tempWritePointer = 0;
	volatile size_t writePointer = 0;			// Store the next sample at this point in the buffer
	size_t limit;								// the limit for the read/write pointers, to avoid wrapping within a single set of sampled variables
	bool full = false;
	bool badSample = false;						// true if we collected data faster than we could send it
};

// All values we put are multiples of 2 bytes long, so it's safe to store 16-bit values directly
inline void SampleBuffer::PutU16(uint16_t val) noexcept
{
	*reinterpret_cast<uint16_t*>(data + tempWritePointer) = val;
	tempWritePointer += sizeof(uint16_t);
}

inline void SampleBuffer::PutU32(uint32_t val) noexcept
{
	*reinterpret_cast<uint32_t*>(data + tempWritePointer) = val;
	tempWritePointer += sizeof(uint32_t);
}

inline void SampleBuffer::PutF16(float val) noexcept
{
	*reinterpret_cast<float16_t*>(data + tempWritePointer) = (float16_t)val;
	tempWritePointer += sizeof(float16_t);
}

inline void SampleBuffer::PutI16(int16_t val) noexcept
{
	*reinterpret_cast<int16_t*>(data + tempWritePointer) = val;
	tempWritePointer += sizeof(int16_t);
}

inline void SampleBuffer::PutI32(int32_t val) noexcept
{
	*reinterpret_cast<int32_t*>(data + tempWritePointer) = val;
	tempWritePointer += sizeof(int32_t);
}

inline void SampleBuffer::PutF32(float val) noexcept
{
	*reinterpret_cast<float*>(data + tempWritePointer) = val;
	tempWritePointer += sizeof(float);
}

#endif /* SRC_CLOSEDLOOP_SAMPLEBUFFER_H_ */
