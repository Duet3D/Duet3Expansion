/*
 * SampleBuffer.h
 *
 *  Created on: 22 May 2023
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_SAMPLEBUFFER_H_
#define SRC_CLOSEDLOOP_SAMPLEBUFFER_H_

#include <cstdint>
#include <cstddef>
#include <Duet3Common.h>

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
	static constexpr size_t DataBufferSize = 2000 * MaxClosedLoopSampleLength;	// When collecting data we can accommodate 2000 samples with up to 38 bytes per sample

	alignas(4) uint8_t data[DataBufferSize];	// Ring buffer to store the samples in
	size_t numBytesPerSample;
	volatile size_t readPointer = 0;			// Send this sample next to the main board
	size_t tempWritePointer = 0;
	volatile size_t writePointer = 0;			// Store the next sample at this point in the buffer
	size_t limit;								// the limit for the read/write pointers, to avoid wrapping within a single set of sampled variables
	bool full = false;
	bool badSample = false;						// true if we collected data faster than we could send it
};

#endif /* SRC_CLOSEDLOOP_SAMPLEBUFFER_H_ */
