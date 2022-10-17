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
	Encoder() noexcept { }
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
	virtual int32_t GetReading() noexcept = 0;

	// Get diagnostic information and append it to a string
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	// Append brief encoder status as a string
	virtual void AppendStatus(const StringRef& reply) noexcept = 0;

	// Return true if this is an absolute encoder, false if it is relative
	virtual bool IsAbsolute() const noexcept = 0;

	// Offset management
	void AdjustOffset(int32_t newOffset) noexcept { offset += newOffset; }

protected:

	// For calculating the raw reading
	int32_t offset = 0;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
