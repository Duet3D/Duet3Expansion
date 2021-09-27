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

#include <GCodes/GCodeResult.h>

NamedEnum(EncoderPositioningType, uint8_t, absolute, relative);

class Encoder
{
public:
	Encoder() noexcept { }
	virtual ~Encoder() { }

	// Get the type of this encoder
	virtual EncoderType GetType() const noexcept = 0;

	// Get the positioning type of this encoder (absolute or relative)
	virtual EncoderPositioningType GetPositioningType() const noexcept = 0;

	// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
	virtual GCodeResult Init(const StringRef& reply) noexcept = 0;

	// Enable the encoder
	virtual void Enable() noexcept = 0;

	// Disable the encoder
	virtual void Disable() noexcept = 0;

	// Get the current reading
	virtual int32_t GetReading() noexcept = 0;

	// Get diagnostic information and append it to a string
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	// Append brief encoder status as a string
	virtual void AppendStatus(const StringRef& reply) noexcept = 0;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
