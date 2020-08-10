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

class Encoder
{
public:
	Encoder() noexcept { }
	virtual ~Encoder() { }

	virtual EncoderType GetType() const noexcept = 0;
	virtual void Enable() noexcept = 0;
	virtual void Disable() noexcept = 0;
	virtual int32_t GetReading() noexcept = 0;
	virtual void AppendDiagnostics(const StringRef& reply) noexcept = 0;

	static void Init() noexcept;
};

#endif

#endif /* SRC_CLOSEDLOOP_ENCODER_H_ */
