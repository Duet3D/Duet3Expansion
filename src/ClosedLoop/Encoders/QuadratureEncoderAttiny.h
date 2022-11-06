/*
 * QuadratureDecoder.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODERATTINY_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODERATTINY_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP && defined(EXP1HCE)

#include <General/FreelistManager.h>

class QuadratureEncoderAttiny : public Encoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<QuadratureEncoderAttiny>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<QuadratureEncoderAttiny>(p); }

	QuadratureEncoderAttiny(bool p_linear) noexcept;
	~QuadratureEncoderAttiny();

	EncoderType GetType() const noexcept override { return (linear) ? EncoderType::linearQuadrature : EncoderType::rotaryQuadrature; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	int32_t GetReading() noexcept override;			// Get the 32-bit position
	void AppendDiagnostics(const StringRef& reply) noexcept override;

	void SetReading(int32_t pos) noexcept;			// Set the position. Call this after homing.

private:
	uint16_t counterLow, counterHigh;
	bool linear;									// true if linear, false if rotary
};

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODERATTINY_H_ */
