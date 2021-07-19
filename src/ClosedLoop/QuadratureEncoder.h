/*
 * QuadratureDecoder.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODER_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODER_H_

#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <General/FreelistManager.h>
#include <General/NamedEnum.h>

class QuadratureEncoder : public Encoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<QuadratureEncoder>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<QuadratureEncoder>(p); }

	QuadratureEncoder(bool p_linear) noexcept;
	~QuadratureEncoder();

	EncoderType GetType() const noexcept override { return (linear) ? EncoderType::linearQuadrature : EncoderType::rotaryQuadrature; }
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

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODER_H_ */
