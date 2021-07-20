/*
 * PositionDecoder.h
 *
 *  Created on: 31 Aug 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP && defined(EXP1HCL)

#include <General/FreelistManager.h>

// Class to use the Position Decoder peripheral in the SAME5x as a quadrature decoder
class QuadratureEncoderPdec : public Encoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<QuadratureEncoderPdec>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<QuadratureEncoderPdec>(p); }

	QuadratureEncoderPdec(bool p_linear) noexcept;
	~QuadratureEncoderPdec();

	EncoderType GetType() const noexcept override { return (cpr == 0) ? EncoderType::linearQuadrature : EncoderType::rotaryQuadrature; }
	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	int32_t GetReading() noexcept override;			// Get the 32-bit position
	void AppendDiagnostics(const StringRef& reply) noexcept override;

private:
	// Set the counts per motor revolution, for encoders attached to a motor shaft. A value of zero means we are using a linear encoder instead.
	void SetCountsPerRev(uint16_t p_cpr) noexcept;

	// Set the position. In linear mode, 'revs' is the linear position and 'pos' is not used.
	void SetPosition(int32_t revs, uint16_t pos) noexcept;

	// Get the current position. In linear mode, 'pos' is not used.
	int32_t GetPosition(uint16_t& pos) noexcept;

	unsigned int positionBits;
	uint32_t counterHigh;
	uint16_t lastCount;
	uint16_t cpr;
};

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_ */
