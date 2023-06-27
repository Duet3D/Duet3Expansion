/*
 * PositionDecoder.h
 *
 *  Created on: 31 Aug 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_

#include "RelativeEncoder.h"

#if SUPPORT_CLOSED_LOOP && SAME5x

#include <General/FreelistManager.h>

// Class to use the Position Decoder peripheral in the SAME5x as a quadrature decoder
class QuadratureEncoderPdec : public RelativeEncoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<QuadratureEncoderPdec>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<QuadratureEncoderPdec>(p); }

	QuadratureEncoderPdec(uint32_t p_countsPerRev, uint32_t p_stepsPerRev) noexcept
		: RelativeEncoder(4 * p_countsPerRev, p_stepsPerRev), lastCount(0), counterHigh(0) {}
	~QuadratureEncoderPdec() { QuadratureEncoderPdec::Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::rotaryQuadrature; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	void ClearFullRevs() noexcept override;
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

protected:
	// Get the current position relative to the starting position
	int32_t GetRelativePosition(bool& error) noexcept override;

private:
	// Set the position to the 32 bit signed value 'position'
	void SetPosition(int32_t position) noexcept;

	uint16_t lastCount;
	uint32_t counterHigh;
};

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODERPDEC_H_ */
