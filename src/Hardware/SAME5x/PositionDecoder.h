/*
 * PositionDecoder.h
 *
 *  Created on: 31 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_POSITIONDECODER_H_
#define SRC_HARDWARE_SAME5X_POSITIONDECODER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

// Class to use the Position Decoder peripheral in the SAME5x as a quadrature decoder
class PositionDecoder
{
public:
	PositionDecoder();

	// Set the counts per motor revolution, for encoders attached to a motor shaft. A value of zero means we are using a linear encoder instead.
	void SetCountsPerRev(uint16_t p_cpr) noexcept;

	// Set the position. In linear mode, 'revs' is the linear position and 'pos' is not used.
	void SetPosition(int32_t revs, uint16_t pos) noexcept;

	// Get the current position. In linear mode, 'pos' is not used.
	int32_t GetPosition(uint16_t& pos) noexcept;

	// Enable or disable the decoder
	void Run(bool enable) noexcept;

private:
	unsigned int positionBits;
	uint32_t counterHigh;
	uint16_t lastCount;
	uint16_t cpr;
};

#endif

#endif /* SRC_HARDWARE_SAME5X_POSITIONDECODER_H_ */
