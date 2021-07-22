/*
 * PositionDecoder.cpp
 *
 *  Created on: 31 Aug 2020
 *      Author: David
 */

#include <RepRapFirmware.h>

#if SAME5x && SUPPORT_CLOSED_LOOP

#include "QuadratureEncoderPdec.h"
#include <hri_mclk_e54.h>
#include <cmath>

QuadratureEncoderPdec::QuadratureEncoderPdec(bool linear) : counterHigh(0), lastCount(0), cpr((linear) ? 0 : 1)
{
	hri_mclk_set_APBCMASK_PDEC_bit(MCLK);		// enable the bus clock
	hri_gclk_write_PCHCTRL_reg(GCLK, PDEC_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);

	for (Pin p : PositionDecoderPins)
	{
		SetPinFunction(p, PositionDecoderPinFunction);
	}
}

QuadratureEncoderPdec::~QuadratureEncoderPdec()
{
	QuadratureEncoderPdec::Disable();
}

// Overridden virtual functions
void QuadratureEncoderPdec::Enable() noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
}

void QuadratureEncoderPdec::Disable() noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
}

int32_t QuadratureEncoderPdec::GetReading() noexcept
{
	uint16_t pos = 0;
	return GetPosition(pos);
	//TODO what about pos?
}

void QuadratureEncoderPdec::AppendDiagnostics(const StringRef &reply) noexcept
{
	// Nothing needed here yet
}

// End of overridden virtual functions

void QuadratureEncoderPdec::SetCountsPerRev(uint16_t p_cpr) noexcept
{
	cpr = p_cpr;
	uint32_t ctrla = PDEC_CTRLA_MODE_QDEC
					| PDEC_CTRLA_PINEN0 | PDEC_CTRLA_PINEN1 | PDEC_CTRLA_PINEN2;
	if (cpr == 0)
	{
		ctrla |= PDEC_CTRLA_ANGULAR(7) | PDEC_CTRLA_CONF_X4;
		positionBits = 16;
	}
	else
	{
		positionBits = 9;
		while (positionBits < 16 && (1u << positionBits) < cpr)
		{
			++positionBits;
		}
		ctrla |= PDEC_CTRLA_ANGULAR(positionBits - 9) | PDEC_CTRLA_CONF_X4S | PDEC_CTRLA_PEREN;
		PDEC->CC[0].reg = cpr - 1;
	}

	PDEC->CTRLA.reg = ctrla;
}

// Get the current position. In linear mode, 'pos' is not used.
int32_t QuadratureEncoderPdec::GetPosition(uint16_t& pos) noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.bit.COUNT) { }
	const uint16_t count = PDEC->COUNT.reg;
	if (cpr == 0 || positionBits <= 14)
	{
		// Handle wrap around of the high position bits or the low revolution bits
		const uint16_t currentHighBits = count >> 14;
		const uint16_t lastHighBits = count >> 14;
		if (currentHighBits == 3 && lastHighBits == 0)
		{
			--counterHigh;
		}
		else if (currentHighBits == 0 && lastHighBits == 3)
		{
			++counterHigh;
		}
	}
	else if (positionBits == 15)
	{
		// Handle 15-bit rotary quadrature encoders (do they exist?)
		if ((count & 0x8000) != 0 && (lastCount & 0x8000) == 0)
		{
			if (count >= cpr/2 && lastCount < cpr/2)
			{
				--counterHigh;
			}
		}
		else if ((count & 0x8000) == 0 && (lastCount & 0x8000) != 0)
		{
			if (count < cpr/2 && lastCount >= cpr/2)
			{
				++counterHigh;
			}
		}
	}
	lastCount = count;

	int32_t ret;
	if (cpr == 0)
	{
		// Linear mode
		pos = 0;
		ret = (int32_t)((counterHigh << 16) | count);
	}
	else
	{
		// Rotary mode
		pos = count & ((1u << positionBits) - 1);
		ret = (int32_t)((count >> positionBits) | (counterHigh << (16 - positionBits)));
	}
	return ret;
}

// Set the position. In linear mode, 'revs' is the linear position and 'pos' is not used.
void QuadratureEncoderPdec::SetPosition(int32_t revs, uint16_t pos) noexcept
{
	while (PDEC->SYNCBUSY.bit.STATUS) { }
	const bool stopped = PDEC->STATUS.bit.STOP;
	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}

	if (cpr == 0)
	{
		PDEC->COUNT.reg = lastCount = (uint16_t)revs;
		counterHigh = (uint32_t)revs >> 16;
	}
	else
	{
		// Adjust pos and revs so that pos is in the correct range
		ldiv_t v = ldiv(pos, (long int)cpr);
		revs += v.quot;
		if (v.rem < 0)
		{
			v.rem += cpr;
			--revs;
		}

		PDEC->COUNT.reg = lastCount = ((uint16_t)v.rem & ((1u << positionBits) - 1)) | (uint16_t)((uint32_t)revs << positionBits);
		counterHigh = (uint32_t)revs >> (16u - positionBits);
	}

	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}
}

#endif	// SUPPORT_CLOSED_LOOP
