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

QuadratureEncoderPdec::QuadratureEncoderPdec(bool linear) : counterHigh(0), lastCount(0)
{
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_PDEC;
	hri_gclk_write_PCHCTRL_reg(GCLK, PDEC_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN);

	PDEC->CTRLA.bit.ENABLE = 0;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
	PDEC->CTRLA.bit.SWRST = 1;
	while (PDEC->SYNCBUSY.bit.SWRST) { }

	for (Pin p : PositionDecoderPins)
	{
		SetPinFunction(p, PositionDecoderPinFunction);
	}

	SetCountsPerRev((linear) ? 0 : 4000);
}

QuadratureEncoderPdec::~QuadratureEncoderPdec()
{
	QuadratureEncoderPdec::Disable();
}

// Overridden virtual functions
void QuadratureEncoderPdec::Enable() noexcept
{
	SetPosition(0, 0);
	PDEC->CTRLA.bit.ENABLE = 1;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
	while (PDEC->SYNCBUSY.bit.CTRLB) { }
}

void QuadratureEncoderPdec::Disable() noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
	while (PDEC->SYNCBUSY.bit.CTRLB) { }
	PDEC->CTRLA.bit.ENABLE = 0;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
}

int32_t QuadratureEncoderPdec::GetReading() noexcept
{
	uint16_t pos = 0;
	const int32_t r = GetPosition(pos);
	return ((cpr == 0) ? r : (r * (int32_t)cpr) + (int32_t)pos) + offset;
}

void QuadratureEncoderPdec::SetOffset(int32_t offset) noexcept
{
	this->offset += offset;
}

void QuadratureEncoderPdec::AppendDiagnostics(const StringRef &reply) noexcept
{
	// Nothing needed here yet
#if 1	//debug
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.reg & (PDEC_SYNCBUSY_CTRLB | PDEC_SYNCBUSY_COUNT)) { }
	const uint16_t count = PDEC->COUNT.reg;
	reply.catf(", raw count = %u", count);
#endif
}

// End of overridden virtual functions

void QuadratureEncoderPdec::SetCountsPerRev(uint16_t p_cpr) noexcept
{
	cpr = p_cpr;
	uint32_t ctrla = PDEC_CTRLA_MODE_QDEC | PDEC_CTRLA_CONF_X4
					| PDEC_CTRLA_PINEN0 | PDEC_CTRLA_PINEN1 | PDEC_CTRLA_PINEN2;
	if (cpr == 0)
	{
		ctrla |= PDEC_CTRLA_ANGULAR(7);
		positionBits = 16;
	}
	else
	{
		positionBits = 9;
		while (positionBits < 16 && (1u << positionBits) < cpr)
		{
			++positionBits;
		}
		ctrla |= PDEC_CTRLA_ANGULAR(positionBits - 9) | PDEC_CTRLA_PEREN;
		PDEC->CC[0].reg = cpr - 1;
	}

	PDEC->CTRLA.reg = ctrla;
}

// Get the current position
// In rotary mode, return the number of rotations, and pass back the position within the current rotation in 'pos'
// In linear mode, 'pos' is not used and a 32-bit position is returned.
int32_t QuadratureEncoderPdec::GetPosition(uint16_t& pos) noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.reg & (PDEC_SYNCBUSY_CTRLB | PDEC_SYNCBUSY_COUNT)) { }
	const uint16_t count = PDEC->COUNT.reg;
	if (cpr == 0 || positionBits <= 14)
	{
		// Handle wrap around of the high position bits or the low revolution bits
		const uint16_t currentHighBits = count >> 14;
		const uint16_t lastHighBits = lastCount >> 14;
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
			if ((count & 0x7FFF) >= cpr/2 && (lastCount & 0x7FFF) < cpr/2)
			{
				--counterHigh;
			}
		}
		else if ((count & 0x8000) == 0 && (lastCount & 0x8000) != 0)
		{
			if ((count & 0x7FFF) < cpr/2 && (lastCount & 0x7FFF) >= cpr/2)
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
