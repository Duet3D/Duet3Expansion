/*
 * PositionDecoder.cpp
 *
 *  Created on: 31 Aug 2020
 *      Author: David
 */

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP && SAME5x

#include "QuadratureEncoderPdec.h"
#include <hri_mclk_e54.h>
#include <cmath>

// Overridden virtual functions

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult QuadratureEncoderPdec::Init(const StringRef& reply) noexcept
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

	// Set count per rev = 0
	uint32_t ctrla = PDEC_CTRLA_MODE_QDEC | PDEC_CTRLA_CONF_X4
					| PDEC_CTRLA_PINEN0 | PDEC_CTRLA_PINEN1			// enable A and B inputs but not the index input
					| PDEC_CTRLA_ANGULAR(7);
	PDEC->CTRLA.reg = ctrla;

	// There's little if anything we can do to test the encoder
	Enable();
	return GCodeResult::ok;
}

void QuadratureEncoderPdec::Enable() noexcept
{
	SetPosition(0);
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

void QuadratureEncoderPdec::ClearFullRevs() noexcept
{
	counterHigh = (lastCount & 0x8000) ? 0xFFFF : 0;
	(void)TakeReading();
}

void QuadratureEncoderPdec::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf("Encoder reverse polarity: %s", (IsBackwards()) ? "yes" : "no");

#if 1	//debug
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.reg & (PDEC_SYNCBUSY_CTRLB | PDEC_SYNCBUSY_COUNT)) { }
	const uint16_t count = PDEC->COUNT.reg;
	reply.catf(", raw count %u", count);
#endif
}

void QuadratureEncoderPdec::AppendStatus(const StringRef& reply) noexcept
{
	reply.lcatf("Quadrature encoder pulses/rev: %.2f", (double)(countsPerRev / 4));
}

// Get the current position relative to the starting position
int32_t QuadratureEncoderPdec::GetRelativePosition(bool& error) noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.reg & (PDEC_SYNCBUSY_CTRLB | PDEC_SYNCBUSY_COUNT)) { }
	const uint16_t count = PDEC->COUNT.reg;

	// Handle wrap around of the high position bits
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

	lastCount = count;

	error = false;
	return (int32_t)((counterHigh << 16) | count);
}

// End of overridden virtual functions

// Set the position to the 32 bit signed value 'position'
void QuadratureEncoderPdec::SetPosition(int32_t position) noexcept
{
	while (PDEC->SYNCBUSY.bit.STATUS) { }
	const bool stopped = PDEC->STATUS.bit.STOP;
	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}

	PDEC->COUNT.reg = lastCount = (uint16_t)position;
	counterHigh = (uint32_t)position >> 16;

	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}
}

#endif	// SUPPORT_CLOSED_LOOP
