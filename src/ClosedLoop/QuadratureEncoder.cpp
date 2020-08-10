/*
 * QuadratureDecoder.cpp
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#include <ClosedLoop/QuadratureEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <Hardware/IoPorts.h>
#include <ClosedLoop/ClosedLoop.h>

QuadratureEncoder::QuadratureEncoder(bool isLinear) noexcept : Encoder(), linear(isLinear)
{
}

// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
void QuadratureEncoder::Enable() noexcept
{
	// Set up the attiny
	ClosedLoop::DisableEncodersSpi();
	IoPort::SetPinMode(EncoderCsPin, OUTPUT_HIGH);				// make sure any attached encoder doesn't respond to data on the SPI bus
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_LOW);			// put the attiny in reset
	IoPort::SetPinMode(QuadratureErrorOutPin, INPUT_PULLUP);	// make the error out pin an input with pullup so that the attiny reads it high
	IoPort::SetPinMode(QuadratureCountUpPin, INPUT_PULLDOWN);
	IoPort::SetPinMode(QuadratureCountDownPin, INPUT_PULLDOWN);
	delayMicroseconds(100);										// give the reset enough time
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_HIGH);		// release the reset
	delayMicroseconds(100);										// give the attiny time to read the error out pin and start running

	// Set up the events on the input pins
	const ExintNumber upExint = AttachEvent(QuadratureCountUpPin, INTERRUPT_MODE_HIGH, false);
	const ExintNumber downExint = AttachEvent(QuadratureCountDownPin, INTERRUPT_MODE_HIGH, false);
	if (upExint != Nx && downExint != Nx)						// should always be true if the pin assignments and pin table are correct
	{
		// Enable the event system and TCC clocks
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_EVSYS;
		EnableTccClock(QuadratureTccNumber, GclkNum48MHz);		// enable clocks

		// Set up the event system
		// TCC can only be used with an event channel in async mode (SAMC21 Errata rev E section 1.21.9)
		static constexpr uint32_t EVSYS_CHANNEL_EVGEN_EIC_EXINT0 = 0x0E;	// see datasheet (this definition is not in the device pack)
		EVSYS->CHANNEL[CountUpEvent].reg = EVSYS_CHANNEL_EVGEN(EVSYS_CHANNEL_EVGEN_EIC_EXINT0 + upExint) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;
		EVSYS->CHANNEL[CountDownEvent].reg = EVSYS_CHANNEL_EVGEN(EVSYS_CHANNEL_EVGEN_EIC_EXINT0 + downExint) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

		EVSYS->USER[QuadratureCountUpEventUser].reg = CountUpEvent + 1;
		EVSYS->USER[QuadratureCountDownEventUser].reg = CountDownEvent + 1;

		// Set up the TCC
		QuadratureTcc->CTRLA.reg = 0;							// datasheet says disable before reset to avoid undefined behaviour
		while (QuadratureTcc->SYNCBUSY.bit.ENABLE) { }
		QuadratureTcc->CTRLA.reg = TCC_CTRLA_SWRST;
		while (QuadratureTcc->SYNCBUSY.bit.SWRST) { }

		QuadratureTcc->EVCTRL.reg = TCC_EVCTRL_EVACT1_DEC | TCC_EVCTRL_EVACT0_INC | TCC_EVCTRL_TCEI1 | TCC_EVCTRL_TCEI0;
		SetReading(0);

		QuadratureTcc->CTRLA.reg = TCC_CTRLA_ENABLE;
	}
}

// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
void QuadratureEncoder::Disable() noexcept
{
	ClosedLoop::DisableEncodersSpi();
	ClosedLoop::TurnAttinyOff();
}

// Get the 32-bit position
// The low 16 bits are held in the TCC register. We hold the high 16 bits in in memory.
// When but 15 of the TCC register changes, we need to work out whether the counter overflowed or underflowed and adjust the high word.
int32_t QuadratureEncoder::GetReading() noexcept
{
	// Read the TCC register
	QuadratureTcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC_Val;
	while (QuadratureTcc->SYNCBUSY.bit.COUNT) { }
	const uint16_t reg = (uint16_t)QuadratureTcc->COUNT.reg;

	// Check for underflow or overflow
	if (((reg ^ counterLow) & 0x8000) != 0)
	{
		if ((counterLow & 0x8000) == 0)
		{
			++counterHigh;					// overflow
		}
		else
		{
			--counterHigh;					// underflow
		}
	}
	counterLow = reg;						// save for next time
	return (int32_t)(((uint32_t)counterHigh << 16) | reg);
}

// Set the position. Call this after homing.
void QuadratureEncoder::SetReading(int32_t pos) noexcept
{
	const uint32_t upos = (uint32_t)pos;
	// In case of pulses arriving from the encoder, we may need to set this more than once
	do
	{
		counterLow = (uint16_t)upos;
		counterHigh = (uint16_t)(upos >> 16);
		QuadratureTcc->COUNT.reg = upos;
	} while (GetReading() != pos);
}

void QuadratureEncoder::AppendDiagnostics(const StringRef &reply) noexcept
{
	// Nothing needed here yet
}

#endif

// End
