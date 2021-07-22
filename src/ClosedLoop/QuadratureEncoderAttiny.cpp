/*
 * QuadratureDecoder.cpp
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#include "QuadratureEncoderAttiny.h"

#if SUPPORT_CLOSED_LOOP && defined(EXP1HCE)

#include <Hardware/IoPorts.h>
#include <ClosedLoop/ClosedLoop.h>

QuadratureEncoderAttiny::QuadratureEncoderAttiny(bool isLinear) noexcept : Encoder(), linear(isLinear)
{
}

QuadratureEncoderAttiny::~QuadratureEncoderAttiny()
{
	QuadratureEncoderAttiny::Disable();
}

// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
void QuadratureEncoderAttiny::Enable() noexcept
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

	// Enable the event system and TCC clocks
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_EVSYS;

#if 1
	// The datasheet says we don't need the GCLKs for the async event path, so we skip the following
#else
# if SAMC21
	constexpr uint32_t GCLK_EVSYS_CH0 = 6;
# endif
	hri_gclk_write_PCHCTRL_reg(GCLK, GCLK_EVSYS_CH0 + CountUpEvent, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, GCLK_EVSYS_CH0 + CountDownEvent, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);
#endif

	EnableTccClock(QuadratureTccNumber, GclkNum48MHz);			// enable clocks

	// SAMC21 datasheet 29.6.2.1 step 1: enable output of event in the peripheral generating the event
	// In this case the event is transitions of the input pins, managed by the EXINT
	pinMode(QuadratureCountUpPin, INPUT);
	pinMode(QuadratureCountDownPin, INPUT);
	const ExintNumber upExint = AttachEvent(QuadratureCountUpPin, InterruptMode::high, false);
	const ExintNumber downExint = AttachEvent(QuadratureCountDownPin, InterruptMode::high, false);

	if (upExint != Nx && downExint != Nx)						// should always be true if the pin assignments and pin table are correct
	{
		// SAMC21 datasheet 29.6.2.1 step 2: configure the EVSYS
		// TCC can only be used with an event channel in async mode (SAMC21 Errata rev E section 1.21.9)
		// Step 2.1: configure the Event User multiplexer
		EVSYS->USER[QuadratureCountUpEventUser].reg = CountUpEvent + 1;
		EVSYS->USER[QuadratureCountDownEventUser].reg = CountDownEvent + 1;

		// Step 2.2: configure the Event Channel
		static constexpr uint32_t EVSYS_CHANNEL_EVGEN_EIC_EXINT0 = 0x0E;	// see datasheet (this definition is not in the device pack)
		EVSYS->CHANNEL[CountUpEvent].reg = EVSYS_CHANNEL_EVGEN(EVSYS_CHANNEL_EVGEN_EIC_EXINT0 + upExint) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_ONDEMAND;
		EVSYS->CHANNEL[CountDownEvent].reg = EVSYS_CHANNEL_EVGEN(EVSYS_CHANNEL_EVGEN_EIC_EXINT0 + downExint) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_ONDEMAND;

		// Set up the TCC
		QuadratureTcc->CTRLA.reg = 0;							// datasheet says disable before reset to avoid undefined behaviour
		while (QuadratureTcc->SYNCBUSY.bit.ENABLE) { }
		QuadratureTcc->CTRLA.reg = TCC_CTRLA_SWRST;
		while (QuadratureTcc->SYNCBUSY.bit.SWRST) { }

		SetReading(0);

		QuadratureTcc->CTRLBSET.reg = TCC_CTRLBSET_DIR;			// try setting the count down bit

		// Step 3: configure the action to be taken when the peripheral received the event
		QuadratureTcc->EVCTRL.reg = TCC_EVCTRL_EVACT1_DEC | TCC_EVCTRL_EVACT0_INC;

		// Step 4: in the event user peripheral, enable event input by writing a '1' to the respective Event Input Enable bit
		QuadratureTcc->EVCTRL.reg = TCC_EVCTRL_EVACT1_DEC | TCC_EVCTRL_EVACT0_INC | TCC_EVCTRL_TCEI1 | TCC_EVCTRL_TCEI0;

		QuadratureTcc->CTRLA.reg = TCC_CTRLA_ENABLE;
		while (QuadratureTcc->SYNCBUSY.bit.ENABLE) { }
	}
}

// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
void QuadratureEncoderAttiny::Disable() noexcept
{
	//TODO disable the events
	ClosedLoop::DisableEncodersSpi();
	ClosedLoop::TurnAttinyOff();
}

// Get the 32-bit position
// The low 16 bits are held in the TCC register. We hold the high 16 bits in in memory.
// When but 15 of the TCC register changes, we need to work out whether the counter overflowed or underflowed and adjust the high word.
int32_t QuadratureEncoderAttiny::GetReading() noexcept
{
	// Read the TCC register
	QuadratureTcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
	// On the EXP3HC board it wasn't enough just to wait for SYNCBUSY.COUNT here in similar code for the step timer
	while (QuadratureTcc->CTRLBSET.bit.CMD != 0) { }
	while (QuadratureTcc->SYNCBUSY.bit.COUNT) { }
	const uint16_t reg = (uint16_t)QuadratureTcc->COUNT.reg;

	// Sadly we can't program the TCC to wrap on both underflow and overflow, so we can't extend it to a true 32-bit count
	const int16_t ireg = (int16_t)(reg ^ 0x8000);
	return (int32_t)ireg;
}

// Set the position. Call this after homing.
void QuadratureEncoderAttiny::SetReading(int32_t pos) noexcept
{
	// Constrain the position, otherwise we will hang while reading it back if it is out of range
	const uint32_t upos = (uint16_t)(constrain<int32_t>(pos, -32767, 32767) + 0x8000);
	// In case of pulses arriving from the encoder, we may need to set this more than once
	do
	{
		QuadratureTcc->COUNT.reg = upos;
	} while (GetReading() != pos);
}

void QuadratureEncoderAttiny::AppendDiagnostics(const StringRef &reply) noexcept
{
	// Nothing needed here yet
}

#endif

// End
