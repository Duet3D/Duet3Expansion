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
	ClosedLoop::DisableEncodersSpi();
	IoPort::SetPinMode(EncoderCsPin, OUTPUT_HIGH);				// make sure any attached encoder doesn't respond to data on the SPI bus
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_LOW);			// put the attiny in reset
	IoPort::SetPinMode(QuadratureErrorOutPin, INPUT_PULLUP);	// make the error out pin an input with pullup so that the attiny reads it high
	IoPort::SetPinMode(QuadratureCountUpPin, INPUT_PULLDOWN);
	IoPort::SetPinMode(QuadratureCountDownPin, INPUT_PULLDOWN);
	delayMicroseconds(100);										// give the reset enough time
	SetReading(0);
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_HIGH);		// release the reset
	delayMicroseconds(100);										// give the attiny time to read the error out pin and start running
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
