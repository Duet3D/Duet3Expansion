/*
 * ExtendedAnalog.c
 *
 *  Created on: 14 Nov 2020
 *      Author: David
 */

#include "ExtendedAnalog.h"
#include <AnalogIn.h>

#ifdef ATEIO

#include <Platform/Platform.h>
#include <Hardware/SharedSpiClient.h>

// AD7327 latches the data in the falling edge of SCLK. Max clock frequency 10MHz, minimum 50kHz. It expects SCLK to be high when /CS changes state. This is SPI mode 2.
constexpr uint32_t AdcClockFrequency = 4000000;
constexpr SpiMode AdcSpiMode = SpiMode::mode2;
constexpr uint16_t ControlRegisterValue = (1 << 15)		// write
										| (0 << 13)		// register number 0 = control register
										| (0 << 10)		// channel number will be or'ed in to this
										| (0 << 8)		// mode = single ended
										| (0 << 6)		// no power management
										| (1 << 5)		// coding = straight binary
										| (1 << 4)		// internal reference
										| (0 << 2);		// sequencer not used

static SharedSpiClient *device = nullptr;

// Write and read 16 bits to the ADC
static uint16_t AdcTransfer(uint16_t dataOut) noexcept
{
	const uint8_t wb[2] = { (uint8_t)(dataOut >> 8), (uint8_t)(dataOut & 255) };
	uint8_t rb[2];
	delayNanoseconds(100);
	device->TransceivePacket(wb, rb, 2);
	delayNanoseconds(100);
	return ((uint16_t)rb[0] << 8) | rb[1];
}

void ExtendedAnalog::Init(SharedSpiDevice& sharedSpi) noexcept
{
	pinMode(ExtendedAdcCsPin, PinMode::OUTPUT_HIGH);
	device = new SharedSpiClient(sharedSpi, AdcClockFrequency, AdcSpiMode, ExtendedAdcCsPin, false);

	device->Select(TaskBase::TimeoutUnlimited);

	// Write the two range registers, select 0V to +10V range
	(void)AdcTransfer((1 << 15) | (1 << 13) | (3 << 11) | (3 << 9) | (3 << 7) | (3 << 5));
	digitalWrite(ExtendedAdcCsPin, true);
	delayMicroseconds(1);
	digitalWrite(ExtendedAdcCsPin, false);
	(void)AdcTransfer((1 << 15) | (2 << 13) | (3 << 11) | (3 << 9) | (3 << 7) | (3 << 5));

	for (uint16_t chan = 0; chan < 8; ++chan)
	{
		digitalWrite(ExtendedAdcCsPin, true);
		delayMicroseconds(1);
		digitalWrite(ExtendedAdcCsPin, false);

		// Write the control register, select single ended input for this channel, internal reference, sequencer not used
		(void)AdcTransfer(ControlRegisterValue | (chan << 10));
	}

	device->Deselect();
}

// Read an ADC channel
uint16_t ExtendedAnalog::AnalogIn(unsigned int chan) noexcept
{
	if (device->Select(200))
	{
		(void)AdcTransfer(ControlRegisterValue | ((chan & 7) << 10));	// set channel to sample next
		// We need to pulse CS low and high again to get the ADC to convert the channel we just selected
		digitalWrite(ExtendedAdcCsPin, true);
		delayMicroseconds(10);											// the ADC data acquisition time is this delay plus about 1.5clock cycles
		digitalWrite(ExtendedAdcCsPin, false);
		const uint16_t rslt = AdcTransfer(0);
		device->Deselect();
		if ((rslt >> 13) == chan)										// if the result is for the channel we asked for
		{
			static_assert(AnalogIn::AdcBits >= 14);
			return (rslt & 8191) << (AnalogIn::AdcBits - 13 - 1);		// extend 13-bit result to the required number of bits, leaving the top bit clear
		}
		return 0x8001;													// indicate channel reading error
	}

	return 0x8000;														// indicate select error
}

#endif

// End
