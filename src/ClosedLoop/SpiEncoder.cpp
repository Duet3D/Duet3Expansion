/*
 * SpiEncoder.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include "SpiEncoder.h"
#include "QuadratureEncoder.h"

#if SUPPORT_CLOSED_LOOP

SharedSpiDevice *SpiEncoder::encoderSpi = nullptr;

/*static*/ void SpiEncoder::Init() noexcept
{
	// Create the shared SPI device
	if (encoderSpi == nullptr)
	{
		encoderSpi = new SharedSpiDevice(EncoderSspiSercomNumber);
	}
	QuadratureEncoder::TurnAttinyOff();
}

SpiEncoder::SpiEncoder(uint32_t clockFreq, SpiMode m, bool polarity, Pin p_csPin)
	: spi(*encoderSpi, clockFreq, m, polarity), csPin(p_csPin)
{
	spi.SetCsPin(p_csPin);
}

// Set up the shared encoder pins for SPI use
void SpiEncoder::EnableSpi()
{
#ifdef EXP1HCE
	gpio_set_pin_function(EncoderMosiPin, EncoderMosiPinPeriphMode);
	gpio_set_pin_function(EncoderSclkPin, EncoderSclkPinPeriphMode);
	gpio_set_pin_function(EncoderMisoPin, EncoderMisoPinPeriphMode);
#else
# error Undefined hardware
#endif
}

// Set up the shared encoder pins for counting from the attiny
void SpiEncoder::DisableSpi()
{
#ifdef EXP1HCE
	gpio_set_pin_function(EncoderMosiPin, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(EncoderSclkPin, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(EncoderMisoPin, GPIO_PIN_FUNCTION_OFF);
#else
# error Undefined hardware
#endif
}

#endif
