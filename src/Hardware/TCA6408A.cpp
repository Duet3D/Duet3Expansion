/*
 * TCA6408A.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "TCA6408A.h"

#if SUPPORT_TCA6408A

TCA6408A::TCA6408A(SharedI2CMaster& dev) noexcept
	: SharedI2CClient(dev, TCA6408A_I2CAddress)
{
}

bool TCA6408A::Init(uint8_t inputPins, uint8_t initialOutputs) noexcept
{
	outputRegister = initialOutputs;
	uint8_t val;
	return (   Write8(TCA6408ARegister::output, outputRegister)				// ensure that the LEDs are off
			&& Write8(TCA6408ARegister::polarityInversion, 0)				// don't invert anything
			&& Write8(TCA6408ARegister::config, inputPins)					// configure I/O
			&& Read8(TCA6408ARegister::input, inputRegister)				// read the initial inputs
			&& Read8(TCA6408ARegister::config, val)							// check that the config register reads back correctly
			&& val == inputPins
	   	   );
}

bool TCA6408A::Read8(TCA6408ARegister reg, uint8_t& val) noexcept
{
	uint8_t data[2];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 1, TCA6408A_I2CTimeout);
	if (ok)
	{
		val = data[1];
	}
	return ok;
}

void TCA6408A::SetOneOutputBitState(unsigned int bitnum, bool on) noexcept
{
	uint8_t newOutputRegister = outputRegister;
	if (on)
	{
		newOutputRegister &= ~(1u << bitnum);
	}
	else
	{
		newOutputRegister |= (1u << bitnum);
	}
	if (newOutputRegister != outputRegister)
	{
		outputRegister = newOutputRegister;
		outputNeedsUpdating = true;
	}
}

void TCA6408A::SetOutputBitsState(uint8_t bitsToSet, uint8_t mask) noexcept
{
	const uint8_t newOutputRegister = (outputRegister & (~mask)) | bitsToSet;
	if (newOutputRegister != outputRegister)
	{
		outputRegister = newOutputRegister;
		outputNeedsUpdating = true;
	}
}

void TCA6408A::Poll() noexcept
{
	if (outputNeedsUpdating)
	{
		outputNeedsUpdating = false;
		Write8(TCA6408ARegister::output, outputRegister);
	}
	(void)Read8(TCA6408ARegister::input, inputRegister);			// this will update the saved input register if it succeeds
}

bool TCA6408A::Write8(TCA6408ARegister reg, uint8_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)reg, val };
	return Transfer(data, nullptr, 2, 0, TCA6408A_I2CTimeout);
}

#endif
