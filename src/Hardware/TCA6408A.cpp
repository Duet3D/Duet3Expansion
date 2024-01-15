/*
 * TCA6408A.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "TCA6408A.h"

#if SUPPORT_AS5601				// currently we only use the TCA6408A in conjunction with the AS5601

constexpr unsigned int ButtonBitnum = 0;
constexpr unsigned int RedLedBitnum = 4;
constexpr unsigned int GreenLedBitnum = 5;

constexpr uint8_t TCA8408A_config = (1u << ButtonBitnum);				// button pin is input, LED and unused pins are outputs

TCA6408A::TCA6408A(SharedI2CMaster& dev) noexcept
	: SharedI2CClient(dev, TCA6408A_I2CAddress)
{
}

void TCA6408A::Init() noexcept
{
	if (   Write8(TCA6408ARegister::output, 0xFF)						// ensure that LEDs are off
		&& Write8(TCA6408ARegister::polarityInversion, 0)				// don't invert anything
		&& Write8(TCA6408ARegister::config, TCA8408A_config)			// configure I/O
	   )
	{
		found = true;
	}
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

bool TCA6408A::Write8(TCA6408ARegister reg, uint8_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)reg, val };
	return Transfer(data, nullptr, 2, 0, TCA6408A_I2CTimeout);
}

#endif
