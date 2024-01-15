/*
 * AS5601.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "AS5601.h"

#if SUPPORT_AS5601

AS5601::AS5601(SharedI2CMaster& dev) noexcept
	: SharedI2CClient(dev, AS5601_I2cAddress)
{
	// TODO Auto-generated constructor stub

}

bool AS5601::Read8(AS5601Register reg, uint8_t& val) noexcept
{
	uint8_t data[2];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 1, AS5601_I2CTimeout);
	if (ok)
	{
		val = data[1];
	}
	return ok;
}

bool AS5601::Read16(AS5601Register reg, uint16_t val) noexcept
{
	uint8_t data[3];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 2, AS5601_I2CTimeout);
	if (ok)
	{
		val = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
	}
	return ok;
}

bool AS5601::Write8(AS5601Register reg, uint8_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)reg, val };
	return Transfer(data, nullptr, 2, 0, AS5601_I2CTimeout);
}

bool AS5601::Write16(AS5601Register reg, uint16_t val) noexcept
{
	uint8_t data[3] = { (uint8_t)reg, (uint8_t)((val >> 8) & 0xff), (uint8_t)(val & 0xff) };
	return Transfer(data, nullptr, 3, 0, AS5601_I2CTimeout);
}

#endif
