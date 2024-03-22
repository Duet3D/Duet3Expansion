/*
 * AS5601.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "AS5601.h"

#if SUPPORT_AS5601

// AS5601 register bit assignments

// Power mode
constexpr uint16_t ConfPM_Shift = 0;
constexpr uint16_t ConfPM_Mask = 0x03 << ConfPM_Shift;

constexpr uint16_t ConfPM_NOM = 0x00 << ConfPM_Shift;
constexpr uint16_t ConfPM_LPM1 = 0x01 << ConfPM_Shift;
constexpr uint16_t ConfPM_LPM2 = 0x02 << ConfPM_Shift;
constexpr uint16_t ConfPM_LPM3 = 0x03 << ConfPM_Shift;

//	Hysteresis
constexpr uint16_t ConfHYST_Shift = 2;
constexpr uint16_t ConfHYST_Mask = 0x03 << ConfHYST_Shift;

constexpr uint16_t ConfHYST_OF = 0x00 << ConfHYST_Shift;
constexpr uint16_t ConfHYST_LSB1 = 0x01 << ConfHYST_Shift;
constexpr uint16_t ConfHYST_LSB2 = 0x02 << ConfHYST_Shift;
constexpr uint16_t ConfHYST_LSB3 = 0x03 << ConfHYST_Shift;

// Slow filter
constexpr uint16_t ConfSF_Shift = 8;
constexpr uint16_t ConfSF_Mask = 0x03 << ConfSF_Shift;

constexpr uint16_t ConfSF_16X = 0x00 << ConfSF_Shift;
constexpr uint16_t ConfSF_8X = 0x01 << ConfSF_Shift;
constexpr uint16_t ConfSF_4X = 0x02 << ConfSF_Shift;
constexpr uint16_t ConfSF_2X = 0x03 << ConfSF_Shift;

// Fast filter threshold
constexpr uint16_t ConfFTH_Shift = 10;
constexpr uint16_t ConfFTH_Mask = 0x07 << ConfFTH_Shift;

constexpr uint16_t ConfFTH_SF = 0x00 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_6LSB = 0x01 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_7LSB = 0x02 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_9LSB = 0x03 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_18LSB = 0x04 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_21LSB = 0x05 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_24LSB = 0x06 << ConfFTH_Shift;
constexpr uint16_t ConfFTH_10LSB = 0x07 << ConfFTH_Shift;

// Watchdog Timer
constexpr uint16_t ConfWD_Shift = 13;
constexpr uint16_t ConfWD_Mask = 0x01 << ConfWD_Shift;

constexpr uint16_t ConfWD_OFF = 0x00 << ConfWD_Shift;
constexpr uint16_t ConfWD_ON = 0x01 << ConfWD_Shift;

// ABN Mapping
constexpr uint8_t ABN_Mask = 0x0F;

constexpr uint8_t ABN_8_61HZ = 0x00;
constexpr uint8_t ABN_16_122HZ = 0x01;
constexpr uint8_t ABN_32_244HZ = 0x02;
constexpr uint8_t ABN_64_488HZ = 0x03;
constexpr uint8_t ABN_128_976HZ = 0x04;
constexpr uint8_t ABN_256_1K9HZ = 0x05;
constexpr uint8_t ABN_512_3K9H = 0x06;
constexpr uint8_t ABN_1024_7K8HZ = 0x07;
constexpr uint8_t ABN_2048_15K6HZ = 0x08;

// STATUS bits
constexpr uint8_t StatusMD = 0b00100000;			// Magnet detected
constexpr uint8_t StatusML = 0b00010000;			// Magnet too weak (AGC maximum gain overflow)
constexpr uint8_t StatusMH = 0b00001000;			// Magnet too strong (AGC minimum gain overflow)

// The configuration we use
constexpr uint16_t AS5601Config = ConfFTH_18LSB | ConfSF_2X | ConfHYST_LSB3 | ConfPM_NOM;

AS5601::AS5601(SharedI2CMaster& dev) noexcept
	: SharedI2CClient(dev, AS5601_I2CAddress)
{
}

// Initialise the device
bool AS5601::Init() noexcept
{
	if (Write16(AS5601Register::config, AS5601Config))
	{
		uint16_t val;
		if (Read16(AS5601Register::config, val) && val == AS5601Config)
		{
			return true;
		}
	}
	return false;
}

// Read the status, agc and angle registers. Read the angle last because the time is captured just after calling this.
// We don't want to capture the time just before calling this because we may have to wait for the I2C bus to be released by another task.
bool AS5601::Read(uint16_t& angle, uint8_t& status, uint8_t& agc) noexcept
{
	return Read8(AS5601Register::status, status, false) && Read8(AS5601Register::agc, agc, false) && Read16(AS5601Register::rawAngle, angle, true);
}

bool AS5601::Read8(AS5601Register reg, uint8_t& val, bool releaseBus) noexcept
{
	uint8_t data[2];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 1, AS5601_I2CTimeout, releaseBus);
	if (ok)
	{
		val = data[1];
	}
	return ok;
}

bool AS5601::Read16(AS5601Register reg, uint16_t& val, bool releaseBus) noexcept
{
	uint8_t data[3];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 2, AS5601_I2CTimeout, releaseBus);
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
