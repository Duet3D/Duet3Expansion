/*
 * LDC1612.cpp
 *
 *  Created on: 14 Jun 2023
 *      Author: David
 */

#include "LDC1612.h"
#include <cmath>

constexpr uint32_t LDCI2CTimeout = 25;

// Error bits
constexpr uint16_t UR_ERR2OUT = 1u << 15;
constexpr uint16_t OR_ERR2OUT = 1u << 14;
constexpr uint16_t WD_ERR2OUT = 1u << 13;
constexpr uint16_t AH_ERR2OUT = 1u << 12;
constexpr uint16_t AL_ERR2OUT = 1u << 11;
constexpr uint16_t UR_ERR2INT = 1u << 7;
constexpr uint16_t OR_ERR2INT = 1u << 6;
constexpr uint16_t WD_ERR2INT = 1u << 5;
constexpr uint16_t AH_ERR2INT = 1u << 4;
constexpr uint16_t AL_ERR2INT = 1u << 3;
constexpr uint16_t ZC_ERR2INT = 1u << 2;
constexpr uint16_t DRDY_2INT = 1u << 0;

// Sensor configuration bits
constexpr uint16_t CFG_SLEEP_MODE_EN = 1u << 13;
constexpr uint16_t CFG_RP_OVERRIDE_EN = 1u << 12;
constexpr uint16_t CFG_SENSOR_ACTIVATE_SEL = 1u << 11;
constexpr uint16_t CFG_AUTO_AMP_DIS = 1u << 10;
constexpr uint16_t CFG_REF_CLK_SRC = 1u << 9;
constexpr uint16_t CFG_INTB_DIS = 1u << 7;
constexpr uint16_t CFG_HIGH_CURRENT_DRV = 1u << 6;
constexpr uint16_t CFG_RESERVED_BITS = 0x0001;

constexpr uint16_t MUX_AUTOSCAN_EN = 1u << 15;
constexpr uint16_t MUX_RR_SEQUENCE = 1u << 13;
constexpr uint16_t MUX_DEGLITCH_1MHZ = 1u << 0;
constexpr uint16_t MUX_DEGLITCH_3P3MHZ = 4u << 0;
constexpr uint16_t MUX_DEGLITCH_10MHZ = 5u << 0;
constexpr uint16_t MUX_DEGLITCH_33MHZ = 7u << 0;
constexpr uint16_t MUX_RESERVED_BITS = 0b0'0010'0000'1000;

LDC1612::LDC1612(SharedI2CMaster& dev, uint16_t i2cAddress) noexcept
	: SharedI2CClient(dev, i2cAddress)
{
}

// Do a quick test to check whether the sensor is present, returning true if it is
bool LDC1612::CheckPresent() noexcept
{
	uint16_t val;
	return Read16bits(LDCRegister::READ_MANUFACTURER_ID, val) && val == 0x5449
		&& Read16bits(LDCRegister::READ_DEVICE_ID, val) && val == 0x3055;
}

bool LDC1612::SetDefaultConfiguration(uint8_t channel) noexcept
{
	SetLC(channel, DefaultInductance, DefaultCapacitance);
	constexpr uint16_t DefaultConfigRegVal = CFG_RP_OVERRIDE_EN | CFG_AUTO_AMP_DIS | CFG_REF_CLK_SRC | CFG_RESERVED_BITS;
	return UpdateConfiguration(DefaultConfigRegVal | CFG_SLEEP_MODE_EN)				// put the device in sleep mode
		&& SetDivisors(channel, DefaultClockFrequency)
		&& SetLCStabilizeTime(channel, DefaultLCStabilizeTime)
		&& SetConversionTime(channel, 0x0546)
		&& SetDriverCurrent(channel, 0xa000)
		&& SetMuxConfiguration(MUX_RESERVED_BITS | MUX_DEGLITCH_1MHZ)			// single conversion
		&& UpdateConfiguration(DefaultConfigRegVal | (channel << 14));			// this also takes the device out of sleep mode
 }

/** @brief read the raw channel result from register.
    @param channel LDC1612 has total two channels.
    @param result raw data
 * */
bool LDC1612::GetChannelResult(uint8_t channel, uint32_t& result) noexcept
{
	uint16_t value;
	bool ok = Read16bits((LDCRegister)((uint8_t)LDCRegister::CONVERSION_RESULT_REG_START + channel * 2), value);
	if (ok)
	{
		const uint32_t raw_value = (uint32_t)value << 16;
		ok = Read16bits((LDCRegister)((uint8_t)LDCRegister::CONVERSION_RESULT_REG_START + channel * 2 + 1), value);
		if (ok)
		{
			result = (raw_value | (uint32_t)value) & 0x0fffffff;
		}
	}
	return ok;
}

// Set the conversion time in units of 16 divided reference clocks. Minimum is 5.
bool LDC1612::SetConversionTime(uint8_t channel, uint16_t value) noexcept
{
    return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_CONVERSION_TIME_REG_START + channel), value);
}

/** @brief set conversion offset.
    @param channel LDC1612 has total two channels.
    @param result The value to be set.
 * */
bool LDC1612::SetConversionOffset(uint8_t channel, uint16_t value) noexcept
{
    return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_CONVERSION_OFFSET_REG_START + channel), value);
}

/** @brief Before conversion,wait LC sensor stabilize for a short time.
    @param channel LDC1612 has total two channels.
    @param result The value to be set.
 * */
bool LDC1612::SetLCStabilizeTime(uint8_t channel, uint16_t value) noexcept
{
    return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_LC_STABILIZE_REG_START + channel), value);
}

bool LDC1612::SetDivisors(uint8_t channel, float clockFrequency) noexcept
{
	const float resonantFrequency = 1 / (2 * 3.14 * sqrtf(inductance[channel] * capacitance[channel] * powf(10, -18))) * powf(10, -6);
	const uint16_t FIN_DIV = (uint16_t)(resonantFrequency / 8.75 + 1);
	const uint16_t FREF_DIV = (resonantFrequency * 4 < clockFrequency) ? 2 : 4;
	const uint16_t value = (FIN_DIV << 12) | FREF_DIV;
	return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_FREQ_REG_START + channel), value);
}

/** @brief Error output config.
    @param result The value to be set.
 * */
bool LDC1612::SetErrorConfiguration(uint16_t value) noexcept
{
	return Write16bits(LDCRegister::ERROR_CONFIG_REG, value);
}

/** @brief mux  config.
    @param result The value to be set.
 * */
bool LDC1612::SetMuxConfiguration(uint16_t value) noexcept
{
	return Write16bits(LDCRegister::MUL_CONFIG_REG, value);
}

/** @brief reset sensor.

 * */
bool LDC1612::Reset() noexcept
{
	return Write16bits(LDCRegister::SENSOR_RESET_REG, 0x8000);
}

/** @brief set drive current of sensor.
    @param result The value to be set.
 * */
bool LDC1612::SetDriverCurrent(uint8_t channel, uint16_t value) noexcept
{
	return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_DRIVER_CURRENT_REG + channel), value);
}

/** @brief Main config part of sensor.Contains select channel、start conversion、sleep mode、sensor activation mode、INT pin disable ..
    @param result The value to be set.
 * */
bool LDC1612::UpdateConfiguration(uint16_t value) noexcept
{
	return Write16bits(LDCRegister::SENSOR_CONFIG_REG, value);
}

void LDC1612::SetLC(uint8_t channel, float uh, float pf) noexcept
{
	inductance[channel] = uh;
	capacitance[channel] = pf;
}

constexpr const char* status_str[] =
{
	"conversion under range error",
	"conversion over range error",
	"watch dog timeout error",
	"amplitude high error",
	"amplitude low error",
	"zero Count error",
	"data ready",
	"unread conversion is present for channel 0",
	"unread conversion is present for Channel 1",
	"unread conversion is present for Channel 2",
	"unread conversion is present for Channel 3"
};

// Get sensor status

uint16_t LDC1612::GetStatus() noexcept
{
    uint16_t value = 0;
    Read16bits(LDCRegister::SENSOR_STATUS_REG, value);
    return value;
}

// Read a single 8-bit register
bool LDC1612::ReadRegister(LDCRegister reg, uint8_t& val) noexcept
{
	return Transfer((uint8_t)reg, &val, 1, 1, LDCI2CTimeout);
}

// Write a single 8-bit register
bool LDC1612::WriteRegister(LDCRegister reg, uint8_t val) noexcept
{
	return Transfer((uint8_t)reg, &val, 2, 0, LDCI2CTimeout);
}

// Read a pair of registers to give a 16-bit result, where the lower register is the MSB
bool LDC1612::Read16bits(LDCRegister reg, uint16_t& val) noexcept
{
	uint8_t data[2];
	const bool ok = Transfer((uint8_t)reg, data, 1, 2, LDCI2CTimeout);
	if (ok)
	{
		val = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
	}
	return ok;
}

bool LDC1612::Write16bits(LDCRegister reg, uint16_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)((val >> 8) & 0xff), (uint8_t)(val & 0xff) };
	return Transfer((uint8_t)reg, data, 3, 0, LDCI2CTimeout);
}

// End
