/*
 * LDC1612.cpp
 *
 *  Created on: 14 Jun 2023
 *      Author: David
 */

// TI app note https://www.ti.com/lit/an/snoa944/snoa944.pdf contains important information how the resolution achieved
// varies with RCOUNT (which sets the sampling time) and input frequency. We make the following choices:
// fRef = 32MHz (this is available from the SAMC21 and is below the 35MHz limit for continuous sampling mode)
// Sampling time = 1ms. This gives RCOUNT = 32000/16 = 2000 or approx. 0x800.
// The app note says that for fsensor/fref = 0.1 and RCOUNT = 0x800 the resolution is approx. 8e-7 which is close to 1e-6 or 20 bits.

#include "LDC1612.h"

#if SUPPORT_LDC1612

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

// MUX register configuration
constexpr uint16_t MUX_AUTOSCAN_EN = 1u << 15;
constexpr uint16_t MUX_RR_SEQUENCE = 1u << 13;
constexpr uint16_t MUX_DEGLITCH_1MHZ = 1u << 0;
constexpr uint16_t MUX_DEGLITCH_3P3MHZ = 4u << 0;
constexpr uint16_t MUX_DEGLITCH_10MHZ = 5u << 0;
constexpr uint16_t MUX_DEGLITCH_33MHZ = 7u << 0;
constexpr uint16_t MUX_RESERVED_BITS = 0x41 << 3;

LDC1612::LDC1612(SharedI2CMaster& dev, uint16_t i2cAddress) noexcept
	: SharedI2CClient(dev, i2cAddress)
{
	for (size_t channel = 0; channel < NumChannels; ++channel)
	{
		currentSetting[channel] = DefaultDriveCurrentVal;
	}
}

// Do a quick test to check whether the sensor is present, returning true if it is
bool LDC1612::CheckPresent() noexcept
{
	uint16_t val;
	return Read16bits(LDCRegister::READ_MANUFACTURER_ID, val) && val == 0x5449
		&& Read16bits(LDCRegister::READ_DEVICE_ID, val) && val == 0x3055;
}

// Set the default configuration for a channel and enable it
bool LDC1612::SetDefaultConfiguration(uint8_t channel, bool calibrationMode) noexcept
{
	uint16_t configRegVal = CFG_REF_CLK_SRC | CFG_RESERVED_BITS;
	if (!calibrationMode)
	{
		configRegVal |= (CFG_RP_OVERRIDE_EN | CFG_AUTO_AMP_DIS);
	}
	return UpdateConfiguration(configRegVal | CFG_SLEEP_MODE_EN)				// put the device in sleep mode
		&& SetDivisors(channel)
		&& SetLCStabilizeTime(channel, DefaultLCStabilizeTime)
		&& SetConversionTime(channel, DefaultConversionTime)
		&& SetDriveCurrent(channel, currentSetting[channel])
		&& SetMuxConfiguration(MUX_RESERVED_BITS | MUX_DEGLITCH_10MHZ)			// convert a single channel
		&& SetErrorConfiguration(UR_ERR2OUT | OR_ERR2OUT | WD_ERR2OUT | AH_ERR2OUT | AL_ERR2OUT | DRDY_2INT)
		&& SetConversionOffset(channel, 0)
		&& UpdateConfiguration(configRegVal | ((uint16_t)channel << 14));		// this also takes the device out of sleep mode
}

// Use the auto calibration function to establish the correct drive current
bool LDC1612::CalibrateDriveCurrent(uint8_t channel) noexcept
{
	bool ok = SetDefaultConfiguration(channel, true);
	if (ok)
	{
		delay(4);																// this assumes that the conversion time is set to 3ms or less
		ok = ReadInitCurrent(channel, currentSetting[channel])
				&& SetDefaultConfiguration(channel, false);
	}
	return ok;
}

// Read the result register of a channel
bool LDC1612::GetChannelResult(uint8_t channel, uint32_t& result) noexcept
{
	uint16_t msw;
	bool ok = Read16bits((LDCRegister)((uint8_t)LDCRegister::CONVERSION_RESULT_REG_START + channel * 2), msw, false);
	if (ok)
	{
		uint16_t lsw;
		ok = Read16bits((LDCRegister)((uint8_t)LDCRegister::CONVERSION_RESULT_REG_START + channel * 2 + 1), lsw);
		if (ok)
		{
			result = ((uint32_t)msw << 16) | (uint32_t)lsw;
		}
	}
	return ok;
}

// Set the conversion time in units of 16 divided reference clocks. Minimum is 5.
bool LDC1612::SetConversionTime(uint8_t channel, float microseconds) noexcept
{
	const uint16_t value = max<uint16_t>((uint16_t)(microseconds * (FRef/16.0)), 5);
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
bool LDC1612::SetLCStabilizeTime(uint8_t channel, float microseconds) noexcept
{
	const uint16_t value = max<uint16_t>((uint16_t)(microseconds * (FRef/16.0)), 4);
    return Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_LC_STABILIZE_REG_START + channel), value);
}

bool LDC1612::SetDivisors(uint8_t channel) noexcept
{
	const uint16_t FIN_DIV = 1;														// input frequency after division must be less than 8.75MHz - assume this is always the case
	const uint16_t FREF_DIV = ClockDivisor;											// higher divisors are only needed for very low resonant frequencies
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
	return Write16bits(LDCRegister::MUX_CONFIG_REG, value);
}

// Reset the sensor
bool LDC1612::Reset() noexcept
{
	return Write16bits(LDCRegister::SENSOR_RESET_REG, 0x8000);
}

/** @brief set drive current of sensor.
    @param result The value to be set.
 * */
bool LDC1612::SetDriveCurrent(uint8_t channel, uint16_t value) noexcept
{
	value &= 0x1F;
	const bool ok = Write16bits((LDCRegister)((uint8_t)LDCRegister::SET_DRIVER_CURRENT_REG + channel), (uint16_t)(value << 11));
	if (ok)
	{
		currentSetting[channel] = value;
	}
	return ok;
}

bool LDC1612::ReadInitCurrent(uint8_t channel, uint16_t& value) noexcept
{
	uint16_t val;
	const bool ok = Read16bits((LDCRegister)((uint8_t)LDCRegister::SET_DRIVER_CURRENT_REG + channel), val);
	if (ok)
	{
		value = (val >> 6) & 0x1F;
	}
	return ok;
}

// Return true if there is an unread conversion for the specified channel. Returns false if there was an error getting the status.
bool LDC1612::IsChannelReady(uint8_t channel) noexcept
{
	return GetStatus() & (1u << (3 - channel));
}

/** @brief Main config part of sensor.Contains select channel、start conversion、sleep mode、sensor activation mode、INT pin disable ..
    @param result The value to be set.
 * */
bool LDC1612::UpdateConfiguration(uint16_t value) noexcept
{
	return Write16bits(LDCRegister::SENSOR_CONFIG_REG, value);
}

// Get sensor status, returning 0 if an I2C error occurred
uint16_t LDC1612::GetStatus() noexcept
{
    uint16_t value = 0;
    Read16bits(LDCRegister::SENSOR_STATUS_REG, value);
    return value;
}

// Read a pair of registers to give a 16-bit result, where the lower register is the MSB
bool LDC1612::Read16bits(LDCRegister reg, uint16_t& val, bool releaseBus) noexcept
{
	uint8_t data[3];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 2, LDCI2CTimeout, releaseBus);
	if (ok)
	{
		val = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
	}
	return ok;
}

bool LDC1612::Write16bits(LDCRegister reg, uint16_t val) noexcept
{
	uint8_t data[3] = { (uint8_t)reg, (uint8_t)((val >> 8) & 0xff), (uint8_t)(val & 0xff) };
	return Transfer(data, nullptr, 3, 0, LDCI2CTimeout);
}

#endif	// SUPPORT_LDC1612

// End
