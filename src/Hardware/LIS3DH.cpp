/*
 * LIS3DH.cpp
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#include "LIS3DH.h"

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH

constexpr uint16_t Lis3dAddress = 0b0011000;						// bottom bit is 1 if SDO/SA0 connected to Vcc, 0 if connected to ground
constexpr uint32_t Lis3dI2CTimeout = 25;

LIS3DH::LIS3DH(SharedI2CMaster& dev, bool addressLSB) noexcept : SharedI2CClient(dev, (addressLSB) ? Lis3dAddress | 0x0001 : Lis3dAddress)
{
	ctrlRegisters.ctrlReg0 = 0x10;									// leave SA0 pullup enabled
	ctrlRegisters.tempCfgReg = 0x40;								// enable temperature sensor
	ctrlRegisters.ctrlReg1 = (0x9 << 4) | 0x07;						// 1.344kHz high resolution mode, XYZ enabled
	ctrlRegisters.ctrlReg2 = (0x2 << 6) | (0x0 << 4) | (0 << 3);	// filter selection
	ctrlRegisters.ctrlReg3 = 0;										// no interrupts enabled
	ctrlRegisters.ctrlReg4 = (1 << 7);								// block data update, little-endian, high resolution mode disabled, full scale = +/- 2g, self test disabled
	ctrlRegisters.ctrlReg5 = 0;										// FIFO disabled
	ctrlRegisters.ctrlReg6 = 0;										// no interrupts
}

template<class T> bool LIS3DH::ReadRegisters(T& registers) noexcept
{
	return Transfer(T::FirstRegNum, (uint8_t*)&registers, 1, sizeof(T), Lis3dI2CTimeout);
}

template<class T> bool LIS3DH::WriteRegisters(const T& registers) noexcept
{
	return Transfer(T::FirstRegNum, (uint8_t*)&registers, 1 + sizeof(T), 0, Lis3dI2CTimeout);
}

bool LIS3DH::ReadRegister(uint8_t regNum, uint8_t& val) noexcept
{
	return Transfer(regNum, &val, 1, 1, Lis3dI2CTimeout);
}

bool LIS3DH::WriteRegister(uint8_t regNum, uint8_t val) noexcept
{
	if (regNum < 0x1E)						// don't overwrite the factory calibration values
	{
		return false;
	}
	return Transfer(regNum, &val, 2, 0, Lis3dI2CTimeout);
}

bool LIS3DH::CheckPresent() noexcept
{
	uint8_t val;
	return ReadRegister(WhoAmIRegister, val) && val == WhoAmIValue;
}

bool LIS3DH::ReadOutputRegisters() noexcept
{
	return ReadRegisters(outRegisters);
}

bool LIS3DH::WriteControlRegisters() noexcept
{
	return WriteRegisters(ctrlRegisters);
}

#endif

// End
