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

LIS3DH::LIS3DH(SharedI2CMaster& dev, bool addressLSB) noexcept : SharedI2CClient(dev, (addressLSB) ? Lis3dAddress | 0x01 : Lis3dAddress)
{
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
	return Transfer(regNum, &val, 2, 0, Lis3dI2CTimeout);
}

#endif

// End
