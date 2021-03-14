/*
 * SharedI2CClient.cpp
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#include <Hardware/SharedI2CClient.h>

#if SUPPORT_I2C_SENSORS

SharedI2CClient::SharedI2CClient(SharedI2CMaster& dev, uint16_t addr) noexcept : device(dev), address(addr)
{
}

size_t SharedI2CClient::Transfer(uint8_t *buffer, size_t numToWrite, size_t numToRead, uint32_t timeout) noexcept
{
	device.Take(timeout);
	const size_t ret = device.Transfer(address, buffer, numToWrite, numToRead);
	device.Release();
	return ret;
}

#endif

// End
