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

// Transfer some data returning true if successful. Release the bus if either releaseBus is true or we return false because the transfer failed.
bool SharedI2CClient::Transfer(const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead, uint32_t timeout, bool releaseBus) noexcept
{
	if (!device.Take(timeout))
	{
		return false;
	}
	const bool ret = device.Transfer(address, txBuffer, rxBuffer, numToWrite, numToRead);
	if (releaseBus || !ret)
	{
		device.Release();
	}
	return ret;
}

#endif

// End
