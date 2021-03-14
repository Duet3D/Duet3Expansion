/*
 * SharedI2CClient.h
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDI2CCLIENT_H_
#define SRC_HARDWARE_SHAREDI2CCLIENT_H_

#include <RepRapFirmware.h>

#if SUPPORT_I2C_SENSORS

#include "SharedI2CMaster.h"

class SharedI2CClient
{
public:
	SharedI2CClient(SharedI2CMaster& dev, uint16_t addr) noexcept;
	bool Transfer(uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead, uint32_t timeout) noexcept;

private:
	SharedI2CMaster& device;
	uint16_t address;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CCLIENT_H_ */
