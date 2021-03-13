/*
 * SharedI2CMaster.h
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDI2CMASTER_H_
#define SRC_HARDWARE_SHAREDI2CMASTER_H_

#include <RepRapFirmware.h>

#if SUPPORT_I2C_SENSORS

class SharedI2CMaster
{
public:
	SharedI2CMaster(uint8_t sercomNum);
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
