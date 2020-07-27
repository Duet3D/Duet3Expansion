/*
 * EEPROM.h
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_EEPROM_H_
#define SRC_HARDWARE_EEPROM_H_

#include <RepRapFirmware.h>

namespace EEPROM
{
	uint32_t GetSize();
	bool Read(char *data, uint32_t offset, uint32_t length);
	bool Write(const char *data, uint32_t offset, uint32_t length);
}

#endif /* SRC_HARDWARE_EEPROM_H_ */
