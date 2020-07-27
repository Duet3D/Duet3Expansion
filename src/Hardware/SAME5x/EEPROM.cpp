/*
 * EEPROM_SAME51.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include <Hardware/EEPROM.h>

#if SAME5x

#include <RepRapFirmware.h>

uint32_t EEPROM::GetSize()
{
	const uint64_t nvmUserRow0 = *reinterpret_cast<const uint64_t*>(NVMCTRL_USER);		// we only need values in the first 64 bits of the user area
	const uint32_t seesblk = (nvmUserRow0 >> 32) & 0x0F;
	const uint32_t seepsz = (nvmUserRow0 >> 36) & 0x07;
	const uint32_t maxPossible[16] = { 0, 4096, 8192, 16384, 16384, 32768, 32768, 32768, 32768, 65536, 65536, 0, 0, 0, 0, 0 };
	return min<uint32_t>(512 << seepsz, maxPossible[seesblk]);
}

bool EEPROM::Read(char *data, uint32_t offset, uint32_t length)
{
	const uint32_t size = GetSize();
	if (offset >= size || length > size - offset)
	{
		return false;
	}

	while (NVMCTRL->SEESTAT.bit.BUSY) { }

	const uint8_t * const SmartEEPROM8 = reinterpret_cast<uint8_t *>(SEEPROM_ADDR);
	memcpy(data, SmartEEPROM8 + offset, length);
	return true;
}

bool EEPROM::Write(const char *data, uint32_t offset, uint32_t length)
{
	const uint32_t size = GetSize();
	if (offset >= size || length > size - offset)
	{
		return false;
	}

	while (NVMCTRL->SEESTAT.bit.BUSY) { }

	uint8_t * const SmartEEPROM8 = reinterpret_cast<uint8_t *>(SEEPROM_ADDR);
	memcpy(SmartEEPROM8 + offset, data, length);
	return true;
}

#endif
