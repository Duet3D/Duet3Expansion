/*
 * Flash.h
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_FLASH_H_
#define SRC_HARDWARE_FLASH_H_

#include "RepRapFirmware.h"

namespace Flash
{
#if SAMC21
	constexpr size_t FlashPageSize = 64;					// minimum size we can write
	constexpr size_t FlashRowSize = FlashPageSize * 4;		// minimum size we can erase
#endif
#if SAME5x
	constexpr size_t FlashPageSize = 512;					// minimum size we can write
	constexpr size_t FlashBlockSize = FlashPageSize * 16;	// minimum size we can erase, except in the AUX area
#endif

	bool Init();
	bool Unlock(uint32_t start, uint32_t length);
	bool Erase(uint32_t start, uint32_t length);
	bool Lock(uint32_t start, uint32_t length);
	bool Write(uint32_t start, uint32_t length, uint8_t *data);
}

#endif /* SRC_HARDWARE_FLASH_H_ */
