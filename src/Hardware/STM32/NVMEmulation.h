// NVRAM emulation for STM32H5 and STM32H7 processors

#ifndef NVMEMULATION_H_
#define NVMEMULATION_H_

#include "Core.h"

#if STM32H5
# define FLASH_DATA_LENGTH (8*1024)		 	// size of the Software Reset Data in Flash. This is the erase sector size.
#elif STM32H7
# define FLASH_DATA_LENGTH (128*1024)		 // size of the Software Reset Data in Flash. This is the erase sector size.
#endif

void NVMEmulationRead(void *data, uint32_t dataLength) noexcept;
bool NVMEmulationErase() noexcept;
bool NVMEmulationWrite(const void *data, uint32_t dataLength) noexcept;

#endif
