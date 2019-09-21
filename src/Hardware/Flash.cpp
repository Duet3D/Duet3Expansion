/*
 * Flash.cpp
 *
 *  Created on: 8 Aug 2019
 *      Author: David
 */

#include "Flash.h"

#include <hal/include/hal_flash.h>

namespace Flash
{
	static flash_descriptor flash;
}

bool Flash::Init()
{
	return flash_init(&flash, NVMCTRL) == 0;
}

bool Flash::Unlock(uint32_t start, uint32_t length)
{
	// The flash_unlock command only works if the number of pages passed is exactly 1 lock region. So we need to loop calling it.
	const uint32_t pageSize = flash_get_page_size(&flash);
#ifdef SAMC21
	constexpr uint32_t NVMCTRL_REGIONS_NUM = 16;						// from hpl_nvmctrl.c
#else
	constexpr uint32_t NVMCTRL_REGIONS_NUM = 32;						// from hpl_nvmctrl.c
#endif
	const uint32_t pagesPerRegion = FLASH_SIZE / (NVMCTRL_REGIONS_NUM * pageSize);
	for (uint32_t lengthDone = 0; lengthDone < length; )
	{
		if (flash_unlock(&flash, start, pagesPerRegion) != (int32_t)pagesPerRegion)
		{
			return false;
		}
		start += pagesPerRegion * pageSize;
		lengthDone += pagesPerRegion * pageSize;
	}
	return true;
}

bool Flash::Erase(uint32_t start, uint32_t length)
{
	const uint32_t pageSize = flash_get_page_size(&flash);
	__disable_irq();
	const bool ok = flash_erase(&flash, start, length/pageSize) == 0;
	__enable_irq();
	return ok;
}

bool Flash::Lock(uint32_t start, uint32_t length)
{
	// The flash_lock command only works if the number of pages passed is exactly 1 lock region. So we need to loop calling it.
	const uint32_t pageSize = flash_get_page_size(&flash);
#ifdef SAMC21
	constexpr uint32_t NVMCTRL_REGIONS_NUM = 16;						// from hpl_nvmctrl.c
#else
	constexpr uint32_t NVMCTRL_REGIONS_NUM = 32;						// from hpl_nvmctrl.c
#endif
	const uint32_t pagesPerRegion = FLASH_SIZE / (NVMCTRL_REGIONS_NUM * pageSize);
	for (uint32_t lengthDone = 0; lengthDone < length; )
	{
		if (flash_lock(&flash, start, pagesPerRegion) != (int32_t)pagesPerRegion)
		{
			return false;
		}
		start += pagesPerRegion * pageSize;
		lengthDone += pagesPerRegion * pageSize;
	}
	return true;
}

bool Flash::Write(uint32_t start, uint32_t length, uint8_t *data)
{
	__disable_irq();
	const bool ok = flash_write(&flash, start, data, length) == 0;
	__enable_irq();
	return ok;
}

// End
