/*
 * EEPROM_SAMC21.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include <Hardware/EEPROM.h>

#if SAMC21

#include <RepRapFirmware.h>
#include <hri_nvmctrl_c21.h>

constexpr uint32_t RWWEEEaddess = 0x00400000;
constexpr uint32_t FlashPagesPerRow = NVMCTRL_ROW_PAGES;								// 4 pages per row on the SAMC21
constexpr uint32_t FlashRowSize = FlashPagesPerRow * NVMCTRL_PAGE_SIZE;

static void flash_program(const uint32_t dst_addr, const uint8_t *buffer, const uint16_t size, uint32_t nvmctrl_cmd)
{
	uint32_t nvm_address = dst_addr / 2;
	uint16_t i, data;

	while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL))
	{
		/* Wait until this module isn't busy */
	}

	hri_nvmctrl_write_CTRLA_reg(NVMCTRL, NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY);

	while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL))
	{
		/* Wait until this module isn't busy */
	}

	/* Clear flags */
	hri_nvmctrl_clear_STATUS_reg(NVMCTRL, NVMCTRL_STATUS_MASK);

	for (i = 0; i < size; i += 2)
	{
		data = buffer[i];
		if (i < NVMCTRL_PAGE_SIZE - 1)
		{
			data |= (buffer[i + 1] << 8);
		}
		((volatile uint16_t *)FLASH_ADDR)[nvm_address++] = data;
	}

	while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL)) {
		/* Wait until this module isn't busy */
	}

	hri_nvmctrl_write_ADDR_reg(NVMCTRL, dst_addr / 2);
	hri_nvmctrl_write_CTRLA_reg(NVMCTRL, nvmctrl_cmd | NVMCTRL_CTRLA_CMDEX_KEY);
}

uint32_t EEPROM::GetSize()
{
	const uint32_t nvmUserRow0 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER);		// we only need values in the first 32 bits of the user area
	const uint32_t eeprom = (nvmUserRow0 >> 4) & 0x07;									// get BOOTPROT
	return (eeprom == 7) ? 0 : 256 << (6 - eeprom);
}

bool EEPROM::Read(char *data, uint32_t offset, uint32_t length)
{
	const uint32_t size = GetSize();
	if (offset >= size || length > size - offset)
	{
		return false;
	}

	memcpy(data, reinterpret_cast<const char *>(RWWEEEaddess + offset), length);
	return true;
}

bool EEPROM::Write(const char *data, uint32_t offset, uint32_t length)
{
	const uint32_t size = GetSize();
	if (offset >= size || length > size - offset)
	{
		return false;
	}

	while (length != 0)
	{
		const uint32_t rowStartOffset = offset & ~(FlashRowSize - 1);					// offset of the start or the row that we may need to erase
		uint32_t thisOffset = offset & (FlashRowSize - 1);						// offset within the row that we start at
		uint32_t thisLength = min<uint32_t>(FlashRowSize - thisOffset, length);
		length -= thisLength;
		offset += thisLength;
		uint32_t pageBuffer[FlashRowSize/4];											// buffer to hold the data we may need to erase
		memcpy(static_cast<void*>(pageBuffer), reinterpret_cast<const void *>(RWWEEEaddess + rowStartOffset), FlashRowSize);
		char *bufp = reinterpret_cast<char *>(pageBuffer) + thisOffset;
		bool eraseNeeded = false;														// true if we are changing any bits from 0 to 1
		uint32_t writesNeeded = 0;														// bitmap of pages in the row that need to be written
		while (thisLength != 0)
		{
			if ((~bufp[thisOffset] & *data & 0xFF) != 0)
			{
				eraseNeeded = true;
			}
			else if (*bufp != *data)
			{
				writesNeeded |= 1 << (thisOffset/FLASH_PAGE_SIZE);
			}
			bufp[thisOffset] = *data;
			++thisOffset;
			++data;
			--thisLength;
		}

		if (eraseNeeded)
		{
			while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL))
			{
				/* Wait until this module isn't busy */
			}

			/* Clear flags */
			hri_nvmctrl_clear_STATUS_reg(NVMCTRL, NVMCTRL_STATUS_MASK);

			/* Set address and command */
			hri_nvmctrl_write_ADDR_reg(NVMCTRL, rowStartOffset / 2);
			hri_nvmctrl_write_CTRLA_reg(NVMCTRL, NVMCTRL_CTRLA_CMD_RWWEEER | NVMCTRL_CTRLA_CMDEX_KEY);
		}

		for (uint32_t i = 0; i < NVMCTRL_ROW_PAGES; i++)
		{
			if (eraseNeeded || (writesNeeded & (1u << i)) != 0)
			{
				flash_program(rowStartOffset + i * NVMCTRL_PAGE_SIZE,
								reinterpret_cast<const uint8_t*>(pageBuffer) + (i * NVMCTRL_PAGE_SIZE),
								NVMCTRL_PAGE_SIZE,
								NVMCTRL_CTRLA_CMD_RWWEEWP);
			}
		}
	}

	return true;
}

#endif
