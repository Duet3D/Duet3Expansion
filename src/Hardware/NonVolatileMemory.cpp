/*
 * NonVolatileMemory.cpp
 *
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#include "NonVolatileMemory.h"

#if SAM4E || SAM4S || SAME70
# include <Cache.h>
# include <flash_efc.h>
#endif
#if SAMC21
# include <Flash.h>
constexpr uint32_t RWW_ADDR = FLASH_ADDR + 0x00400000;
#endif

NonVolatileMemory::NonVolatileMemory(NvmPage whichPage) noexcept : state(NvmState::notRead), page(whichPage)
{
}

void NonVolatileMemory::EnsureRead() noexcept
{
	if (state == NvmState::notRead)
	{
#if SAME5x
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(SEEPROM_ADDR + (512 * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#elif SAMC21
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(RWW_ADDR + (512 * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#else
# error Unsupported processor
#endif
		if (buffer.commonPage.magic != GetMagicValue())
		{
//			debugPrintf("Invalid user area\n");
			memset(&buffer, 0xFF, sizeof(buffer));
			buffer.commonPage.magic =GetMagicValue();
			state = NvmState::eraseAndWriteNeeded;
		}
		else
		{
			state = NvmState::clean;
//			debugPrintf("user area valid\n");
		}
	}
}

void NonVolatileMemory::EnsureWritten() noexcept
{
#if SAME5x
	if (state >= NvmState::writeNeeded)
	{
		// No need to erase on the SAME5x because the EEPROM emulation manages it
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
        memcpyu32(reinterpret_cast<uint32_t*>(SEEPROM_ADDR + (512 * (unsigned int)page)), reinterpret_cast<const uint32_t*>(&buffer), sizeof(buffer)/sizeof(uint32_t));
		state = NvmState::clean;
        while (NVMCTRL->SEESTAT.bit.BUSY) { }
	}
#elif SAMC21
	if (state == NvmState::eraseAndWriteNeeded)
	{
		Flash::RwwErase(RWW_ADDR + (512 * (unsigned int)page), 512);
		state = NvmState::writeNeeded;
	}

	if (state == NvmState::writeNeeded)
	{
		Flash::RwwWrite(RWW_ADDR + (512 * (unsigned int)page), 512, (uint8_t*)&buffer);
		state = NvmState::clean;
	}
#else
# error Unsupported processor
#endif
}

SoftwareResetData* NonVolatileMemory::GetLastWrittenResetData(unsigned int &slot) noexcept
{
	EnsureRead();
	for (unsigned int i = CommonPage::NumberOfResetDataSlots; i != 0; )
	{
		--i;
		if (buffer.commonPage.resetData[i].IsValid())
		{
			slot = i;
			return &buffer.commonPage.resetData[i];
		}
	}
	return nullptr;
}

SoftwareResetData* NonVolatileMemory::AllocateResetDataSlot() noexcept
{
	EnsureRead();
	for (unsigned int i = 0; i < CommonPage::NumberOfResetDataSlots; ++i)
	{
		if (buffer.commonPage.resetData[i].IsVacant())
		{
			if (state == NvmState::clean)			// need this test because state may already be EraseAndWriteNeeded after EnsureRead
			{
				state = NvmState::writeNeeded;		// assume the caller will write to the allocated slot
			}
			return &buffer.commonPage.resetData[i];
		}
	}

	// All slots are full, so clear them out and start again
	for (unsigned int i = 0; i < CommonPage::NumberOfResetDataSlots; ++i)
	{
		buffer.commonPage.resetData[i].Clear();
	}
	state = NvmState::eraseAndWriteNeeded;
	return &buffer.commonPage.resetData[0];
}

int8_t NonVolatileMemory::GetThermistorLowCalibration(unsigned int inputNumber) noexcept
{
	return GetThermistorCalibration(inputNumber, buffer.commonPage.thermistorLowCalibration);
}

int8_t NonVolatileMemory::GetThermistorHighCalibration(unsigned int inputNumber) noexcept
{
	return GetThermistorCalibration(inputNumber, buffer.commonPage.thermistorHighCalibration);
}

void NonVolatileMemory::SetThermistorLowCalibration(unsigned int inputNumber, int8_t val) noexcept
{
	SetThermistorCalibration(inputNumber, val, buffer.commonPage.thermistorLowCalibration);
}

void NonVolatileMemory::SetThermistorHighCalibration(unsigned int inputNumber, int8_t val) noexcept
{
	SetThermistorCalibration(inputNumber, val, buffer.commonPage.thermistorHighCalibration);
}

int8_t NonVolatileMemory::GetThermistorCalibration(unsigned int inputNumber, uint8_t *calibArray) noexcept
{
	EnsureRead();
	return (inputNumber >= CommonPage::MaxCalibratedThermistors || calibArray[inputNumber] == 0xFF) ? 0 : calibArray[inputNumber] - 0x7F;
}

void NonVolatileMemory::SetThermistorCalibration(unsigned int inputNumber, int8_t val, uint8_t *calibArray) noexcept
{
	if (inputNumber < CommonPage::MaxCalibratedThermistors)
	{
		EnsureRead();
		const uint8_t oldVal = calibArray[inputNumber];
		const uint8_t newVal = val + 0x7F;
		if (oldVal != newVal)
		{
			// If we are only changing 1 bits to 0 then we don't need to erase
			calibArray[inputNumber] = newVal;
			if ((newVal & ~oldVal) != 0)
			{
				state = NvmState::eraseAndWriteNeeded;
			}
			else if (state == NvmState::clean)
			{
				state = NvmState::writeNeeded;
			}
		}
	}
}

bool NonVolatileMemory::GetClosedLoopDataWritten() noexcept
{
	EnsureRead();
	return !buffer.closedLoopPage.neverWritten;
}

float* NonVolatileMemory::GetClosedLoopLUTHarmonicAngles() noexcept
{
	EnsureRead();
	return buffer.closedLoopPage.absEncoderLUTHarmonicAngles;
}

float* NonVolatileMemory::GetClosedLoopLUTHarmonicMagnitudes() noexcept
{
	EnsureRead();
	return buffer.closedLoopPage.absEncoderLUTHarmonicMagnitudes;
}

void NonVolatileMemory::SetClosedLoopLUTHarmonicValue(float* harmonicArray, size_t harmonic, float value) noexcept
{
	EnsureRead();
	const float oldValue = harmonicArray[harmonic];
	const bool hasBeenWrittenBefore = !buffer.closedLoopPage.neverWritten;
	if (oldValue != value)
	{
		buffer.closedLoopPage.neverWritten = 0;
		harmonicArray[harmonic] = value;

		// If we are only changing 1s to 0s then we don't need to erase
		if (hasBeenWrittenBefore)
		{
			state = NvmState::eraseAndWriteNeeded;
		}
		else
		{
			state = NvmState::writeNeeded;
		}
	}
}

void NonVolatileMemory::SetClosedLoopLUTHarmonicAngle(size_t harmonic, float value) noexcept
{
	SetClosedLoopLUTHarmonicValue(buffer.closedLoopPage.absEncoderLUTHarmonicAngles, harmonic, value);
}

void NonVolatileMemory::SetClosedLoopLUTHarmonicMagnitude(size_t harmonic, float value) noexcept
{
	SetClosedLoopLUTHarmonicValue(buffer.closedLoopPage.absEncoderLUTHarmonicMagnitudes, harmonic, value);
}

// End
