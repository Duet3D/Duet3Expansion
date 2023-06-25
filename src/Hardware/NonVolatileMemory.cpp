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
#elif SAMC21
# include <Flash.h>
constexpr uint32_t RWW_ADDR = FLASH_ADDR + 0x00400000;
#elif RP2040
# include <hardware/flash.h>
// We allocate one sector for each type of non-volatile memory page. We store the page within the sector using wear levelling.
constexpr uint32_t FlashSectorSize = 4096;									// the flash chip has 4K sectors
constexpr uint32_t FlashSize = 2 * 1024 * 1024;								// the flash chip size in bytes (2Mbytes = 16Mbits)
constexpr uint32_t NvmPage0Offset = FlashSize - FlashSectorSize;			// the offset into flash memory where we store page 0 of the non-volatile data
constexpr uint32_t NvmPage0Addr = XIP_BASE + FlashSize - FlashSectorSize;	// the address where we can read page 0 of the non-volatile data
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
#elif RP2040
		//TODO don't just read the first page in the sector, search for the most recent one written
		memcpyu32(reinterpret_cast<uint32_t *>(&buffer), reinterpret_cast<const uint32_t *>(NvmPage0Addr - (FlashSectorSize * (unsigned int)page)), sizeof(buffer)/sizeof(uint32_t));
#else
# error Unsupported processor
#endif
		if (buffer.commonPage.magic != GetMagicValue())
		{
//			debugPrintf("Invalid user area\n");
			memset(&buffer, 0xFF, sizeof(buffer));
			buffer.commonPage.magic = GetMagicValue();
			SetDirty(true);
		}
		else
		{
			state = NvmState::clean;
//			debugPrintf("user area valid\n");
		}
	}
}

void NonVolatileMemory::SetDirty(bool eraseNeeded) noexcept
{
	if (eraseNeeded && state < NvmState::eraseAndWriteNeeded)
	{
		state = NvmState::eraseAndWriteNeeded;
	}
	else if (state < NvmState::writeNeeded)
	{
		state = NvmState::writeNeeded;
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
#elif RP2040
	if (state == NvmState::eraseAndWriteNeeded)
	{
		DisableCore1Processing();
		IrqDisable();
		flash_range_erase(NvmPage0Offset, FlashSectorSize);
		IrqEnable();
		//TODO allocate a new page in the sector, if there is one, else erase the sector
		state = NvmState::writeNeeded;
		EnableCore1Processing();
	}

	if (state == NvmState::writeNeeded)
	{
		DisableCore1Processing();
		IrqDisable();
		flash_range_program(NvmPage0Offset, (uint8_t *)&buffer, 512);
		IrqEnable();
		state = NvmState::clean;
		EnableCore1Processing();
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
			SetDirty(false);
			return &buffer.commonPage.resetData[i];
		}
	}

	// All slots are full, so clear them out and start again
	for (unsigned int i = 0; i < CommonPage::NumberOfResetDataSlots; ++i)
	{
		buffer.commonPage.resetData[i].Clear();
	}
	SetDirty(true);
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
			SetDirty((newVal & ~oldVal) != 0);
		}
	}
}

bool NonVolatileMemory::GetClosedLoopCalibrationDataValid() noexcept
{
	EnsureRead();
	return !buffer.closedLoopPage.calibrationNotValid;
}


void NonVolatileMemory::SetClosedLoopCalibrationDataNotValid()
{
	EnsureRead();
	if (!buffer.closedLoopPage.calibrationNotValid || !buffer.closedLoopPage.quadratureDirectionNotValid)
	{
		// Set the data to all 0xFF so that we will be able to write it without erasing again
		buffer.closedLoopPage.calibrationNotValid = true;
		buffer.closedLoopPage.quadratureDirectionNotValid = true;
		buffer.closedLoopPage.unusedAllOnes = 0x3FFF;
		buffer.closedLoopPage.magneticEncoderZeroCountPhase = 0xFFFFFFFF;
		buffer.closedLoopPage.magneticEncoderBackwards = true;
		buffer.closedLoopPage.quadratureEncoderBackwards = true;
		buffer.closedLoopPage.unusedAllOnes2 = 0x3FFFFFFF;
		for (size_t i = 0; i < ClosedLoopPage::MaxHarmonicDataSlots; ++i)
		{
			buffer.closedLoopPage.harmonicData[i].u = 0xFFFFFFFF;
		}
		SetDirty(true);
	}
}

const NonVolatileMemory::HarmonicDataElement *NonVolatileMemory::GetClosedLoopHarmonicValues() noexcept
{
	EnsureRead();
	return buffer.closedLoopPage.harmonicData;
}

void NonVolatileMemory::SetClosedLoopHarmonicValue(size_t index, float value) noexcept
{
	EnsureRead();
	const float oldValue = buffer.closedLoopPage.harmonicData[index].f;
	if (oldValue != value)
	{
		buffer.closedLoopPage.calibrationNotValid = 0;
		const uint32_t oldBits = buffer.closedLoopPage.harmonicData[index].u;
		buffer.closedLoopPage.harmonicData[index].f = value;

		// If we are only changing 1s to 0s then we don't need to erase
		const uint32_t newBits = buffer.closedLoopPage.harmonicData[index].u;
		SetDirty((~oldBits & newBits) != 0);
	}
}

// Get the zero count phase and direction. Check that the data is flagged as valid before calling this.
void NonVolatileMemory::GetClosedLoopZeroCountPhaseAndDirection(uint32_t& phase, bool& backwards) noexcept
{
	EnsureRead();
	phase = buffer.closedLoopPage.magneticEncoderZeroCountPhase;
	backwards = buffer.closedLoopPage.magneticEncoderBackwards;
}

// Set the zero count phase and direction. Also flags the calibration data as valid, so the harmonic data should be set first. Call EnsureWritten after this to save it to NVM.
void NonVolatileMemory::SetClosedLoopZeroCountPhaseAndDirection(uint32_t phase, bool backwards) noexcept
{
	EnsureRead();
	const uint32_t oldPhase = buffer.closedLoopPage.magneticEncoderZeroCountPhase;
	if (oldPhase != phase)
	{
		buffer.closedLoopPage.magneticEncoderZeroCountPhase = phase;
		SetDirty((phase & ~oldPhase) != 0);
	}
	if (backwards != buffer.closedLoopPage.magneticEncoderBackwards)
	{
		buffer.closedLoopPage.magneticEncoderBackwards = backwards;
		SetDirty(backwards);
	}
	if (buffer.closedLoopPage.calibrationNotValid)
	{
		buffer.closedLoopPage.calibrationNotValid = false;
		SetDirty(false);
	}
}

// Get the closed loop quadrature encoder direction, if available. If successful, return true with 'backwards' set; else return false.
bool NonVolatileMemory::GetClosedLoopQuadratureDirection(bool& backwards) noexcept
{
	EnsureRead();
	if (buffer.closedLoopPage.quadratureDirectionNotValid)
	{
		return false;
	}
	backwards = buffer.closedLoopPage.quadratureEncoderBackwards;
	return true;
}

// Set the closed loop quadrature encoder direction and mark it as valid. Call EnsureWritten after this to save it to NVM.
void NonVolatileMemory::SetClosedLoopQuadratureDirection(bool backwards) noexcept
{
	EnsureRead();
	if (backwards != buffer.closedLoopPage.quadratureEncoderBackwards)
	{
		buffer.closedLoopPage.quadratureEncoderBackwards = backwards;
		SetDirty(backwards);
	}
	if (buffer.closedLoopPage.quadratureDirectionNotValid)
	{
		buffer.closedLoopPage.quadratureDirectionNotValid = false;
		SetDirty(false);
	}
}

#if RP2040
	bool NonVolatileMemory::GetCanSettings(CanUserAreaData& canSettings) noexcept
	{
		EnsureRead();
		canSettings = buffer.commonPage.canSettings;
		// TODO: Perform some sort of validation?
		return true;
	}

	void NonVolatileMemory::SetCanSettings(CanUserAreaData& canSettings) noexcept
	{
		EnsureRead();
		buffer.commonPage.canSettings = canSettings;
		SetDirty(true);
	}
#endif

	// End
