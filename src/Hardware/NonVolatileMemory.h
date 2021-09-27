/*
 * NonVolatileMemory.h
 *
 * Class to manage an area of flash memory that we use to store data that persists over a reset
 *  Created on: 24 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_NONVOLATILEMEMORY_H_
#define SRC_HARDWARE_NONVOLATILEMEMORY_H_

#include <Hardware/SoftwareReset.h>

// This class manages nonvolatile settings that are specific to the board, and the software reset data that is stored by the crash handler.
// On most Duets there is a 512-byte User Page that we use for this.
// The SAMC21 and SAME5x processors already store various data in the user page, however both those processor families support EEPROM emulation so we use 512 bytes of that instead.

enum class NvmPage : uint8_t { common, closedLoop };

class NonVolatileMemory
{
public:
	NonVolatileMemory(NvmPage whichPage) noexcept;

	void *operator new(size_t, void *p) noexcept { return p; }			// for placement new

	void EnsureWritten() noexcept;
	SoftwareResetData *GetLastWrittenResetData(unsigned int &slot) noexcept pre(page == NvmPage::common);
	SoftwareResetData *AllocateResetDataSlot() noexcept pre(page == NvmPage::common);
	int8_t GetThermistorLowCalibration(unsigned int inputNumber) noexcept pre(page == NvmPage::common);
	int8_t GetThermistorHighCalibration(unsigned int inputNumber) noexcept pre(page == NvmPage::common);
	void SetThermistorLowCalibration(unsigned int inputNumber, int8_t val) noexcept pre(page == NvmPage::common);
	void SetThermistorHighCalibration(unsigned int inputNumber, int8_t val) noexcept pre(page == NvmPage::common);
	bool GetClosedLoopDataWritten() noexcept pre(page == NvmPage::closedLoop);
	float* GetClosedLoopLUTHarmonicAngles() noexcept pre(page == NvmPage::closedLoop);
	float* GetClosedLoopLUTHarmonicMagnitudes() noexcept pre(page == NvmPage::closedLoop);
	void SetClosedLoopLUTHarmonicAngle(size_t harmonic, float value) noexcept pre(page == NvmPage::closedLoop);
	void SetClosedLoopLUTHarmonicMagnitude(size_t harmonic, float value) noexcept pre(page == NvmPage::closedLoop);

private:
	void EnsureRead() noexcept;
	int8_t GetThermistorCalibration(unsigned int inputNumber, uint8_t *calibArray) noexcept pre(page == NvmPage::common);
	void SetThermistorCalibration(unsigned int inputNumber, int8_t val, uint8_t *calibArray) noexcept pre(page == NvmPage::common);
	void SetClosedLoopLUTHarmonicValue(float* harmonicArray, size_t harmonic, float value) noexcept pre(page == NvmPage::closedLoop);

	// In the following structs, the magic value must always be the first 2 bytes
	struct CommonPage
	{
		static constexpr unsigned int NumberOfResetDataSlots = 3;
		static constexpr unsigned int MaxCalibratedThermistors = 8;

		uint16_t magic;
		uint8_t thermistorLowCalibration[MaxCalibratedThermistors];
		uint8_t thermistorHighCalibration[MaxCalibratedThermistors];
		uint8_t spare[38];
		// 56 bytes up to here
		SoftwareResetData resetData[NumberOfResetDataSlots];			// 3 slots of 152 bytes each
	};

	struct ClosedLoopPage
	{
		static constexpr unsigned int MaxHarmonics = 17;

		uint16_t magic;
		// define your data here and pad it out to 510 bytes
		// start with 2 bytes of padding if you will be using 32-bit quantities, so as to 4-byte align them
		uint16_t neverWritten : 1,		// Will be 1 if no calibration values are present (will be initialised to 1)
				 zero : 15;
		float absEncoderLUTHarmonicAngles[MaxHarmonics];
		float absEncoderLUTHarmonicMagnitudes[MaxHarmonics];

		uint8_t spare[508 - 8*MaxHarmonics];
	};

	union NVM
	{
		CommonPage commonPage;
		ClosedLoopPage closedLoopPage;
	};

	static_assert(sizeof(NVM) == 512);

	uint32_t GetMagicValue() const noexcept { return 0x41E5 + (unsigned int)page + ((unsigned int)page << 8); }

	enum class NvmState : uint8_t { notRead, clean, writeNeeded, eraseAndWriteNeeded };

	alignas(4) NVM buffer;
	NvmState state;
	NvmPage page;
};

#endif /* SRC_HARDWARE_NONVOLATILEMEMORY_H_ */
