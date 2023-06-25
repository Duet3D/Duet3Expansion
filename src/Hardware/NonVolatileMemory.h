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

#if RP2040
# include <CanSettings.h>
#endif

// This class manages nonvolatile settings that are specific to the board, and the software reset data that is stored by the crash handler.
// On most Duets there is a 512-byte User Page that we use for this.
// The SAMC21 and SAME5x processors already store various data in the user page, however both those processor families support EEPROM emulation so we use 512 bytes of that instead.

enum class NvmPage : uint8_t { common = 0, closedLoop };

class NonVolatileMemory
{
public:
	union HarmonicDataElement
	{
		float f;
		uint32_t u;
	};

	static_assert(sizeof(HarmonicDataElement) == sizeof(float));
	static_assert(sizeof(HarmonicDataElement) == sizeof(uint32_t));

	NonVolatileMemory(NvmPage whichPage) noexcept;

	void *operator new(size_t, void *p) noexcept { return p; }			// for placement new

	void EnsureWritten() noexcept;
	SoftwareResetData *GetLastWrittenResetData(unsigned int &slot) noexcept pre(page == NvmPage::common);
	SoftwareResetData *AllocateResetDataSlot() noexcept pre(page == NvmPage::common);
	int8_t GetThermistorLowCalibration(unsigned int inputNumber) noexcept pre(page == NvmPage::common);
	int8_t GetThermistorHighCalibration(unsigned int inputNumber) noexcept pre(page == NvmPage::common);
	void SetThermistorLowCalibration(unsigned int inputNumber, int8_t val) noexcept pre(page == NvmPage::common);
	void SetThermistorHighCalibration(unsigned int inputNumber, int8_t val) noexcept pre(page == NvmPage::common);

	bool GetClosedLoopCalibrationDataValid() noexcept pre(page == NvmPage::closedLoop);
	const HarmonicDataElement *GetClosedLoopHarmonicValues() noexcept pre(page == NvmPage::closedLoop);
	void GetClosedLoopZeroCountPhaseAndDirection(uint32_t& phase, bool& backwards) noexcept pre(page == NvmPage::closedLoop);

	void SetClosedLoopCalibrationDataNotValid() noexcept pre(page == NvmPage::closedLoop);
	void SetClosedLoopHarmonicValue(size_t index, float value) noexcept pre(page == NvmPage::closedLoop; index < MaxHarmonicDataSlots);
	void SetClosedLoopZeroCountPhaseAndDirection(uint32_t phase, bool backwards) noexcept;

	bool GetClosedLoopQuadratureDirection(bool& backwards) noexcept pre(page == NvmPage::closedLoop);
	void SetClosedLoopQuadratureDirection(bool backwards) noexcept pre(page == NvmPage::closedLoop);

#if RP2040
	bool GetCanSettings(CanUserAreaData& canSettings) noexcept pre(page == NvmPage::common);
	void SetCanSettings(CanUserAreaData& canSettings) noexcept pre(page == NvmPage::common);
#endif

private:
	void EnsureRead() noexcept;
	void SetDirty(bool eraseNeeded) noexcept pre(state != NvmState::notRead);

	int8_t GetThermistorCalibration(unsigned int inputNumber, uint8_t *calibArray) noexcept pre(page == NvmPage::common);
	void SetThermistorCalibration(unsigned int inputNumber, int8_t val, uint8_t *calibArray) noexcept pre(page == NvmPage::common);

	// In the following structs, the magic value must always be the first 2 bytes
	struct CommonPage
	{
		static constexpr unsigned int NumberOfResetDataSlots = 3;
		static constexpr unsigned int MaxCalibratedThermistors = 8;

		uint16_t magic;
		uint8_t thermistorLowCalibration[MaxCalibratedThermistors];
		uint8_t thermistorHighCalibration[MaxCalibratedThermistors];
#if RP2040
		CanUserAreaData canSettings;
		uint8_t spare[38-16];
#else
		uint8_t spare[38];
#endif
		// 56 bytes up to here
		SoftwareResetData resetData[NumberOfResetDataSlots];			// 3 slots of 152 bytes each
	};

	struct ClosedLoopPage
	{
		static constexpr unsigned int MaxHarmonicDataSlots = 40;

		uint16_t closedLoopMagic;
		// define your data here and pad it out to 510 bytes
		// start with 2 bytes of padding if you will be using 32-bit quantities, so as to 4-byte align them
		uint16_t calibrationNotValid : 1,								// will be 1 if no magnetic encoder calibration values are present
				 quadratureDirectionNotValid : 1,						// will be 1 if the direction of the quadrature encoder has not been set
				 unusedAllOnes : 14;
		uint32_t magneticEncoderZeroCountPhase;
		uint32_t magneticEncoderBackwards : 1,
				 quadratureEncoderBackwards : 1,
				 unusedAllOnes2 : 30;
		HarmonicDataElement harmonicData[MaxHarmonicDataSlots];

		uint8_t spare[512 - 4 - 8 - sizeof(harmonicData)];
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
