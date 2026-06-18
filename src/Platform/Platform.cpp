/*
 * Platform.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "Platform.h"

#include <Hardware/IoPorts.h>
#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Movement/Move.h>
#include <Movement/StepperDrivers/SmartDrivers.h>
#include "Movement/StepTimer.h"
#include <CAN/CanInterface.h>
#include <CanMessageBuffer.h>
#include "Tasks.h"
#include "Heating/Heat.h"
#include "Heating/Sensors/TemperatureSensor.h"
#include "Fans/FansManager.h"
#include <CanMessageFormats.h>
#include <CanMessageGenericTables.h>
#include <CanMessageGenericParser.h>
#include <Hardware/Devices.h>
#include <Math/Isqrt.h>
#include <Version.h>

#if NUM_ASYNC_PORTS != 0
# include <AsyncSerial.h>
#endif

#if NUM_I2C_CHANNELS != 0
# include <I2C/SharedI2CMaster.h>
#endif

#if SUPPORT_LIS3DH
# include <CommandProcessing/AccelerometerHandler.h>
#endif

#if SUPPORT_LDC1612
# include <CommandProcessing/ScanningSensorHandler.h>
#endif

#if SUPPORT_AS5601
# include <CommandProcessing/MFMHandler.h>
#endif

#if SUPPORT_CLOSED_LOOP
# include <ClosedLoop/ClosedLoop.h>
#endif

#if SUPPORT_INDUCTIVE_HEATER
# include "InductiveHeaterPort.h"
#endif

#if SUPPORT_LP5817
# include "LedStatusControl.h"
#endif

#ifdef ATEIO
# include <Hardware/ATEIO/ExtendedAnalog.h>
#endif

#if !RP2040
# include <hpl_user_area.h>
#endif

#if RP2040
# include <hardware/structs/watchdog.h>
#endif

#if SAME5x

# include <hri_nvmctrl_e54.h>
constexpr uint32_t FlashBlockSize = 0x00010000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 64K for the bootloader

#elif SAMC21

# include <hri_nvmctrl_c21.h>
constexpr uint32_t FlashBlockSize = 0x00004000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 16K for the bootloader

#elif RP2040
// TODO
#else
# error Unsupported processor
#endif

enum class DeferredCommand : uint8_t
{
	none = 0,
	firmwareUpdate,
	bootloaderUpdate,
	reset,
	testWatchdog,
	testDivideByZero,
	testUnalignedMemoryAccess,
	testBadMemoryAccess,
	testMemoryLeak,
	testSpinLockup
};

static volatile DeferredCommand deferredCommand = DeferredCommand::none;
static volatile uint32_t whenDeferredCommandRequested;
static bool deliberateError = false;

namespace Platform
{
	static uint32_t errorCodeBits = 0;
	static DebugFlags debugMaps[Module::numModules];

	UniqueIdBase uniqueId;
	bool isPrinting = false;
	uint32_t realTime = 0;

#if defined(EXP3HC) || defined(TOOL1LC) || defined(EXP1HCL)
	static uint8_t boardVariant = 0;
#endif

#if NUM_SHARED_SPI != 0
	// Currently we support just one shared SPI device. This can be expanded if/when we need to.
	SharedSpiDevice *sharedSpi = nullptr;
#endif

#if NUM_I2C_CHANNELS != 0
	SharedI2CMaster *sharedI2C[NUM_I2C_CHANNELS] = { 0 };
#endif

#if SUPPORT_ADS131M02
	ADS131M02 *loadCellAdc = nullptr;
#endif

#if SUPPORT_LP5817
	LedStatusControl *ledStatusControl = nullptr;
#endif

#if HAS_VOLTAGE_MONITOR
	static volatile uint16_t currentVin, highestVin, lowestVin;
//	static uint16_t lastUnderVoltageValue, lastOverVoltageValue;
	static uint32_t numUnderVoltageEvents, previousUnderVoltageEvents;
	static volatile uint32_t numOverVoltageEvents, previousOverVoltageEvents;
#endif

#if HAS_12V_MONITOR
	static volatile uint16_t currentV12, highestV12, lowestV12;
#endif

	static MinCurMax mcuTemperature;
	static float mcuTemperatureAdjust = 0.0;

	static uint32_t lastPollTime;
	static uint32_t lastFanCheckTime = 0;
	static uint32_t heatTaskIdleTicks = 0;
	static uint32_t syncedIdleTicks = 0;

	static uint32_t whenLastCanMessageProcessed = 0;

#if SUPPORT_THERMISTORS
	static ThermistorAveragingFilter thermistorFilters[NumThermistorFilters];
#endif

#if HAS_VOLTAGE_MONITOR
	static AveragingFilter<VinReadingsAveraged> vinFilter;
#endif
#if HAS_12V_MONITOR
	static AveragingFilter<VinReadingsAveraged> v12Filter;
#endif

#if SUPPORT_INDUCTIVE_HEATER
	static InductiveHeaterPort inductiveHeaterPort;
#endif

#if NUM_CURRENT_SENSORS != 0
	static int32_t currentSensorReadings[NUM_CURRENT_SENSORS] = { 0 };
	static int32_t currentSensorCallbackThresholds[NUM_CURRENT_SENSORS] = { 0 };
	static CallbackParameter currentSensorCallbackParameters[NUM_CURRENT_SENSORS];
	static StandardCallbackFunction *null currentSensorCallbackFunctions[NUM_CURRENT_SENSORS] = { 0 };

	void CurrentSensorAinCallback(CallbackParameter cp, int32_t val) noexcept;
#endif

#if NUM_ASYNC_PORTS != 0
	AsyncSerial *_ecv_null asyncPorts[NUM_ASYNC_PORTS] = { 0 };
#endif

#if SAME5x
	static AveragingFilter<McuTempReadingsAveraged> tpFilter;
	static AveragingFilter<McuTempReadingsAveraged> tcFilter;
#elif SAMC21 || RP2040
	static AveragingFilter<McuTempReadingsAveraged> tsensFilter;
#endif

#if HAS_VOLTAGE_MONITOR

	inline float AdcReadingToVinVoltage(uint16_t adcVal) noexcept
	{
# ifdef EXP3HC
		return adcVal * ((boardVariant >= 1)
							? VinMonitorVoltageRange102AndLater/(1u << AnalogIn::AdcBits)
								: VinMonitorVoltageRangePre102/(1u << AnalogIn::AdcBits)
						);
# else
		return adcVal * (VinMonitorVoltageRange/(1u << AnalogIn::AdcBits));
# endif
	}

#endif

#if HAS_12V_MONITOR

	inline constexpr float AdcReadingToV12Voltage(uint16_t adcVal) noexcept
	{
		return adcVal * (V12MonitorVoltageRange/(1u << AnalogIn::AdcBits));
	}

	inline constexpr uint16_t V12VoltageToAdcReading(float voltage) noexcept
	{
		return (uint16_t)(voltage * ((1u << AnalogIn::AdcBits)/V12MonitorVoltageRange));
	}

#endif

#if SAME5x
	static int32_t tempCalF1, tempCalF2, tempCalF3, tempCalF4;		// temperature calibration factors

	static void ADC_temperature_init()
	{
		// Temperature sense stuff
		constexpr uint32_t NVM_TEMP_CAL_TLI_POS = 0;
		constexpr uint32_t NVM_TEMP_CAL_TLI_SIZE = 8;
		constexpr uint32_t NVM_TEMP_CAL_TLD_POS = 8;
		constexpr uint32_t NVM_TEMP_CAL_TLD_SIZE = 4;
		constexpr uint32_t NVM_TEMP_CAL_THI_POS = 12;
		constexpr uint32_t NVM_TEMP_CAL_THI_SIZE = 8;
		constexpr uint32_t NVM_TEMP_CAL_THD_POS = 20;
		constexpr uint32_t NVM_TEMP_CAL_THD_SIZE = 4;
		constexpr uint32_t NVM_TEMP_CAL_VPL_POS = 40;
		constexpr uint32_t NVM_TEMP_CAL_VPL_SIZE = 12;
		constexpr uint32_t NVM_TEMP_CAL_VPH_POS = 52;
		constexpr uint32_t NVM_TEMP_CAL_VPH_SIZE = 12;
		constexpr uint32_t NVM_TEMP_CAL_VCL_POS = 64;
		constexpr uint32_t NVM_TEMP_CAL_VCL_SIZE = 12;
		constexpr uint32_t NVM_TEMP_CAL_VCH_POS = 76;
		constexpr uint32_t NVM_TEMP_CAL_VCH_SIZE = 12;

		const uint16_t temp_cal_vpl = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VPL_POS / 32)) >> (NVM_TEMP_CAL_VPL_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VPL_SIZE) - 1);
		const uint16_t temp_cal_vph = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VPH_POS / 32)) >> (NVM_TEMP_CAL_VPH_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VPH_SIZE) - 1);
		const uint16_t temp_cal_vcl = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VCL_POS / 32)) >> (NVM_TEMP_CAL_VCL_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VCL_SIZE) - 1);
		const uint16_t temp_cal_vch = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_VCH_POS / 32)) >> (NVM_TEMP_CAL_VCH_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VCH_SIZE) - 1);

		const uint8_t temp_cal_tli = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_TLI_POS / 32)) >> (NVM_TEMP_CAL_TLI_POS % 32))
		               & ((1u << NVM_TEMP_CAL_TLI_SIZE) - 1);
		const uint8_t temp_cal_tld = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_TLD_POS / 32)) >> (NVM_TEMP_CAL_TLD_POS % 32))
		               & ((1u << NVM_TEMP_CAL_TLD_SIZE) - 1);
		const uint16_t temp_cal_tl = ((uint16_t)temp_cal_tli) << 4 | ((uint16_t)temp_cal_tld);

		const uint8_t temp_cal_thi = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_THI_POS / 32)) >> (NVM_TEMP_CAL_THI_POS % 32))
		               & ((1u << NVM_TEMP_CAL_THI_SIZE) - 1);
		const uint8_t temp_cal_thd = (*((uint32_t *)(NVMCTRL_TEMP_LOG) + (NVM_TEMP_CAL_THD_POS / 32)) >> (NVM_TEMP_CAL_THD_POS % 32))
		               & ((1u << NVM_TEMP_CAL_THD_SIZE) - 1);
		const uint16_t temp_cal_th = ((uint16_t)temp_cal_thi) << 4 | ((uint16_t)temp_cal_thd);

		tempCalF1 = (int32_t)temp_cal_tl * (int32_t)temp_cal_vph - (int32_t)temp_cal_th * (int32_t)temp_cal_vpl;
		tempCalF2 = (int32_t)temp_cal_tl * (int32_t)temp_cal_vch - (int32_t)temp_cal_th * (int32_t)temp_cal_vcl;
		tempCalF3 = (int32_t)temp_cal_vcl - (int32_t)temp_cal_vch;
		tempCalF4 = (int32_t)temp_cal_vpl - (int32_t)temp_cal_vph;
	}
#endif

	static void InitialiseInterrupts()
	{
		// Note, I2C interrupt priority is set up in the I2C driver

#if SAME5x || SAMC21
		if constexpr(CANInstanceNumber == 1)
		{
# if defined(ID_CAN1)
			NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
# endif
		}
		else
		{
			NVIC_SetPriority(CAN0_IRQn, NvicPriorityCan);
		}
#endif

#if SAME5x
		NVIC_SetPriority(StepTcIRQn, NvicPriorityStep);
		SetInterruptPriority(DMAC_0_IRQn, 5, NvicPriorityDmac);
		SetInterruptPriority(EIC_0_IRQn, 16, NvicPriorityPins);
#elif SAMC21
		NVIC_SetPriority(StepTcIRQn, NvicPriorityStep);
		NVIC_SetPriority(DMAC_IRQn, NvicPriorityDmac);
		NVIC_SetPriority(EIC_IRQn, NvicPriorityPins);
#elif RP2040
		NVIC_SetPriority((IRQn_Type)StepTcIRQn, NvicPriorityStep);
		NVIC_SetPriority(IO_IRQ_BANK0_IRQn, NvicPriorityPins);
#else
# error Undefined processor
#endif
	}

#if !RP2040
	// Erase the firmware (but not the bootloader) and reset the processor
	[[noreturn]] RAMFUNC static void EraseAndReset()
	{
# if SAME5x
		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Unlock the block of flash
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_UR | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Set address and command
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }
# elif SAMC21
		while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL)) { }
		hri_nvmctrl_clear_STATUS_reg(NVMCTRL, NVMCTRL_STATUS_MASK);

		// Unlock the block of flash
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart / 2);		// note the /2 because the command takes the address in 16-bit words
		hri_nvmctrl_write_CTRLA_reg(NVMCTRL, NVMCTRL_CTRLA_CMD_UR | NVMCTRL_CTRLA_CMDEX_KEY);

		while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL)) { }
		hri_nvmctrl_clear_STATUS_reg(NVMCTRL, NVMCTRL_STATUS_MASK);

		// Set address and command
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart / 2);		// note the /2 because the command takes the address in 16-bit words
		hri_nvmctrl_write_CTRLA_reg(NVMCTRL, NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY);

		while (!hri_nvmctrl_get_interrupt_READY_bit(NVMCTRL)) { }
		hri_nvmctrl_clear_STATUS_reg(NVMCTRL, NVMCTRL_STATUS_MASK);
# else
#  error Unsupported processor
# endif
		ResetProcessor();
	}
#endif

	static void ShutdownAll()
	{
		Heat::SwitchOffAll();
#if SUPPORT_DRIVERS
# if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI || SUPPORT_TMC22xx
		IoPort::WriteDigital(GlobalTmcEnablePin, true);
# endif
		moveInstance->DisableAllDrives();
#endif
		CanInterface::Shutdown();
		WriteLed(0, false);
		WriteLed(1, false);
	}

	[[noreturn]] static void ShutdownAndReset()
	{
		ShutdownAll();
		ResetProcessor();
	}

	[[noreturn]] static void DoFirmwareUpdate()
	{
		ShutdownAll();

//		DisableCache();

		// Disable all IRQs
		__disable_irq();
		SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception

#if SAME5x
		for (size_t i = 0; i < 8; i++)
		{
			NVIC->ICER[i] = 0xFFFFFFFF;					// Disable IRQs
			NVIC->ICPR[i] = 0xFFFFFFFF;					// Clear pending IRQs
		}
#elif SAMC21
		NVIC->ICER[0] = 0xFFFFFFFF;						// Disable IRQs
		NVIC->ICPR[0] = 0xFFFFFFFF;						// Clear pending IRQs
#elif RP2040
		// We reboot and update the firmware in a similar manner to bootloader updates on other boards
		watchdog_hw->scratch[UpdateFirmwareMagicWordIndex] = UpdateFirmwareMagicValue;
		ResetProcessor();
#else
# error Unsupported processor
#endif

#if !RP2040
		EraseAndReset();
#endif
	}

	// Update the CAN bootloader
	[[noreturn]] static void DoBootloadereUpdate()
	{
		ShutdownAll();									// turn everything off

#if RP2040
		// We don't need to update the bootloader
#else
		// Remove the bootloader protection and set the bootloader update flag
		// On the SAME5x the first 8x 32-bit words are reserved. We store the bootloader flag in the 10th word.
		// On the SAMC21 the first 2x 32-bit words are reserved. We store the bootloader flag in the 10th word.
		union
		{
			uint64_t b64[5];
			uint32_t b32[10];
		} nvmUserRow;

		memcpy(&nvmUserRow, reinterpret_cast<const void*>(NVMCTRL_USER), sizeof(nvmUserRow));

# if SAME5x
		nvmUserRow.b64[0] |= (0x0F << 26);												// clear bootloader protection
# elif SAMC21
		nvmUserRow.b32[0] |= (0x07 << NVMCTRL_FUSES_BOOTPROT_Pos);						// clear bootloader protection
# endif
		nvmUserRow.b32[UpdateBootloaderMagicWordIndex] = UpdateBootloaderMagicValue;	// set the bootloader update flag
		_user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), 0, reinterpret_cast<const uint8_t*>(&nvmUserRow), sizeof(nvmUserRow));

		// If we reset immediately then the user area write doesn't complete and the bits get set to all 1s.
		delayMicroseconds(10000);
#endif

		ResetProcessor();
	}

#if SUPPORT_THERMISTORS && HAS_VREF_MONITOR
	static void SetupThermistorFilter(Pin pin, size_t filterIndex, bool useAlternateAdc) noexcept
	{
		thermistorFilters[filterIndex].Init(0);
		IoPort::SetPinMode(pin, PinMode::AIN);
# if SAMC21
		const AdcInput adcChan = (useAlternateAdc) ? PinToSdAdcChannel(pin) : PinToAdcChannel(pin);
# else
		const AdcInput adcChan = PinToAdcChannel(pin);
# endif
		AnalogIn::EnableChannel(adcChan, thermistorFilters[filterIndex].CallbackFeedIntoFilter, CallbackParameter(&thermistorFilters[filterIndex]), 1);
	}
#endif

	static CanAddress GetCanAddress() noexcept
	{
#if defined(EXP3HC)
		const CanAddress switches = ReadBoardAddress();
		return (switches == 0) ? CanId::Exp3HCFirmwareUpdateAddress : switches;
#elif defined(TOOL1LC) || defined(TOOL1RR) || defined(F3PTB) || defined(TOOLINDX)
		return CanId::ToolBoardDefaultAddress;
#elif defined(SAMMYC21) || defined(RPI_PICO) || defined(FLY36RRF)
		return CanId::SammyC21DefaultAddress;
#elif defined(EXP1XD)
		return CanId::Exp1XDBoardDefaultAddress;
#elif defined(EXP1HCL) || defined(M23CL)
		return CanId::Exp1HCLBoardDefaultAddress;
#elif defined(ATECM)
		return CanId::ATECMBoardDefaultAddress;
#elif defined(ATEIO)
		return CanId::ATEIOBoardDefaultAddress;
#elif defined(SZP)
		return CanId::SZPDefaultAddress;
#else
# 	 error Unknown board
#endif
	}

	static void InitVinMonitor()
	{
#if HAS_VOLTAGE_MONITOR
		currentVin = 0;
		highestVin = 0;
		lowestVin = 65535;
		numUnderVoltageEvents = 0;
		previousUnderVoltageEvents = 0;
		numOverVoltageEvents = 0;
		previousOverVoltageEvents = 0;

		vinFilter.Init(0);
		IoPort::SetPinMode(VinMonitorPin, AIN);
		AnalogIn::EnableChannel(PinToAdcChannel(VinMonitorPin), vinFilter.CallbackFeedIntoFilter, CallbackParameter(&vinFilter), 1);
#endif
	}

	static void EstablishBoardVariant();
	static void InitLeds();
}	// end namespace Platform

// LED management
static void Platform::InitLeds()
{
	// Set up the LED pins
#if defined(TOOL1LC)
	if (boardVariant == 1)
	{
		// The LEDs are connected to the SWDIO and SWCLK pins, so don't activate them in a debug build or if a debugger is attached
# ifndef DEBUG
		if (!DSU->STATUSB.bit.DBGPRES)
		{
			for (Pin pin : LedPinsV11)
			{
				IoPort::SetPinMode(pin, (LedActiveHighV11) ? OUTPUT_LOW : OUTPUT_HIGH);
			}
		}
# endif
	}
	else
	{
		for (Pin pin : LedPinsV10)
		{
			IoPort::SetPinMode(pin, (LedActiveHighV10) ? OUTPUT_LOW : OUTPUT_HIGH);
		}
	}
#elif defined(EXP3HC)
	if (boardVariant >= 2)
	{
		// The LEDs are connected to the SWDIO and SWCLK pins, so don't activate them in a debug build or if a debugger is attached
# ifndef DEBUG
		if (!DSU->STATUSB.bit.DBGPRES)
		{
			for (Pin pin : LedPins_v103)
			{
				IoPort::SetPinMode(pin, (LedActiveHigh_v103) ? OUTPUT_LOW : OUTPUT_HIGH);
			}
		}
# endif
	}
	else
	{
		for (Pin pin : LedPins_v102)
		{
			IoPort::SetPinMode(pin, (LedActiveHigh_v102) ? OUTPUT_LOW : OUTPUT_HIGH);
		}
	}
#elif !((defined(EXP1HCL) || defined(M23CL) || defined(SZP) || defined(TOOL1RR) || defined(F3PTB)) && defined(DEBUG))		// EXP1HCL has the LEDs connected to the SWD pins
	for (Pin pin : LedPins)
	{
		IoPort::SetPinMode(pin, (LedActiveHigh) ? OUTPUT_LOW : OUTPUT_HIGH);
	}
#endif
	Platform::WriteLed(0, true);					// turn LED on for debugging
}

void Platform::WriteLed(uint8_t ledNumber, bool turnOn)
{
#if defined(TOOL1LC)
	if (boardVariant == 1)
	{
		if (ledNumber < ARRAY_SIZE(LedPinsV11))
		{
			digitalWrite(LedPinsV11[ledNumber], (LedActiveHighV11) ? turnOn : !turnOn);
		}
	}
	else
	{
		if (ledNumber < ARRAY_SIZE(LedPinsV10))
		{
			digitalWrite(LedPinsV10[ledNumber], (LedActiveHighV10) ? turnOn : !turnOn);
		}
	}
#elif defined(EXP3HC)
	if (boardVariant >= 2)
	{
		if (ledNumber < ARRAY_SIZE(LedPins_v103))
		{
			digitalWrite(LedPins_v103[ledNumber], (LedActiveHigh_v103) ? turnOn : !turnOn);
		}
	}
	else
	{
		if (ledNumber < ARRAY_SIZE(LedPins_v102))
		{
			digitalWrite(LedPins_v102[ledNumber], (LedActiveHigh_v102) ? turnOn : !turnOn);
		}
	}
#else
	if (ledNumber < ARRAY_SIZE(LedPins))
	{
		digitalWrite(LedPins[ledNumber], (LedActiveHigh) ? turnOn : !turnOn);
	}
#endif
}

void Platform::EstablishBoardVariant()
{
#if defined(TOOL1LC)
	// On the board detect pin:
	// Tool board V1.0 and earlier has 1K lower resistor, 10K upper, so will read as low
	// Tool board V1.1 has 10K lower resistor, 1K upper, so will read as high
	SetPinMode(BoardTypePin, INPUT, false);
	boardVariant = (digitalRead(BoardTypePin)) ? 1 : 0;
#elif defined(EXP1HCL)
	// EXP1HCL v1.0 has 10K lower resistor, 1K upper giving 3.0V on the board detect pin
	// EXP1HCL v2.0 has 25.5K lower resistor, 16K upper giving 2.03V on the board detect pin
	SetPinMode(BoardTypePin, AIN);
	const AdcInput chan = PinToAdcChannel(BoardTypePin);
	AnalogIn::EnableChannel(chan, nullptr, CallbackParameter(), 0);
	constexpr float ratioV1 = 10.0/(10.0+1.0);
	constexpr float ratioV2 = 25.5/(25.5+16.0);
	constexpr float midRatio = (ratioV1 + ratioV2) * 0.5;
	constexpr uint16_t threshold = (uint16_t)(midRatio * (1u << AnalogIn::AdcBits));
	uint16_t res;
	do
	{
		res = AnalogIn::ReadChannel(chan);
	} while (res == 0);
	static_assert(ratioV1 > ratioV2);
	boardVariant = (res > threshold) ? 0 : 1;
	AnalogIn::DisableChannel(chan);									// this does nothing currently, but might in future
#elif defined(EXP3HC)
	// Version 0.9 board has pulldown resistors on BoardTypePins 0, 1 and 2 but we don't support it
	// Version 1.01 or earlier board has a pulldown resistor on BoardTypePins[0] only
	// Version 1.02 board has pulldown resistors on BoardTypePins[0]  and BoardTypePins[1]
	// Version 1.03 board has a pulldown resistor on BoardTypePins[1] only
	SetPinMode(BoardTypePins[0], INPUT_PULLUP, false);
	SetPinMode(BoardTypePins[1], INPUT_PULLUP, false);
	SetPinMode(BoardTypePins[2], INPUT_PULLUP, false);
	delayMicroseconds(10);
	boardVariant = (digitalRead(BoardTypePins[1])) ? 0
					: (digitalRead(BoardTypePins[0])) ? 2 : 1;
#endif
}

// Initialisation
void Platform::Init()
{
	IoPort::Init();

#if NUM_ASYNC_PORTS != 0
	asyncPorts[0] = new AsyncSerial(Serial0Params);
	asyncPorts[0]->setInterruptPriority(NvicPriorityUart, NvicPriorityUart);
# if NUM_ASYNC_PORTS > 1
	asyncPorts[1] = new AsyncSerial(Serial1Params);
	asyncPorts[1]->setInterruptPriority(NvicPriorityUart, NvicPriorityUart);
# endif
#endif

	EstablishBoardVariant();
	InitLeds();

#if SUPPORT_INDUCTIVE_HEATER
	inductiveHeaterPort.Init();
#endif

	// Turn all outputs off
	for (size_t pin = 0; pin < ARRAY_SIZE(PinTable); ++pin)
	{
		const PinDescription& p = PinTable[pin];
		if (p.pinNames != nullptr)
		{
			if (   StringStartsWith(p.pinNames, "out")
				&& strlen(p.pinNames) < 5							// don't set "outN.tach" pins to outputs
		       )
			{
#if SAMC21
				// Set high driver strength on the output pins because they drive the heater and fan mosfets directly
				SetDriveStrength(pin, 1);
#endif

#ifdef TOOL1LC
				// OUT2 is intended to drive the hot end fan, so default it to on
				IoPort::SetPinMode(pin, (StringEqualsIgnoreCase(p.pinNames, "out2")) ? OUTPUT_HIGH : OUTPUT_LOW);
																	// turn on fan on out2, turn off heaters and other fans
#else
				IoPort::SetPinMode(pin, OUTPUT_LOW);				// turn off heaters and fans (although this will turn on PWM fans)
#endif
			}
			else if (StringStartsWith(p.pinNames, "io") && StringStartsWith(p.pinNames + 3,".out"))
			{
				IoPort::SetPinMode(pin, INPUT_PULLDOWN);			// looks like BLTouch doesn't like its command input to be floating, so enable pulldown on ioX.out pins
			}
			else if (StringStartsWith(p.pinNames, "spi.cs"))
			{
				IoPort::SetPinMode(pin, INPUT_PULLUP);				// ensure SPI CS lines are high so that temp daughter boards don't drive the bus before they are configured
			}
#ifdef ATEIO
			else if (StringStartsWith(p.pinNames, "!rsel") || StringStartsWith(p.pinNames, "!tsel"))
			{
				SetDriveStrength(pin, 2);
				IoPort::SetPinMode(pin, OUTPUT_HIGH);
			}
#endif
		}
	}

	InitialiseInterrupts();											// set interrupt priorities before we enable any interrupts

#if defined(SAMMYC21) && USE_SERIAL_DEBUG
	asyncPorts[0]->begin(115200);											// set up the UART with the same baud rate as the bootloader
#elif defined(RPI_PICO) || defined(FLY36RRF)
	serialUSB.Start(NoPin);
#elif USE_SERIAL_DEBUG
	// Set up the UART to send to PanelDue for debugging
	// CAUTION! This sends data to pin io0.out on a tool board, which interferes with a BLTouch connected to that pin. So don't do it in normal use.
	asyncPorts[0]->begin(57600);
#endif

	// Initialise the rest of the IO subsystem
#if SAME5x
	ADC_temperature_init();
#endif

#if NUM_CURRENT_SENSORS != 0
	// Initialise the current sensor system. Each sensor is associated with an ADC input.
	for (size_t sensorNum = 0; sensorNum < NUM_CURRENT_SENSORS; ++sensorNum)
	{
		IoPort::SetPinMode(CurrentSensorPins[sensorNum], AIN);
		AnalogIn::EnableChannel(PinToAdcChannel(CurrentSensorPins[sensorNum]), Platform::CurrentSensorAinCallback, CallbackParameter(sensorNum), 1);
	}
#endif

#if HAS_ADDRESS_SWITCHES
	// Set up the board ID switch inputs
	for (unsigned int i = 0; i < 4; ++i)
	{
		IoPort::SetPinMode(BoardAddressPins[i], INPUT_PULLUP);
	}
#endif

	// Set up VIN voltage monitoring
#if HAS_VOLTAGE_MONITOR
	InitVinMonitor();
#endif

#if HAS_12V_MONITOR
	currentV12 = 0;
	highestV12 = 0;
	lowestV12 = 65535;

	v12Filter.Init(0);
	IoPort::SetPinMode(V12MonitorPin, AIN);
	AnalogIn::EnableChannel(PinToAdcChannel(V12MonitorPin), v12Filter.CallbackFeedIntoFilter, CallbackParameter(&v12Filter), 1);
#endif

#if HAS_VREF_MONITOR
	// Set up the Vref and Vssa filters
	SetupThermistorFilter(VrefPin, VrefFilterIndex, false);
	SetupThermistorFilter(VssaPin, VssaFilterIndex, false);
# if SAMC21 && SUPPORT_SDADC
	SetupThermistorFilter(VrefPin, SdAdcVrefFilterIndex, true);
# endif
#endif

	// Set up the MCU temperature sensors
	mcuTemperature.current = 0.0;
	mcuTemperature.maximum = -273.16;
	mcuTemperature.minimum = 999.0;
	mcuTemperatureAdjust = 0.0;

	// Set up the MCU temperature sense filters
#if SAME5x
	tpFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, CallbackParameter(&tpFilter), 1, 0);
	tcFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, CallbackParameter(&tcFilter), 1, 0);
#elif SAMC21 || RP2040
	tsensFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(tsensFilter.CallbackFeedIntoFilter, CallbackParameter(&tsensFilter), 1);
#else
# error Unsupported processor
#endif

#if HAS_BUTTONS
	for (Pin pin : ButtonPins)
	{
		IoPort::SetPinMode(pin, PinMode::INPUT_PULLUP);
	}
#endif

#if NUM_SHARED_SPI != 0
	// Currently we support only 0 or 1 shared SPI channels
	sharedSpi = new SharedSpiDevice(SharedSpiParams);
#endif

#if NUM_I2C_CHANNELS != 0
# ifdef TOOL1LC
	if (boardVariant != 0)
# endif
	{
		sharedI2C[0] = new SharedI2CMaster(I2C0Params);
	}
#endif

#if NUM_I2C_CHANNELS >= 2
	sharedI2C[1] = new SharedI2CMaster(I2C1Params);
#endif

#if SUPPORT_ADS131M02
	loadCellAdc = new ADS131M02;
#endif
#if SUPPORT_LP5817
	ledStatusControl = new LedStatusControl(LP5817_I2CChannel);
#endif

#ifdef ATEIO
	ExtendedAnalog::Init(*sharedSpi);					// must init sharedSpi before calling this
#endif

	uniqueId.SetFromCurrentBoard();

#if SUPPORT_LIS3DH
# ifdef TOOL1LC
	if (boardVariant != 0)
# endif
	{
# if ACCELEROMETER_USES_SPI
		AccelerometerHandler::Init(*sharedSpi);
# else
		AccelerometerHandler::Init(GetSharedI2C(Lis_I2CChannel));
# endif
	}
#endif

#if SUPPORT_LDC1612
# ifdef TOOL1LC
	if (boardVariant != 0)
# endif
	{
		ScanningSensorHandler::Init(GetSharedI2C(LDC1612_I2CChannel));
	}
#endif

#if SUPPORT_AS5601
	MFMHandler::Init(GetSharedI2C(0));
#endif

	CanInterface::Init(GetCanAddress(), CANInstanceNumber, UseLaterCanPins, true);
	lastPollTime = millis();
}

// Perform minimal initialisation prior to updating the bootloader
void Platform::InitMinimal()
{
	EstablishBoardVariant();
	InitLeds();
	InitVinMonitor();
	InitialiseInterrupts();
#if RP2040
	serialUSB.Start(NoPin);
#endif
	CanInterface::Init(GetCanAddress(), CANInstanceNumber, UseLaterCanPins, false);
}

void Platform::Spin()
{
#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	static bool powered = false;
#endif

	if (deferredCommand != DeferredCommand::none && millis() - whenDeferredCommandRequested > 200)
	{
		switch (deferredCommand)
		{
		case DeferredCommand::firmwareUpdate:
			DoFirmwareUpdate();
			break;

		case DeferredCommand::bootloaderUpdate:
			DoBootloadereUpdate();
			break;

		case DeferredCommand::reset:
			ShutdownAndReset();
			break;

		case DeferredCommand::testWatchdog:
			deliberateError = true;
			SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);							// disable the system tick interrupt so that we get a watchdog timeout reset
			break;

		case DeferredCommand::testDivideByZero:
			deliberateError = true;
			(void)Tasks::DoDivide(1, 0);
			__ISB();
			deliberateError = false;
			break;

		case DeferredCommand::testUnalignedMemoryAccess:
			deliberateError = true;
			(void)Tasks::DoMemoryRead(reinterpret_cast<const uint32_t*>(
#if RP2040
										SRAM_BASE
#else
										HSRAM_ADDR
#endif
													+ 1));
			__ISB();
			deliberateError = false;
			break;

		case DeferredCommand::testBadMemoryAccess:
			deliberateError = true;
			(void)Tasks::DoMemoryRead(reinterpret_cast<const uint32_t*>(0x30000000));	// 0x30000000 is invalid on both the SAME5x and the SAMC21
			__ISB();
			deliberateError = false;
			break;

		case DeferredCommand::testMemoryLeak:
			deliberateError = true;
			(void)Tasks::DoMemoryLeak();
			__ISB();
			deliberateError = false;
			break;

		case DeferredCommand::testSpinLockup:
			deliberateError = true;
			while (true) { }
			break;

		default:
			break;
		}
	}

	SpinMinimal();				// update the activity LED and currentVin

#if HAS_VOLTAGE_MONITOR
	const float voltsVin = GetCurrentVinVoltage();
#endif

#if HAS_12V_MONITOR
	currentV12 = v12Filter.GetSum()/v12Filter.NumAveraged();
	if (v12Filter.IsValid())
	{
		if (currentV12 < lowestV12 || millis64() < 1000)				// don't record lowest V12 while we are powering up
		{
			lowestV12 = currentV12;
		}
		if (currentV12 > highestV12)
		{
			highestV12 = currentV12;
		}
	}

	const float volts12 = (currentV12 * V12MonitorVoltageRange)/(1u << AnalogIn::AdcBits);
	if (!powered && voltsVin >= 10.5 && volts12 >= 10.5)
	{
		powered = true;
	}
	else if (powered && (voltsVin < 10.0 || volts12 < 10.0))
	{
		powered = false;
		++numUnderVoltageEvents;
	}
#elif HAS_VOLTAGE_MONITOR

	if (!powered && voltsVin >= 10.5)
	{
		powered = true;
	}
	else if (powered && voltsVin < 10.0)
	{
		powered = false;
	}
#endif

#if SUPPORT_DRIVERS
	moveInstance->Spin(
# if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
						powered
# endif
					  );
#endif	// SUPPORT_DRIVERS

	// Thermostatically-controlled fans (do this after getting TMC driver status)
	const uint32_t now = millis();
	const bool checkSensors = (now - lastFanCheckTime >= FanCheckInterval);
	(void)FansManager::CheckFans(checkSensors);
	if (checkSensors)
	{
		lastFanCheckTime = now;
	}

	// Update the Status LED. Flash it quickly (8Hz) if we are not synced to the master, else flash in sync with the master (about 2Hz).
	const bool synced = StepTimer::CheckSynced();
	if (synced) {
		syncedIdleTicks = 0;
	}
	WriteLed(0,
				(synced)
					? (StepTimer::GetMasterTime() & (1u << 19)) != 0
						: (StepTimer::GetTimerTicks() & (1u << 17)) != 0
		    );

	if (now - lastPollTime > 2000)
	{
		lastPollTime = now;

		// Get the chip temperature
#if SAME5x
		if (tcFilter.IsValid() && tpFilter.IsValid())
		{
			// From the datasheet:
			// T = (tl * vph * tc - th * vph * tc - tl * tp *vch + th * tp * vcl)/(tp * vcl - tp * vch - tc * vpl * tc * vph)
			const uint16_t tc_result = tcFilter.GetSum()/(tcFilter.NumAveraged() << (AnalogIn::AdcBits - 12));
			const uint16_t tp_result = tpFilter.GetSum()/(tpFilter.NumAveraged() << (AnalogIn::AdcBits - 12));

			int32_t result =  (tempCalF1 * tc_result - tempCalF2 * tp_result);
			const int32_t divisor = (tempCalF3 * tp_result - tempCalF4 * tc_result);
			result = (divisor == 0) ? 0 : result/divisor;
			mcuTemperature.current = (float)result/16 + mcuTemperatureAdjust;
#elif SAMC21
		if (tsensFilter.IsValid())
		{
			const int16_t temperatureTimes100 = (int16_t)((uint16_t)(tsensFilter.GetSum()/tsensFilter.NumAveraged()) ^ (1u << 15));
			mcuTemperature.current = (float)temperatureTimes100 * 0.01;
#elif RP2040
			if (tsensFilter.IsValid())
			{
				const float tempSensorAdcVoltage = (tsensFilter.GetSum()/tsensFilter.NumAveraged()) * (3.3/(float)(1u << AnalogIn::AdcBits));
				mcuTemperature.current = 27.0 - ((tempSensorAdcVoltage - 0.706) * (1.0/0.001721));
#else
# error Unsupported processor
#endif
			if (mcuTemperature.current < mcuTemperature.minimum)
			{
				mcuTemperature.minimum = mcuTemperature.current;
			}
			if (mcuTemperature.current > mcuTemperature.maximum)
			{
				mcuTemperature.maximum = mcuTemperature.current;
			}
		}

		static unsigned int nextSensor = 0;

		const auto ts = Heat::FindSensorAtOrAbove(nextSensor);
		if (ts.IsNotNull())
		{
#if 0
			float temp;
			const TemperatureError err = ts->GetLatestTemperature(temp);
			debugPrintf("Sensor %u err %u temp %.1f", ts->GetSensorNumber(), (unsigned int)err, (double)temp);
#endif
			nextSensor = ts->GetSensorNumber() + 1;
		}
		else
		{
			nextSensor = 0;
#if 0
			String<100> status;
			SmartDrivers::AppendDriverStatus(2, status.GetRef());
			debugPrintf("%s", status.c_str());
#elif 0
			moveInstance->Diagnostics(AuxMessage);
#elif 0
#elif 0
//			uint32_t conversionsStarted, conversionsCompleted;
//			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted);
			debugPrintf(
//							"Conv %u %u"
						"Addr %u"
#if HAS_12V_MONITOR
						" %.1fV %.1fV"
#elif HAS_VOLTAGE_MONITOR
						" %.1fV"
#endif
						" %.1fC"
#if HAS_VREF_MONITOR
						" %u %u"
#endif
//						", ptat %d, ctat %d"
#if HAS_SMART_DRIVERS
						", stat %08" PRIx32 " %08" PRIx32 " %08" PRIx32
#endif
						,
//							(unsigned int)conversionsStarted, (unsigned int)conversionsCompleted,
//							StepTimer::GetInterruptClocks(),
						(unsigned int)CanInterface::GetCanAddress(),
#if HAS_12V_MONITOR
						(double)voltsVin, (double)volts12,
#elif HAS_VOLTAGE_MONITOR
						(double)voltsVin,
#endif
						(double)currentMcuTemperature
#if HAS_VREF_MONITOR
						, (unsigned int)thermistorFilters[VrefFilterIndex].GetSum(), (unsigned int)thermistorFilters[VssaFilterIndex].GetSum()
#endif
//						, tp_result, tc_result
#if HAS_SMART_DRIVERS
						, SmartDrivers::GetAccumulatedStatus(0, 0), SmartDrivers::GetAccumulatedStatus(1, 0), SmartDrivers::GetAccumulatedStatus(2, 0)
#endif
//							, SmartDrivers::GetLiveStatus(0), SmartDrivers::GetLiveStatus(1), SmartDrivers::GetLiveStatus(2)
					   );
#endif
		}
	}

#if defined(SAMMYC21) || defined(RPI_PICO) || defined(FLY36RRF)
	//debugPrintf("IR=%" PRIx32 " ERR=%" PRIx32 " RXF0S=%" PRIx32 " RXF1S=%" PRIx32 " PSR=%" PRIx32 " CCCR=%" PRIx32 "\n",
	//	CAN0->IR.reg, CAN0->ECR.reg, CAN0->RXF0S.reg, CAN0->RXF1S.reg, CAN0->PSR.reg, CAN0->CCCR.reg);

	// If D is received from the USB port, output some diagnostics
# if defined(RPI_PICO) || defined(FLY36RRF)
	while (serialUSB.available() != 0)
# elif defined(SAMMYC21)
	while (asyncPorts[0]->available() != 0)
# endif
	{
# if defined(SAMMYC21)
		const char c = asyncPorts[0]->read();
# elif defined(RPI_PICO) || defined(FLY36RRF)
		const char c = serialUSB.read();
# endif
		if (c == 'D')
		{
			debugPrintf("Version %s\n", VERSION);
			String<StringLength256> reply;
			Tasks::Diagnostics(reply.GetRef());
			debugPrintf("%s\n", reply.c_str());
			reply.Clear();
			CanInterface::Diagnostics(reply.GetRef());
			debugPrintf("%s\n", reply.c_str());
			reply.Clear();
			StepTimer::Diagnostics(reply.GetRef());
			debugPrintf("%s\n", reply.c_str());
			reply.Clear();

			uint32_t conversionsStarted, conversionsCompleted, conversionTimeouts, errors;
			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
			debugPrintf("ADC conversions started %" PRIu32 ", completed %" PRIu32 ", timed out %" PRIu32 ", errs %" PRIu32 "\n",
						conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
# if SUPPORT_DRIVERS
			moveInstance->AppendDiagnostics(reply.GetRef());
			debugPrintf("%s\n", reply.c_str());
			reply.Clear();
# endif

			//moveInstance->DebugPrintCdda();
# if SUPPORT_LIS3DH
			debugPrintf("Accelerometer detected: %s", AccelerometerHandler::IsPresent() ? "yes" : "no");
# endif
		}
	}
#endif
}

void Platform::SpinMinimal()
{
	if (millis() - whenLastCanMessageProcessed > ActLedFlashTime)
	{
		WriteLed(1, false);
	}

#if HAS_VOLTAGE_MONITOR
	// Get the VIN voltage
	currentVin = vinFilter.GetSum()/vinFilter.NumAveraged();			// we must store this even if it is not valid yet
	if (vinFilter.IsValid())
	{
		if (currentVin < lowestVin || millis64() < 1000)				// don't record lowest Vin while we are powering up
		{
			lowestVin = currentVin;
		}
		if (currentVin > highestVin)
		{
			highestVin = currentVin;
		}
	}
#endif
}

#if SUPPORT_THERMISTORS

// Get the index of the averaging filter for an analog port
int Platform::GetAveragingFilterIndex(const IoPort& port)
{
	for (size_t i = 0; i < ARRAY_SIZE(TempSensePins); ++i)
	{
		if (port.GetPin() == TempSensePins[i])
		{
# if SAMC21 && SUPPORT_SDADC
			return (i == 0 && port.UseAlternateConfig()) ? SdAdcTemp0FilterIndex : (int) i;
# endif
			return (int)i;
		}
	}
	return -1;
}

// Setup the filter for a thermistor input. The port has already been set to analog in mode.
void Platform::InitThermistorFilter(const IoPort& port) noexcept
{
	const int adcFilterChannel = GetAveragingFilterIndex(port);
	if (adcFilterChannel >= 0)
	{
		InitThermistorFilter(port.GetPin(), (unsigned int)adcFilterChannel, port.UseAlternateConfig());
	}
}

// This one is used for direct initialisation of a filter channel
void Platform::InitThermistorFilter(Pin p, unsigned int adcFilterChannel, bool useAlternateConfig) noexcept
{
	ThermistorAveragingFilter& filter = thermistorFilters[adcFilterChannel];
	filter.Init((1u << AnalogIn::AdcBits) - 1);
	const AdcInput adcChan = IoPort::PinToAdcInput(p, useAlternateConfig);
	AnalogIn::EnableChannel(adcChan, filter.CallbackFeedIntoFilter, CallbackParameter(&filter), 1);
}

ThermistorAveragingFilter *Platform::GetAdcFilter(unsigned int filterNumber)
{
	return &thermistorFilters[filterNumber];
}

#endif

#if HAS_VREF_MONITOR

ThermistorAveragingFilter *Platform::GetVssaFilter(unsigned int filterNumber)
{
#if SAMC21
	// The SDADC channel has INN connected to VSSA and no separate VSSA monitor
	return (filterNumber < NumThermistorInputs) ? &thermistorFilters[VssaFilterIndex] : nullptr;
#else
	return &thermistorFilters[VssaFilterIndex];
#endif
}

ThermistorAveragingFilter *Platform::GetVrefFilter(unsigned int filterNumber)
{
#if SAMC21
	// The SDADC channel has a separate VSSA monitor
	return (filterNumber == SdAdcTemp0FilterIndex) ? &thermistorFilters[SdAdcVrefFilterIndex] : &thermistorFilters[VrefFilterIndex];
#else
	return &thermistorFilters[VrefFilterIndex];
#endif
}

#endif

#if NUM_CURRENT_SENSORS != 0

void Platform::CurrentSensorAinCallback(CallbackParameter cp, int32_t val) noexcept
{
	const size_t sensorNumber = cp.u32;
	if (sensorNumber < NUM_CURRENT_SENSORS)
	{
		currentSensorReadings[sensorNumber] = val;
		if (currentSensorCallbackThresholds[sensorNumber] != 0 && val > currentSensorCallbackThresholds[sensorNumber])
		{
			StandardCallbackFunction *null const cbfunc = currentSensorCallbackFunctions[sensorNumber];
			if (cbfunc != nullptr)
			{
				(*cbfunc)(currentSensorCallbackParameters[sensorNumber]);
			}
		}
	}
}

#endif

const MinCurMax& Platform::GetMcuTemperatures()
{
	return mcuTemperature;
}

void Platform::KickHeatTaskWatchdog()
{
	heatTaskIdleTicks = 0;
}

uint32_t Platform::GetHeatTaskIdleTicks()
{
	return heatTaskIdleTicks;
}

uint32_t Platform::GetSyncedIdleTicks()
{
	return syncedIdleTicks;
}

#if USE_SERIAL_DEBUG

// Output a character to the debug channel
bool Platform::DebugPutc(char c) noexcept
{
	if (c != 0)
	{
#if defined(RPI_PICO) || defined(FLY36RRF)
		serialUSB.write(c);
#else
		asyncPorts[0]->write(c);
#endif
	}
	return true;
}

#endif

void Platform::LogError(ErrorCode e)
{
	errorCodeBits |= (uint32_t)e;
}

bool Platform::Debug(Module module) noexcept
{
	return debugMaps[module.ToBaseType()].IsNonEmpty();
}

DebugFlags Platform::GetDebugFlags(Module m) noexcept
{
	return debugMaps[m.ToBaseType()];
}

GCodeResult Platform::ProcessRemoteM111(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M111Params);

	// Debug flags are as set by the D parameter. If D is not specified then S0 means all off, S1 means lower 8 bits on.
	uint32_t flags = 0;
	bool seen = parser.GetUintParam('D', flags);
	if (!seen)
	{
		uint8_t sParam;
		seen = parser.GetUintParam('S', sParam);
		if (seen && sParam != 0)
		{
			flags = DefaultDebugFlags;
		}
	}

	uint8_t module = Module::numModules;
	if (parser.GetUintParam('P', module))
	{
		seen = true;
	}

	if (seen)
	{
		if (module < Module::numModules)
		{
			debugMaps[module].SetFromRaw(flags);
		}
		else if (flags != 0)
		{
			// Repetier Host sends M111 with various S parameters to enable echo and similar features, which used to turn on all our debugging.
			// But it's not useful to enable all debugging anyway. So we no longer allow debugging to be enabled without a P parameter.
			reply.copy("Use P parameter to specify which module to debug");
			return GCodeResult::error;
		}
		else
		{
			// M111 S0 with no P parameter still clears all debugging
			for (DebugFlags& dbf : debugMaps)
			{
				dbf.Clear();
			}
		}
	}

	reply.copy("Debugging on for modules:");
	for (size_t i = 0; i < Module::numModules; i++)
	{
		if (debugMaps[i].IsNonEmpty())
		{
			reply.catf(" %s(%u - %#" PRIx16 ")", Module(i).ToString(), i, debugMaps[i].GetRaw());
		}
	}

	reply.lcat("Debugging off for modules:");
	for (size_t i = 0; i < Module::numModules; i++)
	{
		if (debugMaps[i].IsEmpty())
		{
			reply.catf(" %s(%u)", Module(i).ToString(), i);
		}
	}

	return GCodeResult::ok;
}

#if HAS_ADDRESS_SWITCHES

uint8_t Platform::ReadBoardAddress()
{
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 4; ++i)
	{
		if (!digitalRead(BoardAddressPins[i]))
		{
			rslt |= 1 << i;
		}
	}
	return rslt;
}

#endif

// Get a pointer to the four unique ID dwords
const UniqueIdBase& Platform::GetUniqueId() noexcept
{
	return uniqueId;
}

void Platform::Tick() noexcept
{
	++heatTaskIdleTicks;
	++syncedIdleTicks;
}

void Platform::StartFirmwareUpdate()
{
	whenDeferredCommandRequested = millis();
	deferredCommand = DeferredCommand::firmwareUpdate;
}

void Platform::StartBootloaderUpdate()
{
	whenDeferredCommandRequested = millis();
	deferredCommand = DeferredCommand::bootloaderUpdate;
}

void Platform::StartReset()
{
	whenDeferredCommandRequested = millis();
	deferredCommand = DeferredCommand::reset;
}

void Platform::EmergencyStop()
{
	whenDeferredCommandRequested = millis();
	deferredCommand = DeferredCommand::reset;
}

// This is called when we start processing any CAN message except for regular messages e.g. time sync
void Platform::OnProcessingCanMessage()
{
	whenLastCanMessageProcessed = millis();
	WriteLed(1, true);				// turn the ACT LED on
}

GCodeResult Platform::DoDiagnosticTest(const CanMessageDiagnosticTest& msg, const StringRef& reply)
{
	if ((uint16_t)~msg.invertedTestType != msg.testType)
	{
		reply.copy("Bad diagnostic test message");
		return GCodeResult::error;
	}

	switch (msg.testType)
	{
	case 102:		// Show the square root calculation time. Caution: may disable interrupt for several tens of microseconds.
		{
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			bool ok = true;
			uint32_t tim3 = 0;
			float val = 10000.0;
			for (unsigned int i = 0; i < iterations; ++i)
			{
				IrqDisable();
				asm volatile("":::"memory");
				uint32_t now1 = SysTick->VAL;
				const float nval = fastSqrtf(val);
				uint32_t now2 = SysTick->VAL;
				asm volatile("":::"memory");
				IrqEnable();
				now1 &= 0x00FFFFFF;
				now2 &= 0x00FFFFFF;
				tim3 += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
				if (nval != sqrtf(val))
				{
					ok = false;
				}
				val = nval;
			}

			reply.printf("Square roots: float %.2fus %s",
							(double)((float)(tim3 * (1'000'000/iterations))/SystemCoreClock), (ok) ? "ok" : "ERROR");
			return (ok) ? GCodeResult::ok : GCodeResult::error;
		}

	case 108:
		{
			unsigned int i = 100;
			IrqDisable();
			asm volatile("":::"memory");
			uint32_t now1 = SysTick->VAL;
			do
			{
				--i;
				(void)StepTimer::GetTimerTicksWhenInterruptsDisabled();
			} while (i != 0);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			IrqEnable();
			now1 &= 0x00FFFFFF;
			now2 &= 0x00FFFFFF;
			uint32_t tim1 = ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			reply.printf("Reading step timer 100 times took %.2fus", (double)((1'000'000.0f * (float)tim1)/(float)SystemCoreClock));
		}

#if !RP2040
		// Also check the correspondence between the CAN timestamp timer and the step clock
		{
			uint32_t startClocks, endClocks;
			uint16_t startTimeStamp, endTimeStamp;
			{
				AtomicCriticalSectionLocker lock;
				startClocks = StepTimer::GetTimerTicksWhenInterruptsDisabled();
				startTimeStamp = CanInterface::GetTimeStampCounter();
			}
			delay(2);
			{
				AtomicCriticalSectionLocker lock;
				endClocks = StepTimer::GetTimerTicksWhenInterruptsDisabled();
				endTimeStamp = CanInterface::GetTimeStampCounter();
			}
			const uint32_t tsDiff = (((endTimeStamp - startTimeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;
			reply.lcatf("Clock diff %" PRIu32 ", ts diff %" PRIu32, endClocks - startClocks, tsDiff);
		}
#endif
		return GCodeResult::ok;

#if SAME5x
	case 500:												// report write buffer
		reply.printf("Write buffer is %s", (SCnSCB->ACTLR & SCnSCB_ACTLR_DISDEFWBUF_Msk) ? "disabled" : "enabled");
		return GCodeResult::ok;

	case 501:
		SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;		// disable write buffer
		return GCodeResult::ok;

	case 502:
		SCnSCB->ACTLR &= ~SCnSCB_ACTLR_DISDEFWBUF_Msk;		// enable write buffer
		return GCodeResult::ok;
#endif

	case 1001:	// test watchdog
		deferredCommand = DeferredCommand::testWatchdog;
		return GCodeResult::ok;

	case 1002: // test that we get a software reset if a Spin() function takes too long
		deferredCommand = DeferredCommand::testSpinLockup;
		return GCodeResult::ok;

	case 1004:
		deferredCommand = DeferredCommand::testDivideByZero;
		return GCodeResult::ok;

	case 1005:
		deferredCommand = DeferredCommand::testUnalignedMemoryAccess;
		return GCodeResult::ok;

	case 1006:
		deferredCommand = DeferredCommand::testBadMemoryAccess;
		return GCodeResult::ok;

	case 1007:						// read or write memory
		deliberateError = true;		// in case this causes a crash
		if (msg.param16 == 1)		// if writing
		{
			for (uint32_t i = 0; i < msg.param32[2]; ++i)
			{
				reinterpret_cast<uint32_t*>(msg.param32[0])[i] = msg.param32[1];
			}
		}
		else						// reading
		{
			reply.printf("Address 0x%08" PRIx32 ":", msg.param32[0]);
			for (uint32_t i = 0; i < msg.param32[2]; ++i)
			{
				reply.catf(" %08" PRIx32, reinterpret_cast<const uint32_t*>(msg.param32[0])[i]);
			}
		}
		deliberateError = false;
		return GCodeResult::ok;

	case 1008:
		deferredCommand = DeferredCommand::testMemoryLeak;
		return GCodeResult::ok;

	default:
		reply.printf("Unknown test type %u", msg.testType);
		return GCodeResult::error;
	}
}

bool Platform::WasDeliberateError() noexcept
{
	return deliberateError;
}

#if SAME5x

// Set a contiguous range of interrupts to the specified priority
void Platform::SetInterruptPriority(IRQn base, unsigned int num, uint32_t prio)
{
	do
	{
		NVIC_SetPriority(base, prio);
		base = (IRQn)(base + 1);
		--num;
	}
	while (num != 0);
}

#endif

#if HAS_VOLTAGE_MONITOR

MinCurMax Platform::GetPowerVoltages(bool resetMinMax) noexcept
{
	MinCurMax result;
	result.minimum = AdcReadingToVinVoltage(lowestVin);
	result.current = AdcReadingToVinVoltage(currentVin);
	result.maximum = AdcReadingToVinVoltage(highestVin);
	if (resetMinMax)
	{
		lowestVin = currentVin;
		highestVin = currentVin;
	}
	return result;
}

float Platform::GetCurrentVinVoltage() noexcept
{
	return AdcReadingToVinVoltage(currentVin);
}

#endif

#if HAS_12V_MONITOR

MinCurMax Platform::GetV12Voltages(bool resetMinMax) noexcept
{
	MinCurMax result;
	result.minimum = AdcReadingToV12Voltage(lowestV12);
	result.current = AdcReadingToV12Voltage(currentV12);
	result.maximum = AdcReadingToV12Voltage(highestV12);
	if (resetMinMax)
	{
		lowestV12 = currentV12;
		highestV12 = currentV12;
	}
	return result;
}

float Platform::GetCurrentV12Voltage() noexcept
{
	return AdcReadingToV12Voltage(currentV12);
}

#endif

#if SUPPORT_INDUCTIVE_HEATER

void Platform::SetInductiveHeaterPwm(float pwm) noexcept
{
	inductiveHeaterPort.SetPwm(pwm);
}

#endif

#if NUM_CURRENT_SENSORS != 0

// Get the current read from the specified sensor
float Platform::GetCurrentSensorReading(size_t sensorNumber) noexcept
{
	return std::ldexp((float)currentSensorReadings[sensorNumber] * CurrentSensorFullScaleCurrents[sensorNumber], -AnalogIn::AdcBits);
}

// Set up a callback when the current exceed the specified threshold. Set the threshold to zero or the function to nullptr to stop getting callbacks.
void Platform::SetCurrentSensorCallbackThreshold(size_t sensorNumber, float val, StandardCallbackFunction *null func, CallbackParameter param) noexcept
{
	currentSensorCallbackParameters[sensorNumber] = param;
	currentSensorCallbackThresholds[sensorNumber] = (uint32_t)(std::ldexp(val, AnalogIn::AdcBits)/CurrentSensorFullScaleCurrents[sensorNumber]);
	currentSensorCallbackFunctions[sensorNumber] = func;
}

#endif

void Platform::AppendBoardAndFirmwareDetails(const StringRef& reply) noexcept
{
	// This must be formatted in a specific way for the ATE
#if defined(TOOL1LC)
	reply.lcatf("Duet " BOARD_TYPE_NAME " rev %s firmware version " VERSION " (%s%s)",
				(boardVariant == 1) ? "1.1 or later" : "1.0 or earlier",
				DateText, TimeSuffix);
#elif defined(EXP1HCL)
	reply.lcatf("Duet " BOARD_TYPE_NAME " rev %s firmware version " VERSION " (%s%s)",
				(boardVariant == 1) ? "2.0 or later" : "1.0a or earlier",
				DateText, TimeSuffix);
#elif defined(EXP3HC)
	reply.lcatf("Duet " BOARD_TYPE_NAME " rev %s firmware version " VERSION " (%s%s)",
				(boardVariant == 2) ? "1.03 or later"
				: (boardVariant == 1) ? "1.02"
					: "1.01 or earlier",
				DateText, TimeSuffix);
#else
	reply.lcatf("Duet " BOARD_TYPE_NAME " firmware version " VERSION " (%s%s)",
				DateText, TimeSuffix);
#endif
}

void Platform::AppendDiagnostics(const StringRef& reply) noexcept
{
#if SUPPORT_THERMISTORS
	bool ok = true;
	for (const ThermistorAveragingFilter& filter : thermistorFilters)
	{
		if (!filter.CheckIntegrity())
		{
			ok = false;
			reply.lcatf("Averaging filter %u is bad", &filter - thermistorFilters);
		}
	}
	if (ok)
	{
		reply.lcat("All averaging filters OK");
	}
#endif
}

#if defined(EXP3HC) || defined(TOOL1LC) || defined(EXP1HCL)

uint8_t Platform::GetBoardVariant() noexcept
{
	return boardVariant;
}

#endif

// End
