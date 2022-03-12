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
#include "Movement/StepperDrivers/TMC51xx.h"
#include "Movement/StepperDrivers/TMC22xx.h"
#include "AdcAveragingFilter.h"
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

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH
# include <CommandProcessing/AccelerometerHandler.h>
#endif

#if SUPPORT_CLOSED_LOOP
# include <ClosedLoop/ClosedLoop.h>
#endif

#ifdef ATEIO
# include <Hardware/ATEIO/ExtendedAnalog.h>
#endif

#include <hpl_user_area.h>

#if SAME5x

# include <hri_nvmctrl_e54.h>
constexpr uint32_t FlashBlockSize = 0x00010000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 64K for the bootloader

#if USE_TC_FOR_STEP
# include <hri_tc_e54.h>
#endif

#elif SAMC21

# include <hri_nvmctrl_c21.h>
constexpr uint32_t FlashBlockSize = 0x00004000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 16K for the bootloader

#if USE_TC_FOR_STEP
# include <hri_tc_c21.h>
#endif

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
	testBadMemoryAccess
};

static volatile DeferredCommand deferredCommand = DeferredCommand::none;
static volatile uint32_t whenDeferredCommandRequested;
static uint32_t realTime = 0;
static bool deliberateError = false;

namespace Platform
{
	static uint32_t errorCodeBits = 0;

	UniqueIdBase uniqueId;
	bool isPrinting = false;
#ifdef TOOL1LC
	static uint8_t boardVariant = 0;
#endif

#if SUPPORT_DRIVERS
	static constexpr float DefaultStepsPerMm = 80.0;

# if SUPPORT_SLOW_DRIVERS
#  ifdef EXP1XD
#   if USE_TC_FOR_STEP
	// The first element of slowDriverStepTimingClocks is the pulse width in TC clocks. The other elements are in step clock units. The second element is the step high + step low time
	constexpr unsigned int TcPrescaler = 4;															// use prescaler 4
	constexpr uint32_t TcPrescalerRegVal = TC_CTRLA_PRESCALER_DIV4;									// use prescaler 4
	constexpr float StepPulseClocksPerMicrosecond = 48.0/TcPrescaler;								// we clock the TC from the 48MHz clock
	constexpr unsigned int StepClocksToStepTCClocks = 64/TcPrescaler;								// step clock uses the same 48MHx clcok and x64 prescaler
	constexpr float MinimumStepHighMicroseconds = 0.2;

	uint32_t slowDriverStepTimingClocks[4] = { 2 * StepClocksToStepTCClocks, 4, 2, 4 };				// default to slower timing for external drivers (2 clocks = 2.67us)

	// Convert microseconds to StepTC step clocks, rounding up
	static constexpr uint32_t MicrosecondsToStepTCClocks(float us)
	{
		return (uint32_t)((StepPulseClocksPerMicrosecond * us) + 0.99);
	}

#   else
	// All elements are in step clock units. The second element is the step low time.
	uint32_t slowDriverStepTimingClocks[4] = { 2, 2, 2, 2 };										// default to slower timing for external drivers (2 clocks = 2.67us)
#   endif
#  else
	uint32_t slowDriverStepTimingClocks[4] = { 0, 0, 0, 0 };										// default to fast timing
#  endif
#  if SINGLE_DRIVER
#   ifdef EXP1XD
	bool isSlowDriver = true;
#   else
	bool isSlowDriver = false;
#   endif
#  else
	DriversBitmap slowDriversBitmap;
#  endif
# endif

# if !SINGLE_DRIVER
	uint32_t driveDriverBits[NumDrivers];
	uint32_t allDriverBits = 0;
# endif

	static bool directions[NumDrivers];
	static bool driverAtIdleCurrent[NumDrivers];
	static int8_t enableValues[NumDrivers] = { 0 };
	IoPort brakePorts[NumDrivers];
	static DriverStateControl driverStates[NumDrivers];
#if SUPPORT_CLOSED_LOOP
	static bool driverEnableOverride[NumDrivers] = { 0 };
#endif
	static float stepsPerMm[NumDrivers];
	static float motorCurrents[NumDrivers];
	static float pressureAdvanceClocks[NumDrivers];
	static float idleCurrentFactor[NumDrivers];

	static void InternalEnableDrive(size_t driver);
	static void InternalDisableDrive(size_t driver);
	static void InternalSetDriverIdle(size_t driver);
#endif

#if SUPPORT_SPI_SENSORS || defined(ATEIO)
	SharedSpiDevice *sharedSpi = nullptr;
#endif

#if SUPPORT_I2C_SENSORS
	SharedI2CMaster *sharedI2C = nullptr;
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
	static unsigned int heatTaskIdleTicks = 0;

	static uint32_t whenLastCanMessageProcessed = 0;

#if SUPPORT_THERMISTORS
	static ThermistorAveragingFilter thermistorFilters[NumThermistorFilters];
#endif

#if HAS_VOLTAGE_MONITOR
	static AdcAveragingFilter<VinReadingsAveraged> vinFilter;
#endif
#if HAS_12V_MONITOR
	static AdcAveragingFilter<VinReadingsAveraged> v12Filter;
#endif

#if SAME5x
	static AdcAveragingFilter<McuTempReadingsAveraged> tpFilter;
	static AdcAveragingFilter<McuTempReadingsAveraged> tcFilter;
#elif SAMC21
	static AdcAveragingFilter<McuTempReadingsAveraged> tsensFilter;
#else
# error Unsupported processor
#endif

#if SUPPORT_DRIVERS
# if HAS_SMART_DRIVERS
	static DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers;
	static uint8_t nextDriveToPoll;
	static StandardDriverStatus lastEventStatus[NumDrivers];			// the status which we last reported as an event
	static MillisTimer openLoadTimers[NumDrivers];
# else
	static bool driverIsEnabled[NumDrivers] = { false };
# endif

# if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	bool warnDriversNotPowered;
# endif

# if HAS_STALL_DETECT
	DriversBitmap eventOnStallDrivers;
# endif
#endif

#if HAS_VOLTAGE_MONITOR

	inline constexpr float AdcReadingToVinVoltage(uint16_t adcVal)
	{
		return adcVal * (VinMonitorVoltageRange/(1u << AnalogIn::AdcBits));
	}

	inline constexpr uint16_t VinVoltageToAdcReading(float voltage)
	{
		return (uint16_t)(voltage * ((1u << AnalogIn::AdcBits)/VinMonitorVoltageRange));
	}

	constexpr uint16_t driverPowerOnAdcReading = VinVoltageToAdcReading(10.0);			// minimum voltage at which we initialise the drivers
	constexpr uint16_t driverPowerOffAdcReading = VinVoltageToAdcReading(9.5);			// voltages below this flag the drivers as unusable

#endif

#if HAS_12V_MONITOR

	inline constexpr float AdcReadingToV12Voltage(uint16_t adcVal)
	{
		return adcVal * (V12MonitorVoltageRange/(1u << AnalogIn::AdcBits));
	}

	inline constexpr uint16_t V12VoltageToAdcReading(float voltage)
	{
		return (uint16_t)(voltage * ((1u << AnalogIn::AdcBits)/V12MonitorVoltageRange));
	}

#endif

#if HAS_SMART_DRIVERS
	static void UpdateMotorCurrent(size_t driver)
	{
		SmartDrivers::SetCurrent(driver, (driverAtIdleCurrent[driver]) ? motorCurrents[driver] * idleCurrentFactor[driver] : motorCurrents[driver]);
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

#if SAME5x
	// Set a contiguous range of interrupts to the specified priority
	static void SetInterruptPriority(IRQn base, unsigned int num, uint32_t prio)
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

	static void InitialiseInterrupts()
	{
		// Note, I2C interrupt priority is set up in the I2C driver
		NVIC_SetPriority(StepTcIRQn, NvicPriorityStep);

#if SAME5x
# if defined(EXP3HC)
		NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
# elif defined(EXP1HCLv0_3) || defined(EXP1HCLv1_0)
		NVIC_SetPriority(CAN0_IRQn, NvicPriorityCan);
# endif
		// Set UART interrupt priority. Each SERCOM has up to 4 interrupts, numbered sequentially.
# if NUM_SERIAL_PORTS >= 1
		SetInterruptPriority(Serial0_IRQn, 4, NvicPriorityUart);
# endif
# if NUM_SERIAL_PORTS >= 2
		SetInterruptPriority(Serial1_IRQn, 4, NvicPriorityUart);
# endif
		SetInterruptPriority(DMAC_0_IRQn, 5, NvicPriorityDmac);
		SetInterruptPriority(EIC_0_IRQn, 16, NvicPriorityPins);
#elif SAMC21
		NVIC_SetPriority(CAN0_IRQn, NvicPriorityCan);
# if NUM_SERIAL_PORTS >= 1
		NVIC_SetPriority(Serial0_IRQn, NvicPriorityUart);
# endif
		NVIC_SetPriority(DMAC_IRQn, NvicPriorityDmac);
		NVIC_SetPriority(EIC_IRQn, NvicPriorityPins);
#else
# error Undefined processor
#endif
	}

	[[noreturn]] RAMFUNC static void EraseAndReset()
	{
#if SAME5x
		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Unlock the block of flash
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_UR | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Set address and command
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }
#elif SAMC21
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
#else
# error Unsupported processor
#endif
		ResetProcessor();
	}

	static void ShutdownAll()
	{
		Heat::SwitchOffAll();
#if SUPPORT_DRIVERS
# if SUPPORT_TMC51xx || SUPPORT_TMC2160
		IoPort::WriteDigital(GlobalTmc51xxEnablePin, true);
# endif
# if SUPPORT_TMC22xx
		IoPort::WriteDigital(GlobalTmc22xxEnablePin, true);
# endif
		DisableAllDrives();
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
#else
# error Unsupported processor
#endif

		EraseAndReset();
	}

	[[noreturn]] static void DoBootloadereUpdate()
	{
		ShutdownAll();									// turn everything off

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
		ResetProcessor();
	}

#if SUPPORT_THERMISTORS
	static void SetupThermistorFilter(Pin pin, size_t filterIndex, bool useAlternateAdc)
	{
		thermistorFilters[filterIndex].Init(0);
		IoPort::SetPinMode(pin, PinMode::AIN);
#if SAMC21
		const AdcInput adcChan = (useAlternateAdc) ? PinToSdAdcChannel(pin) : PinToAdcChannel(pin);
#else
		const AdcInput adcChan = PinToAdcChannel(pin);
#endif
		AnalogIn::EnableChannel(adcChan, thermistorFilters[filterIndex].CallbackFeedIntoFilter, CallbackParameter(&thermistorFilters[filterIndex]), 1, useAlternateAdc);
	}
#endif

	static CanAddress GetCanAddress() noexcept
	{
#if defined(EXP3HC)
		const CanAddress switches = ReadBoardAddress();
		return (switches == 0) ? CanId::ExpansionBoardFirmwareUpdateAddress : switches;
#elif defined(TOOL1LC)
		return CanId::ToolBoardDefaultAddress;
#elif defined(SAMMYC21)
		return CanId::SammyC21DefaultAddress;
#elif defined(EXP1XD)
		return CanId::Exp1XDBoardDefaultAddress;
#elif defined(EXP1HCE) || defined(EXP1HCLv0_3) || defined(EXP1HCLv1_0)
		return CanId::Exp1HCEBoardDefaultAddress;
#elif defined(ATECM)
		return CanId::ATECMBoardDefaultAddress;
#elif defined(ATEIO)
		return CanId::ATEIOBoardDefaultAddress;
#else
# 	 error Unknown board
#endif
	}

	static void InitVinMonitor()
	{
#if HAS_VOLTAGE_MONITOR
		currentVin = highestVin = 0;
		lowestVin = 65535;
		numUnderVoltageEvents = previousUnderVoltageEvents = numOverVoltageEvents = previousOverVoltageEvents = 0;

		vinFilter.Init(0);
		IoPort::SetPinMode(VinMonitorPin, AIN);
		AnalogIn::EnableChannel(PinToAdcChannel(VinMonitorPin), vinFilter.CallbackFeedIntoFilter, CallbackParameter(&vinFilter), 1, false);
#endif
	}

	static void InitLeds();
}	// end namespace Platform

// LED management
static void Platform::InitLeds()
{
	// Set up the LED pins
#ifdef TOOL1LC
	if (boardVariant == 1)
	{
		for (Pin pin : LedPinsV11)
		{
			IoPort::SetPinMode(pin, (LedActiveHighV11) ? OUTPUT_LOW : OUTPUT_HIGH);
		}
	}
	else
	{
		for (Pin pin : LedPinsV10)
		{
			IoPort::SetPinMode(pin, (LedActiveHighV10) ? OUTPUT_LOW : OUTPUT_HIGH);
		}
	}
#else
	for (Pin pin : LedPins)
	{
		IoPort::SetPinMode(pin, (LedActiveHigh) ? OUTPUT_LOW : OUTPUT_HIGH);
	}
#endif
	Platform::WriteLed(0, true);				// turn LED on for debugging
}

void Platform::WriteLed(uint8_t ledNumber, bool turnOn)
{
#ifdef TOOL1LC
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
#else
	if (ledNumber < ARRAY_SIZE(LedPins))
	{
		digitalWrite(LedPins[ledNumber], (LedActiveHigh) ? turnOn : !turnOn);
	}
#endif
}

// Initialisation
void Platform::Init()
{
	IoPort::Init();

#ifdef TOOL1LC
	// On the board detect pin:
	// Tool board V1.0 and earlier has 1K lower resistor, 10K upper, so will read as low
	// Tool board V1.1 has 10K lower resistor, 1K upper, so will read as high
	pinMode(BoardTypePin, INPUT);
	boardVariant = (digitalRead(BoardTypePin)) ? 1 : 0;
#endif

	InitLeds();

#if SUPPORT_CLOSED_LOOP
	ClosedLoop::Init();
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
				IoPort::SetHighDriveStrength(pin);
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
				IoPort::SetHighDriveStrength(pin);
				IoPort::SetPinMode(pin, OUTPUT_HIGH);
			}
#endif
		}
	}

#if defined(SAMMYC21)
	uart0.begin(115200);						// set up the UART with the same baud rate as the bootloader
#elif defined(DEBUG)
	// Set up the UART to send to PanelDue for debugging
	// CAUTION! This sends data to pin io0.out on a tool board, which interferes with a BLTouch connected to that pin. So don't do it in normal use.
	uart0.begin(57600);
#endif

	// Initialise the rest of the IO subsystem
#if SAME5x
	ADC_temperature_init();
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
	currentV12 = highestV12 = 0;
	lowestV12 = 65535;

	v12Filter.Init(0);
	IoPort::SetPinMode(V12MonitorPin, AIN);
	AnalogIn::EnableChannel(PinToAdcChannel(V12MonitorPin), v12Filter.CallbackFeedIntoFilter, CallbackParameter(&v12Filter), 1, false);
#endif

#if HAS_VREF_MONITOR
	// Set up the Vref and Vssa filters
	SetupThermistorFilter(VrefPin, VrefFilterIndex, false);
	SetupThermistorFilter(VssaPin, VssaFilterIndex, false);
#endif

#if SUPPORT_THERMISTORS
# if SAMC21 && SUPPORT_SDADC
	// Set up the SDADC input filters too (temp0 and Vref)
	SetupThermistorFilter(TempSensePins[0], SdAdcTemp0FilterIndex, true);
	SetupThermistorFilter(VrefPin, SdAdcVrefFilterIndex, true);
# endif

	// Set up the thermistor filters
	for (size_t i = 0; i < NumThermistorInputs; ++i)
	{
		SetupThermistorFilter(TempSensePins[i], i, false);
	}
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
#elif SAMC21
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

#if SUPPORT_DRIVERS
	// Initialise stepper drivers
# if HAS_SMART_DRIVERS
	SmartDrivers::Init();
	temperatureShutdownDrivers.Clear();
	temperatureWarningDrivers.Clear();
# endif

	for (size_t i = 0; i < NumDrivers; ++i)
	{
# if DIFFERENTIAL_STEPPER_OUTPUTS
		// Step pins
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
		IoPort::SetHighDriveStrength(StepPins[i]);
		IoPort::SetPinMode(InvertedStepPins[i], OUTPUT_HIGH);
		IoPort::SetHighDriveStrength(InvertedStepPins[i]);

		// Set up the CCL to invert the step output from PB10 to the inverted output on PA11
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;

#  if USE_TC_FOR_STEP
		// On the EXP1XD the step pin is also output TC1.0 and TC1 is not used for anything else
		// Use it to generate the step pulse
		EnableTcClock(StepGenTcNumber, GclkNum48MHz);

		hri_tc_set_CTRLA_SWRST_bit(StepGenTc);

		hri_tc_write_CTRLA_reg(StepGenTc, TC_CTRLA_MODE_COUNT16 | Platform::TcPrescalerRegVal);
		hri_tc_set_CTRLB_reg(StepGenTc, TC_CTRLBSET_ONESHOT);
		hri_tc_write_DBGCTRL_reg(StepGenTc, 0);
		hri_tc_write_EVCTRL_reg(StepGenTc, 0);
		hri_tc_write_WAVE_reg(StepGenTc, TC_WAVE_WAVEGEN_MPWM);

		StepGenTc->CC[0].reg = (uint16_t)Platform::slowDriverStepTimingClocks[0];
		StepGenTc->CCBUF[0].reg = (uint16_t)Platform::slowDriverStepTimingClocks[0];

		hri_tc_set_CTRLA_ENABLE_bit(StepGenTc);

		delayMicroseconds(2);												// this avoids a glitch on the step output

		SetPinFunction(StepPins[i], GpioPinFunction::E);					// TC1.0
#  else
		SetPinFunction(StepPins[i], GpioPinFunction::I);					// CCL1in5
#  endif
		CCL->CTRL.reg = 0;													// disable the CCL
		CCL->SEQCTRL[0].reg = CCL_SEQCTRL_SEQSEL_DISABLE;
		CCL->LUTCTRL[1].reg &= ~CCL_LUTCTRL_ENABLE;
		CCL->LUTCTRL[1].reg =
#  if USE_TC_FOR_STEP
			  	  	  	  CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL0_TC_Val)		// take input from TC1.0
#  else
						  CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL0_IO_Val)		// take input from CCL1 IN2
#  endif
						| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_MASK_Val)
						| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_MASK_Val)
						| CCL_LUTCTRL_TRUTH(0b00001111);
		CCL->LUTCTRL[1].reg |= CCL_LUTCTRL_ENABLE;
		CCL->CTRL.reg = CCL_CTRL_ENABLE;
		SetPinFunction(InvertedStepPins[i], GpioPinFunction::I);			// CCL1out1 do this at the end to avoid a glitch on the output

		// Direction pins
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
		IoPort::SetHighDriveStrength(DirectionPins[i]);
		IoPort::SetPinMode(InvertedDirectionPins[i], OUTPUT_HIGH);
		IoPort::SetHighDriveStrength(InvertedDirectionPins[i]);

		// Enable pins
		IoPort::SetPinMode(EnablePins[i], OUTPUT_LOW);
		IoPort::SetHighDriveStrength(EnablePins[i]);
		IoPort::SetPinMode(InvertedEnablePins[i], OUTPUT_HIGH);
		IoPort::SetHighDriveStrength(InvertedEnablePins[i]);
		enableValues[i] = 1;
		driverIsEnabled[i] = false;
# else
		// Step pins
#  if ACTIVE_HIGH_STEP
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
#  else
		IoPort::SetPinMode(StepPins[i], OUTPUT_HIGH);
#  endif
#  if !HAS_SMART_DRIVERS
		IoPort::SetHighDriveStrength(StepPins[i]);
#  endif

		// Direction pins
#  if ACTIVE_HIGH_DIR
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
#  else
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_HIGH);
#  endif
#  if !HAS_SMART_DRIVERS
		IoPort::SetHighDriveStrength(DirectionPins[i]);
#  endif

#  if !HAS_SMART_DRIVERS
		// Enable pins
#   if ACTIVE_HIGH_ENABLE
		IoPort::SetPinMode(EnablePins[i], OUTPUT_LOW);
		enableValues[i] = 1;
#   else
		IoPort::SetPinMode(EnablePins[i], OUTPUT_HIGH);
		enableValues[i] = 0;
#   endif
		IoPort::SetHighDriveStrength(EnablePins[i]);
		driverIsEnabled[i] = false;
#  endif
# endif

# if !SINGLE_DRIVER
		const uint32_t driverBit = 1u << (StepPins[i] & 31);
		driveDriverBits[i] = driverBit;
		allDriverBits |= driverBit;
# endif
		stepsPerMm[i] = DefaultStepsPerMm;
		directions[i] = true;
		driverAtIdleCurrent[i] = false;
		idleCurrentFactor[i] = 0.3;
		motorCurrents[i] = 0.0;
		pressureAdvanceClocks[i] = 0.0;
		driverStates[i] = DriverStateControl(DriverStateControl::driverDisabled);
		// We can't set microstepping here because moveInstance hasn't been created yet
	}

# if HAS_STALL_DETECT
	eventOnStallDrivers.Clear();
#endif

# if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	warnDriversNotPowered = false;
# endif
#endif	//SUPPORT_DRIVERS

#if SUPPORT_SPI_SENSORS || defined(ATEIO)
	// Set the pin functions
	SetPinFunction(SSPIMosiPin, SSPIMosiPinPeriphMode);
	SetPinFunction(SSPISclkPin, SSPISclkPinPeriphMode);
	SetPinFunction(SSPIMisoPin, SSPIMisoPinPeriphMode);
	sharedSpi = new SharedSpiDevice(SspiSercomNumber, SspiDataInPad);
#endif

#if SUPPORT_I2C_SENSORS
# ifdef TOOL1LC
	if (boardVariant != 0)
# endif
	{
		SetPinFunction(I2CSDAPin, I2CSDAPinPeriphMode);
		SetPinFunction(I2CSCLPin, I2CSCLPinPeriphMode);
		sharedI2C = new SharedI2CMaster(I2CSercomNumber);
	}
#endif

#ifdef ATEIO
	ExtendedAnalog::Init(*sharedSpi);					// must init sharedSpi before calling this
#endif

	uniqueId.SetFromCurrentBoard();

	InitialiseInterrupts();

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH
# ifdef TOOL1LC
	if (boardVariant != 0)
# endif
	{
		AccelerometerHandler::Init();
	}
#endif

	CanInterface::Init(GetCanAddress(), UseAlternateCanPins, true);
	lastPollTime = millis();
}

// Perform minimal initialisation prior to updating the bootloader
void Platform::InitMinimal()
{
	InitLeds();
	InitVinMonitor();
	InitialiseInterrupts();
	CanInterface::Init(GetCanAddress(), UseAlternateCanPins, false);
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
			(void)Tasks::DoMemoryRead(reinterpret_cast<const uint32_t*>(HSRAM_ADDR + 1));
			__ISB();
			deliberateError = false;
			break;

		case DeferredCommand::testBadMemoryAccess:
			deliberateError = true;
			(void)Tasks::DoMemoryRead(reinterpret_cast<const uint32_t*>(0x30000000));	// 0x30000000 is invalid on both the SAME5x and the SAMC21
			__ISB();
			deliberateError = false;
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
		if (currentV12 < lowestV12)
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

#if HAS_SMART_DRIVERS
	SmartDrivers::Spin(powered);

	// Check one TMC driver for warnings and errors
	if (enableValues[nextDriveToPoll] >= 0)				// don't poll driver if it is flagged "no poll"
	{
		StandardDriverStatus stat = SmartDrivers::GetStatus(nextDriveToPoll, true, true);
		const DriversBitmap mask = DriversBitmap::MakeFromBits(nextDriveToPoll);

		// First set the driver temperature status for the temperature sensor code
		if (stat.ot)
		{
			temperatureShutdownDrivers |= mask;
		}
		else
		{
			if (stat.otpw)
			{
				temperatureWarningDrivers |= mask;
			}
			else
			{
				temperatureWarningDrivers &= ~mask;
			}
			temperatureShutdownDrivers &= ~mask;
		}

		// Deal with the open load bits
		// The driver often produces a transient open-load error, especially in stealthchop mode, so we require the condition to persist before we report it.
		// So clear them unless they have been active for the minimum time.
		MillisTimer& timer = openLoadTimers[nextDriveToPoll];
		if (stat.IsAnyOpenLoadBitSet())
		{
			if (timer.IsRunning())
			{
				if (!timer.Check(OpenLoadTimeout))
				{
					stat.ClearOpenLoadBits();
				}
			}
			else
			{
				timer.Start();
				stat.ClearOpenLoadBits();
			}
		}
		else
		{
			timer.Stop();
		}

		const StandardDriverStatus oldStatus = lastEventStatus[nextDriveToPoll];
		lastEventStatus[nextDriveToPoll] = stat;
		if (stat.HasNewErrorSince(oldStatus))
		{
			// It's a new error
			CanInterface::RaiseEvent(EventType::driver_error, stat.AsU16(), nextDriveToPoll, "", va_list());
		}
		else if (stat.HasNewWarningSince(oldStatus))
		{
			// It's a new warning
			CanInterface::RaiseEvent(EventType::driver_warning, stat.AsU16(), nextDriveToPoll, "", va_list());
		}

# if HAS_STALL_DETECT
		if (stat.HasNewStallSince(oldStatus) && eventOnStallDrivers.Intersects(mask))
		{
			CanInterface::RaiseEvent(EventType::driver_stall, 0, nextDriveToPoll, "", va_list());
		}
# endif
	}

	// Advance drive number ready for next time
	++nextDriveToPoll;
	if (nextDriveToPoll == MaxSmartDrivers)
	{
		nextDriveToPoll = 0;
	}
#endif

	// Thermostatically-controlled fans (do this after getting TMC driver status)
	const uint32_t now = millis();
	const bool checkSensors = (now - lastFanCheckTime >= FanCheckInterval);
	(void)FansManager::CheckFans(checkSensors);
	if (checkSensors)
	{
		lastFanCheckTime = now;
	}

	// Update the Status LED. Flash it quickly (8Hz) if we are not synced to the master, else flash in sync with the master (about 2Hz).
	WriteLed(0,
				(StepTimer::IsSynced())
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

#ifdef SAMMYC21
		//debugPrintf("IR=%" PRIx32 " ERR=%" PRIx32 " RXF0S=%" PRIx32 " RXF1S=%" PRIx32 " PSR=%" PRIx32 " CCCR=%" PRIx32 "\n",
		//	CAN0->IR.reg, CAN0->ECR.reg, CAN0->RXF0S.reg, CAN0->RXF1S.reg, CAN0->PSR.reg, CAN0->CCCR.reg);

		// If D is received from the USB port, output some diagnostics
		while (uart0.available() != 0)
		{
			const char c = uart0.read();
			if (c == 'D')
			{
				String<StringLength256> reply;
				Tasks::Diagnostics(reply.GetRef());
				debugPrintf("%s\n", reply.c_str());
				reply.Clear();
				CanInterface::Diagnostics(reply.GetRef());
				debugPrintf("%s\n", reply.c_str());
				reply.Clear();
				moveInstance->Diagnostics(reply.GetRef());
				debugPrintf("%s\n", reply.c_str());
				//moveInstance->DebugPrintCdda();
#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH
				debugPrintf("LIS3DH detected: %s", AccelerometerHandler::IsPresent() ? "yes" : "no");
#endif
			}
		}
#endif
	}
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
		if (currentVin < lowestVin)
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

void Platform::HandleHeaterFault(unsigned int heater)
{
	//TODO report the heater fault to the main board
}

// Output a character to the debug channel
bool Platform::DebugPutc(char c)
{
#if defined(SAMMYC21) || defined(DEBUG)
	if (c != 0)
	{
		uart0.write(c);
	}
#endif
	return true;
}

void Platform::LogError(ErrorCode e)
{
	errorCodeBits |= (uint32_t)e;
}

bool Platform::Debug(Module module)
{
#if 0
	return module == moduleMove;	//DEBUG
#else
	return false;
#endif
}

#if SUPPORT_DRIVERS

float Platform::DriveStepsPerUnit(size_t drive) { return stepsPerMm[drive]; }

void Platform::SetDriveStepsPerUnit(size_t drive, float val)
{
	if (drive < NumDrivers)
	{
		stepsPerMm[drive] = val;
	}
}

const float *Platform::GetDriveStepsPerUnit() { return stepsPerMm; }

# if SUPPORT_SLOW_DRIVERS

float Platform::GetSlowDriverStepHighMicroseconds()
{
#  if USE_TC_FOR_STEP
	return (float)slowDriverStepTimingClocks[0]/StepPulseClocksPerMicrosecond;
#  else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[0]);
#  endif
}

float Platform::GetSlowDriverStepLowMicroseconds()
{
#  if USE_TC_FOR_STEP
	const float period = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
	return period - GetSlowDriverStepHighMicroseconds();
#  else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
#  endif
}

float Platform::GetSlowDriverDirSetupMicroseconds()
{
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[2]);
}

float Platform::GetSlowDriverDirHoldMicroseconds()
{
#  if USE_TC_FOR_STEP
	const float dirHoldFromLeadingEdge = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
	return dirHoldFromLeadingEdge - GetSlowDriverStepHighMicroseconds();
#  else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
#  endif
}

// Convert microseconds to step clocks, rounding up
static uint32_t MicrosecondsToStepClocks(float us)
{
	return (uint32_t)(((float)StepTimer::StepClockRate * us * 0.000001) + 0.99);
}

inline void UpdateTiming(uint32_t& timing, uint32_t clocks)
{
#  if SINGLE_DRIVER
		timing = clocks;
#  else
		if (clocks > timing)
		{
			timing = clocks;
		}
#  endif

}

void Platform::SetDriverStepTiming(size_t drive, const float timings[4])
{
	bool isSlow = false;

#  if USE_TC_FOR_STEP

	// Step high time - must do this one first because it affects the conversion of some of the others
	if (timings[0] > MinimumStepHighMicroseconds)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[0], MicrosecondsToStepTCClocks(timings[0]));
	}
#   if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[0] = MicrosecondsToStepTCClocks(MinimumStepHighMicroseconds);
	}
#   endif

	// To get the new width to be applied to the step pulse, we need to update CCBUF[0] and then push it to CC[0]. Writing CC[0] directly doesn't work.
	StepGenTc->CCBUF[0].reg = (uint16_t)Platform::slowDriverStepTimingClocks[0];
	StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_UPDATE;

	// Step low time - must convert this to minimum period
	const float minimumPeriod = timings[1] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (minimumPeriod > 0.4)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[1], MicrosecondsToStepClocks(minimumPeriod));
	}
#   if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[1] = 1;
	}
#   endif

	// Direction setup time - we can just convert this one
	if (timings[2] > 0.2)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[2], MicrosecondsToStepClocks(timings[2]));
	}
#   if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[2] = 0;
	}
#   endif

	// Direction hold time - we need to convert hold time from trailing edge to hold time from leading edge
	const float holdTimeFromLeadingEdge = timings[3] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (holdTimeFromLeadingEdge > 0.4)
	{
		isSlow = true;
		const uint32_t clocks = MicrosecondsToStepClocks(holdTimeFromLeadingEdge);
		UpdateTiming(slowDriverStepTimingClocks[3], clocks);
	}
#   if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[3] = 1;
	}
#   endif

#  else

	// Not using TC to generate step pulses
	for (size_t i = 0; i < 4; ++i)
	{
		if (timings[i] > 0.2)
		{
			isSlow = true;
			const uint32_t clocks = MicrosecondsToStepClocks(timings[i]);
			UpdateTiming(slowDriverStepTimingClocks[i], clocks);
		}
#   if SINGLE_DRIVER		// we can clear the value if we have only one driver
		else
		{
			slowDriverStepTimingClocks[i] = 0;
		}
#   endif
	}

#  endif	// USE_TC_FOR_STEP

#  if SINGLE_DRIVER
	isSlowDriver = isSlow;
#  else
	slowDriversBitmap.SetOrClearBit(drive, isSlow);
#  endif
}

# endif		// SUPPORT_SLOW_DRIVERS

float Platform::GetPressureAdvanceClocks(size_t driver)
{
	return pressureAdvanceClocks[driver];
}

void Platform::SetPressureAdvance(size_t driver, float advance)
{
	pressureAdvanceClocks[driver] = advance * (float)StepTimer::StepClockRate;
}

#if 0	// not used yet and may never be
// Send the status of drivers and filament monitors to the main board
void Platform::BuildDriverStatusMessage(CanMessageBuffer *buf) noexcept
{
	auto msg = buf->SetupStatusMessage<CanMessageDriversStatus>(CanInterface::GetCanAddress(), CanId::MasterAddress);
	for (size_t drive = 0; drive < NumDrivers; ++drive)
	{
		const uint32_t driverStat =
#if HAS_SMART_DRIVERS
			SmartDrivers::GetLiveStatus(drive);
#else
			0;
#endif
		msg->data[drive] = driverStat;
	}
	msg->numDriversReported = NumDrivers;
	msg->zero = 0;
	buf->dataLength = msg->GetActualDataLength();
}
#endif

void Platform::SetDirectionValue(size_t drive, bool dVal)
{
	if (drive < NumDrivers)
	{
		directions[drive] = dVal;
	}
}

bool Platform::GetDirectionValue(size_t driver)
{
	return (driver < NumDrivers) && directions[driver];
}

#if SINGLE_DRIVER

void Platform::SetDirection(bool direction)
{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_DIR
	// Active high direction signal
	const bool d = (direction) ? directions[0] : !directions[0];
# else
	// Active low direction signal
	const bool d = (direction) ? !directions[0] : directions[0];
# endif

# if SUPPORT_CLOSED_LOOP
	ClosedLoop::SetStepDirection(d);
	if (ClosedLoop::GetClosedLoopEnabled())
	{
		return;
	}
# endif
# if SUPPORT_SLOW_DRIVERS
	if (isSlowDriver)
	{
#  if USE_TC_FOR_STEP
		while (StepTimer::GetTimerTicks() - DDA::lastStepHighTime < GetSlowDriverDirHoldFromLeadingEdgeClocks()) { }
#  else
		while (StepTimer::GetTimerTicks() - DDA::lastStepLowTime < GetSlowDriverDirHoldFromTrailingEdgeClocks()) { }
#  endif
	}
# endif
	digitalWrite(DirectionPins[0], d);
# if DIFFERENTIAL_STEPPER_OUTPUTS
	digitalWrite(InvertedDirectionPins[0], !d);
# endif
# if SUPPORT_SLOW_DRIVERS
	if (isSlowDriver)
	{
		DDA::lastDirChangeTime = StepTimer::GetTimerTicks();
	}
# endif
}

#else

void Platform::SetDirection(size_t driver, bool direction)
{
	if (driver < NumDrivers)
	{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_DIR
		// Active high direction signal
		const bool d = (direction) ? directions[driver] : !directions[driver];
# else
		// Active low direction signal
		const bool d = (direction) ? !directions[driver] : directions[driver];
# endif

# if SUPPORT_SLOW_DRIVERS
		const bool isSlowDriver = slowDriversBitmap.IsBitSet(driver);
		if (isSlowDriver)
		{
			while (StepTimer::GetTimerTicks() - DDA::lastStepLowTime < GetSlowDriverDirHoldClocks()) { }
		}
# endif
		digitalWrite(DirectionPins[driver], d);
# if DIFFERENTIAL_STEPPER_OUTPUTS
		digitalWrite(InvertedDirectionPins[driver], !d);
# endif
# if SUPPORT_SLOW_DRIVERS
		if (isSlowDriver)
		{
			DDA::lastDirChangeTime = StepTimer::GetTimerTicks();
		}
# endif
	}
}

#endif

// The following don't do anything yet
void Platform::SetEnableValue(size_t driver, int8_t eVal)
{
	if (driver < NumDrivers)
	{
		enableValues[driver] = eVal;
# if !HAS_SMART_DRIVERS
		if (driverIsEnabled[driver])
		{
			EnableDrive(driver);
		}
		else
		{
			DisableDrive(driver);
		}
# endif
	}
}

int8_t Platform::GetEnableValue(size_t driver)
{
	return (driver < NumDrivers) ? enableValues[driver] : 0;
}

#if SUPPORT_CLOSED_LOOP

void Platform::DriveEnableOverride(size_t driver, bool doOverride)
{
	if (doOverride)
	{
		driverEnableOverride[driver] = true;
		InternalEnableDrive(driver);
	}
	else if (driverEnableOverride[driver])
	{
		driverEnableOverride[driver] = false;				// do this before calling InternalSetDriverIdle
		switch (driverStates[driver].mode)
		{
		case DriverStateControl::driverActive:
			break;											// driver is already enabled

		case DriverStateControl::driverIdle:
			InternalSetDriverIdle(driver);
			break;

		case DriverStateControl::driverDisabled:
		default:
			InternalDisableDrive(driver);
			break;
		}
	}
}

#endif

void Platform::EnableDrive(size_t driver)
{
	driverStates[driver] = DriverStateControl(DriverStateControl::driverActive);
#if SUPPORT_CLOSED_LOOP
	if (!driverEnableOverride[driver])						// if the override flag is set, the driver is already enabled
#endif
	{
		InternalEnableDrive(driver);
	}
}

void Platform::InternalEnableDrive(size_t driver)
{
# if HAS_SMART_DRIVERS
	if (driverAtIdleCurrent[driver])
	{
		driverAtIdleCurrent[driver] = false;
#  if SUPPORT_CLOSED_LOOP
		ClosedLoop::ResetError(driver);
#  endif
		UpdateMotorCurrent(driver);
	}
	SmartDrivers::EnableDrive(driver, true);
# else
	if (enableValues[driver] >= 0)
	{
		digitalWrite(EnablePins[driver], enableValues[driver] != 0);
#  if DIFFERENTIAL_STEPPER_OUTPUTS
		digitalWrite(InvertedEnablePins[driver], enableValues[driver] == 0);
#  endif
	}
# endif
	brakePorts[driver].WriteDigital(true);					// energise the brake solenoid to release the brake
}

void Platform::DisableDrive(size_t driver)
{
	driverStates[driver] = DriverStateControl::driverDisabled;
#if SUPPORT_CLOSED_LOOP
	if (!driverEnableOverride[driver])						// if the override flag is set, don't disable the driver, just flag is as disabled
#endif
	{
		InternalDisableDrive(driver);
	}
}

void Platform::InternalDisableDrive(size_t driver)
{
	brakePorts[driver].WriteDigital(false);					// de-energise the brake solenoid to apply the brake
# if HAS_SMART_DRIVERS
	SmartDrivers::EnableDrive(driver, false);
# else
	if (enableValues[driver] >= 0)
	{
		digitalWrite(EnablePins[driver], enableValues[driver] == 0);
#  if DIFFERENTIAL_STEPPER_OUTPUTS
		digitalWrite(InvertedEnablePins[driver], enableValues[driver] != 0);
#  endif
	}
# endif
}

void Platform::SetDriverIdle(size_t driver, uint16_t idlePercent)
{
	idleCurrentFactor[driver] = (float)idlePercent * 0.01;
	driverStates[driver] = DriverStateControl::driverIdle;
#if SUPPORT_CLOSED_LOOP
	if (!driverEnableOverride)
#endif
	{
		InternalSetDriverIdle(driver);
	}
}

void Platform::InternalSetDriverIdle(size_t driver)
{
	if (idleCurrentFactor[driver] == 0.0)
	{
		InternalDisableDrive(driver);
	}
# if HAS_SMART_DRIVERS
	else
	{
		driverAtIdleCurrent[driver] = true;
		UpdateMotorCurrent(driver);
	}
# endif
}

void Platform::DisableAllDrives()
{
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
# if HAS_SMART_DRIVERS
		SmartDrivers::EnableDrive(driver, false);
# else
		DisableDrive(driver);
# endif
	}
}

GCodeResult Platform::ProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M569Point7Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	if (parser.HasParameter('C'))
	{
		String<StringLength20> portName;
		parser.GetStringParam('C', portName.GetRef());
		return GetGCodeResultFromSuccess(brakePorts[drive].AssignPort(portName.c_str(), reply, PinUsedBy::gpout, PinAccess::write0));
	}

	reply.printf("Driver %u.%u uses brake port ", CanInterface::GetCanAddress(), drive);
	brakePorts[drive].AppendPinName(reply);
	return GCodeResult::ok;
}

# if HAS_SMART_DRIVERS

void Platform::SetMotorCurrent(size_t driver, float current)
{
	motorCurrents[driver] = current;
	UpdateMotorCurrent(driver);
}

// TMC driver temperatures
float Platform::GetTmcDriversTemperature()
{
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(MaxSmartDrivers);
	return (temperatureShutdownDrivers.Intersects(mask)) ? 150.0
			: (temperatureWarningDrivers.Intersects(mask)) ? 100.0
				: 0.0;
}

#  if HAS_STALL_DETECT

void Platform::SetOrResetEventOnStall(DriversBitmap drivers, bool enable) noexcept
{
	if (enable)
	{
		eventOnStallDrivers |= drivers;
	}
	else
	{
		eventOnStallDrivers &= ~drivers;
	}
}

#  endif

# else

StandardDriverStatus Platform::GetStandardDriverStatus(size_t driver)
{
	//TODO implement alarm input
	StandardDriverStatus rslt;
	rslt.all = 0;
	return rslt;
}

# endif

#endif	//SUPPORT_DRIVERS

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

#if SUPPORT_DRIVERS

// Function to broadcast the drivers status message. Called only by the Heat task.
void Platform::SendDriversStatus(CanMessageBuffer& buf)
{
	CanMessageDriversStatus * const msg = buf.SetupStatusMessage<CanMessageDriversStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
# if HAS_SMART_DRIVERS
	msg->SetStandardFields(MaxSmartDrivers);
	for (size_t driver = 0; driver < MaxSmartDrivers; ++driver)
	{
		msg->data[driver] = SmartDrivers::GetStatus(driver).AsU32();
	}
# else
	msg->SetStandardFields(NumDrivers);
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		msg->data[driver] = Platform::GetStandardDriverStatus(driver).AsU32();
	}
# endif
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
}

#endif

void Platform::Tick() noexcept
{
	++heatTaskIdleTicks;
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

// Execute a timed task that takes less than one millisecond
static uint32_t TimedSqrt(uint64_t arg, uint32_t& timeAcc) noexcept
{
	IrqDisable();
	asm volatile("":::"memory");
	uint32_t now1 = SysTick->VAL;
	const uint32_t ret = isqrt64(arg);
	uint32_t now2 = SysTick->VAL;
	asm volatile("":::"memory");
	IrqEnable();
	now1 &= 0x00FFFFFF;
	now2 &= 0x00FFFFFF;
	timeAcc += ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
	return ret;
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
			bool ok1 = true;
			uint32_t tim1 = 0;
			constexpr uint32_t iterations = 100;				// use a value that divides into one million
			for (uint32_t i = 0; i < iterations; ++i)
			{
				const uint32_t num1 = 0x7fffffff - (67 * i);
				const uint64_t sq = (uint64_t)num1 * num1;
				const uint32_t num1a = TimedSqrt(sq, tim1);
				if (num1a != num1)
				{
					ok1 = false;
				}
			}

			bool ok2 = true;
			uint32_t tim2 = 0;
			for (uint32_t i = 0; i < iterations; ++i)
			{
				const uint32_t num2 = 0x0000ffff - (67 * i);
				const uint64_t sq = (uint64_t)num2 * num2;
				const uint32_t num2a = TimedSqrt(sq, tim2);
				if (num2a != num2)
				{
					ok2 = false;
				}
			}

			bool ok3 = true;
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
					ok3 = false;
				}
				val = nval;
			}
			reply.printf("Square roots: 62-bit %.2fus %s, 32-bit %.2fus %s, float %.2fus %s",
						(double)((float)(tim1 * (1'000'000/iterations))/SystemCoreClockFreq), (ok1) ? "ok" : "ERROR",
							(double)((float)(tim2 * (1'000'000/iterations))/SystemCoreClockFreq), (ok2) ? "ok" : "ERROR",
								(double)((float)(tim3 * (1'000'000/iterations))/SystemCoreClock), (ok3) ? "ok" : "ERROR");
			return (ok1 && ok2 && ok3) ? GCodeResult::ok : GCodeResult::error;
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
				(void)StepTimer::GetTimerTicks();
			} while (i != 0);
			uint32_t now2 = SysTick->VAL;
			asm volatile("":::"memory");
			IrqEnable();
			now1 &= 0x00FFFFFF;
			now2 &= 0x00FFFFFF;
			uint32_t tim1 = ((now1 > now2) ? now1 : now1 + (SysTick->LOAD & 0x00FFFFFF) + 1) - now2;
			reply.printf("Reading step timer 100 times took %.2fus", (double)((1'000'000.0f * (float)tim1)/(float)SystemCoreClock));
		}

		// Also check the correspondence between the CAN timestamp timer and the step clock
		{
			uint32_t startClocks, endClocks;
			uint16_t startTimeStamp, endTimeStamp;
			{
				AtomicCriticalSectionLocker lock;
				startClocks = StepTimer::GetTimerTicks();
				startTimeStamp = CanInterface::GetTimeStampCounter();
			}
			delay(2);
			{
				AtomicCriticalSectionLocker lock;
				endClocks = StepTimer::GetTimerTicks();
				endTimeStamp = CanInterface::GetTimeStampCounter();
			}
			const uint32_t tsDiff = (((endTimeStamp - startTimeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;
			reply.lcatf("Clock diff %" PRIu32 ", ts diff %" PRIu32, endClocks - startClocks, tsDiff);
		}
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
			*reinterpret_cast<uint32_t*>(msg.param32[0]) = msg.param32[1];
		}
		else						// reading
		{
			reply.printf("Address 0x%08" PRIx32 " value 0x%08" PRIx32, msg.param32[0], *reinterpret_cast<const uint32_t*>(msg.param32[0]));
		}
		deliberateError = false;
		return GCodeResult::ok;

	default:
		reply.printf("Unknown test type %u", msg.testType);
		return GCodeResult::error;
	}
}

uint32_t Platform::GetDateTime() noexcept
{
	return realTime;
}

void Platform::SetDateTime(uint32_t tim) noexcept
{
	realTime = tim;
}

bool Platform::WasDeliberateError() noexcept
{
	return deliberateError;
}

#if HAS_VOLTAGE_MONITOR

MinCurMax Platform::GetPowerVoltages(bool resetMinMax) noexcept
{
	MinCurMax result;
	result.minimum = AdcReadingToVinVoltage(lowestVin);
	result.current = AdcReadingToVinVoltage(currentVin);
	result.maximum = AdcReadingToVinVoltage(highestVin);
	if (resetMinMax)
	{
		lowestVin = highestVin = currentVin;
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
		lowestV12 = highestV12 = currentV12;
	}
	return result;
}

float Platform::GetCurrentV12Voltage() noexcept
{
	return AdcReadingToV12Voltage(currentV12);
}

#endif

void Platform::AppendBoardAndFirmwareDetails(const StringRef& reply) noexcept
{
#ifdef TOOL1LC
	reply.lcatf("Duet " BOARD_TYPE_NAME " rev %s firmware version " VERSION " (%s%s)",
				(boardVariant == 1) ? "1.1 or later" : "1.0 or earlier",
				IsoDate, TIME_SUFFIX);
#else
	reply.lcatf("Duet " BOARD_TYPE_NAME " firmware version " VERSION " (%s%s)",
				IsoDate, TIME_SUFFIX);
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

#ifdef TOOL1LC

uint8_t Platform::GetBoardVariant() noexcept
{
	return boardVariant;
}

#endif

// Interrupt handlers
#if SUPPORT_I2C_SENSORS

# if SAMC21
#  ifndef I2C_HANDLER
#   error "I2C_HANDLER not defined"
#  endif
	void I2C_HANDLER() noexcept
	{
		Platform::sharedI2C->Interrupt();
	}
# elif SAME5x
#  if !defined(I2C_HANDLER0) || !defined(I2C_HANDLER1) || !defined(I2C_HANDLER3)
#   error "I2C_HANDLER0, 1 or 3 not defined"
#  endif
	void I2C_HANDLER0() noexcept
	{
		Platform::sharedI2C->Interrupt();
	}

	void I2C_HANDLER1() noexcept
	{
		Platform::sharedI2C->Interrupt();
	}

	void I2C_HANDLER3() noexcept
	{
		Platform::sharedI2C->Interrupt();
	}
# endif

#endif

// End
