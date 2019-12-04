/*
 * Platform.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "Platform.h"

#include <Hardware/IoPorts.h>
#include <Hardware/AnalogIn.h>
#include <Hardware/AnalogOut.h>
#include <Hardware/Serial.h>
#include <Movement/Move.h>
#include "Movement/StepperDrivers/TMC51xx.h"
#include "Movement/StepperDrivers/TMC22xx.h"
#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>
#include "AdcAveragingFilter.h"
#include "Movement/StepTimer.h"
#include <CAN/CanInterface.h>
#include "Tasks.h"
#include "Heating/Heat.h"
#include "Heating/Sensors/TemperatureSensor.h"
#include "Fans/FansManager.h"

#if defined(SAME51)

# include <hri_nvmctrl_e51.h>
constexpr uint32_t FlashBlockSize = 0x00010000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 64K for the bootloader

static Uart uart0(3, SERCOM3_0_IRQn);

extern "C" void SERCOM3_0_Handler()
{
	uart0.Interrupt();
}

extern "C" void SERCOM3_1_Handler()
{
	uart0.Interrupt();
}

extern "C" void SERCOM3_2_Handler()
{
	uart0.Interrupt();
}

extern "C" void SERCOM3_3_Handler()
{
	uart0.Interrupt();
}

#elif defined(SAMC21)

# include <hri_nvmctrl_c21.h>
constexpr uint32_t FlashBlockSize = 0x00004000;							// the block size we assume for flash
constexpr uint32_t FirmwareFlashStart = FLASH_ADDR + FlashBlockSize;	// we reserve 16K for the bootloader

static Uart uart0(4, SERCOM4_IRQn);

extern "C" void SERCOM4_Handler()
{
	uart0.Interrupt();
}

#else
# error Unsupported processor
#endif

static bool doFirmwareUpdate = false;

namespace Platform
{
	static constexpr float DefaultStepsPerMm = 80.0;

	Mutex messageMutex;

	static uint32_t errorCodeBits = 0;
	uint32_t slowDriversBitmap = 0;
	uint32_t slowDriverStepTimingClocks[4] = { 0, 0, 0, 0 };

	uint32_t uniqueId[5];

	uint32_t driveDriverBits[NumDrivers];
	uint32_t allDriverBits = 0;

	static bool directions[NumDrivers];
	static bool driverAtIdleCurrent[NumDrivers];
	static int8_t enableValues[NumDrivers] = { 0 };
#if !HAS_SMART_DRIVERS
	static bool driverIsEnabled[NumDrivers] = { false };
#endif
	static float stepsPerMm[NumDrivers];
	static float motorCurrents[NumDrivers];
	static float pressureAdvance[NumDrivers];
	static float idleCurrentFactor;

	static volatile uint16_t currentVin, highestVin, lowestVin;
#if HAS_12V_MONITOR
	static volatile uint16_t currentV12, highestV12, lowestV12;
#endif

//	static uint16_t lastUnderVoltageValue, lastOverVoltageValue;
	static uint32_t numUnderVoltageEvents, previousUnderVoltageEvents;
	static volatile uint32_t numOverVoltageEvents, previousOverVoltageEvents;

	static float currentMcuTemperature, highestMcuTemperature, lowestMcuTemperature;
	static float mcuTemperatureAdjust = 0.0;

	static uint32_t lastPollTime;
	static uint32_t lastFanCheckTime = 0;
	static unsigned int heatTaskIdleTicks = 0;

	// SERCOM3 Rx is on PB21 (OUT_8_TACHO), Tx is on PB20 (OUT_7_TACHO)

	static ThermistorAveragingFilter thermistorFilters[NumThermistorFilters];
	static AdcAveragingFilter<VinReadingsAveraged> vinFilter;
#if HAS_12V_MONITOR
	static AdcAveragingFilter<VinReadingsAveraged> v12Filter;
#endif

#if defined(SAME51)
	static AdcAveragingFilter<McuTempReadingsAveraged> tpFilter;
	static AdcAveragingFilter<McuTempReadingsAveraged> tcFilter;
#elif defined(SAMC21)
	static AdcAveragingFilter<McuTempReadingsAveraged> tsensFilter;
#else
# error Unsupported processor
#endif

#if HAS_SMART_DRIVERS
	static DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers, shortToGroundDrivers;
	static DriversBitmap openLoadADrivers, openLoadBDrivers, notOpenLoadADrivers, notOpenLoadBDrivers;
	MillisTimer openLoadATimer, openLoadBTimer;
	MillisTimer driversFanTimer;		// driver cooling fan timer
	static uint8_t nextDriveToPoll;
#endif

#if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	bool warnDriversNotPowered;
#endif

#if HAS_STALL_DETECT
	DriversBitmap logOnStallDrivers, pauseOnStallDrivers, rehomeOnStallDrivers;
	DriversBitmap stalledDrivers, stalledDriversToLog, stalledDriversToPause, stalledDriversToRehome;
#endif

#if HAS_VOLTAGE_MONITOR

	inline constexpr float AdcReadingToPowerVoltage(uint16_t adcVal)
	{
		return adcVal * (VinMonitorVoltageRange/(1u << AnalogIn::AdcBits));
	}

	inline constexpr uint16_t PowerVoltageToAdcReading(float voltage)
	{
		return (uint16_t)(voltage * ((1u << AnalogIn::AdcBits)/VinMonitorVoltageRange));
	}

	constexpr uint16_t driverPowerOnAdcReading = PowerVoltageToAdcReading(10.0);			// minimum voltage at which we initialise the drivers
	constexpr uint16_t driverPowerOffAdcReading = PowerVoltageToAdcReading(9.5);			// voltages below this flag the drivers as unusable

#endif

#if HAS_SMART_DRIVERS
	static void UpdateMotorCurrent(size_t driver)
	{
		SmartDrivers::SetCurrent(driver, (driverAtIdleCurrent[driver]) ? motorCurrents[driver] * idleCurrentFactor : motorCurrents[driver]);
	}
#endif

#ifdef SAME51
	static int32_t tempCalF1, tempCalF2, tempCalF3, tempCalF4;		// temperature calibration factors

	static void ADC_temperature_init(void)
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

	// Send the specified message to the specified destinations. The Error and Warning flags have already been handled.
	void RawMessage(MessageType type, const char *message)
	{
		MutexLocker lock(messageMutex);

		uart0.PutString("{\"message\":\"");
		uart0.PutString(message);		// should do JSON escaping here
		uart0.PutString("\"}\n");
	}

#ifdef SAME51
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
		NVIC_SetPriority(StepTcIRQn, NvicPriorityStep);

#if defined(SAME51)
		NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
		// Set UART interrupt priority. Each SERCOM has up to 4 interrupts, numbered sequentially.
		SetInterruptPriority(Serial0_IRQn, 4, NvicPriorityUart);
		SetInterruptPriority(Serial1_IRQn, 4, NvicPriorityUart);
		SetInterruptPriority(DMAC_0_IRQn, 5, NvicPriorityDmac);
		SetInterruptPriority(EIC_0_IRQn, 16, NvicPriorityPins);
#elif defined(SAMC21)
		NVIC_SetPriority(CAN0_IRQn, NvicPriorityCan);
		NVIC_SetPriority(Serial0_IRQn, NvicPriorityUart);
		NVIC_SetPriority(DMAC_IRQn, NvicPriorityDmac);
		NVIC_SetPriority(EIC_IRQn, NvicPriorityPins);
#else
# error Undefined processor
#endif

		StepTimer::Init();										// initialise the step pulse timer CAN1
	}

	[[noreturn]] RAMFUNC static void EraseAndReset()
	{
#if defined(SAME51)
		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Unlock the block of flash
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_UR | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }

		// Set address and command
		hri_nvmctrl_write_ADDR_reg(NVMCTRL, FirmwareFlashStart);
		hri_nvmctrl_write_CTRLB_reg(NVMCTRL, NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY);

		while (!hri_nvmctrl_get_STATUS_READY_bit(NVMCTRL)) { }
#elif defined(SAMC21)
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

	[[noreturn]] static void DoFirmwareUpdate()
	{
		CanInterface::Shutdown();
		DisableAllDrives();
		delay(10);										// allow existing processing to complete, drivers to be turned off and CAN replies to be sent
		Heat::SwitchOffAll();
#if SUPPORT_TMC51xx
		IoPort::WriteDigital(GlobalTmc51xxEnablePin, true);
#endif
#if SUPPORT_TMC22xx
		IoPort::WriteDigital(GlobalTmc22xxEnablePin, true);
#endif

//		DisableCache();

		// Disable all IRQs
		__disable_irq();
		SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos);	// disable the system tick exception

#if defined(SAME51)
		for (size_t i = 0; i < 8; i++)
		{
			NVIC->ICER[i] = 0xFFFFFFFF;					// Disable IRQs
			NVIC->ICPR[i] = 0xFFFFFFFF;					// Clear pending IRQs
		}
#elif defined(SAMC21)
		NVIC->ICER[0] = 0xFFFFFFFF;						// Disable IRQs
		NVIC->ICPR[0] = 0xFFFFFFFF;						// Clear pending IRQs
#else
# error Unsupported processor
#endif

		digitalWrite(DiagLedPin, false);					// turn the DIAG LED off

		EraseAndReset();
	}

	static void SetupThermistorFilter(Pin pin, size_t filterIndex, bool useAlternateAdc)
	{
		thermistorFilters[filterIndex].Init(0);
		AnalogIn::EnableChannel(pin, thermistorFilters[filterIndex].CallbackFeedIntoFilter, &thermistorFilters[filterIndex], 1, useAlternateAdc);
	}

}	// end namespace Platform

void Platform::SoftwareReset(uint16_t reason, const uint32_t *stk)
{
#if defined(SAME51)
//	qq;
#elif defined(SAMC21)
//	qq;
#else
# error Unsupported processor
#endif
	ResetProcessor();
}

void Platform::Init()
{
	IoPort::Init();

	// Set up the DIAG LED pin
	IoPort::SetPinMode(DiagLedPin, OUTPUT_HIGH);

	messageMutex.Create("Message");

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
				IoPort::SetPinMode(pin, OUTPUT_LOW);				// turn off heaters and fans (although this will turn on PWM fans)
#ifdef SAMC21
				// Set high driver strength on the output pins because they drive the heater and fan mosfets directly
				IoPort::SetHighDriveStrength(pin);

				// OUT2 is intended to drive the hot end fan, so default it to on
				IoPort::SetPinMode(pin, (StringEqualsIgnoreCase(p.pinNames, "out2")) ? OUTPUT_HIGH : OUTPUT_LOW);
																	// turn on fan on out2, turn off heaters and other fans
#else
				IoPort::SetPinMode(pin, OUTPUT_LOW);				// turn off heaters and fans (although this will turn on PWM fans)
#endif
			}
			else if (StringStartsWith(p.pinNames, "spi.cs"))
			{
				IoPort::SetPinMode(pin, INPUT_PULLUP);				// ensure SPI CS lines are high so that temp daughter boards don't drive the bus before they are configured
			}
		}
	}

	// Set up the UART to send to PanelDue for debugging
#ifdef SAME51
	gpio_set_pin_function(PortBPin(20), PINMUX_PB20C_SERCOM3_PAD0);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	gpio_set_pin_function(PortBPin(21), PINMUX_PB21C_SERCOM3_PAD1);		// RxD
# endif
#elif defined(SAMC21)
	gpio_set_pin_function(PortAPin(12), PINMUX_PA12D_SERCOM4_PAD0);		// TxD
#endif

	uart0.Init(256, 0, 57600, 3);

	// Initialise the rest of the IO subsystem
	AnalogIn::Init();
	AnalogOut::Init();
	InitialisePinChangeInterrupts();

#ifdef SAME51
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
	currentVin = highestVin = 0;
	lowestVin = 9999;
	numUnderVoltageEvents = previousUnderVoltageEvents = numOverVoltageEvents = previousOverVoltageEvents = 0;

	vinFilter.Init(0);
	AnalogIn::EnableChannel(VinMonitorPin, vinFilter.CallbackFeedIntoFilter, &vinFilter, 1, false);

#if HAS_12V_MONITOR
	currentV12 = highestV12 = 0;
	lowestV12 = 9999;

	v12Filter.Init(0);
	AnalogIn::EnableChannel(V12MonitorPin, v12Filter.CallbackFeedIntoFilter, &v12Filter, 1, false);
#endif

#if HAS_VREF_MONITOR
	// Set up the Vref and Vssa filters
	SetupThermistorFilter(VrefPin, VrefFilterIndex, false);
	SetupThermistorFilter(VssaPin, VssaFilterIndex, false);

# ifdef SAMC21
	// Set up the SDADC input filters too (temp0 and Vref)
	SetupThermistorFilter(TempSensePins[0], SdAdcTemp0FilterIndex, true);
	SetupThermistorFilter(VrefPin, SdAdcVrefFilterIndex, true);
# endif

#endif

	// Set up the thermistor filters
	for (size_t i = 0; i < NumThermistorInputs; ++i)
	{
		SetupThermistorFilter(TempSensePins[i], i, false);
	}

	// Set up the MCU temperature sensors
	currentMcuTemperature = 0.0;
	highestMcuTemperature = -273.16;
	lowestMcuTemperature = 999.0;
	mcuTemperatureAdjust = 0.0;

#if defined(SAME51)
	tpFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, &tpFilter, 1, 0);
	tcFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, &tcFilter, 1, 0);
#elif defined(SAMC21)
	tsensFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(tsensFilter.CallbackFeedIntoFilter, &tsensFilter, 1);
#else
# error Unsupported processor
#endif

#if HAS_BUTTONS
	for (size_t i = 0; i < NumButtons; ++i)
	{
		IoPort::SetPinMode(ButtonPins[i], PinMode::INPUT_PULLUP);
	}
#endif

	// Initialise stepper drivers
#if HAS_SMART_DRIVERS
	SmartDrivers::Init();
	temperatureShutdownDrivers = temperatureWarningDrivers = shortToGroundDrivers = openLoadADrivers = openLoadBDrivers = notOpenLoadADrivers = notOpenLoadBDrivers = 0;
#endif

#ifdef TOOL1LC_V04
	IoPort::SetPinMode(Tmc2209DiagPin, INPUT);
#endif

	for (size_t i = 0; i < NumDrivers; ++i)
	{
#if ACTIVE_HIGH_STEP
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
#else
		IoPort::SetPinMode(StepPins[i], OUTPUT_HIGH);
#endif
#if !HAS_SMART_DRIVERS
		IoPort::SetHighDriveStrength(StepPins[i]);
#endif

#if ACTIVE_HIGH_DIR
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
#else
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_HIGH);
#endif
#if !HAS_SMART_DRIVERS
		IoPort::SetHighDriveStrength(DirectionPins[i]);
# if ACTIVE_HIGH_ENABLE
		IoPort::SetPinMode(EnablePins[i], OUTPUT_LOW);
		enableValues[i] = 1;
# else
		IoPort::SetPinMode(EnablePins[i], OUTPUT_HIGH);
		enableValues[i] = 0;
# endif
		IoPort::SetHighDriveStrength(EnablePins[i]);
		driverIsEnabled[i] = false;
#endif
		const uint32_t driverBit = 1u << (StepPins[i] & 31);
		driveDriverBits[i] = driverBit;
		allDriverBits |= driverBit;
		stepsPerMm[i] = DefaultStepsPerMm;
		directions[i] = true;
		driverAtIdleCurrent[i] = false;
		motorCurrents[i] = 0.0;
		pressureAdvance[i] = 0.0;

#if HAS_SMART_DRIVERS
		SmartDrivers::SetMicrostepping(i, 16, true);
#endif
	}

	idleCurrentFactor = 0.3;

#if HAS_STALL_DETECT
	stalledDrivers = 0;
	logOnStallDrivers = pauseOnStallDrivers = rehomeOnStallDrivers = 0;
	stalledDriversToLog = stalledDriversToPause = stalledDriversToRehome = 0;
#endif

#if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	warnDriversNotPowered = false;
#endif

	CanInterface::Init(ReadBoardId());

	InitialiseInterrupts();

	// Read the unique ID
	for (unsigned int i =0; i < 4; ++i)
	{
		uniqueId[i] = *reinterpret_cast<const uint32_t*>(SerialNumberAddresses[i]);
	}

	// Put the checksum at the end
	// We only print 30 5-bit characters = 128 data bits + 22 checksum bits. So compress the 32 checksum bits into 22.
	uniqueId[4] = uniqueId[0] ^ uniqueId[1] ^ uniqueId[2] ^ uniqueId[3];
	uniqueId[4] ^= (uniqueId[4] >> 10);

	lastPollTime = millis();
}

void Platform::Spin()
{
	static bool powered = false;

	if (doFirmwareUpdate)
	{
		DoFirmwareUpdate();
	}

	// Get the VIN voltage
	currentVin = vinFilter.GetSum()/vinFilter.NumAveraged();
	const float voltsVin = (currentVin * VinMonitorVoltageRange)/(1u << AnalogIn::AdcBits);
#if HAS_12V_MONITOR
	currentV12 = v12Filter.GetSum()/v12Filter.NumAveraged();
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
#else
	if (!powered && voltsVin >= 10.5)
	{
		powered = true;
	}
	else if (powered && voltsVin < 10.0)
	{
		powered = false;
	}
#endif

#if 0 //ifdef SAMC21
	if (!digitalRead(ButtonPins[0]))
	{
		SCB->AIRCR = (0x5FA << 16) | (1u << 2);				// reset the processor
	}
#endif

#if HAS_SMART_DRIVERS
	SmartDrivers::Spin(powered);
#endif

	// Thermostatically-controlled fans (do this after getting TMC driver status)
	const uint32_t now = millis();
	if (now - lastFanCheckTime >= FanCheckInterval)
	{
		(void)FansManager::CheckFans();

#if HAS_SMART_DRIVERS
		// Check one TMC driver for temperature warning or temperature shutdown
		if (enableValues[nextDriveToPoll] >= 0)				// don't poll driver if it is flagged "no poll"
		{
			const uint32_t stat = SmartDrivers::GetAccumulatedStatus(nextDriveToPoll, 0);
			const DriversBitmap mask = MakeBitmap<DriversBitmap>(nextDriveToPoll);
			if (stat & TMC_RR_OT)
			{
				temperatureShutdownDrivers |= mask;
			}
			else if (stat & TMC_RR_OTPW)
			{
				temperatureWarningDrivers |= mask;
			}
			if (stat & TMC_RR_S2G)
			{
				shortToGroundDrivers |= mask;
			}
			else
			{
				shortToGroundDrivers &= ~mask;
			}

			// The driver often produces a transient open-load error, especially in stealthchop mode, so we require the condition to persist before we report it.
			// Also, false open load indications persist when in standstill, if the phase has zero current in that position
			if ((stat & TMC_RR_OLA) != 0)
			{
				if (!openLoadATimer.IsRunning())
				{
					openLoadATimer.Start();
					openLoadADrivers = notOpenLoadADrivers = 0;
				}
				openLoadADrivers |= mask;
			}
			else if (openLoadATimer.IsRunning())
			{
				notOpenLoadADrivers |= mask;
				if ((openLoadADrivers & ~notOpenLoadADrivers) == 0)
				{
					openLoadATimer.Stop();
				}
			}

			if ((stat & TMC_RR_OLB) != 0)
			{
				if (!openLoadBTimer.IsRunning())
				{
					openLoadBTimer.Start();
					openLoadBDrivers = notOpenLoadBDrivers = 0;
				}
				openLoadBDrivers |= mask;
			}
			else if (openLoadBTimer.IsRunning())
			{
				notOpenLoadBDrivers |= mask;
				if ((openLoadBDrivers & ~notOpenLoadBDrivers) == 0)
				{
					openLoadBTimer.Stop();
				}
			}

# if HAS_STALL_DETECT
			if ((stat & TMC_RR_SG) != 0)
			{
				if ((stalledDrivers & mask) == 0)
				{
					// This stall is new so check whether we need to perform some action in response to the stall
					if ((rehomeOnStallDrivers & mask) != 0)
					{
						stalledDriversToRehome |= mask;
					}
					else if ((pauseOnStallDrivers & mask) != 0)
					{
						stalledDriversToPause |= mask;
					}
					else if ((logOnStallDrivers & mask) != 0)
					{
						stalledDriversToLog |= mask;
					}
				}
				stalledDrivers |= mask;
			}
			else
			{
				stalledDrivers &= ~mask;
			}
# endif
		}

# if 0 //HAS_STALL_DETECT
		// Action any pause or rehome actions due to motor stalls. This may have to be done more than once.
		if (stalledDriversToRehome != 0)
		{
			if (reprap.GetGCodes().ReHomeOnStall(stalledDriversToRehome))
			{
				stalledDriversToRehome = 0;
			}
		}
		else if (stalledDriversToPause != 0)
		{
			if (reprap.GetGCodes().PauseOnStall(stalledDriversToPause))
			{
				stalledDriversToPause = 0;
			}
		}
# endif

		// Advance drive number ready for next time
		++nextDriveToPoll;
		if (nextDriveToPoll == MaxSmartDrivers)
		{
			nextDriveToPoll = 0;
		}
#endif
	}

	// Update the Diag LED. Flash it quickly (8Hz) if we are not synced to the master, else flash in sync with the master (about 2Hz).
	gpio_set_pin_level(DiagLedPin,
						(StepTimer::IsSynced()) ? (StepTimer::GetMasterTime() & (1u << 19)) != 0
							: (StepTimer::GetTimerTicks() & (1u << 17)) != 0
					  );

	if (now - lastPollTime > 2000)
	{
		lastPollTime = now;

#if 0
		static uint8_t oldAddr = 0xFF;
		const uint8_t addr = ReadBoardId();
		if (addr != oldAddr)
		{
			oldAddr = addr;
			const float current = (addr == 3) ? 6300.0 : (addr == 2) ? 2000.0 : (addr == 1) ? 1000.0 : 500.0;
			for (size_t i = 0; i < NumDrivers; ++i)
			{
				SmartDrivers::SetCurrent(i, current);
			}
		}
#endif

		// Get the chip temperature
#if defined(SAME51)
		if (tcFilter.IsValid() && tpFilter.IsValid())
		{
			// From the datasheet:
			// T = (tl * vph * tc - th * vph * tc - tl * tp *vch + th * tp * vcl)/(tp * vcl - tp * vch - tc * vpl * tc * vph)
			const uint16_t tc_result = tcFilter.GetSum()/(tcFilter.NumAveraged() << (AnalogIn::AdcBits - 12));
			const uint16_t tp_result = tpFilter.GetSum()/(tpFilter.NumAveraged() << (AnalogIn::AdcBits - 12));

			int32_t result =  (tempCalF1 * tc_result - tempCalF2 * tp_result);
			const int32_t divisor = (tempCalF3 * tp_result - tempCalF4 * tc_result);
			result = (divisor == 0) ? 0 : result/divisor;
			currentMcuTemperature = (float)result/16 + mcuTemperatureAdjust;
#elif defined(SAMC21)
		if (tsensFilter.IsValid())
		{
			const int16_t temperatureTimes100 = (int16_t)((uint16_t)(tsensFilter.GetSum()/tsensFilter.NumAveraged()) ^ (1u << 15));
			currentMcuTemperature = (float)temperatureTimes100 * 0.01;
#else
# error Unsupported processor
#endif
			if (currentMcuTemperature < lowestMcuTemperature)
			{
				lowestMcuTemperature = currentMcuTemperature;
			}
			if (currentMcuTemperature > highestMcuTemperature)
			{
				highestMcuTemperature = currentMcuTemperature;
			}
		}

		static unsigned int nextSensor = 0;

		const auto ts = Heat::FindSensorAtOrAbove(nextSensor);
		if (ts.IsNotNull())
		{
			float temp;
			const TemperatureError err = ts->GetLatestTemperature(temp);
			debugPrintf("Sensor %u err %u temp %.1f", ts->GetSensorNumber(), (unsigned int)err, (double)temp);
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
#else
//			uint32_t conversionsStarted, conversionsCompleted;
//			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted);
			debugPrintf(
//							"Conv %u %u"
						"Addr %u"
#if HAS_12V_MONITOR
						" %.1fV %.1fV"
#else
						" %.1fV"
#endif
						" %.1fC"
						" %u %u"
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
#else
						(double)voltsVin,
#endif
						(double)currentMcuTemperature
						, (unsigned int)thermistorFilters[VrefFilterIndex].GetSum(), (unsigned int)thermistorFilters[VssaFilterIndex].GetSum()
//						, tp_result, tc_result
#if HAS_SMART_DRIVERS
						, SmartDrivers::GetAccumulatedStatus(0, 0), SmartDrivers::GetAccumulatedStatus(1, 0), SmartDrivers::GetAccumulatedStatus(2, 0)
#endif
//							, SmartDrivers::GetLiveStatus(0), SmartDrivers::GetLiveStatus(1), SmartDrivers::GetLiveStatus(2)
					   );
#endif
		}
	}
}

// Get the index of the averaging filter for an analog port
int Platform::GetAveragingFilterIndex(const IoPort& port)
{
	for (size_t i = 0; i < NumThermistorFilters; ++i)
	{
		if (port.GetPin() == TempSensePins[i])
		{
#ifdef SAMC21
			return (i == 0 && port.UseAlternateConfig()) ? SdAdcTemp0FilterIndex : (int) i;
#endif
			return (int)i;
		}
	}
	return -1;
}

ThermistorAveragingFilter *Platform::GetAdcFilter(unsigned int filterNumber)
{
	return &thermistorFilters[filterNumber];
}

ThermistorAveragingFilter *Platform::GetVssaFilter(unsigned int filterNumber)
{
#ifdef SAMC21
	// The SDADC channel has INN connected to VSSA and no separate VSSA monitor
	return (filterNumber < NumThermistorInputs) ? &thermistorFilters[VssaFilterIndex] : nullptr;
#else
	return &thermistorFilters[VssaFilterIndex];
#endif
}

ThermistorAveragingFilter *Platform::GetVrefFilter(unsigned int filterNumber)
{
#ifdef SAMC21
	// The SDADC channel has a separate VSSA monitor
	return (filterNumber == SdAdcTemp0FilterIndex) ? &thermistorFilters[SdAdcVrefFilterIndex] : &thermistorFilters[VrefFilterIndex];
#else
	return &thermistorFilters[VrefFilterIndex];
#endif
}

void Platform::GetMcuTemperatures(float& minTemp, float& currentTemp, float& maxTemp)
{
	minTemp = lowestMcuTemperature;
	currentTemp = currentMcuTemperature;
	maxTemp = highestMcuTemperature;
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

void Platform::MessageF(MessageType type, const char *fmt, va_list vargs)
{
	String<FormatStringLength> formatString;
	if ((type & ErrorMessageFlag) != 0)
	{
		formatString.copy("Error: ");
		formatString.vcatf(fmt, vargs);
	}
	else if ((type & WarningMessageFlag) != 0)
	{
		formatString.copy("Warning: ");
		formatString.vcatf(fmt, vargs);
	}
	else
	{
		formatString.vprintf(fmt, vargs);
	}

	RawMessage((MessageType)(type & ~(ErrorMessageFlag | WarningMessageFlag)), formatString.c_str());
}

void Platform::MessageF(MessageType type, const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	MessageF(type, fmt, vargs);
	va_end(vargs);
}

void Platform::Message(MessageType type, const char *message)
{
	if ((type & (ErrorMessageFlag | WarningMessageFlag)) == 0)
	{
		RawMessage(type, message);
	}
	else
	{
		String<FormatStringLength> formatString;
		formatString.copy(((type & ErrorMessageFlag) != 0) ? "Error: " : "Warning: ");
		formatString.cat(message);
		RawMessage((MessageType)(type & ~(ErrorMessageFlag | WarningMessageFlag)), formatString.c_str());
	}
}

void Platform::LogError(ErrorCode e)
{
	errorCodeBits |= (uint32_t)e;
}

bool Platform::Debug(Module module)
{
	return false;
}

float Platform::DriveStepsPerUnit(size_t drive) { return stepsPerMm[drive]; }

const float *Platform::GetDriveStepsPerUnit() { return stepsPerMm; }

void Platform::SetDriverStepTiming(size_t drive, const float timings[4])
{
	for (size_t i = 0; i < 4; ++i)
	{
		if (slowDriverStepTimingClocks[i] < timings[i])
		{
			slowDriverStepTimingClocks[i] = min<float>(timings[i], 50.0);
		}
	}
}

float Platform::GetPressureAdvance(size_t driver)
{
	return pressureAdvance[driver];
}

void Platform::SetPressureAdvance(size_t driver, float advance)
{
	pressureAdvance[driver] = advance;
}

void Platform::StepDriversLow()
{
#if ACTIVE_HIGH_STEP
	// Active high step output
	StepPio->OUTCLR.reg = allDriverBits;
#else
	// Active low step output
	StepPio->OUTSET.reg = allDriverBits;
#endif
}

void Platform::StepDriversHigh(uint32_t driverMap)
{
#if ACTIVE_HIGH_STEP
	// Active high step output
	StepPio->OUTSET.reg = driverMap;
#else
	// Active low step output
	StepPio->OUTCLR.reg = driverMap;
#endif
}

//	uint32_t CalcDriverBitmap(size_t driver);

uint32_t Platform::GetDriversBitmap(size_t axisOrExtruder) 	// get the bitmap of driver step bits for this axis or extruder
{
	return driveDriverBits[axisOrExtruder];
}

//	unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions);

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

void Platform::SetDirection(size_t driver, bool direction)
{
	if (driver < NumDrivers)
	{
#if ACTIVE_HIGH_DIR
		// Active high direction signal
		const bool d = (direction) ? directions[driver] : !directions[driver];
#else
		// Active low direction signal
		const bool d = (direction) ? !directions[driver] : directions[driver];
#endif
		const bool isSlowDriver = IsBitSet(slowDriversBitmap, driver);
		if (isSlowDriver)
		{
			while (StepTimer::GetTimerTicks() - DDA::lastStepLowTime < GetSlowDriverDirHoldClocks()) { }
		}
		digitalWrite(DirectionPins[driver], d);
		if (isSlowDriver)
		{
			DDA::lastDirChangeTime = StepTimer::GetTimerTicks();
		}
	}
}

// The following don't do anything yet
void Platform::SetEnableValue(size_t driver, int8_t eVal)
{
	if (driver < NumDrivers)
	{
		enableValues[driver] = eVal;
#if !HAS_SMART_DRIVERS
		if (driverIsEnabled[driver])
		{
			EnableDrive(driver);
		}
		else
		{
			DisableDrive(driver);
		}
#endif
	}
}

int8_t Platform::GetEnableValue(size_t driver)
{
	return (driver < NumDrivers) ? enableValues[driver] : 0;
}

void Platform::EnableDrive(size_t driver)
{
#if HAS_SMART_DRIVERS
	if (driverAtIdleCurrent[driver])
	{
		driverAtIdleCurrent[driver] = false;
		UpdateMotorCurrent(driver);
	}
	SmartDrivers::EnableDrive(driver, true);
#else
	if (enableValues[driver] >= 0)
	{
		digitalWrite(EnablePins[driver], enableValues[driver] > 0);
	}
#endif
}

void Platform::DisableDrive(size_t driver)
{
#if HAS_SMART_DRIVERS
	SmartDrivers::EnableDrive(driver, false);
#else
	if (enableValues[driver] >= 0)
	{
		digitalWrite(EnablePins[driver], enableValues[driver] == 0);
	}
#endif
}

void Platform::SetDriverIdle(size_t driver)
{
	if (idleCurrentFactor == 0.0)
	{
		DisableDrive(driver);
	}
#if HAS_SMART_DRIVERS
	else
	{
		driverAtIdleCurrent[driver] = true;
		UpdateMotorCurrent(driver);
	}
#endif
}

void Platform::DisableAllDrives()
{
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
#if HAS_SMART_DRIVERS
		SmartDrivers::EnableDrive(driver, false);
#else
		DisableDrive(driver);
#endif
	}
}

#if HAS_SMART_DRIVERS

void Platform::SetMotorCurrent(size_t driver, float current)
{
	motorCurrents[driver] = current;
	UpdateMotorCurrent(driver);
}

#endif

uint8_t Platform::ReadBoardId()
{
#if HAS_ADDRESS_SWITCHES
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 4; ++i)
	{
		if (!digitalRead(BoardAddressPins[i]))
		{
			rslt |= 1 << i;
		}
	}
	return rslt;
#elif defined(BOARD_ADDRESS)
	return BOARD_ADDRESS;
#else
	return 10;			//TODO temporary!
#endif
}

// Append the unique processor ID to a string as 30 base5 alphanumeric digits with 5 embedded separators
void Platform::AppendUniqueId(const StringRef& str)
{
	for (size_t i = 0; i < 30; ++i)
	{
		if ((i % 5) == 0 && i != 0)
		{
			str.cat('-');
		}
		const size_t index = (i * 5) / 32;
		const size_t shift = (i * 5) % 32;
		uint32_t val = uniqueId[index] >> shift;
		if (shift > 32 - 5)
		{
			// We need some bits from the next dword too
			val |= uniqueId[index + 1] << (32 - shift);
		}
		val &= 31;
		char c;
		if (val < 10)
		{
			c = val + '0';
		}
		else
		{
			c = val + ('A' - 10);
			// We have 26 letters in the usual A-Z alphabet and we only need 22 of them plus 0-9.
			// So avoid using letters C, E, I and O which are easily mistaken for G, F, 1 and 0.
			if (c >= 'C')
			{
				++c;
			}
			if (c >= 'E')
			{
				++c;
			}
			if (c >= 'I')
			{
				++c;
			}
			if (c >= 'O')
			{
				++c;
			}
		}
		str.cat(c);
	}
}

#if HAS_SMART_DRIVERS
// TMC driver temperatures
float Platform::GetTmcDriversTemperature()
{
	const uint16_t mask = (1u << MaxSmartDrivers) - 1;
	return ((temperatureShutdownDrivers & mask) != 0) ? 150.0
			: ((temperatureWarningDrivers & mask) != 0) ? 100.0
				: 0.0;
}
#endif

void Platform::Tick()
{
	++heatTaskIdleTicks;
}

void Platform::StartFirmwareUpdate()
{
	doFirmwareUpdate = true;
}

#if HAS_VOLTAGE_MONITOR

float Platform::GetMinVinVoltage()
{
	return AdcReadingToPowerVoltage(lowestVin);
}

float Platform::GetCurrentVinVoltage()
{
	return AdcReadingToPowerVoltage(currentVin);
}

float Platform::GetMaxVinVoltage()
{
	return AdcReadingToPowerVoltage(highestVin);
}

#endif

#if HAS_12V_MONITOR

float Platform::GetMinV12Voltage()
{
	return AdcReadingToPowerVoltage(lowestV12);
}

float Platform::GetCurrentV12Voltage()
{
	return AdcReadingToPowerVoltage(currentV12);
}

float Platform::GetMaxV12Voltage()
{
	return AdcReadingToPowerVoltage(highestV12);
}

#endif

// End
