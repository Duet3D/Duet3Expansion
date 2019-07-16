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
#include <Movement/Move.h>
#include "Movement/StepperDrivers/TMC51xx.h"
#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>
#include "AdcAveragingFilter.h"
#include "Movement/StepTimer.h"
#include "CAN/CanSlaveInterface.h"

static bool txBusy = false;

extern "C" void tx_cb_USART_0(const struct usart_async_descriptor *const io_descr)
{
	txBusy = false;
}

namespace Platform
{
	static struct io_descriptor *io;

	static uint32_t errorCodeBits = 0;
	uint32_t slowDriversBitmap = 0;
	uint32_t slowDriverStepTimingClocks[4] = { 0, 0, 0, 0 };

	uint32_t driveDriverBits[NumDrivers];
	uint32_t allDriverBits = 0;

	static float stepsPerMm[NumDrivers] = { 80.0, 80.0, 80.0 };
	static float axisMinima[MaxAxes] = { 0.0, 0.0, 0.0 };
	static float axisMaxima[MaxAxes] = { 500.0, 500.0, 500.0 };
	static float accelerations[NumDrivers] = { 1000.0, 1000.0, 1000.0 };
	static float feedrates[NumDrivers] = { 100.0, 100.0, 100.0 };			// mm/sec
	static float instantDVs[NumDrivers] = { 5.0, 5.0, 5.0 };				// mm/sec

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
	static unsigned int heatTaskIdleTicks = 0;

	// SERCOM3 Rx is on PB21 (OUT_8_TACHO), Tx is on PB20 (OUT_7_TACHO)

	static ThermistorAveragingFilter thermistorFilters[NumThermistorFilters];
	static AdcAveragingFilter<VinReadingsAveraged> vinFilter;
#if HAS_12V_MONITOR
	static AdcAveragingFilter<VinReadingsAveraged> v12Filter;
#endif
	static AdcAveragingFilter<McuTempReadingsAveraged> tpFilter;
	static AdcAveragingFilter<McuTempReadingsAveraged> tcFilter;

	// Temperature sense stuff
	#define NVM_TEMP_CAL_TLI_POS 0
	#define NVM_TEMP_CAL_TLI_SIZE 8
	#define NVM_TEMP_CAL_TLD_POS 8
	#define NVM_TEMP_CAL_TLD_SIZE 4
	#define NVM_TEMP_CAL_THI_POS 12
	#define NVM_TEMP_CAL_THI_SIZE 8
	#define NVM_TEMP_CAL_THD_POS 20
	#define NVM_TEMP_CAL_THD_SIZE 4
	#define NVM_TEMP_CAL_VPL_POS 40
	#define NVM_TEMP_CAL_VPL_SIZE 12
	#define NVM_TEMP_CAL_VPH_POS 52
	#define NVM_TEMP_CAL_VPH_SIZE 12
	#define NVM_TEMP_CAL_VCL_POS 64
	#define NVM_TEMP_CAL_VCL_SIZE 12
	#define NVM_TEMP_CAL_VCH_POS 76
	#define NVM_TEMP_CAL_VCH_SIZE 12

	static uint16_t temp_cal_tl, temp_cal_th;
	static uint16_t temp_cal_vpl, temp_cal_vph, temp_cal_vcl, temp_cal_vch;

	static void ADC_temperature_init(void)
	{
		temp_cal_vpl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPL_POS / 32)) >> (NVM_TEMP_CAL_VPL_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VPL_SIZE) - 1);
		temp_cal_vph = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPH_POS / 32)) >> (NVM_TEMP_CAL_VPH_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VPH_SIZE) - 1);
		temp_cal_vcl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCL_POS / 32)) >> (NVM_TEMP_CAL_VCL_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VCL_SIZE) - 1);
		temp_cal_vch = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCH_POS / 32)) >> (NVM_TEMP_CAL_VCH_POS % 32))
		               & ((1u << NVM_TEMP_CAL_VCH_SIZE) - 1);

		const uint8_t temp_cal_tli = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLI_POS / 32)) >> (NVM_TEMP_CAL_TLI_POS % 32))
		               & ((1u << NVM_TEMP_CAL_TLI_SIZE) - 1);
		const uint8_t temp_cal_tld = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLD_POS / 32)) >> (NVM_TEMP_CAL_TLD_POS % 32))
		               & ((1u << NVM_TEMP_CAL_TLD_SIZE) - 1);
		temp_cal_tl = ((uint16_t)temp_cal_tli) << 4 | ((uint16_t)temp_cal_tld);

		const uint8_t temp_cal_thi = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THI_POS / 32)) >> (NVM_TEMP_CAL_THI_POS % 32))
		               & ((1u << NVM_TEMP_CAL_THI_SIZE) - 1);
		const uint8_t temp_cal_thd = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THD_POS / 32)) >> (NVM_TEMP_CAL_THD_POS % 32))
		               & ((1u << NVM_TEMP_CAL_THD_SIZE) - 1);
		temp_cal_th = ((uint16_t)temp_cal_thi) << 4 | ((uint16_t)temp_cal_thd);
	}

	// Send the specified message to the specified destinations. The Error and Warning flags have already been handled.
	void RawMessage(MessageType type, const char *message)
	{
		// io_write requires that the message doesn't go out of scope until transmission is complete, so copy it to a static buffer
		static String<200> buffer;
		buffer.copy("{\"message\":\"");
		buffer.cat(message);		// should do JSON escaping here
		buffer.cat("\"}\n");
		txBusy = true;
		io_write(io, (const unsigned char *)buffer.c_str(), buffer.strlen());
		//while (txBusy) { delay(1); }
	}

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

	static void InitialiseInterrupts()
	{
		// Set UART interrupt priority. Each SERCOM has up to 4 interrupts, numbered sequentially.
		SetInterruptPriority(Serial0_IRQn, 4, NvicPriorityUart);
		SetInterruptPriority(Serial1_IRQn, 4, NvicPriorityUart);

		NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
		NVIC_SetPriority(StepTcIRQn, NvicPriorityStep);

		SetInterruptPriority(DMAC_0_IRQn, 5, NvicPriorityDmac);
		SetInterruptPriority(EIC_0_IRQn, 16, NvicPriorityPins);

		StepTimer::Init();										// initialise the step pulse timer
	}

}	// end namespace Platform

void Platform::Init()
{
	IoPort::Init();

	// Set up the DIAG LED pin
	IoPort::SetPinMode(DiagLedPin, OUTPUT_HIGH);

	// Turn all outputs off
	for (size_t pin = 0; pin < ARRAY_SIZE(PinTable); ++pin)
	{
		const PinDescription& p = PinTable[pin];
		if (   p.pinNames != nullptr
			&& StringStartsWith(p.pinNames, "out")
			&& strlen(p.pinNames) < 5							// don't set "outN.tach" pins to outputs
		   )
		{
			IoPort::SetPinMode(pin, OUTPUT_LOW);
		}
	}

	// Set up the UART to send to PanelDue
	const uint32_t baudDiv = 65536 - ((65536 * 16.0f * 57600) / CONF_GCLK_SERCOM3_CORE_FREQUENCY);
	usart_async_set_baud_rate(&USART_0, baudDiv);

	usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_USART_0);
	//usart_async_register_callback(&USART_0, USART_ASYNC_RXC_CB, rx_cb);
	//usart_async_register_callback(&USART_0, USART_ASYNC_ERROR_CB, err_cb);
	usart_async_get_io_descriptor(&USART_0, &io);
	usart_async_enable(&USART_0);

	// Initialise the rest of the IO subsystem
	AnalogIn::Init();
	AnalogOut::Init();
	ADC_temperature_init();

	// Set up the board ID switch inputs
	for (unsigned int i = 0; i < 4; ++i)
	{
		IoPort::SetPinMode(BoardAddressPins[i], INPUT_PULLUP);
	}

	// Set up VIN voltage monitoring
	currentVin = highestVin = 0;
	lowestVin = 9999;
	numUnderVoltageEvents = previousUnderVoltageEvents = numOverVoltageEvents = previousOverVoltageEvents = 0;

	vinFilter.Init(0);
	AnalogIn::EnableChannel(VinMonitorPin, vinFilter.CallbackFeedIntoFilter, &vinFilter);

#if HAS_12V_MONITOR
	currentV12 = highestV12 = 0;
	lowestV12 = 9999;

	v12Filter.Init(0);
	AnalogIn::EnableChannel(V12MonitorPin, vinFilter.CallbackFeedIntoFilter, &v12Filter);
#endif

	// Set up the MCU temperature sensors
	currentMcuTemperature = 0.0;
	highestMcuTemperature = -273.16;
	lowestMcuTemperature = 999.0;
	mcuTemperatureAdjust = 0.0;

	tpFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, &tpFilter, 1);
	tcFilter.Init(0);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, &tcFilter, 1);

	// Set up the thermistor filters
	for (size_t i = 0; i < NumThermistorInputs; ++i)
	{
		thermistorFilters[i].Init(0);
		AnalogIn::EnableChannel(TempSensePins[i], thermistorFilters[i].CallbackFeedIntoFilter, &thermistorFilters[i]);
	}

#if HAS_VREF_MONITOR
	thermistorFilters[VrefFilterIndex].Init(0);
	AnalogIn::EnableChannel(VrefPin, thermistorFilters[VrefFilterIndex].CallbackFeedIntoFilter, &thermistorFilters[VrefFilterIndex]);
	thermistorFilters[VssaFilterIndex].Init(0);
	AnalogIn::EnableChannel(VssaPin, thermistorFilters[VssaFilterIndex].CallbackFeedIntoFilter, &thermistorFilters[VssaFilterIndex]);
#endif

	// Initialise stepper drivers
	SmartDrivers::Init();

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
		const uint32_t driverBit = 1u << (StepPins[i] & 31);
		driveDriverBits[i] = driverBit;
		allDriverBits |= driverBit;

		SmartDrivers::SetMicrostepping(i, 16, true);
	}

	CanSlaveInterface::Init();

	InitialiseInterrupts();

	lastPollTime = millis();

	//TEST set up some PWM values
	static PwmPort out0, out1, out2, out3, out4, out5, out6, out7, out8;
	String<100> reply;

	out0.AssignPort("out0", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out1.AssignPort("out1", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out2.AssignPort("out2", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out3.AssignPort("out3", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out4.AssignPort("out4", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out5.AssignPort("out5", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out6.AssignPort("out6", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out7.AssignPort("out7", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);
	out8.AssignPort("out8", reply.GetRef(), PinUsedBy::gpio, PinAccess::pwm);

	out0.SetFrequency(100);
	out0.WriteAnalog(0.1);

	out1.SetFrequency(200);
	out1.WriteAnalog(0.2);

	out2.SetFrequency(400);
	out2.WriteAnalog(0.3);

	out3.SetFrequency(500);
	out3.WriteAnalog(0.4);

	out4.SetFrequency(1000);
	out4.WriteAnalog(0.5);

	out5.SetFrequency(2000);
	out5.WriteAnalog(0.6);

	out6.SetFrequency(4000);
	out6.WriteAnalog(0.7);

	out7.SetFrequency(10000);
	out7.WriteAnalog(0.8);

	out8.SetFrequency(25000);
	out8.WriteAnalog(0.9);
}

void Platform::Spin()
{
	static uint8_t oldAddr = 0;
	static bool powered = false;

	// Get the VIN voltage
	const float volts = (vinFilter.GetSum() * (3.3 * 11))/(4096 * vinFilter.NumAveraged());
	if (!powered && volts >= 10.0)
	{
		powered = true;
		SmartDrivers::Spin(true);
	}
	else if (powered && volts < 9.5)
	{
		powered = false;
		SmartDrivers::Spin(false);
	}

	// Update the Diag LED. Flash it quickly (8Hz) if we are not synced to the master, else flash in sync with the master (about 2Hz).
	gpio_set_pin_level(DiagLedPin,
						(StepTimer::IsSynced()) ? (StepTimer::GetMasterTime() & (1u << 19)) != 0
							: (StepTimer::GetInterruptClocks() & (1u << 17)) != 0
					  );

	const uint32_t now = millis();
	if (now - lastPollTime > 2000)
	{
		lastPollTime = now;

		const uint8_t addr = ReadBoardSwitches() >> 2;
		if (addr != oldAddr)
		{
			oldAddr = addr;
			const float current = (addr == 3) ? 3000.0 : (addr == 2) ? 2000.0 : (addr == 1) ? 1000.0 : 500.0;
			for (size_t i = 0; i < NumDrivers; ++i)
			{
				SmartDrivers::SetCurrent(i, current);
			}
		}

		// Get the chip temperature
		if (tcFilter.IsValid() && tpFilter.IsValid())
		{
			const uint16_t tc_result = tcFilter.GetSum()/tcFilter.NumAveraged();
			const uint16_t tp_result = tpFilter.GetSum()/tpFilter.NumAveraged();

			int32_t result = (int64_t)temp_cal_tl * temp_cal_vph * tc_result
							- (int64_t)temp_cal_vpl * temp_cal_th * tc_result
							- (int64_t)temp_cal_tl * temp_cal_vch * tp_result
							+ (int64_t)temp_cal_th * temp_cal_vcl * tp_result;
			const int32_t divisor = ((int32_t)temp_cal_vcl * tp_result - (int32_t)temp_cal_vch * tp_result
							   - (int32_t)temp_cal_vpl * tc_result + (int32_t)temp_cal_vph * tc_result);
			result = (divisor == 0) ? 0 : result/divisor;
			currentMcuTemperature = (float)result/16 + mcuTemperatureAdjust;
			if (currentMcuTemperature < lowestMcuTemperature)
			{
				lowestMcuTemperature = currentMcuTemperature;
			}
			if (currentMcuTemperature > highestMcuTemperature)
			{
				highestMcuTemperature = currentMcuTemperature;
			}
		}

#if 0
		String<100> status;
		SmartDrivers::AppendDriverStatus(2, status.GetRef());
		debugPrintf("%s", status.c_str());
#elif 0
		moveInstance->Diagnostics(AuxMessage);
#elif 1
#else
		uint32_t conversionsStarted, conversionsCompleted;
		AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted);
		debugPrintf(
//							"Conv %u %u"
						"Addr %u"
						", %.1fV, %.1fdegC"
//						", ptat %d, ctat %d"
						", stat %08" PRIx32 " %08" PRIx32 " %08" PRIx32,
//							(unsigned int)conversionsStarted, (unsigned int)conversionsCompleted,
//							StepTimer::GetInterruptClocks(),
						(unsigned int)addr,
						(double)volts, (double)currentMcuTemperature
//						, tp_result, tc_result
						, SmartDrivers::GetAccumulatedStatus(0, 0), SmartDrivers::GetAccumulatedStatus(1, 0), SmartDrivers::GetAccumulatedStatus(2, 0)
//							, SmartDrivers::GetLiveStatus(0), SmartDrivers::GetLiveStatus(1), SmartDrivers::GetLiveStatus(2)
					);
#endif

	}
}

ThermistorAveragingFilter& Platform::GetAdcFilter(unsigned int filterNumber)
{
	return thermistorFilters[filterNumber];
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

void MessageF(MessageType type, const char *fmt, ...)
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

void Platform::SetDriversIdle() { }

float Platform::DriveStepsPerUnit(size_t drive) { return stepsPerMm[drive]; }

const float *Platform::GetDriveStepsPerUnit() { return stepsPerMm; }

float Platform::AxisMaximum(size_t axis) { return axisMaxima[axis]; }
//	void SetAxisMaximum(size_t axis, float value, bool byProbing);
float Platform::AxisMinimum(size_t axis) { return axisMinima[axis]; }
//	void SetAxisMinimum(size_t axis, float value, bool byProbing);
//	float AxisTotalLength(size_t axis) ;
float Platform::GetPressureAdvance(size_t extruder) { return 0.4; }
//	void SetPressureAdvance(size_t extruder, float factor);
float Platform::Acceleration(size_t axisOrExtruder) { return accelerations[axisOrExtruder]; }
const float* Platform::Accelerations() { return accelerations; }
//	void SetAcceleration(size_t axisOrExtruder, float value);
float Platform::MaxFeedrate(size_t axisOrExtruder) { return feedrates[axisOrExtruder]; }
const float* Platform::MaxFeedrates() { return feedrates; }
//	void SetMaxFeedrate(size_t axisOrExtruder, float value);
float Platform::GetInstantDv(size_t axis) { return instantDVs[axis]; }
//	void SetInstantDv(size_t axis, float value);
EndStopHit Platform::Stopped(size_t axisOrExtruder) { return EndStopHit::lowHit; }
bool Platform::EndStopInputState(size_t axis) { return false; }

void Platform::StepDriversLow()
{
	StepPio->OUTCLR.reg = allDriverBits;
}

void Platform::StepDriversHigh(uint32_t driverMap)
{
	StepPio->OUTSET.reg = driverMap;
}

//	uint32_t CalcDriverBitmap(size_t driver);

uint32_t Platform::GetDriversBitmap(size_t axisOrExtruder) 	// get the bitmap of driver step bits for this axis or extruder
{
	return driveDriverBits[axisOrExtruder];
}

//	unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions);

void Platform::SetDirection(size_t axisOrExtruder, bool direction)
{
	digitalWrite(DirectionPins[axisOrExtruder], direction);
}

EndStopHit Platform::GetZProbeResult()
{
	return EndStopHit::lowHit;
}

void Platform::EnableDrive(size_t axisOrExtruder)
{
	SmartDrivers::EnableDrive(axisOrExtruder, true);
}

//	void Platform::DisableDrive(size_t axisOrExtruder);
//	void Platform::DisableAllDrives();
//	void Platform::SetDriversIdle();

uint8_t Platform::ReadBoardSwitches()
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

uint8_t Platform::ReadBoardId()
{
	const uint8_t addr = ReadBoardSwitches() & 3;		// currently the top 2 bits are used to set motor current
	return (addr == 0) ? 4 : addr;
}

void Platform::Tick()
{
	//TODO
}

// End
