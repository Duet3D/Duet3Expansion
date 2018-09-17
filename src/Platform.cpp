/*
 * Platform.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "Platform.h"

#include "HAL/IoPorts.h"
#include "HAL/AnalogIn.h"
#include <Movement/Move.h>
#include "Movement/StepperDrivers/TMC51xx.h"
#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>
#include "AdcAveragingFilter.h"
#include "Movement/StepTimer.h"

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

	uint32_t driveDriverBits[DRIVES];
	uint32_t allDriverBits = 0;

	static float stepsPerMm[DRIVES] = { 80.0, 80.0, 80.0 };
	static float axisMinima[MaxAxes] = { 0.0, 0.0, 0.0 };
	static float axisMaxima[MaxAxes] = { 500.0, 500.0, 500.0 };
	static float accelerations[DRIVES] = { 1000.0, 1000.0, 1000.0 };
	static float feedrates[DRIVES] = { 100.0, 100.0, 100.0 };			// mm/sec
	static float instantDVs[DRIVES] = { 5.0, 5.0, 5.0 };				// mm/sec

	static uint32_t lastFlashTime;

	const size_t NumReadingsAveraged = 32;

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

	uint16_t temp_cal_tl, temp_cal_th;
	uint16_t temp_cal_vpl, temp_cal_vph, temp_cal_vcl, temp_cal_vch;

	// SERCOM3 Rx is on PB21 (OUT_8_TACHO), Tx is on PB20 (OUT_7_TACHO)

	static AdcAveragingFilter<NumReadingsAveraged> vinFilter;
	static AdcAveragingFilter<NumReadingsAveraged> tpFilter;
	static AdcAveragingFilter<NumReadingsAveraged> tcFilter;

	void ADC_temperature_init(void)
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

	void Init()
	{
		// Set up the DIAG LED pin
		gpio_set_pin_direction(DiagLedPin, GPIO_DIRECTION_OUT);
		gpio_set_pin_level(DiagLedPin, true);

		// Set up the UART to send to PanelDue
		const uint32_t baudDiv = 65536 - ((65536 * 16.0f * 57600) / CONF_GCLK_SERCOM3_CORE_FREQUENCY);
		usart_async_set_baud_rate(&USART_0, baudDiv);

		usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_USART_0);
		//usart_async_register_callback(&USART_0, USART_ASYNC_RXC_CB, rx_cb);
		//usart_async_register_callback(&USART_0, USART_ASYNC_ERROR_CB, err_cb);
		usart_async_get_io_descriptor(&USART_0, &io);
		usart_async_enable(&USART_0);

		AnalogIn::Init();
		ADC_temperature_init();

		for (unsigned int i = 0; i < 4; ++i)
		{
			pinMode(BoardAddressPins[i], INPUT_PULLUP);
		}

		// Set up ADC to read VIN and the temperature sensors
		vinFilter.Init(0);
		tpFilter.Init(0);
		tcFilter.Init(0);
		AnalogIn::EnableChannel(VinMonitorPin, vinFilter.CallbackFeedIntoFilter, &vinFilter);
		AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, &tpFilter, 1);
		AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, &tcFilter, 1);

		SmartDrivers::Init();

		for (size_t i = 0; i < DRIVES; ++i)
		{
			pinMode(StepPins[i], OUTPUT_LOW);
			pinMode(DirectionPins[i], OUTPUT_LOW);
			const uint32_t driverBit = 1u << (StepPins[i] & 31);
			driveDriverBits[i] = driverBit;
			allDriverBits |= driverBit;

			SmartDrivers::SetMicrostepping(i, 16, true);
		}

		lastFlashTime = millis();
	}

	void Spin()
	{
		static uint8_t oldAddr = 0;
		static bool powered = false;

		// Get the VIN voltage
		const float volts = (vinFilter.GetSum() * (3.3 * 11))/(4096 * NumReadingsAveraged);
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

		const uint32_t now = millis();
		if (now - lastFlashTime > 500)
		{
			lastFlashTime = now;

			gpio_toggle_pin_level(DiagLedPin);

			const uint8_t addr = ReadBoardAddress();
			if (addr != oldAddr)
			{
				oldAddr = addr;
				const float current = (addr >= 12) ? 3000.0 : (addr >= 8) ? 2000.0 : (addr >= 4) ? 1000.0 : 500.0;
				for (size_t i = 0; i < DRIVES; ++i)
				{
					SmartDrivers::SetCurrent(i, current);
				}
			}

			// Get the chip temperature
			const uint16_t tc_result = tcFilter.GetSum()/(NumReadingsAveraged);
			const uint16_t tp_result = tpFilter.GetSum()/(NumReadingsAveraged);

			int32_t result = (int64_t)temp_cal_tl * temp_cal_vph * tc_result
							- (int64_t)temp_cal_vpl * temp_cal_th * tc_result
							- (int64_t)temp_cal_tl * temp_cal_vch * tp_result
							+ (int64_t)temp_cal_th * temp_cal_vcl * tp_result;
			const int32_t divisor = ((int32_t)temp_cal_vcl * tp_result - (int32_t)temp_cal_vch * tp_result
			           	   	   - (int32_t)temp_cal_vpl * tc_result + (int32_t)temp_cal_vph * tc_result);
			result = (divisor == 0) ? 0 : result/divisor;
			const float temperature = (float)result/16;

#if 1
			String<100> status;
			SmartDrivers::AppendDriverStatus(2, status.GetRef());
			debugPrintf("%s", status.c_str());
#elif 0
			moveInstance->Diagnostics(AuxMessage);
#else
			uint32_t conversionsStarted, conversionsCompleted;
			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted);
			debugPrintf(
//							"Conv %u %u"
							"Addr %u"
							", %.1fV, %.1fdegC"
							", ptat %d, ctat %d"
							", stat %08" PRIx32 " %08" PRIx32 " %08" PRIx32,
//							(unsigned int)conversionsStarted, (unsigned int)conversionsCompleted,
//							StepTimer::GetInterruptClocks(),
							(unsigned int)addr,
							(double)volts, (double)temperature
							, tp_result, tc_result
							, SmartDrivers::GetAccumulatedStatus(0, 0), SmartDrivers::GetAccumulatedStatus(1, 0), SmartDrivers::GetAccumulatedStatus(2, 0)
//							, SmartDrivers::GetLiveStatus(0), SmartDrivers::GetLiveStatus(1), SmartDrivers::GetLiveStatus(2)
						);
#endif

		}
	}

	// Send the specified message to the specified destinations. The Error and Warning flags have already been handled.
	void RawMessage(MessageType type, const char *message)
	{
		static String<200> buffer;
		buffer.copy("{\"message\":\"");
		buffer.cat(message);		// should do JSON escaping here
		buffer.cat("\"}\n");
		txBusy = true;
		io_write(io, (const unsigned char *)buffer.c_str(), buffer.strlen());
		// io_write requires that the message doesn't go out of scope until transmission is complete
		//while (txBusy) { delay(1); }
	}

	void MessageF(MessageType type, const char *fmt, va_list vargs)
	{
		String<FORMAT_STRING_LENGTH> formatString;
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

	void Message(MessageType type, const char *message)
	{
		if ((type & (ErrorMessageFlag | WarningMessageFlag)) == 0)
		{
			RawMessage(type, message);
		}
		else
		{
			String<FORMAT_STRING_LENGTH> formatString;
			formatString.copy(((type & ErrorMessageFlag) != 0) ? "Error: " : "Warning: ");
			formatString.cat(message);
			RawMessage((MessageType)(type & ~(ErrorMessageFlag | WarningMessageFlag)), formatString.c_str());
		}
	}

	void LogError(ErrorCode e)
	{
		errorCodeBits |= (uint32_t)e;
	}

	bool Debug(Module module)
	{
		return false;
	}

	void SetDriversIdle() { }

	float DriveStepsPerUnit(size_t drive) { return stepsPerMm[drive]; }

	const float *GetDriveStepsPerUnit() { return stepsPerMm; }

	float AxisMaximum(size_t axis) { return axisMaxima[axis]; }
//	void SetAxisMaximum(size_t axis, float value, bool byProbing);
	float AxisMinimum(size_t axis) { return axisMinima[axis]; }
//	void SetAxisMinimum(size_t axis, float value, bool byProbing);
//	float AxisTotalLength(size_t axis) ;
	float GetPressureAdvance(size_t extruder) { return 0.4; }
//	void SetPressureAdvance(size_t extruder, float factor);
	float Acceleration(size_t axisOrExtruder) { return accelerations[axisOrExtruder]; }
	const float* Accelerations() { return accelerations; }
//	void SetAcceleration(size_t axisOrExtruder, float value);
	float MaxFeedrate(size_t axisOrExtruder) { return feedrates[axisOrExtruder]; }
	const float* MaxFeedrates() { return feedrates; }
//	void SetMaxFeedrate(size_t axisOrExtruder, float value);
	float GetInstantDv(size_t axis) { return instantDVs[axis]; }
//	void SetInstantDv(size_t axis, float value);
	EndStopHit Stopped(size_t axisOrExtruder) { return EndStopHit::lowHit; }
	bool EndStopInputState(size_t axis) { return false; }

	void StepDriversLow()
	{
		StepPio->OUTCLR.reg = allDriverBits;
	}

	void StepDriversHigh(uint32_t driverMap)
	{
		StepPio->OUTSET.reg = driverMap;
	}

//	uint32_t CalcDriverBitmap(size_t driver);

	uint32_t GetDriversBitmap(size_t axisOrExtruder) 	// get the bitmap of driver step bits for this axis or extruder
	{
		return driveDriverBits[axisOrExtruder];
	}

//	unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions);

	void SetDirection(size_t axisOrExtruder, bool direction)
	{
		digitalWrite(DirectionPins[axisOrExtruder], direction);
	}

	EndStopHit GetZProbeResult()
	{
		return EndStopHit::lowHit;
	}

	void EnableDrive(size_t axisOrExtruder)
	{
		SmartDrivers::EnableDrive(axisOrExtruder, true);
	}

//	void DisableDrive(size_t axisOrExtruder);
//	void DisableAllDrives();
//	void SetDriversIdle();

	uint8_t ReadBoardAddress()
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
}

// End
