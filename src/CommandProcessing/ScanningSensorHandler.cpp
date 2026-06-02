/*
 * ScanningSensorHandler.cpp
 *
 *  Created on: 16 Jun 2023
 *      Author: David
 * 		Major changes to touch detection: Andy
 *
 *  This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "ScanningSensorHandler.h"

#if SUPPORT_LDC1612

#include <InputMonitors/InputMonitor.h>
#include <I2C/SharedI2CMaster.h>
#include <Hardware/Drivers/LDC1612.h>
#include <CanMessageFormats.h>
#include <AnalogIn.h>
#include <Movement/StepTimer.h>
#include <Platform/TaskPriorities.h>
#include <Platform/AveragingFilter.h>
#include <AppNotifyIndices.h>
#include <Interrupts.h>

constexpr unsigned int ResultBitsDropped = 8;		// we drop this number of least significant bits in the result

constexpr unsigned int LdcTaskStackWords = 150;		// 100 was too little

static Task<LdcTaskStackWords> *ldcTask = nullptr;

static LDC1612 *sensor = nullptr;
static volatile uint32_t lastReading = 0;
static volatile bool isCalibrating = false;
static uint32_t lastReadingTakenAt = 0;
static uint32_t offset = 0;
static InputMonitor *volatile inputMonitor = nullptr;	// when the sensor is active this point to the associated input monitor; when inactive it is null

namespace TouchMode
{
//private:
	// Butterworth bandpass filter code and coefficients borrowed from https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
	//
	// The coefficients can be generated using the following python code, it assumes a sample rate of 500 samples/s:
	// sos: List[List[float]] = None
	// sos = scipy.signal.butter(
	//            2,
	//            [5, 25],
	//            btype="bandpass",
	//            fs=500,
	//            output="sos",
	//        ).tolist()
	// print(sos)

	// Notes on touch sensing:
	// The touch sensing code although using the Butterworth filter as used by Klipper is significantly different in how it detects
	// a touch event. In particular we no longer look for a peak before the event, instead just detect the rapid fall in the
	// output of the filter. This seems to provide a faster response and also avoids some false positives.
	static const size_t sosSections = 2;
	static float sosState[sosSections][2];
	static constexpr float sosButterworthFilter500[sosSections][6] =
			{
				{	0.013359200027856505,
					0.02671840005571301,
					0.013359200027856505,
					1.0,
					-1.686278256753083,
					0.753714473246724
				 },
				 {	1.0,
					-2.0,
					1.0,
					1.0,
					-1.9250515947328444,
					0.9299234737648037
				 }
			};
	static float SosFilter(float value, const float filter[][6], float state[][2]) noexcept;
	static uint32_t baseReading;
	static float lastValue;
	static float startValue;
	static bool falling;
	static size_t goodCnt;						// for debug use
	static float threshold;
	static bool enabled = false;
	static uint32_t startTime;					// the time we started taking touch mode readings, in step clocks
	static uint32_t lastReading;				// the previous reading
	static unsigned int numBadReadings;

//public:
	static void Start(uint32_t p_threshold) noexcept;
	static void Stop() noexcept;
	static void ProcessReading(uint32_t reading) noexcept;
	static bool IsEnabled() noexcept { return enabled; }
};

// The incoming parameter p_threshold holds a value in the lower 16 bits, where 0x0000FFFF represents the maximum configurable threshold, which is TouchModeMaxThreshold.
void TouchMode::Start(uint32_t p_threshold) noexcept
{
	lastReading = baseReading = 0;
	numBadReadings = 0;
	startTime = StepTimer::GetTimerTicks();
	enabled = true;
	for(size_t i = 0; i < sosSections; i++)
	{
		sosState[i][0] = 0.0f;
		sosState[i][1] = 0.0f;
	}
	lastValue = startValue = 0.0f;
	falling = false;

	// The following scaling makes a configured sensitivity of 0.5 about right on DC's toolchanger
	threshold = ((31250.0 * TouchModeMaxThreshold)/(65535.0 * LDC1612::FRef)) * (float)(p_threshold & 0x0000FFFF);

	//debugPrintf("Rec threshold %" PRIu32 ", scaled %.0f\n", p_threshold, (double)threshold);
	goodCnt = 0;
	enabled = true;
}

void TouchMode::Stop() noexcept
{
	enabled = false;
}

// Butterworth bandpass filter code and coefficients borrowed from https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
float TouchMode::SosFilter(float value, const float filter[][6], float state[][2]) noexcept
{
	for (size_t i = 0; i < sosSections; i++)
	{
		const float w1 = state[i][0];
		const float w2 = state[i][1];
		const float w0 = value - filter[i][4]*w1 - filter[i][5]*w2;
		value = filter[i][0]*w0 + filter[i][1]*w1 + filter[i][2]*w2;
		state[i][0] = w0;
		state[i][1] = w1;
	}
	return value;
}

// Process a sensor reading when we are in touch mode
// A typical probing speed is 5mm/sec. At this speed, a processing interval of 1ms will give us a probing resolution of 5um.
void TouchMode::ProcessReading(uint32_t reading) noexcept
{
	if ((reading & 0xE0000000) != 0)								// if it's a bad reading (ignoring amplitude errors)
	{
		++numBadReadings;
		if (numBadReadings == 3)									// if we get 3 bad readings in a row, give up
		{
			InputMonitor *const locInputMonitor = inputMonitor;		// capture volatile variable
			if (locInputMonitor != nullptr)
			{
				locInputMonitor->SetTriggered();
			}
//			debugPrintf("Bad reading %08" PRIx32 "\n", reading);
			Stop();
		}
	}
	else
	{
		reading &= 0x0FFFFFFF;										// clear Amplitude Error bit
		const uint32_t now = StepTimer::GetTimerTicks();

		// Butterworth bandpass filter code and coefficients borrowed from see https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
		if (now - startTime >= StepTimer::StepClockRate/10)			// allow for the movement start delay and some more
		{
			const float value = SosFilter((float)(int32_t)(reading - baseReading), sosButterworthFilter500, sosState);
			//debugPrintf("%d F %" PRIi32 " V %f\n", goodCnt++, (int32_t)(reading - baseReading), (double)value);
			if (now - startTime >= StepTimer::StepClockRate/4)		// allow probing speed and filter to stabilise before we look at the output

			{
				if (value < lastValue)
				{
					if (falling)
					{
						if (-value >= threshold)
						{
							InputMonitor *const locInputMonitor = inputMonitor;		// capture volatile variable
							if (locInputMonitor != nullptr)
							{
								locInputMonitor->SetTriggered();
							}
							Stop();
							//debugPrintf("%d Trig F %" PRIi32 " V %f LV %f SV %f BV %" PRIu32 " TH %f\n", goodCnt++, (int32_t)(reading - baseReading), (double)value, (double)lastValue, (double)startValue, baseReading, (double)threshold);
						}
					}
					falling = true;
				}
				else if (value > lastValue)
				{
					falling = false;
					startValue = value;
				}
				lastValue = value;
			}
		}
		else
		{
			baseReading = reading;
		}
		numBadReadings = 0;
	}
}

[[noreturn]] static void LdcTaskLoop(void* param) noexcept;

// Activate the scanning sensor returning true if successful
bool ScanningSensorHandler::Activate(InputMonitor& monitor) noexcept
{
	if (sensor == nullptr)
	{
		return false;
	}

	if (ldcTask == nullptr)
	{
		ldcTask = new Task<LdcTaskStackWords>;
		ldcTask->Create(LdcTaskLoop, "ScanSens", nullptr, TaskPriority::LdcTask);
	}
	inputMonitor = &monitor;
	return true;
}

void ScanningSensorHandler::Deactivate()
{
	inputMonitor = nullptr;
}

// Align this on a cache line boundary for SAMC21
__attribute__ ((aligned (8))) static void Ldc1612Interrupt(CallbackParameter) noexcept
{
	TaskBase::GiveFromISR(ldcTask, NotifyIndices::LDC1612);
	DisablePinInterrupt(LDC1612InterruptPin);
}

// Function executed by the LDC task
[[noreturn]] static void LdcTaskLoop(void* param) noexcept
{
	AttachPinInterrupt(LDC1612InterruptPin, Ldc1612Interrupt, InterruptMode::low, CallbackParameter(), false);
	for (;;)
	{
		if (inputMonitor == nullptr || isCalibrating)
		{
			delay(5);
		}
		else
		{
			if (sensor->IsChannelReady(0))					// this also clears the interrupt
			{
				uint32_t val;
				if (sensor->GetChannelResult(0, val))		// if no error
				{
					lastReading = val;						// save all 28 bits of data + 4 error bits
					lastReadingTakenAt = millis();			// record when we took it
					if (TouchMode::IsEnabled())
					{
						TouchMode::ProcessReading(val);
					}
					else
					{
						// The input monitor may have been deactivated and inputMonitor set to nullptr while we were getting the reading, so we need to check it again here
						InputMonitor *const locInputMonitor = inputMonitor;
						if (locInputMonitor != nullptr)
						{
							locInputMonitor->AnalogInterrupt(ScanningSensorHandler::GetReading());
						}
					}
				}
				else if (millis() - lastReadingTakenAt > 5)	// we get occasional reading errors, so don't report a bad reading unless it's 5ms since we had a good reading
				{
					lastReading = 0;
				}
			}

			EnablePinInterrupt(LDC1612InterruptPin);
			TaskBase::TakeIndexed(NotifyIndices::LDC1612, 5);
		}
	}
}

void ScanningSensorHandler::Init(SharedI2CMaster& i2cDevice) noexcept
{
	// Set up the external clock to the LDC1612.
	// The higher the better, but the maximum is 40MHz
#if defined(SAMMYC21) || defined(TOOL1LC) || defined(SHT36) || defined(FLYSB2040V3_0) || defined(FYSETCSB2040V2)
	// Assume we are using a LDC1612 breakout board with its own crystal, so we don't need to generate a clock
#elif defined(SZP)
	// We can use the 96MHz DPLL output divided by 3 to get 32MHz but it is probably better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 32.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::xosc, 1, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
	}
	else if constexpr(LDC1612::ClockFrequency == 32.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::dpll, 3, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
	}
#elif defined(TOOL1RR) || defined(TOOLINDX)
	// We use the 120MHz DPLL output divided by 4 to get 30MHz. It might be better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 30.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(LDC1612GClkNumber, (GclkSource)((uint8_t)GclkSource::xosc0 + AppGetXoscNumber()), 1, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
	}
	else if constexpr(LDC1612::ClockFrequency == 30.0)
	{
		ConfigureGclk(LDC1612GClkNumber, GclkSource::dpll0, 4, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
	}
#else
# error LDC support not implemented for this processor
#endif

	sensor = new LDC1612(i2cDevice);

	// Soft-reset before probing: the chip stays powered across MCU resets and can be stuck in an internal state that
	// survives our reset but clears on power-cycle. Its I2C front-end stays responsive in those states, so this write
	// lands even when the ID register reads would otherwise return garbage
	(void)sensor->Reset();
	delay(2);

	if (sensor->CheckPresent())
	{
		sensor->SetDefaultConfiguration(0, false);
		lastReadingTakenAt = millis();
		SetPinMode(LDC1612InterruptPin, PinMode::INPUT, false);
	}
	else
	{
		DeleteObject(sensor);
#if defined(TOOL1LC) || defined(SZP)
		ClearPinFunction(LDC1612ClockGenPin);
#endif
	}
}

bool ScanningSensorHandler::IsPresent() noexcept
{
	return sensor != nullptr;
}

uint32_t ScanningSensorHandler::GetReading() noexcept
{
	if ((lastReading & 0xF0000000) != 0) { return ScanningSensorBadReadingVal; }
	const uint32_t reading = lastReading >> ResultBitsDropped;
	return (reading > offset) ? reading - offset : 0;
}

GCodeResult ScanningSensorHandler::SetOrCalibrateCurrent(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor == nullptr)
	{
		reply.copy("scanning probe not present");
	}
	else
	{
		if (param == CanMessageChangeInputMonitorV1::paramAutoCalibrateDriveLevelAndReport)
		{
			isCalibrating = true;
			delay(2);												// avoid race with LDC task
			bool ok = sensor->CalibrateDriveCurrent(0);
			if (ok)
			{
				extra = sensor->GetDriveCurrent(0);
				delay(4);											// give time for a reading to become available
				uint32_t val;
				ok = sensor->GetChannelResult(0, val);
				isCalibrating = false;
				if (ok && (val & 0xF0000000) == 0)
				{
					val >>= ResultBitsDropped;
					offset = val - (val >> 4);						// set the offset to 15/16 of the reading
					reply.printf("Calibration successful, sensor drive current is %u, offset is %" PRIu32, extra, offset);
					return GCodeResult::ok;
				}
			}
			isCalibrating = false;
			reply.copy("failed to calibrate sensor drive current");
		}
		else if (param == CanMessageChangeInputMonitorV1::paramReportDriveLevel)
		{
			extra = sensor->GetDriveCurrent(0);
			reply.printf("Sensor drive current is %u, offset is %" PRIu32, extra, offset);
			return GCodeResult::ok;
		}
		else
		{
			const uint16_t driveCurrent = param & CanMessageChangeInputMonitorV1::paramDriveLevelMask;
			const uint32_t newOffset = param >> CanMessageChangeInputMonitorV1::paramOffsetShift;
			isCalibrating = true;
			delay(2);												// avoid race with LDC task
			const bool ok = sensor->SetDriveCurrent(0, driveCurrent);
			isCalibrating = false;
			if (ok)
			{
				offset = newOffset;
				extra = param;
				return GCodeResult::ok;
			}
			reply.copy("failed to set sensor drive current");
		}
	}
	extra = 0xFF;
	return GCodeResult::error;
}

GCodeResult ScanningSensorHandler::SelectTouchMode(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor == nullptr)
	{
		reply.copy("scanning probe not present");
	}
	else
	{
		TouchMode::Start(param);
		return GCodeResult::ok;
	}
	extra = 0xFF;
	return GCodeResult::error;
}

void ScanningSensorHandler::ClearTouchMode() noexcept
{
	TouchMode::Stop();
}

// Return the oscillation frequency in MHz
float ScanningSensorHandler::GetFrequency() noexcept
{
	return ldexpf((lastReading & 0x0FFFFFFF) * LDC1612::FRef, -28);
}

void ScanningSensorHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcat("Inductive sensor: ");
	if (IsPresent())
	{
		if (ldcTask == nullptr)
		{
			reply.cat("never activated");
		}
		else if (inputMonitor == nullptr)
		{
			reply.cat("not currently active");
		}
		else
		{
			// Append diagnostic data to string
			const uint32_t val = lastReading;
			if (val != 0)
			{
				reply.catf("raw value %" PRIu32 ", frequency %.2fMHz, current setting %u", val & 0x0FFFFFFF, (double)GetFrequency(), sensor->GetDriveCurrent(0));
				if ((val >> 28) == 0)
				{
					reply.cat(", ok");
				}
				else
				{
					if ((val >> 28) & LDC1612::ERR_UR0)
					{
						reply.cat(", under-range error");
					}
					if ((val >> 28) & LDC1612::ERR_OR0)
					{
						reply.cat(", over-range error");
					}
					if ((val >> 28) & LDC1612::ERR_WD0)
					{
						reply.cat(", watchdog error");
					}
					if ((val >> 28) & LDC1612::ERR_AE0)
					{
						reply.cat(", amplitude error");
					}
				}
			}
			else
			{
				reply.cat("error retrieving data from LDC1612");
			}
		}
	}
	else
	{
		reply.cat("not found");
	}
}

#endif

// End
