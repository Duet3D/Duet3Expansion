/*
 * ADS131M02.cpp
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#include "ADS131M02.h"

#if SUPPORT_ADS131M02

#include <Platform/TaskPriorities.h>
#include <Hardware/IoPorts.h>
#include <RTOSIface/RTOSIface.h>
#include <AppNotifyIndices.h>

#if SUPPORT_LOADCELL_DIAGNOSTICS
# include <CommandProcessing/LoadCellDiagnostics.h>
#endif

#define USE_GLOBAL_CHOP		(1)

// The 24-bit conversion results are stored in the top 24 bits of a 32-bit word, so full scale is 2^31. Treat readings above about 97% of
// full scale as saturated - the input is overdriven at gain 128, which means the cell is overloaded, disconnected or faulty
constexpr int32_t SaturationThreshold = 0x7C000000;

extern "C" [[noreturn]] void AdcTaskStart(void* param) noexcept
{
	((ADS131M02*)param)->TaskLoop();
}

ADS131M02::ADS131M02() noexcept : SpiDevice(Ads131M02SpiParams)
{
	SetPinMode(ADS131M02_CsPin, OUTPUT_HIGH);
	SetPinMode(ADS131M02_DRDYPin, INPUT);
	ConfigureGclk(ADA131M02_GclkNumber, GclkSource::dpll0, 120/8, true);			// set up 8MHz clock
	SetDriveStrength(ADS131M02_GclkPin, 1);											// set high drive strength on clock pin
	SetPinFunction(ADS131M02_GclkPin, ADS131M02_GclkPinFunction);					// enable 8MHz clock on output pin

	SetClockFrequencyAndMode(6'000'000, SpiMode::mode1
#if SAME5x
								, false												// don't use 9-bit mode
#endif
							);														// 6MHz SPI clock

	// Reset the device
	if (!SendSimpleCommand(Ads131M02Command::reset)) return;
	delayMicroseconds(10);															// need to delay at least 5us after a reset
	if (!ReadRegister(Ads131M02Register::id)) return;								// get the response to reset and read the ID register
	if (rslt != 0xFF22) return;
	if (!SendSimpleCommand(Ads131M02Command::nullCmd)) return;
	if ((rslt >> 8) != 0x22) return;

	// Device has been reset and identified OK so initialise it
	if (!WriteRegister(Ads131M02Register::mode,  (1u << 8))) return;										// 24 bit word length, no CRCs
	if (!WriteRegister(Ads131M02Register::gain,  (3u << 4) | (7u << 0))) return;							// channel 0 gain 128, channel 1 gain 8
#if USE_GLOBAL_CHOP
	// If we enable global chop then we need to reduce OSR from 4096 to 1024 to preserve the data rate. We get slightly less resolution but still about 14 bits.
	if (!WriteRegister(Ads131M02Register::cfg,   (3u << 9) | (1u << 8))) return;							// global chop delay enabled
	if (!WriteRegister(Ads131M02Register::clock, (1u << 9) | (1u << 8) | (3u << 2) | (2u << 0))) return;	// enable both channels, oversampling 1024, high resolution
#else
	// With global chop disabled, OSR of 4096 with an 8MHz clock gives us just under one sample per millisecond.
	// Input noise with gain 128 is 0.77uV RMS and the effective resolution is 14.6 bits.
	if (!WriteRegister(Ads131M02Register::cfg,   (3u << 9))) return;										// global chop delay disabled
	if (!WriteRegister(Ads131M02Register::clock, (1u << 9) | (1u << 8) | (5u << 2) | (2u << 0))) return;	// enable both channels, oversampling 4096, high resolution
#endif

	// We don't need to write the channel registers, the defaults are good
	initOk = true;
	adcTask = new Task<TaskStackWords>;
	adcTask->Create(AdcTaskStart, "ADS131M02", (void*)this, TaskPriority::Ads131M02);
}

bool ADS131M02::Activate(InputMonitor& monitor) noexcept
{
	if (!initOk)
	{
		return false;
	}
	inputMonitor = &monitor;
	return true;
}

void ADS131M02::Deactivate() noexcept
{
	inputMonitor = nullptr;
}

static void DataReadyCallback(CallbackParameter param) noexcept
{
	TaskBase *const task = (TaskBase *)param.vp;
	TaskBase::GiveFromISR(task, NotifyIndices::Ads131M02);
	DisablePinInterrupt(ADS131M02_DRDYPin);
}

[[noreturn]] void ADS131M02::TaskLoop() noexcept
{
	AttachPinInterrupt(ADS131M02_DRDYPin, DataReadyCallback, InterruptMode::low, CallbackParameter(this->adcTask), false);
	for (;;)
	{
		if (!IoPort::ReadPin(ADS131M02_DRDYPin))
		{
			do
			{
				// Read the data until the fifo is empty
				if (!SendSimpleCommand(Ads131M02Command::nullCmd))
				{
					// This should never happen
					delay(10);
				}
			} while (!IoPort::ReadPin(ADS131M02_DRDYPin));

			// Calculate the load cell output
			/*const uint32_t*/ channel0Data = ((uint32_t)regReadBuffer[3] << 24) | ((uint32_t)regReadBuffer[4] << 16) | ((uint32_t)regReadBuffer[5] << 8);
			/*const uint32_t*/ channel1Data = ((uint32_t)regReadBuffer[6] << 24) | ((uint32_t)regReadBuffer[7] << 16) | ((uint32_t)regReadBuffer[8] << 8);
			if (channel1Data < 256)
			{
				compositeData = 0;					// error, the reference channel should read much higher, such a low value may cause the result of the next division to exceed 32 bits
			}
			else if ((int32_t)channel0Data >= SaturationThreshold || (int32_t)channel0Data <= -SaturationThreshold)
			{
				compositeData = 0;					// the signal is at the ADC rail, so the reading is meaningless
			}
			else
			{
				compositeData = (int32_t)(((int64_t)(int32_t)channel0Data << 16)/(int32_t)channel1Data);
			}

#if SUPPORT_LOADCELL_DIAGNOSTICS
			LoadCellDiagnostics::RecordSample(compositeData);
#endif

			// The input monitor may have been deactivated and inputMonitor set to nullptr while we were reading the data, so capture it before we test it
			InputMonitor *const locInputMonitor = inputMonitor;
			if (locInputMonitor != nullptr)
			{
				locInputMonitor->AnalogInterrupt(compositeData);
			}
		}

		// Wait for data to become ready again
		EnablePinInterrupt(ADS131M02_DRDYPin);
		TaskBase::TakeIndexed(NotifyIndices::Ads131M02, 10);
	}
}

// Send a simple command, not a read or write register
bool ADS131M02::SendSimpleCommand(Ads131M02Command cmd) noexcept
{
	regWriteBuffer[0] = (uint16_t)cmd >> 8;
	regWriteBuffer[1] =  (uint16_t)cmd & 0xFF;
	regWriteBuffer[2] =  0;
	memset(regWriteBuffer + 3, 0, sizeof(regWriteBuffer) - 3);
	IoPort::WriteDigital(ADS131M02_CsPin, false);
	const bool ret = TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
	IoPort::WriteDigital(ADS131M02_CsPin, true);
	rslt = ((uint16_t)regReadBuffer[0] << 8) | regReadBuffer[1];
	return ret;
}

// Send the command to read a 16-bit register, need to do another transfer to actually read it
bool ADS131M02::ReadRegister(Ads131M02Register regNum) noexcept
{
	const uint16_t cmd = (uint16_t)Ads131M02Command::rreg | ((uint16_t)regNum << 7);
	regWriteBuffer[0] = cmd >> 8;
	regWriteBuffer[1] =  cmd & 0xFF;
	regWriteBuffer[2] =  0;
	memset(regWriteBuffer + 3, 0, sizeof(regWriteBuffer) - 3);
	IoPort::WriteDigital(ADS131M02_CsPin, false);
	const bool ret = TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
	IoPort::WriteDigital(ADS131M02_CsPin, true);
	rslt = ((uint16_t)regReadBuffer[0] << 8) | regReadBuffer[1];
	return ret;
}

// Write a 16-bit register
bool ADS131M02::WriteRegister(Ads131M02Register regNum, uint16_t val) noexcept
{
	const uint32_t cmd = (uint32_t)Ads131M02Command::wreg | ((uint32_t)regNum << 7);
	regWriteBuffer[0] = cmd >> 8;
	regWriteBuffer[1] =  cmd & 0xFF;
	regWriteBuffer[2] =  0;
	regWriteBuffer[3] = val >> 8;
	regWriteBuffer[4] = val & 0xFF;
	memset(regWriteBuffer + 5, 0, sizeof(regWriteBuffer) - 5);
	IoPort::WriteDigital(ADS131M02_CsPin, false);
	const bool ret = TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
	IoPort::WriteDigital(ADS131M02_CsPin, true);
	rslt = ((uint16_t)regReadBuffer[0] << 8) | regReadBuffer[1];
	return ret;
}

#endif
