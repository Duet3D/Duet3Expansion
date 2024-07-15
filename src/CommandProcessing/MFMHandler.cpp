/*
 * MFMhandler.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "MFMHandler.h"

#if SUPPORT_AS5601

#include <Hardware/AS5601.h>
#include <Hardware/TCA6408A.h>
#include <Platform/TaskPriorities.h>
#include <InputMonitors/InputMonitor.h>

namespace MFMHandler
{
	// General constants and variables
	constexpr uint32_t MfmSampleIntervalMillis = 40;				// how many milliseconds between device polling
	constexpr size_t AS5601TaskStackWords = 100;

	Task<AS5601TaskStackWords> *as5601Task = nullptr;

	[[noreturn]] void MfmTaskCode(void*) noexcept;

	// Encoder variables
	constexpr unsigned int AngleBitsToDiscard = 2;

	AS5601 *encoder = nullptr;
	StandardCallbackFunction filamentMonitorCallbackFn = nullptr;
	FilamentMonitor *filamentMonitorCallbackObject = nullptr;
	volatile uint16_t angleReading;
	volatile bool readingAvailable = false;
	uint16_t lastAngle;
	uint8_t lastStatus = 0;
	uint8_t lastAgc = 0;

#if SUPPORT_TCA6408A
	// Expander variables
	constexpr uint8_t ButtonDebounceCount = 3;
	constexpr unsigned int ButtonBitnum = 0;
	constexpr unsigned int RedLedBitnum = 5;
	constexpr unsigned int GreenLedBitnum = 4;
	constexpr uint8_t LedBitsMask = (1u << RedLedBitnum) | (1u << GreenLedBitnum);
	constexpr uint8_t TCA8408A_initialOutputs = (1u << RedLedBitnum) | (1u << GreenLedBitnum);
	constexpr uint8_t TCA8408A_inputs = (1u << ButtonBitnum);		// button pin is input, LED and unused pins are outputs

	TCA6408A *expander = nullptr;
	InputMonitor *buttonMonitor = nullptr;
	uint8_t buttonCount = 0;										// incremented (up to max. buttonDebounceCount) when button is read down, decremented when button is read up
	bool buttonPressed = false;
#endif
}

// Initialise this module. Called from Platform initialisation.
void MFMHandler::Init(SharedI2CMaster& i2cDevice) noexcept
{
	if (encoder == nullptr)
	{
		encoder = new AS5601(i2cDevice);
		if (!encoder->Init())
		{
			DeleteObject(encoder);
		}
#if SUPPORT_TCA6408A
		expander = new TCA6408A(i2cDevice);
		if (!expander->Init(TCA8408A_inputs, TCA8408A_initialOutputs))
		{
			DeleteObject(expander);
		}
#endif
	}

	if (   encoder != nullptr
#if SUPPORT_TCA6408A
		|| expander != nullptr
#endif
	   )
	{
		as5601Task = new Task<AS5601TaskStackWords>();
		as5601Task->Create(MfmTaskCode, "MFM", nullptr, TaskPriority::MfmNormal);
	}
}

// Append diagnostics to a reply
void MFMHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	if (   EncoderPresent()
#if SUPPORT_TCA6408A
		&& ExpanderPresent()
#endif
	   )
	{
		reply.lcatf("Integrated filament monitor found, agc %u, magnet %sdetected", lastAgc, (lastStatus & AS5601::StatusMD) ? "" : "not ");
		if (lastStatus & AS5601::StatusMH) { reply.cat(", magnet too strong"); }
		if (lastStatus & AS5601::StatusML) { reply.cat(", magnet too weak"); }
	}
#if SUPPORT_TCA6408A
	else if (EncoderPresent() || ExpanderPresent())
	{
		reply.lcatf("Error: integrated AS5601%s found but TCA6408A%s found", (EncoderPresent()) ? "" : " not", ExpanderPresent() ? "" : " not");
	}
#endif
	else
	{
		reply.lcat("Integrated filament monitor not present");
	}
}

// Return true if the encoder was found during initialisation and can be used
bool MFMHandler::EncoderPresent() noexcept
{
	return encoder != nullptr;
}

#if SUPPORT_TCA6408A

// Return true if the I/O expander was found during initialisation and can be used
bool MFMHandler::ExpanderPresent() noexcept
{
	return expander != nullptr;
}

#endif

// This is called when a filament monitor that wants to use the encoder is configured. Return true if successful.
bool MFMHandler::AttachEncoderVirtualInterrupt(StandardCallbackFunction callback, FilamentMonitor *fm) noexcept
{
	if (encoder == nullptr) { return false; }						// MFM not present
	if (filamentMonitorCallbackFn != nullptr) { return false; }		// MFM already in use by a filament monitor

	TaskCriticalSectionLocker lock;
	filamentMonitorCallbackObject = fm;
	filamentMonitorCallbackFn = callback;
	return true;
}

// This is called when we delete the filament monitor that is using the encoder, or wanted to use it.
void MFMHandler::DetachEncoderVirtualInterrupt(FilamentMonitor *fm) noexcept
{
	TaskCriticalSectionLocker lock;
	if (filamentMonitorCallbackObject == fm)						// check that the filament monitor owns the encoder
	{
		filamentMonitorCallbackFn = nullptr;
		filamentMonitorCallbackObject = nullptr;
	}
}

// Fetch the previously stored reading. We emulate the standard MFM.
bool MFMHandler::GetEncoderReading(uint16_t& reading, uint8_t& agc, uint8_t& errorCode) noexcept
{
	if (readingAvailable)
	{
		reading = angleReading >> AngleBitsToDiscard;
		agc = lastAgc;
		// Map the encoder status to MFM error codes. See RotatingMagnetFilamentMonitor.cpp lines 150-165.
		// TODO name these status values
		errorCode = (lastStatus & AS5601::StatusML) ? 7
					: (lastStatus & AS5601::StatusMH) ? 8
						: !(lastStatus & AS5601::StatusMD) ? 6
							: 0;
		readingAvailable = false;
		return true;
	}
	return false;
}

uint16_t MFMHandler::GetLastAngle() noexcept { return lastAngle; }

#if SUPPORT_TCA6408A

// Enable the button and store a pointer to the input monitor to call back, or disable the button if nullptr is passed
bool MFMHandler::EnableButton(InputMonitor *monitor) noexcept
{
	buttonMonitor = monitor;
	return buttonPressed;
}

// Set the LED to red. Note, LED bits are inverted.
void MFMHandler::SetLedRed() noexcept
{
	if (expander != nullptr) { expander->SetOutputBitsState(1u << GreenLedBitnum, LedBitsMask); }
}

// Set the LED to green. Note, LED bits are inverted.
void MFMHandler::SetLedGreen() noexcept
{
	if (expander != nullptr) { expander->SetOutputBitsState(1u << RedLedBitnum, LedBitsMask); }
}

// Set the LED to red+green. Note, LED bits are inverted.
void MFMHandler::SetLedBoth() noexcept
{
	if (expander != nullptr) { expander->SetOutputBitsState(0, LedBitsMask); }
}

// Set the LED to off. Note, LED bits are inverted.
void MFMHandler::SetLedOff() noexcept
{
	if (expander != nullptr) { expander->SetOutputBitsState((1u << RedLedBitnum) | (1u << GreenLedBitnum), LedBitsMask); }
}

#endif

// Code executed by the task that polls the AS5601 and TC6408A
void MFMHandler::MfmTaskCode(void *) noexcept
{
	uint32_t nextWakeTime = millis();
	for (;;)
	{
		// Wait until we are woken or it's time to poll the I2C devices. If we are really unlucky, we could end up waiting for one tick too long.
		nextWakeTime += MfmSampleIntervalMillis;
		{
			const uint32_t now = millis();
			int32_t delayTime = (int32_t)(nextWakeTime - now);
			// If we're late (e.g. due to problems sending CAN messages), wait at least until the next tick to let other tasks run
			if (delayTime < 1)
			{
				nextWakeTime = now + 1;
				delayTime = 1;
			}
			delay((uint32_t)delayTime);
		}

		if (encoder != nullptr)
		{
			TaskBase::SetCurrentTaskPriority(TaskPriority::MfmHigh);
			uint16_t tempAngle;
			if (encoder->Read(tempAngle, lastStatus, lastAgc))
			{
				lastAngle = ~tempAngle & 0x0FFF;			// invert the angle because the Roto MFM runs backwards with positive extrusion
				if (!readingAvailable && filamentMonitorCallbackFn != nullptr)
				{
					angleReading = lastAngle;
					filamentMonitorCallbackFn(CallbackParameter(filamentMonitorCallbackObject));
					readingAvailable = true;
				}
			}
			TaskBase::SetCurrentTaskPriority(TaskPriority::MfmNormal);
		}

#if SUPPORT_TCA6408A
		if (expander != nullptr)
		{
			// Read and debounce the button
			expander->Poll();
			const bool pressedNow = !(expander->GetInputRegister() & (1u << ButtonBitnum));
			if (pressedNow == buttonPressed)
			{
				buttonCount = 0;
			}
			else
			{
				++buttonCount;
				if (buttonCount == ButtonDebounceCount)
				{
					buttonPressed = pressedNow;
					buttonCount = 0;
					TaskCriticalSectionLocker lock;
					if (buttonMonitor != nullptr)
					{
						buttonMonitor->UpdateState(buttonPressed);
					}
				}
			}
		}
#endif
	}
}

#endif
