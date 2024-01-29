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
	AS5601 *encoder = nullptr;
	StandardCallbackFunction filamentMonitorCallbackFn = nullptr;
	FilamentMonitor *filamentMonitorCallbackObject = nullptr;
	volatile uint16_t angleReading;
	volatile bool readingAvailable = false;
	uint16_t lastAngle;
	uint8_t lastStatus = 0;
	uint8_t lastAgc = 0;

	// Expander variables
	constexpr uint8_t ButtonDebounceCount = 3;
	constexpr unsigned int ButtonBitnum = 0;
	constexpr unsigned int RedLedBitnum = 4;
	constexpr unsigned int GreenLedBitnum = 5;
	constexpr uint8_t TCA8408A_initialOutputs = (1u << RedLedBitnum) | (1u << GreenLedBitnum);
	constexpr uint8_t TCA8408A_inputs = (1u << ButtonBitnum);		// button pin is input, LED and unused pins are outputs

	TCA6408A *expander = nullptr;
	InputMonitor *buttonMonitor = nullptr;
	uint8_t buttonCount = 0;										// incremented (up to max. buttonDebounceCount) when button is read down, decremented when button is read up
	bool buttonPressed = false;
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
		expander = new TCA6408A(i2cDevice);
		if (!expander->Init(TCA8408A_inputs, TCA8408A_initialOutputs))
		{
			DeleteObject(expander);
		}
	}

	if (encoder != nullptr || expander != nullptr)
	{
		as5601Task = new Task<AS5601TaskStackWords>();
		as5601Task->Create(MfmTaskCode, "MFM", nullptr, TaskPriority::Mfm);
	}
}

// Append diagnostics to a reply
void MFMHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	if (EncoderPresent() && ExpanderPresent())
	{
		reply.lcatf("Integrated filament monitor found, agc %u, magnet %sdetected", lastAgc, (lastStatus & AS5601::StatusMD) ? "" : "not ");
		if (lastStatus & AS5601::StatusMH) { reply.cat(", magnet too strong"); }
		if (lastStatus & AS5601::StatusML) { reply.cat(", magnet too weak"); }
	}
	else if (!EncoderPresent() && !ExpanderPresent())
	{
		reply.lcat("Integrated filament monitor not present");
	}
	else
	{
		reply.lcatf("Error: integrated AS5601%s found but TCA6408A%s found", (EncoderPresent()) ? "" : " not", ExpanderPresent() ? "" : " not");
	}
}

// Return true if the encoder was found during initialisation and can be used
bool MFMHandler::EncoderPresent() noexcept
{
	return encoder != nullptr;
}

// Return true if the I/O expander was found during initialisation and can be used
bool MFMHandler::ExpanderPresent() noexcept
{
	return expander != nullptr;
}

// This is called when a filament monitor that wants to use the encoder is configured. Return true if successful.
bool MFMHandler::AttachEncoderVirtualInterrupt(StandardCallbackFunction callback, FilamentMonitor *fm) noexcept
{
	if (encoder == nullptr) { return false; }						// MFM not present
	if (filamentMonitorCallbackFn != nullptr) { return false; }		// MFM already in use by a filament monitor

	TaskCriticalSectionLocker lock;
	filamentMonitorCallbackFn = callback;
	filamentMonitorCallbackObject = fm;
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

// Fetch the previously stored reading
bool MFMHandler::GetEncoderReading(uint16_t& reading, uint8_t& agc, uint8_t& errorCode) noexcept
{
	if (readingAvailable)
	{
		reading = angleReading;
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

// Enable the button and store a pointer to the input monitor to call back, or disable the button if nullptr is passed
bool MFMHandler::EnableButton(InputMonitor *monitor) noexcept
{
	buttonMonitor = monitor;
	return buttonPressed;
}

// Turn the red LED on or off
void MFMHandler::SetRedLed(bool on) noexcept
{
	if (expander != nullptr) { expander->SetOutputBitState(RedLedBitnum, on); }
}

// Turn the green LED on or off
void MFMHandler::SetGreenLed(bool on) noexcept
{
	if (expander != nullptr) { expander->SetOutputBitState(GreenLedBitnum, on); }
}

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
			if (encoder->Read(lastAngle, lastStatus, lastAgc))
			{
				TaskCriticalSectionLocker lock;
				if (filamentMonitorCallbackFn != nullptr && !readingAvailable)
				{
					angleReading = lastAngle;
					readingAvailable = true;
					filamentMonitorCallbackFn(CallbackParameter(filamentMonitorCallbackObject));
				}
			}
		}

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
	}
}

#endif
