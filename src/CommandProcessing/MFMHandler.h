/*
 * MFMhandler.h
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#ifndef SRC_COMMANDPROCESSING_MFMHANDLER_H_
#define SRC_COMMANDPROCESSING_MFMHANDLER_H_

#include <RepRapFirmware.h>

#if SUPPORT_AS5601

#include <Hardware/SharedI2CMaster.h>

class InputMonitor;
class FilamentMonitor;

namespace MFMHandler
{
	// General functions
	void Init(SharedI2CMaster& i2cDevice) noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept;

	// AS5601 functions
	bool EncoderPresent() noexcept;
	bool AttachEncoderVirtualInterrupt(StandardCallbackFunction callback, FilamentMonitor *fm) noexcept;
	void DetachEncoderVirtualInterrupt(FilamentMonitor *fm) noexcept;
	bool GetEncoderReading(uint16_t& reading) noexcept;

	// Expander functions
	bool ExpanderPresent() noexcept;
	void SetRedLed(bool on) noexcept;
	void SetGreenLed(bool on) noexcept;
	bool EnableButton(InputMonitor *monitor) noexcept;
}

#endif

#endif /* SRC_COMMANDPROCESSING_MFMHANDLER_H_ */
