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
	bool GetEncoderReading(uint16_t& reading, uint8_t& agc, uint8_t& errorCode) noexcept;
	uint16_t GetLastAngle() noexcept;

#if SUPPORT_TCA6408A
	// Expander functions
	bool ExpanderPresent() noexcept;
	void SetLedRed() noexcept;
	void SetLedGreen() noexcept;
	void SetLedBoth() noexcept;
	void SetLedOff() noexcept;
	bool EnableButton(InputMonitor *monitor) noexcept;
#endif
}

#endif

#endif /* SRC_COMMANDPROCESSING_MFMHANDLER_H_ */
