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

namespace MFMHandler
{
	// General functions
	void Init(SharedI2CMaster& i2cDevice) noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept;

	// AS5601 functions
	bool EncoderPresent() noexcept;
	void Start() noexcept;													// start taking regular readings
	void Stop() noexcept;													// stop taking regular readings

	// Expander functions
	bool ExpanderPresent() noexcept;
	void SetRedLed(bool on) noexcept;
	void SetGreenLed(bool on) noexcept;
	bool IsButtonPressed() noexcept;

}

#endif

#endif /* SRC_COMMANDPROCESSING_MFMHANDLER_H_ */
