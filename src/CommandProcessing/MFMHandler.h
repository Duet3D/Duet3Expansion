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
	void Init(SharedI2CMaster& i2cDevice) noexcept;
	bool EncoderPresent() noexcept;
	bool ExpanderPresent() noexcept;
	void Start() noexcept;													// start taking regular readings
	void Stop() noexcept;													// stop taking regular readings
	void AppendDiagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_COMMANDPROCESSING_MFMHANDLER_H_ */
