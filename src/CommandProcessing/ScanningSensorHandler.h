/*
 * ScanningSensorHandler.h
 *
 *  Created on: 16 Jun 2023
 *      Author: David
 */

#ifndef SRC_COMMANDPROCESSING_SCANNINGSENSORHANDLER_H_
#define SRC_COMMANDPROCESSING_SCANNINGSENSORHANDLER_H_

#include <RepRapFirmware.h>

#if SUPPORT_LDC1612

#include <AnalogIn.h>		// for AnalogInCallbackFunction

namespace ScanningSensorHandler
{
	void Init() noexcept;
	bool IsPresent() noexcept;
	uint32_t GetReading() noexcept;
	GCodeResult SetOrCalibrateCurrent(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept;
	bool SetCallback(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept;
	float GetFrequency() noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_COMMANDPROCESSING_SCANNINGSENSORHANDLER_H_ */
