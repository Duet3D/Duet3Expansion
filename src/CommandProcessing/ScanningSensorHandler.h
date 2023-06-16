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

namespace ScanningSensorHandler
{
	void Init() noexcept;
	bool IsPresent() noexcept;
	uint16_t GetReading() noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_COMMANDPROCESSING_SCANNINGSENSORHANDLER_H_ */
