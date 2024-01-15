/*
 * MFMhandler.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "MFMhandler.h"

#if SUPPORT_AS5601

#include <Hardware/AS5601.h>

namespace MFMHandler
{
	AS5601 *encoder = nullptr;
}

void MFMHandler::Init(SharedI2CMaster& i2cDevice) noexcept
{
	if (encoder == nullptr)
	{
		encoder = new AS5601(i2cDevice);
		encoder->Init();
	}
}

void MFMHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Integrated magnetic filament monitor%s found", (Present()) ? "" : " not");
}

bool MFMHandler::Present() noexcept
{
	return encoder != nullptr && encoder->Present();
}

#endif
