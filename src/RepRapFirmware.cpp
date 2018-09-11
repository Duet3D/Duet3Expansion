/*
 * RepRapFirmware.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Movement/Move.h"
#include "GCodes/GCodes.h"
#include "Platform.h"
#include "RTOSIface/RTOSIface.h"

Move *moveInstance;

namespace RepRap
{
	void Init()
	{
		Platform::Init();
		GCodes::Init();
		moveInstance = new Move();
		moveInstance->Init();
	}

	void Spin()
	{
		Platform::Spin();
		GCodes::Spin();
		moveInstance->Spin();
//		//RTOSIface::Yield();
//		delay(1);
	}
}

void debugPrintf(const char* fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	Platform::MessageF(DebugMessage, fmt, vargs);
	va_end(vargs);
}

// End
