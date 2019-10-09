/*
 * FansManager.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_FANSMANAGER_H_
#define SRC_FANS_FANSMANAGER_H_

#include "RepRapFirmware.h"
#include "Fan.h"
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

class CanMessageGeneric;
class CanMessageFanParameters;
class CanMessageSetFanSpeed;
class CanMessageFanRpms;

namespace FansManager
{
	void Init();
	bool CheckFans();
	GCodeResult ConfigureFanPort(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult ConfigureFan(const CanMessageFanParameters& gb, const StringRef& reply);
	GCodeResult SetFanSpeed(const CanMessageSetFanSpeed& msg, const StringRef& reply);
	unsigned int PopulateFanRpmsReport(CanMessageFanRpms& msg);
#if 0
	void SetFanValue(uint32_t fanNum, float speed);
#endif
};

#endif /* SRC_FANS_FANSMANAGER_H_ */
