/*
 * Endstop.h
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOPMANAGER_H_
#define SRC_ENDSTOPS_ENDSTOPMANAGER_H_

#include "RepRapFirmware.h"
#include "EndstopDefs.h"
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

#include "CanId.h"

class CanMessageBuffer;
struct CanMessageCreateZProbe;
struct CanMessageConfigureZProbe;
struct CanMessageGetZProbePinNames;
struct CanMessageDestroyZProbe;
struct CanMessageSetProbing;

// Endstop manager class
namespace EndstopsManager
{
	void Init();

	GCodeResult CreateZProbe(const CanMessageCreateZProbe& msg, size_t dataLength, const StringRef& reply);
	GCodeResult ConfigureZProbe(const CanMessageConfigureZProbe& msg, const StringRef& reply, uint8_t& extra);
	GCodeResult GetZProbePinNames(const CanMessageGetZProbePinNames& msg, const StringRef& reply);
	GCodeResult DestroyZProbe(const CanMessageDestroyZProbe& msg, const StringRef& reply);
	GCodeResult SetProbing(const CanMessageSetProbing& msg, const StringRef& reply);

	// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command
//	void EnableAxisEndstops(AxesBitmap axes, bool forHoming);

	// Set up the active endstops for Z probing
	void EnableZProbe(size_t probeNumber);

	// Enable extruder endstops
//	void EnableExtruderEndstop(size_t extruder);

	// Get the first endstop that has triggered and remove it from the active list if appropriate
	EndstopHitDetails CheckEndstops(bool goingSlow);

//	EndStopHit Stopped(size_t axis) const;

//	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply);
};

#endif /* SRC_ENDSTOPS_ENDSTOPMANAGER_H_ */
