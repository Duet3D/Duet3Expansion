/*
 * AccelerometerHandler.h
 *
 *  Created on: 16 Mar 2021
 *      Author: David
 */

#ifndef SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_
#define SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_

#include <RepRapFirmware.h>
#include <GCodes/GCodeResult.h>

#if SUPPORT_LIS3DH

class CanMessageAccelerometerSettings;

// Interface to the accelerometer. These functions are called by the main task.
namespace AccelerometerHandler
{
	void Init() noexcept;
	bool Present() noexcept;
	GCodeResult ProcessCanCommand(const CanMessageAccelerometerSettings& msg, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
};

#endif

#endif /* SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_ */
