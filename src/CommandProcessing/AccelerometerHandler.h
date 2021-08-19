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

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH

class CanMessageGeneric;
class CanMessageStartAccelerometer;

// Interface to the accelerometer. These functions are called by the main task.
namespace AccelerometerHandler
{
	void Init() noexcept;
	bool IsPresent() noexcept;
	GCodeResult ProcessConfigRequest(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessStartRequest(const CanMessageStartAccelerometer& msg, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
};

#endif

#endif /* SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_ */
