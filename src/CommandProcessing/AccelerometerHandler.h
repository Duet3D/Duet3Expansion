/*
 * AccelerometerHandler.h
 *
 *  Created on: 16 Mar 2021
 *      Author: David
 */

#ifndef SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_
#define SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_

#include <RepRapFirmware.h>

#if SUPPORT_LIS3DH

#if ACCELEROMETER_USES_SPI
# include <Hardware/SharedSpiDevice.h>
#else
# include <Hardware/SharedI2CMaster.h>
#endif

class CanMessageGeneric;
class CanMessageStartAccelerometer;

// Interface to the accelerometer. These functions are called by the main task.
namespace AccelerometerHandler
{
#if ACCELEROMETER_USES_SPI
	void Init(SharedSpiDevice& dev) noexcept;
#else
	void Init(SharedI2CMaster& dev) noexcept;
#endif
	bool IsPresent() noexcept;
	GCodeResult ProcessConfigRequest(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessStartRequest(const CanMessageStartAccelerometer& msg, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
};

#endif

#endif /* SRC_COMMANDPROCESSING_ACCELEROMETERHANDLER_H_ */
