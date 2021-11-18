/*
 * GpioDevice.h - General purpose output device
 *
 *  Created on: 8 Jul 2019
 *      Author: David
 */

#ifndef SRC_GPIO_GPODEVICE_H_
#define SRC_GPIO_GPODEVICE_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <CanMessageFormats.h>

namespace GpioPorts
{
	GCodeResult HandleM950Gpio(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult HandleGpioWrite(const CanMessageWriteGpio& msg, const StringRef& reply);
}

#endif /* SRC_GPIO_GPODEVICE_H_ */
