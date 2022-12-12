/*
 * Devices.h
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_DEVICES_H_
#define SRC_HARDWARE_DEVICES_H_

#include <RepRapFirmware.h>

#if RP2040

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC_tusb.h>

extern SerialCDC serialUSB;

#else

#include <AsyncSerial.h>

extern AsyncSerial uart0;

#endif

void DeviceInit() noexcept;

#endif /* SRC_HARDWARE_DEVICES_H_ */
