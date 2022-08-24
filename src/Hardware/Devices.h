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

//TODO expose the USB interface as a serial device

#else

#include <AsyncSerial.h>

extern AsyncSerial uart0;

#endif

void DeviceInit() noexcept;

#endif /* SRC_HARDWARE_DEVICES_H_ */
