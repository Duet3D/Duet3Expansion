/*
 * Devices.h
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_DEVICES_H_
#define SRC_HARDWARE_DEVICES_H_

#include <RepRapFirmware.h>
#include <Uart.h>

extern Uart uart0;

void DeviceInit() noexcept;

#endif /* SRC_HARDWARE_DEVICES_H_ */
