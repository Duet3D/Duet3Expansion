/*
 * GpioDevice.h - General purpose output device
 *
 *  Created on: 8 Jul 2019
 *      Author: David
 */

#ifndef SRC_GPIO_GPODEVICE_H_
#define SRC_GPIO_GPODEVICE_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"

class GpoDevice
{
public:
	GpoDevice(uint8_t instanceNum);

	uint8_t GetInstanceNumber() const { return instanceNumber; }

	bool ConfigurePort(const char* portName);
	void SetFrequency(float freq) { port.SetFrequency(freq); }

private:
	PwmPort port;
	uint8_t instanceNumber;
};

#endif /* SRC_GPIO_GPODEVICE_H_ */
