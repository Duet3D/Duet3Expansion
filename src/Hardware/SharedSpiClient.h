/*
 * SharedSpiClient.h
 *
 *  Created on: 4 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDSPICLIENT_H_
#define SRC_HARDWARE_SHAREDSPICLIENT_H_

#include <RepRapFirmware.h>

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)

#include "SharedSpiDevice.h"

class SharedSpiClient
{
public:
	SharedSpiClient(SharedSpiDevice& dev, uint32_t clockFreq, SpiMode m, bool polarity);

	void InitMaster();
	bool Select(uint32_t timeout) const;												// get SPI ownership and select the device, return true if successful
	void Deselect() const;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const;
	void SetCsPin(Pin p) { csPin = p; }

private:
	SharedSpiDevice& device;
	uint32_t clockFrequency;
	Pin csPin;
	SpiMode mode;
	bool csActivePolarity;
};

#endif

#endif /* SRC_HARDWARE_SHAREDSPICLIENT_H_ */
