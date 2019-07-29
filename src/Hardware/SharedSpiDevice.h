/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SHAREDSPIDEVICE_H_

#include "RepRapFirmware.h"

#if SUPPORT_SPI_SENSORS

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

class SharedSpiDevice
{
public:
	SharedSpiDevice(uint32_t clockFreq, SpiMode m, bool polarity);

	void InitMaster();
	void SetupMaster() const;
	void Select() const;
	void Deselect() const;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const;
	void SetCsPin(Pin p) { csPin = p; }

private:
	uint32_t clockFrequency;
	Pin csPin;
	SpiMode mode;
	bool csActivePolarity;
};

#endif

#endif /* SRC_HARDWARE_SHAREDSPIDEVICE_H_ */
