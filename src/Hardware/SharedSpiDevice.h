/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 *
 *  This currently supports only a single SPI channel. To support multiple SPI channels we would need to make the underlying SERCOM device
 *  configured in SPI mode a separate object, and have a pointer or reference to it in SharedSpiDevice.
 */

#ifndef SRC_HARDWARE_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SHAREDSPIDEVICE_H_

#include "RepRapFirmware.h"

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)

#include <RTOSIface/RTOSIface.h>

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

class SharedSpiDevice
{
public:
	SharedSpiDevice(uint8_t sercomNum);

	void Disable() const;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const;
	bool Take(uint32_t timeout) const { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() const { mutex.Release(); }

private:
	void Enable() const;
	bool waitForTxReady() const;
	bool waitForTxEmpty() const;
	bool waitForRxReady() const;

	Sercom * const hardware;
	Mutex mutex;
};

#endif

#endif /* SRC_HARDWARE_SHAREDSPIDEVICE_H_ */
