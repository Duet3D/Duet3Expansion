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

#if RP2040
# include "hardware/spi.h"
#endif

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

class SharedSpiDevice
{
public:
#if SAME5x || SAMC21
	SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad) noexcept;
#elif RP2040
	SharedSpiDevice(uint8_t spiInstanceNum) noexcept;
#endif

	void Disable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() noexcept { mutex.Release(); }

	static constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;

private:

#if SAME5x || SAMC21
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

	Sercom * const hardware;
#elif RP2040
	spi_inst_t *hardware;
#endif

	Mutex mutex;
};

#endif

#endif /* SRC_HARDWARE_SHAREDSPIDEVICE_H_ */
