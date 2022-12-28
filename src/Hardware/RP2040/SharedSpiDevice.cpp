/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Dec 2022
 *      Author: David
 */

#include <Hardware/SharedSpiDevice.h>

#if RP2040 && (SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO))

SharedSpiDevice::SharedSpiDevice(uint8_t spiInstanceNum) noexcept
	: hardware((spiInstanceNum == 0) ? spi0 : spi1)
{
	mutex.Create("SPI");
}

void SharedSpiDevice::Disable() const noexcept
{
	spi_deinit(hardware);
}

void SharedSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	spi_init(hardware, freq);
	spi_set_format(hardware, 8, ((uint8_t)mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
}

bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	const int bytesTransferred = (rx_data == nullptr) ? spi_write_blocking(hardware, tx_data, len)
								: (tx_data == nullptr) ? spi_read_blocking(hardware, 0xFF, rx_data, len)
									: spi_write_read_blocking(hardware, tx_data, rx_data, len);
	return bytesTransferred == (int)len;
}

#endif

// End
