/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Jul 2019
 *      Author: David
 */

#include <Hardware/SharedSpiDevice.h>

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)

#include <Hardware/IoPorts.h>
#include "DmacManager.h"
#include "Serial.h"

#if SAME5x
# include <hri_sercom_e54.h>
#elif SAMC21
# include <hri_sercom_c21.h>
#endif

constexpr uint32_t SpiTimeout = 10000;

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad) noexcept : hardware(Serial::Sercoms[sercomNum])
{
	Serial::EnableSercomClock(sercomNum);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(dataInPad) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0);
	const uint32_t regCtrlB = 0;												// 8 bits, slave select disabled, receiver disabled for now
#if SAME5x
	const uint32_t regCtrlC = 0;												// not 32-bit mode
#endif

	if (!hri_sercomspi_is_syncing(hardware, SERCOM_SPI_SYNCBUSY_SWRST))
	{
		const uint32_t mode = regCtrlA & SERCOM_SPI_CTRLA_MODE_Msk;
		if (hri_sercomspi_get_CTRLA_reg(hardware, SERCOM_SPI_CTRLA_ENABLE))
		{
			hri_sercomspi_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_SYNCBUSY_ENABLE);
		}
		hri_sercomspi_write_CTRLA_reg(hardware, SERCOM_SPI_CTRLA_SWRST | mode);
	}
	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_SYNCBUSY_SWRST);

	hri_sercomspi_write_CTRLA_reg(hardware, regCtrlA);
	hri_sercomspi_write_CTRLB_reg(hardware, regCtrlB);
#if SAME5x
	hri_sercomspi_write_CTRLC_reg(hardware, regCtrlC);
#endif
	hri_sercomspi_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedSpiClockFrequency) - 1));
	hri_sercomspi_write_DBGCTRL_reg(hardware, SERCOM_I2CM_DBGCTRL_DBGSTOP);		// baud rate generator is stopped when CPU halted by debugger

#if 0	// if using DMA
	// Set up the DMA descriptors
	// We use separate write-back descriptors, so we only need to set this up once, but it must be in SRAM
	DmacSetBtctrl(SspiRxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiRxDmaChannel, &(hardware->SPI.DATA.reg));
	DmacSetDestinationAddress(SspiRxDmaChannel, rcvData);
	DmacSetDataLength(SspiRxDmaChannel, ARRAY_SIZE(rcvData));

	DmacSetBtctrl(SspiTxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiTxDmaChannel, sendData);
	DmacSetDestinationAddress(SspiTxDmaChannel, &(hardware->SPI.DATA.reg));
	DmacSetDataLength(SspiTxDmaChannel, ARRAY_SIZE(sendData));

	DmacSetInterruptCallbacks(SspiRxDmaChannel, RxDmaCompleteCallback, nullptr, 0U);
#endif

	hardware->SPI.CTRLB.bit.RXEN = 1;

	mutex.Create("SPI");
}

void SharedSpiDevice::Disable() const noexcept
{
	hardware->SPI.CTRLA.bit.ENABLE = 0;
	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_CTRLA_ENABLE);
}

// Wait for transmitter ready returning true if timed out
inline bool SharedSpiDevice::waitForTxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SPI.INTFLAG.bit.DRE))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

// Wait for transmitter empty returning true if timed out
inline bool SharedSpiDevice::waitForTxEmpty() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SPI.INTFLAG.bit.TXC))
	{
		if (!timeout--)
		{
			return true;
		}
	}
	return false;
}

// Wait for receive data available returning true if timed out
inline bool SharedSpiDevice::waitForRxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SPI.INTFLAG.bit.RXC))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

void SharedSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	// We have to disable SPI device in order to change the baud rate and mode
	Disable();
	hri_sercomspi_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * freq) - 1));

	uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0) | SERCOM_SPI_CTRLA_ENABLE;
	if (((uint8_t)mode & 2) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPHA;
	}
	hri_sercomspi_write_CTRLA_reg(hardware, regCtrlA);

	hardware->SPI.CTRLA.bit.ENABLE = 1;
	hri_sercomspi_wait_for_sync(hardware, SERCOM_SPI_CTRLA_ENABLE);
}

bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	for (uint32_t i = 0; i < len; ++i)
	{
		const uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
		if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
		{
			return false;
		}

		// Write to transmit register
		hardware->SPI.DATA.reg = dOut;

		// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
		if (rx_data != nullptr)
		{
			// Wait for receive register
			if (waitForRxReady())
			{
				return false;
			}

			// Get data from receive register
			const uint8_t dIn = (uint8_t)hardware->SPI.DATA.reg;
			*rx_data++ = dIn;
		}
	}

	// If we didn't wait to receive, then we need to wait for transmit to finish and clear the receive buffer
	if (rx_data == nullptr)
	{
		waitForTxEmpty();				// wait for the last character to be transmitted

		// The SAME5x seems to buffer more than one received character
		while (hardware->SPI.INTFLAG.bit.RXC)
		{
			(void)hardware->SPI.DATA.reg;
		}
	}

	return true;	// success
}

#endif

// End
