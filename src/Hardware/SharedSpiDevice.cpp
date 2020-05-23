/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Jul 2019
 *      Author: David
 */

#include "SharedSpiDevice.h"

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP

#include "IoPorts.h"
#include "DmacManager.h"
#include "Serial.h"
#include "peripheral_clk_config.h"

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

static void InitSpi()
{
	Serial::EnableSercomClock(SERCOM_SSPI_NUMBER);

	// Set the pin functions
	gpio_set_pin_function(SSPIMosiPin, SSPIMosiPinPeriphMode);
	gpio_set_pin_function(SSPISclkPin, SSPISclkPinPeriphMode);
	gpio_set_pin_function(SSPIMisoPin, SSPIMisoPinPeriphMode);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0);
	const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
	const uint32_t regCtrlC = 0;											// not 32-bit mode

	if (!hri_sercomusart_is_syncing(SERCOM_SSPI, SERCOM_USART_SYNCBUSY_SWRST))
	{
		const uint32_t mode = regCtrlA & SERCOM_USART_CTRLA_MODE_Msk;
		if (hri_sercomusart_get_CTRLA_reg(SERCOM_SSPI, SERCOM_USART_CTRLA_ENABLE))
		{
			hri_sercomusart_clear_CTRLA_ENABLE_bit(SERCOM_SSPI);
			hri_sercomusart_wait_for_sync(SERCOM_SSPI, SERCOM_USART_SYNCBUSY_ENABLE);
		}
		hri_sercomusart_write_CTRLA_reg(SERCOM_SSPI, SERCOM_USART_CTRLA_SWRST | mode);
	}
	hri_sercomusart_wait_for_sync(SERCOM_SSPI, SERCOM_USART_SYNCBUSY_SWRST);

	hri_sercomusart_write_CTRLA_reg(SERCOM_SSPI, regCtrlA);
	hri_sercomusart_write_CTRLB_reg(SERCOM_SSPI, regCtrlB);
	hri_sercomusart_write_CTRLC_reg(SERCOM_SSPI, regCtrlC);
	hri_sercomusart_write_BAUD_reg(SERCOM_SSPI, SERCOM_SPI_BAUD_BAUD(SystemPeripheralClock/(2 * DefaultSharedSpiClockFrequency) - 1));
	hri_sercomusart_write_DBGCTRL_reg(SERCOM_SSPI, SERCOM_I2CM_DBGCTRL_DBGSTOP);			// baud rate generator is stopped when CPU halted by debugger

#if 0	// if using DMA
	// Set up the DMA descriptors
	// We use separate write-back descriptors, so we only need to set this up once, but it must be in SRAM
	DmacSetBtctrl(SspiRxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiRxDmaChannel, &(SERCOM_SSPI->SPI.DATA.reg));
	DmacSetDestinationAddress(SspiRxDmaChannel, rcvData);
	DmacSetDataLength(SspiRxDmaChannel, ARRAY_SIZE(rcvData));

	DmacSetBtctrl(SspiTxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiTxDmaChannel, sendData);
	DmacSetDestinationAddress(SspiTxDmaChannel, &(SERCOM_SSPI->SPI.DATA.reg));
	DmacSetDataLength(SspiTxDmaChannel, ARRAY_SIZE(sendData));

	DmacSetInterruptCallbacks(SspiRxDmaChannel, RxDmaCompleteCallback, nullptr, 0U);
#endif

	SERCOM_SSPI->SPI.CTRLB.bit.RXEN = 1;
}

static inline void DisableSpi()
{
	SERCOM_SSPI->SPI.CTRLA.bit.ENABLE = 0;
	hri_sercomusart_wait_for_sync(SERCOM_SSPI, SERCOM_USART_CTRLA_ENABLE);
}

static inline void EnableSpi()
{
	SERCOM_SSPI->SPI.CTRLA.bit.ENABLE = 1;
	hri_sercomusart_wait_for_sync(SERCOM_SSPI, SERCOM_USART_CTRLA_ENABLE);
}

// Wait for transmitter ready returning true if timed out
static inline bool waitForTxReady()
{
	uint32_t timeout = SpiTimeout;
	while (!(SERCOM_SSPI->SPI.INTFLAG.bit.DRE))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

// Wait for transmitter empty returning true if timed out
static inline bool waitForTxEmpty()
{
	uint32_t timeout = SpiTimeout;
	while (!(SERCOM_SSPI->SPI.INTFLAG.bit.TXC))
	{
		if (!timeout--)
		{
			return true;
		}
	}
	return false;
}

// Wait for receive data available returning true if timed out
static inline bool waitForRxReady()
{
	uint32_t timeout = SpiTimeout;
	while (!(SERCOM_SSPI->SPI.INTFLAG.bit.RXC))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

// SharedSpiDevice class members
SharedSpiDevice::SharedSpiDevice(uint32_t clockFreq, SpiMode m, bool polarity)
	: clockFrequency(clockFreq), csPin(NoPin), mode(m), csActivePolarity(polarity)
{
}

void SharedSpiDevice::InitMaster()
{
	static bool commsInitDone = false;

	IoPort::SetPinMode(csPin, (csActivePolarity) ? OUTPUT_LOW : OUTPUT_HIGH);

	if (!commsInitDone)
	{
		InitSpi();
		commsInitDone = true;
	}
}

void SharedSpiDevice::Select() const
{
	// We have to disable SPI device in order to change the baud rate and mode
	DisableSpi();
	hri_sercomusart_write_BAUD_reg(SERCOM_SSPI, SERCOM_SPI_BAUD_BAUD(SystemPeripheralClock/(2 * clockFrequency) - 1));

	uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0) | SERCOM_SPI_CTRLA_ENABLE;
	if (((uint8_t)mode & 2) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPHA;
	}
	hri_sercomusart_write_CTRLA_reg(SERCOM_SSPI, regCtrlA);
	EnableSpi();

	IoPort::WriteDigital(csPin, csActivePolarity);
}

void SharedSpiDevice::Deselect() const
{
	IoPort::WriteDigital(csPin, !csActivePolarity);
	DisableSpi();
}

bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const
{
	for (uint32_t i = 0; i < len; ++i)
	{
		const uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
		if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
		{
			return false;
		}

		// Write to transmit register
		SERCOM_SSPI->SPI.DATA.reg = dOut;

		// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
		if (rx_data != nullptr)
		{
			// Wait for receive register
			if (waitForRxReady())
			{
				return false;
			}

			// Get data from receive register
			const uint8_t dIn = (uint8_t)SERCOM_SSPI->SPI.DATA.reg;
			*rx_data++ = dIn;
		}
	}

	// If we didn't wait to receive, then we need to wait for transmit to finish and clear the receive buffer
	if (rx_data == nullptr)
	{
		waitForTxEmpty();
		(void)SERCOM_SSPI->SPI.DATA.reg;
	}

	return true;	// success
}

#endif

// End
