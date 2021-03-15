/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#include "SharedI2CMaster.h"

#if SUPPORT_I2C_SENSORS

#include "Serial.h"

#if SAME5x
# include <hri_sercom_e54.h>
#elif SAMC21
# include <hri_sercom_c21.h>
#endif

constexpr uint32_t DefaultSharedI2CClockFrequency = 100000;
constexpr uint32_t I2CTimeoutTicks = 10;

void SharedI2CMaster::ErrorCounts::Clear() noexcept
{
	naks = sendTimeouts = recvTimeouts = finishTimeouts = resets = 0;
}

SharedI2CMaster::SharedI2CMaster(uint8_t sercomNum) noexcept : hardware(Serial::Sercoms[sercomNum]), taskWaiting(nullptr)
{
	Serial::EnableSercomClock(sercomNum);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_I2CM_CTRLA_MODE(5) | SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(2) | SERCOM_I2CM_CTRLA_MEXTTOEN | SERCOM_I2CM_CTRLA_SEXTTOEN;
	const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
#if SAME5x
	const uint32_t regCtrlC = 0;											// not 32-bit mode
#endif

	if (!hri_sercomi2cm_is_syncing(hardware, SERCOM_I2CM_SYNCBUSY_SWRST))
	{
		const uint32_t mode = regCtrlA & SERCOM_I2CM_CTRLA_MODE_Msk;
		if (hri_sercomi2cm_get_CTRLA_reg(hardware, SERCOM_I2CM_CTRLA_ENABLE))
		{
			hri_sercomi2cm_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_ENABLE);
		}
		hri_sercomi2cm_write_CTRLA_reg(hardware, SERCOM_I2CM_CTRLA_SWRST | mode);
	}
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_SWRST);

	hri_sercomi2cm_write_CTRLA_reg(hardware, regCtrlA);
	hri_sercomi2cm_write_CTRLB_reg(hardware, regCtrlB);
#if SAME5x
	hri_sercomi2cm_write_CTRLC_reg(hardware, regCtrlC);
#endif
	hri_sercomi2cm_write_BAUD_reg(hardware, SERCOM_I2CM_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedI2CClockFrequency) - 1));
	hri_sercomi2cm_write_DBGCTRL_reg(hardware, SERCOM_I2CM_DBGCTRL_DBGSTOP);			// baud rate generator is stopped when CPU halted by debugger

#if 0	// if using DMA
	// Set up the DMA descriptors
	// We use separate write-back descriptors, so we only need to set this up once, but it must be in SRAM
	DmacSetBtctrl(I2CRxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(I2CRxDmaChannel, &(hardware->I2CM.DATA.reg));
	DmacSetDestinationAddress(I2CRxDmaChannel, rcvData);
	DmacSetDataLength(I2CRxDmaChannel, ARRAY_SIZE(rcvData));

	DmacSetBtctrl(I2CTxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(I2CTxDmaChannel, sendData);
	DmacSetDestinationAddress(I2CTxDmaChannel, &(hardware->I2CM.DATA.reg));
	DmacSetDataLength(I2CTxDmaChannel, ARRAY_SIZE(sendData));

	DmacSetInterruptCallbacks(I2CRxDmaChannel, RxDmaCompleteCallback, nullptr, 0U);
#endif

	const IRQn irqn = Serial::GetSercomIRQn(sercomNum);
	NVIC_SetPriority(irqn, NvicPriorityI2C);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
#if SAME5x
	NVIC_SetPriority(irqn + 1, NvicPriorityI2C);
	NVIC_EnableIRQ(irqn + 1);
	NVIC_SetPriority(irqn + 3, NvicPriorityI2C);
	NVIC_EnableIRQ(irqn + 3);
#endif

	mutex.Create("I2C");

	Enable();
}

void SharedI2CMaster::SetClockFrequency(uint32_t freq) const noexcept
{
	// We have to disable SPI device in order to change the baud rate and mode
	Disable();
	hri_sercomspi_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * freq) - 1));
	Enable();
}

void SharedI2CMaster::Enable() const noexcept
{
	hardware->I2CM.CTRLA.bit.ENABLE = 1;
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_CTRLA_ENABLE);
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSSTATE(0x01);
}

void SharedI2CMaster::Disable() const noexcept
{
	hardware->I2CM.CTRLA.bit.ENABLE = 0;
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_CTRLA_ENABLE);
}

bool SharedI2CMaster::WaitForStatus(uint8_t statusBits) noexcept
{
	uint8_t status = hardware->I2CM.INTFLAG.reg;
	if ((status & statusBits) == 0)
	{
		TaskBase::ClearNotifyCount();
		taskWaiting = TaskBase::GetCallerTaskHandle();
		hardware->I2CM.INTENSET.reg = statusBits;
		TaskBase::Take(I2CTimeoutTicks);
		taskWaiting = nullptr;
		status = hardware->I2CM.INTFLAG.reg;
		hardware->I2CM.INTENCLR.reg = 0xFF;
	}
	hardware->I2CM.INTFLAG.reg = status;
	return (status & statusBits) != 0;
}

void SharedI2CMaster::Interrupt() noexcept
{
	hardware->I2CM.INTENCLR.reg = 0xFF;
	if (taskWaiting != nullptr)
	{
		TaskBase::GiveFromISR(taskWaiting);
		taskWaiting = nullptr;
	}
}

// Write then read data
bool SharedI2CMaster::Transfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept
{
	// If an empty transfer, nothing to do
	if (numToRead + numToWrite == 0)
	{
		return true;
	}

	size_t bytesTransferred;
	for (unsigned int triesDone = 0; triesDone < 3; ++triesDone)
	{
		bytesTransferred = InternalTransfer(address, firstByte, buffer, numToWrite, numToRead);
		if (bytesTransferred == numToRead + numToWrite)
		{
			return true;
		}

		// Had an I2C error, so re-initialise
		Disable();
		Enable();
		++errorCounts.resets;
	}
	return false;
}

size_t SharedI2CMaster::InternalTransfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept
{
	// Send the address
	address <<= 1;			// SERCOM uses the bottom bit as the Read flag
	hardware->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR;
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST;					// clear all status bits
	if (numToWrite != 0 || address >= 0x80)
	{
		hardware->I2CM.ADDR.reg = (address >= 0x100) ? address | SERCOM_I2CM_ADDR_TENBITEN : address;
		if (!WaitForStatus(SERCOM_I2CM_INTFLAG_MB) || (hardware->I2CM.STATUS.reg & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST)))
		{
			return 0;
		}
	}

	size_t bytesSent = 0;
	if (numToWrite != 0)
	{
		hardware->I2CM.ADDR.reg = (address >= 0x100) ? address | SERCOM_I2CM_ADDR_TENBITEN : address;
		if (!WaitForStatus(SERCOM_I2CM_INTFLAG_MB) || (hardware->I2CM.STATUS.reg & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST)))
		{
			return 0;
		}

		while (bytesSent < numToWrite)
		{
			hardware->I2CM.DATA.reg = (bytesSent == 0) ? firstByte : *buffer++;
			if (!WaitForStatus(SERCOM_I2CM_INTFLAG_MB) || (hardware->I2CM.STATUS.reg & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST)))
			{
				return bytesSent;
			}
			++bytesSent;
		}

		if (numToRead == 0)
		{
			hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_CMD(0x03);				// send stop command
			return bytesSent;
		}
	}

	// There are bytes to read, and if there were any bytes to send then we have sent them all
	if (address >= 0x100)
	{
		hardware->I2CM.ADDR.reg = (address >> 8) | 0b1111001;
	}
	else if (numToWrite != 0)
	{
		hardware->I2CM.ADDR.reg = address | 0x0001;
	}

	size_t bytesReceived = 0;
	for (;;)
	{
		if (!WaitForStatus(SERCOM_I2CM_INTFLAG_SB) || (hardware->I2CM.STATUS.reg & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST)))
		{
			return bytesSent;
		}
		*buffer++ = hardware->I2CM.DATA.reg;
		++bytesReceived;
		if (bytesReceived == numToRead)
		{
			break;
		}
		hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_CMD(0x02);					// acknowledge and read another byte
	}

	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_CMD(0x03);						// acknowledge and stop
	return bytesSent + bytesReceived;
}

SharedI2CMaster::ErrorCounts SharedI2CMaster::GetErrorCounts(bool clear) noexcept
{
	const irqflags_t flags = cpu_irq_save();
	const ErrorCounts ret = errorCounts;
	if (clear)
	{
		errorCounts.Clear();
	}
	cpu_irq_restore(flags);
	return ret;
}

#endif

// End
