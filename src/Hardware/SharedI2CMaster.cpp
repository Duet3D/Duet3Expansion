/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#include "SharedI2CMaster.h"

#if SUPPORT_I2C_SENSORS && !RP2040

#include "Serial.h"

#if SAME5x
# include <hri_sercom_e54.h>
#elif SAMC21
# include <hri_sercom_c21.h>
#endif

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;

SharedI2CMaster::SharedI2CMaster(uint8_t sercomNum) noexcept
	: hardware(Serial::Sercoms[sercomNum]), taskWaiting(nullptr), busErrors(0), naks(0), otherErrors(0), state(I2cState::idle)
{
	Serial::EnableSercomClock(sercomNum);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_I2CM_CTRLA_MODE(5) | SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(2) | SERCOM_I2CM_CTRLA_MEXTTOEN | SERCOM_I2CM_CTRLA_SEXTTOEN;

	if (!hardware->I2CM.SYNCBUSY.bit.SWRST)
	{
		const uint32_t mode = regCtrlA & SERCOM_I2CM_CTRLA_MODE_Msk;
		if (hardware->I2CM.CTRLA.bit.ENABLE)
		{
			hri_sercomi2cm_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_ENABLE);
		}
		hri_sercomi2cm_write_CTRLA_reg(hardware, SERCOM_I2CM_CTRLA_SWRST | mode);
	}
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_SWRST);

	hri_sercomi2cm_write_CTRLA_reg(hardware, regCtrlA);
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
#if SAME5x
	hardware->I2CM.CTRLC.reg = 0;												// 8-bit mode
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
	while (hardware->I2CM.SYNCBUSY.bit.ENABLE) { }
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSSTATE(0x01);
	while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
}

void SharedI2CMaster::Disable() const noexcept
{
	hardware->I2CM.CTRLA.bit.ENABLE = 0;
	while (hardware->I2CM.SYNCBUSY.bit.ENABLE) { }
}

// Write then read data
bool SharedI2CMaster::Transfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept
{
	// If an empty transfer, nothing to do
	if (numToRead + numToWrite == 0)
	{
		return true;
	}

	for (unsigned int triesDone = 0; triesDone < 3; ++triesDone)
	{
		if (InternalTransfer(address, firstByte, buffer, numToWrite, numToRead))
		{
			return true;
		}

		// Had an I2C error, so re-initialise
		Disable();
		Enable();
	}
	return false;
}

void SharedI2CMaster::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("I2C bus errors %u, naks %u, other errors %u", busErrors, naks, otherErrors);
	busErrors = naks = otherErrors = 0;
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept
{
	currentAddress = address << 1;											// SERCOM uses the bottom bit as the Read flag
	firstByteToWrite = firstByte;
	transferBuffer = buffer;
	numLeftToRead = numToRead;
	numLeftToWrite = numToWrite;
	hardware->I2CM.INTFLAG.reg = 0xFF;										// clear all flag bits
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST;		// clear all status bits
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;						// make sure the ACKACT bit is clear

	TaskBase::ClearCurrentTaskNotifyCount();
	taskWaiting = TaskBase::GetCallerTaskHandle();

	// Send the address
	if (numToWrite != 0)
	{
		state = I2cState::sendingAddressForWrite;
		hardware->I2CM.ADDR.reg = (currentAddress >= 0x100) ? currentAddress | SERCOM_I2CM_ADDR_TENBITEN : currentAddress;
		while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
		hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
	}
	else if (currentAddress >= 0x100)
	{
		state = I2cState::sendingTenBitAddressForRead;
		hardware->I2CM.ADDR.reg = currentAddress | SERCOM_I2CM_ADDR_TENBITEN;
		while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
		hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
	}
	else
	{
		state = I2cState::sendingAddressForRead;
		hardware->I2CM.ADDR.reg = currentAddress | 0x0001;
		while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
		hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
	}

	TaskBase::Take(I2CTimeoutTicks);
	if (state == I2cState::idle)
	{
		return true;
	}
	state = I2cState::idle;
	return false;
}

void SharedI2CMaster::ProtocolError() noexcept
{
	hardware->I2CM.INTFLAG.reg = 0xFF;
	const uint16_t status = hardware->I2CM.STATUS.reg;
	hardware->I2CM.STATUS.reg = status;
	if (status & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST))
	{
		++busErrors;
	}
	else if (status & SERCOM_I2CM_STATUS_RXNACK)
	{
		++naks;
	}
	else
	{
		++otherErrors;
	}
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_CMD(0x03);			// send stop command, get off bus
	state = I2cState::protocolError;
	TaskBase::GiveFromISR(taskWaiting);
	taskWaiting = nullptr;
}

void SharedI2CMaster::Interrupt() noexcept
{
	const uint8_t flags = hardware->I2CM.INTFLAG.reg & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_ERROR);
	hardware->I2CM.INTENCLR.reg = 0xFF;
	switch (state)
	{
	default:			// should not occur
		break;

	case I2cState::sendingAddressForWrite:
		if (flags == SERCOM_I2CM_INTFLAG_MB)
		{
			// Address sent successfully and we have at least one byte to write
			hardware->I2CM.DATA.reg = firstByteToWrite;
			--numLeftToWrite;
			while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
			hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
			state = I2cState::writing;
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::writing:
		if (flags == SERCOM_I2CM_INTFLAG_MB)
		{
			if (numLeftToWrite != 0)
			{
				hardware->I2CM.DATA.reg = *transferBuffer++;
				--numLeftToWrite;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
			}
			else if (numLeftToRead == 0)
			{
				hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_CMD(0x03);			// send stop command
				state = I2cState::idle;
				TaskBase::GiveFromISR(taskWaiting);
				taskWaiting = nullptr;
			}
			else if (currentAddress >= 0x100)
			{
				hardware->I2CM.ADDR.reg = (currentAddress >> 8) | 0b1111001;
				state = I2cState::sendingAddressForRead;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
			}
			else
			{
				hardware->I2CM.ADDR.reg = currentAddress | 0x0001;
				state = I2cState::sendingAddressForRead;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
			}
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::sendingTenBitAddressForRead:
		if (flags == SERCOM_I2CM_INTFLAG_MB)
		{
			hardware->I2CM.ADDR.reg = (currentAddress >> 8) | 0b1111001;
			state = I2cState::sendingAddressForRead;
			while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
			hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::sendingAddressForRead:
		state = I2cState::reading;
		// no break
	case I2cState::reading:
		if (flags == SERCOM_I2CM_INTFLAG_SB)
		{
			--numLeftToRead;
			if (numLeftToRead == 0)
			{
				// App note says we need to NAK the last byte and send the stop command before we read the data
				hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(0x03);	// NAK and stop
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				*transferBuffer++ = hardware->I2CM.DATA.reg;
				state = I2cState::idle;
				TaskBase::GiveFromISR(taskWaiting);
				taskWaiting = nullptr;
			}
			else
			{
				*transferBuffer++ = hardware->I2CM.DATA.reg;			// read the data and acknowledge it because we have set SMEN in CTRLB
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
			}
		}
		else
		{
			ProtocolError();
		}
		break;
	}
}

#endif

// End
