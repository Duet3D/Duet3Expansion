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

SharedI2CMaster::SharedI2CMaster(uint8_t sercomNum) : hardware(Serial::Sercoms[sercomNum])
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

	mutex.Create("I2C");
}

void SharedI2CMaster::SetClockFrequency(uint32_t freq) const
{
	// We have to disable SPI device in order to change the baud rate and mode
	Disable();
	hri_sercomspi_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * freq) - 1));
	Enable();
}

void SharedI2CMaster::Disable() const
{
	hardware->I2CM.CTRLA.bit.ENABLE = 0;
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_CTRLA_ENABLE);
}

inline void SharedI2CMaster::Enable() const
{
	hardware->I2CM.CTRLA.bit.ENABLE = 1;
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_CTRLA_ENABLE);
}

#endif

// End
