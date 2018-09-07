/*
 * Dmac.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include <utils.h>
#include <hri_dmac_e51.h>
#include <hpl_dmac_config.h>
#include <HAL/DmacManager.h>
#include <RTOSIface/RTOSIface.h>

// Descriptors for all used DMAC channels
COMPILER_ALIGNED(16)
static DmacDescriptor descriptor_section[NumDmaChannelsUsed];

// Write back descriptors for all used DMAC channels
static DmacDescriptor write_back_section[NumDmaChannelsUsed];

// Array containing callbacks for DMAC channels. We assume the same callback parameter for both types of callback.
static StandardCallbackFunction transferCompleteCallbacks[NumDmaChannelsUsed];
static StandardCallbackFunction errorCallbacks[NumDmaChannelsUsed];
static CallbackParameter callbackParams[NumDmaChannelsUsed];

// Initialize the DMA controller
void DmacInit()
{
	hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
	hri_dmac_clear_CRCCTRL_reg(DMAC, DMAC_CRCCTRL_CRCSRC_Msk);
	hri_dmac_set_CTRL_SWRST_bit(DMAC);
	while (hri_dmac_get_CTRL_SWRST_bit(DMAC)) { }

	hri_dmac_write_CTRL_reg(DMAC,
	                        (CONF_DMAC_LVLEN0 << DMAC_CTRL_LVLEN0_Pos) | (CONF_DMAC_LVLEN1 << DMAC_CTRL_LVLEN1_Pos)
	                            | (CONF_DMAC_LVLEN2 << DMAC_CTRL_LVLEN2_Pos)
	                            | (CONF_DMAC_LVLEN3 << DMAC_CTRL_LVLEN3_Pos));
	hri_dmac_write_DBGCTRL_DBGRUN_bit(DMAC, CONF_DMAC_DBGRUN);

	hri_dmac_write_PRICTRL0_reg(
	    DMAC,
	    DMAC_PRICTRL0_LVLPRI0(CONF_DMAC_LVLPRI0) | DMAC_PRICTRL0_LVLPRI1(CONF_DMAC_LVLPRI1)
	        | DMAC_PRICTRL0_LVLPRI2(CONF_DMAC_LVLPRI2) | DMAC_PRICTRL0_LVLPRI3(CONF_DMAC_LVLPRI3)
	        | (CONF_DMAC_RRLVLEN0 << DMAC_PRICTRL0_RRLVLEN0_Pos) | (CONF_DMAC_RRLVLEN1 << DMAC_PRICTRL0_RRLVLEN1_Pos)
	        | (CONF_DMAC_RRLVLEN2 << DMAC_PRICTRL0_RRLVLEN2_Pos) | (CONF_DMAC_RRLVLEN3 << DMAC_PRICTRL0_RRLVLEN3_Pos));
	hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)descriptor_section);
	hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)write_back_section);

	for (unsigned int i = 0; i < 5; i++)
	{
		NVIC_DisableIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_ClearPendingIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_EnableIRQ((IRQn)(DMAC_0_IRQn + i));
	}

	hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

void DmacSetBtctrl(const uint8_t channel, const uint16_t val)
{
	hri_dmacdescriptor_write_BTCTRL_reg(&descriptor_section[channel], val);
}

void DmacSetDestinationAddress(const uint8_t channel, const void *const dst)
{
	hri_dmacdescriptor_write_DSTADDR_reg(&descriptor_section[channel], (uint32_t)dst);
}

void DmacSetSourceAddress(const uint8_t channel, const void *const src)
{
	hri_dmacdescriptor_write_SRCADDR_reg(&descriptor_section[channel], (uint32_t)src);
}

void DmacSetDataLength(const uint8_t channel, const uint32_t amount)
{
	const uint8_t beat_size = hri_dmacdescriptor_read_BTCTRL_BEATSIZE_bf(&descriptor_section[channel]);

	const uint32_t dstAddress = hri_dmacdescriptor_read_DSTADDR_reg(&descriptor_section[channel]);
	if (hri_dmacdescriptor_get_BTCTRL_DSTINC_bit(&descriptor_section[channel]))
	{
		hri_dmacdescriptor_write_DSTADDR_reg(&descriptor_section[channel], dstAddress + amount * (1 << beat_size));
	}

	const uint32_t srcAddress = hri_dmacdescriptor_read_SRCADDR_reg(&descriptor_section[channel]);
	if (hri_dmacdescriptor_get_BTCTRL_SRCINC_bit(&descriptor_section[channel]))
	{
		hri_dmacdescriptor_write_SRCADDR_reg(&descriptor_section[channel], srcAddress + amount * (1 << beat_size));
	}

	hri_dmacdescriptor_write_BTCNT_reg(&descriptor_section[channel], amount);
}

void DmacEnableChannel(const uint8_t channel, const bool software_trigger)
{
	hri_dmacdescriptor_set_BTCTRL_VALID_bit(&descriptor_section[channel]);
	hri_dmac_set_CHCTRLA_ENABLE_bit(DMAC, channel);

	if (software_trigger)
	{
		hri_dmac_set_SWTRIGCTRL_reg(DMAC, 1 << channel);
	}
}

void DmacSetInterruptCallbacks(const uint8_t channel, StandardCallbackFunction tfrEndedFn, StandardCallbackFunction errorFn, CallbackParameter param)
{
	InterruptCriticalSectionLocker lock;
	errorCallbacks[channel] = errorFn;
	transferCompleteCallbacks[channel] = tfrEndedFn;
	callbackParams[channel] = param;
}

void DmacEnableCompletedInterrupt(const uint8_t channel)
{
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, channel);
}

void DmacEnableErrorInterrupt(const uint8_t channel)
{
	hri_dmac_set_CHINTEN_TERR_bit(DMAC, channel);
}

void DmacDisableCompletedInterrupt(const uint8_t channel)
{
	hri_dmac_clear_CHINTEN_TCMPL_bit(DMAC, channel);
}

void DmacDisableErrorInterrupt(const uint8_t channel)
{
	hri_dmac_clear_CHINTEN_TERR_bit(DMAC, channel);
}

// Internal DMAC interrupt handler
static inline void CommonDmacHandler(uint8_t channel)
{
	if (hri_dmac_get_CHINTFLAG_TERR_bit(DMAC, channel))
	{
		hri_dmac_clear_CHINTFLAG_TERR_bit(DMAC, channel);
		const StandardCallbackFunction fn = errorCallbacks[channel];
		if (fn != nullptr)
		{
			fn(callbackParams[channel]);
		}
	}
	else if (hri_dmac_get_CHINTFLAG_TCMPL_bit(DMAC, channel))
	{
		hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, channel);
		const StandardCallbackFunction fn = transferCompleteCallbacks[channel];
		if (fn != nullptr)
		{
			fn(callbackParams[channel]);
		}
	}
}

extern "C" void DMAC_0_Handler()
{
	CommonDmacHandler(0);
}

extern "C" void DMAC_1_Handler(void)
{
	CommonDmacHandler(1);
}

extern "C" void DMAC_2_Handler(void)
{
	CommonDmacHandler(2);
}

extern "C" void DMAC_3_Handler(void)
{
	CommonDmacHandler(3);
}

extern "C" void DMAC_4_Handler(void)
{
	CommonDmacHandler(hri_dmac_get_INTPEND_reg(DMAC, DMAC_INTPEND_ID_Msk));
}

// End
