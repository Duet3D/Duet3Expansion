/*
 * Dmac.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include <utils.h>
#include "Peripherals.h"

#if defined(SAME51)
# include <hri_dmac_e51.h>
#elif defined(SAMC21)
# include <hri_dmac_c21.h>
#else
# error Unsupported processor
#endif

#include <hpl_dmac_config.h>
#include <Hardware/DmacManager.h>
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
void DmacManager::Init()
{
	hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
#ifdef SAMC21
	hri_dmac_clear_CTRL_CRCENABLE_bit(DMAC);
#else
	hri_dmac_clear_CRCCTRL_reg(DMAC, DMAC_CRCCTRL_CRCSRC_Msk);
#endif
	hri_dmac_set_CTRL_SWRST_bit(DMAC);
	while (hri_dmac_get_CTRL_SWRST_bit(DMAC)) { }

	hri_dmac_write_CTRL_reg(DMAC,
	                        (CONF_DMAC_LVLEN0 << DMAC_CTRL_LVLEN0_Pos) | (CONF_DMAC_LVLEN1 << DMAC_CTRL_LVLEN1_Pos)
	                            | (CONF_DMAC_LVLEN2 << DMAC_CTRL_LVLEN2_Pos)
	                            | (CONF_DMAC_LVLEN3 << DMAC_CTRL_LVLEN3_Pos));
	hri_dmac_write_DBGCTRL_DBGRUN_bit(DMAC, CONF_DMAC_DBGRUN);

#ifdef SAMC21
	hri_dmac_write_QOSCTRL_reg(DMAC,
	                           DMAC_QOSCTRL_WRBQOS(CONF_DMAC_WRBQOS) | DMAC_QOSCTRL_FQOS(CONF_DMAC_FQOS)
	                               | DMAC_QOSCTRL_DQOS(CONF_DMAC_DQOS));
#endif

	hri_dmac_write_PRICTRL0_reg(
	    DMAC,
	    DMAC_PRICTRL0_LVLPRI0(CONF_DMAC_LVLPRI0) | DMAC_PRICTRL0_LVLPRI1(CONF_DMAC_LVLPRI1)
	        | DMAC_PRICTRL0_LVLPRI2(CONF_DMAC_LVLPRI2) | DMAC_PRICTRL0_LVLPRI3(CONF_DMAC_LVLPRI3)
	        | (CONF_DMAC_RRLVLEN0 << DMAC_PRICTRL0_RRLVLEN0_Pos) | (CONF_DMAC_RRLVLEN1 << DMAC_PRICTRL0_RRLVLEN1_Pos)
	        | (CONF_DMAC_RRLVLEN2 << DMAC_PRICTRL0_RRLVLEN2_Pos) | (CONF_DMAC_RRLVLEN3 << DMAC_PRICTRL0_RRLVLEN3_Pos));
	hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)descriptor_section);
	hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)write_back_section);

#if defined(SAME51)
	for (unsigned int i = 0; i < 5; i++)
	{
		NVIC_DisableIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_ClearPendingIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_SetPriority((IRQn)(DMAC_0_IRQn + i), NvicPriorityDmac);
		NVIC_EnableIRQ((IRQn)(DMAC_0_IRQn + i));
	}
#elif defined(SAMC21)
	NVIC_DisableIRQ(DMAC_IRQn);
	NVIC_ClearPendingIRQ(DMAC_IRQn);
	NVIC_SetPriority(DMAC_IRQn, NvicPriorityDmac);
	NVIC_EnableIRQ(DMAC_IRQn);
#else
# error Unsupported processor
#endif

	hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

void DmacManager::SetBtctrl(const uint8_t channel, const uint16_t val)
{
	hri_dmacdescriptor_write_BTCTRL_reg(&descriptor_section[channel], val);
}

void DmacManager::SetDestinationAddress(const uint8_t channel, volatile void *const dst)
{
	hri_dmacdescriptor_write_DSTADDR_reg(&descriptor_section[channel], reinterpret_cast<uint32_t>(dst));
}

void DmacManager::SetSourceAddress(const uint8_t channel, const volatile void *const src)
{
	hri_dmacdescriptor_write_SRCADDR_reg(&descriptor_section[channel], reinterpret_cast<uint32_t>(src));
}

void DmacManager::SetDataLength(const uint8_t channel, const uint32_t amount)
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

void DmacManager::SetTriggerSource(uint8_t channel, DmaTrigSource source)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC((uint32_t)source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetTriggerSourceSercomRx(uint8_t channel, uint8_t sercomNumber)
{
	const uint32_t source = GetSercomRxTrigSource(sercomNumber);
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

// Transmit
void DmacManager::SetTriggerSourceSercomTx(uint8_t channel, uint8_t sercomNumber)
{
	const uint32_t source = GetSercomTxTrigSource(sercomNumber);
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetArbitrationLevel(uint8_t channel, uint8_t level)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHPRILVL.reg = level;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = (DMAC->CHCTRLB.reg & ~DMAC_CHCTRLB_LVL_Msk) | (level << DMAC_CHCTRLB_LVL_Pos);
#else
# error Unsupported processor
#endif
}

void DmacManager::EnableChannel(const uint8_t channel)
{
	hri_dmacdescriptor_set_BTCTRL_VALID_bit(&descriptor_section[channel]);
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 1;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.bit.ENABLE = 1;
#else
# error Unsupported processor
#endif
}

void DmacManager::DisableChannel(const uint8_t channel)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0;
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.bit.ENABLE = 0;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetInterruptCallbacks(const uint8_t channel, StandardCallbackFunction tfrEndedFn, StandardCallbackFunction errorFn, CallbackParameter param)
{
	InterruptCriticalSectionLocker lock;
	errorCallbacks[channel] = errorFn;
	transferCompleteCallbacks[channel] = tfrEndedFn;
	callbackParams[channel] = param;
}

void DmacManager::EnableCompletedInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, channel);
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC);
#else
# error Unsupported processor
#endif
}

void DmacManager::EnableErrorInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	hri_dmac_set_CHINTEN_TERR_bit(DMAC, channel);
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	hri_dmac_set_CHINTEN_TERR_bit(DMAC);
#else
# error Unsupported processor
#endif
}

void DmacManager::DisableCompletedInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	hri_dmac_clear_CHINTEN_TCMPL_bit(DMAC, channel);
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	hri_dmac_clear_CHINTEN_TCMPL_bit(DMAC);
#else
# error Unsupported processor
#endif
}

void DmacManager::DisableErrorInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	hri_dmac_clear_CHINTEN_TERR_bit(DMAC, channel);
#elif defined(SAMC21)
	InterruptCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	hri_dmac_clear_CHINTEN_TERR_bit(DMAC);
#else
# error Unsupported processor
#endif
}

#if defined(SAME51)

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
	if (hri_dmac_get_CHINTFLAG_TCMPL_bit(DMAC, channel))
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

extern "C" void DMAC_1_Handler()
{
	CommonDmacHandler(1);
}

extern "C" void DMAC_2_Handler()
{
	CommonDmacHandler(2);
}

extern "C" void DMAC_3_Handler()
{
	CommonDmacHandler(3);
}

extern "C" void DMAC_4_Handler()
{
	hri_dmac_intpend_reg_t intPend;
	while ((intPend = hri_dmac_get_INTPEND_reg(DMAC, DMAC_INTPEND_ID_Msk)) > 3)
	{
		CommonDmacHandler(intPend);
	}
}

#elif defined(SAMC21)

extern "C" void DMAC_Handler()
{
	hri_dmac_intpend_reg_t intPend;
	while (((intPend = DMAC->INTPEND.reg) & DMAC_INTPEND_PEND) != 0)
	{
		const size_t channel = intPend & DMAC_INTPEND_ID_Msk;
		DMAC->CHID.reg = channel;
		if (hri_dmac_get_CHINTFLAG_TERR_bit(DMAC))
		{
			hri_dmac_clear_CHINTFLAG_TERR_bit(DMAC);
			const StandardCallbackFunction fn = errorCallbacks[channel];
			if (fn != nullptr)
			{
				fn(callbackParams[channel]);
			}
		}
		if (hri_dmac_get_CHINTFLAG_TCMPL_bit(DMAC))
		{
			hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC);
			const StandardCallbackFunction fn = transferCompleteCallbacks[channel];
			if (fn != nullptr)
			{
				fn(callbackParams[channel]);
			}
		}
	}
}

#else
# error Unsupported processor
#endif

// End
