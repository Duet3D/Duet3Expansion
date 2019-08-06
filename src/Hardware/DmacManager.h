/*
 * Dmac.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_DMACMANAGER_H_
#define SRC_HARDWARE_DMACMANAGER_H_

#include "RepRapFirmware.h"

enum class DmaTrigSource : uint8_t
{
	disable = 0,
	rtc,
	dsu_dcc0,
	dsu_dcc1,

	sercom0_rx,
	sercom0_tx,
	sercom1_rx,
	sercom1_tx,
	sercom2_rx,
	sercom2_tx,
	sercom3_rx,
	sercom3_tx,
	sercom4_rx,
	sercom4_tx,
	sercom5_rx,
	sercom5_tx,
	sercom6_rx,
	sercom6_tx,
	sercom7_rx,
	sercom7_tx,

	can0_debug,
	can1_debug,

	tcc0_ovf = 0x16,
	tcc0_mc = 0x17,
	tcc1_ovf = 0x1D,
	tcc1_mc = 0x1E,
	tcc2_ovf = 0x22,
	tcc2_mc = 0x23,
	tcc3_ovf = 0x26,
	tcc3_mc = 0x27,
	tcc4_ovf = 0x29,
	tcc4_mc = 0x2A,

	tc0_ovf = 0x2C,
	tc0_mc = 0x2D,
	tc1_ovf = 0x2F,
	tc1_mc = 0x30,
	tc2_ovf = 0x32,
	tc2_mc = 0x33,
	tc3_ovf = 0x35,
	tc3_mc = 0x36,
	tc4_ovf = 0x38,
	tc4_mc = 0x39,
	tc5_ovf = 0x3B,
	tc5_mc = 0x3C,
	tc6_ovf = 0x3E,
	tc6_mc = 0x3F,
	tc7_ovf = 0x40,
	tc7_mc = 0x41,

	adc0_resrdy = 0x44,
	adc0_seq,
	adc1_resrdy,
	adc1_seq,

	dac_empty = 0x48,
	dac_resrdy = 0x4A,

	i2s_rx = 0x4C,
	i2s_tx = 0x4E,

	pcc = 0x50,

	aes_wr = 0x51,
	aes_rd,

	qspi_rx = 0x53,
	qspi_tx
};

void DmacInit();
void DmacSetDestinationAddress(const uint8_t channel, volatile void *const dst);
void DmacSetSourceAddress(const uint8_t channel, const volatile void *const src);
void DmacSetDataLength(const uint8_t channel, const uint32_t amount);
void DmacEnableChannel(const uint8_t channel, const bool software_trigger = false);
void DmacSetBtctrl(const uint8_t channel, const uint16_t val);
void DmacSetInterruptCallbacks(const uint8_t channel, StandardCallbackFunction tfrEndedFn, StandardCallbackFunction errorFn, CallbackParameter param);
void DmacEnableCompletedInterrupt(const uint8_t channel);
void DmacEnableErrorInterrupt(const uint8_t channel);
void DmacDisableCompletedInterrupt(const uint8_t channel);
void DmacDisableErrorInterrupt(const uint8_t channel);

#endif /* SRC_HARDWARE_DMACMANAGER_H_ */
