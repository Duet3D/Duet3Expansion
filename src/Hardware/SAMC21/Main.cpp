/*
 * Main.cpp
 *
 *  Created on: 27 Jul 2020
 *      Author: David
 */

#include "RepRapFirmware.h"

#if SAMC21

#include <hpl_div.h>

static void InitClocks();

extern "C" void AppInit() noexcept
{
	// Disable CAN interrupts, because older bootloaders don't.
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	CAN0->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN0->ILE.reg = 0;
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN1->ILE.reg = 0;

	// If we have entered via the bootloader then we have already configured the clocks.
	// We could just leave them alone, but only if we definitely want to use the same clock configuration.
	// So instead we reset the clock configuration.
	// First reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.

#if 1
	// 2020-06-03: on the SammyC21 board the software reset of GCLK never completed, so reset it manually
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE;				// make sure OSC48M is enabled, clear the on-demand bit
	while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_OSC48MRDY) == 0) { }	// wait for it to become ready

	GCLK->GENCTRL[0].reg = 0x00000106;									// this is the reset default

	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE | OSCCTRL_OSC48MCTRL_ONDEMAND;		// back to reset default
#else
	// The following code works on Duet3D boards, but it hangs on the SammyC21
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }
#endif

	// Function init_mcu sets up DPLL0 to generate the main clock from XOSC0. If we don't disable DPLL0 here then that fails in a release build.
	OSCCTRL->DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	// Now it's safe to configure the clocks
	InitClocks();

	// All done
	SystemCoreClock = SystemCoreClockFreq;
}

static void InitClocks()
{
	hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

	// 32kHz oscillators
	const uint16_t calib = hri_osc32kctrl_read_OSCULP32K_CALIB_bf(OSC32KCTRL);
	hri_osc32kctrl_write_OSCULP32K_reg(OSC32KCTRL, OSC32KCTRL_OSCULP32K_CALIB(calib));
	hri_osc32kctrl_write_RTCCTRL_reg(OSC32KCTRL, OSC32KCTRL_RTCCTRL_RTCSEL(OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val));

	// Crystal oscillator
	hri_oscctrl_write_XOSCCTRL_reg(OSCCTRL,
	    	  OSCCTRL_XOSCCTRL_STARTUP(0)
			| (0 << OSCCTRL_XOSCCTRL_AMPGC_Pos)
	        | OSCCTRL_XOSCCTRL_GAIN(3)
			| (1 << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
	        | (0 << OSCCTRL_XOSCCTRL_SWBEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_CFDEN_Pos)
	        | (1 << OSCCTRL_XOSCCTRL_XTALEN_Pos)
			| (1 << OSCCTRL_XOSCCTRL_ENABLE_Pos));

	hri_oscctrl_write_EVCTRL_reg(OSCCTRL, (0 << OSCCTRL_EVCTRL_CFDEO_Pos));

	while (!hri_oscctrl_get_STATUS_XOSCRDY_bit(OSCCTRL)) { }
	hri_oscctrl_set_XOSCCTRL_AMPGC_bit(OSCCTRL);

	// DPLL
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, OSCCTRL_DPLLRATIO_LDRFRAC(0) | OSCCTRL_DPLLRATIO_LDR(23));
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL,
	    	  OSCCTRL_DPLLCTRLB_DIV(2)
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
	        | OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK(1)
	        | (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| (0 << OSCCTRL_DPLLCTRLB_LPEN_Pos)
	        | OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLPRESC_reg(OSCCTRL, OSCCTRL_DPLLPRESC_PRESC(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));
	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL))) { }

	// MCLK
	hri_mclk_write_CPUDIV_reg(MCLK, MCLK_CPUDIV_CPUDIV(1));

	// GCLK 0
	hri_gclk_write_GENCTRL_reg(GCLK, 0,
		GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 7);

	// GCLK 2
	hri_gclk_write_GENCTRL_reg(GCLK, 2,
		GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| (1 << GCLK_GENCTRL_GENEN_Pos) | 6);

	// Initialise divide and square root accelerator
	_div_init();
}

// Define replacement standard library functions
#include <syscalls.h>

#endif

// End
