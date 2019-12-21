/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hal_can_async.h>

struct can_async_descriptor CAN_0;
struct wdt_descriptor WDT_0;

void WDT_0_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
}

void WDT_0_init(void)
{
	WDT_0_CLOCK_init();
	wdt_init(&WDT_0, WDT);
}

void CAN_0_PORT_init(void)
{
	gpio_set_pin_function(PB13, PINMUX_PB13H_CAN1_RX);
	gpio_set_pin_function(PB12, PINMUX_PB12H_CAN1_TX);
}

/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	hri_mclk_set_AHBMASK_CAN1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN1_GCLK_ID, CONF_GCLK_CAN1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN1);
	CAN_0_PORT_init();
}


void system_init(void)
{
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
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Function init_mcu sets up DPLL0 to generate the main clock from XOSC0. If we don't disable DPLL0 here then that fails in a release build.
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }

	// Also disable DPLL1 because we are going to program it to generate the 48MHz CAN clock
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	init_mcu();
	SystemCoreClock = 120000000;			// GCLK0
	SystemPeripheralClock = 60000000;		// GCLK1

	// We initialise CAN later

//	WDT_0_init();
}
