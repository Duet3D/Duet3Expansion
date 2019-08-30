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

struct can_async_descriptor CAN_0;

struct flash_descriptor FLASH_0;

void FLASH_0_CLOCK_init(void)
{

	hri_mclk_set_AHBMASK_NVMCTRL_bit(MCLK);
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(PB11, PINMUX_PB11G_CAN1_RX);

	gpio_set_pin_function(PB10, PINMUX_PB10G_CAN1_TX);
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
	init_mcu();
}

void DeviceInit()
{
	CAN_0_init();
//	FLASH_0_init();
//	WDT_0_init();
}
