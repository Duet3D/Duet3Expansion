/**
 * \file
 *
 * \brief Instance description for LPMCU_MISC_REGS0
 *
 * Copyright (c) 2017 Atmel Corporation, a wholly owned subsidiary of Microchip Technology Inc.
 *
 * \license_start
 *
 * \page License
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \license_stop
 *
 */

/* file generated from device description version  */
#ifndef _SAMB11_LPMCU_MISC_REGS0_INSTANCE_H_
#define _SAMB11_LPMCU_MISC_REGS0_INSTANCE_H_

/* ========== Register definition for LPMCU_MISC_REGS0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_LPMCU_MISC_REGS0_LPMCU_CHIP_ID (0x4000B000) /**< (LPMCU_MISC_REGS0) The hard-coded ID for the chip. */
#define REG_LPMCU_MISC_REGS0_LPMCU_GLOBAL_RESET_0 (0x4000B004) /**< (LPMCU_MISC_REGS0) Active Low Global Reset */
#define REG_LPMCU_MISC_REGS0_LPMCU_GLOBAL_RESET_1 (0x4000B008) /**< (LPMCU_MISC_REGS0) Active Low Global Reset */
#define REG_LPMCU_MISC_REGS0_LPMCU_CLOCK_ENABLES_0 (0x4000B00C) /**< (LPMCU_MISC_REGS0) LPMCU Clock Enables */
#define REG_LPMCU_MISC_REGS0_LPMCU_CLOCK_ENABLES_1 (0x4000B010) /**< (LPMCU_MISC_REGS0) LPMCU Clock Enables */
#define REG_LPMCU_MISC_REGS0_BTMCU_CTRL (0x4000B014) /**< (LPMCU_MISC_REGS0) Miscellaneous BLE control */
#define REG_LPMCU_MISC_REGS0_LPMCU_CTRL (0x4000B018) /**< (LPMCU_MISC_REGS0) Miscellaneous LPMCU control */
#define REG_LPMCU_MISC_REGS0_LPMCU_MBIST (0x4000B01C) /**< (LPMCU_MISC_REGS0) MBIST designation */
#define REG_LPMCU_MISC_REGS0_BLE_CTRL (0x4000B020) /**< (LPMCU_MISC_REGS0) BLE Control */
#define REG_LPMCU_MISC_REGS0_MBIST_CTRL (0x4000B024) /**< (LPMCU_MISC_REGS0) MBIST Control Register */
#define REG_LPMCU_MISC_REGS0_MBIST_STATUS (0x4000B028) /**< (LPMCU_MISC_REGS0) MBIST Status Register */
#define REG_LPMCU_MISC_REGS0_MBIST_SRAM_ALG_SEL (0x4000B02C) /**< (LPMCU_MISC_REGS0) Enables MBIST algorithms */
#define REG_LPMCU_MISC_REGS0_MBIST_BG (0x4000B030) /**< (LPMCU_MISC_REGS0) MBIST Background for MARCH2 algorithm (bits 31:0) */
#define REG_LPMCU_MISC_REGS0_TEST_BUS_CTRL (0x4000B038) /**< (LPMCU_MISC_REGS0) Controls all of the test bus functionality */
#define REG_LPMCU_MISC_REGS0_WP3_MISR_DATAOUT (0x4000B03C) /**< (LPMCU_MISC_REGS0) ROM MBIST Signature */
#define REG_LPMCU_MISC_REGS0_INVERT_OUTPUT_CTRL (0x4000B040) /**< (LPMCU_MISC_REGS0) Invert Output Control */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_0 (0x4000B044) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_0, LP_GPIO_1, LP_GPIO_2, LP_GPIO_3, LP_GPIO_4, LP_GPIO_5, LP_GPIO_6, LP_GPIO_7 */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_1 (0x4000B048) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_8, LP_GPIO_9, LP_GPIO_10, LP_GPIO_11, LP_GPIO_12, LP_GPIO_13, LP_GPIO_14, LP_GPIO_15 */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_2 (0x4000B04C) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_16, LP_GPIO_17, LP_GPIO_18, LP_GPIO_19, LP_GPIO_20, LP_GPIO_21, LP_GPIO_22, LP_GPIO_23 */
#define REG_LPMCU_MISC_REGS0_PULL_ENABLE (0x4000B050) /**< (LPMCU_MISC_REGS0) Active Low Pull Enables for LPMCU Pads */
#define REG_LPMCU_MISC_REGS0_RTYPE_PAD_0 (0x4000B054) /**< (LPMCU_MISC_REGS0) Controls the RTYPE (Pull Level) pad value for LPMCU Pads (0 = Pull Up) */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_3 (0x4000B068) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_SIP_0, LP_SIP_1, LP_SIP_2, LP_SIP_3, LP_SIP_4 */
#define REG_LPMCU_MISC_REGS0_ISHAPE_PAD_3 (0x4000B06C) /**< (LPMCU_MISC_REGS0) Controls the ISHAPE pad value for LPMCU SIP Pads */
#define REG_LPMCU_MISC_REGS0_LPMCU_CTRL_2 (0x4000B090) /**< (LPMCU_MISC_REGS0) Misc control for the LPMCU */
#define REG_LPMCU_MISC_REGS0_SPIFLASH_VDDIO_CTRL (0x4000B0A0) /**< (LPMCU_MISC_REGS0) Control for VDDIO of SPI FLASH */
#define REG_LPMCU_MISC_REGS0_SPIFLASH_BYPASS (0x4000B0A4) /**< (LPMCU_MISC_REGS0) SPI FLASH Bypass */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_0 (0x4000B0C0) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_1 (0x4000B0C4) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_2 (0x4000B0C8) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_3 (0x4000B0CC) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_4 (0x4000B0D0) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_5 (0x4000B0D4) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_PWM0_CTRL (0x4000B160) /**< (LPMCU_MISC_REGS0) PWM0 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM1_CTRL (0x4000B164) /**< (LPMCU_MISC_REGS0) PWM1 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM2_CTRL (0x4000B168) /**< (LPMCU_MISC_REGS0) PWM2 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM3_CTRL (0x4000B16C) /**< (LPMCU_MISC_REGS0) PWM3 Control Register */
#define REG_LPMCU_MISC_REGS0_MBIST_DUAL_STATUS (0x4000B170) /**< (LPMCU_MISC_REGS0) Reflects the status of MBIST from lpmcu and btmcu */
#define REG_LPMCU_MISC_REGS0_DUALTIMER0_CTRL (0x4000B188) /**< (LPMCU_MISC_REGS0) DUALTIMER0 Control */
#define REG_LPMCU_MISC_REGS0_ARM_SLEEP_WAKEUP_CTRL (0x4000B190) /**< (LPMCU_MISC_REGS0) Sleep and Wakeup Control for the AON Power Sequencer */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_0 (0x4000B1A0) /**< (LPMCU_MISC_REGS0) Mega Mux Selection */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_1 (0x4000B1A4) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_2 (0x4000B1A8) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_3 (0x4000B1AC) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_4 (0x4000B1B0) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_5 (0x4000B1B4) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CLK_CTRL (0x4000B1C0) /**< (LPMCU_MISC_REGS0) Clock control for the Sensor ADC */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_RAW_STATUS (0x4000B1C4) /**< (LPMCU_MISC_REGS0) Raw Status from the Sensor ADC */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH0_DATA (0x4000B1C8) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 0 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH1_DATA (0x4000B1CC) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 1 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH2_DATA (0x4000B1D0) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 2 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH3_DATA (0x4000B1D4) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 3 */
#define REG_LPMCU_MISC_REGS0_IRQ_CTRL (0x4000B1E8) /**< (LPMCU_MISC_REGS0) Configure the IRQ inputs to the system as either Rising Edge or Level Shift */
#define REG_LPMCU_MISC_REGS0_IRQ_STS (0x4000B1EC) /**< (LPMCU_MISC_REGS0) Read the raw (no level or rise detect) IRQ inputs to the system from the designated cores */
#define REG_LPMCU_MISC_REGS0_MSEMI_MEM_CTRL (0x4000B200) /**< (LPMCU_MISC_REGS0) Control the rwm, rm and wm on the Mobile Semi Memories */
#define REG_LPMCU_MISC_REGS0_EFUSE_1_STATUS_3 (0x4000B204) /**< (LPMCU_MISC_REGS0) EFUSE_1_3 Status */
#define REG_LPMCU_MISC_REGS0_LPMCU_FORCE_CTRL (0x4000B210) /**< (LPMCU_MISC_REGS0) Miscellaneous Force control values for LPMCU */
#define REG_LPMCU_MISC_REGS0_ARM_IRQ_STATUS_0 (0x4000B214) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 IRQ Status */
#define REG_LPMCU_MISC_REGS0_ARM_IRQ_STATUS_1 (0x4000B218) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 IRQ Status */
#define REG_LPMCU_MISC_REGS0_BLE_DEEP_SLEEP_ENABLES (0x4000B240) /**< (LPMCU_MISC_REGS0) BLE Deep Sleep Enables */
#define REG_LPMCU_MISC_REGS0_DEEP_SLEEP_HW_TIMER_CORR (0x4000B244) /**< (LPMCU_MISC_REGS0) Configures the HW auto correction algorithm of the fine/base timers after wakeup from deep sleep */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM1_BASE_ADDR (0x4000B254) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 1 remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM1_END_ADDR (0x4000B258) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 1 remap end addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM2_BASE_ADDR (0x4000B25C) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 2 remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM2_END_ADDR (0x4000B260) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 2 remap end addr */
#define REG_LPMCU_MISC_REGS0_REMAP_BOOT_BASE_ADDR (0x4000B264) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 bootrom remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_BOOT_END_ADDR (0x4000B268) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 bootrom remap end addr */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC_IRQS (0x4000B280) /**< (LPMCU_MISC_REGS0) Reflects the IRQ status of the Quad Decoders */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_STATUS (0x4000B284) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_STATUS (0x4000B288) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_STATUS (0x4000B28C) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_THRESHOLD (0x4000B290) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_CTRL (0x4000B294) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_THRESHOLD (0x4000B2A0) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_CTRL (0x4000B2A4) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_THRESHOLD (0x4000B2B0) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_CTRL (0x4000B2B4) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP0_COMP (0x4000B2C0) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 0 */
#define REG_LPMCU_MISC_REGS0_FP_COMP1_COMP (0x4000B2C4) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 1 */
#define REG_LPMCU_MISC_REGS0_FP_COMP2_COMP (0x4000B2C8) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP3_COMP (0x4000B2CC) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 3 */
#define REG_LPMCU_MISC_REGS0_FP_COMP4_COMP (0x4000B2D0) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 4 */
#define REG_LPMCU_MISC_REGS0_FP_COMP5_COMP (0x4000B2D4) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 5 */
#define REG_LPMCU_MISC_REGS0_FP_COMP6_COMP (0x4000B2D8) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 6 */
#define REG_LPMCU_MISC_REGS0_FP_COMP7_COMP (0x4000B2DC) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 7 */
#define REG_LPMCU_MISC_REGS0_FP_COMP8_COMP (0x4000B2E0) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 8 */
#define REG_LPMCU_MISC_REGS0_FP_COMP9_COMP (0x4000B2E4) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 9 */
#define REG_LPMCU_MISC_REGS0_FP_COMP10_COMP (0x4000B2E8) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 10 */
#define REG_LPMCU_MISC_REGS0_FP_COMP11_COMP (0x4000B2EC) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 11 */
#define REG_LPMCU_MISC_REGS0_FP_COMP12_COMP (0x4000B2F0) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 12 */
#define REG_LPMCU_MISC_REGS0_FP_COMP13_COMP (0x4000B2F4) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 13 */
#define REG_LPMCU_MISC_REGS0_FP_COMP14_COMP (0x4000B2F8) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP15_COMP (0x4000B2FC) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 15 */
#define REG_LPMCU_MISC_REGS0_FP_COMP0_REPLACE (0x4000B300) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 0 */
#define REG_LPMCU_MISC_REGS0_FP_COMP1_REPLACE (0x4000B304) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 1 */
#define REG_LPMCU_MISC_REGS0_FP_COMP2_REPLACE (0x4000B308) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP3_REPLACE (0x4000B30C) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 3 */
#define REG_LPMCU_MISC_REGS0_FP_COMP4_REPLACE (0x4000B310) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 4 */
#define REG_LPMCU_MISC_REGS0_FP_COMP5_REPLACE (0x4000B314) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 5 */
#define REG_LPMCU_MISC_REGS0_FP_COMP6_REPLACE (0x4000B318) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 6 */
#define REG_LPMCU_MISC_REGS0_FP_COMP7_REPLACE (0x4000B31C) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 7 */
#define REG_LPMCU_MISC_REGS0_FP_COMP8_REPLACE (0x4000B320) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 8 */
#define REG_LPMCU_MISC_REGS0_FP_COMP9_REPLACE (0x4000B324) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 9 */
#define REG_LPMCU_MISC_REGS0_FP_COMP10_REPLACE (0x4000B328) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 10 */
#define REG_LPMCU_MISC_REGS0_FP_COMP11_REPLACE (0x4000B32C) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 11 */
#define REG_LPMCU_MISC_REGS0_FP_COMP12_REPLACE (0x4000B330) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 12 */
#define REG_LPMCU_MISC_REGS0_FP_COMP13_REPLACE (0x4000B334) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 13 */
#define REG_LPMCU_MISC_REGS0_FP_COMP14_REPLACE (0x4000B338) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP15_REPLACE (0x4000B33C) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP_ENABLE (0x4000B340) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Enable Register */
#define REG_LPMCU_MISC_REGS0_PMU_READ_REGS (0x4000B410) /**< (LPMCU_MISC_REGS0) PMU Status Registers */
#define REG_LPMCU_MISC_REGS0_MS_GPIO (0x4000B414) /**< (LPMCU_MISC_REGS0) Control for Mixed Signal GPIOs */
#define REG_LPMCU_MISC_REGS0_AON_SLEEP_TIMER_COUNTER (0x4000B420) /**< (LPMCU_MISC_REGS0) Current value of the AON Sleep Timer (syncd to ARM clock) */

#else

#define REG_LPMCU_MISC_REGS0_LPMCU_CHIP_ID (*(__I  uint32_t*)0x4000B000U) /**< (LPMCU_MISC_REGS0) The hard-coded ID for the chip. */
#define REG_LPMCU_MISC_REGS0_LPMCU_GLOBAL_RESET_0 (*(__IO uint32_t*)0x4000B004U) /**< (LPMCU_MISC_REGS0) Active Low Global Reset */
#define REG_LPMCU_MISC_REGS0_LPMCU_GLOBAL_RESET_1 (*(__IO uint16_t*)0x4000B008U) /**< (LPMCU_MISC_REGS0) Active Low Global Reset */
#define REG_LPMCU_MISC_REGS0_LPMCU_CLOCK_ENABLES_0 (*(__IO uint32_t*)0x4000B00CU) /**< (LPMCU_MISC_REGS0) LPMCU Clock Enables */
#define REG_LPMCU_MISC_REGS0_LPMCU_CLOCK_ENABLES_1 (*(__IO uint32_t*)0x4000B010U) /**< (LPMCU_MISC_REGS0) LPMCU Clock Enables */
#define REG_LPMCU_MISC_REGS0_BTMCU_CTRL (*(__IO uint8_t*)0x4000B014U) /**< (LPMCU_MISC_REGS0) Miscellaneous BLE control */
#define REG_LPMCU_MISC_REGS0_LPMCU_CTRL (*(__IO uint32_t*)0x4000B018U) /**< (LPMCU_MISC_REGS0) Miscellaneous LPMCU control */
#define REG_LPMCU_MISC_REGS0_LPMCU_MBIST (*(__IO uint8_t*)0x4000B01CU) /**< (LPMCU_MISC_REGS0) MBIST designation */
#define REG_LPMCU_MISC_REGS0_BLE_CTRL (*(__IO uint8_t*)0x4000B020U) /**< (LPMCU_MISC_REGS0) BLE Control */
#define REG_LPMCU_MISC_REGS0_MBIST_CTRL (*(__IO uint32_t*)0x4000B024U) /**< (LPMCU_MISC_REGS0) MBIST Control Register */
#define REG_LPMCU_MISC_REGS0_MBIST_STATUS (*(__I  uint32_t*)0x4000B028U) /**< (LPMCU_MISC_REGS0) MBIST Status Register */
#define REG_LPMCU_MISC_REGS0_MBIST_SRAM_ALG_SEL (*(__IO uint16_t*)0x4000B02CU) /**< (LPMCU_MISC_REGS0) Enables MBIST algorithms */
#define REG_LPMCU_MISC_REGS0_MBIST_BG (*(__IO uint32_t*)0x4000B030U) /**< (LPMCU_MISC_REGS0) MBIST Background for MARCH2 algorithm (bits 31:0) */
#define REG_LPMCU_MISC_REGS0_TEST_BUS_CTRL (*(__IO uint8_t*)0x4000B038U) /**< (LPMCU_MISC_REGS0) Controls all of the test bus functionality */
#define REG_LPMCU_MISC_REGS0_WP3_MISR_DATAOUT (*(__I  uint32_t*)0x4000B03CU) /**< (LPMCU_MISC_REGS0) ROM MBIST Signature */
#define REG_LPMCU_MISC_REGS0_INVERT_OUTPUT_CTRL (*(__IO uint32_t*)0x4000B040U) /**< (LPMCU_MISC_REGS0) Invert Output Control */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_0 (*(__IO uint32_t*)0x4000B044U) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_0, LP_GPIO_1, LP_GPIO_2, LP_GPIO_3, LP_GPIO_4, LP_GPIO_5, LP_GPIO_6, LP_GPIO_7 */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_1 (*(__IO uint32_t*)0x4000B048U) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_8, LP_GPIO_9, LP_GPIO_10, LP_GPIO_11, LP_GPIO_12, LP_GPIO_13, LP_GPIO_14, LP_GPIO_15 */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_2 (*(__IO uint32_t*)0x4000B04CU) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_GPIO_16, LP_GPIO_17, LP_GPIO_18, LP_GPIO_19, LP_GPIO_20, LP_GPIO_21, LP_GPIO_22, LP_GPIO_23 */
#define REG_LPMCU_MISC_REGS0_PULL_ENABLE (*(__IO uint32_t*)0x4000B050U) /**< (LPMCU_MISC_REGS0) Active Low Pull Enables for LPMCU Pads */
#define REG_LPMCU_MISC_REGS0_RTYPE_PAD_0 (*(__IO uint32_t*)0x4000B054U) /**< (LPMCU_MISC_REGS0) Controls the RTYPE (Pull Level) pad value for LPMCU Pads (0 = Pull Up) */
#define REG_LPMCU_MISC_REGS0_PINMUX_SEL_3 (*(__IO uint32_t*)0x4000B068U) /**< (LPMCU_MISC_REGS0) Pinmux select for LP_SIP_0, LP_SIP_1, LP_SIP_2, LP_SIP_3, LP_SIP_4 */
#define REG_LPMCU_MISC_REGS0_ISHAPE_PAD_3 (*(__IO uint32_t*)0x4000B06CU) /**< (LPMCU_MISC_REGS0) Controls the ISHAPE pad value for LPMCU SIP Pads */
#define REG_LPMCU_MISC_REGS0_LPMCU_CTRL_2 (*(__IO uint8_t*)0x4000B090U) /**< (LPMCU_MISC_REGS0) Misc control for the LPMCU */
#define REG_LPMCU_MISC_REGS0_SPIFLASH_VDDIO_CTRL (*(__IO uint8_t*)0x4000B0A0U) /**< (LPMCU_MISC_REGS0) Control for VDDIO of SPI FLASH */
#define REG_LPMCU_MISC_REGS0_SPIFLASH_BYPASS (*(__IO uint8_t*)0x4000B0A4U) /**< (LPMCU_MISC_REGS0) SPI FLASH Bypass */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_0 (*(__IO uint32_t*)0x4000B0C0U) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_1 (*(__IO uint32_t*)0x4000B0C4U) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_2 (*(__IO uint32_t*)0x4000B0C8U) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_3 (*(__IO uint32_t*)0x4000B0CCU) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_4 (*(__IO uint32_t*)0x4000B0D0U) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_IRQ_MUX_IO_SEL_5 (*(__IO uint8_t*)0x4000B0D4U) /**< (LPMCU_MISC_REGS0) ARM Cortex IRQ Mux Selection */
#define REG_LPMCU_MISC_REGS0_PWM0_CTRL (*(__IO uint32_t*)0x4000B160U) /**< (LPMCU_MISC_REGS0) PWM0 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM1_CTRL (*(__IO uint32_t*)0x4000B164U) /**< (LPMCU_MISC_REGS0) PWM1 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM2_CTRL (*(__IO uint32_t*)0x4000B168U) /**< (LPMCU_MISC_REGS0) PWM2 Control Register */
#define REG_LPMCU_MISC_REGS0_PWM3_CTRL (*(__IO uint32_t*)0x4000B16CU) /**< (LPMCU_MISC_REGS0) PWM3 Control Register */
#define REG_LPMCU_MISC_REGS0_MBIST_DUAL_STATUS (*(__I  uint32_t*)0x4000B170U) /**< (LPMCU_MISC_REGS0) Reflects the status of MBIST from lpmcu and btmcu */
#define REG_LPMCU_MISC_REGS0_DUALTIMER0_CTRL (*(__IO uint8_t*)0x4000B188U) /**< (LPMCU_MISC_REGS0) DUALTIMER0 Control */
#define REG_LPMCU_MISC_REGS0_ARM_SLEEP_WAKEUP_CTRL (*(__IO uint32_t*)0x4000B190U) /**< (LPMCU_MISC_REGS0) Sleep and Wakeup Control for the AON Power Sequencer */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_0 (*(__IO uint32_t*)0x4000B1A0U) /**< (LPMCU_MISC_REGS0) Mega Mux Selection */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_1 (*(__IO uint32_t*)0x4000B1A4U) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_2 (*(__IO uint32_t*)0x4000B1A8U) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_3 (*(__IO uint32_t*)0x4000B1ACU) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_4 (*(__IO uint32_t*)0x4000B1B0U) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_MEGA_MUX_IO_SEL_5 (*(__IO uint32_t*)0x4000B1B4U) /**< (LPMCU_MISC_REGS0) Mega Mux Selection (Refer to Mega_Mux_IO Spreadsheet) */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CLK_CTRL (*(__IO uint32_t*)0x4000B1C0U) /**< (LPMCU_MISC_REGS0) Clock control for the Sensor ADC */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_RAW_STATUS (*(__I  uint32_t*)0x4000B1C4U) /**< (LPMCU_MISC_REGS0) Raw Status from the Sensor ADC */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH0_DATA (*(__I  uint16_t*)0x4000B1C8U) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 0 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH1_DATA (*(__I  uint16_t*)0x4000B1CCU) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 1 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH2_DATA (*(__I  uint16_t*)0x4000B1D0U) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 2 */
#define REG_LPMCU_MISC_REGS0_SENS_ADC_CH3_DATA (*(__I  uint16_t*)0x4000B1D4U) /**< (LPMCU_MISC_REGS0) Current data of the Sensor ADC for Channel 3 */
#define REG_LPMCU_MISC_REGS0_IRQ_CTRL (*(__IO uint16_t*)0x4000B1E8U) /**< (LPMCU_MISC_REGS0) Configure the IRQ inputs to the system as either Rising Edge or Level Shift */
#define REG_LPMCU_MISC_REGS0_IRQ_STS (*(__I  uint8_t*)0x4000B1ECU) /**< (LPMCU_MISC_REGS0) Read the raw (no level or rise detect) IRQ inputs to the system from the designated cores */
#define REG_LPMCU_MISC_REGS0_MSEMI_MEM_CTRL (*(__IO uint32_t*)0x4000B200U) /**< (LPMCU_MISC_REGS0) Control the rwm, rm and wm on the Mobile Semi Memories */
#define REG_LPMCU_MISC_REGS0_EFUSE_1_STATUS_3 (*(__IO uint32_t*)0x4000B204U) /**< (LPMCU_MISC_REGS0) EFUSE_1_3 Status */
#define REG_LPMCU_MISC_REGS0_LPMCU_FORCE_CTRL (*(__IO uint16_t*)0x4000B210U) /**< (LPMCU_MISC_REGS0) Miscellaneous Force control values for LPMCU */
#define REG_LPMCU_MISC_REGS0_ARM_IRQ_STATUS_0 (*(__I  uint32_t*)0x4000B214U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 IRQ Status */
#define REG_LPMCU_MISC_REGS0_ARM_IRQ_STATUS_1 (*(__I  uint32_t*)0x4000B218U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 IRQ Status */
#define REG_LPMCU_MISC_REGS0_BLE_DEEP_SLEEP_ENABLES (*(__IO uint8_t*)0x4000B240U) /**< (LPMCU_MISC_REGS0) BLE Deep Sleep Enables */
#define REG_LPMCU_MISC_REGS0_DEEP_SLEEP_HW_TIMER_CORR (*(__IO uint32_t*)0x4000B244U) /**< (LPMCU_MISC_REGS0) Configures the HW auto correction algorithm of the fine/base timers after wakeup from deep sleep */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM1_BASE_ADDR (*(__IO uint32_t*)0x4000B254U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 1 remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM1_END_ADDR (*(__IO uint32_t*)0x4000B258U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 1 remap end addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM2_BASE_ADDR (*(__IO uint32_t*)0x4000B25CU) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 2 remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_IDRAM2_END_ADDR (*(__IO uint32_t*)0x4000B260U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 idram 2 remap end addr */
#define REG_LPMCU_MISC_REGS0_REMAP_BOOT_BASE_ADDR (*(__IO uint32_t*)0x4000B264U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 bootrom remap base addr */
#define REG_LPMCU_MISC_REGS0_REMAP_BOOT_END_ADDR (*(__IO uint32_t*)0x4000B268U) /**< (LPMCU_MISC_REGS0) ARM Cortex M0 bootrom remap end addr */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC_IRQS (*(__I  uint8_t*)0x4000B280U) /**< (LPMCU_MISC_REGS0) Reflects the IRQ status of the Quad Decoders */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_STATUS (*(__I  uint16_t*)0x4000B284U) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_STATUS (*(__I  uint16_t*)0x4000B288U) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_STATUS (*(__I  uint16_t*)0x4000B28CU) /**< (LPMCU_MISC_REGS0) Current status of QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_THRESHOLD (*(__IO uint32_t*)0x4000B290U) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC0_CTRL (*(__IO uint8_t*)0x4000B294U) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC0 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_THRESHOLD (*(__IO uint32_t*)0x4000B2A0U) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC1_CTRL (*(__IO uint8_t*)0x4000B2A4U) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC1 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_THRESHOLD (*(__IO uint32_t*)0x4000B2B0U) /**< (LPMCU_MISC_REGS0) Thresholds for QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_QUAD_DEC2_CTRL (*(__IO uint8_t*)0x4000B2B4U) /**< (LPMCU_MISC_REGS0) Control for QUAD_DEC2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP0_COMP (*(__IO uint32_t*)0x4000B2C0U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 0 */
#define REG_LPMCU_MISC_REGS0_FP_COMP1_COMP (*(__IO uint32_t*)0x4000B2C4U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 1 */
#define REG_LPMCU_MISC_REGS0_FP_COMP2_COMP (*(__IO uint32_t*)0x4000B2C8U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP3_COMP (*(__IO uint32_t*)0x4000B2CCU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 3 */
#define REG_LPMCU_MISC_REGS0_FP_COMP4_COMP (*(__IO uint32_t*)0x4000B2D0U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 4 */
#define REG_LPMCU_MISC_REGS0_FP_COMP5_COMP (*(__IO uint32_t*)0x4000B2D4U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 5 */
#define REG_LPMCU_MISC_REGS0_FP_COMP6_COMP (*(__IO uint32_t*)0x4000B2D8U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 6 */
#define REG_LPMCU_MISC_REGS0_FP_COMP7_COMP (*(__IO uint32_t*)0x4000B2DCU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 7 */
#define REG_LPMCU_MISC_REGS0_FP_COMP8_COMP (*(__IO uint32_t*)0x4000B2E0U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 8 */
#define REG_LPMCU_MISC_REGS0_FP_COMP9_COMP (*(__IO uint32_t*)0x4000B2E4U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 9 */
#define REG_LPMCU_MISC_REGS0_FP_COMP10_COMP (*(__IO uint32_t*)0x4000B2E8U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 10 */
#define REG_LPMCU_MISC_REGS0_FP_COMP11_COMP (*(__IO uint32_t*)0x4000B2ECU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 11 */
#define REG_LPMCU_MISC_REGS0_FP_COMP12_COMP (*(__IO uint32_t*)0x4000B2F0U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 12 */
#define REG_LPMCU_MISC_REGS0_FP_COMP13_COMP (*(__IO uint32_t*)0x4000B2F4U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 13 */
#define REG_LPMCU_MISC_REGS0_FP_COMP14_COMP (*(__IO uint32_t*)0x4000B2F8U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP15_COMP (*(__IO uint32_t*)0x4000B2FCU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Compare Register 15 */
#define REG_LPMCU_MISC_REGS0_FP_COMP0_REPLACE (*(__IO uint32_t*)0x4000B300U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 0 */
#define REG_LPMCU_MISC_REGS0_FP_COMP1_REPLACE (*(__IO uint32_t*)0x4000B304U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 1 */
#define REG_LPMCU_MISC_REGS0_FP_COMP2_REPLACE (*(__IO uint32_t*)0x4000B308U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 2 */
#define REG_LPMCU_MISC_REGS0_FP_COMP3_REPLACE (*(__IO uint32_t*)0x4000B30CU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 3 */
#define REG_LPMCU_MISC_REGS0_FP_COMP4_REPLACE (*(__IO uint32_t*)0x4000B310U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 4 */
#define REG_LPMCU_MISC_REGS0_FP_COMP5_REPLACE (*(__IO uint32_t*)0x4000B314U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 5 */
#define REG_LPMCU_MISC_REGS0_FP_COMP6_REPLACE (*(__IO uint32_t*)0x4000B318U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 6 */
#define REG_LPMCU_MISC_REGS0_FP_COMP7_REPLACE (*(__IO uint32_t*)0x4000B31CU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 7 */
#define REG_LPMCU_MISC_REGS0_FP_COMP8_REPLACE (*(__IO uint32_t*)0x4000B320U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 8 */
#define REG_LPMCU_MISC_REGS0_FP_COMP9_REPLACE (*(__IO uint32_t*)0x4000B324U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 9 */
#define REG_LPMCU_MISC_REGS0_FP_COMP10_REPLACE (*(__IO uint32_t*)0x4000B328U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 10 */
#define REG_LPMCU_MISC_REGS0_FP_COMP11_REPLACE (*(__IO uint32_t*)0x4000B32CU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 11 */
#define REG_LPMCU_MISC_REGS0_FP_COMP12_REPLACE (*(__IO uint32_t*)0x4000B330U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 12 */
#define REG_LPMCU_MISC_REGS0_FP_COMP13_REPLACE (*(__IO uint32_t*)0x4000B334U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 13 */
#define REG_LPMCU_MISC_REGS0_FP_COMP14_REPLACE (*(__IO uint32_t*)0x4000B338U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP15_REPLACE (*(__IO uint32_t*)0x4000B33CU) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Replace Register 14 */
#define REG_LPMCU_MISC_REGS0_FP_COMP_ENABLE (*(__IO uint16_t*)0x4000B340U) /**< (LPMCU_MISC_REGS0) Flash Breakpoint Enable Register */
#define REG_LPMCU_MISC_REGS0_PMU_READ_REGS (*(__I  uint32_t*)0x4000B410U) /**< (LPMCU_MISC_REGS0) PMU Status Registers */
#define REG_LPMCU_MISC_REGS0_MS_GPIO (*(__IO uint8_t*)0x4000B414U) /**< (LPMCU_MISC_REGS0) Control for Mixed Signal GPIOs */
#define REG_LPMCU_MISC_REGS0_AON_SLEEP_TIMER_COUNTER (*(__I  uint32_t*)0x4000B420U) /**< (LPMCU_MISC_REGS0) Current value of the AON Sleep Timer (syncd to ARM clock) */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_LPMCU_MISC_REGS0_INSTANCE_ */
