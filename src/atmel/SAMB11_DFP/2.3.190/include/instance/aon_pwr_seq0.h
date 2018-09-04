/**
 * \file
 *
 * \brief Instance description for AON_PWR_SEQ0
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
#ifndef _SAMB11_AON_PWR_SEQ0_INSTANCE_H_
#define _SAMB11_AON_PWR_SEQ0_INSTANCE_H_

/* ========== Register definition for AON_PWR_SEQ0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_AON_PWR_SEQ0_GPIO_WAKEUP_CTRL (0x4000E000) /**< (AON_PWR_SEQ0) Controls the wakeup enable for GPIO_0, GPIO_1 and GPIO_2 (Pinmux will control the ARM wakeup) */
#define REG_AON_PWR_SEQ0_AON_ST_WAKEUP_CTRL (0x4000E00C) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the Always On Sleep Timer (AON_ST) */
#define REG_AON_PWR_SEQ0_LPMCU_WAKEUP_CTRL (0x4000E010) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the Low Power Micro Controller Unit (LPMCU) */
#define REG_AON_PWR_SEQ0_BLE_ST_WAKEUP_CTRL (0x4000E014) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the BLE Sleep Timer (BLE_ST) */
#define REG_AON_PWR_SEQ0_LPMCU_SLEEP_1_CTRL (0x4000E020) /**< (AON_PWR_SEQ0) Controls the sleep and retention options for SLEEP request 1 from the ARM */
#define REG_AON_PWR_SEQ0_OFF_DELAY_0_CTRL (0x4000E040) /**< (AON_PWR_SEQ0) Power OFF Delays */
#define REG_AON_PWR_SEQ0_OFF_DELAY_1_CTRL (0x4000E044) /**< (AON_PWR_SEQ0) Power OFF Delays */
#define REG_AON_PWR_SEQ0_ON_DELAY_0_CTRL (0x4000E048) /**< (AON_PWR_SEQ0) Power ON Delays */
#define REG_AON_PWR_SEQ0_ON_DELAY_1_CTRL (0x4000E04C) /**< (AON_PWR_SEQ0) Power ON Delays */
#define REG_AON_PWR_SEQ0_VDD_DCDC_EN_DELAY_CTRL (0x4000E050) /**< (AON_PWR_SEQ0) VDD_DCDC_EN Delay Control */
#define REG_AON_PWR_SEQ0_MISC_BYPASS_0_CTRL (0x4000E218) /**< (AON_PWR_SEQ0) Bypass control for misc signals */
#define REG_AON_PWR_SEQ0_RAW_WAKEUP_BITS (0x4000E300) /**< (AON_PWR_SEQ0) Raw Wakeup Bits */
#define REG_AON_PWR_SEQ0_PD_WAKEUP_BITS (0x4000E304) /**< (AON_PWR_SEQ0) Individual Power Domain Wakeup Bits */
#define REG_AON_PWR_SEQ0_SERVICED_REQUEST (0x4000E308) /**< (AON_PWR_SEQ0) Serviced Request Status Bits */
#define REG_AON_PWR_SEQ0_ACTIVE_REQUEST (0x4000E30C) /**< (AON_PWR_SEQ0) Active Request Status Bits */
#define REG_AON_PWR_SEQ0_LOGIC_FSM_STATES (0x4000E3F0) /**< (AON_PWR_SEQ0) Current Logic FSM States */

#else

#define REG_AON_PWR_SEQ0_GPIO_WAKEUP_CTRL (*(__IO uint8_t*)0x4000E000U) /**< (AON_PWR_SEQ0) Controls the wakeup enable for GPIO_0, GPIO_1 and GPIO_2 (Pinmux will control the ARM wakeup) */
#define REG_AON_PWR_SEQ0_AON_ST_WAKEUP_CTRL (*(__IO uint8_t*)0x4000E00CU) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the Always On Sleep Timer (AON_ST) */
#define REG_AON_PWR_SEQ0_LPMCU_WAKEUP_CTRL (*(__IO uint16_t*)0x4000E010U) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the Low Power Micro Controller Unit (LPMCU) */
#define REG_AON_PWR_SEQ0_BLE_ST_WAKEUP_CTRL (*(__IO uint8_t*)0x4000E014U) /**< (AON_PWR_SEQ0) Controls the wakeup enable for the BLE Sleep Timer (BLE_ST) */
#define REG_AON_PWR_SEQ0_LPMCU_SLEEP_1_CTRL (*(__IO uint32_t*)0x4000E020U) /**< (AON_PWR_SEQ0) Controls the sleep and retention options for SLEEP request 1 from the ARM */
#define REG_AON_PWR_SEQ0_OFF_DELAY_0_CTRL (*(__IO uint32_t*)0x4000E040U) /**< (AON_PWR_SEQ0) Power OFF Delays */
#define REG_AON_PWR_SEQ0_OFF_DELAY_1_CTRL (*(__IO uint32_t*)0x4000E044U) /**< (AON_PWR_SEQ0) Power OFF Delays */
#define REG_AON_PWR_SEQ0_ON_DELAY_0_CTRL (*(__IO uint16_t*)0x4000E048U) /**< (AON_PWR_SEQ0) Power ON Delays */
#define REG_AON_PWR_SEQ0_ON_DELAY_1_CTRL (*(__IO uint32_t*)0x4000E04CU) /**< (AON_PWR_SEQ0) Power ON Delays */
#define REG_AON_PWR_SEQ0_VDD_DCDC_EN_DELAY_CTRL (*(__IO uint16_t*)0x4000E050U) /**< (AON_PWR_SEQ0) VDD_DCDC_EN Delay Control */
#define REG_AON_PWR_SEQ0_MISC_BYPASS_0_CTRL (*(__IO uint16_t*)0x4000E218U) /**< (AON_PWR_SEQ0) Bypass control for misc signals */
#define REG_AON_PWR_SEQ0_RAW_WAKEUP_BITS (*(__I  uint8_t*)0x4000E300U) /**< (AON_PWR_SEQ0) Raw Wakeup Bits */
#define REG_AON_PWR_SEQ0_PD_WAKEUP_BITS (*(__I  uint16_t*)0x4000E304U) /**< (AON_PWR_SEQ0) Individual Power Domain Wakeup Bits */
#define REG_AON_PWR_SEQ0_SERVICED_REQUEST (*(__I  uint16_t*)0x4000E308U) /**< (AON_PWR_SEQ0) Serviced Request Status Bits */
#define REG_AON_PWR_SEQ0_ACTIVE_REQUEST (*(__I  uint16_t*)0x4000E30CU) /**< (AON_PWR_SEQ0) Active Request Status Bits */
#define REG_AON_PWR_SEQ0_LOGIC_FSM_STATES (*(__I  uint32_t*)0x4000E3F0U) /**< (AON_PWR_SEQ0) Current Logic FSM States */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_AON_PWR_SEQ0_INSTANCE_ */
