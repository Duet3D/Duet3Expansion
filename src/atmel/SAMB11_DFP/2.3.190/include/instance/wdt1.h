/**
 * \file
 *
 * \brief Instance description for WDT1
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
#ifndef _SAMB11_WDT1_INSTANCE_H_
#define _SAMB11_WDT1_INSTANCE_H_

/* ========== Register definition for WDT1 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_WDT1_WDOGLOAD       (0x40009000) /**< (WDT1) Watchdog Load Register */
#define REG_WDT1_WDOGVALUE      (0x40009004) /**< (WDT1) Watchdog Value Register */
#define REG_WDT1_WDOGCONTROL    (0x40009008) /**< (WDT1) Watchdog Control Register */
#define REG_WDT1_WDOGINTCLR     (0x4000900C) /**< (WDT1) Watchdog Clear Interrupt Register */
#define REG_WDT1_WDOGRIS        (0x40009010) /**< (WDT1) Watchdog Raw Interrupt Status Register */
#define REG_WDT1_WDOGMIS        (0x40009014) /**< (WDT1) Watchdog Interrupt Status Register */
#define REG_WDT1_WDOGLOCK       (0x40009C00) /**< (WDT1) Watchdog Lock Register (Write 0x1ACCE551 to enable write access to all other registers) */
#define REG_WDT1_WDOGITCR       (0x40009F00) /**< (WDT1) Watchdog Integration Test Control Register */
#define REG_WDT1_WDOGITOP       (0x40009F04) /**< (WDT1) Watchdog Integration Test Output Set Register */
#define REG_WDT1_WDOGPERIPHID4  (0x40009FD0) /**< (WDT1) Peripheral ID Register 4 */
#define REG_WDT1_WDOGPERIPHID5  (0x40009FD4) /**< (WDT1) Peripheral ID Register 5 */
#define REG_WDT1_WDOGPERIPHID6  (0x40009FD8) /**< (WDT1) Peripheral ID Register 6 */
#define REG_WDT1_WDOGPERIPHID7  (0x40009FDC) /**< (WDT1) Peripheral ID Register 7 */
#define REG_WDT1_WDOGPERIPHID0  (0x40009FE0) /**< (WDT1) Peripheral ID Register 0 */
#define REG_WDT1_WDOGPERIPHID1  (0x40009FE4) /**< (WDT1) Peripheral ID Register 1 */
#define REG_WDT1_WDOGPERIPHID2  (0x40009FE8) /**< (WDT1) Peripheral ID Register 2 */
#define REG_WDT1_WDOGPERIPHID3  (0x40009FEC) /**< (WDT1) Peripheral ID Register 3 */
#define REG_WDT1_WDOGPCELLID0   (0x40009FF0) /**< (WDT1) Component ID Register 0 */
#define REG_WDT1_WDOGPCELLID1   (0x40009FF4) /**< (WDT1) Component ID Register 1 */
#define REG_WDT1_WDOGPCELLID2   (0x40009FF8) /**< (WDT1) Component ID Register 2 */
#define REG_WDT1_WDOGPCELLID3   (0x40009FFC) /**< (WDT1) Component ID Register 3 */

#else

#define REG_WDT1_WDOGLOAD       (*(__IO uint32_t*)0x40009000U) /**< (WDT1) Watchdog Load Register */
#define REG_WDT1_WDOGVALUE      (*(__I  uint32_t*)0x40009004U) /**< (WDT1) Watchdog Value Register */
#define REG_WDT1_WDOGCONTROL    (*(__IO uint8_t*)0x40009008U) /**< (WDT1) Watchdog Control Register */
#define REG_WDT1_WDOGINTCLR     (*(__O  uint8_t*)0x4000900CU) /**< (WDT1) Watchdog Clear Interrupt Register */
#define REG_WDT1_WDOGRIS        (*(__I  uint8_t*)0x40009010U) /**< (WDT1) Watchdog Raw Interrupt Status Register */
#define REG_WDT1_WDOGMIS        (*(__I  uint8_t*)0x40009014U) /**< (WDT1) Watchdog Interrupt Status Register */
#define REG_WDT1_WDOGLOCK       (*(__IO uint32_t*)0x40009C00U) /**< (WDT1) Watchdog Lock Register (Write 0x1ACCE551 to enable write access to all other registers) */
#define REG_WDT1_WDOGITCR       (*(__IO uint8_t*)0x40009F00U) /**< (WDT1) Watchdog Integration Test Control Register */
#define REG_WDT1_WDOGITOP       (*(__IO uint8_t*)0x40009F04U) /**< (WDT1) Watchdog Integration Test Output Set Register */
#define REG_WDT1_WDOGPERIPHID4  (*(__I  uint8_t*)0x40009FD0U) /**< (WDT1) Peripheral ID Register 4 */
#define REG_WDT1_WDOGPERIPHID5  (*(__I  uint8_t*)0x40009FD4U) /**< (WDT1) Peripheral ID Register 5 */
#define REG_WDT1_WDOGPERIPHID6  (*(__I  uint8_t*)0x40009FD8U) /**< (WDT1) Peripheral ID Register 6 */
#define REG_WDT1_WDOGPERIPHID7  (*(__I  uint8_t*)0x40009FDCU) /**< (WDT1) Peripheral ID Register 7 */
#define REG_WDT1_WDOGPERIPHID0  (*(__I  uint8_t*)0x40009FE0U) /**< (WDT1) Peripheral ID Register 0 */
#define REG_WDT1_WDOGPERIPHID1  (*(__I  uint8_t*)0x40009FE4U) /**< (WDT1) Peripheral ID Register 1 */
#define REG_WDT1_WDOGPERIPHID2  (*(__I  uint8_t*)0x40009FE8U) /**< (WDT1) Peripheral ID Register 2 */
#define REG_WDT1_WDOGPERIPHID3  (*(__I  uint8_t*)0x40009FECU) /**< (WDT1) Peripheral ID Register 3 */
#define REG_WDT1_WDOGPCELLID0   (*(__I  uint8_t*)0x40009FF0U) /**< (WDT1) Component ID Register 0 */
#define REG_WDT1_WDOGPCELLID1   (*(__I  uint8_t*)0x40009FF4U) /**< (WDT1) Component ID Register 1 */
#define REG_WDT1_WDOGPCELLID2   (*(__I  uint8_t*)0x40009FF8U) /**< (WDT1) Component ID Register 2 */
#define REG_WDT1_WDOGPCELLID3   (*(__I  uint8_t*)0x40009FFCU) /**< (WDT1) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_WDT1_INSTANCE_ */
