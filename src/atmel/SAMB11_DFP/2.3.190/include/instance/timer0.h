/**
 * \file
 *
 * \brief Instance description for TIMER0
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
#ifndef _SAMB11_TIMER0_INSTANCE_H_
#define _SAMB11_TIMER0_INSTANCE_H_

/* ========== Register definition for TIMER0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_TIMER0_CTRL         (0x40000000) /**< (TIMER0) Timer Control */
#define REG_TIMER0_VALUE        (0x40000004) /**< (TIMER0) Current Value */
#define REG_TIMER0_RELOAD       (0x40000008) /**< (TIMER0) Reload Value */
#define REG_TIMER0_INTSTATUSCLEAR (0x4000000C) /**< (TIMER0) Timer Interrupt, write 1 to clear */
#define REG_TIMER0_PID4         (0x40000FD0) /**< (TIMER0) Peripheral ID Register 4 */
#define REG_TIMER0_PID5         (0x40000FD4) /**< (TIMER0) Peripheral ID Register 5 */
#define REG_TIMER0_PID6         (0x40000FD8) /**< (TIMER0) Peripheral ID Register 6 */
#define REG_TIMER0_PID7         (0x40000FDC) /**< (TIMER0) Peripheral ID Register 7 */
#define REG_TIMER0_PID0         (0x40000FE0) /**< (TIMER0) Peripheral ID Register 0 */
#define REG_TIMER0_PID1         (0x40000FE4) /**< (TIMER0) Peripheral ID Register 1 */
#define REG_TIMER0_PID2         (0x40000FE8) /**< (TIMER0) Peripheral ID Register 2 */
#define REG_TIMER0_PID3         (0x40000FEC) /**< (TIMER0) Peripheral ID Register 3 */
#define REG_TIMER0_CID0         (0x40000FF0) /**< (TIMER0) Component ID Register 0 */
#define REG_TIMER0_CID1         (0x40000FF4) /**< (TIMER0) Component ID Register 1 */
#define REG_TIMER0_CID2         (0x40000FF8) /**< (TIMER0) Component ID Register 2 */
#define REG_TIMER0_CID3         (0x40000FFC) /**< (TIMER0) Component ID Register 3 */

#else

#define REG_TIMER0_CTRL         (*(__IO uint8_t*)0x40000000U) /**< (TIMER0) Timer Control */
#define REG_TIMER0_VALUE        (*(__IO uint32_t*)0x40000004U) /**< (TIMER0) Current Value */
#define REG_TIMER0_RELOAD       (*(__IO uint32_t*)0x40000008U) /**< (TIMER0) Reload Value */
#define REG_TIMER0_INTSTATUSCLEAR (*(__IO uint8_t*)0x4000000CU) /**< (TIMER0) Timer Interrupt, write 1 to clear */
#define REG_TIMER0_PID4         (*(__I  uint8_t*)0x40000FD0U) /**< (TIMER0) Peripheral ID Register 4 */
#define REG_TIMER0_PID5         (*(__I  uint8_t*)0x40000FD4U) /**< (TIMER0) Peripheral ID Register 5 */
#define REG_TIMER0_PID6         (*(__I  uint8_t*)0x40000FD8U) /**< (TIMER0) Peripheral ID Register 6 */
#define REG_TIMER0_PID7         (*(__I  uint8_t*)0x40000FDCU) /**< (TIMER0) Peripheral ID Register 7 */
#define REG_TIMER0_PID0         (*(__I  uint8_t*)0x40000FE0U) /**< (TIMER0) Peripheral ID Register 0 */
#define REG_TIMER0_PID1         (*(__I  uint8_t*)0x40000FE4U) /**< (TIMER0) Peripheral ID Register 1 */
#define REG_TIMER0_PID2         (*(__I  uint8_t*)0x40000FE8U) /**< (TIMER0) Peripheral ID Register 2 */
#define REG_TIMER0_PID3         (*(__I  uint8_t*)0x40000FECU) /**< (TIMER0) Peripheral ID Register 3 */
#define REG_TIMER0_CID0         (*(__I  uint8_t*)0x40000FF0U) /**< (TIMER0) Component ID Register 0 */
#define REG_TIMER0_CID1         (*(__I  uint8_t*)0x40000FF4U) /**< (TIMER0) Component ID Register 1 */
#define REG_TIMER0_CID2         (*(__I  uint8_t*)0x40000FF8U) /**< (TIMER0) Component ID Register 2 */
#define REG_TIMER0_CID3         (*(__I  uint8_t*)0x40000FFCU) /**< (TIMER0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_TIMER0_INSTANCE_ */
