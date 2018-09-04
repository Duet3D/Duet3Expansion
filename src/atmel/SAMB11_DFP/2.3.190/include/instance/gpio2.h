/**
 * \file
 *
 * \brief Instance description for GPIO2
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
#ifndef _SAMB11_GPIO2_INSTANCE_H_
#define _SAMB11_GPIO2_INSTANCE_H_

/* ========== Register definition for GPIO2 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_GPIO2_DATA          (0x40013000) /**< (GPIO2) Data Value */
#define REG_GPIO2_DATAOUT       (0x40013004) /**< (GPIO2) Data Output Register Value */
#define REG_GPIO2_OUTENSET      (0x40013010) /**< (GPIO2) Output Enable Set */
#define REG_GPIO2_OUTENCLR      (0x40013014) /**< (GPIO2) Output Enable Clear */
#define REG_GPIO2_INTENSET      (0x40013020) /**< (GPIO2) Interrupt Enable Set */
#define REG_GPIO2_INTENCLR      (0x40013024) /**< (GPIO2) Interrupt Enable Clear */
#define REG_GPIO2_INTTYPESET    (0x40013028) /**< (GPIO2) Interrupt Type Set */
#define REG_GPIO2_INTTYPECLR    (0x4001302C) /**< (GPIO2) Interrupt Type Clear */
#define REG_GPIO2_INTPOLSET     (0x40013030) /**< (GPIO2) Polarity-level, edge IRQ Configuration */
#define REG_GPIO2_INTPOLCLR     (0x40013034) /**< (GPIO2) IRQ Configuration Clear */
#define REG_GPIO2_INTSTATUSCLEAR (0x40013038) /**< (GPIO2) Interrupt Status */
#define REG_GPIO2_PID4          (0x40013FD0) /**< (GPIO2) Peripheral ID Register 4 */
#define REG_GPIO2_PID5          (0x40013FD4) /**< (GPIO2) Peripheral ID Register 5 */
#define REG_GPIO2_PID6          (0x40013FD8) /**< (GPIO2) Peripheral ID Register 6 */
#define REG_GPIO2_PID7          (0x40013FDC) /**< (GPIO2) Peripheral ID Register 7 */
#define REG_GPIO2_PID0          (0x40013FE0) /**< (GPIO2) Peripheral ID Register 0 */
#define REG_GPIO2_PID1          (0x40013FE4) /**< (GPIO2) Peripheral ID Register 1 */
#define REG_GPIO2_PID2          (0x40013FE8) /**< (GPIO2) Peripheral ID Register 2 */
#define REG_GPIO2_PID3          (0x40013FEC) /**< (GPIO2) Peripheral ID Register 3 */
#define REG_GPIO2_CID0          (0x40013FF0) /**< (GPIO2) Component ID Register 0 */
#define REG_GPIO2_CID1          (0x40013FF4) /**< (GPIO2) Component ID Register 1 */
#define REG_GPIO2_CID2          (0x40013FF8) /**< (GPIO2) Component ID Register 2 */
#define REG_GPIO2_CID3          (0x40013FFC) /**< (GPIO2) Component ID Register 3 */

#else

#define REG_GPIO2_DATA          (*(__IO uint16_t*)0x40013000U) /**< (GPIO2) Data Value */
#define REG_GPIO2_DATAOUT       (*(__IO uint16_t*)0x40013004U) /**< (GPIO2) Data Output Register Value */
#define REG_GPIO2_OUTENSET      (*(__IO uint16_t*)0x40013010U) /**< (GPIO2) Output Enable Set */
#define REG_GPIO2_OUTENCLR      (*(__IO uint16_t*)0x40013014U) /**< (GPIO2) Output Enable Clear */
#define REG_GPIO2_INTENSET      (*(__IO uint16_t*)0x40013020U) /**< (GPIO2) Interrupt Enable Set */
#define REG_GPIO2_INTENCLR      (*(__IO uint16_t*)0x40013024U) /**< (GPIO2) Interrupt Enable Clear */
#define REG_GPIO2_INTTYPESET    (*(__IO uint16_t*)0x40013028U) /**< (GPIO2) Interrupt Type Set */
#define REG_GPIO2_INTTYPECLR    (*(__IO uint16_t*)0x4001302CU) /**< (GPIO2) Interrupt Type Clear */
#define REG_GPIO2_INTPOLSET     (*(__IO uint16_t*)0x40013030U) /**< (GPIO2) Polarity-level, edge IRQ Configuration */
#define REG_GPIO2_INTPOLCLR     (*(__IO uint16_t*)0x40013034U) /**< (GPIO2) IRQ Configuration Clear */
#define REG_GPIO2_INTSTATUSCLEAR (*(__IO uint16_t*)0x40013038U) /**< (GPIO2) Interrupt Status */
#define REG_GPIO2_PID4          (*(__I  uint8_t*)0x40013FD0U) /**< (GPIO2) Peripheral ID Register 4 */
#define REG_GPIO2_PID5          (*(__I  uint8_t*)0x40013FD4U) /**< (GPIO2) Peripheral ID Register 5 */
#define REG_GPIO2_PID6          (*(__I  uint8_t*)0x40013FD8U) /**< (GPIO2) Peripheral ID Register 6 */
#define REG_GPIO2_PID7          (*(__I  uint8_t*)0x40013FDCU) /**< (GPIO2) Peripheral ID Register 7 */
#define REG_GPIO2_PID0          (*(__I  uint8_t*)0x40013FE0U) /**< (GPIO2) Peripheral ID Register 0 */
#define REG_GPIO2_PID1          (*(__I  uint8_t*)0x40013FE4U) /**< (GPIO2) Peripheral ID Register 1 */
#define REG_GPIO2_PID2          (*(__I  uint8_t*)0x40013FE8U) /**< (GPIO2) Peripheral ID Register 2 */
#define REG_GPIO2_PID3          (*(__I  uint8_t*)0x40013FECU) /**< (GPIO2) Peripheral ID Register 3 */
#define REG_GPIO2_CID0          (*(__I  uint8_t*)0x40013FF0U) /**< (GPIO2) Component ID Register 0 */
#define REG_GPIO2_CID1          (*(__I  uint8_t*)0x40013FF4U) /**< (GPIO2) Component ID Register 1 */
#define REG_GPIO2_CID2          (*(__I  uint8_t*)0x40013FF8U) /**< (GPIO2) Component ID Register 2 */
#define REG_GPIO2_CID3          (*(__I  uint8_t*)0x40013FFCU) /**< (GPIO2) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_GPIO2_INSTANCE_ */
