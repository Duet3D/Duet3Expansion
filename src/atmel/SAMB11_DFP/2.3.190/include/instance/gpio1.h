/**
 * \file
 *
 * \brief Instance description for GPIO1
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
#ifndef _SAMB11_GPIO1_INSTANCE_H_
#define _SAMB11_GPIO1_INSTANCE_H_

/* ========== Register definition for GPIO1 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_GPIO1_DATA          (0x40011000) /**< (GPIO1) Data Value */
#define REG_GPIO1_DATAOUT       (0x40011004) /**< (GPIO1) Data Output Register Value */
#define REG_GPIO1_OUTENSET      (0x40011010) /**< (GPIO1) Output Enable Set */
#define REG_GPIO1_OUTENCLR      (0x40011014) /**< (GPIO1) Output Enable Clear */
#define REG_GPIO1_INTENSET      (0x40011020) /**< (GPIO1) Interrupt Enable Set */
#define REG_GPIO1_INTENCLR      (0x40011024) /**< (GPIO1) Interrupt Enable Clear */
#define REG_GPIO1_INTTYPESET    (0x40011028) /**< (GPIO1) Interrupt Type Set */
#define REG_GPIO1_INTTYPECLR    (0x4001102C) /**< (GPIO1) Interrupt Type Clear */
#define REG_GPIO1_INTPOLSET     (0x40011030) /**< (GPIO1) Polarity-level, edge IRQ Configuration */
#define REG_GPIO1_INTPOLCLR     (0x40011034) /**< (GPIO1) IRQ Configuration Clear */
#define REG_GPIO1_INTSTATUSCLEAR (0x40011038) /**< (GPIO1) Interrupt Status */
#define REG_GPIO1_PID4          (0x40011FD0) /**< (GPIO1) Peripheral ID Register 4 */
#define REG_GPIO1_PID5          (0x40011FD4) /**< (GPIO1) Peripheral ID Register 5 */
#define REG_GPIO1_PID6          (0x40011FD8) /**< (GPIO1) Peripheral ID Register 6 */
#define REG_GPIO1_PID7          (0x40011FDC) /**< (GPIO1) Peripheral ID Register 7 */
#define REG_GPIO1_PID0          (0x40011FE0) /**< (GPIO1) Peripheral ID Register 0 */
#define REG_GPIO1_PID1          (0x40011FE4) /**< (GPIO1) Peripheral ID Register 1 */
#define REG_GPIO1_PID2          (0x40011FE8) /**< (GPIO1) Peripheral ID Register 2 */
#define REG_GPIO1_PID3          (0x40011FEC) /**< (GPIO1) Peripheral ID Register 3 */
#define REG_GPIO1_CID0          (0x40011FF0) /**< (GPIO1) Component ID Register 0 */
#define REG_GPIO1_CID1          (0x40011FF4) /**< (GPIO1) Component ID Register 1 */
#define REG_GPIO1_CID2          (0x40011FF8) /**< (GPIO1) Component ID Register 2 */
#define REG_GPIO1_CID3          (0x40011FFC) /**< (GPIO1) Component ID Register 3 */

#else

#define REG_GPIO1_DATA          (*(__IO uint16_t*)0x40011000U) /**< (GPIO1) Data Value */
#define REG_GPIO1_DATAOUT       (*(__IO uint16_t*)0x40011004U) /**< (GPIO1) Data Output Register Value */
#define REG_GPIO1_OUTENSET      (*(__IO uint16_t*)0x40011010U) /**< (GPIO1) Output Enable Set */
#define REG_GPIO1_OUTENCLR      (*(__IO uint16_t*)0x40011014U) /**< (GPIO1) Output Enable Clear */
#define REG_GPIO1_INTENSET      (*(__IO uint16_t*)0x40011020U) /**< (GPIO1) Interrupt Enable Set */
#define REG_GPIO1_INTENCLR      (*(__IO uint16_t*)0x40011024U) /**< (GPIO1) Interrupt Enable Clear */
#define REG_GPIO1_INTTYPESET    (*(__IO uint16_t*)0x40011028U) /**< (GPIO1) Interrupt Type Set */
#define REG_GPIO1_INTTYPECLR    (*(__IO uint16_t*)0x4001102CU) /**< (GPIO1) Interrupt Type Clear */
#define REG_GPIO1_INTPOLSET     (*(__IO uint16_t*)0x40011030U) /**< (GPIO1) Polarity-level, edge IRQ Configuration */
#define REG_GPIO1_INTPOLCLR     (*(__IO uint16_t*)0x40011034U) /**< (GPIO1) IRQ Configuration Clear */
#define REG_GPIO1_INTSTATUSCLEAR (*(__IO uint16_t*)0x40011038U) /**< (GPIO1) Interrupt Status */
#define REG_GPIO1_PID4          (*(__I  uint8_t*)0x40011FD0U) /**< (GPIO1) Peripheral ID Register 4 */
#define REG_GPIO1_PID5          (*(__I  uint8_t*)0x40011FD4U) /**< (GPIO1) Peripheral ID Register 5 */
#define REG_GPIO1_PID6          (*(__I  uint8_t*)0x40011FD8U) /**< (GPIO1) Peripheral ID Register 6 */
#define REG_GPIO1_PID7          (*(__I  uint8_t*)0x40011FDCU) /**< (GPIO1) Peripheral ID Register 7 */
#define REG_GPIO1_PID0          (*(__I  uint8_t*)0x40011FE0U) /**< (GPIO1) Peripheral ID Register 0 */
#define REG_GPIO1_PID1          (*(__I  uint8_t*)0x40011FE4U) /**< (GPIO1) Peripheral ID Register 1 */
#define REG_GPIO1_PID2          (*(__I  uint8_t*)0x40011FE8U) /**< (GPIO1) Peripheral ID Register 2 */
#define REG_GPIO1_PID3          (*(__I  uint8_t*)0x40011FECU) /**< (GPIO1) Peripheral ID Register 3 */
#define REG_GPIO1_CID0          (*(__I  uint8_t*)0x40011FF0U) /**< (GPIO1) Component ID Register 0 */
#define REG_GPIO1_CID1          (*(__I  uint8_t*)0x40011FF4U) /**< (GPIO1) Component ID Register 1 */
#define REG_GPIO1_CID2          (*(__I  uint8_t*)0x40011FF8U) /**< (GPIO1) Component ID Register 2 */
#define REG_GPIO1_CID3          (*(__I  uint8_t*)0x40011FFCU) /**< (GPIO1) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_GPIO1_INSTANCE_ */
