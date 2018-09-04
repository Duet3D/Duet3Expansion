/**
 * \file
 *
 * \brief Instance description for GPIO0
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
#ifndef _SAMB11_GPIO0_INSTANCE_H_
#define _SAMB11_GPIO0_INSTANCE_H_

/* ========== Register definition for GPIO0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_GPIO0_DATA          (0x40010000) /**< (GPIO0) Data Value */
#define REG_GPIO0_DATAOUT       (0x40010004) /**< (GPIO0) Data Output Register Value */
#define REG_GPIO0_OUTENSET      (0x40010010) /**< (GPIO0) Output Enable Set */
#define REG_GPIO0_OUTENCLR      (0x40010014) /**< (GPIO0) Output Enable Clear */
#define REG_GPIO0_INTENSET      (0x40010020) /**< (GPIO0) Interrupt Enable Set */
#define REG_GPIO0_INTENCLR      (0x40010024) /**< (GPIO0) Interrupt Enable Clear */
#define REG_GPIO0_INTTYPESET    (0x40010028) /**< (GPIO0) Interrupt Type Set */
#define REG_GPIO0_INTTYPECLR    (0x4001002C) /**< (GPIO0) Interrupt Type Clear */
#define REG_GPIO0_INTPOLSET     (0x40010030) /**< (GPIO0) Polarity-level, edge IRQ Configuration */
#define REG_GPIO0_INTPOLCLR     (0x40010034) /**< (GPIO0) IRQ Configuration Clear */
#define REG_GPIO0_INTSTATUSCLEAR (0x40010038) /**< (GPIO0) Interrupt Status */
#define REG_GPIO0_PID4          (0x40010FD0) /**< (GPIO0) Peripheral ID Register 4 */
#define REG_GPIO0_PID5          (0x40010FD4) /**< (GPIO0) Peripheral ID Register 5 */
#define REG_GPIO0_PID6          (0x40010FD8) /**< (GPIO0) Peripheral ID Register 6 */
#define REG_GPIO0_PID7          (0x40010FDC) /**< (GPIO0) Peripheral ID Register 7 */
#define REG_GPIO0_PID0          (0x40010FE0) /**< (GPIO0) Peripheral ID Register 0 */
#define REG_GPIO0_PID1          (0x40010FE4) /**< (GPIO0) Peripheral ID Register 1 */
#define REG_GPIO0_PID2          (0x40010FE8) /**< (GPIO0) Peripheral ID Register 2 */
#define REG_GPIO0_PID3          (0x40010FEC) /**< (GPIO0) Peripheral ID Register 3 */
#define REG_GPIO0_CID0          (0x40010FF0) /**< (GPIO0) Component ID Register 0 */
#define REG_GPIO0_CID1          (0x40010FF4) /**< (GPIO0) Component ID Register 1 */
#define REG_GPIO0_CID2          (0x40010FF8) /**< (GPIO0) Component ID Register 2 */
#define REG_GPIO0_CID3          (0x40010FFC) /**< (GPIO0) Component ID Register 3 */

#else

#define REG_GPIO0_DATA          (*(__IO uint16_t*)0x40010000U) /**< (GPIO0) Data Value */
#define REG_GPIO0_DATAOUT       (*(__IO uint16_t*)0x40010004U) /**< (GPIO0) Data Output Register Value */
#define REG_GPIO0_OUTENSET      (*(__IO uint16_t*)0x40010010U) /**< (GPIO0) Output Enable Set */
#define REG_GPIO0_OUTENCLR      (*(__IO uint16_t*)0x40010014U) /**< (GPIO0) Output Enable Clear */
#define REG_GPIO0_INTENSET      (*(__IO uint16_t*)0x40010020U) /**< (GPIO0) Interrupt Enable Set */
#define REG_GPIO0_INTENCLR      (*(__IO uint16_t*)0x40010024U) /**< (GPIO0) Interrupt Enable Clear */
#define REG_GPIO0_INTTYPESET    (*(__IO uint16_t*)0x40010028U) /**< (GPIO0) Interrupt Type Set */
#define REG_GPIO0_INTTYPECLR    (*(__IO uint16_t*)0x4001002CU) /**< (GPIO0) Interrupt Type Clear */
#define REG_GPIO0_INTPOLSET     (*(__IO uint16_t*)0x40010030U) /**< (GPIO0) Polarity-level, edge IRQ Configuration */
#define REG_GPIO0_INTPOLCLR     (*(__IO uint16_t*)0x40010034U) /**< (GPIO0) IRQ Configuration Clear */
#define REG_GPIO0_INTSTATUSCLEAR (*(__IO uint16_t*)0x40010038U) /**< (GPIO0) Interrupt Status */
#define REG_GPIO0_PID4          (*(__I  uint8_t*)0x40010FD0U) /**< (GPIO0) Peripheral ID Register 4 */
#define REG_GPIO0_PID5          (*(__I  uint8_t*)0x40010FD4U) /**< (GPIO0) Peripheral ID Register 5 */
#define REG_GPIO0_PID6          (*(__I  uint8_t*)0x40010FD8U) /**< (GPIO0) Peripheral ID Register 6 */
#define REG_GPIO0_PID7          (*(__I  uint8_t*)0x40010FDCU) /**< (GPIO0) Peripheral ID Register 7 */
#define REG_GPIO0_PID0          (*(__I  uint8_t*)0x40010FE0U) /**< (GPIO0) Peripheral ID Register 0 */
#define REG_GPIO0_PID1          (*(__I  uint8_t*)0x40010FE4U) /**< (GPIO0) Peripheral ID Register 1 */
#define REG_GPIO0_PID2          (*(__I  uint8_t*)0x40010FE8U) /**< (GPIO0) Peripheral ID Register 2 */
#define REG_GPIO0_PID3          (*(__I  uint8_t*)0x40010FECU) /**< (GPIO0) Peripheral ID Register 3 */
#define REG_GPIO0_CID0          (*(__I  uint8_t*)0x40010FF0U) /**< (GPIO0) Component ID Register 0 */
#define REG_GPIO0_CID1          (*(__I  uint8_t*)0x40010FF4U) /**< (GPIO0) Component ID Register 1 */
#define REG_GPIO0_CID2          (*(__I  uint8_t*)0x40010FF8U) /**< (GPIO0) Component ID Register 2 */
#define REG_GPIO0_CID3          (*(__I  uint8_t*)0x40010FFCU) /**< (GPIO0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_GPIO0_INSTANCE_ */
