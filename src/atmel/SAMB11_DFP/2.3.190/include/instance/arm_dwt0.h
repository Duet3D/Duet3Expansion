/**
 * \file
 *
 * \brief Instance description for ARM_DWT0
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
#ifndef _SAMB11_ARM_DWT0_INSTANCE_H_
#define _SAMB11_ARM_DWT0_INSTANCE_H_

/* ========== Register definition for ARM_DWT0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_ARM_DWT0_DWT_CTRL   (0xE0001000) /**< (ARM_DWT0) Control Register */
#define REG_ARM_DWT0_DWT_PCSR   (0xE000101C) /**< (ARM_DWT0) Program Counter Sample Register */
#define REG_ARM_DWT0_DWT_COMP0  (0xE0001020) /**< (ARM_DWT0) DWT Compare Register 0 */
#define REG_ARM_DWT0_DWT_MASK0  (0xE0001024) /**< (ARM_DWT0) DWT Mask Register 0 */
#define REG_ARM_DWT0_DWT_FUNCTION0 (0xE0001028) /**< (ARM_DWT0) DWT Function Register 0 */
#define REG_ARM_DWT0_DWT_COMP1  (0xE0001030) /**< (ARM_DWT0) DWT Compare Register 1 */
#define REG_ARM_DWT0_DWT_MASK1  (0xE0001034) /**< (ARM_DWT0) DWT Mask Register 1 */
#define REG_ARM_DWT0_DWT_FUNCTION1 (0xE0001038) /**< (ARM_DWT0) DWT Function Register 1 */
#define REG_ARM_DWT0_DWT_PID4   (0xE0001FD0) /**< (ARM_DWT0) Peripheral ID Register 4 */
#define REG_ARM_DWT0_DWT_PID0   (0xE0001FE0) /**< (ARM_DWT0) Peripheral ID Register 0 */
#define REG_ARM_DWT0_DWT_PID1   (0xE0001FE4) /**< (ARM_DWT0) Peripheral ID Register 1 */
#define REG_ARM_DWT0_DWT_PID2   (0xE0001FE8) /**< (ARM_DWT0) Peripheral ID Register 2 */
#define REG_ARM_DWT0_DWT_PID3   (0xE0001FEC) /**< (ARM_DWT0) Peripheral ID Register 3 */
#define REG_ARM_DWT0_DWT_CID0   (0xE0001FF0) /**< (ARM_DWT0) Component ID Register 0 */
#define REG_ARM_DWT0_DWT_CID1   (0xE0001FF4) /**< (ARM_DWT0) Component ID Register 1 */
#define REG_ARM_DWT0_DWT_CID2   (0xE0001FF8) /**< (ARM_DWT0) Component ID Register 2 */
#define REG_ARM_DWT0_DWT_CID3   (0xE0001FFC) /**< (ARM_DWT0) Component ID Register 3 */

#else

#define REG_ARM_DWT0_DWT_CTRL   (*(__I  uint32_t*)0xE0001000U) /**< (ARM_DWT0) Control Register */
#define REG_ARM_DWT0_DWT_PCSR   (*(__I  uint32_t*)0xE000101CU) /**< (ARM_DWT0) Program Counter Sample Register */
#define REG_ARM_DWT0_DWT_COMP0  (*(__IO uint32_t*)0xE0001020U) /**< (ARM_DWT0) DWT Compare Register 0 */
#define REG_ARM_DWT0_DWT_MASK0  (*(__IO uint8_t*)0xE0001024U) /**< (ARM_DWT0) DWT Mask Register 0 */
#define REG_ARM_DWT0_DWT_FUNCTION0 (*(__I  uint32_t*)0xE0001028U) /**< (ARM_DWT0) DWT Function Register 0 */
#define REG_ARM_DWT0_DWT_COMP1  (*(__IO uint32_t*)0xE0001030U) /**< (ARM_DWT0) DWT Compare Register 1 */
#define REG_ARM_DWT0_DWT_MASK1  (*(__IO uint8_t*)0xE0001034U) /**< (ARM_DWT0) DWT Mask Register 1 */
#define REG_ARM_DWT0_DWT_FUNCTION1 (*(__I  uint32_t*)0xE0001038U) /**< (ARM_DWT0) DWT Function Register 1 */
#define REG_ARM_DWT0_DWT_PID4   (*(__I  uint8_t*)0xE0001FD0U) /**< (ARM_DWT0) Peripheral ID Register 4 */
#define REG_ARM_DWT0_DWT_PID0   (*(__I  uint8_t*)0xE0001FE0U) /**< (ARM_DWT0) Peripheral ID Register 0 */
#define REG_ARM_DWT0_DWT_PID1   (*(__I  uint8_t*)0xE0001FE4U) /**< (ARM_DWT0) Peripheral ID Register 1 */
#define REG_ARM_DWT0_DWT_PID2   (*(__I  uint8_t*)0xE0001FE8U) /**< (ARM_DWT0) Peripheral ID Register 2 */
#define REG_ARM_DWT0_DWT_PID3   (*(__I  uint8_t*)0xE0001FECU) /**< (ARM_DWT0) Peripheral ID Register 3 */
#define REG_ARM_DWT0_DWT_CID0   (*(__I  uint8_t*)0xE0001FF0U) /**< (ARM_DWT0) Component ID Register 0 */
#define REG_ARM_DWT0_DWT_CID1   (*(__I  uint8_t*)0xE0001FF4U) /**< (ARM_DWT0) Component ID Register 1 */
#define REG_ARM_DWT0_DWT_CID2   (*(__I  uint8_t*)0xE0001FF8U) /**< (ARM_DWT0) Component ID Register 2 */
#define REG_ARM_DWT0_DWT_CID3   (*(__I  uint8_t*)0xE0001FFCU) /**< (ARM_DWT0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_ARM_DWT0_INSTANCE_ */
