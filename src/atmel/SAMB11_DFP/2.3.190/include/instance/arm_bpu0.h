/**
 * \file
 *
 * \brief Instance description for ARM_BPU0
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
#ifndef _SAMB11_ARM_BPU0_INSTANCE_H_
#define _SAMB11_ARM_BPU0_INSTANCE_H_

/* ========== Register definition for ARM_BPU0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_ARM_BPU0_BP_CTRL    (0xE0002000) /**< (ARM_BPU0) Break Point Control Register */
#define REG_ARM_BPU0_BP_COMP0   (0xE0002008) /**< (ARM_BPU0) Break Point Compare Register 0 */
#define REG_ARM_BPU0_BP_COMP1   (0xE000200C) /**< (ARM_BPU0) Break Point Compare Register 1 */
#define REG_ARM_BPU0_BP_COMP2   (0xE0002010) /**< (ARM_BPU0) Break Point Compare Register 2 */
#define REG_ARM_BPU0_BP_COMP3   (0xE0002014) /**< (ARM_BPU0) Break Point Compare Register 3 */
#define REG_ARM_BPU0_BP_PID4    (0xE0002FD0) /**< (ARM_BPU0) Peripheral ID Register 4 */
#define REG_ARM_BPU0_BP_PID0    (0xE0002FE0) /**< (ARM_BPU0) Peripheral ID Register 0 */
#define REG_ARM_BPU0_BP_PID1    (0xE0002FE4) /**< (ARM_BPU0) Peripheral ID Register 1 */
#define REG_ARM_BPU0_BP_PID2    (0xE0002FE8) /**< (ARM_BPU0) Peripheral ID Register 2 */
#define REG_ARM_BPU0_BP_PID3    (0xE0002FEC) /**< (ARM_BPU0) Peripheral ID Register 3 */
#define REG_ARM_BPU0_BP_CID0    (0xE0002FF0) /**< (ARM_BPU0) Component ID Register 0 */
#define REG_ARM_BPU0_BP_CID1    (0xE0002FF4) /**< (ARM_BPU0) Component ID Register 1 */
#define REG_ARM_BPU0_BP_CID2    (0xE0002FF8) /**< (ARM_BPU0) Component ID Register 2 */
#define REG_ARM_BPU0_BP_CID3    (0xE0002FFC) /**< (ARM_BPU0) Component ID Register 3 */

#else

#define REG_ARM_BPU0_BP_CTRL    (*(__IO uint8_t*)0xE0002000U) /**< (ARM_BPU0) Break Point Control Register */
#define REG_ARM_BPU0_BP_COMP0   (*(__IO uint32_t*)0xE0002008U) /**< (ARM_BPU0) Break Point Compare Register 0 */
#define REG_ARM_BPU0_BP_COMP1   (*(__IO uint32_t*)0xE000200CU) /**< (ARM_BPU0) Break Point Compare Register 1 */
#define REG_ARM_BPU0_BP_COMP2   (*(__IO uint32_t*)0xE0002010U) /**< (ARM_BPU0) Break Point Compare Register 2 */
#define REG_ARM_BPU0_BP_COMP3   (*(__IO uint32_t*)0xE0002014U) /**< (ARM_BPU0) Break Point Compare Register 3 */
#define REG_ARM_BPU0_BP_PID4    (*(__I  uint8_t*)0xE0002FD0U) /**< (ARM_BPU0) Peripheral ID Register 4 */
#define REG_ARM_BPU0_BP_PID0    (*(__I  uint8_t*)0xE0002FE0U) /**< (ARM_BPU0) Peripheral ID Register 0 */
#define REG_ARM_BPU0_BP_PID1    (*(__I  uint8_t*)0xE0002FE4U) /**< (ARM_BPU0) Peripheral ID Register 1 */
#define REG_ARM_BPU0_BP_PID2    (*(__I  uint8_t*)0xE0002FE8U) /**< (ARM_BPU0) Peripheral ID Register 2 */
#define REG_ARM_BPU0_BP_PID3    (*(__I  uint8_t*)0xE0002FECU) /**< (ARM_BPU0) Peripheral ID Register 3 */
#define REG_ARM_BPU0_BP_CID0    (*(__I  uint8_t*)0xE0002FF0U) /**< (ARM_BPU0) Component ID Register 0 */
#define REG_ARM_BPU0_BP_CID1    (*(__I  uint8_t*)0xE0002FF4U) /**< (ARM_BPU0) Component ID Register 1 */
#define REG_ARM_BPU0_BP_CID2    (*(__I  uint8_t*)0xE0002FF8U) /**< (ARM_BPU0) Component ID Register 2 */
#define REG_ARM_BPU0_BP_CID3    (*(__I  uint8_t*)0xE0002FFCU) /**< (ARM_BPU0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_ARM_BPU0_INSTANCE_ */
