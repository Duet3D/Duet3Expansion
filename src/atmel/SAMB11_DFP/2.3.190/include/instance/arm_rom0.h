/**
 * \file
 *
 * \brief Instance description for ARM_ROM0
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
#ifndef _SAMB11_ARM_ROM0_INSTANCE_H_
#define _SAMB11_ARM_ROM0_INSTANCE_H_

/* ========== Register definition for ARM_ROM0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_ARM_ROM0_ROM_SCS    (0xE00FF000) /**< (ARM_ROM0) Points to the SCS at 0xE000E000 */
#define REG_ARM_ROM0_ROM_DWT    (0xE00FF010) /**< (ARM_ROM0) Points to the DWT at 0xE0001000 */
#define REG_ARM_ROM0_ROM_BPU    (0xE00FF020) /**< (ARM_ROM0) Points to the BPU at 0xE0002000 */
#define REG_ARM_ROM0_ROM_EOT    (0xE00FF030) /**< (ARM_ROM0) End of Table Marker */
#define REG_ARM_ROM0_ROM_CSMT   (0xE00FFFCC) /**< (ARM_ROM0) System Memory accessible through DAP */
#define REG_ARM_ROM0_ROM_PID4   (0xE00FFFD0) /**< (ARM_ROM0) Peripheral ID Register 4 */
#define REG_ARM_ROM0_ROM_PID0   (0xE00FFFE0) /**< (ARM_ROM0) Peripheral ID Register 0 */
#define REG_ARM_ROM0_ROM_PID1   (0xE00FFFE4) /**< (ARM_ROM0) Peripheral ID Register 1 */
#define REG_ARM_ROM0_ROM_PID2   (0xE00FFFE8) /**< (ARM_ROM0) Peripheral ID Register 2 */
#define REG_ARM_ROM0_ROM_PID3   (0xE00FFFEC) /**< (ARM_ROM0) Peripheral ID Register 3 */
#define REG_ARM_ROM0_ROM_CID0   (0xE00FFFF0) /**< (ARM_ROM0) Component ID Register 0 */
#define REG_ARM_ROM0_ROM_CID1   (0xE00FFFF4) /**< (ARM_ROM0) Component ID Register 1 */
#define REG_ARM_ROM0_ROM_CID2   (0xE00FFFF8) /**< (ARM_ROM0) Component ID Register 2 */
#define REG_ARM_ROM0_ROM_CID3   (0xE00FFFFC) /**< (ARM_ROM0) Component ID Register 3 */

#else

#define REG_ARM_ROM0_ROM_SCS    (*(__I  uint32_t*)0xE00FF000U) /**< (ARM_ROM0) Points to the SCS at 0xE000E000 */
#define REG_ARM_ROM0_ROM_DWT    (*(__I  uint32_t*)0xE00FF010U) /**< (ARM_ROM0) Points to the DWT at 0xE0001000 */
#define REG_ARM_ROM0_ROM_BPU    (*(__I  uint32_t*)0xE00FF020U) /**< (ARM_ROM0) Points to the BPU at 0xE0002000 */
#define REG_ARM_ROM0_ROM_EOT    (*(__I  uint32_t*)0xE00FF030U) /**< (ARM_ROM0) End of Table Marker */
#define REG_ARM_ROM0_ROM_CSMT   (*(__I  uint8_t*)0xE00FFFCCU) /**< (ARM_ROM0) System Memory accessible through DAP */
#define REG_ARM_ROM0_ROM_PID4   (*(__I  uint8_t*)0xE00FFFD0U) /**< (ARM_ROM0) Peripheral ID Register 4 */
#define REG_ARM_ROM0_ROM_PID0   (*(__I  uint8_t*)0xE00FFFE0U) /**< (ARM_ROM0) Peripheral ID Register 0 */
#define REG_ARM_ROM0_ROM_PID1   (*(__I  uint8_t*)0xE00FFFE4U) /**< (ARM_ROM0) Peripheral ID Register 1 */
#define REG_ARM_ROM0_ROM_PID2   (*(__I  uint8_t*)0xE00FFFE8U) /**< (ARM_ROM0) Peripheral ID Register 2 */
#define REG_ARM_ROM0_ROM_PID3   (*(__I  uint8_t*)0xE00FFFECU) /**< (ARM_ROM0) Peripheral ID Register 3 */
#define REG_ARM_ROM0_ROM_CID0   (*(__I  uint8_t*)0xE00FFFF0U) /**< (ARM_ROM0) Component ID Register 0 */
#define REG_ARM_ROM0_ROM_CID1   (*(__I  uint8_t*)0xE00FFFF4U) /**< (ARM_ROM0) Component ID Register 1 */
#define REG_ARM_ROM0_ROM_CID2   (*(__I  uint8_t*)0xE00FFFF8U) /**< (ARM_ROM0) Component ID Register 2 */
#define REG_ARM_ROM0_ROM_CID3   (*(__I  uint8_t*)0xE00FFFFCU) /**< (ARM_ROM0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_ARM_ROM0_INSTANCE_ */
