/**
 * \file
 *
 * \brief Instance description for PROV_DMA_CTRL0
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
#ifndef _SAMB11_PROV_DMA_CTRL0_INSTANCE_H_
#define _SAMB11_PROV_DMA_CTRL0_INSTANCE_H_

/* ========== Register definition for PROV_DMA_CTRL0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_PROV_DMA_CTRL0_CH0_CMD_REG0 (0x40002000) /**< (PROV_DMA_CTRL0) Channel 0 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG1 (0x40002004) /**< (PROV_DMA_CTRL0) Channel 0 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG2 (0x40002008) /**< (PROV_DMA_CTRL0) Channel 0 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG3 (0x4000200C) /**< (PROV_DMA_CTRL0) Channel 0 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG0 (0x40002010) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG1 (0x40002014) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG2 (0x40002018) /**< (PROV_DMA_CTRL0) Channel 0 Block Mode */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG4 (0x40002020) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH0_RESRICT_REG (0x4000202C) /**< (PROV_DMA_CTRL0) Channel 0 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH0_FIFO_FULLNESS_REG (0x40002038) /**< (PROV_DMA_CTRL0) Channel 0 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_ENABLE_REG (0x40002040) /**< (PROV_DMA_CTRL0) Channel 0 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_START_REG (0x40002044) /**< (PROV_DMA_CTRL0) Channel 0 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_ACTIVE_REG (0x40002048) /**< (PROV_DMA_CTRL0) Channel 0 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH0_COUNT_REG (0x40002050) /**< (PROV_DMA_CTRL0) Channel 0 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH0_INT_RAWSTAT_REG (0x400020A0) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH0_INT_CLEAR_REG (0x400020A4) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH0_INT_ENABLE_REG (0x400020A8) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH0_INT_STATUS_REG (0x400020AC) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG0 (0x40002100) /**< (PROV_DMA_CTRL0) Channel 1 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG1 (0x40002104) /**< (PROV_DMA_CTRL0) Channel 1 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG2 (0x40002108) /**< (PROV_DMA_CTRL0) Channel 1 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG3 (0x4000210C) /**< (PROV_DMA_CTRL0) Channel 1 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG0 (0x40002110) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG1 (0x40002114) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG2 (0x40002118) /**< (PROV_DMA_CTRL0) Channel 1 Block Mode */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG4 (0x40002120) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH1_RESRICT_REG (0x4000212C) /**< (PROV_DMA_CTRL0) Channel 1 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH1_FIFO_FULLNESS_REG (0x40002138) /**< (PROV_DMA_CTRL0) Channel 1 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_ENABLE_REG (0x40002140) /**< (PROV_DMA_CTRL0) Channel 1 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_START_REG (0x40002144) /**< (PROV_DMA_CTRL0) Channel 1 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_ACTIVE_REG (0x40002148) /**< (PROV_DMA_CTRL0) Channel 1 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH1_COUNT_REG (0x40002150) /**< (PROV_DMA_CTRL0) Channel 1 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH1_INT_RAWSTAT_REG (0x400021A0) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH1_INT_CLEAR_REG (0x400021A4) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH1_INT_ENABLE_REG (0x400021A8) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH1_INT_STATUS_REG (0x400021AC) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG0 (0x40002200) /**< (PROV_DMA_CTRL0) Channel 2 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG1 (0x40002204) /**< (PROV_DMA_CTRL0) Channel 2 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG2 (0x40002208) /**< (PROV_DMA_CTRL0) Channel 2 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG3 (0x4000220C) /**< (PROV_DMA_CTRL0) Channel 2 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG0 (0x40002210) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG1 (0x40002214) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG2 (0x40002218) /**< (PROV_DMA_CTRL0) Channel 2 Block Mode */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG4 (0x40002220) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH2_RESRICT_REG (0x4000222C) /**< (PROV_DMA_CTRL0) Channel 2 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH2_FIFO_FULLNESS_REG (0x40002238) /**< (PROV_DMA_CTRL0) Channel 2 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_ENABLE_REG (0x40002240) /**< (PROV_DMA_CTRL0) Channel 2 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_START_REG (0x40002244) /**< (PROV_DMA_CTRL0) Channel 2 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_ACTIVE_REG (0x40002248) /**< (PROV_DMA_CTRL0) Channel 2 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH2_COUNT_REG (0x40002250) /**< (PROV_DMA_CTRL0) Channel 2 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH2_INT_RAWSTAT_REG (0x400022A0) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH2_INT_CLEAR_REG (0x400022A4) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH2_INT_ENABLE_REG (0x400022A8) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH2_INT_STATUS_REG (0x400022AC) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG0 (0x40002300) /**< (PROV_DMA_CTRL0) Channel 3 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG1 (0x40002304) /**< (PROV_DMA_CTRL0) Channel 3 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG2 (0x40002308) /**< (PROV_DMA_CTRL0) Channel 3 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG3 (0x4000230C) /**< (PROV_DMA_CTRL0) Channel 3 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG0 (0x40002310) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG1 (0x40002314) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG2 (0x40002318) /**< (PROV_DMA_CTRL0) Channel 3 Block Mode */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG4 (0x40002320) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH3_RESRICT_REG (0x4000232C) /**< (PROV_DMA_CTRL0) Channel 3 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH3_FIFO_FULLNESS_REG (0x40002338) /**< (PROV_DMA_CTRL0) Channel 3 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_ENABLE_REG (0x40002340) /**< (PROV_DMA_CTRL0) Channel 3 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_START_REG (0x40002344) /**< (PROV_DMA_CTRL0) Channel 3 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_ACTIVE_REG (0x40002348) /**< (PROV_DMA_CTRL0) Channel 3 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH3_COUNT_REG (0x40002350) /**< (PROV_DMA_CTRL0) Channel 3 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH3_INT_RAWSTAT_REG (0x400023A0) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH3_INT_CLEAR_REG (0x400023A4) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH3_INT_ENABLE_REG (0x400023A8) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH3_INT_STATUS_REG (0x400023AC) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CORE_INT_STATUS (0x40002800) /**< (PROV_DMA_CTRL0) Indicates which channels caused interrupt */
#define REG_PROV_DMA_CTRL0_CORE_JOINT_MODE (0x40002830) /**< (PROV_DMA_CTRL0) If set, core works in joint mode */
#define REG_PROV_DMA_CTRL0_CORE_PRIORITY (0x40002838) /**< (PROV_DMA_CTRL0) Core Priority Channels */
#define REG_PROV_DMA_CTRL0_CORE_CLKDIV (0x40002840) /**< (PROV_DMA_CTRL0) Ratio between main clock and core clock */
#define REG_PROV_DMA_CTRL0_CORE_CH_START (0x40002848) /**< (PROV_DMA_CTRL0) Channel Start */
#define REG_PROV_DMA_CTRL0_PERIPH_RX_CTRL (0x40002850) /**< (PROV_DMA_CTRL0) Direct control of peripheral RX request */
#define REG_PROV_DMA_CTRL0_PERIPH_TX_CTRL (0x40002860) /**< (PROV_DMA_CTRL0) Direct control of peripheral TX request */
#define REG_PROV_DMA_CTRL0_CORE_IDLE (0x400028D0) /**< (PROV_DMA_CTRL0) Indicates all channels have stopped and transactions have completed */
#define REG_PROV_DMA_CTRL0_USER_DEF_STATUS (0x400028E0) /**< (PROV_DMA_CTRL0) User Defined Configurations */
#define REG_PROV_DMA_CTRL0_CORE_DEF_STATUS0 (0x400028F0) /**< (PROV_DMA_CTRL0) User Defined Core Configurations 0 */
#define REG_PROV_DMA_CTRL0_CORE_DEF_STATUS1 (0x400028F4) /**< (PROV_DMA_CTRL0) User Defined Core Configurations 1 */

#else

#define REG_PROV_DMA_CTRL0_CH0_CMD_REG0 (*(__IO uint32_t*)0x40002000U) /**< (PROV_DMA_CTRL0) Channel 0 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG1 (*(__IO uint32_t*)0x40002004U) /**< (PROV_DMA_CTRL0) Channel 0 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG2 (*(__IO uint16_t*)0x40002008U) /**< (PROV_DMA_CTRL0) Channel 0 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_CMD_REG3 (*(__IO uint32_t*)0x4000200CU) /**< (PROV_DMA_CTRL0) Channel 0 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG0 (*(__IO uint32_t*)0x40002010U) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG1 (*(__IO uint32_t*)0x40002014U) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG2 (*(__IO uint32_t*)0x40002018U) /**< (PROV_DMA_CTRL0) Channel 0 Block Mode */
#define REG_PROV_DMA_CTRL0_CH0_STATIC_REG4 (*(__IO uint32_t*)0x40002020U) /**< (PROV_DMA_CTRL0) Channel 0 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH0_RESRICT_REG (*(__I  uint16_t*)0x4000202CU) /**< (PROV_DMA_CTRL0) Channel 0 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH0_FIFO_FULLNESS_REG (*(__I  uint32_t*)0x40002038U) /**< (PROV_DMA_CTRL0) Channel 0 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_ENABLE_REG (*(__IO uint8_t*)0x40002040U) /**< (PROV_DMA_CTRL0) Channel 0 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_START_REG (*(__O  uint8_t*)0x40002044U) /**< (PROV_DMA_CTRL0) Channel 0 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH0_CH_ACTIVE_REG (*(__I  uint8_t*)0x40002048U) /**< (PROV_DMA_CTRL0) Channel 0 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH0_COUNT_REG (*(__I  uint32_t*)0x40002050U) /**< (PROV_DMA_CTRL0) Channel 0 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH0_INT_RAWSTAT_REG (*(__IO uint8_t*)0x400020A0U) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH0_INT_CLEAR_REG (*(__IO uint8_t*)0x400020A4U) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH0_INT_ENABLE_REG (*(__IO uint8_t*)0x400020A8U) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH0_INT_STATUS_REG (*(__IO uint8_t*)0x400020ACU) /**< (PROV_DMA_CTRL0) Channel 0 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG0 (*(__IO uint32_t*)0x40002100U) /**< (PROV_DMA_CTRL0) Channel 1 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG1 (*(__IO uint32_t*)0x40002104U) /**< (PROV_DMA_CTRL0) Channel 1 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG2 (*(__IO uint16_t*)0x40002108U) /**< (PROV_DMA_CTRL0) Channel 1 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_CMD_REG3 (*(__IO uint32_t*)0x4000210CU) /**< (PROV_DMA_CTRL0) Channel 1 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG0 (*(__IO uint32_t*)0x40002110U) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG1 (*(__IO uint32_t*)0x40002114U) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG2 (*(__IO uint32_t*)0x40002118U) /**< (PROV_DMA_CTRL0) Channel 1 Block Mode */
#define REG_PROV_DMA_CTRL0_CH1_STATIC_REG4 (*(__IO uint32_t*)0x40002120U) /**< (PROV_DMA_CTRL0) Channel 1 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH1_RESRICT_REG (*(__I  uint16_t*)0x4000212CU) /**< (PROV_DMA_CTRL0) Channel 1 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH1_FIFO_FULLNESS_REG (*(__I  uint32_t*)0x40002138U) /**< (PROV_DMA_CTRL0) Channel 1 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_ENABLE_REG (*(__IO uint8_t*)0x40002140U) /**< (PROV_DMA_CTRL0) Channel 1 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_START_REG (*(__O  uint8_t*)0x40002144U) /**< (PROV_DMA_CTRL0) Channel 1 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH1_CH_ACTIVE_REG (*(__I  uint8_t*)0x40002148U) /**< (PROV_DMA_CTRL0) Channel 1 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH1_COUNT_REG (*(__I  uint32_t*)0x40002150U) /**< (PROV_DMA_CTRL0) Channel 1 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH1_INT_RAWSTAT_REG (*(__IO uint8_t*)0x400021A0U) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH1_INT_CLEAR_REG (*(__IO uint8_t*)0x400021A4U) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH1_INT_ENABLE_REG (*(__IO uint8_t*)0x400021A8U) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH1_INT_STATUS_REG (*(__IO uint8_t*)0x400021ACU) /**< (PROV_DMA_CTRL0) Channel 1 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG0 (*(__IO uint32_t*)0x40002200U) /**< (PROV_DMA_CTRL0) Channel 2 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG1 (*(__IO uint32_t*)0x40002204U) /**< (PROV_DMA_CTRL0) Channel 2 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG2 (*(__IO uint16_t*)0x40002208U) /**< (PROV_DMA_CTRL0) Channel 2 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_CMD_REG3 (*(__IO uint32_t*)0x4000220CU) /**< (PROV_DMA_CTRL0) Channel 2 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG0 (*(__IO uint32_t*)0x40002210U) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG1 (*(__IO uint32_t*)0x40002214U) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG2 (*(__IO uint32_t*)0x40002218U) /**< (PROV_DMA_CTRL0) Channel 2 Block Mode */
#define REG_PROV_DMA_CTRL0_CH2_STATIC_REG4 (*(__IO uint32_t*)0x40002220U) /**< (PROV_DMA_CTRL0) Channel 2 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH2_RESRICT_REG (*(__I  uint16_t*)0x4000222CU) /**< (PROV_DMA_CTRL0) Channel 2 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH2_FIFO_FULLNESS_REG (*(__I  uint32_t*)0x40002238U) /**< (PROV_DMA_CTRL0) Channel 2 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_ENABLE_REG (*(__IO uint8_t*)0x40002240U) /**< (PROV_DMA_CTRL0) Channel 2 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_START_REG (*(__O  uint8_t*)0x40002244U) /**< (PROV_DMA_CTRL0) Channel 2 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH2_CH_ACTIVE_REG (*(__I  uint8_t*)0x40002248U) /**< (PROV_DMA_CTRL0) Channel 2 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH2_COUNT_REG (*(__I  uint32_t*)0x40002250U) /**< (PROV_DMA_CTRL0) Channel 2 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH2_INT_RAWSTAT_REG (*(__IO uint8_t*)0x400022A0U) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH2_INT_CLEAR_REG (*(__IO uint8_t*)0x400022A4U) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH2_INT_ENABLE_REG (*(__IO uint8_t*)0x400022A8U) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH2_INT_STATUS_REG (*(__IO uint8_t*)0x400022ACU) /**< (PROV_DMA_CTRL0) Channel 2 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG0 (*(__IO uint32_t*)0x40002300U) /**< (PROV_DMA_CTRL0) Channel 3 First Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG1 (*(__IO uint32_t*)0x40002304U) /**< (PROV_DMA_CTRL0) Channel 3 Second Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG2 (*(__IO uint16_t*)0x40002308U) /**< (PROV_DMA_CTRL0) Channel 3 Third Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_CMD_REG3 (*(__IO uint32_t*)0x4000230CU) /**< (PROV_DMA_CTRL0) Channel 3 Fourth Line Channel Command */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG0 (*(__IO uint32_t*)0x40002310U) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Read */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG1 (*(__IO uint32_t*)0x40002314U) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Write */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG2 (*(__IO uint32_t*)0x40002318U) /**< (PROV_DMA_CTRL0) Channel 3 Block Mode */
#define REG_PROV_DMA_CTRL0_CH3_STATIC_REG4 (*(__IO uint32_t*)0x40002320U) /**< (PROV_DMA_CTRL0) Channel 3 Static Configuration Peripheral */
#define REG_PROV_DMA_CTRL0_CH3_RESRICT_REG (*(__I  uint16_t*)0x4000232CU) /**< (PROV_DMA_CTRL0) Channel 3 Restrictions Status Register */
#define REG_PROV_DMA_CTRL0_CH3_FIFO_FULLNESS_REG (*(__I  uint32_t*)0x40002338U) /**< (PROV_DMA_CTRL0) Channel 3 FIFO Fullness Status Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_ENABLE_REG (*(__IO uint8_t*)0x40002340U) /**< (PROV_DMA_CTRL0) Channel 3 Channel Enable Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_START_REG (*(__O  uint8_t*)0x40002344U) /**< (PROV_DMA_CTRL0) Channel 3 Channel Start Register */
#define REG_PROV_DMA_CTRL0_CH3_CH_ACTIVE_REG (*(__I  uint8_t*)0x40002348U) /**< (PROV_DMA_CTRL0) Channel 3 Channel Active Status Register */
#define REG_PROV_DMA_CTRL0_CH3_COUNT_REG (*(__I  uint32_t*)0x40002350U) /**< (PROV_DMA_CTRL0) Channel 3 Buffer Counter Status Register */
#define REG_PROV_DMA_CTRL0_CH3_INT_RAWSTAT_REG (*(__IO uint8_t*)0x400023A0U) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Raw Status (Write 1 to any field to issue interrupt) */
#define REG_PROV_DMA_CTRL0_CH3_INT_CLEAR_REG (*(__IO uint8_t*)0x400023A4U) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Clear (Write 1 to clear) */
#define REG_PROV_DMA_CTRL0_CH3_INT_ENABLE_REG (*(__IO uint8_t*)0x400023A8U) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Enable */
#define REG_PROV_DMA_CTRL0_CH3_INT_STATUS_REG (*(__IO uint8_t*)0x400023ACU) /**< (PROV_DMA_CTRL0) Channel 3 Interrupt Status */
#define REG_PROV_DMA_CTRL0_CORE_INT_STATUS (*(__I  uint8_t*)0x40002800U) /**< (PROV_DMA_CTRL0) Indicates which channels caused interrupt */
#define REG_PROV_DMA_CTRL0_CORE_JOINT_MODE (*(__IO uint8_t*)0x40002830U) /**< (PROV_DMA_CTRL0) If set, core works in joint mode */
#define REG_PROV_DMA_CTRL0_CORE_PRIORITY (*(__IO uint16_t*)0x40002838U) /**< (PROV_DMA_CTRL0) Core Priority Channels */
#define REG_PROV_DMA_CTRL0_CORE_CLKDIV (*(__IO uint8_t*)0x40002840U) /**< (PROV_DMA_CTRL0) Ratio between main clock and core clock */
#define REG_PROV_DMA_CTRL0_CORE_CH_START (*(__O  uint8_t*)0x40002848U) /**< (PROV_DMA_CTRL0) Channel Start */
#define REG_PROV_DMA_CTRL0_PERIPH_RX_CTRL (*(__IO uint32_t*)0x40002850U) /**< (PROV_DMA_CTRL0) Direct control of peripheral RX request */
#define REG_PROV_DMA_CTRL0_PERIPH_TX_CTRL (*(__IO uint32_t*)0x40002860U) /**< (PROV_DMA_CTRL0) Direct control of peripheral TX request */
#define REG_PROV_DMA_CTRL0_CORE_IDLE (*(__I  uint8_t*)0x400028D0U) /**< (PROV_DMA_CTRL0) Indicates all channels have stopped and transactions have completed */
#define REG_PROV_DMA_CTRL0_USER_DEF_STATUS (*(__I  uint32_t*)0x400028E0U) /**< (PROV_DMA_CTRL0) User Defined Configurations */
#define REG_PROV_DMA_CTRL0_CORE_DEF_STATUS0 (*(__I  uint32_t*)0x400028F0U) /**< (PROV_DMA_CTRL0) User Defined Core Configurations 0 */
#define REG_PROV_DMA_CTRL0_CORE_DEF_STATUS1 (*(__I  uint16_t*)0x400028F4U) /**< (PROV_DMA_CTRL0) User Defined Core Configurations 1 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_PROV_DMA_CTRL0_INSTANCE_ */
