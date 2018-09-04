/**
 * \file
 *
 * \brief Instance description for SPI_FLASH0
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
#ifndef _SAMB11_SPI_FLASH0_INSTANCE_H_
#define _SAMB11_SPI_FLASH0_INSTANCE_H_

/* ========== Register definition for SPI_FLASH0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_SPI_FLASH0_MODE_CTRL (0x40012000) /**< (SPI_FLASH0) SPI Flash Mode Control */
#define REG_SPI_FLASH0_TRANSACTION_CTRL (0x40012004) /**< (SPI_FLASH0) Transaction Control (Cleared after each transaction completes) */
#define REG_SPI_FLASH0_READ_CTRL (0x40012008) /**< (SPI_FLASH0) Read Control (Cleared after each transaction completes) */
#define REG_SPI_FLASH0_CMD_BUFFER0 (0x4001200C) /**< (SPI_FLASH0) Command Buffer 0 (Bytes 3 - 0) */
#define REG_SPI_FLASH0_CMD_BUFFER1 (0x40012010) /**< (SPI_FLASH0) Command Buffer 1 (Bytes 7 - 4) */
#define REG_SPI_FLASH0_DIRECTION (0x40012014) /**< (SPI_FLASH0) Read/Write bit for Bytes 7 - 0 */
#define REG_SPI_FLASH0_IRQ_STATUS (0x40012018) /**< (SPI_FLASH0) IRQ Status (Write 0 to bit to clear, Read clears interupts) */
#define REG_SPI_FLASH0_DMA_START_ADDRESS (0x4001201C) /**< (SPI_FLASH0) DMA Starting Address */
#define REG_SPI_FLASH0_CONFIG   (0x40012020) /**< (SPI_FLASH0) SPI Flash Configuration */
#define REG_SPI_FLASH0_TX_CONTROL (0x40012024) /**< (SPI_FLASH0) TX Control */
#define REG_SPI_FLASH0_STATUS   (0x40012028) /**< (SPI_FLASH0) Misc Status */

#else

#define REG_SPI_FLASH0_MODE_CTRL (*(__IO uint8_t*)0x40012000U) /**< (SPI_FLASH0) SPI Flash Mode Control */
#define REG_SPI_FLASH0_TRANSACTION_CTRL (*(__IO uint32_t*)0x40012004U) /**< (SPI_FLASH0) Transaction Control (Cleared after each transaction completes) */
#define REG_SPI_FLASH0_READ_CTRL (*(__IO uint32_t*)0x40012008U) /**< (SPI_FLASH0) Read Control (Cleared after each transaction completes) */
#define REG_SPI_FLASH0_CMD_BUFFER0 (*(__IO uint32_t*)0x4001200CU) /**< (SPI_FLASH0) Command Buffer 0 (Bytes 3 - 0) */
#define REG_SPI_FLASH0_CMD_BUFFER1 (*(__IO uint32_t*)0x40012010U) /**< (SPI_FLASH0) Command Buffer 1 (Bytes 7 - 4) */
#define REG_SPI_FLASH0_DIRECTION (*(__IO uint8_t*)0x40012014U) /**< (SPI_FLASH0) Read/Write bit for Bytes 7 - 0 */
#define REG_SPI_FLASH0_IRQ_STATUS (*(__O  uint8_t*)0x40012018U) /**< (SPI_FLASH0) IRQ Status (Write 0 to bit to clear, Read clears interupts) */
#define REG_SPI_FLASH0_DMA_START_ADDRESS (*(__IO uint32_t*)0x4001201CU) /**< (SPI_FLASH0) DMA Starting Address */
#define REG_SPI_FLASH0_CONFIG   (*(__IO uint16_t*)0x40012020U) /**< (SPI_FLASH0) SPI Flash Configuration */
#define REG_SPI_FLASH0_TX_CONTROL (*(__IO uint16_t*)0x40012024U) /**< (SPI_FLASH0) TX Control */
#define REG_SPI_FLASH0_STATUS   (*(__I  uint16_t*)0x40012028U) /**< (SPI_FLASH0) Misc Status */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

/* ========== Instance Parameter definitions for SPI_FLASH0 peripheral ========== */
#define SPI_FLASH0_FLASH_SIZE                    131072    
#define SPI_FLASH0_ADDRESS_SPACE_REF             extflash  
#define SPI_FLASH0_PAGES                         2048      
#define SPI_FLASH0_PAGE_SIZE                     64        

#endif /* _SAMB11_SPI_FLASH0_INSTANCE_ */
