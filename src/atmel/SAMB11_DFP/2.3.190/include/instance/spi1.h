/**
 * \file
 *
 * \brief Instance description for SPI1
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
#ifndef _SAMB11_SPI1_INSTANCE_H_
#define _SAMB11_SPI1_INSTANCE_H_

/* ========== Register definition for SPI1 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_SPI1_TRANSMIT_DATA  (0x40007000) /**< (SPI1) Writes one byte to SPI Transmit Data FIFO.  */
#define REG_SPI1_RECEIVE_DATA   (0x40007004) /**< (SPI1) Read one byte from SPI Receive Data FIFO.  */
#define REG_SPI1_TRANSMIT_STATUS (0x40007008) /**< (SPI1) Status of the SPI transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_SPI1_RECEIVE_STATUS (0x4000700C) /**< (SPI1) Status of the SPI receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_SPI1_CLOCK_SOURCE_SELECT (0x40007010) /**< (SPI1) Clock Source Select */
#define REG_SPI1_CLK_DIVIDER    (0x40007014) /**< (SPI1) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_SPI1_MODULE_ENABLE  (0x40007018) /**< (SPI1) SPI Enable */
#define REG_SPI1_MASTER_MODE    (0x4000701C) /**< (SPI1) SPI Master/Slave Mode. When clear, SPI is in Slave Mode.  */
#define REG_SPI1_FAULT_ENABLE   (0x40007020) /**< (SPI1) SPI Fault Detection Mode. If set, SPI bus contention will be detected, and the fault  bit in the rx_status register will be set, forcing the SPI Module into idle state.  When a fault is detected, the current SPI transaction is abandoned and the interface switches  to slave mode in the wait state.   */
#define REG_SPI1_CONFIGURATION  (0x40007024) /**< (SPI1) SPI Operation Configuration Register. This register should not be modified while SPI bus is active,  otherwise the SPI Module state shall not be guaranteed.  */
#define REG_SPI1_BUS_STATUS     (0x40007028) /**< (SPI1) Status of SPI bus.  */
#define REG_SPI1_TX_INTERRUPT_MASK (0x4000702C) /**< (SPI1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_SPI1_RX_INTERRUPT_MASK (0x40007030) /**< (SPI1) Enable or Disable the generation of interrupts by the rx_status register.  */

#else

#define REG_SPI1_TRANSMIT_DATA  (*(__O  uint8_t*)0x40007000U) /**< (SPI1) Writes one byte to SPI Transmit Data FIFO.  */
#define REG_SPI1_RECEIVE_DATA   (*(__I  uint8_t*)0x40007004U) /**< (SPI1) Read one byte from SPI Receive Data FIFO.  */
#define REG_SPI1_TRANSMIT_STATUS (*(__I  uint8_t*)0x40007008U) /**< (SPI1) Status of the SPI transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_SPI1_RECEIVE_STATUS (*(__I  uint8_t*)0x4000700CU) /**< (SPI1) Status of the SPI receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_SPI1_CLOCK_SOURCE_SELECT (*(__IO uint8_t*)0x40007010U) /**< (SPI1) Clock Source Select */
#define REG_SPI1_CLK_DIVIDER    (*(__IO uint16_t*)0x40007014U) /**< (SPI1) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_SPI1_MODULE_ENABLE  (*(__IO uint8_t*)0x40007018U) /**< (SPI1) SPI Enable */
#define REG_SPI1_MASTER_MODE    (*(__IO uint8_t*)0x4000701CU) /**< (SPI1) SPI Master/Slave Mode. When clear, SPI is in Slave Mode.  */
#define REG_SPI1_FAULT_ENABLE   (*(__IO uint8_t*)0x40007020U) /**< (SPI1) SPI Fault Detection Mode. If set, SPI bus contention will be detected, and the fault  bit in the rx_status register will be set, forcing the SPI Module into idle state.  When a fault is detected, the current SPI transaction is abandoned and the interface switches  to slave mode in the wait state.   */
#define REG_SPI1_CONFIGURATION  (*(__IO uint8_t*)0x40007024U) /**< (SPI1) SPI Operation Configuration Register. This register should not be modified while SPI bus is active,  otherwise the SPI Module state shall not be guaranteed.  */
#define REG_SPI1_BUS_STATUS     (*(__I  uint8_t*)0x40007028U) /**< (SPI1) Status of SPI bus.  */
#define REG_SPI1_TX_INTERRUPT_MASK (*(__IO uint8_t*)0x4000702CU) /**< (SPI1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_SPI1_RX_INTERRUPT_MASK (*(__IO uint8_t*)0x40007030U) /**< (SPI1) Enable or Disable the generation of interrupts by the rx_status register.  */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_SPI1_INSTANCE_ */
