/**
 * \file
 *
 * \brief Instance description for SPI0
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
#ifndef _SAMB11_SPI0_INSTANCE_H_
#define _SAMB11_SPI0_INSTANCE_H_

/* ========== Register definition for SPI0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_SPI0_TRANSMIT_DATA  (0x40006000) /**< (SPI0) Writes one byte to SPI Transmit Data FIFO.  */
#define REG_SPI0_RECEIVE_DATA   (0x40006004) /**< (SPI0) Read one byte from SPI Receive Data FIFO.  */
#define REG_SPI0_TRANSMIT_STATUS (0x40006008) /**< (SPI0) Status of the SPI transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_SPI0_RECEIVE_STATUS (0x4000600C) /**< (SPI0) Status of the SPI receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_SPI0_CLOCK_SOURCE_SELECT (0x40006010) /**< (SPI0) Clock Source Select */
#define REG_SPI0_CLK_DIVIDER    (0x40006014) /**< (SPI0) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_SPI0_MODULE_ENABLE  (0x40006018) /**< (SPI0) SPI Enable */
#define REG_SPI0_MASTER_MODE    (0x4000601C) /**< (SPI0) SPI Master/Slave Mode. When clear, SPI is in Slave Mode.  */
#define REG_SPI0_FAULT_ENABLE   (0x40006020) /**< (SPI0) SPI Fault Detection Mode. If set, SPI bus contention will be detected, and the fault  bit in the rx_status register will be set, forcing the SPI Module into idle state.  When a fault is detected, the current SPI transaction is abandoned and the interface switches  to slave mode in the wait state.   */
#define REG_SPI0_CONFIGURATION  (0x40006024) /**< (SPI0) SPI Operation Configuration Register. This register should not be modified while SPI bus is active,  otherwise the SPI Module state shall not be guaranteed.  */
#define REG_SPI0_BUS_STATUS     (0x40006028) /**< (SPI0) Status of SPI bus.  */
#define REG_SPI0_TX_INTERRUPT_MASK (0x4000602C) /**< (SPI0) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_SPI0_RX_INTERRUPT_MASK (0x40006030) /**< (SPI0) Enable or Disable the generation of interrupts by the rx_status register.  */

#else

#define REG_SPI0_TRANSMIT_DATA  (*(__O  uint8_t*)0x40006000U) /**< (SPI0) Writes one byte to SPI Transmit Data FIFO.  */
#define REG_SPI0_RECEIVE_DATA   (*(__I  uint8_t*)0x40006004U) /**< (SPI0) Read one byte from SPI Receive Data FIFO.  */
#define REG_SPI0_TRANSMIT_STATUS (*(__I  uint8_t*)0x40006008U) /**< (SPI0) Status of the SPI transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_SPI0_RECEIVE_STATUS (*(__I  uint8_t*)0x4000600CU) /**< (SPI0) Status of the SPI receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_SPI0_CLOCK_SOURCE_SELECT (*(__IO uint8_t*)0x40006010U) /**< (SPI0) Clock Source Select */
#define REG_SPI0_CLK_DIVIDER    (*(__IO uint16_t*)0x40006014U) /**< (SPI0) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_SPI0_MODULE_ENABLE  (*(__IO uint8_t*)0x40006018U) /**< (SPI0) SPI Enable */
#define REG_SPI0_MASTER_MODE    (*(__IO uint8_t*)0x4000601CU) /**< (SPI0) SPI Master/Slave Mode. When clear, SPI is in Slave Mode.  */
#define REG_SPI0_FAULT_ENABLE   (*(__IO uint8_t*)0x40006020U) /**< (SPI0) SPI Fault Detection Mode. If set, SPI bus contention will be detected, and the fault  bit in the rx_status register will be set, forcing the SPI Module into idle state.  When a fault is detected, the current SPI transaction is abandoned and the interface switches  to slave mode in the wait state.   */
#define REG_SPI0_CONFIGURATION  (*(__IO uint8_t*)0x40006024U) /**< (SPI0) SPI Operation Configuration Register. This register should not be modified while SPI bus is active,  otherwise the SPI Module state shall not be guaranteed.  */
#define REG_SPI0_BUS_STATUS     (*(__I  uint8_t*)0x40006028U) /**< (SPI0) Status of SPI bus.  */
#define REG_SPI0_TX_INTERRUPT_MASK (*(__IO uint8_t*)0x4000602CU) /**< (SPI0) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_SPI0_RX_INTERRUPT_MASK (*(__IO uint8_t*)0x40006030U) /**< (SPI0) Enable or Disable the generation of interrupts by the rx_status register.  */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_SPI0_INSTANCE_ */
