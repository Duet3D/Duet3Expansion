/**
 * \file
 *
 * \brief Instance description for I2C1
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
#ifndef _SAMB11_I2C1_INSTANCE_H_
#define _SAMB11_I2C1_INSTANCE_H_

/* ========== Register definition for I2C1 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_I2C1_TRANSMIT_DATA  (0x40003400) /**< (I2C1) Writes one byte to I2C Transmit Data FIFO.  */
#define REG_I2C1_RECEIVE_DATA   (0x40003404) /**< (I2C1) Read one byte from I2C Receive Data FIFO.  */
#define REG_I2C1_TRANSMIT_STATUS (0x40003408) /**< (I2C1) Status of the I2C transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_I2C1_RECEIVE_STATUS (0x4000340C) /**< (I2C1) Status of the I2C receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_I2C1_CLOCK_SOURCE_SELECT (0x40003410) /**< (I2C1) Clock Source Select */
#define REG_I2C1_MODULE_ENABLE  (0x40003414) /**< (I2C1) I2C Enable */
#define REG_I2C1_CLK_DIVIDER    (0x40003418) /**< (I2C1) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_I2C1_MASTER_MODE    (0x4000341C) /**< (I2C1) Selects I2C Master or Slave Mode.  */
#define REG_I2C1_ONBUS          (0x40003420) /**< (I2C1) Enable/Disable transactions when in Master Mode.  */
#define REG_I2C1_SLAVE_ADDRESS  (0x40003424) /**< (I2C1) Configures the I2C slave Address.  */
#define REG_I2C1_STATUS         (0x40003428) /**< (I2C1) Status of I2C Module.  */
#define REG_I2C1_TX_INTERRUPT_MASK (0x4000342C) /**< (I2C1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_I2C1_RX_INTERRUPT_MASK (0x40003430) /**< (I2C1) Enable or Disable the generation of interrupts by the rx_status register.  */
#define REG_I2C1_FLUSH          (0x40003434) /**< (I2C1) Writing to this address  flushes the contents of both the Tx and Rx FIFOs. The value written has no effect. Flushing the Tx FIFO will abort ongoing transactions when the current byte has been transmitted.  */

#else

#define REG_I2C1_TRANSMIT_DATA  (*(__O  uint16_t*)0x40003400U) /**< (I2C1) Writes one byte to I2C Transmit Data FIFO.  */
#define REG_I2C1_RECEIVE_DATA   (*(__I  uint8_t*)0x40003404U) /**< (I2C1) Read one byte from I2C Receive Data FIFO.  */
#define REG_I2C1_TRANSMIT_STATUS (*(__I  uint8_t*)0x40003408U) /**< (I2C1) Status of the I2C transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_I2C1_RECEIVE_STATUS (*(__I  uint8_t*)0x4000340CU) /**< (I2C1) Status of the I2C receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_I2C1_CLOCK_SOURCE_SELECT (*(__IO uint8_t*)0x40003410U) /**< (I2C1) Clock Source Select */
#define REG_I2C1_MODULE_ENABLE  (*(__IO uint8_t*)0x40003414U) /**< (I2C1) I2C Enable */
#define REG_I2C1_CLK_DIVIDER    (*(__IO uint16_t*)0x40003418U) /**< (I2C1) Register sets the divide ratio used to generate the sck clock from the module's input clock.  */
#define REG_I2C1_MASTER_MODE    (*(__IO uint8_t*)0x4000341CU) /**< (I2C1) Selects I2C Master or Slave Mode.  */
#define REG_I2C1_ONBUS          (*(__IO uint8_t*)0x40003420U) /**< (I2C1) Enable/Disable transactions when in Master Mode.  */
#define REG_I2C1_SLAVE_ADDRESS  (*(__IO uint8_t*)0x40003424U) /**< (I2C1) Configures the I2C slave Address.  */
#define REG_I2C1_STATUS         (*(__I  uint8_t*)0x40003428U) /**< (I2C1) Status of I2C Module.  */
#define REG_I2C1_TX_INTERRUPT_MASK (*(__IO uint8_t*)0x4000342CU) /**< (I2C1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_I2C1_RX_INTERRUPT_MASK (*(__IO uint8_t*)0x40003430U) /**< (I2C1) Enable or Disable the generation of interrupts by the rx_status register.  */
#define REG_I2C1_FLUSH          (*(__O  uint8_t*)0x40003434U) /**< (I2C1) Writing to this address  flushes the contents of both the Tx and Rx FIFOs. The value written has no effect. Flushing the Tx FIFO will abort ongoing transactions when the current byte has been transmitted.  */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_I2C1_INSTANCE_ */
