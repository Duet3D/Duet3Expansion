/**
 * \file
 *
 * \brief Instance description for UART1
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
#ifndef _SAMB11_UART1_INSTANCE_H_
#define _SAMB11_UART1_INSTANCE_H_

/* ========== Register definition for UART1 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_UART1_TRANSMIT_DATA (0x40005000) /**< (UART1) Writes one byte to UART Transmit Data FIFO.  */
#define REG_UART1_TRANSMIT_STATUS (0x40005004) /**< (UART1) Status of the UART transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_UART1_TX_INTERRUPT_MASK (0x40005008) /**< (UART1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_UART1_RECEIVE_DATA  (0x40005010) /**< (UART1) Read one byte from UART Receive Data FIFO.  */
#define REG_UART1_RECEIVE_STATUS (0x40005014) /**< (UART1) Status of the UART receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_UART1_RX_INTERRUPT_MASK (0x40005018) /**< (UART1) Enable or Disable the generation of interrupts by the rx_status register.  */
#define REG_UART1_RECEIVE_TIMEOUT (0x4000501C) /**< (UART1) Timeout counter configuration.  */
#define REG_UART1_CONFIGURATION (0x40005020) /**< (UART1) UART Operation Configuration Register, for both Rx and Tx.  */
#define REG_UART1_BAUD_RATE     (0x40005024) /**< (UART1) Baud Rate Control Register. Bits 15:3 specify the integral division of the clock (divide by n),  and bit 2:0 specify the fractional division.  */
#define REG_UART1_CLOCK_SOURCE  (0x40005028) /**< (UART1) Selects Source of UART Clock  */

#else

#define REG_UART1_TRANSMIT_DATA (*(__O  uint8_t*)0x40005000U) /**< (UART1) Writes one byte to UART Transmit Data FIFO.  */
#define REG_UART1_TRANSMIT_STATUS (*(__I  uint8_t*)0x40005004U) /**< (UART1) Status of the UART transmitter. Each field can generate an interrupt if corresponding bit in  the Tx interrupt mask register is set.  */
#define REG_UART1_TX_INTERRUPT_MASK (*(__IO uint8_t*)0x40005008U) /**< (UART1) Enable or Disable the generation of interrupts by the tx_status register.  */
#define REG_UART1_RECEIVE_DATA  (*(__I  uint8_t*)0x40005010U) /**< (UART1) Read one byte from UART Receive Data FIFO.  */
#define REG_UART1_RECEIVE_STATUS (*(__I  uint8_t*)0x40005014U) /**< (UART1) Status of the UART receiver. Each field can generate an interrupt if corresponding bit in  the Rx interrupt mask register is set.  */
#define REG_UART1_RX_INTERRUPT_MASK (*(__IO uint8_t*)0x40005018U) /**< (UART1) Enable or Disable the generation of interrupts by the rx_status register.  */
#define REG_UART1_RECEIVE_TIMEOUT (*(__IO uint8_t*)0x4000501CU) /**< (UART1) Timeout counter configuration.  */
#define REG_UART1_CONFIGURATION (*(__IO uint8_t*)0x40005020U) /**< (UART1) UART Operation Configuration Register, for both Rx and Tx.  */
#define REG_UART1_BAUD_RATE     (*(__IO uint16_t*)0x40005024U) /**< (UART1) Baud Rate Control Register. Bits 15:3 specify the integral division of the clock (divide by n),  and bit 2:0 specify the fractional division.  */
#define REG_UART1_CLOCK_SOURCE  (*(__IO uint8_t*)0x40005028U) /**< (UART1) Selects Source of UART Clock  */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_UART1_INSTANCE_ */
