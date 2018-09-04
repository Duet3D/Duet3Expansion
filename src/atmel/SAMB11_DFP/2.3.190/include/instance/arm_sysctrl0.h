/**
 * \file
 *
 * \brief Instance description for ARM_SYSCTRL0
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
#ifndef _SAMB11_ARM_SYSCTRL0_INSTANCE_H_
#define _SAMB11_ARM_SYSCTRL0_INSTANCE_H_

/* ========== Register definition for ARM_SYSCTRL0 peripheral ========== */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))

#define REG_ARM_SYSCTRL0_ACTLR  (0xE000E008) /**< (ARM_SYSCTRL0) Auxiliary Control Register (not implemented) */
#define REG_ARM_SYSCTRL0_SYST_CSR (0xE000E010) /**< (ARM_SYSCTRL0) SysTick Control and Status Register */
#define REG_ARM_SYSCTRL0_SYST_RVR (0xE000E014) /**< (ARM_SYSCTRL0) SysTick Reload Value Register */
#define REG_ARM_SYSCTRL0_SYST_CVR (0xE000E018) /**< (ARM_SYSCTRL0) SysTick Current Value Register (Any Write Clears to 0) */
#define REG_ARM_SYSCTRL0_SYST_CALIB (0xE000E01C) /**< (ARM_SYSCTRL0) SysTick Calibration Value Register */
#define REG_ARM_SYSCTRL0_NVIC_ISER (0xE000E100) /**< (ARM_SYSCTRL0) Interrupt Set-Enable Register */
#define REG_ARM_SYSCTRL0_NVIC_ICER (0xE000E180) /**< (ARM_SYSCTRL0) Interrupt Clear Enable Register */
#define REG_ARM_SYSCTRL0_NVIC_ISPR (0xE000E200) /**< (ARM_SYSCTRL0) Interrupt Set-Pending Register */
#define REG_ARM_SYSCTRL0_NVIC_ICPR (0xE000E280) /**< (ARM_SYSCTRL0) Interrupt Clear-Pending Register */
#define REG_ARM_SYSCTRL0_NVIC_IPR0 (0xE000E400) /**< (ARM_SYSCTRL0) Interrupt Priority Register 0 */
#define REG_ARM_SYSCTRL0_NVIC_IPR1 (0xE000E404) /**< (ARM_SYSCTRL0) Interrupt Priority Register 1 */
#define REG_ARM_SYSCTRL0_NVIC_IPR2 (0xE000E408) /**< (ARM_SYSCTRL0) Interrupt Priority Register 2 */
#define REG_ARM_SYSCTRL0_NVIC_IPR3 (0xE000E40C) /**< (ARM_SYSCTRL0) Interrupt Priority Register 3 */
#define REG_ARM_SYSCTRL0_NVIC_IPR4 (0xE000E410) /**< (ARM_SYSCTRL0) Interrupt Priority Register 4 */
#define REG_ARM_SYSCTRL0_NVIC_IPR5 (0xE000E414) /**< (ARM_SYSCTRL0) Interrupt Priority Register 5 */
#define REG_ARM_SYSCTRL0_NVIC_IPR6 (0xE000E418) /**< (ARM_SYSCTRL0) Interrupt Priority Register 6 */
#define REG_ARM_SYSCTRL0_NVIC_IPR7 (0xE000E41C) /**< (ARM_SYSCTRL0) Interrupt Priority Register 7 */
#define REG_ARM_SYSCTRL0_CPUID  (0xE000ED00) /**< (ARM_SYSCTRL0) CPU Identification Register */
#define REG_ARM_SYSCTRL0_ICSR   (0xE000ED04) /**< (ARM_SYSCTRL0) Interrupt Control State Register */
#define REG_ARM_SYSCTRL0_AIRCR  (0xE000ED0C) /**< (ARM_SYSCTRL0) Application Interrupt and Reset Control Register */
#define REG_ARM_SYSCTRL0_SCR    (0xE000ED10) /**< (ARM_SYSCTRL0) System Control Register */
#define REG_ARM_SYSCTRL0_CCR    (0xE000ED14) /**< (ARM_SYSCTRL0) Configuration and Control Register */
#define REG_ARM_SYSCTRL0_SHPR2  (0xE000ED1C) /**< (ARM_SYSCTRL0) System Handler Priority Register 2 */
#define REG_ARM_SYSCTRL0_SHPR3  (0xE000ED20) /**< (ARM_SYSCTRL0) System Handler Priority Register 3 */
#define REG_ARM_SYSCTRL0_SHCSR  (0xE000ED24) /**< (ARM_SYSCTRL0) System Handler Control and State Register */
#define REG_ARM_SYSCTRL0_DFSR   (0xE000ED30) /**< (ARM_SYSCTRL0) Debug Fault Status Register */
#define REG_ARM_SYSCTRL0_DHCSR  (0xE000EDF0) /**< (ARM_SYSCTRL0) Debug Halting Control and Status Register */
#define REG_ARM_SYSCTRL0_DCRSR  (0xE000EDF4) /**< (ARM_SYSCTRL0) Debug Core Register Selector Register */
#define REG_ARM_SYSCTRL0_DCRDR  (0xE000EDF8) /**< (ARM_SYSCTRL0) Debug Core Register Data Register */
#define REG_ARM_SYSCTRL0_DEMCR  (0xE000EDFC) /**< (ARM_SYSCTRL0) Debug Exception and Monitor Control Register */
#define REG_ARM_SYSCTRL0_SCS_PID4 (0xE000EFD0) /**< (ARM_SYSCTRL0) Peripheral ID Register 4 */
#define REG_ARM_SYSCTRL0_SCS_PID0 (0xE000EFE0) /**< (ARM_SYSCTRL0) Peripheral ID Register 0 */
#define REG_ARM_SYSCTRL0_SCS_PID1 (0xE000EFE4) /**< (ARM_SYSCTRL0) Peripheral ID Register 1 */
#define REG_ARM_SYSCTRL0_SCS_PID2 (0xE000EFE8) /**< (ARM_SYSCTRL0) Peripheral ID Register 2 */
#define REG_ARM_SYSCTRL0_SCS_PID3 (0xE000EFEC) /**< (ARM_SYSCTRL0) Peripheral ID Register 3 */
#define REG_ARM_SYSCTRL0_SCS_CID0 (0xE000EFF0) /**< (ARM_SYSCTRL0) Component ID Register 0 */
#define REG_ARM_SYSCTRL0_SCS_CID1 (0xE000EFF4) /**< (ARM_SYSCTRL0) Component ID Register 1 */
#define REG_ARM_SYSCTRL0_SCS_CID2 (0xE000EFF8) /**< (ARM_SYSCTRL0) Component ID Register 2 */
#define REG_ARM_SYSCTRL0_SCS_CID3 (0xE000EFFC) /**< (ARM_SYSCTRL0) Component ID Register 3 */

#else

#define REG_ARM_SYSCTRL0_ACTLR  (*(__I  uint32_t*)0xE000E008U) /**< (ARM_SYSCTRL0) Auxiliary Control Register (not implemented) */
#define REG_ARM_SYSCTRL0_SYST_CSR (*(__IO uint32_t*)0xE000E010U) /**< (ARM_SYSCTRL0) SysTick Control and Status Register */
#define REG_ARM_SYSCTRL0_SYST_RVR (*(__IO uint32_t*)0xE000E014U) /**< (ARM_SYSCTRL0) SysTick Reload Value Register */
#define REG_ARM_SYSCTRL0_SYST_CVR (*(__IO uint32_t*)0xE000E018U) /**< (ARM_SYSCTRL0) SysTick Current Value Register (Any Write Clears to 0) */
#define REG_ARM_SYSCTRL0_SYST_CALIB (*(__I  uint32_t*)0xE000E01CU) /**< (ARM_SYSCTRL0) SysTick Calibration Value Register */
#define REG_ARM_SYSCTRL0_NVIC_ISER (*(__IO uint32_t*)0xE000E100U) /**< (ARM_SYSCTRL0) Interrupt Set-Enable Register */
#define REG_ARM_SYSCTRL0_NVIC_ICER (*(__IO uint32_t*)0xE000E180U) /**< (ARM_SYSCTRL0) Interrupt Clear Enable Register */
#define REG_ARM_SYSCTRL0_NVIC_ISPR (*(__IO uint32_t*)0xE000E200U) /**< (ARM_SYSCTRL0) Interrupt Set-Pending Register */
#define REG_ARM_SYSCTRL0_NVIC_ICPR (*(__IO uint32_t*)0xE000E280U) /**< (ARM_SYSCTRL0) Interrupt Clear-Pending Register */
#define REG_ARM_SYSCTRL0_NVIC_IPR0 (*(__IO uint32_t*)0xE000E400U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 0 */
#define REG_ARM_SYSCTRL0_NVIC_IPR1 (*(__IO uint32_t*)0xE000E404U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 1 */
#define REG_ARM_SYSCTRL0_NVIC_IPR2 (*(__IO uint32_t*)0xE000E408U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 2 */
#define REG_ARM_SYSCTRL0_NVIC_IPR3 (*(__IO uint32_t*)0xE000E40CU) /**< (ARM_SYSCTRL0) Interrupt Priority Register 3 */
#define REG_ARM_SYSCTRL0_NVIC_IPR4 (*(__IO uint32_t*)0xE000E410U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 4 */
#define REG_ARM_SYSCTRL0_NVIC_IPR5 (*(__IO uint32_t*)0xE000E414U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 5 */
#define REG_ARM_SYSCTRL0_NVIC_IPR6 (*(__IO uint32_t*)0xE000E418U) /**< (ARM_SYSCTRL0) Interrupt Priority Register 6 */
#define REG_ARM_SYSCTRL0_NVIC_IPR7 (*(__IO uint32_t*)0xE000E41CU) /**< (ARM_SYSCTRL0) Interrupt Priority Register 7 */
#define REG_ARM_SYSCTRL0_CPUID  (*(__I  uint32_t*)0xE000ED00U) /**< (ARM_SYSCTRL0) CPU Identification Register */
#define REG_ARM_SYSCTRL0_ICSR   (*(__I  uint32_t*)0xE000ED04U) /**< (ARM_SYSCTRL0) Interrupt Control State Register */
#define REG_ARM_SYSCTRL0_AIRCR  (*(__I  uint32_t*)0xE000ED0CU) /**< (ARM_SYSCTRL0) Application Interrupt and Reset Control Register */
#define REG_ARM_SYSCTRL0_SCR    (*(__IO uint8_t*)0xE000ED10U) /**< (ARM_SYSCTRL0) System Control Register */
#define REG_ARM_SYSCTRL0_CCR    (*(__I  uint16_t*)0xE000ED14U) /**< (ARM_SYSCTRL0) Configuration and Control Register */
#define REG_ARM_SYSCTRL0_SHPR2  (*(__IO uint32_t*)0xE000ED1CU) /**< (ARM_SYSCTRL0) System Handler Priority Register 2 */
#define REG_ARM_SYSCTRL0_SHPR3  (*(__IO uint32_t*)0xE000ED20U) /**< (ARM_SYSCTRL0) System Handler Priority Register 3 */
#define REG_ARM_SYSCTRL0_SHCSR  (*(__IO uint16_t*)0xE000ED24U) /**< (ARM_SYSCTRL0) System Handler Control and State Register */
#define REG_ARM_SYSCTRL0_DFSR   (*(__IO uint8_t*)0xE000ED30U) /**< (ARM_SYSCTRL0) Debug Fault Status Register */
#define REG_ARM_SYSCTRL0_DHCSR  (*(__IO uint32_t*)0xE000EDF0U) /**< (ARM_SYSCTRL0) Debug Halting Control and Status Register */
#define REG_ARM_SYSCTRL0_DCRSR  (*(__IO uint32_t*)0xE000EDF4U) /**< (ARM_SYSCTRL0) Debug Core Register Selector Register */
#define REG_ARM_SYSCTRL0_DCRDR  (*(__IO uint32_t*)0xE000EDF8U) /**< (ARM_SYSCTRL0) Debug Core Register Data Register */
#define REG_ARM_SYSCTRL0_DEMCR  (*(__IO uint32_t*)0xE000EDFCU) /**< (ARM_SYSCTRL0) Debug Exception and Monitor Control Register */
#define REG_ARM_SYSCTRL0_SCS_PID4 (*(__I  uint8_t*)0xE000EFD0U) /**< (ARM_SYSCTRL0) Peripheral ID Register 4 */
#define REG_ARM_SYSCTRL0_SCS_PID0 (*(__I  uint8_t*)0xE000EFE0U) /**< (ARM_SYSCTRL0) Peripheral ID Register 0 */
#define REG_ARM_SYSCTRL0_SCS_PID1 (*(__I  uint8_t*)0xE000EFE4U) /**< (ARM_SYSCTRL0) Peripheral ID Register 1 */
#define REG_ARM_SYSCTRL0_SCS_PID2 (*(__I  uint8_t*)0xE000EFE8U) /**< (ARM_SYSCTRL0) Peripheral ID Register 2 */
#define REG_ARM_SYSCTRL0_SCS_PID3 (*(__I  uint8_t*)0xE000EFECU) /**< (ARM_SYSCTRL0) Peripheral ID Register 3 */
#define REG_ARM_SYSCTRL0_SCS_CID0 (*(__I  uint8_t*)0xE000EFF0U) /**< (ARM_SYSCTRL0) Component ID Register 0 */
#define REG_ARM_SYSCTRL0_SCS_CID1 (*(__I  uint8_t*)0xE000EFF4U) /**< (ARM_SYSCTRL0) Component ID Register 1 */
#define REG_ARM_SYSCTRL0_SCS_CID2 (*(__I  uint8_t*)0xE000EFF8U) /**< (ARM_SYSCTRL0) Component ID Register 2 */
#define REG_ARM_SYSCTRL0_SCS_CID3 (*(__I  uint8_t*)0xE000EFFCU) /**< (ARM_SYSCTRL0) Component ID Register 3 */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMB11_ARM_SYSCTRL0_INSTANCE_ */
